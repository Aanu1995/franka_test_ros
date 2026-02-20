# franka_kitting_controller

Real-time state acquisition and Cartesian micro-lift controller for the Franka Panda with Phase 2 hybrid contact detection and rosbag data collection. Reads all robot state, model, and Cartesian signals, publishes them as a single `KittingState` message, autonomously detects contact using **hybrid gripper + arm signal fusion**, and executes smooth Cartesian micro-lift (UPLIFT) internally. Claims `FrankaPoseCartesianInterface` for Cartesian pose control.

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity

**Phase 2** adds:
- 6-state grasp machine: `START` -> `BASELINE` -> `CLOSING` -> `CONTACT` -> `SECURE_GRASP` -> `UPLIFT`
- Hybrid contact detection fusing gripper stall + arm torque signals (OR fusion)
- One rosbag per trial with all state transitions (recording starts automatically on launch)
- Automatic CSV export on stop for analysis-ready datasets
- Per-trial metadata output for offline analysis reproducibility

## Dependencies

- ROS Noetic (or Melodic)
- [libfranka](https://github.com/frankaemika/libfranka) >= 0.8.0
- [franka_ros](https://github.com/frankaemika/franka_ros) (specifically `franka_hw`, `franka_control`)

## Building

```bash
# From your catkin workspace root
catkin_make
# or
catkin build franka_kitting_controller
```

## Usage

### Phase 1: Launch the controller (required first)

Phase 1 connects to the robot, starts `franka_control`, and spawns the `kitting_state_controller`. This must be running before Phase 2.

```bash
roslaunch franka_kitting_controller kitting_state_controller.launch robot_ip:=<ROBOT_IP>
```

| Argument         | Default | Description               |
|------------------|---------|---------------------------|
| `robot_ip`       | (required) | Franka robot IP address |
| `load_gripper`   | `true`  | Load gripper driver       |

### Phase 2: Launch the logger (requires Phase 1)

Phase 2 launches **only** the C++ rosbag recording manager. It does not start the robot or the controller — it depends on Phase 1 already running.

```bash
# In a separate terminal (Phase 1 must already be running)
roslaunch franka_kitting_controller kitting_phase2.launch object_name:=cup base_directory:=/data/kitting
```

| Argument               | Default          | Description                              |
|------------------------|------------------|------------------------------------------|
| `base_directory`       | `~/kitting_bags` | Root directory for all trial data        |
| `object_name`          | `default_object` | Object name for bag file naming          |
| `auto_stop_on_contact` | `false`          | Auto-stop recording on CONTACT           |
| `export_csv_on_stop`   | `true`           | Auto-export CSV when recording stops     |

## Phase 2: Interaction States

Six interaction states structure grasp execution into measurable phases. The controller starts in `START` and waits for the user to publish `BASELINE` before any Phase 2 activity begins. States are published on `/kitting_phase2/state` for signal labeling and offline analysis. Recording starts automatically when the logger is launched and continues through all state transitions.

```
START -> BASELINE -> CLOSING -> CONTACT -> SECURE_GRASP -> UPLIFT
```

### START

Initial state after the controller is launched. No baseline collection, no contact detection — just Cartesian passthrough (hold position) and state data publishing at 250 Hz.

- Controller is running and publishing `KittingState` data
- No Phase 2 activity: no baseline statistics, no gripper commands accepted
- **All commands are rejected** until the Phase 2 logger is running (see below)
- Even after the logger is detected, all commands on `/kitting_phase2/state_cmd` are rejected until `BASELINE` is published
- Transition: launch the Phase 2 logger, then publish `BASELINE` on `/kitting_phase2/state`

**Logger readiness gate**: The controller subscribes to `/kitting_phase2/logger_ready` (a latched `std_msgs/Bool` topic published by the logger node). Until this signal is received, **all state commands are rejected** with a warning instructing the user to launch the Phase 2 logger first. This enforces the documented two-step launch sequence and prevents data collection without a running logger.

### BASELINE

Establish reference signal behavior before interaction.

- Gripper fully open, end-effector stationary, no object contact
- External torque norm `x(t) = ||τ_ext||` reflects system noise only
- Collects `N` samples over `T_base` seconds (default 0.7 s)
- Computes baseline mean `μ`, standard deviation `σ`, and contact threshold `θ = μ + kσ`

### CLOSING

Observe approach dynamics before contact. Hybrid contact detection is active.

- Gripper begins closing toward object at width `w` and speed `v`
- Two independent detectors run in parallel:
  - **Arm detector**: checks `x(t) > θ` AND `x(t) - μ > δ_min` with `T_arm_hold` debounce
  - **Gripper detector**: checks `|ẇ| < v_stall` AND `(w - w_cmd) > ε_w` AND `w > w_min` with `T_gripper_hold` debounce
- CONTACT = ArmContact OR GripperContact (either detector can trigger)

### CONTACT

Detect first stable physical interaction. Published **automatically** by the controller.

- Object touches gripper fingers — detected by **either** gripper stall or arm torque exceedance
- CONTACT log identifies which detector triggered (ARM or GRIPPER)
- CONTACT is **latched** once detected — cannot return to CLOSING
- **Gripper auto-stop**: When `stop_on_contact` is true (default), the active gripper action is cancelled and the gripper holds its current width at the moment of contact. This prevents over-compression of the object. The stop is executed asynchronously (non-blocking) via a non-RT timer.

### SECURE_GRASP

Characterize stable object compression.

- Gripper applies force `F` via GraspAction to width `w` with tolerance `ε_inner`/`ε_outer`
- External torque `x(t)` stabilizes with reduced variance compared to CONTACT transient
- Stable force distribution indicates object is securely held

### UPLIFT

Validate grasp robustness under load. The **controller internally executes** a smooth Cartesian micro-lift using cosine-smoothed trajectory interpolation — no external motion planner is involved.

- Controller displaces end-effector upward by distance `d` (default 3 mm, max 10 mm) over duration `T` (default 1.0 s)
- Cosine-smoothed trajectory `s = 0.5(1 - cos(π · s_raw))` ensures zero velocity at start and end
- Only z-translation of `O_T_EE_d[14]` is modified — orientation and x/y unchanged
- Object weight transfers to gripper; `x(t)` shows predictable change if grasp is secure
- Parameters `d` and `T` configurable via YAML defaults or per-command overrides

### Topics

Recording and state labeling are independent concerns.

| Topic                                    | Type                      | Direction  | Description                              |
|------------------------------------------|---------------------------|------------|------------------------------------------|
| `<ns>/kitting_state_data`                | KittingState              | Published  | Full state data at 250 Hz                |
| `/kitting_phase2/state_cmd`              | KittingGripperCommand     | Subscribed | Commands with per-object gripper params  |
| `/kitting_phase2/state`                  | std_msgs/String           | Pub/Sub    | State labels for offline segmentation    |
| `/kitting_phase2/record_control`         | std_msgs/String           | Subscribed | Recording control: STOP, ABORT           |
| `/kitting_phase2/logger_ready`           | std_msgs/Bool             | Subscribed | Latched readiness signal from logger node|
| `/franka_gripper/joint_states`           | sensor_msgs/JointState    | Subscribed | Gripper finger positions for stall detect|

- `/kitting_phase2/state_cmd` — The **user** publishes a `KittingGripperCommand` with `command` field set to `CLOSING`, `SECURE_GRASP`, or `UPLIFT`, plus optional per-object parameters. Any parameter left at `0.0` falls back to the YAML config default. The **controller** executes the corresponding action (gripper move/grasp, or Cartesian micro-lift), publishes the state label on `/kitting_phase2/state`, and updates its internal state machine.
- `/kitting_phase2/state` — The **user** publishes BASELINE here. The **controller** publishes CLOSING, SECURE_GRASP, UPLIFT (from `state_cmd`), and CONTACT (auto-detected). States are labels for offline analysis — they do NOT control recording.
- `/kitting_phase2/record_control` — The **user** publishes STOP or ABORT to end recording. Recording starts automatically when the logger launches — no START command is needed. The **logger** subscribes to this topic.
- `/kitting_phase2/logger_ready` — The **logger** publishes `true` (latched) on startup. The **controller** subscribes and gates all Phase 2 commands behind this signal. If the logger is not running, BASELINE and all `state_cmd` commands are rejected.

### Recording

One rosbag per trial. **Recording starts automatically** when the logger node is launched — no START command is needed. The logger opens a new trial bag immediately on startup and records all configured topics.

| Command | Action                                           |
|---------|--------------------------------------------------|
| `STOP`  | Close bag, save metadata, export CSV (if enabled) |
| `ABORT` | Close bag, delete trial directory (no CSV)        |

- **STOP** is ignored if not recording.
- **ABORT** is ignored if not recording.
- If the logger node is shut down (Ctrl+C), recording is automatically stopped (equivalent to STOP).

If `auto_stop_on_contact` is enabled and the controller publishes CONTACT, recording is auto-stopped (equivalent to STOP).

### Sending Commands

```bash
# Recording starts automatically when the logger is launched — no START needed.

# Set BASELINE — REQUIRED before any other state command
# Controller starts in START state; BASELINE begins the grasp sequence
rostopic pub /kitting_phase2/state std_msgs/String "data: 'BASELINE'" --once

# CLOSING — use YAML defaults for gripper parameters (all params = 0 -> defaults)
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'CLOSING'}" --once

# CLOSING — override width and speed for a specific object
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'CLOSING', closing_width: 0.06, closing_speed: 0.02}" --once

# ... CONTACT is published by the controller automatically ...

# SECURE_GRASP — use YAML defaults
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'SECURE_GRASP'}" --once

# SECURE_GRASP — override for a small, fragile object
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'SECURE_GRASP', grasp_width: 0.01, grasp_force: 5.0}" --once

# SECURE_GRASP — override all parameters for a heavy object
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'SECURE_GRASP', grasp_width: 0.03, epsilon_inner: 0.008, epsilon_outer: 0.008, grasp_speed: 0.02, grasp_force: 30.0}" --once

# UPLIFT — use YAML defaults (3mm over 1s)
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'UPLIFT'}" --once

# UPLIFT — override distance and duration for a specific object
rostopic pub /kitting_phase2/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'UPLIFT', uplift_distance: 0.005, uplift_duration: 2.0}" --once

# Stop recording
rostopic pub /kitting_phase2/record_control std_msgs/String "data: 'STOP'" --once

# Or abort (delete trial)
rostopic pub /kitting_phase2/record_control std_msgs/String "data: 'ABORT'" --once
```

## Phase 2: Hybrid Contact Detection

The controller autonomously detects physical contact using **two independent detectors** fused with OR logic:

```
CONTACT = GripperContact OR ArmContact
```

This hybrid approach ensures robustness across object types:
- **Gripper-only closure** on free objects may not produce measurable arm torques → gripper stall detects contact
- **Heavier or constrained objects** may produce arm torques before finger stall → arm detector catches it

Both detectors run at the publish rate (250 Hz) inside the real-time `update()` loop. Each has independent debounce. CONTACT is latched once declared by either detector.

### Arm-Based Contact Detector

#### Symbols

| Symbol    | Name                    | Unit  | Default | Description                                                                 |
|-----------|-------------------------|-------|---------|-----------------------------------------------------------------------------|
| `x(t)`   | Signal                  | Nm    | —       | External torque norm: `x(t) = \|\|τ_ext\|\| = √(Σ τ_ext_i²)` at time `t` |
| `N`      | Sample count            | —     | —       | Number of baseline samples collected (must reach `N_min`)                  |
| `μ`      | Baseline mean           | Nm    | —       | Average of `x(t)` during BASELINE — the system noise floor                |
| `σ`      | Baseline std. deviation | Nm    | —       | Spread of `x(t)` during BASELINE — how much noise naturally fluctuates    |
| `k`      | Sigma multiplier        | —     | 3.0     | Number of standard deviations above `μ` for the threshold                 |
| `θ`      | Contact threshold       | Nm    | —       | Trigger level: `θ = μ + kσ`                                               |
| `δ_min`  | Minimum absolute rise   | Nm    | 0.3     | Minimum rise above `μ` required (noise guard)                              |
| `T_base` | Baseline duration       | s     | 0.7     | Minimum time to collect baseline samples                                   |
| `N_min`  | Minimum sample count    | —     | 50      | Minimum samples before baseline statistics are valid                       |
| `T_arm_hold` | Arm debounce time   | s     | 0.10    | Duration conditions must hold continuously                                 |

#### Step 1: Baseline Collection (during BASELINE state)

The controller collects `N` samples of `x(t) = ||τ_ext||` over at least `T_base` seconds. Statistics are computed using a single-pass algorithm (no array storage):

```
         1
  μ  =  ─── Σ x_i
         N

              ┌─────────────────────────────────┐
              │  1                               │
  σ  =  sqrt │ ───── Σ (x_i - μ)²              │
              │ N - 1                            │
              └─────────────────────────────────┘

  θ  =  μ + k · σ
```

#### Step 2: Arm Contact Condition

ArmContact is TRUE when **both** conditions hold continuously for `T_arm_hold` seconds:

```
  1. Statistical exceedance:   x(t) > θ         where θ = μ + kσ

  2. Minimum absolute rise:    x(t) - μ > δ_min
```

The `δ_min` guard prevents false triggers when `σ` is very small (low noise). Even if `x(t)` exceeds `θ`, contact is not declared unless the absolute rise above `μ` exceeds `δ_min`.

### Gripper-Based Contact Detector

#### Symbols

| Symbol          | Name                  | Unit  | Default | Description                                                         |
|-----------------|-----------------------|-------|---------|---------------------------------------------------------------------|
| `w(t)`          | Measured gripper width | m     | —       | `w(t) = q_finger1 + q_finger2` from `/franka_gripper/joint_states` |
| `ẇ(t)`          | Gripper width velocity | m/s   | —       | `ẇ(t) = (w(t) - w(t-Δt)) / Δt`                                    |
| `w_cmd`         | Commanded width        | m     | —       | Target width from the CLOSING command                               |
| `v_stall`       | Stall velocity threshold | m/s | 0.003   | Below this speed, fingers are considered stalled                    |
| `ε_w`           | Directional width gap  | m     | 0.002   | Minimum gap above target to confirm early blockage                  |
| `w_min`         | Minimum width guard    | m     | 0.002   | Reject contact at gripper mechanical limit                          |
| `T_gripper_hold`| Gripper debounce time  | s     | 0.10    | Duration conditions must hold continuously                          |

#### Gripper Stall Condition (Directional)

GripperContact is TRUE when **all three** conditions hold continuously for `T_gripper_hold` seconds:

```
  1. Stall:              |ẇ(t)| < v_stall

  2. Directional gap:    (w(t) - w_cmd) > ε_w        ← signed, NOT absolute value

  3. Not fully closed:   w(t) > w_min
```

**Why directional?** The previous `|w - w_cmd|` check caused false positives when the gripper reached its target normally (velocity → 0, but w ≈ w_cmd so no real contact). The signed condition `(w - w_cmd) > ε_w` ensures w(t) > w_cmd, meaning the gripper stopped **before** reaching its target because an object blocked the fingers. The `w_min` guard prevents false contact when the gripper reaches its mechanical limit.

### Fusion and Latching

CONTACT is declared when **either** detector fires:

```
  CONTACT = GripperContact OR ArmContact
```

Once CONTACT is declared, it is **latched** — the detectors stop evaluating. A new trial (publishing BASELINE) is required to re-arm detection. The CONTACT log identifies which detector triggered.

## Phase 2: UPLIFT Trajectory Mathematics

The UPLIFT state executes a smooth Cartesian micro-lift internally using cosine-smoothed time-based trajectory interpolation. The trajectory runs at the control loop rate (1 kHz) and commands the end-effector pose via `FrankaPoseCartesianInterface`.

### Symbols

| Symbol     | Name                      | Unit  | Default | Description                                                                 |
|------------|---------------------------|-------|---------|-----------------------------------------------------------------------------|
| `d`        | Uplift distance           | m     | 0.003   | Total upward displacement along the z-axis (max 0.01)                      |
| `T`        | Uplift duration           | s     | 1.0     | Total time for the cosine-smoothed trajectory                              |
| `t`        | Elapsed time              | s     | —       | Time since UPLIFT started, incremented by `Δt` (control period) each tick  |
| `Δt`       | Control period            | s     | 0.001   | Time between consecutive `update()` calls (1 kHz)                          |
| `s_raw`    | Normalized time           | —     | —       | Linear progress: `s_raw = min(t/T, 1)`, clamped to [0, 1]                 |
| `s`        | Smoothed progress         | —     | —       | Cosine-smoothed: `s = ½(1 − cos(π · s_raw))`, maps [0,1] → [0,1]         |
| `z₀`       | Start z-position          | m     | —       | z-translation of `O_T_EE_d` at the moment UPLIFT begins: `z₀ = O_T_EE_d[14]` |
| `z(t)`     | Commanded z-position      | m     | —       | `z(t) = z₀ + s · d`                                                       |
| `v(t)`     | Commanded z-velocity      | m/s   | —       | `v(t) = dz/dt = (π·d)/(2T) · sin(π·t/T)`                                 |
| `O_T_EE_d` | Desired end-effector pose | —     | —       | 4×4 column-major homogeneous transform; index [14] = z-translation        |

### Trajectory Equation

The trajectory smoothly moves the end-effector from `z₀` to `z₀ + d` over duration `T`:

```
              t
  s_raw  =  min( ─── , 1 )
              T

         1
  s  =  ─── ( 1  −  cos( π · s_raw ) )
         2

  z(t)  =  z₀  +  s · d
```

The **velocity profile** (first derivative) is:

```
  dz       π · d
  ── (t) = ───── · sin( π · t / T )
  dt        2T
```

### Trajectory Properties

| Property              | Mathematical expression              | Physical meaning                              |
|-----------------------|--------------------------------------|-----------------------------------------------|
| Position at `t = 0`   | `z(0) = z₀`                         | Starts at the current height                  |
| Position at `t = T`   | `z(T) = z₀ + d`                     | Ends exactly `d` meters above start           |
| Velocity at `t = 0`   | `dz/dt = 0`                         | Zero velocity at start (no step discontinuity)|
| Velocity at `t = T`   | `dz/dt = 0`                         | Zero velocity at end (smooth stop)            |
| Peak velocity         | `v_max = πd/(2T)` at `t = T/2`      | Maximum speed at the midpoint of the motion   |
| Peak velocity (default) | `v_max = π·0.003/(2·1.0) ≈ 0.0047 m/s` | ~4.7 mm/s — well within Franka limits     |

### Execution Details

1. **Start**: When UPLIFT is received via `state_cmd`, the controller captures the current `O_T_EE_d` as the start pose and records `z₀ = O_T_EE_d[14]`. Parameters `d` and `T` are snapshotted into RT-local copies (`rt_uplift_distance_`, `rt_uplift_duration_`) to prevent mid-trajectory corruption from concurrent subscriber writes.

2. **Per-tick** (1 kHz): The controller increments `t ← t + Δt`, computes `s_raw`, `s`, and `z(t)`, then calls `setCommand(pose)`. Only the z-translation (index [14]) is modified — orientation and x/y position remain unchanged from the start pose.

3. **Completion**: When `t ≥ T`, the trajectory is done. `uplift_active` is cleared and the controller transitions to passthrough mode, holding position at `z₀ + d`.

4. **Passthrough mode**: When not executing UPLIFT, the controller reads `O_T_EE_d` and writes it back as the command every tick. This produces zero tracking error — the robot holds position with no drift or jerk.

### Safety Constraints

| Constraint              | Symbol / Value     | Description                                                    |
|-------------------------|--------------------|----------------------------------------------------------------|
| Maximum distance        | `d ≤ 0.01 m`      | Hard clamp — any `d > 10 mm` is clamped with a warning        |
| Minimum distance        | `d > 0`            | Zero or negative `d` is rejected                               |
| Minimum duration        | `T > 0`            | Zero or negative `T` is rejected                               |
| Maximum velocity        | `v_max = πd/(2T)`  | Bounded by the clamp on `d` and the minimum `T`               |
| Precondition            | Configurable       | `require_secure_grasp: true` requires SECURE_GRASP state       |
| Duplicate rejection     | —                  | UPLIFT is ignored while a trajectory is already active         |
| BASELINE interruption   | —                  | BASELINE clears UPLIFT, returns to passthrough (with warning)  |

## Configuration

### Controller Parameters (`config/kitting_state_controller.yaml`)

| Parameter                  | Type   | Default | Description                                    |
|----------------------------|--------|---------|------------------------------------------------|
| `arm_id`                   | string | `panda` | Robot arm identifier                           |
| `publish_rate`             | double | `250.0` | State data publish rate [Hz]                   |
| `T_base`                   | double | `0.7`   | Baseline collection duration [s]               |
| `N_min`                    | int    | `50`    | Minimum samples before arming detection        |
| `enable_arm_contact`       | bool   | `true`  | Enable arm-based contact detector              |
| `k_sigma`                  | double | `3.0`   | Threshold multiplier (theta = mu + k*sigma)    |
| `delta_min`                | double | `0.3`   | Minimum absolute rise above mu [Nm]            |
| `T_arm_hold`               | double | `0.10`  | Arm detector debounce hold time [s]            |
| `enable_gripper_contact`   | bool   | `true`  | Enable gripper-based contact detector          |
| `v_stall`                  | double | `0.003` | Gripper stall velocity threshold [m/s]         |
| `epsilon_w`                | double | `0.002` | Directional width gap threshold [m]            |
| `w_min`                    | double | `0.002` | Minimum width guard (mechanical limit) [m]     |
| `T_gripper_hold`           | double | `0.10`  | Gripper detector debounce hold time [s]        |
| `stop_on_contact`          | bool   | `true`  | Cancel gripper action and hold width on CONTACT |
| `execute_gripper_actions`  | bool   | `true`  | Execute gripper actions (false = signal-only)   |
| `closing_width`            | double | `0.04`  | Default width for MoveAction in CLOSING [m]    |
| `closing_speed`            | double | `0.04`  | Default speed for MoveAction in CLOSING [m/s]  |
| `grasp_width`              | double | `0.02`  | Default width for GraspAction [m]              |
| `epsilon_inner`            | double | `0.005` | Default inner epsilon for GraspAction [m]      |
| `epsilon_outer`            | double | `0.005` | Default outer epsilon for GraspAction [m]      |
| `grasp_speed`              | double | `0.04`  | Default speed for GraspAction [m/s]            |
| `grasp_force`              | double | `10.0`  | Default force for GraspAction [N]              |
| `uplift_distance`          | double | `0.003` | Default upward displacement [m] (max 0.01)     |
| `uplift_duration`          | double | `1.0`   | Cosine-smoothed motion duration [s]            |
| `uplift_reference_frame`   | string | `world` | Reference frame for z-axis displacement        |
| `require_secure_grasp`     | bool   | `true`  | Only allow UPLIFT from SECURE_GRASP state      |

Gripper and UPLIFT parameters are **defaults**. They can be overridden per-command by setting non-zero values in the `KittingGripperCommand` message published on `/kitting_phase2/state_cmd`.

### Logger Parameters (`config/kitting_phase2_logger.yaml`)

| Parameter              | Type        | Default              | Description                                  |
|------------------------|-------------|----------------------|----------------------------------------------|
| `base_directory`       | string      | `~/kitting_bags`     | Root directory for bag files                 |
| `object_name`          | string      | `default_object`     | Object identifier for file naming            |
| `auto_stop_on_contact` | bool        | `false`              | Auto-stop recording on CONTACT               |
| `export_csv_on_stop`   | bool        | `true`               | Auto-export CSV when recording stops         |
| `topics_to_record`     | string list | (see below)          | Topics recorded in rosbag                    |

Default recorded topics:
- `/kitting_state_controller/kitting_state_data`
- `/kitting_phase2/state`
- `/kitting_phase2/state_cmd`
- `/kitting_phase2/record_control`
- `/joint_states`

## File Output Structure

One bag per trial, organized by object. CSV is automatically exported alongside the bag when recording stops.

```
~/kitting_bags/
  cup/
    trial_001/
      20260218_143025_cup_trial_001.bag
      trial_001_signals.csv
      metadata.yaml
    trial_002/
      20260218_144500_cup_trial_002.bag
      trial_002_signals.csv
      metadata.yaml
```

The single bag contains all data from all states (BASELINE through UPLIFT) for that trial. Recording starts automatically on launch and stops on STOP, ABORT, or node shutdown. The CSV is exported in a background thread so STOP returns immediately.

### metadata.yaml Contents

```yaml
object_name: cup
trial_number: 1
bag_filename: "20260218_143025_cup_trial_001.bag"
csv_filename: "trial_001_signals.csv"
start_time: "20260218_143025"
stop_time: "20260218_143052"
total_samples: 6750
topics_recorded:
  - /kitting_state_controller/kitting_state_data
  - /kitting_phase2/state
  - /kitting_phase2/record_control
  - /joint_states
auto_stop_on_contact: false
export_csv_on_stop: true
detector_parameters:
  T_base: 0.7
  N_min: 50
  arm_detector:
    enabled: true
    k_sigma: 3.0
    delta_min: 0.3
    T_arm_hold: 0.10
  gripper_detector:
    enabled: true
    v_stall: 0.003
    epsilon_w: 0.002
    w_min: 0.002
    T_gripper_hold: 0.10
baseline_statistics:
  baseline_mu_tau_ext_norm: 0.342
  baseline_sigma_tau_ext_norm: 0.051
  contact_threshold_theta: 0.597
```

### CSV Export

When `export_csv_on_stop` is `true` (default), the logger automatically reads back the rosbag after recording stops and writes a flattened CSV file using the C++ `rosbag::View` API. The export runs in a background thread so the stop operation returns immediately.

The CSV contains one row per `KittingState` message (71 columns):

| Group            | Columns                                                          |
|------------------|------------------------------------------------------------------|
| Time             | `timestamp_sec`, `timestamp_nsec`, `time_float`                  |
| State            | `state_label` (most recent state at that timestamp)              |
| Joint positions  | `q_1` ... `q_7`                                                  |
| Joint velocities | `dq_1` ... `dq_7`                                                |
| Joint torques    | `tau_J_1` ... `tau_J_7`                                          |
| External torques | `tau_ext_1` ... `tau_ext_7`, `tau_ext_norm`                      |
| Wrench           | `wrench_fx`, `wrench_fy`, `wrench_fz`, `wrench_tx`, `wrench_ty`, `wrench_tz`, `wrench_norm` |
| EE velocity      | `ee_vx`, `ee_vy`, `ee_vz`, `ee_wx`, `ee_wy`, `ee_wz`           |
| Gravity          | `gravity_1` ... `gravity_7`                                      |
| Coriolis         | `coriolis_1` ... `coriolis_7`                                    |
| Gripper          | `gripper_width`                                                  |
| Contact diag.    | `baseline_mu`, `baseline_sigma`, `contact_threshold`, `arm_margin`, `arm_delta`, `gripper_cmd_width`, `gripper_velocity`, `arm_detector_active`, `gripper_detector_active`, `contact_detected` |

`O_T_EE` (16 values) and `jacobian` (42 values) are not included in the CSV. They remain in the rosbag.

State labels are synchronized by iterating the bag chronologically: each signal row gets the most recent `/kitting_phase2/state` label at that timestamp.

## KittingGripperCommand Message

Per-object gripper command published on `/kitting_phase2/state_cmd`. Any parameter left at `0.0` (the ROS default for `float64`) falls back to the YAML config value loaded at startup. This lets you override only the parameters that differ for a particular object.

| Field              | Type    | Description                                              |
|--------------------|---------|----------------------------------------------------------|
| `command`          | string  | `"CLOSING"`, `"SECURE_GRASP"`, or `"UPLIFT"`            |
| `closing_width`    | float64 | Target width for MoveAction [m] (0 = use default)       |
| `closing_speed`    | float64 | Speed for MoveAction [m/s] (0 = use default)            |
| `grasp_width`      | float64 | Target width for GraspAction [m] (0 = use default)      |
| `epsilon_inner`    | float64 | Inner epsilon for GraspAction [m] (0 = use default)     |
| `epsilon_outer`    | float64 | Outer epsilon for GraspAction [m] (0 = use default)     |
| `grasp_speed`      | float64 | Speed for GraspAction [m/s] (0 = use default)           |
| `grasp_force`      | float64 | Force for GraspAction [N] (0 = use default)             |
| `uplift_distance`  | float64 | Upward displacement [m] (0 = use default, max 0.01)     |
| `uplift_duration`  | float64 | Duration of cosine-smoothed micro-lift [s] (0 = default)|

Only the parameters relevant to the command are used:
- `CLOSING` uses `closing_width` and `closing_speed`
- `SECURE_GRASP` uses `grasp_width`, `epsilon_inner`, `epsilon_outer`, `grasp_speed`, and `grasp_force`
- `UPLIFT` uses `uplift_distance` and `uplift_duration`

## KittingState Message

| Field          | Type         | Description                                          |
|----------------|--------------|------------------------------------------------------|
| `header`       | Header       | Timestamp and frame info                             |
| `q`            | float64[7]   | Joint positions [rad]                                |
| `dq`           | float64[7]   | Joint velocities [rad/s]                             |
| `tau_J`        | float64[7]   | Measured joint torques [Nm]                          |
| `tau_ext`      | float64[7]   | Estimated external joint torques [Nm]                |
| `wrench_ext`   | float64[6]   | External wrench in base frame (Fx,Fy,Fz,Tx,Ty,Tz)  |
| `O_T_EE`       | float64[16]  | EE pose in base frame (4x4 column-major)             |
| `jacobian`     | float64[42]  | Zero Jacobian at EE (6x7 column-major)               |
| `gravity`      | float64[7]   | Gravity torque vector [Nm]                           |
| `coriolis`     | float64[7]   | Coriolis torque vector [Nm]                          |
| `ee_velocity`  | float64[6]   | End-effector velocity (J * dq) [m/s, rad/s]         |
| `tau_ext_norm`  | float64      | Euclidean norm of `tau_ext`                          |
| `wrench_norm`  | float64      | Euclidean norm of `wrench_ext`                       |
| `gripper_width` | float64     | Measured gripper width (q_finger1 + q_finger2) [m]   |
| `baseline_mu`   | float64     | Baseline mean of tau_ext_norm [Nm]                   |
| `baseline_sigma` | float64    | Baseline std dev of tau_ext_norm [Nm]                |
| `contact_threshold` | float64 | theta = mu + k * sigma [Nm]                          |
| `arm_margin`    | float64     | tau_ext_norm − theta (>0 = above threshold) [Nm]     |
| `arm_delta`     | float64     | tau_ext_norm − mu (absolute rise above baseline) [Nm]|
| `gripper_cmd_width` | float64 | Commanded gripper target width w_cmd [m]             |
| `gripper_velocity` | float64  | Finite-difference gripper velocity dw/dt [m/s]       |
| `arm_detector_active` | bool  | True if arm detector condition is met this tick      |
| `gripper_detector_active` | bool | True if gripper stall condition is met this tick  |
| `contact_detected` | bool     | True once CONTACT is latched                         |

## Interfaces

The controller claims three hardware interfaces:

- `FrankaStateInterface` -- provides access to `franka::RobotState`
- `FrankaModelInterface` -- provides access to dynamics/kinematics (Jacobian, gravity, Coriolis)
- `FrankaPoseCartesianInterface` -- provides exclusive Cartesian pose command authority for UPLIFT micro-lift

When not executing UPLIFT, the controller operates in **passthrough mode**: it reads the robot's own desired pose (`O_T_EE_d`) and writes it back as the command every tick. This results in zero tracking error and the robot holds position. During UPLIFT, the controller commands a cosine-smoothed trajectory upward. Gripper actions are dispatched via `actionlib::SimpleActionClient` from the subscriber callback thread, not from the RT `update()` loop.

## Real-Time Safety

- Uses `realtime_tools::RealtimePublisher` with non-blocking `trylock()`
- No dynamic memory allocation in `update()`
- No blocking operations in `update()`
- Model queries are only called at the publish rate, not every control tick
- Hybrid contact detection uses only scalar arithmetic and atomic loads (no Eigen, no allocation)
- State publisher uses `trylock()` pattern (publish only on transition, non-blocking)
- Gripper action clients use `sendGoal()` (non-blocking) from subscriber thread, never from `update()`
- Action clients created with `spin_thread=true` for plugin context (no external spin required)
- Cartesian pose command issued every tick (1 kHz) — passthrough when idle, trajectory during UPLIFT
- UPLIFT trajectory uses only `std::array<double, 16>`, `std::cos()`, and `std::min()` — no allocation
- `starting()` captures initial desired pose to avoid step discontinuity on controller start
- Rosbag management runs in a separate C++ node (no Python overhead)

## Recording Performance

The Phase 2 logger is written in C++ using the `rosbag::Bag` API directly and `topic_tools::ShapeShifter` for generic topic subscription. This eliminates:

- **No subprocess spawn** — bag is opened via API, not `rosbag record` subprocess
- **No Python GIL** — pure C++ callbacks, no interpreter overhead
- **No subscription handshake delay** — topics are already subscribed and recording starts automatically on launch; the bag file is opened immediately
- **Multi-threaded** — uses `ros::AsyncSpinner(4)` so data callbacks and command callbacks run concurrently
- **Single mutex** — one lock protects all trial state, minimal contention

## Phase 1 Validation

### Test 1 -- Free Space Motion

Move the robot without contact. **Expected**: `wrench_ext` near zero, `tau_ext` near noise floor.

### Test 2 -- Manual External Push

Push the robot gently. **Expected**: increase in `tau_ext_norm` and `wrench_norm`.

### Test 3 -- Controlled Table Contact

Move EE into table. **Expected**: clear increase in Fz, `wrench_norm`, `tau_ext_norm`.

## Phase 2 Acceptance Checks

- **Logger must be running before any state commands are accepted** — controller rejects BASELINE and all `state_cmd` commands until `/kitting_phase2/logger_ready` is received
- Recording starts automatically when the logger launches — no START command needed
- Recording and state labeling are independent
- STOP/ABORT on `/kitting_phase2/record_control` end recording
- State labels on `/kitting_phase2/state` are for offline segmentation only
- One rosbag per trial containing all signals and all state transitions
- STOP saves bag + metadata + CSV, ABORT deletes trial (no CSV)
- Logger shutdown (Ctrl+C) automatically stops recording (equivalent to STOP)
- CONTACT auto-stop works when `auto_stop_on_contact` is enabled
- Hybrid CONTACT detection: arm torque OR gripper stall (independently debounced)
- Finger contact on free objects triggers CONTACT via gripper stall detector (directional: w > w_cmd)
- Gripper reaching target width normally does NOT trigger false CONTACT
- Load transfer or constrained contact triggers CONTACT via arm torque detector
- CONTACT log identifies which detector triggered (ARM or GRIPPER)
- Gripper stops immediately on CONTACT when `stop_on_contact` is true (cancels active action, holds width)
- No further gripper compression occurs after CONTACT (width remains constant until SECURE_GRASP)
- Gripper stop is non-blocking (asynchronous via non-RT timer, does not block control loop)
- `stop_on_contact: false` allows gripper to continue (testing mode)
- If both detectors are disabled, controller refuses to start (FATAL)
- Bag contains 5 topics: kitting_state_data, state, state_cmd, record_control, joint_states
- CSV contains 71 flattened columns with correct state labels per row (includes gripper_width + 10 contact diagnostics)
- CSV export does not block the ROS spin loop (runs in background thread)
- metadata.yaml contains bag_filename, csv_filename, total_samples, start/stop times, and detector parameters
- Publishing CLOSING on `state_cmd` triggers gripper MoveAction + state label
- Publishing SECURE_GRASP on `state_cmd` triggers gripper GraspAction + state label
- Publishing UPLIFT on `state_cmd` triggers internal Cartesian micro-lift + state label
- UPLIFT moves robot upward by `uplift_distance` over `uplift_duration` with cosine smoothing
- UPLIFT orientation remains unchanged throughout the motion
- UPLIFT is rejected if `require_secure_grasp` is true and current state is not SECURE_GRASP
- UPLIFT distance is clamped to 10 mm maximum (with warning)
- Duplicate UPLIFT commands are ignored while UPLIFT is active
- BASELINE during UPLIFT clears the active motion and returns to passthrough
- Per-command gripper/UPLIFT parameters override YAML defaults when non-zero
- Parameters left at 0.0 fall back to YAML config values
- Duplicate CLOSING/SECURE_GRASP commands are ignored
- Controller holds position (passthrough) when not executing UPLIFT — no drift or jerk
- Gripper server unavailability does not crash the controller (state change still occurs)
- `execute_gripper_actions: false` enables signal-only testing mode

## License

Apache 2.0
