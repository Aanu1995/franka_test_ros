# franka_kitting_controller

Real-time state acquisition and Cartesian micro-lift controller for the Franka Panda with Phase 2 hybrid contact detection and rosbag data collection. Reads all robot state, model, Cartesian, and gripper signals, publishes them as a single `KittingState` message, autonomously detects contact using dual arm torque + gripper stall detection (OR logic), immediately stops the gripper on contact via `franka::Gripper::stop()`, and executes smooth Cartesian micro-lift (UPLIFT) internally. Claims `FrankaPoseCartesianInterface` for Cartesian pose control. Gripper operations use the libfranka `franka::Gripper` API directly (no dependency on `franka_gripper` ROS package).

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity
- **Gripper telemetry**: finger width, max width, width velocity, commanded width, grasp state (from `franka::GripperState` via direct libfranka API)

**Phase 2** adds:
- 6-state grasp machine: `START` -> `BASELINE` -> `CLOSING` -> `CONTACT` -> `SECURE_GRASP` -> `UPLIFT`
- Hybrid contact detection: arm external torque + gripper stall detection (OR logic)
- Immediate gripper stop on contact: calls `franka::Gripper::stop()` to physically halt the motor
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
| `load_gripper`   | `false` | Load gripper driver (must be false — controller owns the gripper connection) |

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

Observe approach dynamics before contact.

- Gripper begins closing toward object at width `w` and speed `v` (max 0.10 m/s)
- Two contact detectors run concurrently (hybrid OR logic):
  - **Arm detector**: External torque norm `x(t) > θ` (baseline statistics + debounce)
  - **Gripper detector**: Finger width velocity stalls while gap to target width remains (stall + debounce)
- First detector to trigger wins — CONTACT is latched immediately

### CONTACT

Detect first stable physical interaction. Published **automatically** by the controller.

- Object touches gripper fingers — detected by **either** the arm or gripper detector
- **Arm detection**: External torque norm exceeds threshold `x(t) > θ` continuously for `T_hold_arm` seconds (default 0.10 s)
- **Gripper detection**: Width velocity drops below `stall_velocity_threshold` (0.005 m/s) while `(width - w_cmd) > width_gap_threshold` (0.002 m), sustained for `T_hold_gripper` seconds (computed dynamically from `closing_speed`; see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper))
- **Immediate stop**: On contact, `franka::Gripper::stop()` is called via the read thread to physically halt the motor at the contact width
- CONTACT is **latched** once detected — cannot return to CLOSING
- `contact_source` records which detector fired: `"ARM"` or `"GRIPPER"`

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

## Phase 2: Contact Detection

The controller uses **hybrid contact detection** with two independent detectors running concurrently during CLOSING. Either detector triggering is sufficient to declare CONTACT (OR logic). Both detectors share a single latch (`contact_latched_`) — the first one to trigger wins.

### Why Hybrid Detection?

During gripper CLOSING, the arm joints remain stationary — only the gripper fingers move. This means the arm's external torque signal (`tau_ext`) may not change significantly on contact, making arm-only detection unreliable for gripper-based grasping. The gripper stall detector addresses this by monitoring the finger width velocity directly. The arm detector remains as a secondary safety net for cases where external forces propagate to the arm joints.

Each detector can be independently enabled/disabled via `enable_arm_contact` and `enable_gripper_contact` parameters. This allows testing each detector in isolation to verify which one is working for a given scenario. The master switch `enable_contact_detector` must be `true` for either individual detector to run.

### Detector 1: Arm External Torque (Secondary)

Statistical thresholding on the external torque norm signal. Runs at 250 Hz inside the real-time `update()` loop.

#### Symbols

| Symbol | Name                    | Unit  | Default | Description                                                                 |
|--------|-------------------------|-------|---------|-----------------------------------------------------------------------------|
| `x(t)` | Signal                  | Nm    | —       | External torque norm: `x(t) = \|\|τ_ext\|\| = √(Σ τ_ext_i²)` at time `t` |
| `N`    | Sample count            | —     | —       | Number of baseline samples collected (must reach `N_min`)                  |
| `μ`    | Baseline mean           | Nm    | —       | Average of `x(t)` during BASELINE — the system noise floor                |
| `σ`    | Baseline std. deviation | Nm    | —       | Spread of `x(t)` during BASELINE — how much noise naturally fluctuates    |
| `k`    | Sigma multiplier        | —     | 3.0     | Number of standard deviations above `μ` for the threshold                 |
| `θ`    | Contact threshold       | Nm    | —       | Trigger level: `θ = μ + kσ`                                               |
| `T_base` | Baseline duration     | s     | 0.7     | Minimum time to collect baseline samples                                   |
| `N_min`  | Minimum sample count  | —     | 50      | Minimum samples before baseline statistics are valid                       |
| `T_hold_arm` | Debounce hold time | s    | 0.10    | Duration `x(t)` must continuously exceed `θ` to declare contact           |
| `dx/dt`  | Signal slope          | Nm/s  | —       | Time derivative of `x(t)`, used by optional slope gate                    |
| `slope_min` | Minimum slope       | Nm/s  | 5.0     | Minimum `dx/dt` required for contact (only if slope gate enabled)         |

#### Step 1: Baseline Collection (during BASELINE state)

The controller collects `N` samples of `x(t) = ||τ_ext||` over at least `T_base` seconds. Collection continues until **both** conditions are met: elapsed time `≥ T_base` **and** sample count `N ≥ N_min`. This dual condition prevents silent failure when `N_min > publish_rate × T_base`.

Statistics are computed using a single-pass algorithm (no array storage):

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

Where:
- `μ` (mu) is the **mean** — average noise level when nothing is touching the robot
- `σ` (sigma) is the **standard deviation** — how much that noise fluctuates
- `θ` (theta) is the **threshold** — the trigger level `k` standard deviations above the mean
- The variance uses Bessel's correction (`N-1`) for an unbiased estimate from a finite sample
- Negative variance (possible due to floating-point) is clamped to zero before taking the square root

**Interpretation of `k`**: With `k = 3`, the threshold `θ` is 3 standard deviations above the noise floor. Higher `k` = fewer false positives but requires stronger contact. Lower `k` = more sensitive but risks false triggers.

#### Step 2: Arm Contact Detection (during CLOSING state)

Once baseline statistics are computed (`baseline_armed = true`), the controller checks every sample during CLOSING:

```
  Contact condition:   x(t)  >  θ       where θ = μ + kσ
```

**Debounce**: A single sample exceeding `θ` is not sufficient. The signal must remain above `θ` **continuously** for `T_hold_arm` seconds. If `x(t)` drops below `θ` at any point, the debounce timer resets to zero.

```
  Let t₀ = first time x(t) > θ

  CONTACT declared when:   x(t) > θ   ∀ t ∈ [t₀, t₀ + T_hold_arm]
```

**Optional slope gate** (disabled by default, `use_slope_gate: false`): When enabled, contact additionally requires the signal to be **rising**:

```
  dx       x(t) - x(t - Δt)
  ── (t) = ─────────────────  >  slope_min
  dt              Δt
```

Where `Δt = slope_dt`. This filters out slow drift that might cross `θ` without actual contact.

### Detector 2: Gripper Stall Detection (Primary)

Monitors gripper finger width velocity via finite difference. When the gripper is commanded to close but the fingers stop moving before reaching the target width, an object is blocking them.

#### Data Source

The controller reads gripper state directly from `franka::Gripper::readOnce()` at firmware rate in a dedicated read thread. This provides:

```
  w         = GripperState.width       (direct finger width, no reconstruction needed)
  max_width = GripperState.max_width   (maximum opening width)
  is_grasped = GripperState.is_grasped (firmware-level grasp detection)
```

Width velocity is computed via finite difference in the read thread:

```
  w_dot = (w_current - w_previous) / dt
```

The `GripperData` struct `{width, max_width, width_dot, is_grasped, stamp}` is passed to the RT `update()` loop via a lock-free `realtime_tools::RealtimeBuffer`.

#### Stall Detection Logic

During CLOSING, the controller checks every RT tick:

```
  velocity_stalled = |w_dot| < stall_velocity_threshold    (default 0.005 m/s)
  width_gap_exists = (w - w_cmd) > width_gap_threshold     (default 0.002 m)

  stall_detected = velocity_stalled AND width_gap_exists
```

The **width gap check** is essential: without it, normal gripper completion (velocity drops to 0 at target width) would false-trigger as contact. The gap ensures we only detect stalls where the gripper stopped **before** reaching its commanded width.

**Debounce**: The stall condition must persist for `T_hold_gripper` seconds (computed dynamically from `closing_speed`; see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper)). If the stall condition breaks at any point, the debounce timer resets.

#### Symbols

| Symbol | Name                      | Unit  | Default | Description                                                        |
|--------|---------------------------|-------|---------|--------------------------------------------------------------------|
| `w`    | Gripper width             | m     | —       | Current finger width (from `franka::GripperState.width`)          |
| `w_dot`| Width velocity            | m/s   | —       | Finite difference of `w` between consecutive `readOnce()` calls  |
| `w_cmd`| Commanded width           | m     | 0.04    | Target width for the active MoveAction (from CLOSING command)     |
| `v_stall` | Stall velocity threshold | m/s | 0.005   | Speed below this is considered stalled                            |
| `Δw`   | Width gap threshold       | m     | 0.002   | Minimum `(w - w_cmd)` to distinguish stall from normal completion |
| `T_hold_gripper` | Debounce time    | s     | computed | Duration stall must persist to declare contact (see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper)) |

### Gripper Stop on Contact

When CONTACT is detected by **either** detector, the controller immediately stops the gripper to prevent object damage:

1. **Atomic flag**: The RT `update()` loop sets `stop_requested_` (single atomic store, nanoseconds)
2. **Read thread executes stop**: The gripper read thread checks `stop_requested_` after each `readOnce()` and calls `franka::Gripper::stop()`, which communicates directly with the firmware to physically halt the motor
3. **One-shot guard**: `gripper_stop_sent_` flag prevents duplicate stop requests

The atomic store in `update()` is a single-word write (nanoseconds), fully RT-safe. The actual `stop()` call executes in the non-RT read thread. Worst-case latency from contact detection to physical halt is one firmware update period. The stop is only executed when `stop_on_contact` is `true` (default) and `execute_gripper_actions` is `true`.

### Dynamic Gripper Debounce Time (`T_hold_gripper`)

The gripper stall debounce time is **computed dynamically** from `closing_speed` rather than configured statically. Higher closing speeds produce more velocity noise — `w_dot` briefly reads ~0 even when the gripper is still moving — so the debounce window must increase with speed to avoid false contact triggers.

#### Formula

```
  T_hold_gripper  =  0.35  +  0.5 · clamp(v, 0, 0.10)
```

Where `v` is the resolved `closing_speed` (YAML default or per-command override, clamped to `[0, v_max]`).

#### Constants

| Symbol             | Value     | Description                                              |
|--------------------|-----------|----------------------------------------------------------|
| `T_base_hold`      | 0.35 s    | Hold time intercept at zero speed                        |
| `k_hold`           | 0.5 s/(m/s) | Linear slope: hold time increase per unit speed        |
| `v_max`            | 0.10 m/s  | Hard cap on closing speed (speeds above this are clamped)|

#### Resulting Hold Times

| `closing_speed` (m/s) | `T_hold_gripper` (s) | Notes                            |
|------------------------|----------------------|----------------------------------|
| 0.01                   | 0.355                |                                  |
| 0.02                   | 0.360                | Verified experimentally          |
| 0.04                   | 0.370                | Default speed, verified          |
| 0.06                   | 0.380                | Verified experimentally          |
| 0.08                   | 0.390                |                                  |
| 0.10                   | 0.400                | Maximum speed, verified          |
| > 0.10                 | 0.400                | Speed clamped to 0.10 m/s        |

#### Experimental Validation

The formula parameters were derived from physical experiments on the Franka Panda gripper:

| `closing_speed` | Works     | Fails     | Formula gives |
|-----------------|-----------|-----------|---------------|
| 0.02 m/s        | 0.25 s    | 0.20 s    | 0.36 s        |
| 0.04 m/s        | 0.35 s    | 0.31 s    | 0.37 s        |
| 0.10 m/s        | 0.40 s    | 0.30 s    | 0.40 s        |

All computed values are above the experimentally determined minimum working thresholds and below the 0.50 s danger threshold (where detection may be too late for fragile objects).

#### Implementation

The hold time is computed once when the RT thread transitions to CLOSING (in `applyPendingPhaseTransition()`), using the RT-local copy of closing speed:

```
  rt_T_hold_gripper_ = computeGripperHoldTime(rt_closing_v_cmd_)
```

This value is then used for all stall debounce checks during that CLOSING phase. No new synchronization is needed — the value is written and read by the same (RT) thread.

### Latching

Once CONTACT is declared by either detector, it is **latched** — both detectors stop evaluating. This prevents oscillation at the contact boundary. A new trial (publishing BASELINE) is required to re-arm the detectors and reset all debounce state.

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
| Maximum closing speed   | `v ≤ 0.10 m/s`    | Hard clamp — speeds above 0.10 m/s are clamped with a warning |
| Maximum uplift distance | `d ≤ 0.01 m`      | Hard clamp — any `d > 10 mm` is clamped with a warning        |
| Minimum uplift distance | `d > 0`            | Zero or negative `d` is rejected                               |
| Minimum uplift duration | `T > 0`            | Zero or negative `T` is rejected                               |
| Maximum uplift velocity | `v_max = πd/(2T)`  | Bounded by the clamp on `d` and the minimum `T`               |
| Precondition            | Configurable       | `require_secure_grasp: true` requires SECURE_GRASP state       |
| Duplicate rejection     | —                  | UPLIFT is ignored while a trajectory is already active         |
| BASELINE interruption   | —                  | BASELINE clears UPLIFT, returns to passthrough (with warning)  |

## Configuration

### Controller Parameters (`config/kitting_state_controller.yaml`)

| Parameter                  | Type   | Default | Description                                    |
|----------------------------|--------|---------|------------------------------------------------|
| `arm_id`                   | string | `panda` | Robot arm identifier                           |
| `publish_rate`             | double | `250.0` | State data publish rate [Hz]                   |
| `enable_contact_detector`  | bool   | `true`  | Enable Phase 2 contact detection               |
| `T_base`                   | double | `0.7`   | Baseline collection duration [s]               |
| `N_min`                    | int    | `50`    | Minimum samples before arming detection        |
| `k_sigma`                  | double | `3.0`   | Threshold multiplier (theta = mu + k*sigma)    |
| `T_hold_arm`               | double | `0.10`  | Arm torque debounce hold time [s]              |
| `use_slope_gate`           | bool   | `false` | Enable slope gate (for drift false positives)  |
| `slope_dt`                 | double | `0.02`  | Slope finite difference dt [s]                 |
| `slope_min`                | double | `5.0`   | Minimum slope for contact [1/s]                |
| `stall_velocity_threshold` | double | `0.005` | Gripper speed below this = stalled [m/s]       |
| `width_gap_threshold`      | double | `0.002` | Min gap (w - w_cmd) for stall detection [m]    |
| `stop_on_contact`          | bool   | `true`  | Call `stop()` on contact detection             |
| `enable_arm_contact`       | bool   | `true`  | Enable arm torque contact detector             |
| `enable_gripper_contact`   | bool   | `true`  | Enable gripper stall contact detector          |
| `execute_gripper_actions`  | bool   | `true`  | Execute gripper actions (false = signal-only)   |
| `closing_width`            | double | `0.04`  | Default width for MoveAction in CLOSING [m]    |
| `closing_speed`            | double | `0.04`  | Default speed for MoveAction in CLOSING [m/s] (clamped to max 0.10) |
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
  k_sigma: 3.0
  T_hold_arm: 0.10
  use_slope_gate: false
  slope_dt: 0.02
  slope_min: 5.0
  stall_velocity_threshold: 0.005
  width_gap_threshold: 0.002
  T_hold_gripper: "computed: 0.35 + 0.5 * closing_speed"
  stop_on_contact: true
baseline_statistics:
  baseline_mu_tau_ext_norm: 0.342
  baseline_sigma_tau_ext_norm: 0.051
  contact_threshold_theta: 0.597
```

### CSV Export

When `export_csv_on_stop` is `true` (default), the logger automatically reads back the rosbag after recording stops and writes a flattened CSV file using the C++ `rosbag::View` API. The export runs in a background thread so the stop operation returns immediately.

The CSV contains one row per `KittingState` message (63 columns):

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
| Gripper          | `gripper_width`, `gripper_width_dot`, `gripper_width_cmd`, `gripper_max_width`, `gripper_is_grasped` |

`O_T_EE` (16 values) and `jacobian` (42 values) are not included in the CSV. They remain in the rosbag.

State labels are synchronized by iterating the bag chronologically: each signal row gets the most recent `/kitting_phase2/state` label at that timestamp.

## KittingGripperCommand Message

Per-object gripper command published on `/kitting_phase2/state_cmd`. Any parameter left at `0.0` (the ROS default for `float64`) falls back to the YAML config value loaded at startup. This lets you override only the parameters that differ for a particular object.

| Field              | Type    | Description                                              |
|--------------------|---------|----------------------------------------------------------|
| `command`          | string  | `"CLOSING"`, `"SECURE_GRASP"`, or `"UPLIFT"`            |
| `closing_width`    | float64 | Target width for MoveAction [m] (0 = use default)       |
| `closing_speed`    | float64 | Speed for MoveAction [m/s] (0 = use default, max 0.10)  |
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
| `tau_ext_norm`      | float64      | Euclidean norm of `tau_ext`                          |
| `wrench_norm`       | float64      | Euclidean norm of `wrench_ext`                       |
| `gripper_width`     | float64      | Gripper finger width [m] (from `franka::GripperState`) |
| `gripper_width_dot` | float64      | Gripper width velocity (finite difference) [m/s]    |
| `gripper_width_cmd` | float64      | Commanded closing width [m] (0 if not CLOSING)      |
| `gripper_max_width` | float64      | Maximum gripper opening width [m] (from `franka::GripperState`) |
| `gripper_is_grasped`| bool         | Firmware-level grasp detection (from `franka::GripperState`) |

## Interfaces

The controller claims three hardware interfaces:

- `FrankaStateInterface` -- provides access to `franka::RobotState`
- `FrankaModelInterface` -- provides access to dynamics/kinematics (Jacobian, gravity, Coriolis)
- `FrankaPoseCartesianInterface` -- provides exclusive Cartesian pose command authority for UPLIFT micro-lift

When not executing UPLIFT, the controller operates in **passthrough mode**: it reads the robot's own desired pose (`O_T_EE_d`) and writes it back as the command every tick. This results in zero tracking error and the robot holds position. During UPLIFT, the controller commands a cosine-smoothed trajectory upward. Gripper operations use the libfranka `franka::Gripper` API directly via two dedicated threads:

- **Read thread**: Continuously calls `readOnce()` at firmware rate, computes width velocity via finite difference, writes `GripperData` to `RealtimeBuffer`, and executes `stop()` when the RT loop sets the `stop_requested_` atomic flag
- **Command thread**: Waits on a condition variable for queued `move()`/`grasp()` commands from the subscriber callback thread, then executes them (blocking calls, interruptible by `stop()`)

This eliminates the dependency on the `franka_gripper` ROS package. The launch file sets `load_gripper: false` to prevent conflicting connections (only one `franka::Gripper` connection per robot is allowed).

## Real-Time Safety

- Uses `realtime_tools::RealtimePublisher` with non-blocking `trylock()`
- No dynamic memory allocation in `update()`
- No blocking operations in `update()`
- Model queries are only called at the publish rate, not every control tick
- Contact detection uses only scalar arithmetic (no Eigen, no allocation)
- Gripper data passed to RT loop via lock-free `RealtimeBuffer` (no mutex in `update()`)
- Gripper stop on contact uses single atomic store (`stop_requested_`), nanoseconds, RT-safe
- Read thread checks `stop_requested_` flag after each `readOnce()` and calls `stop()` (non-RT)
- Command thread executes blocking `move()`/`grasp()` (non-RT), interruptible by `stop()`
- No blocking gripper calls in `update()` — all gripper I/O runs in dedicated threads
- State publisher uses `trylock()` pattern (publish only on transition, non-blocking)
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
- Bag contains 5 topics: kitting_state_data, state, state_cmd, record_control, joint_states
- CSV contains 60 flattened columns with correct state labels per row
- CSV export does not block the ROS spin loop (runs in background thread)
- metadata.yaml contains bag_filename, csv_filename, total_samples, start/stop times, and detector parameters
- Hybrid contact detection: arm torque OR gripper stall, first trigger wins
- Gripper stall detection: velocity < threshold AND width gap > threshold for T_hold_gripper (computed: 0.35 + 0.5 * closing_speed)
- Free closing (no object): width reaches w_cmd, gap ~0 → no false CONTACT
- Object contact: width stalls before w_cmd, w_dot ~0, gap > threshold → CONTACT (gripper)
- Arm disturbance during CLOSING: tau_ext_norm > theta → CONTACT (arm)
- Gripper stops immediately on contact: `stop()` called via atomic flag and read thread
- Contact source recorded: "ARM" or "GRIPPER" logged on CONTACT transition
- KittingState includes gripper_width, gripper_width_dot, gripper_width_cmd, gripper_max_width, gripper_is_grasped fields
- Publishing CLOSING on `state_cmd` triggers gripper `move()` + state label
- Publishing SECURE_GRASP on `state_cmd` triggers gripper `grasp()` + state label
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
- Gripper connection failure at init returns false (controller not loaded)
- Controller destructor joins gripper threads cleanly on unload
- `execute_gripper_actions: false` enables signal-only testing mode

## License

Apache 2.0
