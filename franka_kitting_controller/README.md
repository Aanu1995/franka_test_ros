# franka_kitting_controller

Real-time state acquisition and automated force ramp controller for the Franka Panda with gripper stall contact detection and rosbag data collection. Reads all robot state, model, and Cartesian signals, publishes them as a single `KittingState` message, autonomously detects contact using gripper stall detection, immediately stops the gripper on contact via `franka::Gripper::stop()`, and runs an automated force ramp loop (GRASPING → UPLIFT → EVALUATE → SUCCESS, with slip-triggered retry at higher force) internally. Claims `FrankaPoseCartesianInterface` for Cartesian pose control. Gripper operations use the libfranka `franka::Gripper` API directly. Also exposes `franka_gripper`-compatible action servers (move, grasp, homing, stop) for external gripper control when the internal state machine is idle.

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity

**Grasp** adds:

- 11-state grasp machine with automated force ramp: `START` → `BASELINE` → `CLOSING` → `CONTACT` → `GRASPING` → `UPLIFT` → `EVALUATE` → `SUCCESS` (with slip retry loop via `DOWNLIFT` → `SETTLING` → `GRASPING`)
- Gripper stall contact detection during CLOSING
- Immediate gripper stop on contact: calls `franka::Gripper::stop()` to physically halt the motor
- Automated force ramp: increments grasp force from `f_min` to `f_max` in `f_step` increments until stable grasp or failure
- Load-transfer drop metric slip detection: two-gate wrench_norm evaluation during EVALUATE hold
- One rosbag per trial with all state transitions (recording starts automatically on launch)
- Automatic CSV export on stop for analysis-ready datasets
- Per-trial metadata output for offline analysis reproducibility

## Dependencies

- ROS Noetic (or Melodic)
- [libfranka](https://github.com/frankaemika/libfranka) >= 0.8.0
- [franka_ros](https://github.com/frankaemika/franka_ros) (specifically `franka_hw`, `franka_control`, `franka_gripper` for action type definitions)
- `actionlib` (for gripper action servers)

## Building

```bash
# From your catkin workspace root
catkin_make
# or
catkin build franka_kitting_controller
```

## Usage

### Prerequisites: Launch the robot

The kitting controller does **not** launch the robot — it assumes `franka_control` is already running. Launch the robot first:

```bash
roslaunch franka_control franka_control.launch robot_ip:=<ROBOT_IP> load_gripper:=false
```

Use `load_gripper:=false` because the kitting controller connects to the gripper directly via `libfranka`. Running `franka_gripper_node` simultaneously would create a connection conflict.

### Launch the controller

Loads config, spawns the `kitting_state_controller` into the running `controller_manager`, and connects to the gripper.

```bash
roslaunch franka_kitting_controller kitting_state_controller.launch robot_ip:=<ROBOT_IP>
```

| Argument   | Default    | Description               |
| ---------- | ---------- | ------------------------- |
| `robot_ip` | (required) | Franka robot IP address   |

### Recording (optional)

To enable rosbag recording, pass `record:=true` to the same launch command. The logger node starts automatically alongside the controller — no separate launch needed.

```bash
# With recording
roslaunch franka_kitting_controller kitting_state_controller.launch robot_ip:=<ROBOT_IP> \
  record:=true object_name:=cup base_directory:=/data/kitting
```

| Argument             | Default          | Description                          |
| -------------------- | ---------------- | ------------------------------------ |
| `record`             | `false`          | Enable rosbag recording              |
| `base_directory`     | `~/kitting_bags` | Root directory for all trial data    |
| `object_name`        | `default_object` | Object name for bag file naming      |
| `export_csv_on_stop` | `true`           | Auto-export CSV when recording stops |

## Grasp: Interaction States

Eleven interaction states structure the grasp execution sequence. The controller starts in `START` and waits for the user to publish `BASELINE` before any Grasp activity begins. Two modes of operation:

- **Manual mode**: The user sends three commands (`BASELINE`, `CLOSING`, `GRASPING`); all subsequent states are driven internally by the automated force ramp.
- **Auto mode**: A single `AUTO` command runs the full sequence automatically with configurable delays between transitions.

States are published on `/kitting_controller/state` for signal labeling and offline analysis. If recording is enabled (`record:=true`), recording starts automatically and continues through all state transitions.

```
Manual:          BASELINE ──> CLOSING ──> CONTACT ──> GRASPING
                 (user cmd)   (user cmd)  (auto)      (user cmd)

Auto (single command):
                 BASELINE ─(delay)─> CLOSING ─(wait)─> CONTACT ─(delay)─> GRASPING
                                                                              │
Force ramp (both modes):  ┌───────────────────────────────────────────────────┘
                          │
                          v
                      GRASPING ──> UPLIFT ──> EVALUATE ──> SUCCESS
                          ^                       │
                          │                  [slip detected]
                          │                       │
                          │                       v
                      SETTLING <── DOWNLIFT <─────┘
                          │
                     [F > f_max]
                          │
                          v
                        FAILED
```

CONTACT is auto-detected during CLOSING. SUCCESS and FAILED are terminal states.

### START

Initial state after the controller is launched. No baseline collection, no contact detection — just Cartesian passthrough (hold position) and state data publishing at 250 Hz.

- Controller is running and publishing `KittingState` data
- No Grasp activity: no baseline statistics, no gripper commands accepted
- All commands on `/kitting_controller/state_cmd` are rejected until `BASELINE` or `AUTO` is published
- If `record:=true`, commands are also gated until the logger node is ready
- Transition: publish `BASELINE` on `/kitting_controller/state_cmd`

**Logger readiness gate** (only when `record:=true`): The controller subscribes to `/kitting_controller/logger_ready` (a latched `std_msgs/Bool` topic published by the logger node). Until this signal is received, all state commands are rejected. When `record:=false` (default), this gate is skipped and commands are accepted immediately.

### BASELINE

Prepare for a new grasp trial. Published via `/kitting_controller/state_cmd` with `command: "BASELINE"`.

- **Optional gripper open**: If `open_gripper: true`, the gripper opens to `open_width` (or `max_width` from firmware if not specified) at 0.1 m/s
- Resets all state machine variables (contact latch, force ramp, trajectories) for a fresh grasp cycle
- BASELINE is a one-shot state — publish once from START to begin the grasp sequence

### CLOSING

Observe approach dynamics before contact.

- Gripper begins closing toward object at width `w` and speed `v` (max 0.10 m/s)
- Gripper stall contact detection runs: finger width velocity stalls while gap to target width remains
- CONTACT is latched immediately when detected
- If gripper reaches target width without stalling (no object present), transitions to **FAILED** — "no contact detected"

### CONTACT

Detect first stable physical interaction. Published **automatically** by the controller.

- Object touches gripper fingers — detected by gripper stall detection
- Width velocity drops below `stall_velocity_threshold` (0.008 m/s) while `(width - w_cmd) > width_gap_threshold` (0.002 m), sustained for `T_hold_gripper` seconds (computed dynamically from `closing_speed`; see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper))
- `contact_width` is saved immediately at stall detection — used as the grasp width in GRASPING
- **Deferred transition**: On stall detection, `franka::Gripper::stop()` is requested via the read thread. The state remains CLOSING until the gripper has physically stopped (`gripper_stopped_` flag set by the read thread), then transitions to CONTACT. This ensures all data recorded during CONTACT reflects a stopped gripper.
- CONTACT is **latched** once detected — cannot return to CLOSING

### GRASPING

Initiate automated force ramp. Published via `/kitting_controller/state_cmd` with `command: "GRASPING"`. This is the **last user command** — all subsequent states are driven internally by the force ramp.

- Gripper applies force `F` via GraspAction to width `w` with tolerance `ε`
- Grasp width is always the `contact_width` captured at CONTACT (not configurable)
- Initial force is `f_min` (default 3.0 N); on slip, force increments by `f_step` (default 3.0 N) up to `f_max` (default 30.0 N)
- After grasp completion + stabilization delay, the controller automatically transitions to UPLIFT
- **Timeout**: If the grasp command does not complete within 10 seconds, the controller transitions to FAILED

### UPLIFT

Validate grasp robustness under load. **Auto-triggered** by the controller after GRASPING completes — not a user command.

- Controller displaces end-effector upward by `fr_uplift_distance` (default 5 mm, max 10 mm)
- Duration computed from distance and speed: `T = fr_uplift_distance / fr_lift_speed`
- Cosine-smoothed trajectory `s = 0.5(1 - cos(π · s_raw))` ensures zero velocity at start and end
- Only z-translation of `O_T_EE_d[14]` is modified — orientation and x/y unchanged
- Reference signals (`tau_ext_norm`, gripper width) are recorded before lift for slip comparison
- On trajectory completion, automatically transitions to EVALUATE

### EVALUATE

Assess grasp stability at the lifted position. **Auto-triggered** after UPLIFT completes.

- Holds position for `uplift_hold` seconds (default 0.10 s): early window (first half) + late window (second half)
- Uses `wrench_norm` (Cartesian external wrench) for load-transfer slip detection
- Two-gate evaluation (see [Slip Detection](#grasp-slip-detection-load-transfer-drop-metric)):
  - **Gate 1**: Load transfer confirmation — `delta_F > max(3*sigma_pre, 1.0)`
  - **Gate 2**: Drop ratio + width change thresholds
- If **secure** (both gates pass): transitions to SUCCESS
- If **slip** (either gate fails): transitions to DOWNLIFT to retry with higher force

### DOWNLIFT

Return end-effector to pre-lift height after slip detection. **Auto-triggered** after EVALUATE detects slip.

- Controller displaces end-effector downward by `fr_uplift_distance` (same distance as UPLIFT)
- Same cosine-smoothed trajectory as UPLIFT but descending: `z(t) = z_start - s * d`
- Duration computed identically: `T = fr_uplift_distance / fr_lift_speed`
- On trajectory completion, automatically transitions to SETTLING

### SETTLING

Post-downlift stabilization before force increment. **Auto-triggered** after DOWNLIFT completes.

- Waits for `fr_stabilization` seconds (default 0.5 s) for signals to settle
- Increments grasp force: `f_current += f_step`
- If `f_current > f_max`: transitions to FAILED (max force exceeded)
- Otherwise: re-enters GRASPING with the incremented force, starting a new force ramp iteration

### SUCCESS

Terminal state indicating stable grasp confirmed. **Auto-triggered** when EVALUATE finds no slip.

- The grasp held at force `f_current` without slip
- Robot remains at the lifted position holding the object
- State label published for offline analysis
- **Recording auto-stops**: if `record:=true`, the logger detects SUCCESS and automatically stops recording (saves bag + metadata + CSV)
- To start a new trial, publish `BASELINE` (resets the entire state machine)

### FAILED

Terminal state indicating grasp failure. **Auto-triggered** on any of:

- No contact detected during CLOSING (gripper reached target width without stalling)
- Maximum force exceeded (`f_current > f_max`) after all force ramp iterations
- GRASPING timeout (gripper command did not complete within 10 seconds)
- State label published for offline analysis with diagnostic information
- **Recording auto-stops**: if `record:=true`, the logger detects FAILED and automatically stops recording (saves bag + metadata + CSV)
- To retry, publish `BASELINE` (resets the entire state machine)

### AUTO (Automatic Full Sequence)

A single command that runs the full grasp sequence automatically. Published via `/kitting_controller/state_cmd` with `command: "AUTO"`.

- Executes BASELINE → *(delay)* → CLOSING → *(wait for CONTACT)* → *(delay)* → GRASPING automatically
- `auto_delay` parameter controls the wait time between transitions (default 5.0 seconds)
- All BASELINE, CLOSING, and GRASPING parameters are accepted and forwarded to each stage
- The force ramp runs autonomously after GRASPING (same as manual mode)
- If CLOSING results in FAILED (no contact), auto mode ends at FAILED
- Any manual command (`BASELINE`, `CLOSING`, `GRASPING`) published during auto mode cancels auto mode
- Re-sending `AUTO` cancels the current auto sequence and starts a new one

### Topics

Recording and state labeling are independent concerns.

| Topic                                | Type                  | Direction  | Description                                |
| ------------------------------------ | --------------------- | ---------- | ------------------------------------------ |
| `<ns>/kitting_state_data`            | KittingState          | Published  | Full state data at 250 Hz                  |
| `/kitting_controller/state_cmd`      | KittingGripperCommand | Subscribed | User commands: BASELINE, CLOSING, GRASPING, AUTO |
| `/kitting_controller/state`          | std_msgs/String       | Published  | State labels for offline segmentation      |
| `/kitting_controller/record_control` | std_msgs/String       | Subscribed | Recording control: STOP, ABORT             |
| `/kitting_controller/logger_ready`   | std_msgs/Bool         | Subscribed | Latched readiness signal from logger node  |
| `/franka_gripper/move`               | franka_gripper/MoveAction   | ActionServer | Move fingers to width at speed        |
| `/franka_gripper/grasp`              | franka_gripper/GraspAction  | ActionServer | Grasp with width, speed, force, epsilon |
| `/franka_gripper/homing`             | franka_gripper/HomingAction | ActionServer | Gripper homing (recalibrate)          |
| `/franka_gripper/stop`               | franka_gripper/StopAction   | ActionServer | Emergency gripper stop (always allowed) |

- `/kitting_controller/state_cmd` — The **user** publishes a `KittingGripperCommand` with `command` field set to `BASELINE`, `CLOSING`, `GRASPING`, or `AUTO`, plus optional per-object parameters. Any float64 parameter left at `0.0` falls back to the YAML config default. The **controller** executes the corresponding action, publishes the state label on `/kitting_controller/state`, and drives the force ramp internally. `AUTO` runs the full sequence automatically with configurable delays.
- `/kitting_controller/state` — The **controller** publishes all 11 state labels: START, BASELINE, CLOSING, CONTACT, GRASPING, UPLIFT, EVALUATE, DOWNLIFT, SETTLING, SUCCESS, FAILED. States are labels for offline analysis. Terminal states (SUCCESS, FAILED) also trigger automatic recording stop when the logger is running.
- `/kitting_controller/record_control` — The **user** publishes STOP or ABORT to end recording. Recording starts automatically when the logger launches — no START command is needed. The **logger** subscribes to this topic.
- `/kitting_controller/logger_ready` — The **logger** publishes `true` (latched) on startup. When `record:=true`, the **controller** subscribes and gates Grasp commands behind this signal. When `record:=false`, this topic is not used.

### Recording

One rosbag per trial. **Recording starts automatically** when the logger node is launched — no START command is needed. The logger opens a new trial bag immediately on startup and records all configured topics.

| Command | Action                                            |
| ------- | ------------------------------------------------- |
| `STOP`  | Close bag, save metadata, export CSV (if enabled) |
| `ABORT` | Close bag, delete trial directory (no CSV)        |

- **STOP** is ignored if not recording.
- **ABORT** is ignored if not recording.
- If the logger node is shut down (Ctrl+C), recording is automatically stopped (equivalent to STOP).
- **Auto-stop on terminal state**: When the logger detects SUCCESS or FAILED on `/kitting_controller/state`, recording stops automatically (equivalent to STOP). The terminal state message is written to the bag before it is closed.

Recording continues through all state transitions until stopped by STOP, ABORT, terminal state (SUCCESS/FAILED), or node shutdown.

### Sending Commands

```bash
# Recording starts automatically when the logger is launched — no START needed.

# BASELINE — REQUIRED before any other state command
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'BASELINE'}" --once

# BASELINE — open gripper to max_width before collecting
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'BASELINE', open_gripper: true}" --once

# BASELINE — open gripper to specific width
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'BASELINE', open_gripper: true, open_width: 0.06}" --once

# CLOSING — use YAML defaults for gripper parameters (all params = 0 -> defaults)
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'CLOSING'}" --once

# CLOSING — override width and speed for a specific object
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'CLOSING', closing_width: 0.01, closing_speed: 0.05}" --once

# ... CONTACT is published by the controller automatically ...

# GRASPING — use YAML defaults (f_min=3N, f_max=30N, f_step=3N)
# Uses contact_width from CONTACT as grasp width. After this command, the force ramp
# runs autonomously: GRASPING → UPLIFT → EVALUATE → SUCCESS (or retry → FAILED)
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING'}" --once

# GRASPING — override force range for a specific object
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', f_min: 3.0, f_max: 30.0, f_step: 3.0}" --once

# GRASPING — override all force ramp parameters for a specific object
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', \
    f_min: 3.0, f_step: 3.0, f_max: 30.0, \
    fr_grasp_speed: 0.02, fr_epsilon: 0.008, \
    fr_uplift_distance: 0.005, fr_lift_speed: 0.01, fr_uplift_hold: 0.10, \
    fr_stabilization: 0.5, fr_slip_tau_drop: 0.20, fr_slip_width_change: 0.001}" --once

# Stop recording
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'STOP'" --once

# Or abort (delete trial)
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'ABORT'" --once
```

### Auto Mode

A single `AUTO` command runs the full sequence: BASELINE → *(delay)* → CLOSING → *(wait for CONTACT)* → *(delay)* → GRASPING. All parameters from BASELINE, CLOSING, and GRASPING are accepted and forwarded to each stage.

```bash
# AUTO — run full sequence automatically (default 5s delay between transitions)
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'AUTO', open_gripper: true}" --once

# AUTO — with custom delay and parameters
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'AUTO', open_gripper: true, auto_delay: 3.0, \
    closing_width: 0.01, closing_speed: 0.05, \
    f_min: 3.0, f_max: 30.0, f_step: 3.0}" --once
```

## Gripper Action Servers

The controller exposes four `franka_gripper`-compatible action servers under the `/franka_gripper/` namespace. These allow external nodes (e.g., a MoveIt-based main controller) to command the gripper through standard ROS action interfaces — without running `franka_gripper_node` (which would conflict with the controller's own gripper connection).

### State Guard

Action server requests are only accepted when the internal state machine is **idle**: `START`, `SUCCESS`, or `FAILED`. During an active grasp sequence (BASELINE through EVALUATE), the state machine owns the gripper and external actions are rejected with an error describing the current state.

### Available Actions

| Action                    | Type                       | Description                                              |
| ------------------------- | -------------------------- | -------------------------------------------------------- |
| `/franka_gripper/move`    | `franka_gripper/MoveAction`   | Move fingers to a target width at a given speed          |
| `/franka_gripper/grasp`   | `franka_gripper/GraspAction`  | Grasp with specified width, speed, force, and epsilon    |
| `/franka_gripper/homing`  | `franka_gripper/HomingAction` | Run the gripper homing routine (recalibrate)             |
| `/franka_gripper/stop`    | `franka_gripper/StopAction`   | Immediately stop the gripper (always allowed, any state) |

**Move**, **Grasp**, and **Homing** are routed through the existing gripper command thread to prevent concurrent `libfranka` calls. **Stop** calls `gripper_->stop()` directly — it is thread-safe and must work even when another command is executing (to abort it).

### Usage Examples

```bash
# Open gripper to 8 cm at 0.1 m/s
rostopic pub /franka_gripper/move/goal franka_gripper/MoveActionGoal \
  "goal: {width: 0.08, speed: 0.1}" --once

# Grasp at 4 cm width, 0.02 m/s, 10 N force, epsilon 0.005
rostopic pub /franka_gripper/grasp/goal franka_gripper/GraspActionGoal \
  "goal: {width: 0.04, speed: 0.02, force: 10.0, epsilon: {inner: 0.005, outer: 0.005}}" --once

# Run homing routine
rostopic pub /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}" --once

# Emergency stop
rostopic pub /franka_gripper/stop/goal franka_gripper/StopActionGoal "{}" --once
```

### Drop-in Compatibility

The `/franka_gripper/` namespace matches the default namespace used by `franka_gripper_node`. Clients that already target `franka_gripper_node` (e.g., MoveIt pick-and-place pipelines) can use these action servers without modification — just ensure `franka_gripper_node` is not running simultaneously.

## Grasp: Contact Detection

The controller uses **gripper stall detection** during CLOSING. When the gripper is commanded to close but the fingers stop moving before reaching the target width, an object is blocking them. Contact detection is always enabled.

### Gripper Stall Detection

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
  velocity_stalled = |w_dot| < stall_velocity_threshold    (default 0.008 m/s)
  width_gap_exists = (w - w_cmd) > width_gap_threshold     (default 0.002 m)

  stall_detected = velocity_stalled AND width_gap_exists
```

The **width gap check** is essential: without it, normal gripper completion (velocity drops to 0 at target width) would false-trigger as contact. The gap ensures we only detect stalls where the gripper stopped **before** reaching its commanded width.

**Debounce**: The stall condition must persist for `T_hold_gripper` seconds (computed dynamically from `closing_speed`; see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper)). If the stall condition breaks at any point, the debounce timer resets.

#### Symbols

| Symbol           | Name                     | Unit | Default  | Description                                                                                                                         |
| ---------------- | ------------------------ | ---- | -------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `w`              | Gripper width            | m    | —        | Current finger width (from `franka::GripperState.width`)                                                                            |
| `w_dot`          | Width velocity           | m/s  | —        | Finite difference of `w` between consecutive `readOnce()` calls                                                                     |
| `w_cmd`          | Commanded width          | m    | 0.01     | Target width for the active MoveAction (from CLOSING command)                                                                       |
| `v_stall`        | Stall velocity threshold | m/s  | 0.008   | Speed below this is considered stalled                                                                                              |
| `Δw`             | Width gap threshold      | m    | 0.002    | Minimum `(w - w_cmd)` to distinguish stall from normal completion                                                                   |
| `T_hold_gripper` | Debounce time            | s    | computed | Duration stall must persist to declare contact (see [Dynamic Gripper Debounce Time](#dynamic-gripper-debounce-time-t_hold_gripper)) |

### Gripper Stop on Contact

When stall contact is detected, the controller stops the gripper and defers the CONTACT state transition until the stop is confirmed:

1. **Stall detected (RT thread)**: Sets `contact_latched_`, stores `contact_width_`, and sets `stop_requested_` (single atomic store, nanoseconds). State remains **CLOSING**.
2. **Read thread executes stop**: Checks `stop_requested_` after each `readOnce()`, calls `franka::Gripper::stop()` to physically halt the motor, then sets `gripper_stopped_` to signal the RT thread.
3. **Deferred transition (RT thread)**: On the next tick, sees `contact_latched_ && gripper_stopped_` and transitions to **CONTACT**.
4. **One-shot guard**: `gripper_stop_sent_` flag prevents duplicate stop requests; `contact_latched_` prevents re-detection.

The atomic stores in `update()` are single-word writes (nanoseconds), fully RT-safe. The actual `stop()` call executes in the non-RT read thread. The state transition is deferred by ~20–30 ms (one gripper firmware update period) to ensure the gripper has physically stopped before CONTACT data is recorded.

### Dynamic Gripper Debounce Time (`T_hold_gripper`)

The gripper stall debounce time is **computed dynamically** from `closing_speed` rather than configured statically. Higher closing speeds produce more velocity noise — `w_dot` briefly reads ~0 even when the gripper is still moving — so the debounce window must increase with speed to avoid false contact triggers.

#### Formula

```
  T_hold_gripper  =  0.35  +  0.5 · clamp(v, 0, 0.10)
```

Where `v` is the resolved `closing_speed` (YAML default or per-command override, clamped to `[0, v_max]`).

#### Constants

| Symbol        | Value       | Description                                               |
| ------------- | ----------- | --------------------------------------------------------- |
| `T_base_hold` | 0.35 s      | Hold time intercept at zero speed                         |
| `k_hold`      | 0.5 s/(m/s) | Linear slope: hold time increase per unit speed           |
| `v_max`       | 0.10 m/s    | Hard cap on closing speed (speeds above this are clamped) |

#### Resulting Hold Times

| `closing_speed` (m/s) | `T_hold_gripper` (s) | Notes                     |
| --------------------- | -------------------- | ------------------------- |
| 0.01                  | 0.355                |                           |
| 0.02                  | 0.360                | Verified                  |
| 0.04                  | 0.370                | Verified experimentally   |
| 0.05                  | 0.375                | Default speed             |
| 0.06                  | 0.380                | Verified experimentally   |
| 0.08                  | 0.390                |                           |
| 0.10                  | 0.400                | Maximum speed, verified   |
| > 0.10                | 0.400                | Speed clamped to 0.10 m/s |

#### Experimental Validation

The formula parameters were derived from physical experiments on the Franka Panda gripper:

| `closing_speed` | Works  | Fails  | Formula gives |
| --------------- | ------ | ------ | ------------- |
| 0.02 m/s        | 0.25 s | 0.20 s | 0.36 s        |
| 0.04 m/s        | 0.35 s | 0.31 s | 0.37 s        |
| 0.10 m/s        | 0.40 s | 0.30 s | 0.40 s        |

All computed values are above the experimentally determined minimum working thresholds and below the 0.50 s danger threshold (where detection may be too late for fragile objects).

#### Implementation

The hold time is computed once when the RT thread transitions to CLOSING (in `applyPendingStateTransition()`), using the RT-local copy of closing speed:

```
  rt_T_hold_gripper_ = computeGripperHoldTime(rt_closing_v_cmd_)
```

This value is then used for all stall debounce checks during that CLOSING state. No new synchronization is needed — the value is written and read by the same (RT) thread.

### Latching

Once CONTACT is declared, it is **latched** — the detector stops evaluating. This prevents oscillation at the contact boundary. A new BASELINE command resets the latch for the next trial.

## Grasp: Slip Detection (Load-Transfer Drop Metric)

After each UPLIFT, the controller evaluates grasp quality using a two-gate load-transfer approach. Uses `wrench_norm` (Cartesian external wrench norm, in Newtons) rather than joint torques for direct physical interpretability.

### Time Windows

The slip evaluation uses three time windows across the grasp-lift-hold sequence:

| Window         | Duration         | When                            | Signal           | Purpose                     |
| -------------- | ---------------- | ------------------------------- | ---------------- | --------------------------- |
| **W_pre**      | `stabilization`  | Post-grasp stabilization        | `wrench_norm`    | Baseline before lift        |
| **W_hold_early** | `uplift_hold/2`| First half of EVALUATE          | `wrench_norm`    | Loaded reference after lift |
| **W_hold_late**  | `uplift_hold/2`| Second half of EVALUATE         | `wrench_norm`    | Late hold for drop check   |

```
  GRASPING (stab.)            UPLIFT         EVALUATE (uplift_hold)
  ├── W_pre [stabilization]   [lift]         ├── W_hold_early [0, hold/2)
  │   mu_pre, sigma_pre                      └── W_hold_late  [hold/2, hold)
  │                                               mu_early, mu_late
```

### Gate 1: Load Transfer Confirmation

Verify the object weight is actually on the gripper after lifting:

```
  mu_pre    = mean(wrench_norm over W_pre)
  sigma_pre = std(wrench_norm over W_pre)
  mu_early  = mean(wrench_norm over W_hold_early)

  delta_F = mu_early - mu_pre

  Load transfer confirmed if:  delta_F > max(3 * sigma_pre, 1.0)
```

The `3 * sigma_pre` threshold adapts to sensor noise. The `1.0` N floor prevents declaring load transfer from noise alone. If load transfer is NOT confirmed, the grasp is treated as unstable (SLIP) regardless of the drop ratio.

### Gate 2: Drop Ratio (Slip During Hold)

If load transfer is confirmed, check whether the wrench dropped during the hold:

```
  mu_late = mean(wrench_norm over W_hold_late)

  drop_ratio = (mu_early - mu_late) / max(mu_early, 1e-6)
```

A positive `drop_ratio` means the wrench decreased from the early to late window, suggesting the object is slipping out. Additionally, gripper width increase is tracked:

```
  width_change = max_width_during_hold - width_before_uplift
```

### Final Decision

```
  SECURE if:  load_transfer confirmed
              AND drop_ratio <= fr_slip_tau_drop  (default 0.20)
              AND width_change <= fr_slip_width_change  (default 0.001 m)

  SLIP otherwise → DOWNLIFT → SETTLING → GRASPING (retry with F += f_step)
```

| Parameter              | Default | Description                                           |
| ---------------------- | ------- | ----------------------------------------------------- |
| `fr_slip_tau_drop`     | 0.20    | Drop ratio threshold (0.20 = 20% wrench drop)        |
| `fr_slip_width_change` | 0.001 m | Gripper width increase threshold                      |

### Example Log Output

Secure grasp (load transfer confirmed, no drop):

```
  [EVALUATE]  mu_pre=3.210  sigma_pre=0.142  mu_early=5.834  mu_late=5.712
              delta_F=2.624  load_xfer=YES  drop=0.021 (thresh=0.200)
              width_change=0.0002 (thresh=0.0010)  SECURE
```

- `delta_F=2.624` > `max(3*0.142, 1.0)` = 1.0 → load transfer confirmed
- `drop=0.021` (2.1%) < 0.20 → no significant drop
- **Verdict: SECURE → SUCCESS**

Slip detected — load transferred but object slipped during hold:

```
  [EVALUATE]  mu_pre=3.180  sigma_pre=0.138  mu_early=5.920  mu_late=4.510
              delta_F=2.740  load_xfer=YES  drop=0.238 (thresh=0.200)
              width_change=0.0014 (thresh=0.0010)  SLIP
```

- `delta_F=2.740` > `max(0.414, 1.0)` = 1.0 → load transfer confirmed (object was lifted)
- `drop=0.238` (23.8%) > 0.20 → wrench dropped during hold (object sliding out)
- `width_change=0.0014` > 0.001 → gripper opened 1.4 mm (fingers pushed apart)
- **Verdict: SLIP → DOWNLIFT → retry with more force**

Failed load transfer (object not lifted):

```
  [EVALUATE]  mu_pre=3.210  sigma_pre=0.142  mu_early=3.352  mu_late=3.298
              delta_F=0.142  load_xfer=NO  drop=0.016 (thresh=0.200)
              width_change=0.0001 (thresh=0.0010)  SLIP
```

- `delta_F=0.142` < `max(0.426, 1.0)` = 1.0 → load transfer NOT confirmed
- Drop ratio and width are fine, but Gate 1 failed — object weight never appeared on the gripper
- **Verdict: SLIP → DOWNLIFT → retry with more force**

### How to Read the Log

| Field | What it tells you |
| ----- | ----------------- |
| `mu_pre` | Baseline wrench at table level (before lift) |
| `sigma_pre` | Sensor noise during baseline — higher means noisier signal |
| `mu_early` | Wrench right after lift — should be much higher than `mu_pre` if object is lifted |
| `mu_late` | Wrench later in hold — should stay close to `mu_early` if no slip |
| `delta_F` | How much wrench increased from lift; must exceed `max(3*sigma_pre, 1.0)` |
| `load_xfer` | YES/NO — did the object weight actually transfer to the gripper? |
| `drop` | Fractional wrench decrease during hold; > 0.20 means slip |
| `width_change` | How much the gripper opened during hold; > 0.001 m means slip |
| `SECURE/SLIP` | Final verdict — SECURE needs all three checks to pass |

**Quick diagnostic guide:**
- `load_xfer=NO` → object not gripped or too light; increase force
- `load_xfer=YES` + `drop > thresh` → object slipping out during hold; increase force
- `load_xfer=YES` + `width_change > thresh` → fingers opening; increase force
- `load_xfer=YES` + `drop` and `width` both within thresholds → **SECURE grasp**

## Grasp: UPLIFT / DOWNLIFT Trajectory Mathematics

The UPLIFT and DOWNLIFT states execute smooth Cartesian micro-lifts internally using cosine-smoothed time-based trajectory interpolation. Both trajectories run at the control loop rate (1 kHz) and command the end-effector pose via `FrankaPoseCartesianInterface`. These are **auto-triggered** by the force ramp — not user commands.

### Symbols

| Symbol     | Name                      | Unit | Default  | Description                                                                |
| ---------- | ------------------------- | ---- | -------- | -------------------------------------------------------------------------- |
| `d`        | Lift distance             | m    | 0.005    | Total displacement along the z-axis (max 0.01)                             |
| `v_lift`   | Lift speed                | m/s  | 0.01     | Speed for both UPLIFT and DOWNLIFT                                         |
| `T`        | Lift duration             | s    | computed | `T = d / v_lift` (computed from distance and speed)                        |
| `t`        | Elapsed time              | s    | —        | Time since lift started, incremented by `Δt` (control period) each tick    |
| `Δt`       | Control period            | s    | 0.001    | Time between consecutive `update()` calls (1 kHz)                          |
| `s_raw`    | Normalized time           | —    | —        | Linear progress: `s_raw = min(t/T, 1)`, clamped to [0, 1]                  |
| `s`        | Smoothed progress         | —    | —        | Cosine-smoothed: `s = ½(1 − cos(π · s_raw))`, maps [0,1] → [0,1]           |
| `z₀`       | Start z-position          | m    | —        | z-translation of `O_T_EE_d` at the moment lift begins: `z₀ = O_T_EE_d[14]` |
| `z(t)`     | Commanded z-position      | m    | —        | UPLIFT: `z(t) = z₀ + s · d`, DOWNLIFT: `z(t) = z₀ - s · d`                 |
| `O_T_EE_d` | Desired end-effector pose | —    | —        | 4×4 column-major homogeneous transform; index [14] = z-translation         |

### Trajectory Equation

**UPLIFT** smoothly moves the end-effector from `z₀` to `z₀ + d` over duration `T`:

```
              t
  s_raw  =  min( ─── , 1 )
              T

         1
  s  =  ─── ( 1  −  cos( π · s_raw ) )
         2

  z(t)  =  z₀  +  s · d       (UPLIFT)
  z(t)  =  z₀  -  s · d       (DOWNLIFT)
```

The **velocity profile** (first derivative) is:

```
  dz       π · d
  ── (t) = ───── · sin( π · t / T )
  dt        2T
```

### Trajectory Properties

| Property                | Mathematical expression                | Physical meaning                               |
| ----------------------- | -------------------------------------- | ---------------------------------------------- |
| Position at `t = 0`     | `z(0) = z₀`                            | Starts at the current height                   |
| Position at `t = T`     | `z(T) = z₀ ± d`                        | Ends exactly `d` meters above/below start      |
| Velocity at `t = 0`     | `dz/dt = 0`                            | Zero velocity at start (no step discontinuity) |
| Velocity at `t = T`     | `dz/dt = 0`                            | Zero velocity at end (smooth stop)             |
| Peak velocity           | `v_max = πd/(2T)` at `t = T/2`         | Maximum speed at the midpoint of the motion    |
| Peak velocity (default) | `v_max = π·0.005/(2·0.5) ≈ 0.0157 m/s` | ~15.7 mm/s — well within Franka limits         |

### Execution Details

1. **Start**: When the force ramp triggers UPLIFT (after GRASPING completes), the controller captures the current `O_T_EE_d` as the start pose and records `z₀ = O_T_EE_d[14]`. Distance `d` and speed `v_lift` are taken from RT-local copies; duration is computed as `T = d / v_lift`. DOWNLIFT uses the same mechanism but descends.

2. **Per-tick** (1 kHz): The controller increments `t ← t + Δt`, computes `s_raw`, `s`, and `z(t)`, then calls `setCommand(pose)`. Only the z-translation (index [14]) is modified — orientation and x/y position remain unchanged from the start pose.

3. **Completion**: When `t ≥ T`, the trajectory is done. The active flag is cleared and the controller transitions to the next state (EVALUATE after UPLIFT, SETTLING after DOWNLIFT).

4. **Passthrough mode**: When not executing any trajectory, the controller reads `O_T_EE_d` and writes it back as the command every tick. This produces zero tracking error — the robot holds position with no drift or jerk.

### Safety Constraints

| Constraint              | Symbol / Value | Description                                                                |
| ----------------------- | -------------- | -------------------------------------------------------------------------- |
| Maximum closing speed   | `v ≤ 0.10 m/s` | Hard clamp — speeds above 0.10 m/s are clamped with a warning              |
| Maximum uplift distance | `d ≤ 0.01 m`   | Hard clamp — any `d > 10 mm` is clamped with a warning                     |
| Precondition            | —              | GRASPING requires CONTACT state                                           |
| BASELINE interruption   | —              | BASELINE clears active trajectories, returns to passthrough (with warning) |
| GRASPING timeout        | 10 s           | Transitions to FAILED if gripper command does not complete                 |
| Force ramp limit        | `F ≤ f_max`    | Transitions to FAILED if maximum force is exceeded                         |

## Configuration

### Controller Parameters (`config/kitting_state_controller.yaml`)

| Parameter                  | Type   | Default | Description                                                         |
| -------------------------- | ------ | ------- | ------------------------------------------------------------------- |
| `arm_id`                   | string | `panda` | Robot arm identifier                                                |
| `publish_rate`             | double | `250.0` | State data publish rate [Hz]                                        |
| `stall_velocity_threshold` | double | `0.008` | Gripper speed below this = stalled [m/s]                            |
| `width_gap_threshold`      | double | `0.002` | Min gap (w - w_cmd) for stall detection [m]                         |
| `closing_width`            | double | `0.01`  | Default width for MoveAction in CLOSING [m]                         |
| `closing_speed`            | double | `0.05`  | Default speed for MoveAction in CLOSING [m/s] (clamped to max 0.10) |
| `grasp_speed`              | double | `0.02`  | Gripper speed for GraspAction [m/s]                                 |
| `epsilon`                  | double | `0.008` | Epsilon for GraspAction (inner and outer) [m]                       |
| `f_min`                    | double | `3.0`   | Starting grasp force [N]                                            |
| `f_step`                   | double | `3.0`   | Force increment per iteration [N]                                   |
| `f_max`                    | double | `30.0`  | Maximum force — FAILED if exceeded [N]                              |
| `uplift_distance`          | double | `0.005` | Micro-uplift distance per iteration [m] (max 0.01)                  |
| `lift_speed`               | double | `0.01`  | Lift speed for UPLIFT and DOWNLIFT [m/s]                            |
| `uplift_hold`              | double | `0.10`  | Hold time: early (first half) + late (second half) windows [s]      |
| `stabilization`            | double | `0.5`   | Post-grasp settle time; also W_pre baseline window [s]              |
| `slip_tau_drop`            | double | `0.20`  | Drop ratio threshold: (mu_early−mu_late)/mu_early                   |
| `slip_width_change`        | double | `0.001` | Width change threshold for slip [m]                                 |

Gripper and GRASPING parameters are **defaults**. They can be overridden per-command by setting non-zero values in the `KittingGripperCommand` message published on `/kitting_controller/state_cmd`.

### Logger Parameters (`config/kitting_logger.yaml`)

| Parameter            | Type        | Default          | Description                          |
| -------------------- | ----------- | ---------------- | ------------------------------------ |
| `base_directory`     | string      | `~/kitting_bags` | Root directory for bag files         |
| `object_name`        | string      | `default_object` | Object identifier for file naming    |
| `export_csv_on_stop` | bool        | `true`           | Auto-export CSV when recording stops |
| `topics_to_record`   | string list | (see below)      | Topics recorded in rosbag            |

Default recorded topics:

- `/kitting_state_controller/kitting_state_data`
- `/kitting_controller/state`

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

The single bag contains all data from all states (BASELINE through SUCCESS/FAILED) for that trial. Recording starts automatically on launch and stops on STOP, ABORT, or node shutdown. The CSV is exported in a background thread so STOP returns immediately.

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
  - /kitting_controller/state
export_csv_on_stop: true
detector_parameters:
  T_hold_gripper: "computed: 0.35 + 0.5 * closing_speed"
  stall_velocity_threshold: 0.008
  width_gap_threshold: 0.002
```

### CSV Export

When `export_csv_on_stop` is `true` (default), the logger automatically reads back the rosbag after recording stops and writes a flattened CSV file using the C++ `rosbag::View` API. The export runs in a background thread so the stop operation returns immediately.

The CSV contains one row per `KittingState` message (67 columns):

| Group            | Columns                                                                                     |
| ---------------- | ------------------------------------------------------------------------------------------- |
| Time             | `timestamp_sec`, `timestamp_nsec`, `time_float`                                             |
| State            | `state_label` (most recent state from `/kitting_controller/state` topic)                    |
| Joint positions  | `q_1` ... `q_7`                                                                             |
| Joint velocities | `dq_1` ... `dq_7`                                                                           |
| Joint torques    | `tau_J_1` ... `tau_J_7`                                                                     |
| External torques | `tau_ext_1` ... `tau_ext_7`, `tau_ext_norm`                                                 |
| Wrench           | `wrench_fx`, `wrench_fy`, `wrench_fz`, `wrench_tx`, `wrench_ty`, `wrench_tz`, `wrench_norm` |
| EE velocity      | `ee_vx`, `ee_vy`, `ee_vz`, `ee_wx`, `ee_wy`, `ee_wz`                                        |
| Gravity          | `gravity_1` ... `gravity_7`                                                                 |
| Coriolis         | `coriolis_1` ... `coriolis_7`                                                               |
| Gripper          | `gripper_width`, `gripper_width_dot`, `gripper_width_cmd`, `gripper_max_width`, `gripper_is_grasped` |
| Force ramp       | `grasp_force`, `grasp_iteration`                                                            |

`O_T_EE` (16 values) and `jacobian` (42 values) are not included in the CSV. They remain in the rosbag.

State labels come from the `/kitting_controller/state` topic. During CSV export, the logger tracks the most recent state label and attaches it to each `KittingState` row, providing per-sample state identification.

## KittingGripperCommand Message

Per-object command published on `/kitting_controller/state_cmd`. Any float64 parameter left at `0.0` falls back to the YAML config default. This lets you override only the parameters that differ for a particular object.

| Field                  | Type    | Description                                                                        |
| ---------------------- | ------- | ---------------------------------------------------------------------------------- |
| `command`              | string  | `"BASELINE"`, `"CLOSING"`, `"GRASPING"`, or `"AUTO"`                               |
| `open_gripper`         | bool    | If true, open gripper before baseline collection (default: false)                  |
| `open_width`           | float64 | Width to open to [m] (0 = max_width from firmware). Only if `open_gripper` is true |
| `closing_width`        | float64 | Target width for MoveAction [m] (0 = use default)                                  |
| `closing_speed`        | float64 | Speed for MoveAction [m/s] (0 = use default, max 0.10)                             |
| `f_min`                | float64 | Starting grasp force [N] (0 = use default 3.0)                                     |
| `f_step`               | float64 | Force increment per iteration [N] (0 = use default 3.0)                            |
| `f_max`                | float64 | Maximum force — FAILED if exceeded [N] (0 = use default 30.0)                      |
| `fr_uplift_distance`   | float64 | Micro-uplift distance per iteration [m] (0 = use default 0.005, max 0.01)          |
| `fr_lift_speed`        | float64 | Lift speed for UPLIFT and DOWNLIFT [m/s] (0 = use default 0.01)                    |
| `fr_uplift_hold`       | float64 | Hold time at top for evaluation [s] (0 = use default 0.10)                         |
| `fr_grasp_speed`       | float64 | Gripper speed for ramp GraspAction [m/s] (0 = use default 0.02)                    |
| `fr_epsilon`           | float64 | Epsilon for ramp GraspAction, inner and outer [m] (0 = use default 0.008)          |
| `fr_stabilization`     | float64 | Post-grasp and post-downlift settle time [s] (0 = use default 0.5)                 |
| `fr_slip_tau_drop`     | float64 | Drop ratio threshold (mu_early−mu_late)/mu_early (0 = use default 0.20)            |
| `fr_slip_width_change` | float64 | Width change threshold for slip [m] (0 = use default 0.001)                        |
| `auto_delay`           | float64 | Delay between auto transitions [s] (0 = default 5.0). Only used by `AUTO` command  |

Only the parameters relevant to the command are used:

- `BASELINE` uses `open_gripper` and `open_width`
- `CLOSING` uses `closing_width` and `closing_speed`
- `GRASPING` uses all `f_*`/`fr_*` force ramp parameters (grasp width is always from `contact_width`)
- `AUTO` uses all of the above, plus `auto_delay`

## KittingState Message

| Field          | Type        | Description                                       |
| -------------- | ----------- | ------------------------------------------------- |
| `header`       | Header      | Timestamp and frame info                          |
| `q`            | float64[7]  | Joint positions [rad]                             |
| `dq`           | float64[7]  | Joint velocities [rad/s]                          |
| `tau_J`        | float64[7]  | Measured joint torques [Nm]                       |
| `tau_ext`      | float64[7]  | Estimated external joint torques [Nm]             |
| `wrench_ext`   | float64[6]  | External wrench in base frame (Fx,Fy,Fz,Tx,Ty,Tz) |
| `O_T_EE`       | float64[16] | EE pose in base frame (4x4 column-major)          |
| `jacobian`     | float64[42] | Zero Jacobian at EE (6x7 column-major)            |
| `gravity`      | float64[7]  | Gravity torque vector [Nm]                        |
| `coriolis`     | float64[7]  | Coriolis torque vector [Nm]                       |
| `ee_velocity`  | float64[6]  | End-effector velocity (J \* dq) [m/s, rad/s]      |
| `tau_ext_norm` | float64     | Euclidean norm of `tau_ext`                       |
| `wrench_norm`  | float64     | Euclidean norm of `wrench_ext`                    |
| `gripper_width` | float64    | Measured finger width [m]                         |
| `gripper_width_dot` | float64 | Finger width velocity (finite difference) [m/s]  |
| `gripper_width_cmd` | float64 | Commanded closing width [m] (0 if not CLOSING)   |
| `gripper_max_width` | float64 | Maximum gripper opening width [m]                |
| `gripper_is_grasped` | bool   | Firmware-level grasp detection flag               |
| `grasp_force`  | float64     | Current grasp force [N] (0 if not in force ramp)  |
| `grasp_iteration` | int32    | Force ramp iteration (0-based; 0 = first attempt) |

## Interfaces

The controller claims three hardware interfaces:

- `FrankaStateInterface` -- provides access to `franka::RobotState`
- `FrankaModelInterface` -- provides access to dynamics/kinematics (Jacobian, gravity, Coriolis)
- `FrankaPoseCartesianInterface` -- provides exclusive Cartesian pose command authority for UPLIFT/DOWNLIFT trajectories

When not executing a trajectory, the controller operates in **passthrough mode**: it reads the robot's own desired pose (`O_T_EE_d`) and writes it back as the command every tick. This results in zero tracking error and the robot holds position. During UPLIFT/DOWNLIFT, the controller commands a cosine-smoothed trajectory. Gripper operations use the libfranka `franka::Gripper` API directly via two dedicated threads:

- **Read thread**: Continuously calls `readOnce()` at firmware rate, computes width velocity via finite difference, writes `GripperData` to `RealtimeBuffer`, executes `stop()` when the RT loop sets the `stop_requested_` atomic flag, and dispatches deferred grasp commands from the RT force ramp via atomic flag polling
- **Command thread**: Waits on a condition variable for queued `move()`/`grasp()`/`homing()` commands from the subscriber callback or action server threads, then executes them (blocking calls, interruptible by `stop()`)

The controller also exposes four `franka_gripper`-compatible action servers (`/franka_gripper/move`, `/franka_gripper/grasp`, `/franka_gripper/homing`, `/franka_gripper/stop`) for external gripper control. Move, grasp, and homing actions are routed through the command thread; stop calls `gripper_->stop()` directly. Actions are guarded by the state machine — only allowed when idle (START, SUCCESS, FAILED). See [Gripper Action Servers](#gripper-action-servers).

The `franka_gripper` package is used only for action type definitions — `franka_gripper_node` is **not** launched. The robot must be launched with `load_gripper:=false` to prevent conflicting gripper connections (only one `franka::Gripper` connection per robot is allowed).

## Real-Time Safety

- Uses `realtime_tools::RealtimePublisher` with non-blocking `trylock()`
- No dynamic memory allocation in `update()`
- No blocking operations in `update()`
- Model queries are only called at the publish rate, not every control tick
- Contact detection uses only scalar arithmetic (no Eigen, no allocation)
- Gripper data passed to RT loop via lock-free `RealtimeBuffer` (no mutex in `update()`)
- Gripper stop on contact uses single atomic store (`stop_requested_`), nanoseconds, RT-safe
- Read thread checks `stop_requested_` flag after each `readOnce()`, calls `stop()` (non-RT), and sets `gripper_stopped_` to confirm
- CONTACT state transition deferred until `gripper_stopped_` is true — ensures stopped gripper before recording
- Command thread executes blocking `move()`/`grasp()` (non-RT), interruptible by `stop()`
- No blocking gripper calls in `update()` — all gripper I/O runs in dedicated threads
- State publisher uses `trylock()` pattern (publish only on transition, non-blocking)
- Cartesian pose command issued every tick (1 kHz) — passthrough when idle, trajectory during UPLIFT/DOWNLIFT
- UPLIFT/DOWNLIFT trajectories use only `std::array<double, 16>`, `std::cos()`, and `std::min()` — no allocation
- Force ramp state machine (`runInternalTransitions`) runs at 250 Hz with only atomic stores, timestamp arithmetic, and trylock publish — fully RT-safe
- Deferred grasp mechanism: RT thread stores params + release-stores flag; read thread acquire-loads and dispatches (no blocking calls in RT)
- `starting()` captures initial desired pose to avoid step discontinuity on controller start
- Action server execute callbacks run in `SimpleActionServer` threads (non-RT); block on `std::future` for command completion
- Action server state guard (`isActionAllowed()`) rejects requests during active grasp sequences — no interference with RT state machine
- Rosbag management runs in a separate C++ node (no Python overhead)

## Recording Performance

The Grasp logger is written in C++ using the `rosbag::Bag` API directly and `topic_tools::ShapeShifter` for generic topic subscription. This eliminates:

- **No subprocess spawn** — bag is opened via API, not `rosbag record` subprocess
- **No Python GIL** — pure C++ callbacks, no interpreter overhead
- **No subscription handshake delay** — topics are already subscribed and recording starts automatically on launch; the bag file is opened immediately
- **Multi-threaded** — uses `ros::AsyncSpinner(4)` so data callbacks and command callbacks run concurrently
- **Single mutex** — one lock protects all trial state, minimal contention

## Grasp Acceptance Checks

- **Separate robot launch** — the controller does not launch the robot; `franka_control.launch` must be running first (with `load_gripper:=false`)
- **Single controller launch file** — `record:=true` enables recording, `record:=false` (default) runs without logger
- **Logger gate** (only when `record:=true`) — controller rejects commands until logger is ready
- **Four user commands** go through `/kitting_controller/state_cmd` — BASELINE, CLOSING, GRASPING, AUTO
- **BASELINE** is published once from START to begin the grasp sequence
- **BASELINE with `open_gripper: true`** waits 3 seconds for gripper to open before collecting
- **BASELINE** can be published from any state to reset the state machine for a new trial
- Recording starts automatically when the logger launches — no START command needed
- Recording continues through all state transitions until STOP, ABORT, terminal state (SUCCESS/FAILED), or node shutdown
- Recording auto-stops on SUCCESS or FAILED (terminal state message is written to bag before closing)
- Recording and state labeling are independent
- State labels on `/kitting_controller/state` are published by the controller for offline segmentation (11 states)
- State labels merged into CSV from `/kitting_controller/state` topic for per-sample state identification
- One rosbag per trial containing all signals and all state transitions
- STOP saves bag + metadata + CSV, ABORT deletes trial (no CSV)
- Logger shutdown (Ctrl+C) automatically stops recording (equivalent to STOP)
- Bag contains 2 topics: kitting_state_data, state
- CSV contains 67 flattened columns with state labels per row
- CSV export does not block the ROS spin loop (runs in background thread)
- metadata.yaml contains bag_filename, csv_filename, total_samples, start/stop times, and detector parameters
- Gripper stall contact detection: velocity < threshold AND width gap > threshold for T_hold_gripper (computed: 0.35 + 0.5 \* closing_speed)
- Free closing (no object): width reaches w_cmd, gap ~0 → no false CONTACT → transitions to FAILED ("no contact detected")
- Object contact: width stalls before w_cmd, w_dot ~0, gap > threshold → CONTACT
- Gripper stop requested immediately on contact: `stop()` called via atomic flag and read thread
- CONTACT state transition deferred until gripper has physically stopped (`gripper_stopped_` flag)
- Contact width saved at stall detection — used as the grasp width in GRASPING
- KittingState contains robot signals, gripper signals, and force ramp state — all published at 250 Hz
- Publishing CLOSING on `state_cmd` triggers gripper `move()` + state label
- Publishing BASELINE on `state_cmd` prepares for new trial (optionally opens gripper)
- Publishing GRASPING on `state_cmd` starts the automated force ramp (uses contact_width from CONTACT)
- GRASPING timeout: 10 seconds maximum for gripper command completion, then FAILED
- Force ramp runs autonomously after GRASPING: GRASPING → UPLIFT → EVALUATE → SUCCESS
- Slip detected during EVALUATE: DOWNLIFT → SETTLING → GRASPING (retry with F += f_step)
- Force ramp terminates with FAILED if f_current exceeds f_max
- UPLIFT and DOWNLIFT trajectories use cosine smoothing with duration computed from distance/speed
- UPLIFT/DOWNLIFT orientation remains unchanged throughout the motion
- Uplift distance is clamped to 10 mm maximum (with warning)
- BASELINE during active force ramp clears trajectories and returns to passthrough
- Per-command force ramp parameters override YAML defaults when non-zero
- Parameters left at 0.0 fall back to YAML config values
- Duplicate CLOSING commands are ignored
- Controller holds position (passthrough) when not executing trajectory — no drift or jerk
- Gripper connection failure at init returns false (controller not loaded)
- Controller destructor shuts down action servers then joins gripper threads cleanly on unload
- Gripper action servers (move, grasp, homing, stop) advertised under `/franka_gripper/` namespace
- Action servers guarded by state machine — only allowed in START, SUCCESS, FAILED
- Action server requests rejected with descriptive error during active grasp sequence (BASELINE through EVALUATE)
- Stop action always allowed regardless of state (thread-safe `gripper_->stop()`)
- Move, grasp, homing actions routed through command thread to prevent concurrent libfranka calls
- Action server shutdown before gripper thread shutdown prevents hanging futures on controller unload
- AUTO command runs full grasp sequence: BASELINE → *(delay)* → CLOSING → *(wait for CONTACT)* → *(delay)* → GRASPING
- AUTO mode uses configurable `auto_delay` (default 5.0 seconds) between BASELINE→CLOSING and CONTACT→GRASPING transitions
- AUTO mode forwards all BASELINE, CLOSING, and GRASPING parameters from the single message to each stage
- AUTO mode cancelled by any manual command (BASELINE, CLOSING, GRASPING) — allows user override at any point
- AUTO mode ends automatically on FAILED during CLOSING (no contact detected) or after GRASPING starts (force ramp is autonomous)
- Auto mode timers run on ROS spinner thread (non-RT); reuse existing handle functions — no duplicated logic

## License

Apache 2.0
