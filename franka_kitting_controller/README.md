# franka_kitting_controller

Real-time state acquisition and automated force ramp controller for the Franka Panda with SMS-CUSUM contact detection and rosbag data collection. Reads all robot state, model, and Cartesian signals, publishes them as a single `KittingState` message, autonomously detects contact using SMS-CUSUM (noise-adaptive CUSUM change-point detection on tau_ext_norm), immediately stops the gripper on contact via `franka::Gripper::stop()`, and runs a multi-step force ramp on the table (GRASP_1 → GRASP_2 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED) internally. Claims `FrankaPoseCartesianInterface` for Cartesian pose control. Gripper operations use the libfranka `franka::Gripper` API directly. Also exposes `franka_gripper`-compatible action servers (move, grasp, homing, stop) for external gripper control when the internal state machine is idle.

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity, support force (Fn = |Fz|), tangential force (Ft = √(Fx²+Fy²))

**Grasp** adds:

- 11-state grasp machine with multi-step force ramp: `START` → `BASELINE` → `CLOSING_COMMAND` → `CLOSING` → `CONTACT_CONFIRMED` → `CONTACT` → `GRASPING` (GRASP_1..GRASP_N) → `UPLIFT` → `EVALUATE` → `SUCCESS`/`FAILED`
- SMS-CUSUM contact detection during CLOSING (noise-adaptive CUSUM change-point detection on tau_ext_norm — automatically scales sensitivity to measured noise level, no per-object threshold tuning)
- Immediate gripper stop on contact: calls `franka::Gripper::stop()` to physically halt the motor
- Multi-step force ramp on table: grasps at f_min, f_min+f_step, ..., up to f_max — each step settles, holds, then advances. After the final step, a single UPLIFT + EVALUATE determines SUCCESS or FAILED
- Three-gate AND slip detection: load transfer + support drop + jaw widening (directional force decomposition, Fn = |Fz|) during EVALUATE hold
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

## Testing

Unit and state-machine tests use GTest (no ROS master or robot hardware required):

```bash
# Build and run tests
catkin build franka_kitting_controller --catkin-make-args run_tests
# or
catkin_make run_tests_franka_kitting_controller

# View results
catkin_test_results build/franka_kitting_controller
```

**Test tiers:**

| Tier | File | What's tested |
|------|------|---------------|
| 1 — Pure unit | `test/kitting_unit_test.cpp` | `arrayNorm`, `resolveParam`, `stateToString`, `isClosingPhase`, `isForceRampPhase` |
| 2 — State machine | `test/kitting_state_machine_test.cpp` | All tick functions, 3-gate slip detection (each gate individually), trajectory math (uplift/downlift cosine smoothing), multi-step force ramp, constants consistency |

Tests bypass `init()` (which requires a real `franka::Gripper`) using a friend-class test fixture with `MockModel` and real `franka_hw` handles backed by test data.

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
Manual mode (3 user commands):

  BASELINE ──> CLOSING_COMMAND ──> CLOSING ──> CONTACT_CONFIRMED ──> CONTACT ──> GRASPING
  (user cmd)   (user cmd)          (auto)      (auto)                (auto)      (user cmd)

Auto mode (1 command):

  BASELINE ─(prep+delay)─> CLOSING_COMMAND ──> CLOSING ──> CONTACT_CONFIRMED ──> CONTACT ─(delay)─> GRASPING

Force ramp (runs autonomously after GRASPING, both modes):

  CONTACT ──> GRASPING ──────────────────────────────────────> UPLIFT ──> EVALUATE ──> SUCCESS
              ┌──────────────────────────────────┐                            │
              │ GRASP_1 → GRASP_2 → ... GRASP_N │                       [slip / no
              │ Each step: grasp → settle → hold │                       load transfer]
              │ contact_width updated each step  │                            │
              └──────────────────────────────────┘                            v
                                                                           FAILED
```

CLOSING_COMMAND transitions to CLOSING when the gripper move is confirmed executing. CONTACT_CONFIRMED is published immediately on contact detection; CONTACT follows when the gripper has physically stopped. SUCCESS and FAILED are terminal states. The force ramp increments grasp force from `f_min` to `f_max` in `f_step` increments entirely on the table (no mid-trial lift/lower), with a single UPLIFT + EVALUATE after the final ramp step.

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

- **Sequential preparation**: If the arm is elevated from a previous uplift, BASELINE first lowers the arm (cosine-smoothed downlift at `lift_speed`), then opens the gripper, then collects baseline data — strictly sequential. A single `baseline_prep_done` flag gates all baseline collection: it starts `false` when BASELINE is entered and becomes `true` only after lowering completes AND gripper open completes (if applicable). No baseline samples are collected until both steps finish
- **Gripper open**: Opens automatically when `open_gripper: true` OR when `record:=true` (ensures clean baseline). Opens to `open_width` (or `max_width` from firmware if not specified) at 0.1 m/s. The open is deferred until any downlift correction completes
- Resets all state machine variables (contact latch, force ramp, trajectories) for a fresh grasp cycle
- BASELINE is a one-shot state — publish once from START to begin the grasp sequence

### CLOSING_COMMAND

Gripper close queued — awaiting execution confirmation. Published via `/kitting_controller/state_cmd` with `command: "CLOSING"`.

- Published immediately when the CLOSING command is received, before the gripper starts moving
- Gripper move command is queued to the command thread but may not yet be executing
- No contact detection runs in this state — data recorded here reflects true pre-closing conditions
- Transitions to **CLOSING** automatically when `cmd_executing_` confirms the gripper move has started

### CLOSING

Observe approach dynamics before contact. Published **automatically** by the controller when gripper movement is confirmed.

- Gripper is confirmed moving toward object at width `w` and speed `v` (max 0.10 m/s)
- SMS-CUSUM contact detection runs during this state (see [SMS-CUSUM Contact Detection](#sms-cusum-contact-detection))
- On contact detection, transitions immediately to **CONTACT_CONFIRMED**
- If gripper reaches target width without detecting contact (no object present), transitions to **FAILED** — "no contact detected"
- If CLOSING phase exceeds 30 seconds without contact, transitions to **FAILED** — "CLOSING timeout" (safety net for stuck gripper)

### CONTACT_CONFIRMED

Contact detected — gripper decelerating. Published **automatically** by the controller on contact detection.

- SMS-CUSUM detects a sustained torque drop (CUSUM statistic S exceeds threshold h for debounce_count consecutive samples) (see [SMS-CUSUM Contact Detection](#sms-cusum-contact-detection))
- `contact_width` is saved when the gripper has physically stopped (CONTACT) — used as the grasp width in GRASPING
- `franka::Gripper::stop()` is requested via the read thread to halt the motor
- Data recorded during this state captures the gripper deceleration dynamics
- Transitions to **CONTACT** when the gripper has physically stopped (`gripper_stopped_` flag)

### CONTACT

Gripper stopped — contact confirmed. Published **automatically** by the controller.

- Gripper motor has fully stopped after contact detection and stop request
- All data recorded during CONTACT reflects a stopped gripper
- CONTACT is **latched** once detected — cannot return to CLOSING

### GRASPING

Initiate automated multi-step force ramp. Published via `/kitting_controller/state_cmd` with `command: "GRASPING"`. This is the **last user command** — all subsequent states are driven internally by the force ramp. **Requires CONTACT state** — rejected otherwise to prevent use of stale width from a previous trial.

The force ramp runs entirely on the table (no mid-trial lift/lower). The controller grasps at increasing force levels from `f_min` to `f_max` in `f_step` increments. Each step is published as a distinct state label: `GRASP_1`, `GRASP_2`, ..., `GRASP_N`.

**Each ramp step (GRASP_k):**
1. **Grasp command**: Dispatches a deferred grasp at force `f_current` to the `contact_width` (updated after each step)
2. **Settle** (`grasp_settle_time`, default 0.5 s): Wait for the grasp command to complete and signals to stabilize. Published `grasp_ramp_phase = "settling"`
3. **Hold** (`grasp_force_hold_time`, default 1.0 s): Hold at force level for data logging and analysis. Published `grasp_ramp_phase = "holding"`. W_pre (support force baseline for slip evaluation) is accumulated during the HOLDING sub-phase of the **last** ramp step only
4. **Advance**: Update `contact_width` from the current gripper width, increment force by `f_step`, advance to next step (or UPLIFT if this was the final step)

After the final ramp step (where `f_current + f_step > f_max`), the controller transitions to UPLIFT.

- Grasp width starts as the `contact_width` captured at CONTACT; updated after each ramp step
- **Grasp failure**: After each ramp step's grasp command completes, the RT thread checks `cmd_success_` (the `grasp()` return value stored by the command thread). If `false`, transitions to FAILED immediately
- **Timeout**: If any grasp command does not complete within 10 seconds, the controller transitions to FAILED
- Uses the deferred grasp mechanism: RT thread stores parameters via atomic flag, read thread calls `stop()` to release any existing grip before dispatching the new grasp. Command thread stores `grasp()` result in `cmd_success_` (release) for RT thread to check (acquire)

### UPLIFT

Validate grasp robustness under load. **Auto-triggered** by the controller after the final ramp step completes — not a user command.

- Controller displaces end-effector upward by `fr_uplift_distance` (default 10 mm, max 300 mm)
- Duration computed from distance and speed: `T = fr_uplift_distance / fr_lift_speed`
- Cosine-smoothed trajectory `s = 0.5(1 - cos(π · s_raw))` ensures zero velocity at start and end
- Only z-translation of `O_T_EE_d[14]` is modified — orientation and x/y unchanged
- Reference signals (`tau_ext_norm`, gripper width) are recorded before lift for slip comparison
- On trajectory completion, automatically transitions to EVALUATE

### EVALUATE

Assess grasp stability at the lifted position. **Auto-triggered** after UPLIFT completes.

- Holds position for `uplift_hold` seconds (default 1.0 s): early window (first half) + late window (second half)
- Uses directional force decomposition for slip detection
- Three-gate AND evaluation (see [Slip Detection](#grasp-slip-detection-directional-force-decomposition--and-gating)):
  - **Gate 1**: Load transfer confirmation — `deltaF > max(3*sigma_pre, load_transfer_min)`
  - **Gate 2**: Support drop check — `dF <= slip_drop_thresh`
  - **Gate 3**: Jaw widening check — `P95-P5 <= slip_width_thresh`
- If **secure** (all gates pass): transitions to SUCCESS
- If **slip** (any gate fails): transitions to FAILED

### SUCCESS

Terminal state indicating stable grasp confirmed. **Auto-triggered** when EVALUATE finds no slip.

- The grasp held at force `f_current` (final ramp step) without slip
- Robot remains at the lifted position holding the object (elevated by one `uplift_distance` from the single UPLIFT after the ramp) — ready for pick-and-place transport
- The accumulated uplift is tracked internally; the next `BASELINE` command automatically corrects the height before the next trial
- State label published for offline analysis
- **Recording auto-stops**: if `record:=true`, the logger detects SUCCESS and automatically stops recording (saves bag + metadata + CSV)
- To start a new trial, publish `BASELINE` (resets the entire state machine and corrects arm height)

### FAILED

Terminal state indicating grasp failure. **Auto-triggered** on any of:

- No contact detected during CLOSING (gripper reached target width without detecting contact)
- CLOSING timeout (CLOSING phase exceeded 30 seconds without contact — gripper stuck or action never completed)
- CLOSING_COMMAND timeout (move command did not start executing within 10 seconds)
- GRASPING timeout (gripper command did not complete within 10 seconds)
- Grasp failed at a ramp step (`grasp()` returned false — hardware rejected the grasp command)
- Slip detected during EVALUATE (any of the three gates failed after the full ramp + UPLIFT)
- State label published for offline analysis with diagnostic information
- **Recording auto-stops**: if `record:=true`, the logger detects FAILED and automatically stops recording (saves bag + metadata + CSV)
- To retry, publish `BASELINE` (resets the entire state machine)

### AUTO (Automatic Full Sequence)

A single command that runs the full grasp sequence automatically. Published via `/kitting_controller/state_cmd` with `command: "AUTO"`.

- Executes BASELINE → *(wait for prep + baseline ready)* → *(delay)* → CLOSING_COMMAND → CLOSING → *(wait for CONTACT_CONFIRMED → CONTACT)* → *(delay)* → GRASPING automatically
- **Baseline-ready polling**: AUTO polls every 100 ms for `baseline_prep_done` AND `cd_baseline_ready` before starting the `auto_delay` countdown. This ensures the arm is lowered, gripper is opened (if applicable), and baseline data is fully collected before advancing to CLOSING — even when prep takes variable time
- `auto_delay` parameter controls the wait time between transitions (default 5.0 seconds), applied *after* baseline is ready
- All BASELINE, CLOSING, and GRASPING parameters are accepted and forwarded to each stage
- The force ramp runs autonomously after GRASPING (same as manual mode): GRASP_1 → GRASP_2 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED
- If BASELINE prep results in FAILED, auto mode ends at FAILED
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
- `/kitting_controller/state` — The **controller** publishes state labels: START, BASELINE, CLOSING_COMMAND, CLOSING, CONTACT_CONFIRMED, CONTACT, GRASP_1..GRASP_N, UPLIFT, EVALUATE, SUCCESS, FAILED. During the force ramp, each step is published as GRASP_1, GRASP_2, etc. (not "GRASPING"). States are labels for offline analysis. Terminal states (SUCCESS, FAILED) also trigger automatic recording stop when the logger is running.
- `/kitting_controller/record_control` — The **user** publishes STOP or ABORT to end recording. Recording starts automatically when the logger launches — no START command is needed. The **logger** subscribes to this topic.
- `/kitting_controller/logger_ready` — The **logger** publishes `true` (latched) on startup. When `record:=true`, the **controller** subscribes and gates Grasp commands behind this signal. When `record:=false`, this topic is not used.

### Recording

One rosbag per trial. **Recording starts automatically** when the logger node is launched — no START command is needed. The logger opens a new trial bag immediately on startup and records all configured topics. **Multi-trial support**: after a trial ends (SUCCESS/FAILED), the logger automatically starts a new trial when the next BASELINE state is detected — no need to relaunch the logger between trials.

| Command | Action                                            |
| ------- | ------------------------------------------------- |
| `STOP`  | Close bag, save metadata, export CSV (if enabled) |
| `ABORT` | Close bag, delete trial directory (no CSV)        |

- **STOP** is ignored if not recording.
- **ABORT** is ignored if not recording.
- If the logger node is shut down (Ctrl+C), recording is automatically stopped (equivalent to STOP).
- **Auto-stop on terminal state**: When the logger detects SUCCESS or FAILED on `/kitting_controller/state`, recording stops automatically (equivalent to STOP). The terminal state message is written to the bag before it is closed.
- **Auto-start on BASELINE**: When not recording (after terminal state or STOP), the logger automatically starts a new trial when it detects BASELINE on `/kitting_controller/state`. The BASELINE message is written as the first message in the new bag. This enables back-to-back trials without relaunching.

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
  "{command: 'CLOSING', closing_width: 0.001, closing_speed: 0.05}" --once

# ... CONTACT is published by the controller automatically ...

# GRASPING — use YAML defaults (f_min=3N, f_max=70N, f_step=3N)
# Uses contact_width from CONTACT as grasp width. After this command, the force ramp
# runs autonomously: GRASP_1 → GRASP_2 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING'}" --once

# GRASPING — override force range for a specific object
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', f_min: 3.0, f_max: 70.0, f_step: 3.0}" --once

# GRASPING — override all force ramp parameters for a specific object
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', \
    f_min: 3.0, f_step: 3.0, f_max: 70.0, \
    grasp_force_hold_time: 1.0, grasp_settle_time: 0.5, \
    fr_grasp_speed: 0.02, fr_epsilon: 0.008, \
    fr_uplift_distance: 0.010, fr_lift_speed: 0.01, fr_uplift_hold: 1.0, \
    fr_slip_drop_thresh: 0.15, fr_slip_width_thresh: 0.0005, \
    fr_load_transfer_min: 1.5}" --once

# Stop recording
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'STOP'" --once

# Or abort (delete trial)
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'ABORT'" --once
```

### Auto Mode

A single `AUTO` command runs the full sequence: BASELINE → *(wait for prep + baseline ready)* → *(delay)* → CLOSING_COMMAND → CLOSING → *(wait for CONTACT_CONFIRMED → CONTACT)* → *(delay)* → GRASPING. AUTO polls for baseline readiness before starting the `auto_delay` countdown, so the delay is always applied *after* arm lowering, gripper open, and baseline data collection complete. All parameters from BASELINE, CLOSING, and GRASPING are accepted and forwarded to each stage.

```bash
# AUTO — run full sequence automatically (default 5s delay between transitions)
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'AUTO', open_gripper: true}" --once

# AUTO — with custom delay and all parameters forwarded to each stage
# All BASELINE, CLOSING, and GRASPING parameters are accepted in a single AUTO
# command and forwarded to each stage when it runs. 0 = use YAML default.
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'AUTO', open_gripper: true, auto_delay: 3.0, \
    closing_width: 0.001, closing_speed: 0.05, \
    f_min: 3.0, f_step: 3.0, f_max: 70.0, \
    grasp_force_hold_time: 1.0, grasp_settle_time: 0.5, \
    fr_grasp_speed: 0.02, fr_epsilon: 0.008, \
    fr_uplift_distance: 0.010, fr_lift_speed: 0.01, fr_uplift_hold: 1.0, \
    fr_slip_drop_thresh: 0.15, fr_slip_width_thresh: 0.0005, \
    fr_load_transfer_min: 1.5}" --once
```

## Gripper Action Servers

The controller exposes four `franka_gripper`-compatible action servers under the `/franka_gripper/` namespace. These allow external nodes (e.g., a MoveIt-based main controller) to command the gripper through standard ROS action interfaces — without running `franka_gripper_node` (which would conflict with the controller's own gripper connection).

### State Guard

Action server requests are only accepted when the internal state machine is **idle**: `START`, `SUCCESS`, or `FAILED`. During any other state (BASELINE through EVALUATE), the state machine owns the gripper and external actions are rejected with an error describing the current state.

### Available Actions

| Action                    | Type                       | Description                                              |
| ------------------------- | -------------------------- | -------------------------------------------------------- |
| `/franka_gripper/move`    | `franka_gripper/MoveAction`   | Move fingers to a target width at a given speed          |
| `/franka_gripper/grasp`   | `franka_gripper/GraspAction`  | Grasp with specified width, speed, force, and epsilon    |
| `/franka_gripper/homing`  | `franka_gripper/HomingAction` | Run the gripper homing routine (recalibrate)             |
| `/franka_gripper/stop`    | `franka_gripper/StopAction`   | Immediately stop the gripper (always allowed, any state) |

**Move**, **Grasp**, and **Homing** are routed through the existing gripper command thread to prevent concurrent `libfranka` calls. **Stop** is routed through the read thread's `stop_requested_` flag to avoid concurrent `gripper_->stop()` calls from multiple threads — the read thread serializes all gripper I/O. The action server polls for completion (up to 500 ms timeout).

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

The controller detects contact during CLOSING using SMS-CUSUM (Sequential Multi-State CUSUM), a noise-adaptive change-point detector that monitors tau_ext_norm (the Euclidean norm of estimated external joint torques) for a sustained drop when the gripper contacts an object. Unlike fixed-threshold detection, SMS-CUSUM automatically scales its sensitivity to the measured noise level, eliminating per-object threshold tuning across objects with different contact drop magnitudes (0.13–0.75 Nm).

### SMS-CUSUM Contact Detection

Detects contact by monitoring tau_ext_norm during CLOSING using a one-sided downward CUSUM detector with noise-adaptive allowance. When the gripper contacts an object, the external torque norm drops suddenly as the gripper mechanics absorb the impact. The detection function is `detectContact()`.

#### Algorithm

1. **Baseline collection**: During BASELINE state (after any accumulated-uplift correction completes), feed tau_ext_norm samples to `sms_detector_.update()`. The `AdaptiveBaseline` collects 50 samples (0.2 s at 250 Hz) to compute both the **mean** (μ) and **noise sigma** (σ). After 50 samples, the baseline transitions to EMA tracking. This captures the at-rest external torque norm and its variability before any gripper motion.
2. **Baseline freeze**: At the CLOSING_COMMAND → CLOSING transition, `sms_detector_.enter_closing()` freezes the baseline (preventing corruption during detection) and adapts the CUSUM allowance: `k_eff = max(k_min, noise_multiplier × σ)`.
3. **CUSUM detection**: Each tick during CLOSING, the CUSUM statistic accumulates evidence of a torque drop: `S = max(0, S + (μ − tau_ext_norm) − k_eff)`. The allowance `k_eff` absorbs noise-level fluctuations; only genuine drops accumulate S.
4. **Debounce**: The alarm condition `S ≥ h` must persist for `debounce_count` (5) consecutive samples before triggering contact. This replaces time-based debounce with sample-based debounce.
5. **On trigger**: Sets `contact_latched_`, transitions to `CONTACT_CONFIRMED`, calls `requestGripperStop()`. `contact_width_` is captured later when the gripper has physically stopped (CONTACT).

#### Parameters

| Parameter              | Name                          | Default | Description                                                                      |
| ---------------------- | ----------------------------- | ------- | -------------------------------------------------------------------------------- |
| `k_min`                | Minimum CUSUM allowance       | 0.02    | Floor for k_eff — ensures detection of very small drops even with low noise      |
| `h`                    | CUSUM threshold               | 0.3     | CUSUM statistic S must reach this value to trigger alarm                         |
| `debounce_count`       | Debounce samples              | 5       | Alarm must persist for this many consecutive samples (20 ms at 250 Hz)           |
| `noise_multiplier`     | Noise scaling factor          | 2.0     | k_eff = max(k_min, noise_multiplier × σ) — scales allowance to noise level      |
| `baseline_init_samples`| Baseline collection count     | 50      | Samples for initial mean and sigma estimation (0.2 s at 250 Hz)                  |
| `baseline_alpha`       | EMA smoothing factor          | 0.01    | Exponential moving average coefficient for baseline tracking after initialization|

#### Symbols

| Symbol                 | Name                        | Unit | Description                                                                    |
| ---------------------- | --------------------------- | ---- | ------------------------------------------------------------------------------ |
| `μ` (baseline mean)    | Baseline torque norm         | Nm   | Mean tau_ext_norm from 50 BASELINE samples (frozen during CLOSING)              |
| `σ` (baseline sigma)   | Baseline noise               | Nm   | Standard deviation of tau_ext_norm during BASELINE (measures noise level)       |
| `tau_ext_norm`         | Current external torque norm | Nm   | Euclidean norm of estimated external joint torques                              |
| `k_eff`                | Effective CUSUM allowance    | Nm   | `max(k_min, noise_multiplier × σ)` — absorbs noise, passes genuine drops       |
| `S`                    | CUSUM statistic              | Nm   | `max(0, S + (μ − tau_ext_norm) − k_eff)` — accumulated evidence of drop        |
| `h`                    | CUSUM threshold              | Nm   | Alarm fires when S ≥ h for debounce_count consecutive samples                  |

### Gripper Stop on Contact

When contact is detected, the controller stops the gripper and transitions through two contact states:

1. **Contact detected (RT thread)**: Sets `contact_latched_`, sets `stop_requested_`, and transitions to **CONTACT_CONFIRMED** immediately.
2. **Read thread executes stop**: Checks `stop_requested_` after each `readOnce()`, calls `franka::Gripper::stop()` to physically halt the motor, then sets `width_capture_pending_` to begin a settle-wait. If `stop()` throws an exception, `stop_requested_` remains true for automatic retry on the next iteration.
3. **Read thread signals stop**: On the next `readOnce()` after `stop()`, sets `gripper_stopped_` to signal the RT thread that the gripper has stopped. `contact_width_` is stored for logging/validation, but the actual grasp width is always read fresh from `gs.width` at the moment the grasp command is dispatched — this guarantees the target matches the gripper's real position.
4. **Deferred transition (RT thread)**: Sees `contact_latched_ && CONTACT_CONFIRMED && gripper_stopped_` and transitions to **CONTACT**. `contact_width_` is already set by the read thread.
5. **One-shot guard**: `gripper_stop_sent_` flag prevents duplicate stop requests; `contact_latched_` prevents re-detection.

The atomic stores in `update()` are single-word writes (nanoseconds), fully RT-safe. The actual `stop()` call and width capture execute in the non-RT read thread. The state transition is deferred until the gripper has fully settled (typically 20–100 ms depending on closing speed and deceleration), ensuring `contact_width_` reflects the true resting position before CONTACT data is recorded.

### Latching

Once CONTACT is declared, it is **latched** — the detector stops evaluating. This prevents oscillation at the contact boundary. A new BASELINE command resets the latch for the next trial.

## Grasp: Slip Detection (Directional Force Decomposition + AND-Gating)

After UPLIFT (which occurs once, after the final ramp step), the controller evaluates grasp quality using directional force decomposition and rigid AND-gating across three independent boolean gates. Instead of using `wrench_norm` (the L2 norm of the full 6D wrench), the controller decomposes `O_F_ext_hat_K` into **support force** Fn = |Fz| (along the lift direction). This gives physically meaningful signals for kitting operations where slip is primarily gravity-driven.

### Why Fn Instead of wrench_norm

The controller uses `Fn = |Fz|` — the absolute z-component of the external force estimate `O_F_ext_hat_K` — as the slip detection signal instead of `wrench_norm = ||[Fx,Fy,Fz,Tx,Ty,Tz]||`. The wrench norm combines all 6 force and torque components into a single scalar, which makes it impossible to isolate gravity-driven slip from unrelated disturbances. During a vertical lift, an object slipping through the fingers loses support along the z-axis — this shows up directly as a drop in Fn. Using wrench_norm, that same drop would be diluted by Fx, Fy, and torque components (Tx, Ty, Tz) that carry no useful slip information for parallel grippers. By extracting Fn alone, the controller gets a physically interpretable signal that maps directly to "how much of the object's weight is the gripper still supporting?"

### Time Windows

The slip evaluation uses three time windows across the grasp-lift-hold sequence:

| Window         | Duration                | When                                          | Signal              | Purpose                     |
| -------------- | ----------------------- | --------------------------------------------- | -------------------- | --------------------------- |
| **W_pre**      | `grasp_force_hold_time` | HOLDING sub-phase of the **last** ramp step   | Fn (support force)   | Baseline before lift        |
| **W_hold_early** | `uplift_hold/2`       | First half of EVALUATE                        | Fn (support force)   | Loaded reference after lift |
| **W_hold_late**  | `uplift_hold/2`       | Second half of EVALUATE                       | Fn (support force)   | Late hold for drop check   |

```
  GRASPING (multi-step ramp on table)                UPLIFT     EVALUATE (uplift_hold)
  GRASP_1 → GRASP_2 → ... → GRASP_N                 [lift]     ├── W_hold_early [0, hold/2)
                              │                                 └── W_hold_late  [hold/2, hold)
                              └── W_pre [hold phase]                Fn_early, Fn_late
                                  Fn_pre, sigma_pre
```

### Load Transfer Gate (Mandatory)

The load transfer gate confirms the object is actually supported by the gripper after lifting. Without it, an empty gripper or failed pickup would pass the drop/motion checks (no drop because nothing was there to drop). This acts as a prerequisite — if not passed, the failure is "pickup failure" not "slip".

```
  Fn_pre    = mean(Fn over W_pre)
  sigma_pre = std(Fn over W_pre)
  Fn_early  = mean(Fn over W_hold_early)

  deltaF = Fn_early - Fn_pre

  Load transfer confirmed if:  deltaF > max(3 * sigma_pre, load_transfer_min)
```

The `3 * sigma_pre` threshold adapts to sensor noise. The `load_transfer_min` floor (default 1.5 N, configurable) prevents declaring load transfer from noise alone. Lower this for light objects (e.g., 0.5 N for a ~51 g object).

### Gate 2: Support Drop Check

In kitting operations (horizontal approach, vertical lift), gravity-driven slip manifests as a drop in the support force Fn during the hold. If the object slides down between the fingers, the gripper supports less of its weight, so Fn_late < Fn_early. This is the dominant physics signal for vertical kitting operations.

```
  dF = (Fn_early - Fn_late) / max(Fn_early, 1e-6)    # relative decay [0,1]

  drop_ok = (dF <= slip_drop_thresh)                   # binary pass/fail
```

A `dF` exceeding `slip_drop_thresh` (default 15%) means the support force dropped too much during the hold — the object is slipping.

### Gate 3: Jaw Widening

Gripper width increase during hold is a direct mechanical indicator of slip — if the object moves, it pushes the fingers apart. This is a secondary confirmation because: (a) it is lower bandwidth than force (gripper width updates slower than the 1 kHz force signal), (b) small slips may not produce measurable width change, but (c) when present it is a strong confirmation signal. Together with the support drop gate, it eliminates false positives where force changes come from arm dynamics rather than actual slip.

```
  dw = P95(width) - P5(width)      # percentile spread over entire hold

  width_ok = (dw <= slip_width_thresh)                 # binary pass/fail
```

### AND-Gating Verdict

All three gates must pass for the grasp to be declared secure. If any gate fails, the grasp is declared slipping and the controller transitions to FAILED.

```
  secure = load_transferred AND drop_ok AND width_ok

  is_slipping = !secure
```

This rigid AND-gating approach ensures that: (1) the object was actually picked up (Gate 1), (2) the support force has not dropped (Gate 2), and (3) the jaw width is stable (Gate 3). Each gate addresses a distinct failure mode, and all three must be satisfied simultaneously.

### Slip Detection Parameters

| Parameter               | Default  | Description                                                        |
| ----------------------- | -------- | ------------------------------------------------------------------ |
| `slip_drop_thresh`      | 0.15     | DF_TH: maximum allowed relative support force drop (15% = fail)    |
| `slip_width_thresh`     | 0.0005 m | W_TH: maximum allowed jaw widening P95-P5 [m] (0.5 mm = fail)     |
| `load_transfer_min`     | 1.5 N    | Floor for load transfer threshold (lower for light objects)        |

### Example Log Output

Secure grasp (all three gates pass):

```
  [SLIP] Gate 1 — Load Transfer:  deltaF=2.624 N  threshold=2.000 N  (Fn_pre=3.210  sigma=0.142  Fn_early=5.834)  PASS
  [SLIP] Gate 2 — Support Drop:    dF=2.1%  threshold=15%  PASS
  [SLIP] Gate 3 — Jaw Widening:   P95-P5=0.00020 m  threshold=0.0005 m  (P5=0.03410  P95=0.03430)  PASS
  [SLIP] Verdict: SECURE
```

- Gate 1: `deltaF=2.624` > `max(3*0.142, 2.0)` = 2.0 → PASS
- Gate 2: `dF=2.1%` ≤ 15% → PASS
- Gate 3: `P95-P5=0.20 mm` ≤ 0.50 mm → PASS
- **Verdict: SECURE → SUCCESS**

Slip detected — load transferred but object slipped during hold:

```
  [SLIP] Gate 1 — Load Transfer:  deltaF=2.740 N  threshold=2.000 N  (Fn_pre=3.180  sigma=0.138  Fn_early=5.920)  PASS
  [SLIP] Gate 2 — Support Drop:    dF=23.8%  threshold=15%  FAIL
  [SLIP] Gate 3 — Jaw Widening:   P95-P5=0.00140 m  threshold=0.0005 m  (P5=0.03400  P95=0.03540)  FAIL
  [SLIP] Verdict: SLIPPING
```

- Gate 1: PASS (load transferred)
- Gate 2: FAIL (23.8% decay >> 15% threshold)
- Gate 3: FAIL (P95-P5 = 1.4 mm >> 0.5 mm threshold)
- **Verdict: SLIPPING → FAILED**

Failed load transfer (object not lifted):

```
  [SLIP] Gate 1 — Load Transfer:  deltaF=0.142 N  threshold=2.000 N  (Fn_pre=3.210  sigma=0.142  Fn_early=3.352)  FAIL
  [SLIP] Gate 2 — Support Drop:    dF=1.6%  threshold=15%  PASS
  [SLIP] Gate 3 — Jaw Widening:   P95-P5=0.00010 m  threshold=0.0005 m  (P5=0.03415  P95=0.03425)  PASS
  [SLIP] Verdict: SLIPPING
```

- Gate 1: FAIL (`deltaF=0.142` < 2.0 — object weight never appeared on gripper)
- Gates 2 & 3 pass, but Gate 1 failure alone triggers SLIPPING
- **Verdict: SLIPPING → FAILED**

### How to Read the Log

| Field | What it tells you |
| ----- | ----------------- |
| `Gate 1 — Load Transfer` | PASS/FAIL — did the object weight actually transfer to the gripper? deltaF must exceed `max(3*sigma, load_transfer_min)` |
| `Gate 2 — Support Drop` | PASS/FAIL — has support force dropped during hold? dF% must be ≤ slip_drop_thresh |
| `Gate 3 — Jaw Widening` | PASS/FAIL — are the jaws stable during hold? P95-P5 must be ≤ slip_width_thresh |
| `Verdict` | SECURE (all gates pass) or SLIPPING (any gate fails) |

**Quick diagnostic guide:**
- `Gate 1: FAIL` → object not gripped or too light; increase force or lower `load_transfer_min`
- `Gate 2: FAIL` → support force dropped during hold (object sliding down); increase force
- `Gate 3: FAIL` → gripper opening during hold (fingers pushed apart); increase force

## Grasp: UPLIFT / DOWNLIFT Trajectory Mathematics

The UPLIFT trajectory executes a smooth Cartesian micro-lift internally using cosine-smoothed time-based trajectory interpolation. DOWNLIFT uses the same math but is used **only** during BASELINE preparation to reverse accumulated uplift from a previous SUCCESS (between-trial arm lowering) — it is **not** used during the force ramp. Both trajectories run at the control loop rate (1 kHz) and command the end-effector pose via `FrankaPoseCartesianInterface`.

### Symbols

| Symbol     | Name                      | Unit | Default  | Description                                                                |
| ---------- | ------------------------- | ---- | -------- | -------------------------------------------------------------------------- |
| `d`        | Lift distance             | m    | 0.010    | Total displacement along the z-axis (max 0.3)                              |
| `v_lift`   | Lift speed                | m/s  | 0.01     | Speed for UPLIFT and BASELINE prep downlift (min 0.001)                    |
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
| Peak velocity (default) | `v_max = π·0.010/(2·1.0) ≈ 0.0157 m/s` | ~15.7 mm/s — well within Franka limits         |

### Execution Details

1. **Start**: When the force ramp triggers UPLIFT (after the final ramp step completes), the controller captures the current `O_T_EE_d` as the start pose and records `z₀ = O_T_EE_d[14]`. Distance `d` and speed `v_lift` are taken from RT-local copies; duration is computed as `T = d / v_lift`. BASELINE prep downlift uses the same mechanism but descends.

2. **Per-tick** (1 kHz): The controller increments `t ← t + Δt`, computes `s_raw`, `s`, and `z(t)`, then calls `setCommand(pose)`. Only the z-translation (index [14]) is modified — orientation and x/y position remain unchanged from the start pose.

3. **Completion**: When `t ≥ T`, the trajectory is done. The active flag is cleared and the controller transitions to the next state (EVALUATE after UPLIFT; BASELINE prep continues after downlift).

4. **Passthrough mode**: When not executing any trajectory, the controller reads `O_T_EE_d` and writes it back as the command every tick. This produces zero tracking error — the robot holds position with no drift or jerk.

### Safety Constraints

| Constraint              | Symbol / Value    | Description                                                                |
| ----------------------- | ----------------- | -------------------------------------------------------------------------- |
| Maximum closing speed   | `v ≤ 0.10 m/s`    | Hard clamp — speeds above 0.10 m/s are clamped with a warning              |
| Maximum uplift distance | `d ≤ 0.3 m`       | Hard clamp — any `d > 300 mm` is clamped with a warning                    |
| Minimum lift speed      | `v ≥ 0.001 m/s`   | Hard clamp — prevents divide-by-zero in UPLIFT/downlift duration calc      |
| Minimum uplift hold     | `t ≥ 0.5 s`       | Hard clamp — ensures W_pre ≥ 0.25 s for reliable pre-lift baseline         |
| Maximum uplift hold     | `t ≤ 120.0 s`     | Hard clamp — prevents width sample vector from exceeding pre-allocated capacity |
| Precondition            | —                 | GRASPING requires CONTACT state                                           |
| BASELINE interruption   | —                 | BASELINE clears active trajectories, returns to passthrough (with warning) |
| CLOSING_COMMAND timeout | 10 s              | Transitions to FAILED if move command never starts executing               |
| CLOSING timeout         | 30 s              | Transitions to FAILED if CLOSING phase exceeds duration without contact    |
| GRASPING timeout        | 10 s              | Transitions to FAILED if gripper command does not complete; sends `stop_requested_` to cancel in-flight command |
| Action server timeout   | 30 s              | Action server commands abort if gripper does not respond within timeout     |
| Force ramp limit        | `F ≤ f_max`       | Ramp clamps to f_max (always reached); slip at EVALUATE transitions to FAILED |

## Configuration

### Controller Parameters (`config/kitting_state_controller.yaml`)

| Parameter                  | Type   | Default | Description                                                         |
| -------------------------- | ------ | ------- | ------------------------------------------------------------------- |
| `arm_id`                   | string | `panda` | Robot arm identifier                                                |
| `publish_rate`             | double | `250.0` | State data publish rate [Hz]                                        |
| `closing_width`            | double | `0.001` | Default width for MoveAction in CLOSING [m]                                |
| `closing_speed`            | double | `0.05`  | Default speed for MoveAction in CLOSING [m/s] (clamped to max 0.10) |
| `grasp_speed`              | double | `0.02`  | Gripper speed for GraspAction [m/s]                                 |
| `epsilon`                  | double | `0.008` | Epsilon for GraspAction (inner and outer) [m]                       |
| `f_min`                    | double | `3.0`   | Starting grasp force [N]                                            |
| `f_step`                   | double | `3.0`   | Force increment per ramp step [N]                                   |
| `f_max`                    | double | `70.0`  | Maximum force for the final ramp step [N] (ramp clamps to this value) |
| `uplift_distance`          | double | `0.010` | Micro-uplift distance after final ramp step [m] (max 0.3)           |
| `lift_speed`               | double | `0.01`  | Lift speed for UPLIFT and BASELINE prep downlift [m/s] (min 0.001)  |
| `uplift_hold`              | double | `1.0`   | Hold time at top for evaluation: early (first half) + late (second half) windows [s] (min 0.5, max 120.0) |
| `grasp_force_hold_time`    | double | `1.0`   | Hold at each force ramp step before advancing [s] (min 0.25). W_pre accumulated during HOLDING of last step |
| `grasp_settle_time`        | double | `0.5`   | Settle time after each grasp command completes [s]                  |
| `slip_drop_thresh`         | double | `0.15`   | DF_TH: max allowed relative support force drop (15% = fail)         |
| `slip_width_thresh`        | double | `0.0005` | W_TH: max allowed jaw widening P95-P5 [m] (0.5 mm = fail)          |
| `load_transfer_min`        | double | `1.5`    | Floor for load transfer threshold [N] (lower for light objects)     |
| `require_logger`           | bool   | `false` | Gate commands behind logger readiness (set true when `record:=true`)|

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
  method: sms_cusum
```

### CSV Export

When `export_csv_on_stop` is `true` (default), the logger automatically reads back the rosbag after recording stops and writes a flattened CSV file using the C++ `rosbag::View` API. The export runs in a background thread so the stop operation returns immediately.

The CSV contains one row per `KittingState` message (70 columns):

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
| Derived forces   | `support_force` (Fn = \|Fz\|), `tangential_force` (Ft = √(Fx²+Fy²))                       |
| Gripper          | `gripper_width`, `gripper_width_dot`, `gripper_width_cmd`, `gripper_max_width`, `gripper_is_grasped` |
| Force ramp       | `grasp_force`, `grasp_iteration`, `grasp_ramp_phase`                                        |

`O_T_EE` (16 values) and `jacobian` (42 values) are not included in the CSV. They remain in the rosbag.

State labels come from the `/kitting_controller/state` topic. During CSV export, the logger tracks the most recent state label and attaches it to each `KittingState` row, providing per-sample state identification.

## KittingGripperCommand Message

Per-object command published on `/kitting_controller/state_cmd`. Any float64 parameter left at `0.0` falls back to the YAML config default. This lets you override only the parameters that differ for a particular object.

| Field                  | Type    | Description                                                                        |
| ---------------------- | ------- | ---------------------------------------------------------------------------------- |
| `command`              | string  | `"BASELINE"`, `"CLOSING"`, `"GRASPING"`, or `"AUTO"`                               |
| `open_gripper`         | bool    | If true, open gripper before baseline collection. Also auto-enabled when `record:=true` (default: false) |
| `open_width`           | float64 | Width to open to [m] (0 = max_width from firmware). Only if `open_gripper` is true |
| `closing_width`        | float64 | Target width for MoveAction [m] (0 = use default)                                  |
| `closing_speed`        | float64 | Speed for MoveAction [m/s] (0 = use default, max 0.10)                             |
| `contact_torque_thresh` | float64 | *Ignored* — kept for message compatibility. SMS-CUSUM auto-tunes detection          |
| `contact_debounce_time` | float64 | *Ignored* — kept for message compatibility. SMS-CUSUM uses sample-based debounce    |
| `f_min`                | float64 | Starting grasp force [N] (0 = use default 3.0)                                     |
| `f_step`               | float64 | Force increment per ramp step [N] (0 = use default 3.0)                            |
| `f_max`                | float64 | Maximum force for the final ramp step [N] (0 = use default 70.0)                   |
| `grasp_force_hold_time` | float64 | Hold at each force ramp step before advancing [s] (0 = use default 1.0)            |
| `grasp_settle_time`    | float64 | Settle after each grasp command completes [s] (0 = use default 0.5)                |
| `fr_uplift_distance`   | float64 | Micro-uplift distance after final ramp step [m] (0 = use default 0.010, max 0.3)   |
| `fr_lift_speed`        | float64 | Lift speed for final UPLIFT [m/s] (0 = use default 0.01, min 0.001)                |
| `fr_uplift_hold`       | float64 | Hold time at top for evaluation [s] (0 = use default 1.0, min 0.5, max 120.0)      |
| `fr_grasp_speed`       | float64 | Gripper speed for ramp GraspAction [m/s] (0 = use default 0.02)                    |
| `fr_epsilon`           | float64 | Epsilon for ramp GraspAction, inner and outer [m] (0 = use default 0.008)          |
| `fr_slip_drop_thresh`     | float64 | DF_TH: max allowed relative support force drop (0 = use default 0.15)           |
| `fr_slip_width_thresh`    | float64 | W_TH: max allowed jaw widening P95-P5 [m] (0 = use default 0.0005)             |
| `fr_load_transfer_min`    | float64 | Floor for load transfer threshold [N] (0 = use default 1.5)                     |
| `auto_delay`           | float64 | Delay between auto transitions [s] (0 = default 5.0). Only used by `AUTO` command  |

Only the parameters relevant to the command are used:

- `BASELINE` uses `open_gripper` and `open_width`
- `CLOSING` uses `closing_width` and `closing_speed` (`contact_torque_thresh` and `contact_debounce_time` are ignored — SMS-CUSUM auto-tunes detection)
- `GRASPING` requires CONTACT state and uses all `f_*`/`fr_*` force ramp parameters plus `grasp_force_hold_time` and `grasp_settle_time` (grasp width is always from `contact_width`; rejected if not in CONTACT or if `contact_width` is out of range `[0, max_width]`)
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
| `support_force` | float64    | Support force Fn = \|Fz\| [N] — used by slip evaluation (Gates 1-2 in EVALUATE) |
| `tangential_force` | float64 | Tangential force Ft = √(Fx²+Fy²) [N] — logged for monitoring |
| `gripper_width` | float64    | Measured finger width [m]                         |
| `gripper_width_dot` | float64 | Finger width velocity (finite difference) [m/s]  |
| `gripper_width_cmd` | float64 | Commanded closing width [m] (0 if not CLOSING_COMMAND/CLOSING/CONTACT_CONFIRMED) |
| `gripper_max_width` | float64 | Maximum gripper opening width [m]                |
| `gripper_is_grasped` | bool   | Firmware-level grasp detection flag               |
| `grasp_force`  | float64     | Current grasp force [N] (0 if not in force ramp)  |
| `grasp_iteration` | int32    | Force ramp iteration (0-based; 0 = GRASP_1)       |
| `grasp_ramp_phase` | string  | Sub-phase within current GRASP step: `"settling"`, `"holding"`, or `""` (empty outside ramp) |

## Interfaces

The controller claims three hardware interfaces:

- `FrankaStateInterface` -- provides access to `franka::RobotState`
- `FrankaModelInterface` -- provides access to dynamics/kinematics (Jacobian, gravity, Coriolis)
- `FrankaPoseCartesianInterface` -- provides exclusive Cartesian pose command authority for UPLIFT trajectories and BASELINE prep lowering

When not executing a trajectory, the controller operates in **passthrough mode**: it reads the robot's own desired pose (`O_T_EE_d`) and writes it back as the command every tick. This results in zero tracking error and the robot holds position. During UPLIFT (and BASELINE prep lowering), the controller commands a cosine-smoothed trajectory. Gripper operations use the libfranka `franka::Gripper` API directly via two dedicated threads:

- **Read thread**: Continuously calls `readOnce()` at firmware rate, computes width velocity via finite difference, writes `GripperData` to `RealtimeBuffer`, executes `stop()` when the RT loop sets the `stop_requested_` atomic flag, and dispatches deferred grasp commands from the RT force ramp via atomic flag polling (calls `stop()` before each deferred grasp to release any existing grip — prevents libfranka hang on re-grasp)
- **Command thread**: Waits on a condition variable for queued `move()`/`grasp()`/`homing()` commands from the subscriber callback or action server threads, then executes them (blocking calls, interruptible by `stop()`). Stores the `grasp()` return value in `cmd_success_` (release) before setting `cmd_executing_` false — allows the RT thread to detect hardware-rejected grasps

The controller also exposes four `franka_gripper`-compatible action servers (`/franka_gripper/move`, `/franka_gripper/grasp`, `/franka_gripper/homing`, `/franka_gripper/stop`) for external gripper control. Move, grasp, and homing actions are routed through the command thread; stop is routed through the read thread's `stop_requested_` flag to serialize all gripper I/O. Actions are guarded by the state machine — only allowed when idle (START, SUCCESS, FAILED). See [Gripper Action Servers](#gripper-action-servers).

The `franka_gripper` package is used only for action type definitions — `franka_gripper_node` is **not** launched. The robot must be launched with `load_gripper:=false` to prevent conflicting gripper connections (only one `franka::Gripper` connection per robot is allowed).

## Real-Time Safety

- Uses `realtime_tools::RealtimePublisher` with non-blocking `trylock()`
- No dynamic memory allocation in `update()` — width sample vector pre-allocated in `starting()` with capacity for 120 s at 250 Hz (`kMaxWidthSamples + 1 = 30001`, the extra slot prevents a heap allocation on the edge-case tick where push_back fires simultaneously with evaluation)
- No blocking operations in `update()`
- Model queries are only called at the publish rate, not every control tick
- Contact detection (SMS-CUSUM) uses O(1) per sample, zero dynamic allocation, only scalar arithmetic
- Gripper data passed to RT loop via lock-free `RealtimeBuffer` (no mutex in `update()`)
- Gripper stop on contact uses single atomic store (`stop_requested_`), nanoseconds, RT-safe
- Read thread checks `stop_requested_` flag after each `readOnce()`, calls `stop()` (non-RT), and sets `gripper_stopped_` to confirm. If `stop()` fails (exception), `stop_requested_` remains true for automatic retry on the next iteration — `gripper_stopped_` is only set on success
- CONTACT state transition deferred until `gripper_stopped_` is true — ensures stopped gripper before recording
- Command thread executes blocking `move()`/`grasp()` (non-RT), interruptible by `stop()`
- No blocking gripper calls in `update()` — all gripper I/O runs in dedicated threads
- State publisher uses `trylock()` pattern (publish only on transition, non-blocking)
- Cartesian pose command issued every tick (1 kHz) — passthrough when idle, trajectory during UPLIFT/BASELINE prep lowering
- UPLIFT/downlift trajectories use only `std::array<double, 16>`, `std::cos()`, and `std::min()` — no allocation
- Force ramp state machine (`runInternalTransitions`) runs at 250 Hz with only atomic stores, timestamp arithmetic, and trylock publish — fully RT-safe
- Deferred grasp mechanism: RT thread stores params + release-stores flag; read thread acquire-loads, calls `stop()` to release any existing grip, then dispatches. Command thread stores `grasp()` result in `cmd_success_` (release); RT thread acquire-loads to detect failure (no blocking calls in RT)
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
- **BASELINE with `open_gripper: true`** (or `record:=true`) defers gripper open until any downlift correction completes, then opens the gripper, then starts baseline collection — sequential preparation ensures clean sensor data
- **BASELINE** can be published from any state to reset the state machine for a new trial
- Recording starts automatically when the logger launches — no START command needed
- Recording continues through all state transitions until STOP, ABORT, terminal state (SUCCESS/FAILED), or node shutdown
- Recording auto-stops on SUCCESS or FAILED (terminal state message is written to bag before closing)
- Recording auto-starts a new trial on BASELINE after a terminal state — each trial gets its own directory and incrementing trial number
- Recording and state labeling are independent
- State labels on `/kitting_controller/state` are published by the controller for offline segmentation (11 states, with GRASPING published as GRASP_1..GRASP_N)
- State labels merged into CSV from `/kitting_controller/state` topic for per-sample state identification
- One rosbag per trial containing all signals and all state transitions
- STOP saves bag + metadata + CSV, ABORT deletes trial (no CSV)
- Logger shutdown (Ctrl+C) automatically stops recording (equivalent to STOP)
- Bag contains 2 topics: kitting_state_data, state
- CSV contains 70 flattened columns with state labels per row (includes `grasp_ramp_phase`)
- CSV export does not block the ROS spin loop (runs in background thread)
- metadata.yaml contains bag_filename, csv_filename, total_samples, start/stop times, and detector parameters
- Contact detection: SMS-CUSUM (noise-adaptive CUSUM: S = max(0, S + (μ − tau) − k_eff), alarm when S ≥ h for 3 consecutive samples)
- Free closing (no object): width reaches w_cmd, gap ~0 → no false CONTACT → transitions to FAILED ("no contact detected")
- Object contact: SMS-CUSUM detects torque drop → CONTACT_CONFIRMED immediately, then CONTACT when gripper stopped
- Gripper stop requested immediately on contact: `stop()` called via atomic flag and read thread
- CONTACT_CONFIRMED published at SMS-CUSUM detection; CONTACT deferred until gripper has physically stopped
- Grasp commands always use the current gripper width (`gs.width` from `readOnce()`) as the target — never a pre-stored value. Both initial and deferred grasps are routed through the read thread, which reads the fresh width at dispatch time. This prevents the gripper from opening to reach a stale target.
- `contact_width_` is stored for logging and RT-thread diagnostics but is NOT used as the grasp target
- KittingState contains robot signals, gripper signals, and force ramp state — all published at 250 Hz
- Publishing CLOSING on `state_cmd` triggers gripper `move()` + CLOSING_COMMAND state label (transitions to CLOSING when move confirmed executing)
- Publishing BASELINE on `state_cmd` prepares for new trial (optionally opens gripper)
- Publishing GRASPING on `state_cmd` starts the automated force ramp (uses contact_width from CONTACT)
- GRASPING timeout: 10 seconds maximum for any ramp step's gripper command completion, then FAILED
- Force ramp runs autonomously after GRASPING: GRASP_1 → GRASP_2 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED
- Each ramp step: grasp at force → settle (grasp_settle_time) → hold/log (grasp_force_hold_time) → update contact_width → advance
- W_pre accumulated during HOLDING sub-phase of the last ramp step only
- Slip detected during EVALUATE transitions to FAILED (no retry loop)
- Deferred grasp mechanism used for ramp step advancement: RT thread stores params, read thread calls `stop()` to release any existing grip, then dispatches
- GRASPING timeout sends `stop_requested_` to cancel the in-flight gripper command — prevents stale commands from blocking future operations
- UPLIFT trajectory uses cosine smoothing with duration computed from distance/speed (downlift used only for BASELINE prep)
- UPLIFT/downlift orientation remains unchanged throughout the motion
- Uplift distance is clamped to 300 mm maximum (with warning)
- Lift speed is clamped to minimum `kMinLiftSpeed` (0.001 m/s) — prevents divide-by-zero in UPLIFT/downlift duration calculation
- Uplift hold is clamped to maximum `kMaxUpliftHold` (120.0 s) — prevents width sample vector from exceeding pre-allocated capacity
- SUCCESS keeps arm elevated for pick-and-place; accumulated uplift corrected automatically on next BASELINE (BASELINE prep downlift)
- BASELINE during active force ramp clears trajectories and returns to passthrough
- Per-command force ramp parameters override YAML defaults when non-zero
- Parameters left at 0.0 fall back to YAML config values
- Duplicate CLOSING commands are ignored (rejected during CLOSING_COMMAND or CLOSING)
- CLOSING_COMMAND times out after `kClosingCmdTimeout` (10 s) if the move command never starts executing — prevents indefinite stall if the command thread is blocked
- CLOSING phase times out after `kClosingTimeout` (30 s) if contact is not detected — prevents indefinite stall if the gripper gets stuck or the action never completes
- Controller holds position (passthrough) when not executing trajectory — no drift or jerk
- Gripper connection failure at init returns false (controller not loaded)
- Controller destructor shuts down action servers then joins gripper threads cleanly on unload
- Gripper action servers (move, grasp, homing, stop) advertised under `/franka_gripper/` namespace
- Action servers guarded by state machine — only allowed in START, SUCCESS, FAILED
- Action server requests rejected with descriptive error during active grasp sequence (BASELINE through EVALUATE)
- Stop action always allowed regardless of state — routed through read thread's `stop_requested_` flag (serializes all gripper I/O)
- Move, grasp, homing actions routed through command thread to prevent concurrent libfranka calls
- Action server commands time out after `kActionTimeoutSec` (30 s) — prevents indefinite blocking if the gripper firmware hangs
- Action server shutdown before gripper thread shutdown prevents hanging futures on controller unload
- AUTO command runs full grasp sequence: BASELINE → *(wait for prep + baseline ready)* → *(delay)* → CLOSING_COMMAND → CLOSING → *(wait for CONTACT_CONFIRMED → CONTACT)* → *(delay)* → GRASPING
- AUTO mode polls for baseline readiness (`baseline_prep_done_` && `cd_baseline_ready_`, both `std::atomic<bool>`) every 100 ms via `autoBaselinePollCallback` — only starts `auto_delay` countdown after baseline is fully ready, ensuring arm lowering, gripper open, and baseline data collection all complete before advancing
- AUTO mode uses configurable `auto_delay` (default 5.0 seconds) between baseline-ready→CLOSING_COMMAND and CONTACT→GRASPING transitions
- AUTO mode forwards all BASELINE, CLOSING, and GRASPING parameters from the single message to each stage
- AUTO mode cancelled by any manual command (BASELINE, CLOSING, GRASPING) — allows user override at any point
- `auto_mode_` flag is `std::atomic<bool>` — safe with `ros::AsyncSpinner` concurrent timer/subscriber callbacks
- AUTO mode ends automatically on FAILED during BASELINE prep or CLOSING (no contact detected), or after GRASPING starts (force ramp runs autonomously: GRASP_1 → ... → GRASP_N → UPLIFT → EVALUATE → SUCCESS/FAILED)
- Auto mode timers run on ROS spinner thread (non-RT); reuse existing handle functions — no duplicated logic

## License

Apache 2.0
