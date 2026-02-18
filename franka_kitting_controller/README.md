# franka_kitting_controller

Passive, real-time state acquisition controller for the Franka Panda with Phase 2 contact detection and rosbag data collection. Reads all robot state, model, and Cartesian signals, publishes them as a single `KittingState` message, and autonomously detects contact using statistical thresholding. This controller does **not** command torques, modify stiffness, or change impedance.

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity

**Phase 2** adds:
- 5-state grasp machine: `BASELINE` -> `CLOSING` -> `CONTACT` -> `SECURE_GRASP` -> `UPLIFT`
- Autonomous contact detection using baseline statistics + threshold + debounce
- One rosbag per trial with all state transitions
- Automatic CSV export on STOP for analysis-ready datasets
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
roslaunch franka_kitting_controller kitting_phase2.launch object_name:=cup
```

| Argument               | Default          | Description                              |
|------------------------|------------------|------------------------------------------|
| `object_name`          | `default_object` | Object name for bag file naming          |
| `auto_stop_on_contact` | `false`          | Auto-stop recording on CONTACT           |

## Phase 2: Interaction States

Five discrete interaction states structure grasp execution into measurable phases. They are published on `/kitting_phase2/state` for signal labeling and offline analysis. States do NOT control recording (except optional auto-stop on CONTACT).

```
BASELINE -> CLOSING -> CONTACT -> SECURE_GRASP -> UPLIFT
```

### BASELINE

Establish reference signal behavior before interaction.

- Gripper fully open, end-effector stationary, no object contact
- External torque norm reflects system noise only
- Used for computing baseline mean (mu), standard deviation (sigma), and contact threshold theta = mu + k*sigma

### CLOSING

Observe approach dynamics before contact.

- Gripper begins closing toward object
- External torque gradually changes
- No confirmed sustained contact yet (no sustained exceedance of threshold theta)

### CONTACT

Detect first stable physical interaction. Published automatically by the controller.

- Object touches gripper fingers
- External torque norm exceeds threshold theta continuously for T_hold seconds
- CONTACT is latched once detected (cannot return to CLOSING)

### SECURE_GRASP

Characterize stable object compression.

- Gripper continues slight compression after contact
- External torque stabilizes with reduced variance compared to CONTACT transient
- Stable force distribution

### UPLIFT

Validate grasp robustness under load.

- End-effector performs micro-lift (2-5 mm)
- Object weight transfers to gripper
- Predictable change in torque distribution; no large instability if grasp is secure

### Topics

Recording and state labeling are independent concerns.

| Topic                                    | Type             | Direction  | Description                              |
|------------------------------------------|------------------|------------|------------------------------------------|
| `<ns>/kitting_state_data`                | KittingState     | Published  | Full state data at 250 Hz                |
| `/kitting_phase2/state`                  | std_msgs/String  | Pub/Sub    | State labels for offline segmentation    |
| `/kitting_phase2/record_control`         | std_msgs/String  | Subscribed | Recording control: START, STOP, ABORT    |

- `/kitting_phase2/state` — The **user** publishes state labels (BASELINE, CLOSING, SECURE_GRASP, UPLIFT) to transition the controller. The **controller** publishes CONTACT when auto-detected. States are labels for offline analysis — they do NOT control recording.
- `/kitting_phase2/record_control` — The **user** publishes START, STOP, ABORT to control recording. The **logger** subscribes to this topic.

### Recording

One rosbag per trial. Recording and state labeling are independent — recording continues regardless of state changes until STOP is published.

| Command | Action                                           |
|---------|--------------------------------------------------|
| `START` | Create new trial directory, open bag, begin recording |
| `STOP`  | Close bag, save metadata, export CSV (if enabled) |
| `ABORT` | Close bag, delete trial directory (no CSV)        |

- **START** is ignored if already recording.
- **STOP** is ignored if not recording.
- **ABORT** is ignored if not recording.

If `auto_stop_on_contact` is enabled and the controller publishes CONTACT, recording is auto-stopped (equivalent to STOP).

### Sending Commands

```bash
# Start recording
rostopic pub /kitting_phase2/record_control std_msgs/String "data: 'START'" --once

# Set state labels (these are just labels, they don't affect recording)
rostopic pub /kitting_phase2/state std_msgs/String "data: 'BASELINE'" --once
rostopic pub /kitting_phase2/state std_msgs/String "data: 'CLOSING'" --once
# ... CONTACT is published by the controller automatically ...
rostopic pub /kitting_phase2/state std_msgs/String "data: 'SECURE_GRASP'" --once
rostopic pub /kitting_phase2/state std_msgs/String "data: 'UPLIFT'" --once

# Stop recording
rostopic pub /kitting_phase2/record_control std_msgs/String "data: 'STOP'" --once

# Or abort (delete trial)
rostopic pub /kitting_phase2/record_control std_msgs/String "data: 'ABORT'" --once
```

## Phase 2: Contact Detection Mathematics

The controller detects contact using the following algorithm on `tau_ext_norm`:

### Baseline (during BASELINE state)

Collect N samples of x(t) = tau_ext_norm over T_base seconds.

Compute:
- Mean: `mu = (1/N) * sum(x_i)`
- Standard deviation: `sigma = sqrt( (1/(N-1)) * sum((x_i - mu)^2) )`
- Threshold: `theta = mu + k * sigma`

### Detection (during CLOSING state)

Contact is declared when `x(t) > theta` continuously for T_hold seconds (debounce).

Optional slope gate (disabled by default): also requires `dx/dt > slope_min`.

Once CONTACT is declared, it is **latched** (cannot return to CLOSING).

## Configuration

### Controller Parameters (`config/kitting_state_controller.yaml`)

| Parameter                  | Type   | Default | Description                                    |
|----------------------------|--------|---------|------------------------------------------------|
| `arm_id`                   | string | `panda` | Robot arm identifier                           |
| `publish_rate`             | double | `250.0` | State data publish rate [Hz]                   |
| `enable_contact_detector`  | bool   | `true`  | Enable Phase 2 contact detection               |
| `T_base`                   | double | `0.7`   | Baseline collection duration [s]               |
| `N_min`                    | int    | `50`    | Minimum samples before arming detection        |
| `k_sigma`                  | double | `5.0`   | Threshold multiplier (theta = mu + k*sigma)    |
| `T_hold`                   | double | `0.12`  | Debounce hold time [s]                         |
| `use_slope_gate`           | bool   | `false` | Enable slope gate (for drift false positives)  |
| `slope_dt`                 | double | `0.02`  | Slope finite difference dt [s]                 |
| `slope_min`                | double | `5.0`   | Minimum slope for contact [1/s]                |

### Logger Parameters (`config/kitting_phase2_logger.yaml`)

| Parameter              | Type        | Default              | Description                                  |
|------------------------|-------------|----------------------|----------------------------------------------|
| `base_directory`       | string      | `~/kitting_bags`     | Root directory for bag files                 |
| `object_name`          | string      | `default_object`     | Object identifier for file naming            |
| `auto_stop_on_contact` | bool        | `false`              | Auto-stop recording on CONTACT               |
| `export_csv_on_stop`   | bool        | `true`               | Auto-export CSV when recording stops         |
| `topics_to_record`     | string list | (see below)          | Topics recorded in rosbag                    |

Default recorded topics (pure signal only):
- `/kitting_state_controller/kitting_state_data`
- `/kitting_phase2/state`
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

The single bag contains all data from all states (BASELINE through UPLIFT) for that trial. The CSV is exported in a background thread so STOP returns immediately.

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
  k_sigma: 5.0
  T_hold: 0.12
  use_slope_gate: false
  slope_dt: 0.02
  slope_min: 5.0
baseline_statistics:
  baseline_mu_tau_ext_norm: 0.342
  baseline_sigma_tau_ext_norm: 0.051
  contact_threshold_theta: 0.597
```

### CSV Export

When `export_csv_on_stop` is `true` (default), the logger automatically reads back the rosbag after STOP and writes a flattened CSV file using the C++ `rosbag::View` API. The export runs in a background thread so STOP returns immediately.

The CSV contains one row per `KittingState` message (60 columns):

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

`O_T_EE` (16 values) and `jacobian` (42 values) are not included in the CSV. They remain in the rosbag.

State labels are synchronized by iterating the bag chronologically: each signal row gets the most recent `/kitting_phase2/state` label at that timestamp.

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

## Interfaces

The controller claims two read-only hardware interfaces:

- `FrankaStateInterface` -- provides access to `franka::RobotState`
- `FrankaModelInterface` -- provides access to dynamics/kinematics (Jacobian, gravity, Coriolis)

No command interfaces (EffortJoint, CartesianPose, etc.) are claimed. The controller is purely passive.

## Real-Time Safety

- Uses `realtime_tools::RealtimePublisher` with non-blocking `trylock()`
- No dynamic memory allocation in `update()`
- No blocking operations in `update()`
- Model queries are only called at the publish rate, not every control tick
- Contact detection uses only scalar arithmetic (no Eigen, no allocation)
- State publisher uses `trylock()` pattern (publish only on transition, non-blocking)
- Rosbag management runs in a separate C++ node (no Python overhead)

## Recording Performance

The Phase 2 logger is written in C++ using the `rosbag::Bag` API directly and `topic_tools::ShapeShifter` for generic topic subscription. This eliminates:

- **No subprocess spawn** — bag is opened via API, not `rosbag record` subprocess
- **No Python GIL** — pure C++ callbacks, no interpreter overhead
- **No subscription handshake delay** — topics are already subscribed when recording starts; the bag file is simply opened and messages are written immediately
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

- Recording and state labeling are independent
- START/STOP/ABORT on `/kitting_phase2/record_control` control recording
- State labels on `/kitting_phase2/state` are for offline segmentation only
- One rosbag per trial containing all signals and all state transitions
- START is ignored if already recording
- STOP saves bag + metadata + CSV, ABORT deletes trial (no CSV)
- CONTACT auto-stop works when `auto_stop_on_contact` is enabled
- Bag contains exactly 4 topics: kitting_state_data, state, record_control, joint_states
- CSV contains 60 flattened columns with correct state labels per row
- CSV export does not block the ROS spin loop (runs in background thread)
- metadata.yaml contains bag_filename, csv_filename, total_samples, start/stop times, and detector parameters

## License

Apache 2.0
