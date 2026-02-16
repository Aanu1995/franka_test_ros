# franka_kitting_controller

Passive, real-time state acquisition controller for the Franka Panda. Reads all robot state, model, and Cartesian signals and publishes them as a single `KittingState` message at configurable rate. This controller does **not** command torques, modify stiffness, or change impedance.

Part of **Phase 1** of the force-controlled grasping development pipeline for automated kitting.

## Overview

This controller runs inside the `ros_control` real-time loop provided by `franka_control`. It acquires:

- **Joint-level signals**: positions, velocities, measured torques, estimated external torques (7 DOF each)
- **Cartesian-level signals**: end-effector pose (4x4), external wrench (6D)
- **Model-level signals**: Zero Jacobian (6x7), gravity vector, Coriolis vector
- **Derived metrics**: external torque norm, wrench norm, end-effector velocity

All signals are published in a single `KittingState` message via a real-time safe publisher.

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

### Launch

```bash
roslaunch franka_kitting_controller kitting_state_controller.launch robot_ip:=<ROBOT_IP>
```

Optional arguments:
- `robot_ip` (required): IP address of the Franka robot
- `load_gripper` (default: `true`): whether to load the gripper driver

### Verify

```bash
# Check the topic exists
rostopic list | grep kitting

# Check publish rate (~250 Hz by default)
rostopic hz /kitting_state_controller/kitting_state_data

# View a single message
rostopic echo /kitting_state_controller/kitting_state_data -n 1
```

### Record data

```bash
rosbag record /kitting_state_controller/kitting_state_data -O kitting_test.bag
```

## Configuration

Parameters in `config/kitting_state_controller.yaml`:

| Parameter      | Type   | Default | Description                        |
|----------------|--------|---------|------------------------------------|
| `arm_id`       | string | `panda` | Robot arm identifier               |
| `publish_rate` | double | `250.0` | Publishing rate in Hz (max 1000)   |

## KittingState Message

Topic: `<controller_ns>/kitting_state_data`

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
- Model queries (`getZeroJacobian`, `getGravity`, `getCoriolis`) are only called at the publish rate, not every control tick
- Rate-gated via `franka_hw::TriggerRate` to downsample from the 1 kHz control loop

## Validation

### Test 1 -- Free Space Motion

Move the robot without contact.

**Expected**: `wrench_ext` near zero, `tau_ext` near noise floor, stable signals.

### Test 2 -- Manual External Push

Push the robot gently by hand.

**Expected**: increase in `tau_ext_norm` and `wrench_norm`.

### Test 3 -- Controlled Table Contact

Move the end-effector slowly into a table surface.

**Expected**: clear increase in `wrench_ext[2]` (Fz), `wrench_norm`, and corresponding increase in `tau_ext_norm`. Signals must show consistent, repeatable patterns.

### Completion Criteria

Phase 1 is complete when:

- All required signals are accessible and published
- Norm metrics are correctly computed
- Noise floor is identified
- Contact produces measurable signal jumps
- `rosbag` logs confirm deterministic behavior
- No safety warnings or torque interference occur

## License

Apache 2.0
