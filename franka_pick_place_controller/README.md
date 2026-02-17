# franka_pick_place_controller

Gazebo-only pick-and-place controller for the Franka Panda robot. **No MoveIt dependency.**

## Overview

This package moves a stone object from a pick tray to a place tray using:

- **Runtime pose sensing** -- queries Gazebo for stone/tray positions, reads joint state from `/joint_states`, monitors EE via TF
- **Numerical IK** -- damped-least-squares Jacobian solver computes joint targets from Cartesian goals (no hardcoded waypoints)
- `position_joint_trajectory_controller` via `FollowJointTrajectoryAction` for arm motion
- `/franka_gripper/move` and `/franka_gripper/grasp` actions for gripper control
- Friction-based grasping through the `FrankaGripperSim` HOLDING state

## Prerequisites

- ROS Noetic
- `franka_ros` workspace built (includes `franka_gazebo`, `franka_gripper`)
- Gazebo (installed with ROS)
- NumPy (standard with ROS Python)

## Build

```bash
cd /path/to/franka_test_ros
catkin build franka_pick_place_controller
source devel/setup.bash
```

## Run

```bash
roslaunch franka_pick_place_controller pick_place.launch
```

### Launch Arguments

| Argument   | Default | Description                     |
|------------|---------|---------------------------------|
| `headless` | `false` | Hide the Gazebo GUI             |
| `paused`   | `false` | Start simulation paused         |

## What Happens

1. Gazebo starts with the `stone.sdf` world (table, pick tray, place tray, stone)
2. Panda spawns at the origin with the `position_joint_trajectory_controller` and gripper
3. The node queries Gazebo for the live positions of the stone and trays
4. For each Cartesian target, the numerical IK solver computes joint angles
5. Motions use smooth multi-point trajectories with cubic interpolation
6. Sequence: home -> pre-grasp -> grasp -> lift -> pre-place -> place -> release -> retreat -> home

## Architecture

```
pick_place.launch
  ├── franka_gazebo/panda.launch   (Gazebo + robot + controllers)
  │     ├── world: stone.sdf       (table, trays, stone)
  │     ├── controller: position_joint_trajectory_controller
  │     └── use_gripper: true      (spawns FrankaGripperSim)
  └── panda_pick_place_gazebo_no_moveit.py
        ├── /gazebo/get_model_state  → stone & tray positions
        ├── TF listener              → panda_link0 -> panda_hand
        ├── Numerical IK (DLS)       → Cartesian target -> joint angles
        ├── SimpleActionClient       → /position_joint_trajectory_controller/follow_joint_trajectory
        ├── SimpleActionClient       → /franka_gripper/move
        └── SimpleActionClient       → /franka_gripper/grasp
```

### How Targets Are Computed

1. **Gazebo query**: `get_model_state('stone', 'world')` returns the stone's live (x, y, z)
2. **Cartesian target**: pre-grasp = stone position + `APPROACH_HEIGHT` (15 cm above); grasp = stone position
3. **IK solve**: `solve_ik(T_target, q_current)` uses damped-least-squares with the Panda DH model, clamped to URDF joint limits
4. **Trajectory**: 10-point cubic interpolation from current joints to IK solution

### Motion Safety

- Reads the current joint state before every move
- Generates 10 intermediate waypoints using cubic interpolation (`3t^2 - 2t^3`)
- Duration computed from largest joint displacement, capped at `MAX_JOINT_SPEED = 0.3 rad/s`
- Minimum motion time is 5 seconds

### Configurable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `APPROACH_HEIGHT` | 0.15 m | Height above stone/tray for approach |
| `PLACE_HEIGHT` | 0.08 m | Height above tray surface when placing |
| `MAX_JOINT_SPEED` | 0.3 rad/s | Maximum joint velocity |
| `GRASP_FORCE` | 50 N | Gripper grasping force |
| `GRASP_WIDTH` | 0.025 m | Target gripper width for stone |

## Known Warnings

**`equilibrium_pose contains unnormalized quaternions`** -- This is a known upstream `franka_ros` warning from the `franka_state_controller`. It occurs because the `O_T_EE` 4x4 matrix published in `FrankaState` messages can produce slightly non-unit quaternions due to floating-point drift. It is harmless and does not affect the pick-and-place behaviour. It is not produced by this package.

**`Zero Jacobian close to singularity`** -- This comes from `FrankaHWSim` (`model_kdl.cpp`) which checks the Jacobian's smallest singular value against `singularity_warning_threshold` (0.0001) on every control cycle. It fires when the robot passes through a kinematic singularity (e.g. wrist joints near zero, elbow near straight). This package mitigates it by biasing the IK solver's nullspace toward a preferred configuration that keeps j4 bent and j5/j6 away from zero. The warning may still appear briefly during transitions but does not affect motion execution.

## Grasping Notes

This package uses **friction-based grasping** -- the `FrankaGripperSim` applies continuous force (50 N) per finger in its HOLDING state. The stone model has a friction coefficient of 0.6 and a mass of 82 g.

If the stone drops during transport, you can:

- Increase `GRASP_FORCE` in the script (up to ~100 N)
- Slow down arm motions by decreasing `MAX_JOINT_SPEED` or increasing `DEFAULT_DURATION`
- Add `gazebo_ros_link_attacher` to the workspace and implement `attach_object()`/`detach_object()` with the `/link_attacher_node/attach` and `/link_attacher_node/detach` services
