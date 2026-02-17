# franka_pick_place_controller

Gazebo-only pick-and-place controller for the Franka Panda robot using direct joint trajectory control. **No MoveIt dependency.**

## Overview

This package moves a stone object from a pick tray to a place tray using:

- `effort_joint_trajectory_controller` via `FollowJointTrajectoryAction` for arm motion
- `/franka_gripper/move` and `/franka_gripper/grasp` actions for gripper control
- Friction-based grasping through the `FrankaGripperSim` HOLDING state

## Prerequisites

- ROS Noetic
- `franka_ros` workspace built (includes `franka_gazebo`, `franka_gripper`)
- Gazebo (installed with ROS)

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

Headless example:

```bash
roslaunch franka_pick_place_controller pick_place.launch headless:=true
```

## What Happens

1. Gazebo starts with the `stone.sdf` world (table, pick tray, place tray, stone)
2. Panda spawns at the origin with the `effort_joint_trajectory_controller` and gripper
3. After controllers initialize, the pick-and-place node runs:
   - Verifies all models are present and resets the stone position
   - Moves to home pose
   - **Pick:** opens gripper → moves above stone → lowers → grasps (50 N) → lifts
   - **Place:** moves above place tray → lowers → releases → retreats → returns home

## Architecture

```
pick_place.launch
  ├── franka_gazebo/panda.launch   (Gazebo + robot + controllers)
  │     ├── world: stone.sdf       (table, trays, stone)
  │     ├── controller: effort_joint_trajectory_controller
  │     └── use_gripper: true      (spawns FrankaGripperSim)
  └── panda_pick_place_gazebo_no_moveit.py
        ├── SimpleActionClient → /effort_joint_trajectory_controller/follow_joint_trajectory
        ├── SimpleActionClient → /franka_gripper/move
        └── SimpleActionClient → /franka_gripper/grasp
```

## Tuning Joint Waypoints

The joint configurations (`PRE_GRASP`, `GRASP`, `PRE_PLACE`, `PLACE`, etc.) are pre-computed estimates. If the end-effector doesn't align with the stone or tray:

1. Launch the simulation:
   ```bash
   roslaunch franka_pick_place_controller pick_place.launch paused:=true
   ```
2. Jog joints interactively:
   ```bash
   rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
   ```
3. Once the gripper is positioned correctly, read back the joint values by calling `_read_current_joints()` in the Python node, or:
   ```bash
   rostopic echo /joint_states -n 1
   ```
4. Update the corresponding constant in `panda_pick_place_gazebo_no_moveit.py`.

## Grasping Notes

This package uses **friction-based grasping** — the `FrankaGripperSim` applies continuous force (50 N) per finger in its HOLDING state. The stone model has a friction coefficient of 0.6 and a mass of 82 g, which is sufficient for holding during moderate-speed motions.

If the stone drops during transport, you can:

- Increase `GRASP_FORCE` in the script (up to ~100 N)
- Slow down arm motions by increasing `DEFAULT_DURATION`
- Add `gazebo_ros_link_attacher` to the workspace and implement `attach_object()`/`detach_object()` with the `/link_attacher_node/attach` and `/link_attacher_node/detach` services
