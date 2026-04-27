# franka_test_ros

ROS 1 codebase for Franka Panda kitting experiments. The main package is `franka_kitting_controller`, a `ros_control` controller that publishes robot state, detects contact with SMS-CUSUM, controls the Franka gripper during grasp trials, and records trial data to rosbag/CSV.

## What Is In This Repository

| Path | Use |
| ---- | --- |
| `franka_kitting_controller/` | Main package. Contains the kitting controller, logger node, custom messages, configs, launch files, and tests. |
| `sms_cusum/` | Contact and secure-grasp detection algorithm. The controller uses the C++ headers in `sms_cusum/cpp/include`. Python tools are included for offline validation and plotting. |
| `franka_control/` | Franka hardware control node and state controller. Use this to start the real robot control loop. |
| `franka_hw/` | Franka `ros_control` hardware interfaces used by `franka_control` and the kitting controller. |
| `franka_gripper/` | Franka gripper action/message definitions. The kitting controller uses these types and connects to the gripper through `libfranka`. |
| `franka_msgs/` | Franka messages, services, and actions required by the control packages. |
| `kitting_bags/` | Recorded trial bags, CSV files, plots, and metadata from tests. Treat this as experiment data. |

## Prerequisites

Use a ROS 1 catkin setup with ROS Noetic or Melodic.

Install or provide:

- `libfranka` 0.8 or 0.9
- `franka_description` from an external Franka ROS installation, needed by `franka_control.launch` for the Panda URDF
- ROS packages for `controller_manager`, `controller_interface`, `hardware_interface`, `combined_robot_hw`, `joint_limits_interface`, `pluginlib`, `realtime_tools`, `actionlib`, `rosbag`, `topic_tools`, `control_msgs`, `sensor_msgs`, `geometry_msgs`, `tf`, and `tf2_msgs`

The repository already includes the local Franka packages needed by the kitting controller: `franka_control`, `franka_hw`, `franka_gripper`, and `franka_msgs`.

## Build

Put this repository inside a catkin workspace source directory:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# clone or copy this repository here
```

Install dependencies and build:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

With `catkin_tools`, build the kitting package and its dependencies:

```bash
cd ~/catkin_ws
catkin build franka_kitting_controller
source devel/setup.bash
```

## Run The Kitting Controller

Start the Franka robot control loop first:

```bash
roslaunch franka_control franka_control.launch robot_ip:=<ROBOT_IP> load_gripper:=false
```

Use `load_gripper:=false`. The kitting controller opens its own direct `franka::Gripper` connection, so `franka_gripper_node` should not be running at the same time.

In a second terminal, source the workspace and launch the kitting controller:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch franka_kitting_controller kitting_state_controller.launch robot_ip:=<ROBOT_IP>
```

The controller loads `franka_kitting_controller/config/kitting_state_controller.yaml`, spawns `kitting_state_controller`, publishes state data, and waits for a trial command.

## Run A Trial

The main command topic is:

```text
/kitting_controller/state_cmd
```

It uses the message type:

```text
franka_kitting_controller/KittingGripperCommand
```

Manual mode uses three commands: `BASELINE`, `CLOSING`, then `GRASPING`.

```bash
# 1. Prepare and collect baseline
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'BASELINE', open_gripper: true}" --once

# 2. Close the gripper until contact is detected
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'CLOSING'}" --once

# 3. Start the force ramp and evaluation sequence
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING'}" --once
```

Automatic mode runs the same sequence from one command:

```bash
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'AUTO', open_gripper: true, auto_delay: 3.0}" --once
```

The controller publishes the current trial state on:

```text
/kitting_controller/state
```

Typical states are `START`, `UNKNOWN`, `BASELINE`, `CLOSING`, `CONTACT`, `GRASP_1`, `GRASP_2`, `UPLIFT`, `EVALUATE`, `SUCCESS`, and `FAILED`.

## Record Trial Data

Launch the controller with recording enabled:

```bash
roslaunch franka_kitting_controller kitting_state_controller.launch \
  robot_ip:=<ROBOT_IP> \
  record:=true \
  object_name:=<OBJECT_NAME> \
  base_directory:=/path/to/kitting_bags
```

The logger starts automatically, records one bag per trial, and exports CSV files when a trial stops. If you want new data saved in this repository, point `base_directory` at this repo's `kitting_bags/` directory.

Stop or abort recording manually with:

```bash
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'STOP'" --once
rostopic pub /kitting_controller/record_control std_msgs/String "data: 'ABORT'" --once
```

Terminal states `SUCCESS` and `FAILED` also stop recording automatically.

## Tune Parameters

Controller parameters live in:

```text
franka_kitting_controller/config/kitting_state_controller.yaml
```

Logger parameters live in:

```text
franka_kitting_controller/config/kitting_logger.yaml
```

Most per-trial command fields can override YAML defaults. For example:

```bash
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', f_min: 3.0, f_max: 70.0, f_step: 3.0}" --once
```

## Test And Develop

Run the kitting controller tests from the catkin workspace:

```bash
catkin_make run_tests_franka_kitting_controller
catkin_test_results build
```

With `catkin_tools`:

```bash
catkin build franka_kitting_controller --catkin-make-args run_tests
catkin_test_results build
```

The kitting controller tests use mocked handles and do not require robot hardware. Some `franka_hw` tests require `franka_description` because they load the Panda URDF.

## More Documentation

- `franka_kitting_controller/README.md` explains the controller state machine, topics, messages, force-ramp behavior, logger behavior, and test structure.
- `sms_cusum/README.md` explains the SMS-CUSUM algorithm, C++ headers, Python validation tools, and detection parameters.
