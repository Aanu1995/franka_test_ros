cmake_minimum_required(VERSION 3.4)
project(franka_pick_place_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  control_msgs
  trajectory_msgs
  franka_gripper
  franka_gazebo
  gazebo_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/panda_pick_place_gazebo_no_moveit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
