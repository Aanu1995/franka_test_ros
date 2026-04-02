cmake_minimum_required(VERSION 3.4)
project(franka_kitting_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_interface
  franka_gripper
  franka_hw
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  rosbag
  roscpp
  std_msgs
  topic_tools
)

find_package(Threads REQUIRED)

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

add_message_files(FILES
  KittingGripperCommand.msg
  KittingState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kitting_state_controller
  CATKIN_DEPENDS
    actionlib
    controller_interface
    franka_gripper
    franka_hw
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    rosbag
    roscpp
    std_msgs
    topic_tools
  DEPENDS Franka
)

## Controller library (loaded by ros_control)
add_library(kitting_state_controller
  src/kitting_state_controller.cpp
  src/kitting_gripper.cpp
  src/kitting_commands.cpp
  src/kitting_force_ramp.cpp
)

add_dependencies(kitting_state_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(kitting_state_controller
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  Threads::Threads
)

target_include_directories(kitting_state_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(kitting_state_controller PUBLIC
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../sms_cusum/cpp/include
)

## Logger executable (standalone node)
add_executable(kitting_logger
  src/kitting_logger.cpp
)

add_dependencies(kitting_logger
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(kitting_logger
  ${catkin_LIBRARIES}
)

target_include_directories(kitting_logger SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(kitting_logger PUBLIC
  include
)

## Tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(kitting_controller_test
    test/main.cpp
    test/kitting_unit_test.cpp
    test/kitting_state_machine_test.cpp
    test/kitting_integration_test.cpp
  )
  add_dependencies(kitting_controller_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
  )
  target_link_libraries(kitting_controller_test
    kitting_state_controller
    ${catkin_LIBRARIES}
    ${Franka_LIBRARIES}
    Threads::Threads
  )
  target_include_directories(kitting_controller_test SYSTEM PUBLIC
    ${Franka_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_include_directories(kitting_controller_test PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sms_cusum/cpp/include
  )

  # SMS-CUSUM C++ header-only library tests (no ROS dependency)
  catkin_add_gtest(sms_cusum_cpp_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../sms_cusum/cpp/test/test_sms_cusum_cpp.cpp
  )
  target_include_directories(sms_cusum_cpp_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../sms_cusum/cpp/include
  )
endif()

## Installation
install(TARGETS kitting_state_controller kitting_logger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES kitting_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
