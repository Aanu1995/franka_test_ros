// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_kitting_controller/kitting_state_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_kitting_controller {

bool KittingStateController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  // Load arm_id parameter
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("KittingStateController: Could not read parameter arm_id");
    return false;
  }

  // Load publish_rate parameter (default 250 Hz)
  double publish_rate(250.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("KittingStateController: publish_rate not found. Using default "
                    << publish_rate << " [Hz].");
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  // Acquire FrankaStateInterface
  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("KittingStateController: Could not get Franka state interface from hardware");
    return false;
  }

  // Acquire FrankaModelInterface
  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR("KittingStateController: Could not get Franka model interface from hardware");
    return false;
  }

  // Get state handle
  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "KittingStateController: Exception getting franka state handle: " << ex.what());
    return false;
  }

  // Get model handle
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "KittingStateController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // Initialize realtime publisher
  kitting_publisher_.init(node_handle, "kitting_state_data", 1);

  return true;
}

void KittingStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  if (rate_trigger_()) {
    // Read robot state
    franka::RobotState robot_state = franka_state_handle_->getRobotState();

    // Read model data
    std::array<double, 42> jacobian =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 7> gravity = model_handle_->getGravity();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();

    // Compute tau_ext_norm: Euclidean norm of tau_ext_hat_filtered
    double tau_ext_norm = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      tau_ext_norm += robot_state.tau_ext_hat_filtered[i] * robot_state.tau_ext_hat_filtered[i];
    }
    tau_ext_norm = std::sqrt(tau_ext_norm);

    // Compute wrench_norm: Euclidean norm of O_F_ext_hat_K
    double wrench_norm = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      wrench_norm += robot_state.O_F_ext_hat_K[i] * robot_state.O_F_ext_hat_K[i];
    }
    wrench_norm = std::sqrt(wrench_norm);

    // Compute ee_velocity = J * dq (6x7 column-major matrix times 7x1 vector)
    std::array<double, 6> ee_velocity{};
    for (size_t row = 0; row < 6; ++row) {
      for (size_t col = 0; col < 7; ++col) {
        ee_velocity[row] += jacobian[col * 6 + row] * robot_state.dq[col];
      }
    }

    // Publish via realtime-safe publisher
    if (kitting_publisher_.trylock()) {
      kitting_publisher_.msg_.header.stamp = time;

      // Joint-level signals
      for (size_t i = 0; i < 7; ++i) {
        kitting_publisher_.msg_.q[i] = robot_state.q[i];
        kitting_publisher_.msg_.dq[i] = robot_state.dq[i];
        kitting_publisher_.msg_.tau_J[i] = robot_state.tau_J[i];
        kitting_publisher_.msg_.tau_ext[i] = robot_state.tau_ext_hat_filtered[i];
      }

      // Cartesian-level signals
      for (size_t i = 0; i < 6; ++i) {
        kitting_publisher_.msg_.wrench_ext[i] = robot_state.O_F_ext_hat_K[i];
      }
      for (size_t i = 0; i < 16; ++i) {
        kitting_publisher_.msg_.O_T_EE[i] = robot_state.O_T_EE[i];
      }

      // Model-level signals
      for (size_t i = 0; i < 42; ++i) {
        kitting_publisher_.msg_.jacobian[i] = jacobian[i];
      }
      for (size_t i = 0; i < 7; ++i) {
        kitting_publisher_.msg_.gravity[i] = gravity[i];
        kitting_publisher_.msg_.coriolis[i] = coriolis[i];
      }

      // Derived quantities
      for (size_t i = 0; i < 6; ++i) {
        kitting_publisher_.msg_.ee_velocity[i] = ee_velocity[i];
      }
      kitting_publisher_.msg_.tau_ext_norm = tau_ext_norm;
      kitting_publisher_.msg_.wrench_norm = wrench_norm;

      kitting_publisher_.unlockAndPublish();
    }
  }
}

}  // namespace franka_kitting_controller

PLUGINLIB_EXPORT_CLASS(franka_kitting_controller::KittingStateController,
                       controller_interface::ControllerBase)
