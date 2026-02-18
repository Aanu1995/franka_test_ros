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

std::string KittingStateController::phaseToString(GraspPhase phase) {
  switch (phase) {
    case GraspPhase::BASELINE:
      return "BASELINE";
    case GraspPhase::CLOSING:
      return "CLOSING";
    case GraspPhase::CONTACT:
      return "CONTACT";
    case GraspPhase::SECURE_GRASP:
      return "SECURE_GRASP";
    case GraspPhase::UPLIFT:
      return "UPLIFT";
    default:
      return "UNKNOWN";
  }
}

void KittingStateController::publishStateLabel(const std::string& label) {
  if (state_publisher_.trylock()) {
    state_publisher_.msg_.data = label;
    state_publisher_.unlockAndPublish();
  }
}

void KittingStateController::stateCallback(const std_msgs::String::ConstPtr& msg) {
  // Listens to /kitting_phase2/state for phase transitions.
  // The user publishes state labels (BASELINE, CLOSING, SECURE_GRASP, UPLIFT) directly.
  // The controller publishes CONTACT on this same topic when auto-detected.
  //
  // This callback runs in the subscriber's spinner thread (non-RT).
  // Phase changes are applied in update() via pending_phase_ / phase_changed_.
  const std::string& label = msg->data;

  if (label == "BASELINE") {
    pending_phase_ = GraspPhase::BASELINE;
    phase_changed_ = true;
  } else if (label == "CLOSING") {
    pending_phase_ = GraspPhase::CLOSING;
    phase_changed_ = true;
  } else if (label == "CONTACT") {
    // CONTACT is published by the controller itself — ignore our own message.
  } else if (label == "SECURE_GRASP") {
    pending_phase_ = GraspPhase::SECURE_GRASP;
    phase_changed_ = true;
  } else if (label == "UPLIFT") {
    pending_phase_ = GraspPhase::UPLIFT;
    phase_changed_ = true;
  }
}

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

  // --- Phase 2: Load contact detector parameters ---
  node_handle.param("enable_contact_detector", enable_contact_detector_, true);
  node_handle.param("T_base", T_base_, 0.7);
  node_handle.param("N_min", N_min_, 50);
  node_handle.param("k_sigma", k_sigma_, 5.0);
  node_handle.param("T_hold", T_hold_, 0.12);
  node_handle.param("use_slope_gate", use_slope_gate_, false);
  node_handle.param("slope_dt", slope_dt_, 0.02);
  node_handle.param("slope_min", slope_min_, 5.0);

  ROS_INFO_STREAM("KittingStateController: Contact detector "
                   << (enable_contact_detector_ ? "ENABLED" : "DISABLED")
                   << " | T_base=" << T_base_ << " N_min=" << N_min_ << " k_sigma=" << k_sigma_
                   << " T_hold=" << T_hold_);

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

  // Initialize realtime publishers
  kitting_publisher_.init(node_handle, "kitting_state_data", 1);

  // Phase 2: State label publisher (controller publishes CONTACT only)
  ros::NodeHandle root_nh;
  state_publisher_.init(root_nh, "/kitting_phase2/state", 1);

  // Phase 2: State subscriber — user publishes state labels directly,
  // controller reads them here to update its phase.
  state_sub_ = root_nh.subscribe("/kitting_phase2/state", 10,
                                  &KittingStateController::stateCallback, this);

  return true;
}

void KittingStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  // --- Phase 2: Apply pending state transition from subscriber ---
  if (phase_changed_) {
    GraspPhase new_phase = pending_phase_;

    // Handle BASELINE reset: clear all baseline stats
    if (new_phase == GraspPhase::BASELINE && current_phase_ != GraspPhase::BASELINE) {
      baseline_sum_ = 0.0;
      baseline_sum_sq_ = 0.0;
      baseline_n_ = 0;
      baseline_collecting_ = false;
      baseline_armed_ = false;
      baseline_mu_ = 0.0;
      baseline_sigma_ = 0.0;
      contact_threshold_ = 0.0;
      contact_latched_ = false;
      exceeding_ = false;
      prev_tau_ext_valid_ = false;
    }

    current_phase_ = new_phase;
    // State label was published by the user (or by controller for CONTACT).
    // No need to re-publish here.
    phase_changed_ = false;
  }

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

    // ====================================================================
    // Phase 2: Contact detection state machine
    // ====================================================================
    if (enable_contact_detector_) {
      // --- BASELINE: collect samples ---
      if (current_phase_ == GraspPhase::BASELINE) {
        if (!baseline_collecting_) {
          baseline_collecting_ = true;
          baseline_start_time_ = time;
          baseline_sum_ = 0.0;
          baseline_sum_sq_ = 0.0;
          baseline_n_ = 0;
        }

        double elapsed = (time - baseline_start_time_).toSec();
        if (!baseline_armed_) {
          // Keep collecting until BOTH T_base elapsed AND N_min reached.
          // This prevents a silent failure when N_min > publish_rate * T_base.
          baseline_sum_ += tau_ext_norm;
          baseline_sum_sq_ += tau_ext_norm * tau_ext_norm;
          baseline_n_++;

          if (elapsed >= T_base_ && baseline_n_ >= N_min_) {
            // Compute baseline statistics
            baseline_mu_ = baseline_sum_ / baseline_n_;
            double variance =
                (baseline_sum_sq_ - baseline_sum_ * baseline_sum_ / baseline_n_) / (baseline_n_ - 1);
            baseline_sigma_ = std::sqrt(std::max(variance, 0.0));
            contact_threshold_ = baseline_mu_ + k_sigma_ * baseline_sigma_;
            baseline_armed_ = true;
            ROS_INFO_STREAM("KittingStateController: Baseline computed | N=" << baseline_n_
                            << " mu=" << baseline_mu_ << " sigma=" << baseline_sigma_
                            << " theta=" << contact_threshold_
                            << " elapsed=" << elapsed << "s");
          }
        }
      }

      // --- CLOSING: detect contact ---
      if (current_phase_ == GraspPhase::CLOSING && baseline_armed_ && !contact_latched_) {
        bool threshold_exceeded = (tau_ext_norm > contact_threshold_);

        // Optional slope gate
        if (threshold_exceeded && use_slope_gate_) {
          if (prev_tau_ext_valid_) {
            double dt = (time - prev_tau_ext_time_).toSec();
            if (dt > 1e-6) {
              double slope = (tau_ext_norm - prev_tau_ext_norm_) / dt;
              if (slope < slope_min_) {
                threshold_exceeded = false;
              }
            }
          } else {
            // No previous sample yet, cannot compute slope
            threshold_exceeded = false;
          }
        }

        // Debounce: require sustained exceedance for T_hold seconds
        if (threshold_exceeded) {
          if (!exceeding_) {
            exceeding_ = true;
            exceed_start_time_ = time;
          } else {
            double hold_elapsed = (time - exceed_start_time_).toSec();
            if (hold_elapsed >= T_hold_) {
              // CONTACT detected!
              contact_latched_ = true;
              current_phase_ = GraspPhase::CONTACT;
              pending_phase_ = GraspPhase::CONTACT;
              publishStateLabel("CONTACT");
              ROS_INFO_STREAM("KittingStateController: CONTACT detected | tau_ext_norm="
                              << tau_ext_norm << " threshold=" << contact_threshold_
                              << " hold_time=" << hold_elapsed);
            }
          }
        } else {
          // Reset debounce
          exceeding_ = false;
        }

        // Update slope gate history
        prev_tau_ext_norm_ = tau_ext_norm;
        prev_tau_ext_time_ = time;
        prev_tau_ext_valid_ = true;
      }
    }

    // ====================================================================
    // Publish KittingState message (Phase 1 data)
    // ====================================================================
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
