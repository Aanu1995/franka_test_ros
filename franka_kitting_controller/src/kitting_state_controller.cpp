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
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_kitting_controller {

std::string KittingStateController::phaseToString(GraspPhase phase) {
  switch (phase) {
    case GraspPhase::START:
      return "START";
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

std::array<double, 16> KittingStateController::computeUpliftPose(double elapsed) const {
  std::array<double, 16> pose = uplift_start_pose_;
  // Uses RT-local copies (rt_uplift_duration_, rt_uplift_distance_) to prevent
  // mid-trajectory corruption from concurrent subscriber writes.
  double s_raw = std::min(elapsed / rt_uplift_duration_, 1.0);
  // Cosine smoothing: zero velocity at start and end
  double s = 0.5 * (1.0 - std::cos(M_PI * s_raw));
  // Index 14 is the z-translation in a column-major 4x4 homogeneous transform
  pose[14] = uplift_z_start_ + s * rt_uplift_distance_;
  return pose;
}

void KittingStateController::loggerReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
  // Only accept if the logger publisher is actually alive right now.
  // A stale latched message from a previous logger run will still trigger this
  // callback, but getNumPublishers() == 0 in that case because the original
  // publisher is gone — the ROS master replays the latched message, not the node.
  if (msg->data && logger_ready_sub_.getNumPublishers() > 0) {
    if (!logger_ready_.load(std::memory_order_relaxed)) {
      logger_ready_.store(true, std::memory_order_relaxed);
      ROS_INFO("============================================================");
      ROS_INFO("  [READY]  Phase 2 logger detected — commands now accepted");
      ROS_INFO("============================================================");
    }
  }
}

void KittingStateController::stateCallback(const std_msgs::String::ConstPtr& msg) {
  // Listens to /kitting_phase2/state for BASELINE from the user.
  // CLOSING, SECURE_GRASP, and UPLIFT are now handled via /kitting_phase2/state_cmd.
  // CONTACT/CLOSING/SECURE_GRASP/UPLIFT are published by the controller — ignore echo.
  //
  // This callback runs in the subscriber's spinner thread (non-RT).
  // Phase changes are applied in update() via pending_phase_ / phase_changed_.
  const std::string& label = msg->data;

  // --- Guard: reject commands if the Phase 2 logger is not running ---
  // Ignore controller's own echo publications (they don't need the logger guard).
  // Check both the flag AND that the logger publisher is currently alive
  // (getNumPublishers > 0). This catches the case where the logger was running
  // previously (setting logger_ready_ = true) but has since been shut down.
  if (label != "UPLIFT" && label != "CONTACT" && label != "CLOSING" &&
      label != "SECURE_GRASP") {
    if (!logger_ready_.load(std::memory_order_relaxed) ||
        logger_ready_sub_.getNumPublishers() == 0) {
      // Reset the flag if the logger went away
      logger_ready_.store(false, std::memory_order_relaxed);
      ROS_WARN("KittingStateController: '%s' rejected — Phase 2 logger not detected. "
               "Launch the logger first:\n"
               "  roslaunch franka_kitting_controller kitting_phase2.launch "
               "object_name:=<OBJ> base_directory:=<DIR>",
               label.c_str());
      return;
    }
  }

  if (label == "BASELINE") {
    pending_phase_.store(GraspPhase::BASELINE, std::memory_order_relaxed);
    phase_changed_.store(true, std::memory_order_release);  // release: publishes pending_phase_
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  BASELINE  <<  Collecting reference signals");
    ROS_INFO("============================================================");
  } else if (label == "UPLIFT" || label == "CONTACT" || label == "CLOSING" ||
             label == "SECURE_GRASP") {
    // Published by the controller itself or handled via state_cmd — ignore echo.
  }
}

void KittingStateController::stateCmdCallback(
    const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
  // Listens to /kitting_phase2/state_cmd for CLOSING, SECURE_GRASP, and UPLIFT
  // commands with optional per-object parameters.
  //
  // This callback runs in the subscriber's spinner thread (non-RT), so it is
  // safe to call sendGoal() on action clients here.
  //
  // Parameter override rule: any field set to 0.0 (the default for float64 in
  // ROS messages) falls back to the YAML config value loaded at init().
  // Non-zero values override the default for that single command.
  //
  // For each valid command:
  //   1. Ignore duplicate (if already in that state)
  //   2. Resolve per-command parameters (msg value if non-zero, else YAML default)
  //   3. Execute action if applicable (gripper or Cartesian motion)
  //   4. Set pending_phase_ + phase_changed_ for RT update() to apply
  //   5. Publish state label on /kitting_phase2/state
  const std::string& cmd = msg->command;

  // --- Guard: reject commands if the Phase 2 logger is not running ---
  // Check both the flag AND that the logger publisher is currently alive.
  if (!logger_ready_.load(std::memory_order_relaxed) ||
      logger_ready_sub_.getNumPublishers() == 0) {
    logger_ready_.store(false, std::memory_order_relaxed);
    ROS_WARN("KittingStateController: Command '%s' rejected — Phase 2 logger not detected. "
             "Launch the logger first:\n"
             "  roslaunch franka_kitting_controller kitting_phase2.launch "
             "object_name:=<OBJ> base_directory:=<DIR>",
             cmd.c_str());
    return;
  }

  // --- Guard: reject commands before BASELINE has been published ---
  if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::START) {
    ROS_WARN("KittingStateController: Command '%s' rejected — controller is in START state. "
             "Publish BASELINE on /kitting_phase2/state first to begin the grasp sequence.",
             cmd.c_str());
    return;
  }

  if (cmd == "CLOSING") {
    if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::CLOSING) {
      ROS_DEBUG("KittingStateController: Already in CLOSING, ignoring duplicate");
      return;
    }

    // Resolve per-command parameters (0 = use YAML default)
    double width = (msg->closing_width > 0.0) ? msg->closing_width : closing_width_;
    double speed = (msg->closing_speed > 0.0) ? msg->closing_speed : closing_speed_;

    // Execute gripper MoveAction (async, non-blocking)
    if (execute_gripper_actions_) {
      if (move_client_ && move_client_->isServerConnected()) {
        franka_gripper::MoveGoal goal;
        goal.width = width;
        goal.speed = speed;
        move_client_->sendGoal(goal);
        ROS_INFO("KittingStateController: Sent MoveAction (width=%.4f, speed=%.4f)",
                 width, speed);
      } else {
        ROS_ERROR("KittingStateController: MoveAction server not connected, "
                  "proceeding with state change only");
      }
    }

    pending_phase_.store(GraspPhase::CLOSING, std::memory_order_relaxed);
    phase_changed_.store(true, std::memory_order_release);
    publishStateLabel("CLOSING");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  CLOSING  <<  Gripper approaching object");
    ROS_INFO("    width=%.4f m  speed=%.4f m/s", width, speed);
    ROS_INFO("============================================================");

  } else if (cmd == "SECURE_GRASP") {
    if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::SECURE_GRASP) {
      ROS_DEBUG("KittingStateController: Already in SECURE_GRASP, ignoring duplicate");
      return;
    }

    // Resolve per-command parameters (0 = use YAML default)
    double width = (msg->grasp_width > 0.0) ? msg->grasp_width : grasp_width_;
    double eps_in = (msg->epsilon_inner > 0.0) ? msg->epsilon_inner : epsilon_inner_;
    double eps_out = (msg->epsilon_outer > 0.0) ? msg->epsilon_outer : epsilon_outer_;
    double speed = (msg->grasp_speed > 0.0) ? msg->grasp_speed : grasp_speed_;
    double force = (msg->grasp_force > 0.0) ? msg->grasp_force : grasp_force_;

    // Execute gripper GraspAction (async, non-blocking)
    if (execute_gripper_actions_) {
      if (grasp_client_ && grasp_client_->isServerConnected()) {
        franka_gripper::GraspGoal goal;
        goal.width = width;
        goal.epsilon.inner = eps_in;
        goal.epsilon.outer = eps_out;
        goal.speed = speed;
        goal.force = force;
        grasp_client_->sendGoal(goal);
        ROS_INFO("KittingStateController: Sent GraspAction "
                 "(width=%.4f, eps=%.4f/%.4f, speed=%.4f, force=%.1f)",
                 width, eps_in, eps_out, speed, force);
      } else {
        ROS_ERROR("KittingStateController: GraspAction server not connected, "
                  "proceeding with state change only");
      }
    }

    pending_phase_.store(GraspPhase::SECURE_GRASP, std::memory_order_relaxed);
    phase_changed_.store(true, std::memory_order_release);
    publishStateLabel("SECURE_GRASP");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  SECURE_GRASP  <<  Grasping object");
    ROS_INFO("    width=%.4f m  force=%.1f N  speed=%.4f m/s", width, force, speed);
    ROS_INFO("============================================================");

  } else if (cmd == "UPLIFT") {
    // --- Precondition: ignore if already executing UPLIFT ---
    if (uplift_active_.load(std::memory_order_relaxed)) {
      ROS_DEBUG("KittingStateController: UPLIFT already active, ignoring duplicate");
      return;
    }

    // --- Precondition: require SECURE_GRASP state if configured ---
    if (require_secure_grasp_ &&
        current_phase_.load(std::memory_order_relaxed) != GraspPhase::SECURE_GRASP) {
      ROS_WARN("KittingStateController: UPLIFT rejected — require_secure_grasp is true "
               "but current state is %s (expected SECURE_GRASP)",
               phaseToString(current_phase_.load(std::memory_order_relaxed)).c_str());
      return;
    }

    // --- Resolve per-command parameters (0 = use YAML default) ---
    double distance = (msg->uplift_distance > 0.0) ? msg->uplift_distance : uplift_distance_;
    double duration = (msg->uplift_duration > 0.0) ? msg->uplift_duration : uplift_duration_;

    // --- Safety: clamp distance ---
    if (distance > kMaxUpliftDistance) {
      ROS_WARN("KittingStateController: uplift_distance %.4f exceeds max %.4f, clamping",
               distance, kMaxUpliftDistance);
      distance = kMaxUpliftDistance;
    }
    if (distance <= 0.0) {
      ROS_WARN("KittingStateController: uplift_distance must be positive, ignoring");
      return;
    }
    if (duration <= 0.0) {
      ROS_WARN("KittingStateController: uplift_duration must be positive, ignoring");
      return;
    }

    // --- Store resolved parameters for RT update loop ---
    // Written here (subscriber thread), read by RT thread in the phase_changed_ block.
    // Safety: the release-store on phase_changed_ below creates a happens-before edge,
    // guaranteeing that when the RT thread's acquire-load of phase_changed_ returns true,
    // it sees these stores. The RT thread then copies them to rt_uplift_distance_ and
    // rt_uplift_duration_ for trajectory isolation.
    uplift_distance_ = distance;
    uplift_duration_ = duration;

    pending_phase_.store(GraspPhase::UPLIFT, std::memory_order_relaxed);
    phase_changed_.store(true, std::memory_order_release);
    publishStateLabel("UPLIFT");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  UPLIFT  <<  Cartesian micro-lift");
    ROS_INFO("    distance=%.4f m  duration=%.2f s", distance, duration);
    ROS_INFO("============================================================");

  } else {
    ROS_WARN("KittingStateController: Unknown command '%s' on state_cmd "
             "(expected CLOSING, SECURE_GRASP, or UPLIFT)", cmd.c_str());
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
  node_handle.param("delta_min", delta_min_, 0.5);
  node_handle.param("T_hold", T_hold_, 0.12);
  node_handle.param("use_slope_gate", use_slope_gate_, false);
  node_handle.param("slope_dt", slope_dt_, 0.02);
  node_handle.param("slope_min", slope_min_, 5.0);

  ROS_INFO_STREAM("KittingStateController: Contact detector "
                   << (enable_contact_detector_ ? "ENABLED" : "DISABLED")
                   << " | T_base=" << T_base_ << " N_min=" << N_min_ << " k_sigma=" << k_sigma_
                   << " delta_min=" << delta_min_ << " T_hold=" << T_hold_);

  // --- Phase 2: Gripper default parameters (overridable per-command via KittingGripperCommand) ---
  node_handle.param("execute_gripper_actions", execute_gripper_actions_, true);
  node_handle.param("closing_width", closing_width_, 0.04);
  node_handle.param("closing_speed", closing_speed_, 0.04);
  node_handle.param("grasp_width", grasp_width_, 0.02);
  node_handle.param("epsilon_inner", epsilon_inner_, 0.005);
  node_handle.param("epsilon_outer", epsilon_outer_, 0.005);
  node_handle.param("grasp_speed", grasp_speed_, 0.04);
  node_handle.param("grasp_force", grasp_force_, 10.0);

  ROS_INFO_STREAM("KittingStateController: Gripper execution "
                   << (execute_gripper_actions_ ? "ENABLED" : "DISABLED (signal-only)")
                   << " | closing: width=" << closing_width_ << " speed=" << closing_speed_
                   << " | grasp: width=" << grasp_width_ << " eps=" << epsilon_inner_
                   << "/" << epsilon_outer_ << " speed=" << grasp_speed_
                   << " force=" << grasp_force_);

  // --- Phase 2: UPLIFT parameters (overridable per-command via KittingGripperCommand) ---
  node_handle.param("uplift_distance", uplift_distance_, 0.003);
  node_handle.param("uplift_duration", uplift_duration_, 1.0);
  node_handle.param("uplift_reference_frame", uplift_reference_frame_, std::string("world"));
  node_handle.param("require_secure_grasp", require_secure_grasp_, true);

  if (uplift_distance_ > kMaxUpliftDistance) {
    ROS_WARN("KittingStateController: uplift_distance %.4f exceeds max %.4f, clamping",
             uplift_distance_, kMaxUpliftDistance);
    uplift_distance_ = kMaxUpliftDistance;
  }
  if (uplift_distance_ <= 0.0) {
    ROS_ERROR("KittingStateController: uplift_distance must be positive (got %.4f)",
              uplift_distance_);
    return false;
  }
  if (uplift_duration_ <= 0.0) {
    ROS_ERROR("KittingStateController: uplift_duration must be positive (got %.4f)",
              uplift_duration_);
    return false;
  }

  ROS_INFO_STREAM("KittingStateController: UPLIFT config | distance=" << uplift_distance_
                   << "m duration=" << uplift_duration_ << "s frame=" << uplift_reference_frame_
                   << " require_secure_grasp=" << (require_secure_grasp_ ? "true" : "false"));

  // --- Phase 2: Gripper action clients (spin_thread=true for plugin context) ---
  if (execute_gripper_actions_) {
    move_client_ = std::make_unique<MoveClient>("/franka_gripper/move", true);
    grasp_client_ = std::make_unique<GraspClient>("/franka_gripper/grasp", true);

    if (move_client_->waitForServer(ros::Duration(2.0))) {
      ROS_INFO("KittingStateController: MoveAction server connected");
    } else {
      ROS_WARN("KittingStateController: MoveAction server not yet available "
               "(will retry on command)");
    }
    if (grasp_client_->waitForServer(ros::Duration(2.0))) {
      ROS_INFO("KittingStateController: GraspAction server connected");
    } else {
      ROS_WARN("KittingStateController: GraspAction server not yet available "
               "(will retry on command)");
    }
  }

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

  // Acquire FrankaPoseCartesianInterface (for UPLIFT micro-lift)
  cartesian_pose_interface_ = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR("KittingStateController: Could not get Cartesian pose interface from hardware");
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

  // Get Cartesian pose handle
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "KittingStateController: Exception getting Cartesian pose handle: " << ex.what());
    return false;
  }

  // Initialize realtime publishers
  kitting_publisher_.init(node_handle, "kitting_state_data", 1);

  // Phase 2: State label publisher
  ros::NodeHandle root_nh;
  state_publisher_.init(root_nh, "/kitting_phase2/state", 1);

  // Phase 2: Logger readiness subscriber (latched topic from logger node).
  // Commands are rejected until this signal is received.
  logger_ready_sub_ = root_nh.subscribe("/kitting_phase2/logger_ready", 1,
                                         &KittingStateController::loggerReadyCallback, this);

  // Phase 2: State subscriber — user publishes BASELINE here.
  // CLOSING/SECURE_GRASP/UPLIFT/CONTACT echoes are ignored.
  state_sub_ = root_nh.subscribe("/kitting_phase2/state", 10,
                                  &KittingStateController::stateCallback, this);

  // Phase 2: Command subscriber — user publishes KittingGripperCommand here
  // with command (CLOSING/SECURE_GRASP/UPLIFT) + optional per-object parameters.
  state_cmd_sub_ = root_nh.subscribe("/kitting_phase2/state_cmd", 10,
                                      &KittingStateController::stateCmdCallback, this);

  return true;
}

void KittingStateController::starting(const ros::Time& /* time */) {
  // Capture the current desired pose and send it as the first command.
  // O_T_EE_d is the robot's last commanded pose — using this avoids any step discontinuity.
  uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  cartesian_pose_handle_->setCommand(uplift_start_pose_);  // Defensive: no uninitialized gap
  uplift_active_.store(false, std::memory_order_relaxed);
  uplift_elapsed_ = 0.0;

  ROS_INFO("============================================================");
  ROS_INFO("  [STATE]  >>  START  <<  Controller running");
  ROS_INFO("    Step 1: Launch Phase 2 logger (kitting_phase2.launch)");
  ROS_INFO("    Step 2: Publish BASELINE on /kitting_phase2/state");
  ROS_INFO("============================================================");
}

void KittingStateController::update(const ros::Time& time, const ros::Duration& period) {
  // ====================================================================
  // 1. Apply pending state transition from subscriber (every tick)
  // ====================================================================
  if (phase_changed_.load(std::memory_order_acquire)) {
    // acquire: guarantees visibility of all stores preceding the subscriber's
    // release-store of phase_changed_ (including uplift_distance_, uplift_duration_,
    // and pending_phase_).
    GraspPhase new_phase = pending_phase_.load(std::memory_order_relaxed);

    // Handle BASELINE reset: clear all baseline stats and UPLIFT state
    if (new_phase == GraspPhase::BASELINE &&
        current_phase_.load(std::memory_order_relaxed) != GraspPhase::BASELINE) {
      if (uplift_active_.load(std::memory_order_relaxed)) {
        ROS_WARN("KittingStateController: BASELINE received while UPLIFT active — "
                 "aborting trajectory (robot will hold current position)");
      }
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
      uplift_active_.store(false, std::memory_order_relaxed);
      uplift_elapsed_ = 0.0;
    }

    // Handle UPLIFT start: capture current desired pose, snapshot params, reset timer
    if (new_phase == GraspPhase::UPLIFT &&
        !uplift_active_.load(std::memory_order_relaxed)) {
      uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      uplift_z_start_ = uplift_start_pose_[14];
      uplift_elapsed_ = 0.0;
      // Snapshot parameters into RT-local copies — isolates trajectory from
      // any concurrent subscriber writes to uplift_distance_/uplift_duration_.
      rt_uplift_distance_ = uplift_distance_;
      rt_uplift_duration_ = uplift_duration_;
      uplift_active_.store(true, std::memory_order_relaxed);
    }

    current_phase_.store(new_phase, std::memory_order_relaxed);
    // State label was published by stateCmdCallback (CLOSING, SECURE_GRASP, UPLIFT),
    // by the user (BASELINE), or by the contact detector (CONTACT).
    phase_changed_.store(false, std::memory_order_relaxed);
  }

  // ====================================================================
  // 2. Cartesian pose command — EVERY TICK (1kHz)
  // ====================================================================
  if (uplift_active_.load(std::memory_order_relaxed)) {
    uplift_elapsed_ += period.toSec();
    std::array<double, 16> desired_pose = computeUpliftPose(uplift_elapsed_);
    cartesian_pose_handle_->setCommand(desired_pose);

    // Check if trajectory is complete (uses RT-local copy)
    if (uplift_elapsed_ >= rt_uplift_duration_) {
      uplift_active_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: UPLIFT trajectory complete (%.3fs, %.4fm)",
               rt_uplift_duration_, rt_uplift_distance_);
    }
  } else {
    // Passthrough: send current desired pose back as command (hold position).
    // O_T_EE_d is the robot's own desired pose — zero error, no motion.
    std::array<double, 16> current_desired = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    cartesian_pose_handle_->setCommand(current_desired);
  }

  // ====================================================================
  // 3. Rate-triggered: contact detection + KittingState publishing (250Hz)
  // ====================================================================
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
      if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::BASELINE) {
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
      if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::CLOSING &&
          baseline_armed_ && !contact_latched_) {
        // Two conditions required:
        //   1. Statistical exceedance: x(t) > theta
        //   2. Minimum absolute rise:  x(t) - mu > delta_min  (noise guard)
        // Without delta_min, a tiny sigma makes theta ≈ mu, and normal
        // noise fluctuations trigger false contact.
        double delta = tau_ext_norm - baseline_mu_;
        bool threshold_exceeded = (tau_ext_norm > contact_threshold_) &&
                                  (delta > delta_min_);

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
              // Use direct store since we are already in the RT thread.
              // No need for the phase_changed_ mechanism (which is for subscriber->RT handoff).
              // CONTACT has no special initialization, so direct assignment is safe.
              current_phase_.store(GraspPhase::CONTACT, std::memory_order_relaxed);
              publishStateLabel("CONTACT");
              ROS_INFO("============================================================");
              ROS_INFO("  [STATE]  >>  CONTACT  <<  Object contact detected!");
              ROS_INFO("    x(t)=%.4f  theta=%.4f  delta=%.4f  hold=%.3fs",
                       tau_ext_norm, contact_threshold_,
                       tau_ext_norm - baseline_mu_, hold_elapsed);
              ROS_INFO("============================================================");
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
    // Signal monitor — log x(t) and θ at 2 Hz for terminal readability
    // ====================================================================
    if (signal_log_trigger_() && baseline_armed_) {
      GraspPhase phase = current_phase_.load(std::memory_order_relaxed);
      if (phase == GraspPhase::CLOSING) {
        double margin = tau_ext_norm - contact_threshold_;
        double sig_delta = tau_ext_norm - baseline_mu_;
        ROS_INFO("  [SIGNAL]  CLOSING  |  x(t)=%.4f  theta=%.4f  margin=%.4f  delta=%.4f  %s",
                 tau_ext_norm, contact_threshold_, margin, sig_delta,
                 (margin > 0.0 && sig_delta > delta_min_) ? "ABOVE" : "below");
      } else if (phase == GraspPhase::CONTACT ||
                 phase == GraspPhase::SECURE_GRASP ||
                 phase == GraspPhase::UPLIFT) {
        ROS_INFO("  [SIGNAL]  %s  |  x(t)=%.4f  theta=%.4f  margin=+%.4f",
                 phaseToString(phase).c_str(),
                 tau_ext_norm, contact_threshold_,
                 tau_ext_norm - contact_threshold_);
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
