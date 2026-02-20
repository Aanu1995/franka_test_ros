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

#include <sensor_msgs/JointState.h>
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

void KittingStateController::gripperJointStateCallback(
    const sensor_msgs::JointState::ConstPtr& msg) {
  // /franka_gripper/joint_states publishes panda_finger_joint1 and panda_finger_joint2.
  // Each finger position = width * 0.5, so w(t) = position[0] + position[1].
  if (msg->position.size() >= 2) {
    double width = msg->position[0] + msg->position[1];
    gripper_width_.store(width, std::memory_order_relaxed);
  }
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

    // Store commanded width for gripper stall detection
    gripper_cmd_width_ = width;

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

  // --- Phase 2: Load hybrid contact detector parameters ---
  // Baseline
  node_handle.param("T_base", T_base_, 0.7);
  node_handle.param("N_min", N_min_, 50);

  // Arm-based detector
  node_handle.param("enable_arm_contact", enable_arm_contact_, true);
  node_handle.param("k_sigma", k_sigma_, 3.0);
  node_handle.param("delta_min", delta_min_, 0.3);
  node_handle.param("T_arm_hold", T_arm_hold_, 0.10);

  // Gripper-based detector
  node_handle.param("enable_gripper_contact", enable_gripper_contact_, true);
  node_handle.param("v_stall", v_stall_, 0.003);
  node_handle.param("epsilon_w", epsilon_w_, 0.002);
  node_handle.param("w_min", w_min_, 0.002);
  node_handle.param("T_gripper_hold", T_gripper_hold_, 0.10);

  if (!enable_arm_contact_ && !enable_gripper_contact_) {
    ROS_FATAL("KittingStateController: BOTH arm and gripper contact detectors are disabled! "
              "At least one must be enabled for CONTACT detection.");
    return false;
  }

  ROS_INFO("KittingStateController: Hybrid contact detection");
  ROS_INFO("  Arm detector:     %s | k_sigma=%.1f delta_min=%.2f T_arm_hold=%.3fs",
           enable_arm_contact_ ? "ENABLED" : "DISABLED",
           k_sigma_, delta_min_, T_arm_hold_);
  ROS_INFO("  Gripper detector: %s | v_stall=%.4f epsilon_w=%.4f w_min=%.4f T_gripper_hold=%.3fs",
           enable_gripper_contact_ ? "ENABLED" : "DISABLED",
           v_stall_, epsilon_w_, w_min_, T_gripper_hold_);
  ROS_INFO("  Baseline: T_base=%.2fs N_min=%d", T_base_, N_min_);

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

  // --- Phase 2: Automatic gripper stop on CONTACT ---
  node_handle.param("stop_on_contact", stop_on_contact_, true);
  ROS_INFO("KittingStateController: stop_on_contact=%s",
           stop_on_contact_ ? "true" : "false");

  // Non-RT timer checks gripper_stop_pending_ flag set by the RT thread.
  // 100 Hz polling (10ms) gives sub-frame latency for the stop command.
  {
    ros::NodeHandle timer_nh;  // global handle for the timer callback queue
    gripper_stop_timer_ = timer_nh.createTimer(
        ros::Duration(0.01),
        &KittingStateController::gripperStopTimerCallback, this);
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

  // Phase 2: Gripper joint state subscriber (for gripper-based contact detection).
  // /franka_gripper/joint_states publishes panda_finger_joint1 + panda_finger_joint2.
  if (enable_gripper_contact_) {
    gripper_joint_state_sub_ = root_nh.subscribe(
        "/franka_gripper/joint_states", 10,
        &KittingStateController::gripperJointStateCallback, this);
    ROS_INFO("KittingStateController: Subscribed to /franka_gripper/joint_states");
  }

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

void KittingStateController::gripperStopTimerCallback(const ros::TimerEvent& /*event*/) {
  // Non-RT callback: checks if the RT thread flagged a gripper stop on CONTACT.
  // Runs at 100 Hz; the flag is set at most once per CLOSING→CONTACT transition.
  if (!gripper_stop_pending_.load(std::memory_order_acquire)) {
    return;
  }
  gripper_stop_pending_.store(false, std::memory_order_relaxed);

  double w_contact = contact_width_.load(std::memory_order_relaxed);

  // 1. Cancel any active gripper action (MoveAction or GraspAction).
  //    cancelGoal() is non-blocking — it publishes a cancel message on the action topic.
  if (move_client_ && move_client_->isServerConnected()) {
    move_client_->cancelAllGoals();
  }
  if (grasp_client_ && grasp_client_->isServerConnected()) {
    grasp_client_->cancelAllGoals();
  }

  // 2. Hold current width: send a MoveAction to the contact width at low speed.
  //    This prevents the gripper from drifting after cancellation.
  if (move_client_ && move_client_->isServerConnected()) {
    franka_gripper::MoveGoal hold_goal;
    hold_goal.width = w_contact;
    hold_goal.speed = 0.01;  // Low hold speed [m/s]
    move_client_->sendGoal(hold_goal);
  }

  ROS_INFO("============================================================");
  ROS_INFO("  [CONTACT]  Gripper stopped at width=%.4f m", w_contact);
  ROS_INFO("    Cancelled active action, holding current position");
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
      gripper_stop_pending_.store(false, std::memory_order_relaxed);
      // Reset arm detector debounce
      arm_exceeding_ = false;
      // Reset gripper detector debounce
      gripper_exceeding_ = false;
      prev_gripper_width_valid_ = false;
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
    // Phase 2: Hybrid Contact Detection (Gripper + Arm Fusion)
    //
    //   CONTACT = GripperContact OR ArmContact
    //
    // Gripper: stall before reaching commanded width (fingers blocked)
    // Arm:     tau_ext_norm > theta AND (tau_ext_norm - mu) > delta_min
    // Each has independent debounce. CONTACT is latched once declared.
    // ====================================================================

    // Read gripper width from subscriber (atomic, non-blocking)
    double gripper_w = gripper_width_.load(std::memory_order_relaxed);

    // Contact detection diagnostics (populated during CLOSING, published every tick)
    double diag_arm_margin = 0.0;       // tau_ext_norm - theta
    double diag_arm_delta = 0.0;        // tau_ext_norm - mu
    double diag_gripper_velocity = 0.0; // finite-difference dw/dt
    bool diag_arm_active = false;       // arm detector condition met this tick
    bool diag_gripper_active = false;   // gripper stall condition met this tick

    // --- BASELINE: collect arm signal samples ---
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
        baseline_sum_ += tau_ext_norm;
        baseline_sum_sq_ += tau_ext_norm * tau_ext_norm;
        baseline_n_++;

        if (elapsed >= T_base_ && baseline_n_ >= N_min_) {
          baseline_mu_ = baseline_sum_ / baseline_n_;
          double variance =
              (baseline_sum_sq_ - baseline_sum_ * baseline_sum_ / baseline_n_) / (baseline_n_ - 1);
          baseline_sigma_ = std::sqrt(std::max(variance, 0.0));
          contact_threshold_ = baseline_mu_ + k_sigma_ * baseline_sigma_;
          baseline_armed_ = true;
          ROS_INFO("============================================================");
          ROS_INFO("  [BASELINE]  Statistics computed");
          ROS_INFO("    N=%d samples over %.3fs", baseline_n_, elapsed);
          ROS_INFO("    mu=%.4f Nm  (noise floor of ||tau_ext||)", baseline_mu_);
          ROS_INFO("    sigma=%.4f Nm  (noise spread)", baseline_sigma_);
          ROS_INFO("    k=%.1f  delta_min=%.2f Nm", k_sigma_, delta_min_);
          ROS_INFO("    theta=%.4f Nm  (contact threshold = mu + k*sigma)", contact_threshold_);
          ROS_INFO("    Current x(t)=%.4f Nm  (should be < theta)", tau_ext_norm);
          ROS_INFO("============================================================");
        }
      }
    }

    // --- CLOSING: hybrid contact detection ---
    if (current_phase_.load(std::memory_order_relaxed) == GraspPhase::CLOSING &&
        baseline_armed_ && !contact_latched_) {

      bool arm_contact = false;
      bool gripper_contact = false;
      std::string trigger_source;

      // ---- Arm-based detector ----
      diag_arm_margin = tau_ext_norm - contact_threshold_;
      diag_arm_delta = tau_ext_norm - baseline_mu_;

      if (enable_arm_contact_) {
        bool arm_triggered = (tau_ext_norm > contact_threshold_) &&
                             ((tau_ext_norm - baseline_mu_) > delta_min_);
        diag_arm_active = arm_triggered;

        if (arm_triggered) {
          if (!arm_exceeding_) {
            arm_exceeding_ = true;
            arm_exceed_start_time_ = time;
          } else {
            double hold = (time - arm_exceed_start_time_).toSec();
            if (hold >= T_arm_hold_) {
              arm_contact = true;
              trigger_source = "ARM";
            }
          }
        } else {
          arm_exceeding_ = false;
        }
      }

      // ---- Gripper-based detector ----
      {
        // Always compute gripper velocity for diagnostics, even if detector disabled
        double dt = (1.0 / 250.0);  // publish rate period
        if (prev_gripper_width_valid_) {
          diag_gripper_velocity = (gripper_w - prev_gripper_width_) / dt;
        }
      }

      if (enable_gripper_contact_ && !arm_contact) {
        double w_dot = diag_gripper_velocity;

        // 1) Stall: fingers stopped moving
        bool stalled = (std::abs(w_dot) < v_stall_) && prev_gripper_width_valid_;
        // 2) Directional gap: gripper stopped BEFORE reaching target (w > w_cmd)
        //    Object blocked the fingers — measured width exceeds commanded width.
        //    Using signed difference (NOT absolute value) prevents false contact
        //    when the gripper reaches its target normally (w ≈ w_cmd → gap ≈ 0).
        bool blocked_early = (gripper_w - gripper_cmd_width_) > epsilon_w_;
        // 3) Not fully closed: avoid false contact at mechanical limit
        bool above_min = (gripper_w > w_min_);
        bool gripper_triggered = stalled && blocked_early && above_min;
        diag_gripper_active = gripper_triggered;

        if (gripper_triggered) {
          if (!gripper_exceeding_) {
            gripper_exceeding_ = true;
            gripper_exceed_start_time_ = time;
          } else {
            double hold = (time - gripper_exceed_start_time_).toSec();
            if (hold >= T_gripper_hold_) {
              gripper_contact = true;
              trigger_source = "GRIPPER";
            }
          }
        } else {
          gripper_exceeding_ = false;
        }
      }

      // Always update prev_gripper_width_ so velocity computation is correct
      // on the next tick, regardless of whether the gripper detector ran.
      prev_gripper_width_ = gripper_w;
      prev_gripper_width_valid_ = true;

      // ---- Fusion: OR ----
      if (arm_contact || gripper_contact) {
        contact_latched_ = true;
        current_phase_.store(GraspPhase::CONTACT, std::memory_order_relaxed);
        publishStateLabel("CONTACT");

        // Signal the non-RT timer to stop the gripper (cancel + hold)
        if (stop_on_contact_ && execute_gripper_actions_) {
          contact_width_.store(gripper_w, std::memory_order_relaxed);
          gripper_stop_pending_.store(true, std::memory_order_release);
        }

        ROS_INFO("============================================================");
        ROS_INFO("  [STATE]  >>  CONTACT  <<  Object contact detected!");
        ROS_INFO("    Triggered by: %s detector", trigger_source.c_str());
        ROS_INFO("    tau_ext_norm=%.4f  theta=%.4f  delta=%.4f",
                 tau_ext_norm, contact_threshold_, tau_ext_norm - baseline_mu_);
        ROS_INFO("    gripper_w=%.4f  w_cmd=%.4f  gap=%.4f",
                 gripper_w, gripper_cmd_width_,
                 gripper_w - gripper_cmd_width_);
        ROS_INFO("    stop_on_contact=%s", stop_on_contact_ ? "true" : "false");
        ROS_INFO("============================================================");
      }
    }

    // ====================================================================
    // Signal monitor — log at 2 Hz for terminal readability
    // ====================================================================
    if (signal_log_trigger_()) {
      GraspPhase phase = current_phase_.load(std::memory_order_relaxed);
      if (phase == GraspPhase::BASELINE && baseline_collecting_ && !baseline_armed_) {
        double running_mu = (baseline_n_ > 0) ? baseline_sum_ / baseline_n_ : 0.0;
        ROS_INFO("  [SIGNAL]  BASELINE  |  x(t)=%.4f  running_mu=%.4f  N=%d",
                 tau_ext_norm, running_mu, baseline_n_);
      } else if (phase == GraspPhase::CLOSING && baseline_armed_) {
        double arm_margin = tau_ext_norm - contact_threshold_;
        double arm_delta = tau_ext_norm - baseline_mu_;
        double gripper_gap = gripper_w - gripper_cmd_width_;  // directional: >0 means blocked early
        ROS_INFO("  [SIGNAL]  CLOSING  |  x(t)=%.4f  θ=%.4f  margin=%.4f  delta=%.4f  "
                 "w=%.4f  w_cmd=%.4f  gap=%.4f  ẇ=%.4f  %s  %s",
                 tau_ext_norm, contact_threshold_, arm_margin, arm_delta,
                 gripper_w, gripper_cmd_width_, gripper_gap, diag_gripper_velocity,
                 (arm_margin > 0.0 && arm_delta > delta_min_) ? "ARM_ABOVE" : "arm_below",
                 (gripper_gap > epsilon_w_ && std::abs(diag_gripper_velocity) < v_stall_)
                     ? "GRIP_STALL" : "grip_moving");
      } else if (baseline_armed_ &&
                 (phase == GraspPhase::CONTACT ||
                  phase == GraspPhase::SECURE_GRASP ||
                  phase == GraspPhase::UPLIFT)) {
        ROS_INFO("  [SIGNAL]  %s  |  x(t)=%.4f  θ=%.4f  w=%.4f",
                 phaseToString(phase).c_str(),
                 tau_ext_norm, contact_threshold_, gripper_w);
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
      kitting_publisher_.msg_.gripper_width = gripper_w;

      // Contact detection diagnostics
      kitting_publisher_.msg_.baseline_mu = baseline_mu_;
      kitting_publisher_.msg_.baseline_sigma = baseline_sigma_;
      kitting_publisher_.msg_.contact_threshold = contact_threshold_;
      kitting_publisher_.msg_.arm_margin = diag_arm_margin;
      kitting_publisher_.msg_.arm_delta = diag_arm_delta;
      kitting_publisher_.msg_.gripper_cmd_width = gripper_cmd_width_;
      kitting_publisher_.msg_.gripper_velocity = diag_gripper_velocity;
      kitting_publisher_.msg_.arm_detector_active = diag_arm_active;
      kitting_publisher_.msg_.gripper_detector_active = diag_gripper_active;
      kitting_publisher_.msg_.contact_detected = contact_latched_;

      kitting_publisher_.unlockAndPublish();
    }
  }
}

}  // namespace franka_kitting_controller

PLUGINLIB_EXPORT_CLASS(franka_kitting_controller::KittingStateController,
                       controller_interface::ControllerBase)
