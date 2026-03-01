// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// State command callback, command handlers (BASELINE, CLOSING, GRASPING),
// auto mode, and logger readiness callback.
// Part of KittingStateController — see kitting_state_controller.h for class definition.

#include <franka_kitting_controller/kitting_state_controller.h>

#include <string>

#include <boost/make_shared.hpp>
#include <ros/ros.h>

namespace franka_kitting_controller {

  // ============================================================================
  // Logger readiness callback
  // ============================================================================

  void KittingStateController::loggerReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Only accept if the logger publisher is actually alive right now.
    // A stale latched message from a previous logger run will still trigger this
    // callback, but getNumPublishers() == 0 in that case because the original
    // publisher is gone — the ROS master replays the latched message, not the node.
    if (msg->data && logger_ready_sub_.getNumPublishers() > 0) {
      if (!logger_ready_.load(std::memory_order_relaxed)) {
        logger_ready_.store(true, std::memory_order_relaxed);
        logStateTransition("READY", "Grasp logger detected — commands now accepted");
      }
    }
  }

  // ============================================================================
  // State command callback + command handlers
  // ============================================================================

  void KittingStateController::stateCmdCallback(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    // Listens to /kitting_controller/state_cmd for BASELINE, CLOSING, and GRASPING
    // commands with optional per-object parameters.
    //
    // This callback runs in the subscriber's spinner thread (non-realtime), so it is
    // safe to queue gripper commands here.
    //
    // Parameter override rule: any field set to 0.0 (the default for float64 in
    // ROS messages) falls back to the YAML config value loaded at init().
    // Non-zero values override the default for that single command.
    //
    // For each valid command:
    //   1. Resolve per-command parameters (msg value if non-zero, else YAML default)
    //   2. Queue gripper command if applicable (non-blocking)
    //   3. Set pending_state_ + state_changed_ for realtime update() to apply
    //   4. Publish state label on /kitting_controller/state
    const std::string& cmd = msg->command;

    // --- Guard: reject commands if the Grasp logger is not running ---
    // Only enforced when require_logger is true (record:=true in launch).
    if (require_logger_) {
      if (!logger_ready_.load(std::memory_order_relaxed) ||
          logger_ready_sub_.getNumPublishers() == 0) {
        logger_ready_.store(false, std::memory_order_relaxed);
        ROS_WARN("KittingStateController: Command '%s' rejected — Grasp logger not detected. "
                "Relaunch with record:=true to enable recording.",
                cmd.c_str());
        return;
      }
    }

    // --- Guard: reject non-BASELINE/AUTO commands before BASELINE has been published ---
    if (current_state_.load(std::memory_order_relaxed) == GraspState::START &&
        cmd != "BASELINE" && cmd != "AUTO") {
      ROS_WARN("KittingStateController: Command '%s' rejected — controller is in START state. "
              "Publish BASELINE or AUTO on /kitting_controller/state_cmd first to begin the grasp sequence.",
              cmd.c_str());
      return;
    }

    // Cancel auto mode if a manual command is received
    if (auto_mode_ && cmd != "AUTO") {
      cancelAutoMode();
    }

    if (cmd == "BASELINE") {
      handleBaselineCmd(msg);
    } else if (cmd == "CLOSING") {
      handleClosingCmd(msg);
    } else if (cmd == "GRASPING") {
      handleGraspingCmd(msg);
    } else if (cmd == "AUTO") {
      handleAutoCmd(msg);
    } else {
      ROS_WARN("KittingStateController: Unknown command '%s' on state_cmd "
              "(expected BASELINE, CLOSING, GRASPING, or AUTO)", cmd.c_str());
    }
  }

  void KittingStateController::handleBaselineCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    if (msg->open_gripper) {
      double open_w = resolveParam(msg->open_width,
                                  gripper_data_buf_.readFromNonRT()->max_width);
      GripperCommand open_cmd;
      open_cmd.type = GripperCommandType::MOVE;
      open_cmd.width = open_w;
      open_cmd.speed = 0.1;
      queueGripperCommand(open_cmd);
      ROS_INFO("  [GRIPPER]  Open queued: move(width=%.4f, speed=0.1)", open_w);
    }

    pending_state_.store(GraspState::BASELINE, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("BASELINE");
    logStateTransition("BASELINE", "Collecting reference signals");
  }

  void KittingStateController::handleClosingCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    auto cur_state = current_state_.load(std::memory_order_relaxed);
    if (isClosingPhase(cur_state)) {
      ROS_DEBUG("KittingStateController: Already in %s, ignoring duplicate CLOSING",
                stateToString(cur_state));
      return;
    }

    double width = resolveParam(msg->closing_width, closing_width_);
    double speed = resolveParam(msg->closing_speed, closing_speed_);

    if (speed > kMaxClosingSpeed) {
      ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
              speed, kMaxClosingSpeed);
      speed = kMaxClosingSpeed;
    }

    GripperCommand gripper_cmd;
    gripper_cmd.type = GripperCommandType::MOVE;
    gripper_cmd.width = width;
    gripper_cmd.speed = speed;
    queueGripperCommand(gripper_cmd);
    ROS_INFO("KittingStateController: Queued move(width=%.4f, speed=%.4f)", width, speed);

    closing_w_cmd_ = width;
    closing_v_cmd_ = speed;

    pending_state_.store(GraspState::CLOSING_COMMAND, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("CLOSING_COMMAND");
    logStateTransition("CLOSING_COMMAND", "Gripper close queued — awaiting execution");
    ROS_INFO("    width=%.4f m  speed=%.4f m/s", width, speed);
  }

  void KittingStateController::handleGraspingCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    double width = contact_width_.load(std::memory_order_relaxed);
    if (width <= 0.0) {
      ROS_WARN("KittingStateController: GRASPING rejected — no contact width available "
              "(CONTACT must be reached before GRASPING)");
      return;
    }

    // Resolve force ramp parameters (0 = use YAML default)
    staging_fr_f_min_             = resolveParam(msg->f_min, fr_f_min_);
    staging_fr_f_step_            = resolveParam(msg->f_step, fr_f_step_);
    staging_fr_f_max_             = resolveParam(msg->f_max, fr_f_max_);
    staging_fr_uplift_distance_   = resolveParam(msg->fr_uplift_distance, fr_uplift_distance_);
    staging_fr_lift_speed_        = resolveParam(msg->fr_lift_speed, fr_lift_speed_);
    staging_fr_uplift_hold_       = resolveParam(msg->fr_uplift_hold, fr_uplift_hold_);
    staging_fr_grasp_speed_       = resolveParam(msg->fr_grasp_speed, fr_grasp_speed_);
    staging_fr_epsilon_           = resolveParam(msg->fr_epsilon, fr_epsilon_);
    staging_fr_slip_drop_thresh_     = resolveParam(msg->fr_slip_drop_thresh, fr_slip_drop_thresh_);
    staging_fr_slip_width_thresh_    = resolveParam(msg->fr_slip_width_thresh, fr_slip_width_thresh_);
    staging_fr_load_transfer_min_    = resolveParam(msg->fr_load_transfer_min, fr_load_transfer_min_);
    staging_fr_slip_score_thresh_    = resolveParam(msg->fr_slip_score_thresh, fr_slip_score_thresh_);
    staging_fr_slip_friction_thresh_ = resolveParam(msg->fr_slip_friction_thresh, fr_slip_friction_thresh_);

    if (staging_fr_uplift_distance_ > kMaxUpliftDistance) {
      ROS_WARN("KittingStateController: fr_uplift_distance %.4f exceeds max %.4f, clamping",
              staging_fr_uplift_distance_, kMaxUpliftDistance);
      staging_fr_uplift_distance_ = kMaxUpliftDistance;
    }
    if (staging_fr_uplift_hold_ < kMinUpliftHold) {
      ROS_WARN("KittingStateController: fr_uplift_hold %.2f below min %.2f, clamping",
              staging_fr_uplift_hold_, kMinUpliftHold);
      staging_fr_uplift_hold_ = kMinUpliftHold;
    }

    // Queue first grasp command at f_min
    GripperCommand gripper_cmd;
    gripper_cmd.type = GripperCommandType::GRASP;
    gripper_cmd.width = width;
    gripper_cmd.speed = staging_fr_grasp_speed_;
    gripper_cmd.force = staging_fr_f_min_;
    gripper_cmd.epsilon_inner = staging_fr_epsilon_;
    gripper_cmd.epsilon_outer = staging_fr_epsilon_;
    queueGripperCommand(gripper_cmd);
    ROS_INFO("KittingStateController: Queued grasp(width=%.4f, eps=%.4f, "
            "speed=%.4f, force=%.1f)",
            width, staging_fr_epsilon_, staging_fr_grasp_speed_, staging_fr_f_min_);

    pending_state_.store(GraspState::GRASPING, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("GRASPING");
    logStateTransition("GRASPING", "Force ramp starting");
    ROS_INFO("    width=%.4f m  f_min=%.1f N  f_max=%.1f N  f_step=%.1f N",
            width, staging_fr_f_min_, staging_fr_f_max_, staging_fr_f_step_);
    ROS_INFO("    uplift: dist=%.4f m  speed=%.4f m/s  hold=%.2f s",
            staging_fr_uplift_distance_, staging_fr_lift_speed_, staging_fr_uplift_hold_);
    ROS_INFO("    grasp: speed=%.4f m/s  eps=%.4f m",
            staging_fr_grasp_speed_, staging_fr_epsilon_);
    ROS_INFO("    slip: DF_TH=%.3f  W_TH=%.4f m  U_TH=%.3f  S_TH=%.3f  load_min=%.2f N",
            staging_fr_slip_drop_thresh_, staging_fr_slip_width_thresh_,
            staging_fr_slip_friction_thresh_, staging_fr_slip_score_thresh_,
            staging_fr_load_transfer_min_);
  }

  // ============================================================================
  // Auto mode — single-command full grasp sequence
  // ============================================================================

  void KittingStateController::handleAutoCmd(
      const KittingGripperCommand::ConstPtr& msg) {
    cancelAutoMode();

    auto_mode_ = true;
    auto_cmd_ = *msg;
    auto_delay_ = (msg->auto_delay > 0.0) ? msg->auto_delay : 5.0;

    // Start with BASELINE (reuse existing handler)
    handleBaselineCmd(msg);

    // After auto_delay seconds, transition to CLOSING
    auto_delay_timer_ = auto_nh_.createTimer(
        ros::Duration(auto_delay_),
        &KittingStateController::autoClosingCallback, this, /*oneshot=*/true);

    ROS_INFO("KittingStateController: AUTO mode started (delay=%.1fs)", auto_delay_);
  }

  void KittingStateController::autoClosingCallback(const ros::TimerEvent&) {
    if (!auto_mode_) return;

    auto msg = boost::make_shared<KittingGripperCommand>(auto_cmd_);
    ROS_INFO("KittingStateController: AUTO -> forwarding to CLOSING"
            " (closing_width=%.4f, closing_speed=%.4f)",
            auto_cmd_.closing_width, auto_cmd_.closing_speed);
    handleClosingCmd(msg);

    // Poll for CONTACT or FAILED at 10 Hz
    auto_contact_poll_timer_ = auto_nh_.createTimer(
        ros::Duration(0.1),
        &KittingStateController::autoContactPollCallback, this);

    ROS_INFO("KittingStateController: AUTO -> CLOSING, polling for CONTACT");
  }

  void KittingStateController::autoContactPollCallback(const ros::TimerEvent&) {
    if (!auto_mode_) {
      auto_contact_poll_timer_.stop();
      return;
    }

    auto state = current_state_.load(std::memory_order_relaxed);
    if (state == GraspState::CONTACT) {
      auto_contact_poll_timer_.stop();
      auto_delay_timer_ = auto_nh_.createTimer(
          ros::Duration(auto_delay_),
          &KittingStateController::autoGraspingCallback, this, /*oneshot=*/true);
      ROS_INFO("KittingStateController: AUTO -> CONTACT detected, GRASPING in %.1fs",
               auto_delay_);
    } else if (state == GraspState::FAILED) {
      auto_contact_poll_timer_.stop();
      auto_mode_ = false;
      ROS_INFO("KittingStateController: AUTO -> FAILED during CLOSING, auto mode ended");
    }
  }

  void KittingStateController::autoGraspingCallback(const ros::TimerEvent&) {
    if (!auto_mode_) return;

    auto msg = boost::make_shared<KittingGripperCommand>(auto_cmd_);
    ROS_INFO("KittingStateController: AUTO -> forwarding to GRASPING"
            " (f_min=%.1f, f_max=%.1f, f_step=%.1f, uplift_dist=%.4f,"
            " uplift_hold=%.2f, lift_speed=%.4f, grasp_speed=%.4f,"
            " eps=%.4f, DF_TH=%.3f, W_TH=%.4f,"
            " U_TH=%.3f, S_TH=%.3f, load_min=%.2f)",
            auto_cmd_.f_min, auto_cmd_.f_max, auto_cmd_.f_step,
            auto_cmd_.fr_uplift_distance, auto_cmd_.fr_uplift_hold,
            auto_cmd_.fr_lift_speed, auto_cmd_.fr_grasp_speed,
            auto_cmd_.fr_epsilon,
            auto_cmd_.fr_slip_drop_thresh, auto_cmd_.fr_slip_width_thresh,
            auto_cmd_.fr_slip_friction_thresh, auto_cmd_.fr_slip_score_thresh,
            auto_cmd_.fr_load_transfer_min);
    handleGraspingCmd(msg);
    auto_mode_ = false;

    ROS_INFO("KittingStateController: AUTO -> GRASPING started, auto mode complete");
  }

  void KittingStateController::cancelAutoMode() {
    if (!auto_mode_) return;
    auto_delay_timer_.stop();
    auto_contact_poll_timer_.stop();
    auto_mode_ = false;
    ROS_INFO("KittingStateController: AUTO mode cancelled");
  }

}  // namespace franka_kitting_controller
