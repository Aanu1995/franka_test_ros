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
    // Ignore stale latched messages from dead publishers
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
    // Routes BASELINE/CLOSING/GRASPING/AUTO commands. Non-realtime (subscriber thread).
    // Parameter override: 0.0 fields fall back to YAML defaults.
    const std::string& cmd = msg->command;

    // Logger gate (when require_logger is true)
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

    // Guard: START state requires BASELINE/AUTO first
    if (current_state_.load(std::memory_order_relaxed) == GraspState::START &&
        cmd != "BASELINE" && cmd != "AUTO") {
      ROS_WARN("KittingStateController: Command '%s' rejected — controller is in START state. "
              "Publish BASELINE or AUTO on /kitting_controller/state_cmd first to begin the grasp sequence.",
              cmd.c_str());
      return;
    }

    // Cancel auto mode if a manual command is received
    if (auto_mode_.load(std::memory_order_relaxed) && cmd != "AUTO") {
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
    // Determine if gripper open is needed before baseline collection.
    // When require_logger_ (record:=true), always open to ensure clean baseline.
    baseline_needs_open_ = msg->open_gripper || require_logger_;

    if (baseline_needs_open_) {
      baseline_open_width_ = resolveParam(msg->open_width,
                                          gripper_data_buf_.readFromNonRT()->max_width);
      ROS_INFO("  [BASELINE]  Gripper open deferred: width=%.4f", baseline_open_width_);
    }

    // baseline_prep_done_ is set FALSE here; it will be set TRUE by the RT thread
    // only after downlift completes AND gripper open completes (if needed).
    baseline_prep_done_.store(false, std::memory_order_release);
    baseline_open_dispatched_.store(false, std::memory_order_relaxed);

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
    // Label published from RT thread in applyPendingStateTransition() to prevent
    // RealtimePublisher race — timer thread publish can be overwritten before delivery.
    logStateTransition("CLOSING_COMMAND", "Gripper close queued — awaiting execution");
    ROS_INFO("    width=%.4f m  speed=%.4f m/s", width, speed);
  }

  void KittingStateController::handleGraspingCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    // Guard: GRASPING requires CONTACT (ramp step advances bypass this via requestDeferredGrasp)
    auto cur = current_state_.load(std::memory_order_relaxed);
    if (cur != GraspState::CONTACT) {
      ROS_WARN("KittingStateController: GRASPING rejected — not in CONTACT state (current: %s)",
              stateToString(cur));
      return;
    }

    double width = contact_width_.load(std::memory_order_relaxed);
    double max_w = gripper_data_buf_.readFromNonRT()->max_width;
    if (width <= 0.0 || width > max_w) {
      ROS_WARN("KittingStateController: GRASPING rejected — contact_width %.4f out of range [0, %.4f]",
              width, max_w);
      return;
    }

    // Resolve force ramp parameters
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
    staging_fr_grasp_force_hold_time_ = resolveParam(msg->grasp_force_hold_time, fr_grasp_force_hold_time_);
    staging_fr_grasp_settle_time_     = resolveParam(msg->grasp_settle_time, fr_grasp_settle_time_);

    if (staging_fr_uplift_distance_ > kMaxUpliftDistance) {
      ROS_WARN("KittingStateController: fr_uplift_distance %.4f exceeds max %.4f, clamping",
              staging_fr_uplift_distance_, kMaxUpliftDistance);
      staging_fr_uplift_distance_ = kMaxUpliftDistance;
    }
    if (staging_fr_lift_speed_ < kMinLiftSpeed) {
      ROS_WARN("KittingStateController: fr_lift_speed %.4f below min %.4f, clamping",
              staging_fr_lift_speed_, kMinLiftSpeed);
      staging_fr_lift_speed_ = kMinLiftSpeed;
    }
    if (staging_fr_uplift_hold_ < kMinUpliftHold) {
      ROS_WARN("KittingStateController: fr_uplift_hold %.2f below min %.2f, clamping",
              staging_fr_uplift_hold_, kMinUpliftHold);
      staging_fr_uplift_hold_ = kMinUpliftHold;
    }
    if (staging_fr_uplift_hold_ > kMaxUpliftHold) {
      ROS_WARN("KittingStateController: fr_uplift_hold %.2f exceeds max %.2f, clamping",
              staging_fr_uplift_hold_, kMaxUpliftHold);
      staging_fr_uplift_hold_ = kMaxUpliftHold;
    }

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

    int total_steps = 1 + static_cast<int>((staging_fr_f_max_ - staging_fr_f_min_) / staging_fr_f_step_);

    pending_state_.store(GraspState::GRASPING, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("GRASP_1");
    logStateTransition("GRASP_1", "Force ramp starting");
    ROS_INFO("    width=%.4f m  f_min=%.1f N  f_max=%.1f N  f_step=%.1f N  (%d steps)",
            width, staging_fr_f_min_, staging_fr_f_max_, staging_fr_f_step_, total_steps);
    ROS_INFO("    ramp: hold=%.2f s  settle=%.2f s",
            staging_fr_grasp_force_hold_time_, staging_fr_grasp_settle_time_);
    ROS_INFO("    uplift: dist=%.4f m  speed=%.4f m/s  eval_hold=%.2f s",
            staging_fr_uplift_distance_, staging_fr_lift_speed_, staging_fr_uplift_hold_);
    ROS_INFO("    grasp: speed=%.4f m/s  eps=%.4f m",
            staging_fr_grasp_speed_, staging_fr_epsilon_);
    ROS_INFO("    eval: DF_TH=%.3f  W_TH=%.4f m  load_min=%.2f N",
            staging_fr_slip_drop_thresh_, staging_fr_slip_width_thresh_,
            staging_fr_load_transfer_min_);
  }

  // ============================================================================
  // Auto mode — single-command full grasp sequence
  // ============================================================================

  void KittingStateController::handleAutoCmd(
      const KittingGripperCommand::ConstPtr& msg) {
    cancelAutoMode();

    auto_mode_.store(true, std::memory_order_relaxed);
    auto_cmd_ = *msg;
    auto_delay_ = (msg->auto_delay > 0.0) ? msg->auto_delay : 5.0;

    handleBaselineCmd(msg);

    // Poll for baseline readiness instead of using a fixed timer.
    // The prep sequence (lower → open → collect) can take variable time,
    // so we must wait until baseline_prep_done_ AND cd_baseline_ready_
    // before advancing to CLOSING.
    auto_baseline_poll_timer_ = auto_nh_.createTimer(
        ros::Duration(0.1),
        &KittingStateController::autoBaselinePollCallback, this);

    ROS_INFO("KittingStateController: AUTO mode started — polling for baseline ready");
  }

  void KittingStateController::autoBaselinePollCallback(const ros::TimerEvent&) {
    if (!auto_mode_.load(std::memory_order_relaxed)) {
      auto_baseline_poll_timer_.stop();
      return;
    }

    auto state = current_state_.load(std::memory_order_relaxed);
    if (state == GraspState::FAILED) {
      auto_baseline_poll_timer_.stop();
      auto_mode_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: AUTO -> FAILED during BASELINE prep, auto mode ended");
      return;
    }

    // Wait for prep done AND baseline data collected
    if (baseline_prep_done_.load(std::memory_order_relaxed) &&
        cd_baseline_ready_.load(std::memory_order_relaxed)) {
      auto_baseline_poll_timer_.stop();
      // Apply auto_delay AFTER baseline is ready
      auto_delay_timer_ = auto_nh_.createTimer(
          ros::Duration(auto_delay_),
          &KittingStateController::autoClosingCallback, this, /*oneshot=*/true);
      ROS_INFO("KittingStateController: AUTO -> BASELINE ready, CLOSING in %.1fs", auto_delay_);
    }
  }

  void KittingStateController::autoClosingCallback(const ros::TimerEvent&) {
    if (!auto_mode_.load(std::memory_order_relaxed)) return;

    auto msg = boost::make_shared<KittingGripperCommand>(auto_cmd_);
    ROS_INFO("KittingStateController: AUTO -> forwarding to CLOSING"
            " (closing_width=%.4f, closing_speed=%.4f)",
            auto_cmd_.closing_width, auto_cmd_.closing_speed);
    handleClosingCmd(msg);

    auto_contact_poll_timer_ = auto_nh_.createTimer(
        ros::Duration(0.1),
        &KittingStateController::autoContactPollCallback, this);

    ROS_INFO("KittingStateController: AUTO -> CLOSING, polling for CONTACT");
  }

  void KittingStateController::autoContactPollCallback(const ros::TimerEvent&) {
    if (!auto_mode_.load(std::memory_order_relaxed)) {
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
      auto_mode_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: AUTO -> FAILED during CLOSING, auto mode ended");
    }
  }

  void KittingStateController::autoGraspingCallback(const ros::TimerEvent&) {
    if (!auto_mode_.load(std::memory_order_relaxed)) return;

    auto msg = boost::make_shared<KittingGripperCommand>(auto_cmd_);
    ROS_INFO("KittingStateController: AUTO -> forwarding to GRASPING"
            " (f_min=%.1f, f_max=%.1f, f_step=%.1f, uplift_dist=%.4f,"
            " uplift_hold=%.2f, lift_speed=%.4f, grasp_speed=%.4f,"
            " eps=%.4f, DF_TH=%.3f, W_TH=%.4f, load_min=%.2f)",
            auto_cmd_.f_min, auto_cmd_.f_max, auto_cmd_.f_step,
            auto_cmd_.fr_uplift_distance, auto_cmd_.fr_uplift_hold,
            auto_cmd_.fr_lift_speed, auto_cmd_.fr_grasp_speed,
            auto_cmd_.fr_epsilon,
            auto_cmd_.fr_slip_drop_thresh, auto_cmd_.fr_slip_width_thresh,
            auto_cmd_.fr_load_transfer_min);
    handleGraspingCmd(msg);
    auto_mode_.store(false, std::memory_order_relaxed);

    ROS_INFO("KittingStateController: AUTO -> GRASPING started, auto mode complete");
  }

  void KittingStateController::cancelAutoMode() {
    if (!auto_mode_.load(std::memory_order_relaxed)) return;
    auto_delay_timer_.stop();
    auto_baseline_poll_timer_.stop();
    auto_contact_poll_timer_.stop();
    auto_mode_.store(false, std::memory_order_relaxed);
    ROS_INFO("KittingStateController: AUTO mode cancelled");
  }

}  // namespace franka_kitting_controller
