// Copyright (c) 2024
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_kitting_controller/kitting_state_controller.h>

#include <algorithm>
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

// C++14 requires out-of-line definitions for static constexpr members
// that are ODR-used (e.g. passed by reference to std::min/std::max).
constexpr double KittingStateController::kMaxClosingSpeed;
constexpr double KittingStateController::kGripperHoldBase;
constexpr double KittingStateController::kGripperHoldSlope;
constexpr double KittingStateController::kMaxUpliftDistance;
constexpr double KittingStateController::kGripperOpenWait;
constexpr double KittingStateController::kGraspSettleDelay;
constexpr double KittingStateController::kGraspTimeout;

const char* KittingStateController::stateToString(GraspState state) {
  switch (state) {
    case GraspState::START:
      return "START";
    case GraspState::BASELINE:
      return "BASELINE";
    case GraspState::CLOSING:
      return "CLOSING";
    case GraspState::CONTACT:
      return "CONTACT";
    case GraspState::GRASPING:
      return "GRASPING";
    case GraspState::UPLIFT:
      return "UPLIFT";
    case GraspState::EVALUATE:
      return "EVALUATE";
    case GraspState::DOWNLIFT:
      return "DOWNLIFT";
    case GraspState::SETTLING:
      return "SETTLING";
    case GraspState::SUCCESS:
      return "SUCCESS";
    case GraspState::FAILED:
      return "FAILED";
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

std::array<double, 16> KittingStateController::computeDownliftPose(double elapsed) const {
  std::array<double, 16> pose = downlift_start_pose_;
  double s_raw = std::min(elapsed / rt_downlift_duration_, 1.0);
  double s = 0.5 * (1.0 - std::cos(M_PI * s_raw));
  pose[14] = downlift_z_start_ - s * rt_downlift_distance_;
  return pose;
}

void KittingStateController::requestGripperStop(const char* source) {
  if (stop_on_contact_ && !gripper_stop_sent_.load(std::memory_order_relaxed) &&
      execute_gripper_actions_) {
    stop_requested_.store(true, std::memory_order_relaxed);
    gripper_stop_sent_.store(true, std::memory_order_relaxed);
    ROS_INFO("  [GRIPPER]  %s contact -> stop() requested", source);
  }
}

void KittingStateController::logStateTransition(const char* label, const char* detail) {
  ROS_INFO("============================================================");
  if (detail) {
    ROS_INFO("  [STATE]  >>  %s  <<  %s", label, detail);
  } else {
    ROS_INFO("  [STATE]  >>  %s  <<", label);
  }
  ROS_INFO("============================================================");
}

double KittingStateController::computeGripperHoldTime(double closing_speed) {
  double clamped = std::min(std::max(closing_speed, 0.0), kMaxClosingSpeed);
  return kGripperHoldBase + kGripperHoldSlope * clamped;
}

void KittingStateController::requestDeferredGrasp(double width, double speed,
                                                   double force, double epsilon) {
  // RT-safe: only atomic stores, no mutex or allocation.
  // Gripper read thread acquire-loads the flag and dispatches to command thread.
  deferred_grasp_width_ = width;
  deferred_grasp_speed_ = speed;
  deferred_grasp_force_ = force;
  deferred_grasp_epsilon_ = epsilon;
  deferred_grasp_pending_.store(true, std::memory_order_release);
}

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

KittingStateController::~KittingStateController() {
  if (gripper_read_thread_.joinable() || gripper_cmd_thread_.joinable()) {
    gripper_shutdown_.store(true, std::memory_order_relaxed);
    stop_requested_.store(true, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd_ready_ = true;  // Wake command thread so it can exit
    }
    cmd_cv_.notify_one();
    if (gripper_read_thread_.joinable()) {
      gripper_read_thread_.join();
    }
    if (gripper_cmd_thread_.joinable()) {
      gripper_cmd_thread_.join();
    }
  }
}

void KittingStateController::gripperReadLoop() {
  // Runs at firmware rate (blocking readOnce()). No artificial rate limit.
  // Computes width velocity via finite difference and writes to RealtimeBuffer.
  // Also checks stop_requested_ flag after each read and calls stop() if set.
  double prev_width = 0.0;
  ros::Time prev_stamp;
  bool prev_valid = false;

  while (!gripper_shutdown_.load(std::memory_order_relaxed)) {
    try {
      franka::GripperState gs = gripper_->readOnce();

      double w_dot = 0.0;
      ros::Time now = ros::Time::now();
      if (prev_valid) {
        double dt = (now - prev_stamp).toSec();
        if (dt > 1e-6) {
          w_dot = (gs.width - prev_width) / dt;
        }
      }
      prev_width = gs.width;
      prev_stamp = now;
      prev_valid = true;

      GripperData data;
      data.width = gs.width;
      data.max_width = gs.max_width;
      data.width_dot = w_dot;
      data.is_grasped = gs.is_grasped;
      data.stamp = now;
      gripper_data_buf_.writeFromNonRT(data);

      // Check if RT thread requested a stop (contact detected)
      if (stop_requested_.load(std::memory_order_relaxed)) {
        try {
          gripper_->stop();
          ROS_INFO("  [GRIPPER]  stop() executed by read thread");
        } catch (const franka::Exception& ex) {
          ROS_WARN_STREAM("KittingStateController: stop() failed: " << ex.what());
        }
        stop_requested_.store(false, std::memory_order_relaxed);
      }

      // Publish baseline statistics to parameter server (deferred from RT thread).
      // The acquire-load ensures we see the baseline_mu_/sigma_/threshold_ values
      // written by the RT thread before its release-store of this flag.
      if (baseline_params_pending_.load(std::memory_order_acquire)) {
        ros::param::set("/kitting_state_controller/baseline_mu", baseline_mu_);
        ros::param::set("/kitting_state_controller/baseline_sigma", baseline_sigma_);
        ros::param::set("/kitting_state_controller/contact_threshold", contact_threshold_);
        baseline_params_pending_.store(false, std::memory_order_relaxed);
      }

      // Dispatch deferred grasp command from RT thread (force ramp iteration).
      // Same pattern as baseline_params_pending_: acquire-load ensures we see the
      // parameter stores made by the RT thread before its release-store of the flag.
      if (deferred_grasp_pending_.load(std::memory_order_acquire)) {
        GripperCommand grasp_cmd;
        grasp_cmd.type = GripperCommandType::GRASP;
        grasp_cmd.width = deferred_grasp_width_;
        grasp_cmd.speed = deferred_grasp_speed_;
        grasp_cmd.force = deferred_grasp_force_;
        grasp_cmd.epsilon_inner = deferred_grasp_epsilon_;
        grasp_cmd.epsilon_outer = deferred_grasp_epsilon_;
        queueGripperCommand(grasp_cmd);
        ROS_INFO("  [GRIPPER]  Deferred grasp queued: width=%.4f speed=%.4f force=%.1f eps=%.4f",
                 deferred_grasp_width_, deferred_grasp_speed_,
                 deferred_grasp_force_, deferred_grasp_epsilon_);
        deferred_grasp_pending_.store(false, std::memory_order_relaxed);
      }
    } catch (const franka::Exception& ex) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "KittingStateController: readOnce() failed: " << ex.what());
      prev_valid = false;
      ros::Duration(0.01).sleep();  // Back off before retry
    }
  }
}

void KittingStateController::gripperCommandLoop() {
  // Waits on condition variable for commands, then executes blocking move()/grasp().
  while (!gripper_shutdown_.load(std::memory_order_relaxed)) {
    GripperCommand cmd;
    {
      std::unique_lock<std::mutex> lock(cmd_mutex_);
      cmd_cv_.wait(lock, [this]() {
        return cmd_ready_ || gripper_shutdown_.load(std::memory_order_relaxed);
      });
      if (gripper_shutdown_.load(std::memory_order_relaxed)) {
        return;
      }
      cmd = pending_cmd_;
      cmd_ready_ = false;
    }

    cmd_executing_.store(true, std::memory_order_relaxed);
    try {
      bool success = false;
      if (cmd.type == GripperCommandType::MOVE) {
        success = gripper_->move(cmd.width, cmd.speed);
        ROS_INFO("KittingStateController: move(%.4f, %.4f) -> %s",
                 cmd.width, cmd.speed, success ? "OK" : "FAIL");
      } else if (cmd.type == GripperCommandType::GRASP) {
        success = gripper_->grasp(cmd.width, cmd.speed, cmd.force,
                                   cmd.epsilon_inner, cmd.epsilon_outer);
        ROS_INFO("KittingStateController: grasp(%.4f, %.4f, %.1f) -> %s",
                 cmd.width, cmd.speed, cmd.force, success ? "OK" : "FAIL");
      }
    } catch (const franka::Exception& ex) {
      // stop() aborts the in-flight move()/grasp(), which throws "command aborted".
      // This is expected after contact detection — log as INFO, not ERROR.
      if (gripper_stop_sent_.load(std::memory_order_relaxed)) {
        ROS_INFO("KittingStateController: Gripper command aborted by contact stop (expected)");
      } else {
        ROS_ERROR_STREAM("KittingStateController: Gripper command failed: " << ex.what());
      }
    }
    cmd_executing_.store(false, std::memory_order_relaxed);
  }
}

void KittingStateController::queueGripperCommand(const GripperCommand& cmd) {
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    // Check if a command is executing; if so, request stop so it unblocks
    // and the new command can proceed. Note: cmd_executing_ is atomic and
    // may transition false between the check and the store — this is benign
    // because a spurious stop() is safely caught by the read thread.
    if (cmd_executing_.load(std::memory_order_relaxed)) {
      stop_requested_.store(true, std::memory_order_relaxed);
    }
    pending_cmd_ = cmd;
    cmd_ready_ = true;
  }
  cmd_cv_.notify_one();
}

void KittingStateController::stateCmdCallback(
    const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
  // Listens to /kitting_controller/state_cmd for BASELINE, CLOSING, and GRASPING
  // commands with optional per-object parameters.
  //
  // This callback runs in the subscriber's spinner thread (non-RT), so it is
  // safe to queue gripper commands here.
  //
  // Parameter override rule: any field set to 0.0 (the default for float64 in
  // ROS messages) falls back to the YAML config value loaded at init().
  // Non-zero values override the default for that single command.
  //
  // For each valid command:
  //   1. Resolve per-command parameters (msg value if non-zero, else YAML default)
  //   2. Queue gripper command if applicable (non-blocking)
  //   3. Set pending_state_ + state_changed_ for RT update() to apply
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

  // --- Guard: reject non-BASELINE commands before BASELINE has been published ---
  if (current_state_.load(std::memory_order_relaxed) == GraspState::START && cmd != "BASELINE") {
    ROS_WARN("KittingStateController: Command '%s' rejected — controller is in START state. "
             "Publish BASELINE on /kitting_controller/state_cmd first to begin the grasp sequence.",
             cmd.c_str());
    return;
  }

  if (cmd == "BASELINE") {
    handleBaselineCmd(msg);
  } else if (cmd == "CLOSING") {
    handleClosingCmd(msg);
  } else if (cmd == "GRASPING") {
    handleGraspingCmd(msg);
  } else {
    ROS_WARN("KittingStateController: Unknown command '%s' on state_cmd "
             "(expected BASELINE, CLOSING, or GRASPING)", cmd.c_str());
  }
}

void KittingStateController::handleBaselineCmd(
    const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
  if (msg->open_gripper && execute_gripper_actions_) {
    double open_w = resolveParam(msg->open_width,
                                 gripper_data_buf_.readFromNonRT()->max_width);
    GripperCommand open_cmd;
    open_cmd.type = GripperCommandType::MOVE;
    open_cmd.width = open_w;
    open_cmd.speed = 0.1;
    queueGripperCommand(open_cmd);
    baseline_gripper_wait_start_ = ros::Time::now();
    baseline_awaiting_gripper_ = true;
    ROS_INFO("  [GRIPPER]  Open queued: move(width=%.4f, speed=0.1)", open_w);
  } else {
    baseline_awaiting_gripper_ = false;
  }

  pending_state_.store(GraspState::BASELINE, std::memory_order_relaxed);
  state_changed_.store(true, std::memory_order_release);
  publishStateLabel("BASELINE");
  logStateTransition("BASELINE", "Collecting reference signals");
}

void KittingStateController::handleClosingCmd(
    const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
  if (current_state_.load(std::memory_order_relaxed) == GraspState::CLOSING) {
    ROS_DEBUG("KittingStateController: Already in CLOSING, ignoring duplicate");
    return;
  }

  if (enable_arm_contact_ && !baseline_armed_.load(std::memory_order_relaxed)) {
    ROS_WARN("KittingStateController: Baseline not yet armed — arm torque contact detection "
             "will NOT work for this closing. Only gripper stall detection is active.");
  }

  double width = resolveParam(msg->closing_width, closing_width_);
  double speed = resolveParam(msg->closing_speed, closing_speed_);

  if (speed > kMaxClosingSpeed) {
    ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
             speed, kMaxClosingSpeed);
    speed = kMaxClosingSpeed;
  }

  if (execute_gripper_actions_) {
    GripperCommand gripper_cmd;
    gripper_cmd.type = GripperCommandType::MOVE;
    gripper_cmd.width = width;
    gripper_cmd.speed = speed;
    queueGripperCommand(gripper_cmd);
    ROS_INFO("KittingStateController: Queued move(width=%.4f, speed=%.4f)", width, speed);
  }

  closing_w_cmd_ = width;
  closing_v_cmd_ = speed;

  pending_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
  state_changed_.store(true, std::memory_order_release);
  publishStateLabel("CLOSING");
  logStateTransition("CLOSING", "Gripper approaching object");
  ROS_INFO("    width=%.4f m  speed=%.4f m/s", width, speed);
}

void KittingStateController::handleGraspingCmd(
    const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
  if (!execute_gripper_actions_) {
    ROS_WARN("KittingStateController: GRASPING rejected — force ramp requires "
             "execute_gripper_actions to be true (signal-only mode is incompatible)");
    return;
  }

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
  staging_fr_stabilization_     = resolveParam(msg->fr_stabilization, fr_stabilization_);
  staging_fr_slip_tau_drop_     = resolveParam(msg->fr_slip_tau_drop, fr_slip_tau_drop_);
  staging_fr_slip_width_change_ = resolveParam(msg->fr_slip_width_change, fr_slip_width_change_);

  if (staging_fr_uplift_distance_ > kMaxUpliftDistance) {
    ROS_WARN("KittingStateController: fr_uplift_distance %.4f exceeds max %.4f, clamping",
             staging_fr_uplift_distance_, kMaxUpliftDistance);
    staging_fr_uplift_distance_ = kMaxUpliftDistance;
  }

  fr_grasp_width_ = width;

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

  // --- Grasp: Load arm contact detector parameters ---
  node_handle.param("enable_contact_detector", enable_contact_detector_, true);
  node_handle.param("T_base", T_base_, 0.7);
  node_handle.param("N_min", N_min_, 50);
  node_handle.param("k_sigma", k_sigma_, 3.0);
  node_handle.param("T_hold_arm", T_hold_arm_, 0.10);
  node_handle.param("use_slope_gate", use_slope_gate_, false);
  node_handle.param("slope_min", slope_min_, 5.0);

  // --- Gripper contact detection parameters ---
  node_handle.param("stall_velocity_threshold", stall_velocity_threshold_, 0.007);
  node_handle.param("width_gap_threshold", width_gap_threshold_, 0.002);
  node_handle.param("stop_on_contact", stop_on_contact_, true);
  node_handle.param("enable_arm_contact", enable_arm_contact_, false);
  node_handle.param("enable_gripper_contact", enable_gripper_contact_, true);

  ROS_INFO_STREAM("KittingStateController: Contact detector "
                   << (enable_contact_detector_ ? "ENABLED" : "DISABLED")
                   << " | ARM: " << (enable_arm_contact_ ? "ON" : "OFF")
                   << " T_base=" << T_base_ << " N_min=" << N_min_
                   << " k_sigma=" << k_sigma_ << " T_hold_arm=" << T_hold_arm_
                   << " | GRIPPER: " << (enable_gripper_contact_ ? "ON" : "OFF")
                   << " stall_vel=" << stall_velocity_threshold_
                   << " gap=" << width_gap_threshold_ << " T_hold_grip=dynamic(speed)"
                   << " | stop_on_contact=" << (stop_on_contact_ ? "true" : "false"));

  // --- Grasp: Gripper default parameters (overridable per-command via KittingGripperCommand) ---
  node_handle.param("execute_gripper_actions", execute_gripper_actions_, true);
  node_handle.param("closing_width", closing_width_, 0.01);
  node_handle.param("closing_speed", closing_speed_, 0.02);
  if (closing_speed_ > kMaxClosingSpeed) {
    ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
             closing_speed_, kMaxClosingSpeed);
    closing_speed_ = kMaxClosingSpeed;
  }
  ROS_INFO_STREAM("KittingStateController: Gripper execution "
                   << (execute_gripper_actions_ ? "ENABLED" : "DISABLED (signal-only)")
                   << " | closing: width=" << closing_width_ << " speed=" << closing_speed_
                   << " | grasp width: from contact_width (auto)");

  // --- GRASPING: Force ramp parameters (overridable per-command via KittingGripperCommand) ---
  node_handle.param("f_min", fr_f_min_, 3.0);
  node_handle.param("f_step", fr_f_step_, 2.0);
  node_handle.param("f_max", fr_f_max_, 15.0);
  node_handle.param("uplift_distance", fr_uplift_distance_, 0.003);
  node_handle.param("lift_speed", fr_lift_speed_, 0.01);
  node_handle.param("uplift_hold", fr_uplift_hold_, 0.5);
  node_handle.param("grasp_speed", fr_grasp_speed_, 0.02);
  node_handle.param("epsilon", fr_epsilon_, 0.008);
  node_handle.param("stabilization", fr_stabilization_, 0.3);
  node_handle.param("slip_tau_drop", fr_slip_tau_drop_, 0.20);
  node_handle.param("slip_width_change", fr_slip_width_change_, 0.001);

  // Validate force ramp parameters
  if (fr_f_min_ <= 0.0) {
    ROS_ERROR("KittingStateController: f_min must be positive (got %.2f)", fr_f_min_);
    return false;
  }
  if (fr_f_step_ <= 0.0) {
    ROS_ERROR("KittingStateController: f_step must be positive (got %.2f)", fr_f_step_);
    return false;
  }
  if (fr_f_max_ < fr_f_min_) {
    ROS_ERROR("KittingStateController: f_max (%.2f) must be >= f_min (%.2f)",
              fr_f_max_, fr_f_min_);
    return false;
  }
  if (fr_uplift_distance_ <= 0.0) {
    ROS_ERROR("KittingStateController: uplift_distance must be positive (got %.4f)",
              fr_uplift_distance_);
    return false;
  }
  if (fr_uplift_distance_ > kMaxUpliftDistance) {
    ROS_WARN("KittingStateController: uplift_distance %.4f exceeds max %.4f, clamping",
             fr_uplift_distance_, kMaxUpliftDistance);
    fr_uplift_distance_ = kMaxUpliftDistance;
  }
  if (fr_lift_speed_ <= 0.0) {
    ROS_ERROR("KittingStateController: lift_speed must be positive (got %.4f)",
              fr_lift_speed_);
    return false;
  }

  ROS_INFO_STREAM("KittingStateController: Force ramp config"
                   << " | f: min=" << fr_f_min_ << " step=" << fr_f_step_
                   << " max=" << fr_f_max_
                   << " | uplift: dist=" << fr_uplift_distance_
                   << " speed=" << fr_lift_speed_ << " hold=" << fr_uplift_hold_
                   << " | grasp: speed=" << fr_grasp_speed_ << " eps=" << fr_epsilon_
                   << " | stab=" << fr_stabilization_
                   << " | slip: tau_drop=" << fr_slip_tau_drop_
                   << " width_change=" << fr_slip_width_change_);

  // --- Direct gripper connection via libfranka ---
  if (execute_gripper_actions_) {
    if (!node_handle.getParam("robot_ip", robot_ip_)) {
      ros::NodeHandle root_nh_ip;
      if (!root_nh_ip.getParam("/franka_control/robot_ip", robot_ip_)) {
        ROS_ERROR("KittingStateController: Could not find robot_ip parameter");
        return false;
      }
    }
    try {
      gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
      ROS_INFO("KittingStateController: Connected to gripper at %s", robot_ip_.c_str());
    } catch (const franka::Exception& ex) {
      ROS_ERROR_STREAM("KittingStateController: Failed to connect to gripper: " << ex.what());
      return false;
    }
    gripper_read_thread_ = std::thread(&KittingStateController::gripperReadLoop, this);
    gripper_cmd_thread_ = std::thread(&KittingStateController::gripperCommandLoop, this);
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

  // Grasp: State label publisher
  ros::NodeHandle root_nh;
  state_publisher_.init(root_nh, "/kitting_controller/state", 1);

  // Grasp: Logger readiness gate.
  // When require_logger is true, commands are rejected until the logger publishes ready.
  // When false, commands are accepted immediately (no recording).
  node_handle.param("require_logger", require_logger_, true);
  if (require_logger_) {
    logger_ready_sub_ = root_nh.subscribe("/kitting_controller/logger_ready", 1,
                                           &KittingStateController::loggerReadyCallback, this);
    ROS_INFO("KittingStateController: Recording enabled — waiting for logger");
  } else {
    ROS_INFO("KittingStateController: Recording disabled — commands accepted immediately");
  }

  // Grasp: Command subscriber — user publishes KittingGripperCommand here
  // with command (BASELINE/CLOSING/GRASPING) + optional per-object parameters.
  state_cmd_sub_ = root_nh.subscribe("/kitting_controller/state_cmd", 10,
                                      &KittingStateController::stateCmdCallback, this);

  // Initialize gripper data buffer with safe defaults (fully open, no motion)
  GripperData initial_gripper_data;
  initial_gripper_data.width = 0.08;
  initial_gripper_data.max_width = 0.08;
  initial_gripper_data.width_dot = 0.0;
  initial_gripper_data.is_grasped = false;
  initial_gripper_data.stamp = ros::Time(0);
  gripper_data_buf_.initRT(initial_gripper_data);

  return true;
}

void KittingStateController::starting(const ros::Time& /* time */) {
  // Capture the current desired pose and send it as the first command.
  // O_T_EE_d is the robot's last commanded pose — using this avoids any step discontinuity.
  uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  cartesian_pose_handle_->setCommand(uplift_start_pose_);  // Defensive: no uninitialized gap
  uplift_active_.store(false, std::memory_order_relaxed);
  uplift_elapsed_ = 0.0;

  // Reset DOWNLIFT and deferred grasp state
  downlift_active_.store(false, std::memory_order_relaxed);
  downlift_elapsed_ = 0.0;
  deferred_grasp_pending_.store(false, std::memory_order_relaxed);

  // Reset force ramp state
  fr_f_current_ = 0.0;
  fr_iteration_ = 0;
  fr_z_initial_ = 0.0;
  fr_grasp_width_ = 0.0;
  fr_grasp_cmd_seen_executing_ = false;
  fr_grasp_stabilizing_ = false;
  fr_grasping_phase_initialized_ = false;

  logStateTransition("START", "Controller running");
  if (require_logger_) {
    ROS_INFO("    Step 1: Wait for Grasp logger to start");
    ROS_INFO("    Step 2: Publish BASELINE on /kitting_controller/state_cmd");
  } else {
    ROS_INFO("    Recording disabled — publish BASELINE on /kitting_controller/state_cmd");
  }
}

// ============================================================================
// update() decomposition: RT-safe helpers (no locks, no allocation)
// ============================================================================

void KittingStateController::applyPendingStateTransition() {
  if (!state_changed_.load(std::memory_order_acquire)) {
    return;
  }
  // acquire: guarantees visibility of all stores preceding the subscriber's
  // release-store of state_changed_ (including staging_fr_* parameters, fr_grasp_width_,
  // and pending_state_).
  GraspState new_state = pending_state_.load(std::memory_order_relaxed);

  // Handle BASELINE start: reset all state-machine variables for a fresh grasp cycle.
  // This enables multi-trial operation without restarting the controller.
  if (new_state == GraspState::BASELINE) {
    // Stop any active trajectories
    uplift_active_.store(false, std::memory_order_relaxed);
    uplift_elapsed_ = 0.0;
    downlift_active_.store(false, std::memory_order_relaxed);
    downlift_elapsed_ = 0.0;
    deferred_grasp_pending_.store(false, std::memory_order_relaxed);

    // Reset contact detection state
    contact_latched_ = false;
    baseline_armed_.store(false, std::memory_order_relaxed);
    baseline_collecting_ = false;
    baseline_sum_ = 0.0;
    baseline_sum_sq_ = 0.0;
    baseline_n_ = 0;

    // Reset debounce state
    arm_debounce_.reset();
    prev_tau_ext_valid_ = false;
    gripper_debounce_.reset();
    gripper_stop_sent_.store(false, std::memory_order_relaxed);
    contact_source_ = "";
    contact_width_.store(0.0, std::memory_order_relaxed);

    // Reset force ramp state
    fr_f_current_ = 0.0;
    fr_iteration_ = 0;
    fr_grasp_cmd_seen_executing_ = false;
    fr_grasp_stabilizing_ = false;
    fr_grasping_phase_initialized_ = false;

    ROS_INFO("  [BASELINE]  State machine reset for new trial");
  }

  // Handle CLOSING start: snapshot command parameters for RT contact detection
  if (new_state == GraspState::CLOSING) {
    rt_closing_w_cmd_ = closing_w_cmd_;
    rt_closing_v_cmd_ = closing_v_cmd_;
    rt_T_hold_gripper_ = computeGripperHoldTime(rt_closing_v_cmd_);
    gripper_debounce_.reset();
    gripper_stop_sent_.store(false, std::memory_order_relaxed);
    contact_source_ = "";
    ROS_INFO("  [CLOSING]  T_hold_gripper=%.3fs (from speed=%.4f m/s)",
             rt_T_hold_gripper_, rt_closing_v_cmd_);
  }

  // Handle GRASPING start: snapshot force ramp parameters, initialize force ramp state
  if (new_state == GraspState::GRASPING) {
    // Snapshot staging → RT-local copies (synchronized via state_changed_ acquire)
    rt_fr_f_min_ = staging_fr_f_min_;
    rt_fr_f_step_ = staging_fr_f_step_;
    rt_fr_f_max_ = staging_fr_f_max_;
    rt_fr_uplift_distance_ = staging_fr_uplift_distance_;
    rt_fr_lift_speed_ = staging_fr_lift_speed_;
    rt_fr_uplift_hold_ = staging_fr_uplift_hold_;
    rt_fr_grasp_speed_ = staging_fr_grasp_speed_;
    rt_fr_epsilon_ = staging_fr_epsilon_;
    rt_fr_stabilization_ = staging_fr_stabilization_;
    rt_fr_slip_tau_drop_ = staging_fr_slip_tau_drop_;
    rt_fr_slip_width_change_ = staging_fr_slip_width_change_;

    // Capture EE z-height before force ramp starts
    fr_z_initial_ = cartesian_pose_handle_->getRobotState().O_T_EE_d[14];

    // Initialize force ramp state
    fr_f_current_ = rt_fr_f_min_;
    fr_iteration_ = 0;
    rt_fr_grasp_width_ = fr_grasp_width_;
    fr_grasping_phase_initialized_ = false;  // Set on first runInternalTransitions tick
    fr_grasp_cmd_seen_executing_ = false;
    fr_grasp_stabilizing_ = false;

    // Clear trajectory and deferred state
    uplift_active_.store(false, std::memory_order_relaxed);
    downlift_active_.store(false, std::memory_order_relaxed);
    deferred_grasp_pending_.store(false, std::memory_order_relaxed);
  }

  current_state_.store(new_state, std::memory_order_relaxed);
  state_changed_.store(false, std::memory_order_relaxed);
}

void KittingStateController::updateCartesianCommand(const ros::Duration& period) {
  if (uplift_active_.load(std::memory_order_relaxed)) {
    uplift_elapsed_ += period.toSec();
    cartesian_pose_handle_->setCommand(computeUpliftPose(uplift_elapsed_));
    if (uplift_elapsed_ >= rt_uplift_duration_) {
      uplift_active_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: UPLIFT trajectory complete (%.3fs, %.4fm)",
               rt_uplift_duration_, rt_uplift_distance_);
    }
  } else if (downlift_active_.load(std::memory_order_relaxed)) {
    downlift_elapsed_ += period.toSec();
    cartesian_pose_handle_->setCommand(computeDownliftPose(downlift_elapsed_));
    if (downlift_elapsed_ >= rt_downlift_duration_) {
      downlift_active_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: DOWNLIFT trajectory complete (%.3fs, %.4fm)",
               rt_downlift_duration_, rt_downlift_distance_);
    }
  } else {
    // Passthrough: send current desired pose back as command (hold position).
    cartesian_pose_handle_->setCommand(cartesian_pose_handle_->getRobotState().O_T_EE_d);
  }
}

void KittingStateController::runContactDetection(const ros::Time& time,
                                                  double tau_ext_norm,
                                                  const GripperData& gripper_snapshot) {
  const GraspState state = current_state_.load(std::memory_order_relaxed);

  if (state == GraspState::BASELINE) {
    collectBaselineSamples(time, tau_ext_norm);
  }

  if (enable_arm_contact_ && state == GraspState::CLOSING &&
      baseline_armed_.load(std::memory_order_relaxed) && !contact_latched_) {
    detectArmContact(time, tau_ext_norm, gripper_snapshot);
  }

  if (enable_gripper_contact_ && state == GraspState::CLOSING && !contact_latched_) {
    detectGripperContact(time, gripper_snapshot);
  }
}

void KittingStateController::collectBaselineSamples(const ros::Time& time,
                                                     double tau_ext_norm) {
  // Wait for gripper open to complete before collecting baseline samples
  if (baseline_awaiting_gripper_) {
    double wait_elapsed = (time - baseline_gripper_wait_start_).toSec();
    if (wait_elapsed >= kGripperOpenWait) {
      baseline_awaiting_gripper_ = false;
      ROS_INFO("KittingStateController: Gripper open wait complete (%.1fs) "
               "— starting baseline collection", wait_elapsed);
    } else {
      return;
    }
  }

  if (!baseline_collecting_) {
    baseline_collecting_ = true;
    baseline_start_time_ = time;
    baseline_sum_ = 0.0;
    baseline_sum_sq_ = 0.0;
    baseline_n_ = 0;
  }

  double elapsed = (time - baseline_start_time_).toSec();
  if (baseline_armed_.load(std::memory_order_relaxed)) {
    return;
  }

  baseline_sum_ += tau_ext_norm;
  baseline_sum_sq_ += tau_ext_norm * tau_ext_norm;
  baseline_n_++;

  if (elapsed >= T_base_ && baseline_n_ >= N_min_) {
    baseline_mu_ = baseline_sum_ / baseline_n_;
    double variance =
        (baseline_sum_sq_ - baseline_sum_ * baseline_sum_ / baseline_n_) / (baseline_n_ - 1);
    baseline_sigma_ = std::sqrt(std::max(variance, 0.0));
    contact_threshold_ = baseline_mu_ + k_sigma_ * baseline_sigma_;
    baseline_armed_.store(true, std::memory_order_relaxed);

    // Signal the gripper read thread to publish baseline params (RT-safe: atomic store only)
    baseline_params_pending_.store(true, std::memory_order_release);

    ROS_INFO_STREAM("KittingStateController: Baseline computed | N=" << baseline_n_
                    << " mu=" << baseline_mu_ << " sigma=" << baseline_sigma_
                    << " theta=" << contact_threshold_
                    << " elapsed=" << elapsed << "s");
  }
}

void KittingStateController::detectArmContact(const ros::Time& time,
                                               double tau_ext_norm,
                                               const GripperData& gripper_snapshot) {
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
      threshold_exceeded = false;
    }
  }

  // Debounce: require sustained exceedance for T_hold_arm seconds
  if (arm_debounce_.check(threshold_exceeded, time, T_hold_arm_)) {
    double hold_elapsed = (time - arm_debounce_.start_time).toSec();
    contact_latched_ = true;
    contact_source_ = "ARM";
    contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);
    current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
    publishStateLabel("CONTACT");
    logStateTransition("CONTACT", "ARM torque contact detected!");
    ROS_INFO("    x(t)=%.4f  theta=%.4f  delta=%.4f  hold=%.3fs  width=%.4f",
             tau_ext_norm, contact_threshold_,
             tau_ext_norm - baseline_mu_, hold_elapsed,
             contact_width_.load(std::memory_order_relaxed));
    requestGripperStop("ARM");
  }

  // Update slope gate history
  prev_tau_ext_norm_ = tau_ext_norm;
  prev_tau_ext_time_ = time;
  prev_tau_ext_valid_ = true;
}

void KittingStateController::detectGripperContact(const ros::Time& time,
                                                    const GripperData& gripper_snapshot) {
  bool gripper_data_valid = (gripper_snapshot.stamp != ros::Time(0)) &&
                             ((time - gripper_snapshot.stamp).toSec() < 0.5);
  if (!gripper_data_valid) {
    return;
  }

  double w = gripper_snapshot.width;
  double w_dot = gripper_snapshot.width_dot;

  bool velocity_stalled = (std::abs(w_dot) < stall_velocity_threshold_);
  bool width_gap_exists = ((w - rt_closing_w_cmd_) > width_gap_threshold_);
  bool stall_detected = velocity_stalled && width_gap_exists;

  if (gripper_debounce_.check(stall_detected, time, rt_T_hold_gripper_)) {
    double hold_elapsed = (time - gripper_debounce_.start_time).toSec();
    contact_latched_ = true;
    contact_source_ = "GRIPPER";
    contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);
    current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
    publishStateLabel("CONTACT");
    logStateTransition("CONTACT", "GRIPPER stall contact detected!");
    ROS_INFO("    w=%.4f  w_cmd=%.4f  gap=%.4f  w_dot=%.6f  hold=%.3fs",
             w, rt_closing_w_cmd_, w - rt_closing_w_cmd_, w_dot, hold_elapsed);
    requestGripperStop("Stall");
  }
}

void KittingStateController::fillKittingStateMsg(
    const ros::Time& time,
    const franka::RobotState& robot_state,
    const std::array<double, 42>& jacobian,
    const std::array<double, 7>& gravity,
    const std::array<double, 7>& coriolis,
    const std::array<double, 6>& ee_velocity,
    double tau_ext_norm, double wrench_norm) {
  kitting_publisher_.msg_.header.stamp = time;

  std::copy(robot_state.q.begin(), robot_state.q.end(),
            kitting_publisher_.msg_.q.begin());
  std::copy(robot_state.dq.begin(), robot_state.dq.end(),
            kitting_publisher_.msg_.dq.begin());
  std::copy(robot_state.tau_J.begin(), robot_state.tau_J.end(),
            kitting_publisher_.msg_.tau_J.begin());
  std::copy(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.end(),
            kitting_publisher_.msg_.tau_ext.begin());
  std::copy(robot_state.O_F_ext_hat_K.begin(), robot_state.O_F_ext_hat_K.end(),
            kitting_publisher_.msg_.wrench_ext.begin());
  std::copy(robot_state.O_T_EE.begin(), robot_state.O_T_EE.end(),
            kitting_publisher_.msg_.O_T_EE.begin());
  std::copy(jacobian.begin(), jacobian.end(),
            kitting_publisher_.msg_.jacobian.begin());
  std::copy(gravity.begin(), gravity.end(),
            kitting_publisher_.msg_.gravity.begin());
  std::copy(coriolis.begin(), coriolis.end(),
            kitting_publisher_.msg_.coriolis.begin());
  std::copy(ee_velocity.begin(), ee_velocity.end(),
            kitting_publisher_.msg_.ee_velocity.begin());
  kitting_publisher_.msg_.tau_ext_norm = tau_ext_norm;
  kitting_publisher_.msg_.wrench_norm = wrench_norm;
}

// ============================================================================
// runInternalTransitions() — Force ramp state machine driver (250Hz)
// Called from update() when state is GRASPING/UPLIFT/EVALUATE/DOWNLIFT/SETTLING.
// All operations are RT-safe: atomic stores, timestamp arithmetic, trylock publish.
// ============================================================================

void KittingStateController::runInternalTransitions(const ros::Time& time,
                                                     double tau_ext_norm,
                                                     const GripperData& gripper_snapshot) {
  switch (current_state_.load(std::memory_order_relaxed)) {
    case GraspState::GRASPING:  tickGrasping(time, tau_ext_norm, gripper_snapshot); break;
    case GraspState::UPLIFT:    tickUplift(time, tau_ext_norm, gripper_snapshot);   break;
    case GraspState::EVALUATE:  tickEvaluate(time, tau_ext_norm, gripper_snapshot); break;
    case GraspState::DOWNLIFT:  tickDownlift(time, tau_ext_norm, gripper_snapshot); break;
    case GraspState::SETTLING:  tickSettling(time, tau_ext_norm, gripper_snapshot); break;
    default: break;
  }
}

void KittingStateController::tickGrasping(const ros::Time& time,
                                           double tau_ext_norm,
                                           const GripperData& gripper_snapshot) {
  // First tick after entering GRASPING: initialize phase timer
  if (!fr_grasping_phase_initialized_) {
    fr_phase_start_time_ = time;
    fr_grasping_phase_initialized_ = true;
  }

  double elapsed = (time - fr_phase_start_time_).toSec();

  // Step 1: Settle delay — wait for command thread to pick up the grasp command
  if (elapsed < kGraspSettleDelay) {
    return;
  }

  // Timeout: if grasp command doesn't complete in time, fail
  if (elapsed > kGraspTimeout) {
    current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
    publishStateLabel("FAILED");
    logStateTransition("FAILED", "GRASPING timeout");
    ROS_INFO("    elapsed=%.1fs  cmd_executing_=%s  seen_executing=%s  stabilizing=%s",
             elapsed,
             cmd_executing_.load(std::memory_order_relaxed) ? "true" : "false",
             fr_grasp_cmd_seen_executing_ ? "true" : "false",
             fr_grasp_stabilizing_ ? "true" : "false");
    return;
  }

  // Step 2: Poll cmd_executing_ — wait for grasp to start then complete
  if (!fr_grasp_stabilizing_) {
    bool executing = cmd_executing_.load(std::memory_order_relaxed);
    if (!fr_grasp_cmd_seen_executing_) {
      if (executing) {
        fr_grasp_cmd_seen_executing_ = true;
      }
      return;
    }
    if (executing) {
      return;
    }
    fr_grasp_stabilizing_ = true;
    fr_grasp_stabilize_start_ = time;
    return;
  }

  // Step 3: Post-grasp stabilization delay
  double stab_elapsed = (time - fr_grasp_stabilize_start_).toSec();
  if (stab_elapsed < rt_fr_stabilization_) {
    return;
  }

  // Step 4: Record reference signals before UPLIFT
  fr_tau_before_ = tau_ext_norm;
  fr_width_before_uplift_ = gripper_snapshot.width;

  // Step 5: Initialize UPLIFT trajectory
  uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  uplift_z_start_ = uplift_start_pose_[14];
  uplift_elapsed_ = 0.0;
  rt_uplift_distance_ = rt_fr_uplift_distance_;
  rt_uplift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
  uplift_active_.store(true, std::memory_order_relaxed);

  // Step 6: Transition to UPLIFT
  current_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
  publishStateLabel("UPLIFT");
  logStateTransition("UPLIFT", "Micro-lift starting");
  ROS_INFO("    iter=%d  F=%.1f N  tau_before=%.4f  width_before=%.4f  "
           "distance=%.4f  duration=%.2f",
           fr_iteration_, fr_f_current_, fr_tau_before_, fr_width_before_uplift_,
           rt_fr_uplift_distance_, rt_uplift_duration_);
}

void KittingStateController::tickUplift(const ros::Time& time,
                                         double tau_ext_norm,
                                         const GripperData& gripper_snapshot) {
  if (uplift_active_.load(std::memory_order_relaxed)) {
    return;  // Trajectory still running
  }

  fr_phase_start_time_ = time;
  fr_tau_min_during_ = tau_ext_norm;
  fr_max_width_during_eval_ = gripper_snapshot.width;

  current_state_.store(GraspState::EVALUATE, std::memory_order_relaxed);
  publishStateLabel("EVALUATE");
  logStateTransition("EVALUATE", "Hold + slip detection");
  ROS_INFO("    hold=%.2fs", rt_fr_uplift_hold_);
}

void KittingStateController::tickEvaluate(const ros::Time& time,
                                           double tau_ext_norm,
                                           const GripperData& gripper_snapshot) {
  double elapsed = (time - fr_phase_start_time_).toSec();
  double gw = gripper_snapshot.width;

  // Continuously track slip signals during hold
  if (tau_ext_norm < fr_tau_min_during_) {
    fr_tau_min_during_ = tau_ext_norm;
  }
  if (gw > fr_max_width_during_eval_) {
    fr_max_width_during_eval_ = gw;
  }

  if (elapsed < rt_fr_uplift_hold_) {
    return;  // Still in hold period
  }

  // Evaluate immediately after hold
  double tau_drop = 0.0;
  if (fr_tau_before_ > 1e-6) {
    tau_drop = (fr_tau_before_ - fr_tau_min_during_) / fr_tau_before_;
  }
  double width_change = fr_max_width_during_eval_ - fr_width_before_uplift_;

  bool slip = (tau_drop > rt_fr_slip_tau_drop_) ||
              (width_change > rt_fr_slip_width_change_);

  ROS_INFO("  [EVALUATE]  tau_drop=%.3f (thresh=%.3f)  width_change=%.4f (thresh=%.4f)  %s",
           tau_drop, rt_fr_slip_tau_drop_, width_change, rt_fr_slip_width_change_,
           slip ? "SLIP" : "STABLE");

  if (!slip) {
    current_state_.store(GraspState::SUCCESS, std::memory_order_relaxed);
    publishStateLabel("SUCCESS");
    logStateTransition("SUCCESS", "Stable grasp confirmed");
    ROS_INFO("    F=%.1f N  iter=%d  tau_drop=%.3f  width_change=%.4f",
             fr_f_current_, fr_iteration_, tau_drop, width_change);
  } else {
    // Initialize DOWNLIFT trajectory
    downlift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    downlift_z_start_ = downlift_start_pose_[14];
    downlift_elapsed_ = 0.0;
    rt_downlift_distance_ = rt_fr_uplift_distance_;
    rt_downlift_duration_ = rt_fr_uplift_distance_ / rt_fr_lift_speed_;
    downlift_active_.store(true, std::memory_order_relaxed);

    current_state_.store(GraspState::DOWNLIFT, std::memory_order_relaxed);
    publishStateLabel("DOWNLIFT");
    logStateTransition("DOWNLIFT", "Returning before force increment");
    ROS_INFO("    distance=%.4f m  duration=%.2f s",
             rt_downlift_distance_, rt_downlift_duration_);
  }
}

void KittingStateController::tickDownlift(const ros::Time& time,
                                           double /* tau_ext_norm */,
                                           const GripperData& /* gripper_snapshot */) {
  if (downlift_active_.load(std::memory_order_relaxed)) {
    return;  // Trajectory still running
  }

  fr_phase_start_time_ = time;
  current_state_.store(GraspState::SETTLING, std::memory_order_relaxed);
  publishStateLabel("SETTLING");
  ROS_INFO("  [STATE]  >>  SETTLING  <<  Post-downlift stabilization (%.2fs)",
           rt_fr_stabilization_);
}

void KittingStateController::tickSettling(const ros::Time& time,
                                           double /* tau_ext_norm */,
                                           const GripperData& /* gripper_snapshot */) {
  double elapsed = (time - fr_phase_start_time_).toSec();
  if (elapsed < rt_fr_stabilization_) {
    return;  // Still settling
  }

  // Increment force for next iteration
  fr_f_current_ += rt_fr_f_step_;
  fr_iteration_++;

  if (fr_f_current_ > rt_fr_f_max_) {
    current_state_.store(GraspState::FAILED, std::memory_order_relaxed);
    publishStateLabel("FAILED");
    logStateTransition("FAILED", "Max force exceeded");
    ROS_INFO("    f_current=%.1f N > f_max=%.1f N  after %d iterations",
             fr_f_current_, rt_fr_f_max_, fr_iteration_);
    return;
  }

  // Re-enter GRASPING with incremented force
  fr_phase_start_time_ = time;
  fr_grasping_phase_initialized_ = true;
  fr_grasp_cmd_seen_executing_ = false;
  fr_grasp_stabilizing_ = false;
  requestDeferredGrasp(rt_fr_grasp_width_, rt_fr_grasp_speed_,
                       fr_f_current_, rt_fr_epsilon_);
  current_state_.store(GraspState::GRASPING, std::memory_order_relaxed);
  publishStateLabel("GRASPING");
  logStateTransition("GRASPING", "Force ramp retry");
  ROS_INFO("    iter=%d  F=%.1f N", fr_iteration_, fr_f_current_);
}

// ============================================================================
// update() — 1kHz RT loop (state transitions + Cartesian command every tick,
//            contact detection + state publishing at 250Hz)
// ============================================================================

void KittingStateController::update(const ros::Time& time, const ros::Duration& period) {
  applyPendingStateTransition();
  updateCartesianCommand(period);

  if (rate_trigger_()) {
    franka::RobotState robot_state = franka_state_handle_->getRobotState();
    std::array<double, 42> jacobian =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 7> gravity = model_handle_->getGravity();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();

    double tau_ext_norm = arrayNorm(robot_state.tau_ext_hat_filtered);
    double wrench_norm = arrayNorm(robot_state.O_F_ext_hat_K);

    std::array<double, 6> ee_velocity{};
    for (size_t row = 0; row < 6; ++row) {
      for (size_t col = 0; col < 7; ++col) {
        ee_velocity[row] += jacobian[col * 6 + row] * robot_state.dq[col];
      }
    }

    // Single gripper snapshot per tick — all consumers see consistent data
    const GripperData gripper_snapshot = *gripper_data_buf_.readFromRT();

    if (enable_contact_detector_) {
      runContactDetection(time, tau_ext_norm, gripper_snapshot);
    }

    // Run internal force ramp transitions (GRASPING→UPLIFT→EVALUATE→...)
    {
      GraspState fr_state = current_state_.load(std::memory_order_relaxed);
      if (fr_state == GraspState::GRASPING || fr_state == GraspState::UPLIFT ||
          fr_state == GraspState::EVALUATE || fr_state == GraspState::DOWNLIFT ||
          fr_state == GraspState::SETTLING) {
        runInternalTransitions(time, tau_ext_norm, gripper_snapshot);
      }
    }

    // Signal monitor — 2 Hz slow-rate logger for terminal readability
    // Re-read state since runInternalTransitions may have changed it
    const GraspState state = current_state_.load(std::memory_order_relaxed);
    if (signal_log_trigger_()) {
      bool armed = baseline_armed_.load(std::memory_order_relaxed);
      if (state == GraspState::CLOSING) {
        double arm_margin = armed ? (tau_ext_norm - contact_threshold_) : 0.0;
        double grip_gap = gripper_snapshot.width - rt_closing_w_cmd_;
        bool grip_stall = (std::abs(gripper_snapshot.width_dot) < stall_velocity_threshold_) &&
                           (grip_gap > width_gap_threshold_);
        ROS_INFO("  [SIGNAL]  CLOSING  |  arm: x=%.4f theta=%.4f margin=%.4f %s  |  "
                 "grip: w=%.4f w_cmd=%.4f gap=%.4f w_dot=%.6f %s",
                 tau_ext_norm,
                 armed ? contact_threshold_ : 0.0,
                 arm_margin,
                 (armed && arm_margin > 0.0) ? "ABOVE" : "below",
                 gripper_snapshot.width, rt_closing_w_cmd_, grip_gap,
                 gripper_snapshot.width_dot,
                 grip_stall ? "STALL" : "moving");
      } else if (armed && (state == GraspState::CONTACT ||
                 state == GraspState::GRASPING ||
                 state == GraspState::UPLIFT ||
                 state == GraspState::EVALUATE ||
                 state == GraspState::DOWNLIFT ||
                 state == GraspState::SETTLING)) {
        ROS_INFO("  [SIGNAL]  %s  |  x(t)=%.4f  theta=%.4f  margin=+%.4f  F=%.1f  iter=%d",
                 stateToString(state),
                 tau_ext_norm, contact_threshold_,
                 tau_ext_norm - contact_threshold_,
                 fr_f_current_, fr_iteration_);
      }
    }

    // Gripper velocity — 10 Hz during CLOSING (DEBUG level to minimize RT allocation)
    if (state == GraspState::CLOSING && gripper_log_trigger_()) {
      ROS_DEBUG("  [GRIP]  w=%.5f  w_dot=%.6f m/s  gap=%.5f  %s",
                gripper_snapshot.width,
                gripper_snapshot.width_dot,
                gripper_snapshot.width - rt_closing_w_cmd_,
                (std::abs(gripper_snapshot.width_dot) < stall_velocity_threshold_) ? "SLOW" : "ok");
    }

    if (kitting_publisher_.trylock()) {
      fillKittingStateMsg(time, robot_state, jacobian, gravity, coriolis,
                          ee_velocity, tau_ext_norm, wrench_norm);
      kitting_publisher_.unlockAndPublish();
    }
  }
}

}  // namespace franka_kitting_controller

PLUGINLIB_EXPORT_CLASS(franka_kitting_controller::KittingStateController,
                       controller_interface::ControllerBase)
