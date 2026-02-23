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

// C++14 requires out-of-line definitions for static constexpr members
// that are ODR-used (e.g. passed by reference to std::min/std::max).
constexpr double KittingStateController::kMaxClosingSpeed;
constexpr double KittingStateController::kGripperHoldBase;
constexpr double KittingStateController::kGripperHoldSlope;
constexpr double KittingStateController::kMaxUpliftDistance;

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
    case GraspState::SECURE_GRASP:
      return "SECURE_GRASP";
    case GraspState::UPLIFT:
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

void KittingStateController::requestGripperStop(const char* source) {
  if (stop_on_contact_ && !gripper_stop_sent_.load(std::memory_order_relaxed) &&
      execute_gripper_actions_) {
    stop_requested_.store(true, std::memory_order_relaxed);
    gripper_stop_sent_.store(true, std::memory_order_relaxed);
    ROS_INFO("  [GRIPPER]  %s contact -> stop() requested", source);
  }
}

double KittingStateController::computeGripperHoldTime(double closing_speed) {
  double clamped = std::min(std::max(closing_speed, 0.0), kMaxClosingSpeed);
  return kGripperHoldBase + kGripperHoldSlope * clamped;
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
      ROS_INFO("  [READY]  Grasp logger detected — commands now accepted");
      ROS_INFO("============================================================");
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
  // Listens to /kitting_controller/state_cmd for BASELINE, CLOSING, SECURE_GRASP,
  // and UPLIFT commands with optional per-object parameters.
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
    // --- Optional gripper open before baseline collection ---
    if (msg->open_gripper && execute_gripper_actions_) {
      double open_w = (msg->open_width > 0.0) ? msg->open_width
                     : gripper_data_buf_.readFromNonRT()->max_width;
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
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  BASELINE  <<  Collecting reference signals");
    ROS_INFO("============================================================");
    return;

  } else if (cmd == "CLOSING") {
    if (current_state_.load(std::memory_order_relaxed) == GraspState::CLOSING) {
      ROS_DEBUG("KittingStateController: Already in CLOSING, ignoring duplicate");
      return;
    }

    // Warn if baseline not yet armed — arm contact detector won't work
    if (enable_arm_contact_ && !baseline_armed_.load(std::memory_order_relaxed)) {
      ROS_WARN("KittingStateController: Baseline not yet armed — arm torque contact detection "
               "will NOT work for this closing. Only gripper stall detection is active.");
    }

    // Resolve per-command parameters (0 = use YAML default)
    double width = (msg->closing_width > 0.0) ? msg->closing_width : closing_width_;
    double speed = (msg->closing_speed > 0.0) ? msg->closing_speed : closing_speed_;

    // Clamp closing speed to safety limit
    if (speed > kMaxClosingSpeed) {
      ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
               speed, kMaxClosingSpeed);
      speed = kMaxClosingSpeed;
    }

    // Queue gripper move command (non-blocking — executed by command thread)
    if (execute_gripper_actions_) {
      GripperCommand gripper_cmd;
      gripper_cmd.type = GripperCommandType::MOVE;
      gripper_cmd.width = width;
      gripper_cmd.speed = speed;
      queueGripperCommand(gripper_cmd);
      ROS_INFO("KittingStateController: Queued move(width=%.4f, speed=%.4f)",
               width, speed);
    }

    // Store resolved CLOSING parameters for RT contact detection.
    // The release-store on state_changed_ below guarantees the RT thread
    // sees these values after its acquire-load returns true.
    closing_w_cmd_ = width;
    closing_v_cmd_ = speed;

    pending_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("CLOSING");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  CLOSING  <<  Gripper approaching object");
    ROS_INFO("    width=%.4f m  speed=%.4f m/s", width, speed);
    ROS_INFO("============================================================");

  } else if (cmd == "SECURE_GRASP") {
    // Resolve per-command parameters:
    //   grasp_width:  message override > contact width > YAML default
    //   epsilon:      unified field > individual fields > YAML default
    double cw = contact_width_.load(std::memory_order_relaxed);
    double width = (msg->grasp_width > 0.0) ? msg->grasp_width
                 : (cw > 0.0)               ? cw
                 :                             grasp_width_;
    double eps_in = (msg->epsilon > 0.0)       ? msg->epsilon
                  : (msg->epsilon_inner > 0.0) ? msg->epsilon_inner
                  :                               epsilon_inner_;
    double eps_out = (msg->epsilon > 0.0)       ? msg->epsilon
                   : (msg->epsilon_outer > 0.0) ? msg->epsilon_outer
                   :                               epsilon_outer_;
    double speed = (msg->grasp_speed > 0.0) ? msg->grasp_speed : grasp_speed_;
    double force = (msg->grasp_force > 0.0) ? msg->grasp_force : grasp_force_;

    if (msg->grasp_width == 0.0 && cw > 0.0) {
      ROS_INFO("KittingStateController: Using contact width %.4f m as grasp_width", cw);
    }

    // Queue gripper grasp command (non-blocking — executed by command thread)
    if (execute_gripper_actions_) {
      GripperCommand gripper_cmd;
      gripper_cmd.type = GripperCommandType::GRASP;
      gripper_cmd.width = width;
      gripper_cmd.speed = speed;
      gripper_cmd.force = force;
      gripper_cmd.epsilon_inner = eps_in;
      gripper_cmd.epsilon_outer = eps_out;
      queueGripperCommand(gripper_cmd);
      ROS_INFO("KittingStateController: Queued grasp(width=%.4f, eps=%.4f/%.4f, "
               "speed=%.4f, force=%.1f)",
               width, eps_in, eps_out, speed, force);
    }

    pending_state_.store(GraspState::SECURE_GRASP, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("SECURE_GRASP");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  SECURE_GRASP  <<  Grasping object");
    ROS_INFO("    width=%.4f m  force=%.1f N  speed=%.4f m/s", width, force, speed);
    ROS_INFO("============================================================");

  } else if (cmd == "UPLIFT") {
    // --- Precondition: require SECURE_GRASP or UPLIFT state if configured ---
    if (require_secure_grasp_) {
      GraspState cur = current_state_.load(std::memory_order_relaxed);
      if (cur != GraspState::SECURE_GRASP && cur != GraspState::UPLIFT) {
        ROS_WARN("KittingStateController: UPLIFT rejected — require_secure_grasp is true "
                 "but current state is %s (expected SECURE_GRASP or UPLIFT)",
                 stateToString(cur));
        return;
      }
    }

    // If UPLIFT is already active, stop current trajectory so it restarts from current position
    if (uplift_active_.load(std::memory_order_relaxed)) {
      uplift_active_.store(false, std::memory_order_relaxed);
      ROS_INFO("KittingStateController: Restarting UPLIFT from current position");
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
    // Written here (subscriber thread), read by RT thread in the state_changed_ block.
    // Safety: the release-store on state_changed_ below creates a happens-before edge,
    // guaranteeing that when the RT thread's acquire-load of state_changed_ returns true,
    // it sees these stores. The RT thread then copies them to rt_uplift_distance_ and
    // rt_uplift_duration_ for trajectory isolation.
    uplift_distance_ = distance;
    uplift_duration_ = duration;

    pending_state_.store(GraspState::UPLIFT, std::memory_order_relaxed);
    state_changed_.store(true, std::memory_order_release);
    publishStateLabel("UPLIFT");
    ROS_INFO("============================================================");
    ROS_INFO("  [STATE]  >>  UPLIFT  <<  Cartesian micro-lift");
    ROS_INFO("    distance=%.4f m  duration=%.2f s", distance, duration);
    ROS_INFO("============================================================");

  } else {
    ROS_WARN("KittingStateController: Unknown command '%s' on state_cmd "
             "(expected BASELINE, CLOSING, SECURE_GRASP, or UPLIFT)", cmd.c_str());
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

  // --- Grasp: Load arm contact detector parameters ---
  node_handle.param("enable_contact_detector", enable_contact_detector_, true);
  node_handle.param("T_base", T_base_, 0.7);
  node_handle.param("N_min", N_min_, 50);
  node_handle.param("k_sigma", k_sigma_, 3.0);
  node_handle.param("T_hold_arm", T_hold_arm_, 0.10);
  node_handle.param("use_slope_gate", use_slope_gate_, false);
  node_handle.param("slope_min", slope_min_, 5.0);

  // --- Gripper contact detection parameters ---
  node_handle.param("stall_velocity_threshold", stall_velocity_threshold_, 0.005);
  node_handle.param("width_gap_threshold", width_gap_threshold_, 0.002);
  node_handle.param("stop_on_contact", stop_on_contact_, true);
  node_handle.param("enable_arm_contact", enable_arm_contact_, true);
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
  node_handle.param("closing_width", closing_width_, 0.04);
  node_handle.param("closing_speed", closing_speed_, 0.04);
  if (closing_speed_ > kMaxClosingSpeed) {
    ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
             closing_speed_, kMaxClosingSpeed);
    closing_speed_ = kMaxClosingSpeed;
  }
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

  // --- Grasp: UPLIFT parameters (overridable per-command via KittingGripperCommand) ---
  node_handle.param("uplift_distance", uplift_distance_, 0.003);
  node_handle.param("uplift_duration", uplift_duration_, 1.0);
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
                   << "m duration=" << uplift_duration_ << "s (world-frame z)"
                   << " require_secure_grasp=" << (require_secure_grasp_ ? "true" : "false"));

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
  // with command (BASELINE/CLOSING/SECURE_GRASP/UPLIFT) + optional per-object parameters.
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

  ROS_INFO("============================================================");
  ROS_INFO("  [STATE]  >>  START  <<  Controller running");
  if (require_logger_) {
    ROS_INFO("    Step 1: Wait for Grasp logger to start");
    ROS_INFO("    Step 2: Publish BASELINE on /kitting_controller/state_cmd");
  } else {
    ROS_INFO("    Recording disabled — publish BASELINE on /kitting_controller/state_cmd");
  }
  ROS_INFO("============================================================");
}

// ============================================================================
// update() decomposition: RT-safe helpers (no locks, no allocation)
// ============================================================================

void KittingStateController::applyPendingStateTransition() {
  if (!state_changed_.load(std::memory_order_acquire)) {
    return;
  }
  // acquire: guarantees visibility of all stores preceding the subscriber's
  // release-store of state_changed_ (including uplift_distance_, uplift_duration_,
  // and pending_state_).
  GraspState new_state = pending_state_.load(std::memory_order_relaxed);

  // Handle BASELINE start: reset all state-machine variables for a fresh grasp cycle.
  // This enables multi-trial operation without restarting the controller.
  if (new_state == GraspState::BASELINE) {
    // Stop any active UPLIFT trajectory
    uplift_active_.store(false, std::memory_order_relaxed);
    uplift_elapsed_ = 0.0;

    // Reset contact detection state
    contact_latched_ = false;
    baseline_armed_.store(false, std::memory_order_relaxed);
    baseline_collecting_ = false;
    baseline_sum_ = 0.0;
    baseline_sum_sq_ = 0.0;
    baseline_n_ = 0;

    // Reset arm debounce
    exceeding_ = false;
    prev_tau_ext_valid_ = false;

    // Reset gripper debounce
    gripper_stall_active_ = false;
    gripper_stop_sent_.store(false, std::memory_order_relaxed);
    contact_source_ = "";
    contact_width_.store(0.0, std::memory_order_relaxed);

    ROS_INFO("  [BASELINE]  State machine reset for new trial");
  }

  // Handle CLOSING start: snapshot command parameters for RT contact detection
  if (new_state == GraspState::CLOSING) {
    rt_closing_w_cmd_ = closing_w_cmd_;
    rt_closing_v_cmd_ = closing_v_cmd_;
    rt_T_hold_gripper_ = computeGripperHoldTime(rt_closing_v_cmd_);
    gripper_stall_active_ = false;
    gripper_stop_sent_.store(false, std::memory_order_relaxed);
    contact_source_ = "";
    ROS_INFO("  [CLOSING]  T_hold_gripper=%.3fs (from speed=%.4f m/s)",
             rt_T_hold_gripper_, rt_closing_v_cmd_);
  }

  // Handle UPLIFT start (or restart): capture current desired pose, snapshot params, reset timer
  if (new_state == GraspState::UPLIFT) {
    uplift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    uplift_z_start_ = uplift_start_pose_[14];
    uplift_elapsed_ = 0.0;
    // Snapshot parameters into RT-local copies — isolates trajectory from
    // any concurrent subscriber writes to uplift_distance_/uplift_duration_.
    rt_uplift_distance_ = uplift_distance_;
    rt_uplift_duration_ = uplift_duration_;
    uplift_active_.store(true, std::memory_order_relaxed);
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
  } else {
    // Passthrough: send current desired pose back as command (hold position).
    cartesian_pose_handle_->setCommand(cartesian_pose_handle_->getRobotState().O_T_EE_d);
  }
}

void KittingStateController::runContactDetection(const ros::Time& time,
                                                  double tau_ext_norm,
                                                  const GripperData& gripper_snapshot) {
  const GraspState state = current_state_.load(std::memory_order_relaxed);

  // --- BASELINE: collect samples ---
  if (state == GraspState::BASELINE) {
    // Wait for gripper open to complete before collecting baseline samples
    if (baseline_awaiting_gripper_) {
      double wait_elapsed = (time - baseline_gripper_wait_start_).toSec();
      if (wait_elapsed >= 3.0) {
        baseline_awaiting_gripper_ = false;
        ROS_INFO("KittingStateController: Gripper open wait complete (%.1fs) "
                 "— starting baseline collection", wait_elapsed);
      } else {
        return;  // Still waiting, skip collection this tick
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
    if (!baseline_armed_.load(std::memory_order_relaxed)) {
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

        // Signal the gripper read thread to publish baseline params (RT-safe: atomic store only).
        // The actual ros::param::set() calls happen in gripperReadLoop() (non-RT).
        baseline_params_pending_.store(true, std::memory_order_release);

        ROS_INFO_STREAM("KittingStateController: Baseline computed | N=" << baseline_n_
                        << " mu=" << baseline_mu_ << " sigma=" << baseline_sigma_
                        << " theta=" << contact_threshold_
                        << " elapsed=" << elapsed << "s");
      }
    }
  }

  // --- CLOSING: detect arm torque contact ---
  if (enable_arm_contact_ && state == GraspState::CLOSING &&
      baseline_armed_.load(std::memory_order_relaxed) && !contact_latched_) {
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
    if (threshold_exceeded) {
      if (!exceeding_) {
        exceeding_ = true;
        exceed_start_time_ = time;
      } else {
        double hold_elapsed = (time - exceed_start_time_).toSec();
        if (hold_elapsed >= T_hold_arm_) {
          contact_latched_ = true;
          contact_source_ = "ARM";
          contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);
          current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
          publishStateLabel("CONTACT");
          ROS_INFO("============================================================");
          ROS_INFO("  [STATE]  >>  CONTACT  <<  ARM torque contact detected!");
          ROS_INFO("    x(t)=%.4f  theta=%.4f  delta=%.4f  hold=%.3fs  width=%.4f",
                   tau_ext_norm, contact_threshold_,
                   tau_ext_norm - baseline_mu_, hold_elapsed,
                   contact_width_.load(std::memory_order_relaxed));
          ROS_INFO("============================================================");
          requestGripperStop("ARM");
        }
      }
    } else {
      exceeding_ = false;
    }

    // Update slope gate history
    prev_tau_ext_norm_ = tau_ext_norm;
    prev_tau_ext_time_ = time;
    prev_tau_ext_valid_ = true;
  }

  // --- CLOSING: detect gripper stall contact ---
  if (enable_gripper_contact_ && state == GraspState::CLOSING && !contact_latched_) {
    bool gripper_data_valid = (gripper_snapshot.stamp != ros::Time(0)) &&
                               ((time - gripper_snapshot.stamp).toSec() < 0.5);

    if (gripper_data_valid) {
      double w = gripper_snapshot.width;
      double w_dot = gripper_snapshot.width_dot;

      // Stall condition: fingers nearly stopped AND not yet at commanded width.
      bool velocity_stalled = (std::abs(w_dot) < stall_velocity_threshold_);
      bool width_gap_exists = ((w - rt_closing_w_cmd_) > width_gap_threshold_);
      bool stall_detected = velocity_stalled && width_gap_exists;

      // Debounce: require sustained stall for rt_T_hold_gripper_ seconds (speed-dependent)
      if (stall_detected) {
        if (!gripper_stall_active_) {
          gripper_stall_active_ = true;
          gripper_stall_start_time_ = time;
        } else {
          double hold_elapsed = (time - gripper_stall_start_time_).toSec();
          if (hold_elapsed >= rt_T_hold_gripper_) {
            contact_latched_ = true;
            contact_source_ = "GRIPPER";
            contact_width_.store(gripper_snapshot.width, std::memory_order_relaxed);
            current_state_.store(GraspState::CONTACT, std::memory_order_relaxed);
            publishStateLabel("CONTACT");
            ROS_INFO("============================================================");
            ROS_INFO("  [STATE]  >>  CONTACT  <<  GRIPPER stall contact detected!");
            ROS_INFO("    w=%.4f  w_cmd=%.4f  gap=%.4f  w_dot=%.6f  hold=%.3fs",
                     w, rt_closing_w_cmd_, w - rt_closing_w_cmd_, w_dot, hold_elapsed);
            ROS_INFO("============================================================");
            requestGripperStop("Stall");
          }
        }
      } else {
        gripper_stall_active_ = false;
      }
    }
  }
}

void KittingStateController::fillKittingStateMsg(
    const ros::Time& time,
    const franka::RobotState& robot_state,
    const std::array<double, 42>& jacobian,
    const std::array<double, 7>& gravity,
    const std::array<double, 7>& coriolis,
    const std::array<double, 6>& ee_velocity,
    double tau_ext_norm, double wrench_norm,
    const GripperData& gripper_snapshot,
    GraspState state) {
  kitting_publisher_.msg_.header.stamp = time;

  for (size_t i = 0; i < 7; ++i) {
    kitting_publisher_.msg_.q[i] = robot_state.q[i];
    kitting_publisher_.msg_.dq[i] = robot_state.dq[i];
    kitting_publisher_.msg_.tau_J[i] = robot_state.tau_J[i];
    kitting_publisher_.msg_.tau_ext[i] = robot_state.tau_ext_hat_filtered[i];
  }

  for (size_t i = 0; i < 6; ++i) {
    kitting_publisher_.msg_.wrench_ext[i] = robot_state.O_F_ext_hat_K[i];
  }
  for (size_t i = 0; i < 16; ++i) {
    kitting_publisher_.msg_.O_T_EE[i] = robot_state.O_T_EE[i];
  }

  for (size_t i = 0; i < 42; ++i) {
    kitting_publisher_.msg_.jacobian[i] = jacobian[i];
  }
  for (size_t i = 0; i < 7; ++i) {
    kitting_publisher_.msg_.gravity[i] = gravity[i];
    kitting_publisher_.msg_.coriolis[i] = coriolis[i];
  }

  for (size_t i = 0; i < 6; ++i) {
    kitting_publisher_.msg_.ee_velocity[i] = ee_velocity[i];
  }
  kitting_publisher_.msg_.tau_ext_norm = tau_ext_norm;
  kitting_publisher_.msg_.wrench_norm = wrench_norm;

  kitting_publisher_.msg_.gripper_width = gripper_snapshot.width;
  kitting_publisher_.msg_.gripper_width_dot = gripper_snapshot.width_dot;
  kitting_publisher_.msg_.gripper_width_cmd =
      (state == GraspState::CLOSING) ? rt_closing_w_cmd_ : 0.0;
  kitting_publisher_.msg_.gripper_max_width = gripper_snapshot.max_width;
  kitting_publisher_.msg_.gripper_is_grasped = gripper_snapshot.is_grasped;
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

    double tau_ext_norm = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      tau_ext_norm += robot_state.tau_ext_hat_filtered[i] * robot_state.tau_ext_hat_filtered[i];
    }
    tau_ext_norm = std::sqrt(tau_ext_norm);

    double wrench_norm = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      wrench_norm += robot_state.O_F_ext_hat_K[i] * robot_state.O_F_ext_hat_K[i];
    }
    wrench_norm = std::sqrt(wrench_norm);

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

    // Signal monitor — 2 Hz slow-rate logger for terminal readability
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
                 state == GraspState::SECURE_GRASP ||
                 state == GraspState::UPLIFT)) {
        ROS_INFO("  [SIGNAL]  %s  |  x(t)=%.4f  theta=%.4f  margin=+%.4f",
                 stateToString(state),
                 tau_ext_norm, contact_threshold_,
                 tau_ext_norm - contact_threshold_);
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
                          ee_velocity, tau_ext_norm, wrench_norm, gripper_snapshot, state);
      kitting_publisher_.unlockAndPublish();
    }
  }
}

}  // namespace franka_kitting_controller

PLUGINLIB_EXPORT_CLASS(franka_kitting_controller::KittingStateController,
                       controller_interface::ControllerBase)
