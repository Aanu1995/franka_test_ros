// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//

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
  constexpr double KittingStateController::kMinUpliftHold;
  constexpr double KittingStateController::kGraspSettleDelay;
  constexpr double KittingStateController::kGraspTimeout;

  // ============================================================================
  // Utilities (used by all translation units)
  // ============================================================================

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

  void KittingStateController::logStateTransition(const char* label, const char* detail) {
    ROS_INFO("============================================================");
    if (detail) {
      ROS_INFO("  [STATE]  >>  %s  <<  %s", label, detail);
    } else {
      ROS_INFO("  [STATE]  >>  %s  <<", label);
    }
    ROS_INFO("============================================================");
  }

  void KittingStateController::requestGripperStop(const char* source) {
    if (!gripper_stop_sent_.load(std::memory_order_relaxed)) {
      stop_requested_.store(true, std::memory_order_relaxed);
      gripper_stop_sent_.store(true, std::memory_order_relaxed);
      ROS_INFO("  [GRIPPER]  %s contact -> stop() requested", source);
    }
  }

  double KittingStateController::computeGripperHoldTime(double closing_speed) {
    double clamped = std::min(std::max(closing_speed, 0.0), kMaxClosingSpeed);
    return kGripperHoldBase + kGripperHoldSlope * clamped;
  }

  // ============================================================================
  // Trajectory math
  // ============================================================================

  std::array<double, 16> KittingStateController::computeUpliftPose(double elapsed) const {
    std::array<double, 16> pose = uplift_start_pose_;
    // Uses Realtime-local copies (rt_uplift_duration_, rt_uplift_distance_) to prevent
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

  // ============================================================================
  // Destructor
  // ============================================================================

  KittingStateController::~KittingStateController() {
    cancelAutoMode();

    // Shutdown action servers before stopping gripper threads
    if (move_action_server_) move_action_server_->shutdown();
    if (grasp_action_server_) grasp_action_server_->shutdown();
    if (homing_action_server_) homing_action_server_->shutdown();
    if (stop_action_server_) stop_action_server_->shutdown();

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

  // ============================================================================
  // init() — Controller lifecycle initialization
  // ============================================================================

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

    // --- Grasp: Load contact detection parameters (gripper stall detection, always enabled) ---
    node_handle.param("stall_velocity_threshold", stall_velocity_threshold_, 0.008);
    node_handle.param("width_gap_threshold", width_gap_threshold_, 0.002);

    ROS_INFO_STREAM("KittingStateController: Contact detector ENABLED"
                    << " | stall_vel=" << stall_velocity_threshold_
                    << " gap=" << width_gap_threshold_ << " T_hold_grip=dynamic(speed)");

    // --- Grasp: Gripper default parameters (overridable per-command via KittingGripperCommand) ---
    node_handle.param("closing_width", closing_width_, 0.01);
    node_handle.param("closing_speed", closing_speed_, 0.05);
    if (closing_speed_ > kMaxClosingSpeed) {
      ROS_WARN("KittingStateController: closing_speed %.4f exceeds max %.4f, clamping",
              closing_speed_, kMaxClosingSpeed);
      closing_speed_ = kMaxClosingSpeed;
    }
    ROS_INFO_STREAM("KittingStateController: Gripper"
                    << " | closing: width=" << closing_width_ << " speed=" << closing_speed_
                    << " | grasp width: from contact_width (auto)");

    // --- GRASPING: Force ramp parameters (overridable per-command via KittingGripperCommand) ---
    node_handle.param("f_min", fr_f_min_, 3.0);
    node_handle.param("f_step", fr_f_step_, 3.0);
    node_handle.param("f_max", fr_f_max_, 30.0);
    node_handle.param("uplift_distance", fr_uplift_distance_, 0.010);
    node_handle.param("lift_speed", fr_lift_speed_, 0.01);
    node_handle.param("uplift_hold", fr_uplift_hold_, 1.0);
    node_handle.param("grasp_speed", fr_grasp_speed_, 0.02);
    node_handle.param("epsilon", fr_epsilon_, 0.008);
    node_handle.param("slip_drop_thresh", fr_slip_drop_thresh_, 0.15);
    node_handle.param("slip_width_thresh", fr_slip_width_thresh_, 0.0005);
    node_handle.param("load_transfer_min", fr_load_transfer_min_, 1.0);
    node_handle.param("slip_score_thresh", fr_slip_score_thresh_, 0.6);
    node_handle.param("slip_friction_thresh", fr_slip_friction_thresh_, 0.5);

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
    if (fr_uplift_hold_ < kMinUpliftHold) {
      ROS_WARN("KittingStateController: uplift_hold %.2f below min %.2f, clamping",
              fr_uplift_hold_, kMinUpliftHold);
      fr_uplift_hold_ = kMinUpliftHold;
    }

    ROS_INFO_STREAM("KittingStateController: Force ramp config"
                    << " | f: min=" << fr_f_min_ << " step=" << fr_f_step_
                    << " max=" << fr_f_max_
                    << " | uplift: dist=" << fr_uplift_distance_
                    << " speed=" << fr_lift_speed_ << " hold=" << fr_uplift_hold_
                    << " | grasp: speed=" << fr_grasp_speed_ << " eps=" << fr_epsilon_
                    << " | slip: DF_TH=" << fr_slip_drop_thresh_
                    << " W_TH=" << fr_slip_width_thresh_
                    << " U_TH=" << fr_slip_friction_thresh_
                    << " S_TH=" << fr_slip_score_thresh_
                    << " load_min=" << fr_load_transfer_min_);

    // --- Direct gripper connection via libfranka ---
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
    auto_nh_ = root_nh;
    state_publisher_.init(root_nh, "/kitting_controller/state", 1);

    // Grasp: Logger readiness gate.
    // When require_logger is true, commands are rejected until the logger publishes ready.
    // When false, commands are accepted immediately (no recording).
    node_handle.param("require_logger", require_logger_, false);
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

    // --- Gripper action servers (franka_gripper-compatible API) ---
    move_action_server_ = std::make_unique<MoveActionServer>(
        root_nh, "/franka_gripper/move",
        boost::bind(&KittingStateController::executeMoveAction, this, _1), false);
    grasp_action_server_ = std::make_unique<GraspActionServer>(
        root_nh, "/franka_gripper/grasp",
        boost::bind(&KittingStateController::executeGraspAction, this, _1), false);
    homing_action_server_ = std::make_unique<HomingActionServer>(
        root_nh, "/franka_gripper/homing",
        boost::bind(&KittingStateController::executeHomingAction, this, _1), false);
    stop_action_server_ = std::make_unique<StopActionServer>(
        root_nh, "/franka_gripper/stop",
        boost::bind(&KittingStateController::executeStopAction, this, _1), false);

    move_action_server_->start();
    grasp_action_server_->start();
    homing_action_server_->start();
    stop_action_server_->start();
    ROS_INFO("KittingStateController: Gripper action servers started (move, grasp, homing, stop)");

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

  // ============================================================================
  // starting() — Called once when the controller is activated
  // ============================================================================

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
    accumulated_uplift_ = 0.0;
    fr_friction_sum_ = 0.0;
    fr_friction_count_ = 0;
    fr_friction_max_ = 0.0;

    logStateTransition("START", "Controller running");
    if (require_logger_) {
      ROS_INFO("    Step 1: Wait for Grasp logger to start");
      ROS_INFO("    Step 2: Publish BASELINE on /kitting_controller/state_cmd");
    } else {
      ROS_INFO("    Recording disabled — publish BASELINE on /kitting_controller/state_cmd");
    }
  }

  // ============================================================================
  // Realtime-safe helpers for update() (no locks, no allocation)
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
      gripper_debounce_.reset();
      gripper_stop_sent_.store(false, std::memory_order_relaxed);
      gripper_stopped_.store(false, std::memory_order_relaxed);
      contact_width_.store(0.0, std::memory_order_relaxed);

      // Reset force ramp state
      fr_f_current_ = 0.0;
      fr_iteration_ = 0;
      fr_grasp_cmd_seen_executing_ = false;
      fr_grasp_stabilizing_ = false;
      fr_grasping_phase_initialized_ = false;
      fr_friction_sum_ = 0.0;
      fr_friction_count_ = 0;
      fr_friction_max_ = 0.0;

      ROS_INFO("  [BASELINE]  State machine reset for new trial");

      // Correct accumulated uplift from previous SUCCESS (arm stayed elevated
      // for pick-and-place; now returning to original height for next trial).
      if (accumulated_uplift_ > 0.001) {
        downlift_start_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        downlift_z_start_ = downlift_start_pose_[14];
        downlift_elapsed_ = 0.0;
        rt_downlift_distance_ = accumulated_uplift_;
        rt_downlift_duration_ = accumulated_uplift_ / fr_lift_speed_;
        downlift_active_.store(true, std::memory_order_relaxed);
        ROS_INFO("  [BASELINE]  Correcting accumulated uplift: %.4f m", accumulated_uplift_);
        accumulated_uplift_ = 0.0;
      }
    }

    // Handle CLOSING start: snapshot command parameters for realtime contact detection
    if (new_state == GraspState::CLOSING) {
      rt_closing_w_cmd_ = closing_w_cmd_;
      rt_closing_v_cmd_ = closing_v_cmd_;
      rt_T_hold_gripper_ = computeGripperHoldTime(rt_closing_v_cmd_);
      gripper_debounce_.reset();
      gripper_stop_sent_.store(false, std::memory_order_relaxed);
      gripper_stopped_.store(false, std::memory_order_relaxed);
      closing_cmd_seen_executing_ = false;
      ROS_INFO("  [CLOSING]  T_hold_gripper=%.3fs (from speed=%.4f m/s)",
              rt_T_hold_gripper_, rt_closing_v_cmd_);
    }

    // Handle GRASPING start: snapshot force ramp parameters, initialize force ramp state
    if (new_state == GraspState::GRASPING) {
      // Snapshot staging → Realtime-local copies (synchronized via state_changed_ acquire)
      rt_fr_f_min_ = staging_fr_f_min_;
      rt_fr_f_step_ = staging_fr_f_step_;
      rt_fr_f_max_ = staging_fr_f_max_;
      rt_fr_uplift_distance_ = staging_fr_uplift_distance_;
      rt_fr_lift_speed_ = staging_fr_lift_speed_;
      rt_fr_uplift_hold_ = staging_fr_uplift_hold_;
      rt_fr_grasp_speed_ = staging_fr_grasp_speed_;
      rt_fr_epsilon_ = staging_fr_epsilon_;
      rt_fr_slip_drop_thresh_ = staging_fr_slip_drop_thresh_;
      rt_fr_slip_width_thresh_ = staging_fr_slip_width_thresh_;
      rt_fr_load_transfer_min_ = staging_fr_load_transfer_min_;
      rt_fr_slip_score_thresh_ = staging_fr_slip_score_thresh_;
      rt_fr_slip_friction_thresh_ = staging_fr_slip_friction_thresh_;

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

  void KittingStateController::fillKittingStateMsg(
      const ros::Time& time,
      const franka::RobotState& robot_state,
      const std::array<double, 42>& jacobian,
      const std::array<double, 7>& gravity,
      const std::array<double, 7>& coriolis,
      const std::array<double, 6>& ee_velocity,
      double tau_ext_norm, double wrench_norm,
      const GripperData& gripper_snapshot) {
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

    // Gripper signals
    kitting_publisher_.msg_.gripper_width = gripper_snapshot.width;
    kitting_publisher_.msg_.gripper_width_dot = gripper_snapshot.width_dot;
    kitting_publisher_.msg_.gripper_width_cmd =
        (current_state_.load(std::memory_order_relaxed) == GraspState::CLOSING)
            ? rt_closing_w_cmd_ : 0.0;
    kitting_publisher_.msg_.gripper_max_width = gripper_snapshot.max_width;
    kitting_publisher_.msg_.gripper_is_grasped = gripper_snapshot.is_grasped;

    // Force ramp state
    kitting_publisher_.msg_.grasp_force = fr_f_current_;
    kitting_publisher_.msg_.grasp_iteration = fr_iteration_;
  }

  // ============================================================================
  // update() — 1kHz realtime loop (state transitions + Cartesian command every tick,
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

      // Directional force decomposition: O_F_ext_hat_K = [Fx, Fy, Fz, Tx, Ty, Tz]
      double support_force = std::abs(robot_state.O_F_ext_hat_K[2]);       // Fn = |Fz|
      double tangential_force = std::sqrt(
          robot_state.O_F_ext_hat_K[0] * robot_state.O_F_ext_hat_K[0] +
          robot_state.O_F_ext_hat_K[1] * robot_state.O_F_ext_hat_K[1]);   // Ft = sqrt(Fx²+Fy²)

      std::array<double, 6> ee_velocity{};
      for (size_t row = 0; row < 6; ++row) {
        for (size_t col = 0; col < 7; ++col) {
          ee_velocity[row] += jacobian[col * 6 + row] * robot_state.dq[col];
        }
      }

      // Single gripper snapshot per tick — all consumers see consistent data
      const GripperData gripper_snapshot = *gripper_data_buf_.readFromRT();

      runContactDetection(time, gripper_snapshot);

      // Run internal force ramp transitions (GRASPING→UPLIFT→EVALUATE→...)
      {
        GraspState fr_state = current_state_.load(std::memory_order_relaxed);
        if (fr_state == GraspState::GRASPING || fr_state == GraspState::UPLIFT ||
            fr_state == GraspState::EVALUATE || fr_state == GraspState::DOWNLIFT ||
            fr_state == GraspState::SETTLING) {
          runInternalTransitions(time, tau_ext_norm, support_force, tangential_force,
                                gripper_snapshot);
        }
      }

      // Signal monitor — 2 Hz slow-rate logger for terminal readability
      // Re-read state since runInternalTransitions may have changed it
      const GraspState state = current_state_.load(std::memory_order_relaxed);
      if (signal_log_trigger_()) {
        if (state == GraspState::CLOSING) {
          double grip_gap = gripper_snapshot.width - rt_closing_w_cmd_;
          bool grip_stall = (std::abs(gripper_snapshot.width_dot) < stall_velocity_threshold_) &&
                            (grip_gap > width_gap_threshold_);
          ROS_INFO("  [SIGNAL]  CLOSING  |  grip: w=%.4f w_cmd=%.4f gap=%.4f w_dot=%.6f %s",
                  gripper_snapshot.width, rt_closing_w_cmd_, grip_gap,
                  gripper_snapshot.width_dot,
                  grip_stall ? "STALL" : "moving");
        } else if (state == GraspState::GRASPING || state == GraspState::UPLIFT ||
                  state == GraspState::EVALUATE || state == GraspState::DOWNLIFT ||
                  state == GraspState::SETTLING) {
          double u_live = tangential_force / std::max(support_force, 1e-6);
          ROS_INFO("  [SIGNAL]  %s  |  F=%.1f  iter=%d  Fn=%.2f  Ft=%.2f  u=%.3f",
                  stateToString(state), fr_f_current_, fr_iteration_,
                  support_force, tangential_force, u_live);
        }
      }

      // Gripper velocity — 10 Hz during CLOSING (DEBUG level to minimize realtime allocation)
      if (state == GraspState::CLOSING && gripper_log_trigger_()) {
        ROS_DEBUG("  [GRIP]  w=%.5f  w_dot=%.6f m/s  gap=%.5f  %s",
                  gripper_snapshot.width,
                  gripper_snapshot.width_dot,
                  gripper_snapshot.width - rt_closing_w_cmd_,
                  (std::abs(gripper_snapshot.width_dot) < stall_velocity_threshold_) ? "SLOW" : "ok");
      }

      if (kitting_publisher_.trylock()) {
        fillKittingStateMsg(time, robot_state, jacobian, gravity, coriolis,
                            ee_velocity, tau_ext_norm, wrench_norm,
                            gripper_snapshot);
        kitting_publisher_.unlockAndPublish();
      }
    }
  }

}  // namespace franka_kitting_controller

PLUGINLIB_EXPORT_CLASS(franka_kitting_controller::KittingStateController,
                       controller_interface::ControllerBase)
