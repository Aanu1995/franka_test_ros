// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Test helpers: MockModel, identity pose builder, and KittingControllerTestFixture.
// The fixture uses friend access to set private members directly, bypassing init().
// Provides accessors for the RampPhase sub-state machine and force ramp timing.

#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>

#include <franka_kitting_controller/kitting_state_controller.h>

// ============================================================================
// MockModel: returns zeros/identity for all dynamics queries
// ============================================================================

class MockModel : public franka_hw::ModelBase {
 public:
  std::array<double, 16> pose(
      franka::Frame /* frame */,
      const std::array<double, 7>& /* q */,
      const std::array<double, 16>& /* F_T_EE */,
      const std::array<double, 16>& /* EE_T_K */) const override {
    // Identity 4x4 (column-major)
    return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
  }

  std::array<double, 42> bodyJacobian(
      franka::Frame /* frame */,
      const std::array<double, 7>& /* q */,
      const std::array<double, 16>& /* F_T_EE */,
      const std::array<double, 16>& /* EE_T_K */) const override {
    return {};
  }

  std::array<double, 42> zeroJacobian(
      franka::Frame /* frame */,
      const std::array<double, 7>& /* q */,
      const std::array<double, 16>& /* F_T_EE */,
      const std::array<double, 16>& /* EE_T_K */) const override {
    return {};
  }

  std::array<double, 49> mass(
      const std::array<double, 7>& /* q */,
      const std::array<double, 9>& /* I_total */,
      double /* m_total */,
      const std::array<double, 3>& /* F_x_Ctotal */) const override {
    return {};
  }

  std::array<double, 7> coriolis(
      const std::array<double, 7>& /* q */,
      const std::array<double, 7>& /* dq */,
      const std::array<double, 9>& /* I_total */,
      double /* m_total */,
      const std::array<double, 3>& /* F_x_Ctotal */) const override {
    return {};
  }

  std::array<double, 7> gravity(
      const std::array<double, 7>& /* q */,
      double /* m_total */,
      const std::array<double, 3>& /* F_x_Ctotal */,
      const std::array<double, 3>& /* gravity_earth */) const override {
    return {};
  }
};

// ============================================================================
// Helper: build a franka::RobotState with identity pose at a given Z height
// ============================================================================

inline franka::RobotState makeTestRobotState(double z = 0.4) {
  franka::RobotState state{};
  // Identity 4x4 in column-major, with Z translation
  state.O_T_EE = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, z, 1}};
  state.O_T_EE_d = state.O_T_EE;  // Desired = actual
  state.O_T_EE_c = state.O_T_EE;
  // Flange-to-EE and EE-to-K: identity
  state.F_T_EE = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
  state.EE_T_K = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
  // Zero external torques / wrenches
  state.tau_ext_hat_filtered = {};
  state.O_F_ext_hat_K = {};
  state.q = {};
  state.dq = {};
  state.tau_J = {};
  return state;
}

// ============================================================================
// Helper: build an identity 4x4 pose (column-major) at given Z
// ============================================================================

inline std::array<double, 16> makeIdentityPose(double z = 0.4) {
  return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, z, 1}};
}

// ============================================================================
// KittingControllerTestFixture
// Uses friend access to set private member state directly, bypassing init().
// ============================================================================

using franka_kitting_controller::GraspState;
using franka_kitting_controller::GripperData;
using franka_kitting_controller::KittingStateController;
using franka_kitting_controller::RampPhase;

class KittingControllerTestFixture : public ::testing::Test {
 protected:
  // Test data backing the handles
  franka::RobotState robot_state_{};
  std::array<double, 16> pose_command_{};
  std::array<double, 2> elbow_{};
  MockModel mock_model_;

  // Real handles backed by test data
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> pose_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  // Controller under test
  KittingStateController controller_;

  void SetUp() override {
    // Build a robot state at z = 0.4m with identity rotation
    robot_state_ = makeTestRobotState(0.4);
    pose_command_ = makeIdentityPose(0.4);

    // Create handles from test data
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        "test_robot", robot_state_);
    pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        *state_handle_, pose_command_, elbow_);
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        "test_model", mock_model_, robot_state_);

    // Assign handles to controller private members (friend access)
    controller_.franka_state_handle_ =
        std::make_unique<franka_hw::FrankaStateHandle>(*state_handle_);
    controller_.cartesian_pose_handle_ =
        std::make_unique<franka_hw::FrankaCartesianPoseHandle>(*pose_handle_);
    controller_.model_handle_ =
        std::make_unique<franka_hw::FrankaModelHandle>(*model_handle_);

    // Set default RT force ramp parameters
    controller_.rt_fr_f_min_ = 3.0;
    controller_.rt_fr_f_step_ = 3.0;
    controller_.rt_fr_f_max_ = 70.0;
    controller_.rt_fr_uplift_distance_ = 0.010;
    controller_.rt_fr_lift_speed_ = 0.01;
    controller_.rt_fr_uplift_hold_ = 1.0;
    controller_.rt_fr_grasp_speed_ = 0.02;
    controller_.rt_fr_epsilon_ = 0.008;
    controller_.rt_fr_slip_drop_thresh_ = 0.15;
    controller_.rt_fr_load_transfer_min_ = 0.02;
    controller_.rt_fr_grasp_force_hold_time_ = 2.0;
    controller_.rt_fr_grasp_settle_time_ = 0.5;
  }

  // --- Accessor / mutator helpers ---

  void setCurrentState(GraspState s) {
    controller_.current_state_.store(s, std::memory_order_relaxed);
  }

  GraspState currentState() const {
    return controller_.current_state_.load(std::memory_order_relaxed);
  }

  void setForceRampParams(double f_min, double f_step, double f_max,
                          double uplift_dist, double lift_speed,
                          double uplift_hold) {
    controller_.rt_fr_f_min_ = f_min;
    controller_.rt_fr_f_step_ = f_step;
    controller_.rt_fr_f_max_ = f_max;
    controller_.rt_fr_uplift_distance_ = uplift_dist;
    controller_.rt_fr_lift_speed_ = lift_speed;
    controller_.rt_fr_uplift_hold_ = uplift_hold;
  }

  void setSlipThresholds(double drop_thresh, double load_transfer_min) {
    controller_.rt_fr_slip_drop_thresh_ = drop_thresh;
    controller_.rt_fr_load_transfer_min_ = load_transfer_min;
  }

  void setCmdExecuting(bool val) {
    controller_.cmd_executing_.store(val, std::memory_order_relaxed);
  }
  void setCmdSuccess(bool val) {
    controller_.cmd_success_.store(val, std::memory_order_relaxed);
  }

  void setPhaseStartTime(const ros::Time& t) {
    controller_.fr_phase_start_time_ = t;
  }

  void setGraspingInitialized(bool val) {
    controller_.fr_grasping_phase_initialized_ = val;
  }

  void setGraspCmdSeenExecuting(bool val) {
    controller_.fr_grasp_cmd_seen_executing_ = val;
  }

  void setRampPhase(RampPhase p) { controller_.fr_ramp_phase_ = p; }
  RampPhase rampPhase() const { return controller_.fr_ramp_phase_; }

  void setRampStepStartTime(const ros::Time& t) {
    controller_.fr_ramp_step_start_time_ = t;
  }

  void setGraspForceHoldTime(double t) {
    controller_.rt_fr_grasp_force_hold_time_ = t;
  }

  void setGraspSettleTime(double t) {
    controller_.rt_fr_grasp_settle_time_ = t;
  }

  void setForceCurrent(double f) { controller_.fr_f_current_ = f; }
  double forceCurrent() const { return controller_.fr_f_current_; }

  void setIteration(int i) { controller_.fr_iteration_ = i; }
  int iteration() const { return controller_.fr_iteration_; }

  void setUpliftActive(bool val) {
    controller_.uplift_active_.store(val, std::memory_order_relaxed);
  }

  void setDownliftActive(bool val) {
    controller_.downlift_active_.store(val, std::memory_order_relaxed);
  }

  bool upliftActive() const {
    return controller_.uplift_active_.load(std::memory_order_relaxed);
  }

  bool downliftActive() const {
    return controller_.downlift_active_.load(std::memory_order_relaxed);
  }

  void setUpliftStartPose(const std::array<double, 16>& pose) {
    controller_.uplift_start_pose_ = pose;
  }

  void setUpliftParams(double distance, double duration) {
    controller_.rt_uplift_distance_ = distance;
    controller_.rt_uplift_duration_ = duration;
  }

  void setDownliftStartPose(const std::array<double, 16>& pose) {
    controller_.downlift_start_pose_ = pose;
  }

  void setDownliftParams(double distance, double duration) {
    controller_.rt_downlift_distance_ = distance;
    controller_.rt_downlift_duration_ = duration;
  }

  double upliftZStart() const { return controller_.uplift_z_start_; }
  double downliftZStart() const { return controller_.downlift_z_start_; }

  void setFnBaseline(double fn) {
    controller_.fn_baseline_ = fn;
  }

  void setEarlyAccumulators(double sum, int count) {
    controller_.fr_early_sum_ = sum;
    controller_.fr_early_count_ = count;
  }

  void setLateAccumulators(double sum, int count) {
    controller_.fr_late_sum_ = sum;
    controller_.fr_late_count_ = count;
  }

  double accumulatedUplift() const { return controller_.accumulated_uplift_; }

  void setContactWidth(double w) {
    controller_.contact_width_.store(w, std::memory_order_relaxed);
  }

  double contactWidth() const {
    return controller_.contact_width_.load(std::memory_order_relaxed);
  }

  bool deferredGraspPending() const {
    return controller_.deferred_grasp_pending_.load(std::memory_order_relaxed);
  }

  // --- Fixed grasp steps ---

  void setFixedGraspSteps(int n) { controller_.rt_fr_fixed_grasp_steps_ = n; }
  int fixedGraspSteps() const { return controller_.rt_fr_fixed_grasp_steps_; }

  // --- SMS-CUSUM detector access ---

  sms_cusum::SMSCusum& smsDetector() { return controller_.sms_detector_; }

  // --- Contact detection state ---

  void setContactLatched(bool val) { controller_.contact_latched_ = val; }
  bool contactLatched() const { return controller_.contact_latched_; }

  void setClosingCommandEntered(bool val) { controller_.closing_command_entered_ = val; }
  void setClosingCmdSeenExecuting(bool val) { controller_.closing_cmd_seen_executing_ = val; }

  void setCdBaselineReady(bool val) {
    controller_.cd_baseline_ready_.store(val, std::memory_order_relaxed);
  }
  bool cdBaselineReady() const {
    return controller_.cd_baseline_ready_.load(std::memory_order_relaxed);
  }

  void setGripperStopped(bool val) {
    controller_.gripper_stopped_.store(val, std::memory_order_relaxed);
  }

  // --- Wrappers for private contact/closing functions ---

  void callDetectContact(const ros::Time& time, const GripperData& g,
                         double tau_ext_norm) {
    controller_.detectContact(time, g, tau_ext_norm);
  }

  void callRunClosingTransitions(const ros::Time& time, const GripperData& g,
                                 double tau_ext_norm) {
    controller_.runClosingTransitions(time, g, tau_ext_norm);
  }

  // --- Wrappers for private tick functions ---

  void callTickGrasping(const ros::Time& time, double tau, double fn,
                        const GripperData& g) {
    controller_.tickGrasping(time, tau, fn, g);
  }

  void callTickUplift(const ros::Time& time, double tau, double fn,
                      const GripperData& g) {
    controller_.tickUplift(time, tau, fn, g);
  }

  void callTickEvaluate(const ros::Time& time, double tau, double fn,
                        const GripperData& g) {
    controller_.tickEvaluate(time, tau, fn, g);
  }

  void callResetForceRampState() { controller_.resetForceRampState(); }

  std::array<double, 16> callComputeUpliftPose(double elapsed) {
    return controller_.computeUpliftPose(elapsed);
  }

  std::array<double, 16> callComputeDownliftPose(double elapsed) {
    return controller_.computeDownliftPose(elapsed);
  }

  // --- Auto mode state ---

  void setAutoMode(bool val) {
    controller_.auto_mode_.store(val, std::memory_order_relaxed);
  }
  bool autoMode() const {
    return controller_.auto_mode_.load(std::memory_order_relaxed);
  }

  void setBaselinePrepDone(bool val) {
    controller_.baseline_prep_done_.store(val, std::memory_order_relaxed);
  }
  bool baselinePrepDone() const {
    return controller_.baseline_prep_done_.load(std::memory_order_relaxed);
  }

  // --- Baseline state ---

  void setUnknownSettleStarted(bool val) { controller_.unknown_settle_started_ = val; }
  void setUnknownSettleStart(const ros::Time& t) { controller_.unknown_settle_start_ = t; }

  void setFnBaselineSum(double s) { controller_.fn_baseline_sum_ = s; }
  void setFnBaselineCount(int n) { controller_.fn_baseline_count_ = n; }
  double fnBaseline() const { return controller_.fn_baseline_; }
  int fnBaselineCount() const { return controller_.fn_baseline_count_; }

  void setCdBaseline(double v) { controller_.cd_baseline_ = v; }
  double cdBaseline() const { return controller_.cd_baseline_; }

  // --- Closing state accessors ---

  void setClosingWidthCmd(double w) { controller_.rt_closing_w_cmd_ = w; }

  bool closingCommandEntered() const { return controller_.closing_command_entered_; }
  bool closingCmdSeenExecuting() const { return controller_.closing_cmd_seen_executing_; }

  // --- Gripper stop state ---

  void setGripperStopSent(bool val) {
    controller_.gripper_stop_sent_.store(val, std::memory_order_relaxed);
  }

  // --- isActionAllowed ---

  bool callIsActionAllowed() const { return controller_.isActionAllowed(); }

  // --- requestGripperStop ---

  void callRequestGripperStop(const char* source) {
    controller_.requestGripperStop(source);
  }
  bool stopRequested() const {
    return controller_.stop_requested_.load(std::memory_order_relaxed);
  }
  bool gripperStopSent() const {
    return controller_.gripper_stop_sent_.load(std::memory_order_relaxed);
  }

  // --- Holding elapsed ---

  double holdingElapsed() const { return controller_.fr_holding_elapsed_; }

  // --- State transition machinery ---

  void setPendingState(GraspState s) {
    controller_.pending_state_.store(s, std::memory_order_relaxed);
  }
  GraspState pendingState() const {
    return controller_.pending_state_.load(std::memory_order_relaxed);
  }
  void setStateChanged(bool val) {
    controller_.state_changed_.store(val, std::memory_order_release);
  }
  bool stateChanged() const {
    return controller_.state_changed_.load(std::memory_order_acquire);
  }

  void callApplyPendingStateTransition() {
    controller_.applyPendingStateTransition();
  }

  // --- Closing params ---

  double rtClosingWCmd() const { return controller_.rt_closing_w_cmd_; }
  double rtClosingVCmd() const { return controller_.rt_closing_v_cmd_; }
  void setClosingWCmd(double w) { controller_.closing_w_cmd_ = w; }
  void setClosingVCmd(double v) { controller_.closing_v_cmd_ = v; }

  // --- Gripper data buffer ---

  void writeGripperData(const GripperData& g) {
    controller_.gripper_data_buf_.writeFromNonRT(g);
  }

  // --- Baseline open state ---

  void setBaselineNeedsOpen(bool val) {
    controller_.baseline_needs_open_.store(val, std::memory_order_relaxed);
  }
  bool baselineNeedsOpen() const {
    return controller_.baseline_needs_open_.load(std::memory_order_relaxed);
  }
  double baselineOpenWidth() const {
    return controller_.baseline_open_width_.load(std::memory_order_relaxed);
  }

  // --- Command handler wrappers ---

  void callHandleBaselineCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    controller_.handleBaselineCmd(msg);
  }
  void callHandleClosingCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    controller_.handleClosingCmd(msg);
  }
  void callHandleGraspingCmd(
      const franka_kitting_controller::KittingGripperCommand::ConstPtr& msg) {
    controller_.handleGraspingCmd(msg);
  }

  // --- Staging param accessors ---

  double stagingFMin() const { return controller_.staging_fr_f_min_; }
  double stagingFStep() const { return controller_.staging_fr_f_step_; }
  double stagingFMax() const { return controller_.staging_fr_f_max_; }
  double stagingUpliftDist() const { return controller_.staging_fr_uplift_distance_; }
  double stagingLiftSpeed() const { return controller_.staging_fr_lift_speed_; }
  double stagingUpliftHold() const { return controller_.staging_fr_uplift_hold_; }
  double stagingGraspForceHoldTime() const { return controller_.staging_fr_grasp_force_hold_time_; }
  double stagingGraspSettleTime() const { return controller_.staging_fr_grasp_settle_time_; }
  double stagingSlipDropThresh() const { return controller_.staging_fr_slip_drop_thresh_; }
  double stagingLoadTransferMin() const { return controller_.staging_fr_load_transfer_min_; }
  int stagingFixedGraspSteps() const { return controller_.staging_fr_fixed_grasp_steps_; }

  // --- RT param accessors ---

  double rtFMin() const { return controller_.rt_fr_f_min_; }
  double rtFStep() const { return controller_.rt_fr_f_step_; }
  double rtFMax() const { return controller_.rt_fr_f_max_; }
  int rtFixedGraspSteps() const { return controller_.rt_fr_fixed_grasp_steps_; }

  // --- Accumulated uplift ---

  void setAccumulatedUplift(double v) { controller_.accumulated_uplift_ = v; }

  // --- Gripper command queue ---

  void callQueueGripperCommand(const franka_kitting_controller::GripperCommand& cmd) {
    controller_.queueGripperCommand(cmd);
  }
  bool cmdReady() const { return controller_.cmd_ready_; }

  // --- Deferred grasp ---

  void callRequestDeferredGrasp(double w, double s, double f, double e) {
    controller_.requestDeferredGrasp(w, s, f, e);
  }
  double deferredGraspWidth() const { return controller_.deferred_grasp_width_; }
  double deferredGraspForce() const { return controller_.deferred_grasp_force_; }

  // --- Auto mode callbacks ---

  void callAutoBaselinePollCallback(const ros::TimerEvent& e) {
    controller_.autoBaselinePollCallback(e);
  }
  void callAutoContactPollCallback(const ros::TimerEvent& e) {
    controller_.autoContactPollCallback(e);
  }
  void callAutoGraspingCallback(const ros::TimerEvent& e) {
    controller_.autoGraspingCallback(e);
  }

  // --- Run internal transitions wrapper ---

  void callRunInternalTransitions(const ros::Time& time, double tau,
                                  double fn, const GripperData& g) {
    controller_.runInternalTransitions(time, tau, fn, g);
  }

  // Make a default GripperData for tests that don't care about gripper
  static GripperData makeDefaultGripper(double width = 0.04) {
    GripperData g;
    g.width = width;
    g.max_width = 0.08;
    g.width_dot = 0.0;
    g.is_grasped = false;
    g.stamp = ros::Time(100.0);
    return g;
  }
};
