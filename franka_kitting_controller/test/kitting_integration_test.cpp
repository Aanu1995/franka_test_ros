// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Integration tests: command handlers, applyPendingStateTransition,
// auto mode callbacks, gripper command queueing, deferred grasp, and
// state guard validation. Uses the same friend-class fixture as unit tests.

#include <cmath>

#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <ros/time.h>

#include <franka_kitting_controller/KittingGripperCommand.h>

#include "test_helpers.h"

using franka_kitting_controller::GraspState;
using franka_kitting_controller::GripperCommand;
using franka_kitting_controller::GripperCommandType;
using franka_kitting_controller::GripperData;
using franka_kitting_controller::KittingGripperCommand;
using franka_kitting_controller::KittingStateController;
using franka_kitting_controller::RampPhase;

// Helper: create a KittingGripperCommand message
static KittingGripperCommand::Ptr makeCmd(const std::string& command) {
  auto msg = boost::make_shared<KittingGripperCommand>();
  msg->command = command;
  return msg;
}

// ============================================================================
// handleBaselineCmd tests
// ============================================================================

TEST_F(KittingControllerTestFixture, HandleBaseline_SetsPendingState) {
  setCurrentState(GraspState::START);
  setStateChanged(false);

  auto msg = makeCmd("BASELINE");
  msg->open_gripper = false;
  callHandleBaselineCmd(msg);

  // Should set pending state to BASELINE (which routes through UNKNOWN)
  EXPECT_TRUE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleBaseline_WithOpenGripper) {
  setCurrentState(GraspState::START);
  setStateChanged(false);
  setBaselineNeedsOpen(false);

  // Write gripper data for max_width resolution
  GripperData gd;
  gd.width = 0.04;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("BASELINE");
  msg->open_gripper = true;
  msg->open_width = 0.06;
  callHandleBaselineCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_TRUE(baselineNeedsOpen());
  EXPECT_NEAR(baselineOpenWidth(), 0.06, 1e-9);
}

TEST_F(KittingControllerTestFixture, HandleBaseline_OpenGripper_ZeroWidth_UsesMax) {
  setCurrentState(GraspState::START);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("BASELINE");
  msg->open_gripper = true;
  msg->open_width = 0.0;  // 0 → use max
  callHandleBaselineCmd(msg);

  EXPECT_TRUE(baselineNeedsOpen());
  EXPECT_NEAR(baselineOpenWidth(), 0.08, 1e-9);
}

// ============================================================================
// handleClosingCmd tests
// ============================================================================

TEST_F(KittingControllerTestFixture, HandleClosing_RejectsWhenAlreadyClosing) {
  setCurrentState(GraspState::CLOSING);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("CLOSING");
  callHandleClosingCmd(msg);

  // Should NOT set state_changed (rejected)
  EXPECT_FALSE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleClosing_RejectsWhenContactConfirmed) {
  setCurrentState(GraspState::CONTACT_CONFIRMED);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("CLOSING");
  callHandleClosingCmd(msg);

  EXPECT_FALSE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleClosing_ClampsWidthToMax) {
  setCurrentState(GraspState::BASELINE);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("CLOSING");
  msg->closing_width = 0.20;  // > max_width
  msg->closing_speed = 0.05;
  callHandleClosingCmd(msg);

  EXPECT_TRUE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleClosing_ClampsSpeedToMax) {
  setCurrentState(GraspState::BASELINE);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("CLOSING");
  msg->closing_width = 0.001;
  msg->closing_speed = 0.50;  // > kMaxClosingSpeed (0.10)
  callHandleClosingCmd(msg);

  EXPECT_TRUE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleClosing_NormalFlow) {
  setCurrentState(GraspState::BASELINE);
  setStateChanged(false);

  GripperData gd;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("CLOSING");
  msg->closing_width = 0.001;
  msg->closing_speed = 0.05;
  callHandleClosingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_EQ(pendingState(), GraspState::CLOSING_COMMAND);
}

// ============================================================================
// handleGraspingCmd tests
// ============================================================================

TEST_F(KittingControllerTestFixture, HandleGrasping_RejectsWhenNotContact) {
  setCurrentState(GraspState::BASELINE);
  setStateChanged(false);

  auto msg = makeCmd("GRASPING");
  callHandleGraspingCmd(msg);

  EXPECT_FALSE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleGrasping_RejectsInvalidContactWidth) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.0);  // Invalid: <= 0
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.04;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  callHandleGraspingCmd(msg);

  EXPECT_FALSE(stateChanged());
}

TEST_F(KittingControllerTestFixture, HandleGrasping_NormalFlow_StagesParams) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->f_min = 5.0;
  msg->f_step = 2.0;
  msg->f_max = 20.0;
  msg->fr_fixed_grasp_steps = 3;
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_EQ(pendingState(), GraspState::GRASPING);
  EXPECT_NEAR(stagingFMin(), 5.0, 1e-9);
  EXPECT_NEAR(stagingFStep(), 2.0, 1e-9);
  EXPECT_NEAR(stagingFMax(), 20.0, 1e-9);
  EXPECT_EQ(stagingFixedGraspSteps(), 3);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_FixedGraspSteps_ZeroUsesDefault) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_fixed_grasp_steps = 0;  // 0 → use YAML default (-1)
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  // YAML default is fr_fixed_grasp_steps_ = -1
  EXPECT_EQ(stagingFixedGraspSteps(), -1);
}

// --- Parameter clamping tests ---

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsUpliftDistance) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_uplift_distance = 1.0;  // > kMaxUpliftDistance (0.3)
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_LE(stagingUpliftDist(), KittingStateController::kMaxUpliftDistance);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsLiftSpeed) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_lift_speed = 0.0001;  // < kMinLiftSpeed (0.001)
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingLiftSpeed(), KittingStateController::kMinLiftSpeed);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsUpliftHoldMin) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_uplift_hold = 0.1;  // < kMinUpliftHold (0.5)
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingUpliftHold(), KittingStateController::kMinUpliftHold);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsUpliftHoldMax) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_uplift_hold = 200.0;  // > kMaxUpliftHold (120.0)
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_LE(stagingUpliftHold(), KittingStateController::kMaxUpliftHold);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsFMinPositive) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->f_min = -5.0;  // Invalid
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GT(stagingFMin(), 0.0);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsFStepPositive) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->f_step = -2.0;  // Invalid
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GT(stagingFStep(), 0.0);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsFMaxAboveFMin) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->f_min = 10.0;
  msg->f_max = 5.0;  // < f_min
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingFMax(), stagingFMin());
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsHoldTimeMin) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->grasp_force_hold_time = 0.1;  // < 0.25
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingGraspForceHoldTime(), 0.25);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsSettleTimeNonNegative) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->grasp_settle_time = -1.0;  // Negative
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingGraspSettleTime(), 0.0);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsSlipDropThresh) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_slip_drop_thresh = 2.0;  // > 1.0
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_LE(stagingSlipDropThresh(), 1.0);
  EXPECT_GE(stagingSlipDropThresh(), 0.0);
}

TEST_F(KittingControllerTestFixture, HandleGrasping_ClampsLoadTransferMinNonNeg) {
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);
  setStateChanged(false);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  auto msg = makeCmd("GRASPING");
  msg->fr_load_transfer_min = -0.5;  // Negative
  callHandleGraspingCmd(msg);

  EXPECT_TRUE(stateChanged());
  EXPECT_GE(stagingLoadTransferMin(), 0.0);
}

// ============================================================================
// applyPendingStateTransition tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ApplyTransition_NoTransitionPending) {
  setStateChanged(false);
  setCurrentState(GraspState::START);

  callApplyPendingStateTransition();
  EXPECT_EQ(currentState(), GraspState::START);
}

TEST_F(KittingControllerTestFixture, ApplyTransition_Baseline_ResetsState) {
  // Simulate dirty state from a previous trial
  setContactLatched(true);
  setClosingCommandEntered(true);
  setClosingCmdSeenExecuting(true);
  setAccumulatedUplift(0.0);
  setBaselineNeedsOpen(false);

  // Trigger BASELINE transition (routed through UNKNOWN)
  setPendingState(GraspState::BASELINE);
  setStateChanged(true);

  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::UNKNOWN);
  EXPECT_FALSE(stateChanged());
  EXPECT_FALSE(contactLatched());
  EXPECT_FALSE(cdBaselineReady());
  EXPECT_NEAR(fnBaseline(), 0.0, 1e-9);
}

TEST_F(KittingControllerTestFixture, ApplyTransition_Baseline_WithDownlift) {
  // Previous trial uplifted 5mm
  setAccumulatedUplift(0.005);
  setBaselineNeedsOpen(false);

  setPendingState(GraspState::BASELINE);
  setStateChanged(true);

  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::UNKNOWN);
  // Downlift should be activated
  EXPECT_TRUE(downliftActive());
}

TEST_F(KittingControllerTestFixture, ApplyTransition_Baseline_NoDownlift_PrepDone) {
  setAccumulatedUplift(0.0);
  setBaselineNeedsOpen(false);

  setPendingState(GraspState::BASELINE);
  setStateChanged(true);

  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::UNKNOWN);
  EXPECT_FALSE(downliftActive());
  EXPECT_TRUE(baselinePrepDone());
}

TEST_F(KittingControllerTestFixture, ApplyTransition_ClosingCommand_SnapshotsParams) {
  setClosingWCmd(0.002);
  setClosingVCmd(0.08);

  setPendingState(GraspState::CLOSING_COMMAND);
  setStateChanged(true);

  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::CLOSING_COMMAND);
  EXPECT_NEAR(rtClosingWCmd(), 0.002, 1e-9);
  EXPECT_NEAR(rtClosingVCmd(), 0.08, 1e-9);
}

TEST_F(KittingControllerTestFixture, ApplyTransition_Grasping_CopiesStagingToRT) {
  // Set staging params
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);

  GripperData gd;
  gd.width = 0.035;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  // Use handler to stage params
  auto msg = makeCmd("GRASPING");
  msg->f_min = 5.0;
  msg->f_step = 2.5;
  msg->f_max = 25.0;
  msg->fr_fixed_grasp_steps = 4;
  callHandleGraspingCmd(msg);

  // Now apply the transition
  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_NEAR(rtFMin(), 5.0, 1e-9);
  EXPECT_NEAR(rtFStep(), 2.5, 1e-9);
  EXPECT_NEAR(rtFMax(), 25.0, 1e-9);
  EXPECT_EQ(rtFixedGraspSteps(), 4);
  EXPECT_NEAR(forceCurrent(), 5.0, 1e-9);  // fr_f_current_ = rt_fr_f_min_
  EXPECT_EQ(iteration(), 0);
}

// ============================================================================
// Auto mode callback tests
// ============================================================================

TEST_F(KittingControllerTestFixture, AutoBaselinePoll_ExitsOnNotAutoMode) {
  setAutoMode(false);
  ros::TimerEvent evt;

  callAutoBaselinePollCallback(evt);
  // Should return immediately, no crash, no state change
  EXPECT_FALSE(autoMode());
}

TEST_F(KittingControllerTestFixture, AutoBaselinePoll_ExitsOnFailed) {
  setAutoMode(true);
  setCurrentState(GraspState::FAILED);
  ros::TimerEvent evt;

  callAutoBaselinePollCallback(evt);
  EXPECT_FALSE(autoMode());
}

TEST_F(KittingControllerTestFixture, AutoBaselinePoll_ProceedsOnReady) {
  setAutoMode(true);
  setCurrentState(GraspState::BASELINE);
  setBaselinePrepDone(true);
  setCdBaselineReady(true);
  ros::TimerEvent evt;

  callAutoBaselinePollCallback(evt);
  // Should proceed: auto_mode still true, timer setup attempted
  // (timer creation may fail without ROS, but no crash)
  EXPECT_TRUE(autoMode());
}

TEST_F(KittingControllerTestFixture, AutoContactPoll_ExitsOnNotAutoMode) {
  setAutoMode(false);
  ros::TimerEvent evt;

  callAutoContactPollCallback(evt);
  EXPECT_FALSE(autoMode());
}

TEST_F(KittingControllerTestFixture, AutoContactPoll_ExitsOnFailed) {
  setAutoMode(true);
  setCurrentState(GraspState::FAILED);
  ros::TimerEvent evt;

  callAutoContactPollCallback(evt);
  EXPECT_FALSE(autoMode());
}

TEST_F(KittingControllerTestFixture, AutoGrasping_ExitsOnNotAutoMode) {
  setAutoMode(false);
  ros::TimerEvent evt;

  callAutoGraspingCallback(evt);
  EXPECT_FALSE(autoMode());
}

// ============================================================================
// Deferred grasp tests
// ============================================================================

TEST_F(KittingControllerTestFixture, RequestDeferredGrasp_SetsParams) {
  callRequestDeferredGrasp(0.035, 0.02, 10.0, 0.008);

  EXPECT_TRUE(deferredGraspPending());
  EXPECT_NEAR(deferredGraspWidth(), 0.035, 1e-9);
  EXPECT_NEAR(deferredGraspForce(), 10.0, 1e-9);
}

// ============================================================================
// Gripper command queue tests
// ============================================================================

TEST_F(KittingControllerTestFixture, QueueGripperCommand_SetsCmdReady) {
  GripperCommand cmd;
  cmd.type = GripperCommandType::MOVE;
  cmd.width = 0.04;
  cmd.speed = 0.05;

  callQueueGripperCommand(cmd);
  EXPECT_TRUE(cmdReady());
}

TEST_F(KittingControllerTestFixture, QueueGripperCommand_InterruptsExecuting) {
  setCmdExecuting(true);

  GripperCommand cmd;
  cmd.type = GripperCommandType::GRASP;
  cmd.width = 0.04;
  cmd.speed = 0.02;
  cmd.force = 10.0;

  callQueueGripperCommand(cmd);
  EXPECT_TRUE(stopRequested());
  EXPECT_TRUE(cmdReady());
}

// ============================================================================
// stateCmdCallback guard tests (via direct state manipulation)
// ============================================================================

TEST_F(KittingControllerTestFixture, StartState_RejectsClosing) {
  // In START state, only BASELINE and AUTO should be accepted
  setCurrentState(GraspState::START);
  setStateChanged(false);

  // We can't call stateCmdCallback directly without a subscriber,
  // but we can verify the guard logic: isClosingPhase and isForceRampPhase
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::START));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::START));
}

// ============================================================================
// Full integration: BASELINE → CLOSING → CONTACT → GRASPING flow
// ============================================================================

TEST_F(KittingControllerTestFixture, FullIntegration_BaselineToGrasping) {
  // Step 1: BASELINE command
  GripperData gd;
  gd.width = 0.04;
  gd.max_width = 0.08;
  gd.stamp = ros::Time(100.0);
  writeGripperData(gd);

  setCurrentState(GraspState::START);
  setAccumulatedUplift(0.0);

  auto baseline_msg = makeCmd("BASELINE");
  baseline_msg->open_gripper = false;
  callHandleBaselineCmd(baseline_msg);
  callApplyPendingStateTransition();
  EXPECT_EQ(currentState(), GraspState::UNKNOWN);
  EXPECT_TRUE(baselinePrepDone());  // No prep needed

  // Step 2: CLOSING command (simulate that we're in BASELINE state after settle)
  setCurrentState(GraspState::BASELINE);
  setCdBaselineReady(true);

  auto closing_msg = makeCmd("CLOSING");
  closing_msg->closing_width = 0.001;
  closing_msg->closing_speed = 0.05;
  callHandleClosingCmd(closing_msg);
  callApplyPendingStateTransition();
  EXPECT_EQ(currentState(), GraspState::CLOSING_COMMAND);

  // Step 3: Simulate contact detection → CONTACT
  setCurrentState(GraspState::CONTACT);
  setContactWidth(0.035);

  // Step 4: GRASPING command
  auto grasping_msg = makeCmd("GRASPING");
  grasping_msg->f_min = 3.0;
  grasping_msg->f_max = 15.0;
  grasping_msg->f_step = 3.0;
  grasping_msg->fr_fixed_grasp_steps = 2;
  callHandleGraspingCmd(grasping_msg);
  callApplyPendingStateTransition();

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_NEAR(rtFMin(), 3.0, 1e-9);
  EXPECT_NEAR(rtFMax(), 15.0, 1e-9);
  EXPECT_EQ(rtFixedGraspSteps(), 2);
  EXPECT_EQ(iteration(), 0);
  EXPECT_NEAR(forceCurrent(), 3.0, 1e-9);
}
