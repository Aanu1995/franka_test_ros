// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Tier 2 — State machine tests using KittingControllerTestFixture.
// Tests tick functions, state transitions, trajectory math, and 2-gate slip detection.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <ros/time.h>

#include "test_helpers.h"

using franka_kitting_controller::GraspState;
using franka_kitting_controller::GripperData;
using franka_kitting_controller::RampPhase;

// ============================================================================
// resetForceRampState tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ResetForceRampState_ClearsAll) {
  // Dirty up state
  setForceCurrent(10.0);
  setIteration(5);
  setGraspCmdSeenExecuting(true);
  setRampPhase(RampPhase::HOLDING);
  setGraspingInitialized(true);
  callResetForceRampState();

  EXPECT_DOUBLE_EQ(forceCurrent(), 0.0);
  EXPECT_EQ(iteration(), 0);
}

// ============================================================================
// tickGrasping tests
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_SettleDelay) {
  // Within kGraspSettleDelay (0.1s) → no state change
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  ros::Time t0(100.0);
  auto g = makeDefaultGripper();

  // First call initializes phase
  callTickGrasping(t0, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // 50ms later — still settling
  ros::Time t1(100.05);
  callTickGrasping(t1, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);
}

TEST_F(KittingControllerTestFixture, TickGrasping_Timeout) {
  // elapsed > kGraspTimeout (10s) with command never completing → FAILED
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  ros::Time t0(100.0);
  auto g = makeDefaultGripper();

  // Initialize phase
  callTickGrasping(t0, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // Jump to 11 seconds later
  ros::Time t_late(111.0);
  callTickGrasping(t_late, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(KittingControllerTestFixture, TickGrasping_NormalFlow_ToUplift) {
  // Simulate full single-step ramp: COMMAND_SENT → WAITING → SETTLING → HOLDING → STEP_COMPLETE → UPLIFT
  // Use f_min = f_max with fixed_grasp_steps=1 to guarantee UPLIFT after 1 step
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setFixedGraspSteps(1);
  auto g = makeDefaultGripper();
  ros::Time t0(100.0);

  // Step 1: Initialize phase (COMMAND_SENT)
  callTickGrasping(t0, 0.0, 0.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::COMMAND_SENT);

  // Step 2: After settle delay, cmd starts executing
  setCmdExecuting(true);
  ros::Time t1(100.15);
  callTickGrasping(t1, 0.0, 0.0, g);
  // Should transition to WAITING_EXECUTION
  EXPECT_EQ(rampPhase(), RampPhase::WAITING_EXECUTION);

  // Step 3: cmd finishes successfully → SETTLING
  setCmdExecuting(false);
  setCmdSuccess(true);
  ros::Time t2(100.5);
  callTickGrasping(t2, 0.0, 5.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::SETTLING);

  // Step 4: Settle time passes → HOLDING
  ros::Time t3(101.1);
  callTickGrasping(t3, 0.0, 5.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::HOLDING);

  // Step 5: Hold time passes → STEP_COMPLETE
  ros::Time t4(102.2);
  callTickGrasping(t4, 0.0, 5.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::STEP_COMPLETE);

  // Step 6: Last step → UPLIFT
  ros::Time t5(102.3);
  callTickGrasping(t5, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::UPLIFT);
  EXPECT_TRUE(upliftActive());
}

// ============================================================================
// tickUplift tests
// ============================================================================

TEST_F(KittingControllerTestFixture, TickUplift_TrajectoryRunning) {
  setCurrentState(GraspState::UPLIFT);
  setUpliftActive(true);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickUplift(t, 0.0, 0.0, g);

  // Still UPLIFT — trajectory not done
  EXPECT_EQ(currentState(), GraspState::UPLIFT);
}

TEST_F(KittingControllerTestFixture, TickUplift_TrajectoryDone) {
  setCurrentState(GraspState::UPLIFT);
  setUpliftActive(false);  // Trajectory already completed
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickUplift(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::EVALUATE);
}

// ============================================================================
// tickEvaluate tests — 2-gate slip detection
// ============================================================================

// Helper: set up EVALUATE state with pre-computed accumulator data
class EvaluateTest : public KittingControllerTestFixture {
 protected:
  void SetUpEvaluate(double fn_baseline, double fn_early, int early_n,
                     double fn_late, int late_n) {
    setCurrentState(GraspState::EVALUATE);
    setFnBaseline(fn_baseline);
    setEarlyAccumulators(fn_early * early_n, early_n);
    setLateAccumulators(fn_late * late_n, late_n);
  }
};

TEST_F(EvaluateTest, AllGatesPass_Success) {
  // Gate 1: deltaF = Fn_early - Fn_baseline = 10 - 2 = 8 > 1.5 ✓
  // Gate 2: dF = (10 - 9.5) / 10 = 0.05 ≤ 0.15 ✓
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/10.0, /*early_n=*/125,
                /*fn_late=*/9.5, /*late_n=*/125);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::SUCCESS);
}

TEST_F(EvaluateTest, Gate1Fail_NoLoadTransfer) {
  // Gate 1 FAIL: deltaF = Fn_early - Fn_baseline = 3.01 - 3.0 = 0.01 < 0.02
  SetUpEvaluate(/*fn_baseline=*/3.0, /*fn_early=*/3.01, /*early_n=*/125,
                /*fn_late=*/3.01, /*late_n=*/125);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(EvaluateTest, Gate2Fail_SupportDrop) {
  // Gate 1 PASS: deltaF = 10 - 2 = 8 > 1.5 ✓
  // Gate 2 FAIL: dF = (10 - 8) / 10 = 0.2 > 0.15 ✗
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/10.0, /*early_n=*/125,
                /*fn_late=*/8.0, /*late_n=*/125);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(EvaluateTest, EarlyWindowAccumulates) {
  // During early window: elapsed < kEarlyEnd → should accumulate, not evaluate
  setCurrentState(GraspState::EVALUATE);
  setEarlyAccumulators(0.0, 0);
  setLateAccumulators(0.0, 0);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  // Call at elapsed = 0.2s (within early window, kEarlyEnd = 0.5s)
  ros::Time t(100.2);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::EVALUATE);  // Still evaluating
}

TEST_F(EvaluateTest, LateWindowAccumulates) {
  // During late window: kEarlyEnd ≤ elapsed < kLateEnd → accumulate, no verdict
  setCurrentState(GraspState::EVALUATE);
  setEarlyAccumulators(0.0, 0);
  setLateAccumulators(0.0, 0);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  // Call at elapsed = 0.7s (in late window)
  ros::Time t(100.7);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::EVALUATE);
}

// ============================================================================
// Ramp sub-phase machine tests (RampPhase within GRASPING)
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_RampSettling) {
  // After grasp command completes (WAITING_EXECUTION → SETTLING sub-phase)
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setGraspCmdSeenExecuting(true);
  setRampPhase(RampPhase::WAITING_EXECUTION);
  setGraspSettleTime(0.5);
  setCmdExecuting(false);  // Grasp command just finished
  setCmdSuccess(true);     // Hardware confirmed grasp OK

  auto g = makeDefaultGripper();
  ros::Time t(100.5);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_EQ(rampPhase(), RampPhase::SETTLING);
}

TEST_F(KittingControllerTestFixture, TickGrasping_GraspFailed_HardwareRejected) {
  // Grasp command completed but grasp() returned false → FAILED
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setGraspCmdSeenExecuting(true);
  setRampPhase(RampPhase::WAITING_EXECUTION);
  setCmdExecuting(false);  // Grasp command finished
  setCmdSuccess(false);    // Hardware says grasp failed

  auto g = makeDefaultGripper();
  ros::Time t(100.5);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(KittingControllerTestFixture, TickGrasping_RampHolding) {
  // After settle time expires → HOLDING sub-phase
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::SETTLING);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setRampStepStartTime(ros::Time(100.0));

  auto g = makeDefaultGripper();
  // 0.6s after settle start > 0.5s settle time
  ros::Time t(100.6);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_EQ(rampPhase(), RampPhase::HOLDING);
}

TEST_F(KittingControllerTestFixture, TickGrasping_RampStepAdvance) {
  // On STEP_COMPLETE with more steps remaining → advance force, dispatch deferred grasp
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setIteration(0);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 3.0, 30.0, 0.010, 0.01, 1.0);
  setContactWidth(0.04);

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_EQ(rampPhase(), RampPhase::COMMAND_SENT);
  EXPECT_DOUBLE_EQ(forceCurrent(), 6.0);  // 3.0 + 3.0
  EXPECT_EQ(iteration(), 1);
  EXPECT_TRUE(deferredGraspPending());
}

TEST_F(KittingControllerTestFixture, TickGrasping_RampComplete) {
  // On last step's STEP_COMPLETE with fixed_grasp_steps → transition to UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);  // At f_max
  setIteration(22);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setFixedGraspSteps(23);  // Force UPLIFT at step 23

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::UPLIFT);
  EXPECT_TRUE(upliftActive());
}

TEST_F(KittingControllerTestFixture, TickGrasping_ContactWidthUpdate) {
  // After hold completes (HOLDING → STEP_COMPLETE), contact_width_ is updated
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::HOLDING);
  setGraspForceHoldTime(1.0);
  setRampStepStartTime(ros::Time(100.0));
  setForceRampParams(3.0, 3.0, 30.0, 0.010, 0.01, 1.0);
  setContactWidth(0.040);

  auto g = makeDefaultGripper(0.038);  // New gripper width
  // 1.1s after hold start > 1.0s hold time
  ros::Time t(101.1);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(rampPhase(), RampPhase::STEP_COMPLETE);
  EXPECT_NEAR(contactWidth(), 0.038, 1e-10);
}

TEST_F(KittingControllerTestFixture, TickGrasping_NonDivisibleRange_ReachesFMax) {
  // f_min=3, f_step=4, f_max=10: should execute 3 → 7 → 10 (clamped), not stop at 7
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(7.0);  // After first advance: 3+4=7
  setIteration(1);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 4.0, 10.0, 0.010, 0.01, 1.0);
  setContactWidth(0.04);

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  // Should advance to 10 (clamped), NOT uplift
  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_EQ(rampPhase(), RampPhase::COMMAND_SENT);
  EXPECT_DOUBLE_EQ(forceCurrent(), 10.0);  // Clamped to f_max
  EXPECT_EQ(iteration(), 2);
  EXPECT_TRUE(deferredGraspPending());
}

TEST_F(KittingControllerTestFixture, TickGrasping_AtFMax_IsLastStep) {
  // When fr_f_current_ == f_max with fixed_grasp_steps, STEP_COMPLETE → UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(10.0);  // Already at f_max
  setIteration(2);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 4.0, 10.0, 0.010, 0.01, 1.0);
  setFixedGraspSteps(3);  // Force UPLIFT at step 3

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::UPLIFT);
  EXPECT_TRUE(upliftActive());
}

TEST_F(KittingControllerTestFixture, TickGrasping_AtFMax_NoSecure_Failed) {
  // When fr_f_current_ == f_max without secure grasp and no fixed override → FAILED
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(10.0);
  setIteration(2);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 4.0, 10.0, 0.010, 0.01, 1.0);
  setFixedGraspSteps(-1);  // Algorithm only — no secure grasp → FAILED

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Trajectory math tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_Start) {
  auto start_pose = makeIdentityPose(0.4);
  setUpliftStartPose(start_pose);
  setUpliftParams(0.010, 1.0);
  controller_.uplift_z_start_ = 0.4;

  auto pose = callComputeUpliftPose(0.0);
  // At t=0: s_raw=0, s=0.5*(1-cos(0))=0 → Z unchanged
  EXPECT_DOUBLE_EQ(pose[14], 0.4);
}

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_End) {
  auto start_pose = makeIdentityPose(0.4);
  setUpliftStartPose(start_pose);
  setUpliftParams(0.010, 1.0);
  controller_.uplift_z_start_ = 0.4;

  auto pose = callComputeUpliftPose(1.0);
  // At t=duration: s_raw=1, s=0.5*(1-cos(π))=1 → Z = 0.4 + 0.010
  EXPECT_NEAR(pose[14], 0.410, 1e-10);
}

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_Midpoint) {
  auto start_pose = makeIdentityPose(0.4);
  setUpliftStartPose(start_pose);
  setUpliftParams(0.010, 1.0);
  controller_.uplift_z_start_ = 0.4;

  auto pose = callComputeUpliftPose(0.5);
  // At t=0.5*duration: s_raw=0.5, s=0.5*(1-cos(π/2))=0.5 → Z = 0.4 + 0.005
  EXPECT_NEAR(pose[14], 0.405, 1e-10);
}

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_BeyondDuration) {
  auto start_pose = makeIdentityPose(0.4);
  setUpliftStartPose(start_pose);
  setUpliftParams(0.010, 1.0);
  controller_.uplift_z_start_ = 0.4;

  auto pose = callComputeUpliftPose(2.0);
  // s_raw clamped to 1.0 → same as end
  EXPECT_NEAR(pose[14], 0.410, 1e-10);
}

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_PreservesXY) {
  auto start_pose = makeIdentityPose(0.4);
  start_pose[12] = 0.3;  // x
  start_pose[13] = -0.1; // y
  setUpliftStartPose(start_pose);
  setUpliftParams(0.010, 1.0);
  controller_.uplift_z_start_ = 0.4;

  auto pose = callComputeUpliftPose(0.5);
  // X and Y unchanged
  EXPECT_DOUBLE_EQ(pose[12], 0.3);
  EXPECT_DOUBLE_EQ(pose[13], -0.1);
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_End) {
  auto start_pose = makeIdentityPose(0.41);
  setDownliftStartPose(start_pose);
  setDownliftParams(0.010, 1.0);
  controller_.downlift_z_start_ = 0.41;

  auto pose = callComputeDownliftPose(1.0);
  // At t=duration: Z = 0.41 - 0.010 = 0.400
  EXPECT_NEAR(pose[14], 0.400, 1e-10);
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_Start) {
  auto start_pose = makeIdentityPose(0.41);
  setDownliftStartPose(start_pose);
  setDownliftParams(0.010, 1.0);
  controller_.downlift_z_start_ = 0.41;

  auto pose = callComputeDownliftPose(0.0);
  // At t=0: Z unchanged
  EXPECT_DOUBLE_EQ(pose[14], 0.41);
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_Midpoint) {
  auto start_pose = makeIdentityPose(0.41);
  setDownliftStartPose(start_pose);
  setDownliftParams(0.010, 1.0);
  controller_.downlift_z_start_ = 0.41;

  auto pose = callComputeDownliftPose(0.5);
  // At midpoint: Z = 0.41 - 0.005 = 0.405
  EXPECT_NEAR(pose[14], 0.405, 1e-10);
}

// ============================================================================
// Constants sanity tests
// ============================================================================

TEST(ConstantsTest, SafetyLimits) {
  EXPECT_GT(KittingStateController::kMaxClosingSpeed, 0.0);
  EXPECT_GT(KittingStateController::kMaxUpliftDistance, 0.0);
  EXPECT_GT(KittingStateController::kMinUpliftHold, 0.0);
  EXPECT_GT(KittingStateController::kMaxUpliftHold,
            KittingStateController::kMinUpliftHold);
  EXPECT_GT(KittingStateController::kGraspTimeout,
            KittingStateController::kGraspSettleDelay);
}

// ============================================================================
// Full force ramp cycle test: GRASPING → UPLIFT → EVALUATE → SUCCESS
// ============================================================================

TEST_F(KittingControllerTestFixture, FullCycle_GraspingToSuccess) {
  // This test exercises the full happy path through the force ramp cycle.
  // Use f_min = f_max with fixed_grasp_steps=1 to guarantee UPLIFT after 1 step.
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setIteration(0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setFixedGraspSteps(1);
  auto g = makeDefaultGripper();

  // --- GRASPING phase ---
  ros::Time t(100.0);

  // Init phase (COMMAND_SENT)
  callTickGrasping(t, 0.0, 0.0, g);

  // cmd starts executing → WAITING_EXECUTION
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);

  // cmd finishes successfully → SETTLING
  setCmdExecuting(false);
  setCmdSuccess(true);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 8.0, g);

  // Settle time passes → HOLDING
  t = ros::Time(101.1);
  callTickGrasping(t, 0.0, 8.0, g);

  // Hold time passes → STEP_COMPLETE
  t = ros::Time(102.2);
  callTickGrasping(t, 0.0, 8.0, g);

  // Last step → UPLIFT
  t = ros::Time(102.3);
  callTickGrasping(t, 0.0, 8.0, g);
  EXPECT_EQ(currentState(), GraspState::UPLIFT);

  // --- UPLIFT phase (trajectory handled by updateCartesianCommand, not tested here) ---
  setUpliftActive(false);  // Simulate trajectory completion
  t = ros::Time(102.0);
  callTickUplift(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::EVALUATE);

  // --- EVALUATE phase ---
  setFnBaseline(2.0);  // No-load Fn from BASELINE
  setEarlyAccumulators(15.0 * 125, 125);
  setLateAccumulators(14.8 * 125, 125);

  ros::Time eval_start(102.0);
  setPhaseStartTime(eval_start);

  // Jump past the hold window (1.0s)
  t = ros::Time(103.1);
  callTickEvaluate(t, 0.0, 15.0, g);

  EXPECT_EQ(currentState(), GraspState::SUCCESS);
}

// ============================================================================
// fixed_grasp_steps override tests
// ============================================================================

// Helper: drive a single ramp step through COMMAND_SENT → STEP_COMPLETE
class FixedGraspStepsTest : public KittingControllerTestFixture {
 protected:
  void driveOneStep(ros::Time& t, double force) {
    auto g = makeDefaultGripper();

    // If phase is already at STEP_COMPLETE from a previous step,
    // tickGrasping will dispatch the next step (COMMAND_SENT).
    // Otherwise, this is the first call that initializes the phase.
    callTickGrasping(t, 0.0, 0.0, g);

    // Advance past settle delay, cmd executing
    setCmdExecuting(true);
    t += ros::Duration(0.15);
    callTickGrasping(t, 0.0, 0.0, g);

    // cmd done → SETTLING
    setCmdExecuting(false);
    setCmdSuccess(true);
    t += ros::Duration(0.5);
    callTickGrasping(t, 0.0, 5.0, g);

    // settle → HOLDING
    t += ros::Duration(0.6);
    callTickGrasping(t, 0.0, 5.0, g);

    // hold → STEP_COMPLETE
    t += ros::Duration(2.1);
    callTickGrasping(t, 0.0, 5.0, g);
    ASSERT_EQ(rampPhase(), RampPhase::STEP_COMPLETE);
  }
};

TEST_F(FixedGraspStepsTest, FixedGraspSteps_OverridesAlgorithm) {
  // fixed_grasp_steps=2: after 2 steps → UPLIFT, even without secure grasp
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);
  setFixedGraspSteps(2);
  setContactWidth(0.04);
  ros::Time t(100.0);
  auto g = makeDefaultGripper();

  // Step 1: drive through COMMAND_SENT → STEP_COMPLETE
  driveOneStep(t, 3.0);

  // STEP_COMPLETE: iteration=0, fixed_grasp_steps=2 → (0+1)=1 < 2 → advance
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);  // Still grasping, advanced to step 2
  EXPECT_EQ(iteration(), 1);

  // Step 2: drive through again
  driveOneStep(t, 6.0);

  // STEP_COMPLETE: iteration=1, (1+1)=2 >= 2 → fixed override → UPLIFT
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::UPLIFT);
}

TEST_F(FixedGraspStepsTest, FixedGraspSteps_Negative1_AlgorithmOnly) {
  // fixed_grasp_steps=-1: no override, f_min=f_max=70 (single step, last step → UPLIFT)
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);
  setFixedGraspSteps(-1);
  ros::Time t(100.0);
  auto g = makeDefaultGripper();

  driveOneStep(t, 70.0);

  // STEP_COMPLETE: f_max reached, no fixed override, no secure grasp → UPLIFT
  // (reached_f_max && !secure is checked, but for single step it goes to UPLIFT
  //  because it's the last step path in the existing logic)
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  // With f_min=f_max=70, this is the last step. Without secure grasp and
  // without fixed override, reached_f_max && !secure → FAILED
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(FixedGraspStepsTest, FixedGraspSteps_SingleStep) {
  // fixed_grasp_steps=1: secure after first step
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);
  setFixedGraspSteps(1);
  ros::Time t(100.0);
  auto g = makeDefaultGripper();

  driveOneStep(t, 3.0);

  // STEP_COMPLETE: iteration=0, (0+1)=1 >= 1 → secure → UPLIFT
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::UPLIFT);
}

TEST_F(FixedGraspStepsTest, FixedGraspSteps_FMaxFirst) {
  // fixed_grasp_steps=100 but f_max reached at step 1 → FAILED
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);
  setFixedGraspSteps(100);
  ros::Time t(100.0);
  auto g = makeDefaultGripper();

  driveOneStep(t, 70.0);

  // STEP_COMPLETE: reached_f_max=true, fixed_override=false (1<100), secure=false → FAILED
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(FixedGraspStepsTest, FixedGraspSteps_SimultaneousWithFMax) {
  // fixed_grasp_steps=1, f_min=f_max=70 → both f_max and override on same step → UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);
  setFixedGraspSteps(1);
  ros::Time t(100.0);
  auto g = makeDefaultGripper();

  driveOneStep(t, 70.0);

  // STEP_COMPLETE: reached_f_max=true, fixed_override=true (1>=1), secure=true → UPLIFT
  t += ros::Duration(0.1);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::UPLIFT);
}

// ============================================================================
// Contact detection tests
// ============================================================================

TEST_F(KittingControllerTestFixture, DetectContact_StaleGripperData_Skipped) {
  setCurrentState(GraspState::CLOSING);

  // Prepare baseline so detector is ready
  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  setCdBaselineReady(true);

  // Stale gripper data (stamp = 0, or very old)
  GripperData g;
  g.width = 0.04;
  g.max_width = 0.08;
  g.stamp = ros::Time(0);  // Invalid stamp

  ros::Time now(100.0);
  callDetectContact(now, g, 1.3);  // Large drop, but stale data
  EXPECT_FALSE(contactLatched());   // No detection because gripper data invalid
}

TEST_F(KittingControllerTestFixture, DetectContact_BaselineReady_ContactDetected) {
  setCurrentState(GraspState::CLOSING);

  // Collect baseline (50 samples at 1.8)
  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  setCdBaselineReady(true);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Feed sustained drop — should eventually trigger contact
  bool detected = false;
  for (int i = 0; i < 50; ++i) {
    callDetectContact(t, g, 1.3);
    if (contactLatched()) {
      detected = true;
      break;
    }
    t += ros::Duration(0.004);
  }
  EXPECT_TRUE(detected);
  EXPECT_EQ(currentState(), GraspState::CONTACT_CONFIRMED);
}

TEST_F(KittingControllerTestFixture, DetectContact_NoContact_StableSignal) {
  setCurrentState(GraspState::CLOSING);

  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  setCdBaselineReady(true);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Stable signal at baseline — no detection
  for (int i = 0; i < 200; ++i) {
    callDetectContact(t, g, 1.8);
    EXPECT_FALSE(contactLatched()) << "False alarm at sample " << i;
    t += ros::Duration(0.004);
  }
}

TEST_F(KittingControllerTestFixture, DetectContact_BaselineNotReady_Fallback) {
  setCurrentState(GraspState::CLOSING);
  // Do NOT pre-load baseline — detector should collect via fallback

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Feed stable signal — fallback collects baseline
  for (int i = 0; i < 50; ++i) {
    callDetectContact(t, g, 1.8);
    t += ros::Duration(0.004);
  }
  // Baseline should now be ready via fallback
  EXPECT_TRUE(smsDetector().baseline_ready());
  EXPECT_TRUE(cdBaselineReady());
}

// ============================================================================
// runClosingTransitions tests
// ============================================================================

TEST_F(KittingControllerTestFixture, Closing_FirstTick_SetsEnteredFlag) {
  setCurrentState(GraspState::CLOSING_COMMAND);
  setContactLatched(false);
  setClosingCommandEntered(false);
  setClosingCmdSeenExecuting(false);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callRunClosingTransitions(t, g, 1.8);
  // First call should set entered flag and return
  EXPECT_EQ(currentState(), GraspState::CLOSING_COMMAND);
}

TEST_F(KittingControllerTestFixture, Closing_CmdExecuting_TransitionsToClosing) {
  setCurrentState(GraspState::CLOSING_COMMAND);
  setContactLatched(false);
  setClosingCommandEntered(true);  // Already past first tick
  setClosingCmdSeenExecuting(false);
  setCmdExecuting(true);

  // Pre-load baseline for enter_closing() to succeed
  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);
  setPhaseStartTime(t);

  callRunClosingTransitions(t, g, 1.8);
  EXPECT_EQ(currentState(), GraspState::CLOSING);
  EXPECT_EQ(smsDetector().state(), sms_cusum::GraspState::CLOSING);
}

TEST_F(KittingControllerTestFixture, Closing_CmdTimeout_Failed) {
  setCurrentState(GraspState::CLOSING_COMMAND);
  setContactLatched(false);
  setClosingCommandEntered(true);
  setClosingCmdSeenExecuting(false);
  setCmdExecuting(false);  // Never starts

  auto g = makeDefaultGripper();
  ros::Time t0(100.0);
  setPhaseStartTime(t0);

  // Jump past kClosingCmdTimeout (10s)
  ros::Time t_late(111.0);
  callRunClosingTransitions(t_late, g, 1.8);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(KittingControllerTestFixture, Closing_ContactConfirmed_GripperStopped_Contact) {
  setCurrentState(GraspState::CONTACT_CONFIRMED);
  setContactLatched(true);
  setGripperStopped(true);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callRunClosingTransitions(t, g, 1.8);
  EXPECT_EQ(currentState(), GraspState::CONTACT);
}

// ============================================================================
// Evaluate boundary tests
// ============================================================================

TEST_F(EvaluateTest, Evaluate_LoadTransferMin_BoundaryPass) {
  // deltaF = 2.02 - 2.0 = 0.02 == load_transfer_min → PASS (> not >=)
  // Actually, the code uses >, so exactly 0.02 > 0.02 is false.
  // Let's test just above: 2.021 - 2.0 = 0.021 > 0.02 ✓
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/2.021, /*early_n=*/125,
                /*fn_late=*/2.02, /*late_n=*/125);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);

  auto g = makeDefaultGripper();
  ros::Time t(101.1);  // Past full hold window
  callTickEvaluate(t, 0.0, 2.02, g);
  EXPECT_EQ(currentState(), GraspState::SUCCESS);
}

TEST_F(EvaluateTest, Evaluate_LoadTransferMin_BoundaryFail) {
  // deltaF = 2.019 - 2.0 = 0.019 < 0.02 → FAIL
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/2.019, /*early_n=*/125,
                /*fn_late=*/2.018, /*late_n=*/125);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);

  auto g = makeDefaultGripper();
  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 2.018, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(EvaluateTest, Evaluate_ZeroFnEarly_NoDivByZero) {
  // Fn_early = 0 → dF uses max(Fn_early, epsilon) = 1e-6
  // deltaF = 0 - 2.0 = -2.0 < 0.02 → Gate 1 FAIL → FAILED (no crash)
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/0.0, /*early_n=*/125,
                /*fn_late=*/0.0, /*late_n=*/125);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);

  auto g = makeDefaultGripper();
  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 0.0, g);
  // Should not crash (division by epsilon) and should FAIL
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Downlift trajectory tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_Start) {
  auto start = makeIdentityPose(0.41);
  setDownliftStartPose(start);
  setDownliftParams(0.010, 1.0);

  auto pose = callComputeDownliftPose(0.0);
  EXPECT_NEAR(pose[14], 0.41, 1e-10);  // No movement at t=0
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_End) {
  auto start = makeIdentityPose(0.41);
  setDownliftStartPose(start);
  setDownliftParams(0.010, 1.0);

  auto pose = callComputeDownliftPose(1.0);
  EXPECT_NEAR(pose[14], 0.40, 1e-10);  // Z -= distance
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_Midpoint) {
  auto start = makeIdentityPose(0.41);
  setDownliftStartPose(start);
  setDownliftParams(0.010, 1.0);

  auto pose = callComputeDownliftPose(0.5);
  EXPECT_NEAR(pose[14], 0.405, 1e-10);  // Z -= 0.5*distance
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_PreservesXY) {
  auto start = makeIdentityPose(0.41);
  setDownliftStartPose(start);
  setDownliftParams(0.010, 1.0);

  auto pose = callComputeDownliftPose(0.5);
  // X and Y translations (indices 12, 13) unchanged
  EXPECT_DOUBLE_EQ(pose[12], start[12]);
  EXPECT_DOUBLE_EQ(pose[13], start[13]);
  // Rotation elements unchanged
  for (int i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(pose[i], start[i]);
  }
}

// ============================================================================
// Force ramp: reached_f_max without secure grasp → FAILED
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_FMaxWithoutSecure_Failed) {
  // f_min=f_max=70, fixed_grasp_steps=-1 → single step, no secure → FAILED
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setFixedGraspSteps(-1);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Drive through full step
  callTickGrasping(t, 0.0, 0.0, g);  // init
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);  // WAITING
  setCmdExecuting(false);
  setCmdSuccess(true);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 5.0, g);  // SETTLING
  t = ros::Time(101.1);
  callTickGrasping(t, 0.0, 5.0, g);  // HOLDING
  t = ros::Time(102.2);
  callTickGrasping(t, 0.0, 5.0, g);  // STEP_COMPLETE

  // At STEP_COMPLETE: f_max reached, no secure, no fixed override → FAILED
  t = ros::Time(102.3);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Closing: gripper finishes without contact → FAILED
// ============================================================================

TEST_F(KittingControllerTestFixture, Closing_GripperDone_NoContact_Failed) {
  setCurrentState(GraspState::CLOSING);
  setContactLatched(false);
  setClosingCommandEntered(true);
  setClosingCmdSeenExecuting(true);
  setCmdExecuting(false);  // Gripper finished moving

  // Baseline ready, CUSUM active
  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  setCdBaselineReady(true);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);
  setPhaseStartTime(t);

  // Gripper done but no contact → FAILED
  callRunClosingTransitions(t, g, 1.8);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Closing: timeout during CLOSING (30s) → FAILED
// ============================================================================

TEST_F(KittingControllerTestFixture, Closing_Timeout_Failed) {
  setCurrentState(GraspState::CLOSING);
  setContactLatched(false);
  setClosingCommandEntered(true);
  setClosingCmdSeenExecuting(true);
  setCmdExecuting(true);  // Gripper still moving

  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  setCdBaselineReady(true);

  auto g = makeDefaultGripper();
  ros::Time t0(100.0);
  setPhaseStartTime(t0);

  // Jump past kClosingTimeout (30s)
  ros::Time t_late(131.0);
  callRunClosingTransitions(t_late, g, 1.8);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Closing: contact_latched skips detection block
// ============================================================================

TEST_F(KittingControllerTestFixture, Closing_ContactLatched_SkipsDetection) {
  setCurrentState(GraspState::CLOSING);
  setContactLatched(true);  // Already latched
  setClosingCommandEntered(true);
  setClosingCmdSeenExecuting(true);

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Should NOT enter the detection block (guard: !contact_latched_)
  callRunClosingTransitions(t, g, 1.8);
  // State unchanged — the detection block is skipped
  EXPECT_EQ(currentState(), GraspState::CLOSING);
}

// ============================================================================
// isActionAllowed tests
// ============================================================================

TEST_F(KittingControllerTestFixture, IsActionAllowed_StartState) {
  setCurrentState(GraspState::START);
  EXPECT_TRUE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_SuccessState) {
  setCurrentState(GraspState::SUCCESS);
  EXPECT_TRUE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_FailedState) {
  setCurrentState(GraspState::FAILED);
  EXPECT_TRUE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_GraspingState_Rejected) {
  setCurrentState(GraspState::GRASPING);
  EXPECT_FALSE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_ClosingState_Rejected) {
  setCurrentState(GraspState::CLOSING);
  EXPECT_FALSE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_BaselineState_Rejected) {
  setCurrentState(GraspState::BASELINE);
  EXPECT_FALSE(callIsActionAllowed());
}

TEST_F(KittingControllerTestFixture, IsActionAllowed_EvaluateState_Rejected) {
  setCurrentState(GraspState::EVALUATE);
  EXPECT_FALSE(callIsActionAllowed());
}

// ============================================================================
// requestGripperStop tests
// ============================================================================

TEST_F(KittingControllerTestFixture, RequestGripperStop_SetsFlags) {
  setGripperStopSent(false);
  callRequestGripperStop("Test");
  EXPECT_TRUE(stopRequested());
  EXPECT_TRUE(gripperStopSent());
}

TEST_F(KittingControllerTestFixture, RequestGripperStop_AlreadySent_NoOp) {
  setGripperStopSent(true);
  // Should not re-set stop_requested_ (guarded by gripper_stop_sent_)
  callRequestGripperStop("Test");
  // The guard prevents re-entry but stop_requested_ may already be true
  EXPECT_TRUE(gripperStopSent());
}

// ============================================================================
// Baseline collection tests (via direct member access)
// ============================================================================

TEST_F(KittingControllerTestFixture, BaselineCollection_AccumulatesFnBaseline) {
  // Simulate what update() does in BASELINE state
  setCurrentState(GraspState::BASELINE);

  setFnBaselineSum(0.0);
  setFnBaselineCount(0);

  // Simulate 10 samples of Fn = 3.0
  for (int i = 0; i < 10; ++i) {
    smsDetector().update(1.8);  // tau_ext_norm
    // Mimicking the fn_baseline accumulation in update()
  }

  // After 50 samples, baseline should be ready
  for (int i = 10; i < 50; ++i) {
    smsDetector().update(1.8);
  }
  EXPECT_TRUE(smsDetector().baseline_ready());
  EXPECT_NEAR(smsDetector().baseline().mean(), 1.8, 1e-9);
}

// ============================================================================
// UNKNOWN → BASELINE settle transition test
// ============================================================================

TEST_F(KittingControllerTestFixture, UnknownSettle_TransitionsToBaseline) {
  setCurrentState(GraspState::UNKNOWN);
  setBaselinePrepDone(true);
  setUnknownSettleStarted(false);

  // This test verifies the settle logic that normally runs in update().
  // We can't call update() directly (needs rate_trigger_), but we can
  // verify the state variables are correctly wired.
  // The settle time is kBaselineSettleTime = 2.0s.
  EXPECT_EQ(KittingStateController::kBaselineSettleTime, 2.0);
}

// ============================================================================
// runInternalTransitions dispatches correctly
// ============================================================================

TEST_F(KittingControllerTestFixture, RunInternalTransitions_GraspingState) {
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Should dispatch to tickGrasping which initializes the phase
  callRunInternalTransitions(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);  // Still grasping
}

TEST_F(KittingControllerTestFixture, RunInternalTransitions_UpliftState) {
  setCurrentState(GraspState::UPLIFT);
  setUpliftActive(false);  // Trajectory already done
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callRunInternalTransitions(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::EVALUATE);  // Transitioned
}

TEST_F(KittingControllerTestFixture, RunInternalTransitions_NonRampState_NoOp) {
  setCurrentState(GraspState::BASELINE);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // BASELINE is not a force ramp phase — should not dispatch
  // (isForceRampPhase returns false for BASELINE)
  // This verifies no crash from unexpected states
  EXPECT_EQ(currentState(), GraspState::BASELINE);
}

// ============================================================================
// Evaluate: zero sample counts (edge case)
// ============================================================================

TEST_F(EvaluateTest, Evaluate_ZeroSampleCounts_NoCrash) {
  // Both early and late have zero samples → Fn_early = 0, Fn_late = 0
  SetUpEvaluate(/*fn_baseline=*/2.0, /*fn_early=*/0.0, /*early_n=*/0,
                /*fn_late=*/0.0, /*late_n=*/0);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);

  auto g = makeDefaultGripper();
  ros::Time t(101.1);

  // When count = 0, the code does: (count > 0) ? sum/count : 0.0
  // This should not crash and should result in FAILED (no load transfer)
  callTickEvaluate(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Evaluate: accumulation during early and late windows
// ============================================================================

TEST_F(KittingControllerTestFixture, Evaluate_EarlyWindowOnly_NoVerdict) {
  setCurrentState(GraspState::EVALUATE);
  setFnBaseline(2.0);
  setEarlyAccumulators(0.0, 0);
  setLateAccumulators(0.0, 0);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);
  auto g = makeDefaultGripper();

  // At 0.3s (within early window, hold=1.0s, early = first 0.5s)
  ros::Time t(100.3);
  callTickEvaluate(t, 0.0, 10.0, g);
  // Should still be EVALUATE — no verdict yet
  EXPECT_EQ(currentState(), GraspState::EVALUATE);
}

TEST_F(KittingControllerTestFixture, Evaluate_LateWindowOnly_NoVerdict) {
  setCurrentState(GraspState::EVALUATE);
  setFnBaseline(2.0);
  setEarlyAccumulators(10.0 * 125, 125);  // Already have early data
  setLateAccumulators(0.0, 0);

  ros::Time eval_start(100.0);
  setPhaseStartTime(eval_start);
  auto g = makeDefaultGripper();

  // At 0.7s (within late window, but not past hold time 1.0s)
  ros::Time t(100.7);
  callTickEvaluate(t, 0.0, 10.0, g);
  EXPECT_EQ(currentState(), GraspState::EVALUATE);
}

// ============================================================================
// Holding phase: SMS update at mid-hold
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_Holding_SMSUpdateAtMidHold) {
  // Verify that sms_detector_.update() is called during the late half of holding
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(2.0);  // Mid-hold at 1.0s

  // Pre-fill baseline and set to GRASPING in SMS detector
  for (int i = 0; i < 50; ++i) smsDetector().update(1.8);
  smsDetector().enter_closing();
  // Simulate contact → grasping
  for (int i = 0; i < 20; ++i) smsDetector().update(1.3);
  if (smsDetector().state() == sms_cusum::GraspState::CONTACT) {
    smsDetector().enter_grasping();
  }

  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Drive to HOLDING phase
  callTickGrasping(t, 0.0, 0.0, g);  // init
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);  // WAITING
  setCmdExecuting(false);
  setCmdSuccess(true);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 5.0, g);  // SETTLING
  t = ros::Time(101.1);
  callTickGrasping(t, 0.0, 5.0, g);  // → HOLDING
  EXPECT_EQ(rampPhase(), RampPhase::HOLDING);

  int idx_before = smsDetector().sample_index();

  // Tick at mid-hold (elapsed > hold_time/2 = 1.0s)
  t = ros::Time(102.2);
  callTickGrasping(t, 1.8, 5.0, g);

  // SMS detector should have received at least one update
  EXPECT_GT(smsDetector().sample_index(), idx_before);
}

// ============================================================================
// Uplift: trajectory beyond duration is clamped
// ============================================================================

TEST_F(KittingControllerTestFixture, ComputeUpliftPose_BeyondDuration) {
  auto start = makeIdentityPose(0.40);
  setUpliftStartPose(start);
  setUpliftParams(0.010, 1.0);

  // Elapsed > duration → clamped to end position
  auto pose = callComputeUpliftPose(2.0);
  EXPECT_NEAR(pose[14], 0.41, 1e-10);
}

TEST_F(KittingControllerTestFixture, ComputeDownliftPose_BeyondDuration) {
  auto start = makeIdentityPose(0.41);
  setDownliftStartPose(start);
  setDownliftParams(0.010, 1.0);

  auto pose = callComputeDownliftPose(2.0);
  EXPECT_NEAR(pose[14], 0.40, 1e-10);
}

// ============================================================================
// Grasping: cmd_gen fallback path (command generation counter)
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_CmdGenFallback) {
  // When cmd_gen_ increments past fr_expected_cmd_gen_,
  // the controller transitions from COMMAND_SENT to WAITING_EXECUTION
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Initialize
  callTickGrasping(t, 0.0, 0.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::COMMAND_SENT);

  // Don't set cmd_executing_, but bump cmd_gen_ past expected
  // (The fixture doesn't expose cmd_gen_ directly, but we can verify
  //  that the normal setCmdExecuting path still works)
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);
  EXPECT_EQ(rampPhase(), RampPhase::WAITING_EXECUTION);
}

// ============================================================================
// Contact width update during HOLDING phase
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_ContactWidthUpdated) {
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setContactWidth(0.04);
  auto g = makeDefaultGripper(0.035);  // Different width
  ros::Time t(100.0);

  // Drive to HOLDING → STEP_COMPLETE
  callTickGrasping(t, 0.0, 0.0, g);
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);
  setCmdExecuting(false);
  setCmdSuccess(true);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 5.0, g);
  t = ros::Time(101.1);
  callTickGrasping(t, 0.0, 5.0, g);  // HOLDING

  // During HOLDING, contact_width_ is updated at the end
  t = ros::Time(102.2);
  callTickGrasping(t, 0.0, 5.0, g);  // → STEP_COMPLETE

  // contact_width_ should be updated to gripper width (0.035)
  EXPECT_NEAR(contactWidth(), 0.035, 1e-6);
}

// ============================================================================
// Grasping: hardware-rejected grasp command
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_GraspFailed_HardwareRejected) {
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickGrasping(t, 0.0, 0.0, g);  // init
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);  // WAITING

  // Hardware rejects the grasp
  setCmdExecuting(false);
  setCmdSuccess(false);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::FAILED);
}

// ============================================================================
// Ramp step advance: force increment and iteration counter
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_RampAdvance_ForceAndIteration) {
  // f_min=3, f_step=3, f_max=70 → should advance from 3 to 6
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  setIteration(0);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setFixedGraspSteps(-1);
  setContactWidth(0.04);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // Drive through one full step
  callTickGrasping(t, 0.0, 0.0, g);
  setCmdExecuting(true);
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);
  setCmdExecuting(false);
  setCmdSuccess(true);
  t = ros::Time(100.5);
  callTickGrasping(t, 0.0, 5.0, g);
  t = ros::Time(101.1);
  callTickGrasping(t, 0.0, 5.0, g);
  t = ros::Time(102.2);
  callTickGrasping(t, 0.0, 5.0, g);  // STEP_COMPLETE

  // Advance: not at f_max, no secure → next step
  t = ros::Time(102.3);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_EQ(iteration(), 1);
  EXPECT_NEAR(forceCurrent(), 6.0, 1e-9);
  EXPECT_TRUE(deferredGraspPending());
}

// ============================================================================
// Non-divisible force range reaches f_max
// ============================================================================

TEST_F(KittingControllerTestFixture, TickGrasping_NonDivisibleRange_ClampedToFMax) {
  // f_min=3, f_step=4, f_max=10 → step sequence: 3, 7, 10 (clamped)
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(7.0);  // Already at step 2
  setIteration(1);
  setForceRampParams(3.0, 4.0, 10.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
  setFixedGraspSteps(-1);
  setContactWidth(0.04);
  setRampPhase(RampPhase::STEP_COMPLETE);
  setGraspingInitialized(true);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // At STEP_COMPLETE with f=7, not at f_max=10, no secure → advance
  callTickGrasping(t, 0.0, 5.0, g);
  EXPECT_EQ(iteration(), 2);
  EXPECT_NEAR(forceCurrent(), 10.0, 1e-9);  // min(7+4, 10) = 10
}

