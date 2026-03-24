// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Tier 2 — State machine tests using KittingControllerTestFixture.
// Tests tick functions, state transitions, trajectory math, and 3-gate slip detection.

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
  widthSamples().push_back(1.0);
  widthSamples().push_back(2.0);

  callResetForceRampState();

  EXPECT_DOUBLE_EQ(forceCurrent(), 0.0);
  EXPECT_EQ(iteration(), 0);
  EXPECT_TRUE(widthSamples().empty());
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
  // Use f_min = f_max so there's only one step (last step)
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
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
// tickEvaluate tests — 3-gate slip detection
// ============================================================================

// Helper: set up EVALUATE state with pre-computed accumulator data
class EvaluateTest : public KittingControllerTestFixture {
 protected:
  void SetUpEvaluate(double fn_pre, double sigma_pre, int pre_n,
                     double fn_early, int early_n,
                     double fn_late, int late_n,
                     const std::vector<double>& widths = {}) {
    setCurrentState(GraspState::EVALUATE);

    // Compute sum and sum_sq from mean and sigma
    double pre_sum = fn_pre * pre_n;
    double pre_sum_sq = (sigma_pre * sigma_pre + fn_pre * fn_pre) * pre_n;
    setPreAccumulators(pre_sum, pre_sum_sq, pre_n);

    setEarlyAccumulators(fn_early * early_n, early_n);
    setLateAccumulators(fn_late * late_n, late_n);

    widthSamples().clear();
    if (widths.empty()) {
      // Generate stable width samples (all same value)
      for (int i = 0; i < 250; ++i) {
        widthSamples().push_back(0.040);
      }
    } else {
      for (double w : widths) {
        widthSamples().push_back(w);
      }
    }
  }
};

TEST_F(EvaluateTest, AllGatesPass_Success) {
  // Gate 1: deltaF = Fn_early - Fn_pre = 10 - 2 = 8 > max(3*0.5, 1.5) = 1.5 ✓
  // Gate 2: dF = (10 - 9.5) / 10 = 0.05 ≤ 0.15 ✓
  // Gate 3: All widths identical → P95-P5 = 0 ≤ 0.0005 ✓
  SetUpEvaluate(/*fn_pre=*/2.0, /*sigma=*/0.5, /*pre_n=*/125,
                /*fn_early=*/10.0, /*early_n=*/125,
                /*fn_late=*/9.5, /*late_n=*/125);

  // uplift_hold = 1.0s, so kEarlyEnd = 0.5, kLateEnd = 1.0
  // We need elapsed >= kLateEnd to trigger evaluation
  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  // Call at elapsed = 1.1s (past kLateEnd)
  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::SUCCESS);
}

TEST_F(EvaluateTest, Gate1Fail_NoLoadTransfer) {
  // Gate 1 FAIL: deltaF = 3 - 2.5 = 0.5 < max(3*0.1, 1.5) = 1.5
  SetUpEvaluate(/*fn_pre=*/2.5, /*sigma=*/0.1, /*pre_n=*/125,
                /*fn_early=*/3.0, /*early_n=*/125,
                /*fn_late=*/3.0, /*late_n=*/125);

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
  SetUpEvaluate(/*fn_pre=*/2.0, /*sigma=*/0.1, /*pre_n=*/125,
                /*fn_early=*/10.0, /*early_n=*/125,
                /*fn_late=*/8.0, /*late_n=*/125);

  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  ros::Time t(101.1);
  callTickEvaluate(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::FAILED);
}

TEST_F(EvaluateTest, Gate3Fail_JawWidening) {
  // Gate 1 PASS, Gate 2 PASS, Gate 3 FAIL: P95 - P5 > 0.0005
  // Create widths that spread: 50% at 0.040, 50% at 0.041 (spread = 0.001 > 0.0005)
  std::vector<double> widths;
  for (int i = 0; i < 125; ++i) widths.push_back(0.040);
  for (int i = 0; i < 125; ++i) widths.push_back(0.041);

  SetUpEvaluate(/*fn_pre=*/2.0, /*sigma=*/0.1, /*pre_n=*/125,
                /*fn_early=*/10.0, /*early_n=*/125,
                /*fn_late=*/9.8, /*late_n=*/125,
                widths);

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
  setPreAccumulators(0.0, 0.0, 0);
  setEarlyAccumulators(0.0, 0);
  setLateAccumulators(0.0, 0);
  widthSamples().clear();

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
  setPreAccumulators(0.0, 0.0, 0);
  setEarlyAccumulators(0.0, 0);
  setLateAccumulators(0.0, 0);
  widthSamples().clear();

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
  // On last step's STEP_COMPLETE → transition to UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(69.0);  // 69 + 3 > 70 → last step
  setIteration(22);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 3.0, 70.0, 0.010, 0.01, 1.0);

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
  // When fr_f_current_ == f_max, STEP_COMPLETE should transition to UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(10.0);  // Already at f_max
  setIteration(2);
  setGraspingInitialized(true);
  setPhaseStartTime(ros::Time(100.0));
  setRampPhase(RampPhase::STEP_COMPLETE);
  setForceRampParams(3.0, 4.0, 10.0, 0.010, 0.01, 1.0);

  auto g = makeDefaultGripper();
  ros::Time t(101.0);
  callTickGrasping(t, 0.0, 5.0, g);

  EXPECT_EQ(currentState(), GraspState::UPLIFT);
  EXPECT_TRUE(upliftActive());
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

TEST(ConstantsTest, WidthSamplesConsistency) {
  // kMaxWidthSamples should be kMaxUpliftHold * kWidthSamplesPerSec
  EXPECT_EQ(KittingStateController::kMaxWidthSamples,
            static_cast<int>(KittingStateController::kMaxUpliftHold) *
                KittingStateController::kWidthSamplesPerSec);
}

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
  // Use f_min = f_max so there's only one step (last step).
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(70.0);
  setIteration(0);
  setForceRampParams(70.0, 3.0, 70.0, 0.010, 0.01, 1.0);
  setGraspSettleTime(0.5);
  setGraspForceHoldTime(1.0);
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

  // Settle time passes → HOLDING (last step, W_pre accumulates)
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
  // Now manually set up accumulators to simulate a secure grasp
  // fn_pre was accumulated during W_pre (≈8.0), fn_early/late ≈ 15.0 (load transferred, stable)
  setPreAccumulators(8.0 * 125, 65.0 * 125, 125);  // mean=8.0, var≈1.0
  setEarlyAccumulators(15.0 * 125, 125);
  setLateAccumulators(14.8 * 125, 125);
  widthSamples().clear();
  for (int i = 0; i < 250; ++i) {
    widthSamples().push_back(0.040);  // Stable width
  }

  ros::Time eval_start(102.0);
  setPhaseStartTime(eval_start);

  // Jump past the hold window (1.0s)
  t = ros::Time(103.1);
  callTickEvaluate(t, 0.0, 15.0, g);

  EXPECT_EQ(currentState(), GraspState::SUCCESS);
}

