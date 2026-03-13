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

// ============================================================================
// resetForceRampState tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ResetForceRampState_ClearsAll) {
  // Dirty up state
  setForceCurrent(10.0);
  setIteration(5);
  setGraspCmdSeenExecuting(true);
  setGraspStabilizing(true);
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
  // Simulate: settle → cmd starts → cmd finishes → W_pre window → UPLIFT
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(3.0);
  auto g = makeDefaultGripper();
  ros::Time t0(100.0);

  // Step 1: Initialize phase
  callTickGrasping(t0, 0.0, 0.0, g);

  // Step 2: After settle delay, cmd not yet executing
  ros::Time t1(100.15);
  callTickGrasping(t1, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // Step 3: cmd starts executing
  setCmdExecuting(true);
  ros::Time t2(100.2);
  callTickGrasping(t2, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // Step 4: cmd still executing
  ros::Time t3(100.5);
  callTickGrasping(t3, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // Step 5: cmd finishes → starts stabilization + W_pre accumulation
  setCmdExecuting(false);
  ros::Time t4(100.7);
  callTickGrasping(t4, 0.0, 5.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);

  // Step 6: W_pre window = uplift_hold/2 = 0.5s after stabilization start
  // t4 was the stabilization start. Feed samples until 0.5s passes.
  for (int i = 0; i < 125; ++i) {  // 125 ticks at 250Hz = 0.5s
    ros::Time t_tick(100.7 + 0.004 * (i + 1));
    callTickGrasping(t_tick, 0.0, 5.0, g);
  }

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

  EXPECT_EQ(currentState(), GraspState::SLIP);
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

  EXPECT_EQ(currentState(), GraspState::SLIP);
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

  EXPECT_EQ(currentState(), GraspState::SLIP);
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
// tickSlip tests
// ============================================================================

TEST_F(KittingControllerTestFixture, TickSlip_InitiatesDownlift) {
  setCurrentState(GraspState::SLIP);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickSlip(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::DOWNLIFT);
  EXPECT_TRUE(downliftActive());
}

// ============================================================================
// tickDownlift tests
// ============================================================================

TEST_F(KittingControllerTestFixture, TickDownlift_TrajectoryRunning) {
  setCurrentState(GraspState::DOWNLIFT);
  setDownliftActive(true);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickDownlift(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::DOWNLIFT);
}

TEST_F(KittingControllerTestFixture, TickDownlift_TrajectoryDone) {
  setCurrentState(GraspState::DOWNLIFT);
  setDownliftActive(false);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  callTickDownlift(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::SETTLING);
}

// ============================================================================
// tickSettling tests
// ============================================================================

TEST_F(KittingControllerTestFixture, TickSettling_StillSettling) {
  setCurrentState(GraspState::SETTLING);
  setForceCurrent(3.0);
  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  // elapsed = 0.2s < uplift_hold/2 = 0.5s
  ros::Time t(100.2);
  callTickSettling(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::SETTLING);
}

TEST_F(KittingControllerTestFixture, TickSettling_ForceIncrement) {
  // elapsed ≥ uplift_hold/2 and f_current + f_step ≤ f_max → GRASPING retry
  setCurrentState(GraspState::SETTLING);
  setForceCurrent(3.0);
  setIteration(0);
  setForceRampParams(3.0, 3.0, 30.0, 0.010, 0.01, 1.0);
  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper(0.04);

  // elapsed = 0.6s ≥ uplift_hold/2 = 0.5s
  ros::Time t(100.6);
  callTickSettling(t, 0.0, 0.0, g);

  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_DOUBLE_EQ(forceCurrent(), 6.0);  // 3.0 + 3.0
  EXPECT_EQ(iteration(), 1);
}

TEST_F(KittingControllerTestFixture, TickSettling_MaxForceExceeded) {
  // f_current + f_step > f_max → FAILED
  setCurrentState(GraspState::SETTLING);
  setForceCurrent(28.0);
  setIteration(8);
  setForceRampParams(3.0, 3.0, 30.0, 0.010, 0.01, 1.0);
  ros::Time phase_start(100.0);
  setPhaseStartTime(phase_start);
  auto g = makeDefaultGripper();

  ros::Time t(100.6);
  callTickSettling(t, 0.0, 0.0, g);

  // f_current becomes 31.0, which > f_max 30.0 → FAILED
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
  setCurrentState(GraspState::GRASPING);
  setForceCurrent(5.0);
  setIteration(0);
  auto g = makeDefaultGripper();

  // --- GRASPING phase ---
  ros::Time t(100.0);

  // Init phase
  callTickGrasping(t, 0.0, 0.0, g);

  // Past settle delay
  t = ros::Time(100.15);
  callTickGrasping(t, 0.0, 0.0, g);

  // cmd starts
  setCmdExecuting(true);
  t = ros::Time(100.2);
  callTickGrasping(t, 0.0, 0.0, g);

  // cmd finishes
  setCmdExecuting(false);
  t = ros::Time(100.7);
  callTickGrasping(t, 0.0, 8.0, g);

  // W_pre accumulation (0.5s of samples at 250Hz)
  for (int i = 0; i < 125; ++i) {
    t = ros::Time(100.7 + 0.004 * (i + 1));
    callTickGrasping(t, 0.0, 8.0, g);
  }
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

// ============================================================================
// Full force ramp retry cycle: EVALUATE→SLIP→DOWNLIFT→SETTLING→GRASPING
// ============================================================================

TEST_F(KittingControllerTestFixture, RetryCycle_SlipToGrasping) {
  // Start at SLIP
  setCurrentState(GraspState::SLIP);
  setForceCurrent(3.0);
  setIteration(0);
  auto g = makeDefaultGripper();
  ros::Time t(100.0);

  // SLIP → DOWNLIFT
  callTickSlip(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::DOWNLIFT);
  EXPECT_TRUE(downliftActive());

  // DOWNLIFT running
  t = ros::Time(101.0);
  callTickDownlift(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::DOWNLIFT);

  // DOWNLIFT done → SETTLING
  setDownliftActive(false);
  t = ros::Time(102.0);
  callTickDownlift(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::SETTLING);

  // SETTLING waits for uplift_hold/2 (0.5s)
  t = ros::Time(102.3);
  callTickSettling(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::SETTLING);

  // SETTLING done → increments force → GRASPING
  t = ros::Time(102.6);
  callTickSettling(t, 0.0, 0.0, g);
  EXPECT_EQ(currentState(), GraspState::GRASPING);
  EXPECT_DOUBLE_EQ(forceCurrent(), 6.0);  // 3.0 + 3.0
  EXPECT_EQ(iteration(), 1);
}
