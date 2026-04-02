// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Tier 1 — Pure unit tests for static/inline utility functions.
// Uses the fixture to access private static methods via friend class.

#include <array>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "test_helpers.h"

using franka_kitting_controller::GraspState;
using franka_kitting_controller::KittingStateController;

// ============================================================================
// arrayNorm tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ArrayNorm_ZeroVector) {
  std::array<double, 7> v = {};
  EXPECT_DOUBLE_EQ(callArrayNorm(v), 0.0);
}

TEST_F(KittingControllerTestFixture, ArrayNorm_UnitVector) {
  std::array<double, 7> v = {{1.0, 0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(callArrayNorm(v), 1.0);
}

TEST_F(KittingControllerTestFixture, ArrayNorm_KnownValue_3_4) {
  std::array<double, 7> v = {{3.0, 4.0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(callArrayNorm(v), 5.0);
}

TEST_F(KittingControllerTestFixture, ArrayNorm_AllOnes) {
  std::array<double, 7> v = {{1, 1, 1, 1, 1, 1, 1}};
  EXPECT_DOUBLE_EQ(callArrayNorm(v), std::sqrt(7.0));
}

TEST_F(KittingControllerTestFixture, ArrayNorm_SixElement) {
  std::array<double, 6> v = {{1, 2, 3, 4, 5, 6}};
  double expected = std::sqrt(1 + 4 + 9 + 16 + 25 + 36);
  EXPECT_NEAR(callArrayNorm(v), expected, 1e-12);
}

TEST_F(KittingControllerTestFixture, ArrayNorm_NegativeValues) {
  std::array<double, 7> v = {{-3.0, -4.0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(callArrayNorm(v), 5.0);
}

// ============================================================================
// resolveParam tests
// ============================================================================

TEST_F(KittingControllerTestFixture, ResolveParam_PositiveMsgValue) {
  EXPECT_DOUBLE_EQ(callResolveParam(5.0, 3.0), 5.0);
}

TEST_F(KittingControllerTestFixture, ResolveParam_ZeroMsgValue) {
  EXPECT_DOUBLE_EQ(callResolveParam(0.0, 3.0), 3.0);
}

TEST_F(KittingControllerTestFixture, ResolveParam_NegativeMsgValue) {
  EXPECT_DOUBLE_EQ(callResolveParam(-1.0, 3.0), 3.0);
}

TEST_F(KittingControllerTestFixture, ResolveParam_SmallPositive) {
  EXPECT_DOUBLE_EQ(callResolveParam(0.001, 10.0), 0.001);
}

TEST_F(KittingControllerTestFixture, ResolveParam_ZeroDefault) {
  EXPECT_DOUBLE_EQ(callResolveParam(0.0, 0.0), 0.0);
}

// ============================================================================
// stateToString tests
// ============================================================================

TEST_F(KittingControllerTestFixture, StateToString_AllStates) {
  EXPECT_STREQ(callStateToString(GraspState::START), "START");
  EXPECT_STREQ(callStateToString(GraspState::BASELINE), "BASELINE");
  EXPECT_STREQ(callStateToString(GraspState::CLOSING_COMMAND), "CLOSING_COMMAND");
  EXPECT_STREQ(callStateToString(GraspState::CLOSING), "CLOSING");
  EXPECT_STREQ(callStateToString(GraspState::CONTACT_CONFIRMED), "CONTACT_CONFIRMED");
  EXPECT_STREQ(callStateToString(GraspState::CONTACT), "CONTACT");
  EXPECT_STREQ(callStateToString(GraspState::GRASPING), "GRASPING");
  EXPECT_STREQ(callStateToString(GraspState::UPLIFT), "UPLIFT");
  EXPECT_STREQ(callStateToString(GraspState::EVALUATE), "EVALUATE");
  EXPECT_STREQ(callStateToString(GraspState::SUCCESS), "SUCCESS");
  EXPECT_STREQ(callStateToString(GraspState::FAILED), "FAILED");
}

TEST_F(KittingControllerTestFixture, StateToString_Unknown) {
  auto unknown = static_cast<GraspState>(999);
  EXPECT_STREQ(callStateToString(unknown), "UNKNOWN");
}

// ============================================================================
// isClosingPhase tests
// ============================================================================

TEST_F(KittingControllerTestFixture, IsClosingPhase_ClosingStates) {
  EXPECT_TRUE(callIsClosingPhase(GraspState::CLOSING_COMMAND));
  EXPECT_TRUE(callIsClosingPhase(GraspState::CLOSING));
  EXPECT_TRUE(callIsClosingPhase(GraspState::CONTACT_CONFIRMED));
}

TEST_F(KittingControllerTestFixture, IsClosingPhase_NonClosingStates) {
  EXPECT_FALSE(callIsClosingPhase(GraspState::START));
  EXPECT_FALSE(callIsClosingPhase(GraspState::BASELINE));
  EXPECT_FALSE(callIsClosingPhase(GraspState::CONTACT));
  EXPECT_FALSE(callIsClosingPhase(GraspState::GRASPING));
  EXPECT_FALSE(callIsClosingPhase(GraspState::UPLIFT));
  EXPECT_FALSE(callIsClosingPhase(GraspState::EVALUATE));
  EXPECT_FALSE(callIsClosingPhase(GraspState::SUCCESS));
  EXPECT_FALSE(callIsClosingPhase(GraspState::FAILED));
}

// ============================================================================
// isForceRampPhase tests
// ============================================================================

TEST_F(KittingControllerTestFixture, IsForceRampPhase_ForceRampStates) {
  EXPECT_TRUE(callIsForceRampPhase(GraspState::GRASPING));
  EXPECT_TRUE(callIsForceRampPhase(GraspState::UPLIFT));
  EXPECT_TRUE(callIsForceRampPhase(GraspState::EVALUATE));
}

TEST_F(KittingControllerTestFixture, IsForceRampPhase_NonForceRampStates) {
  EXPECT_FALSE(callIsForceRampPhase(GraspState::START));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::BASELINE));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::CLOSING_COMMAND));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::CLOSING));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::CONTACT_CONFIRMED));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::CONTACT));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::SUCCESS));
  EXPECT_FALSE(callIsForceRampPhase(GraspState::FAILED));
}
