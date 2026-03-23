// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// Tier 1 — Pure unit tests for static/inline utility functions.
// These tests exercise code directly from the header with no fixture needed.

#include <array>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <franka_kitting_controller/kitting_state_controller.h>

using franka_kitting_controller::GraspState;
using franka_kitting_controller::KittingStateController;

// ============================================================================
// arrayNorm tests
// ============================================================================

TEST(ArrayNormTest, ZeroVector) {
  std::array<double, 7> v = {};
  EXPECT_DOUBLE_EQ(KittingStateController::arrayNorm(v), 0.0);
}

TEST(ArrayNormTest, UnitVector) {
  std::array<double, 7> v = {{1.0, 0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(KittingStateController::arrayNorm(v), 1.0);
}

TEST(ArrayNormTest, KnownValue_3_4) {
  // Classic 3-4-5 triangle
  std::array<double, 7> v = {{3.0, 4.0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(KittingStateController::arrayNorm(v), 5.0);
}

TEST(ArrayNormTest, AllOnes) {
  std::array<double, 7> v = {{1, 1, 1, 1, 1, 1, 1}};
  EXPECT_DOUBLE_EQ(KittingStateController::arrayNorm(v), std::sqrt(7.0));
}

TEST(ArrayNormTest, SixElement) {
  // arrayNorm is templated — verify it works for 6-element arrays too (wrench)
  std::array<double, 6> v = {{1, 2, 3, 4, 5, 6}};
  double expected = std::sqrt(1 + 4 + 9 + 16 + 25 + 36);
  EXPECT_NEAR(KittingStateController::arrayNorm(v), expected, 1e-12);
}

TEST(ArrayNormTest, NegativeValues) {
  // Norm should be same regardless of sign
  std::array<double, 7> v = {{-3.0, -4.0, 0, 0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(KittingStateController::arrayNorm(v), 5.0);
}

// ============================================================================
// resolveParam tests
// ============================================================================

TEST(ResolveParamTest, PositiveMsgValue) {
  // msg_value > 0 → use msg_value
  EXPECT_DOUBLE_EQ(KittingStateController::resolveParam(5.0, 3.0), 5.0);
}

TEST(ResolveParamTest, ZeroMsgValue) {
  // msg_value == 0 → use default
  EXPECT_DOUBLE_EQ(KittingStateController::resolveParam(0.0, 3.0), 3.0);
}

TEST(ResolveParamTest, NegativeMsgValue) {
  // msg_value < 0 → use default
  EXPECT_DOUBLE_EQ(KittingStateController::resolveParam(-1.0, 3.0), 3.0);
}

TEST(ResolveParamTest, SmallPositiveMsgValue) {
  // Any positive value (even tiny) should be used
  EXPECT_DOUBLE_EQ(KittingStateController::resolveParam(0.001, 10.0), 0.001);
}

TEST(ResolveParamTest, ZeroDefault) {
  // Zero default with zero msg → return 0
  EXPECT_DOUBLE_EQ(KittingStateController::resolveParam(0.0, 0.0), 0.0);
}

// ============================================================================
// stateToString tests
// ============================================================================

TEST(StateToStringTest, AllStates) {
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::START), "START");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::BASELINE), "BASELINE");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::CLOSING_COMMAND), "CLOSING_COMMAND");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::CLOSING), "CLOSING");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::CONTACT_CONFIRMED), "CONTACT_CONFIRMED");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::CONTACT), "CONTACT");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::GRASPING), "GRASPING");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::UPLIFT), "UPLIFT");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::EVALUATE), "EVALUATE");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::SUCCESS), "SUCCESS");
  EXPECT_STREQ(KittingStateController::stateToString(GraspState::FAILED), "FAILED");
}

TEST(StateToStringTest, UnknownState) {
  // Cast an out-of-range value to GraspState
  auto unknown = static_cast<GraspState>(999);
  EXPECT_STREQ(KittingStateController::stateToString(unknown), "UNKNOWN");
}

// ============================================================================
// isClosingPhase tests
// ============================================================================

TEST(IsClosingPhaseTest, ClosingStates) {
  EXPECT_TRUE(KittingStateController::isClosingPhase(GraspState::CLOSING_COMMAND));
  EXPECT_TRUE(KittingStateController::isClosingPhase(GraspState::CLOSING));
  EXPECT_TRUE(KittingStateController::isClosingPhase(GraspState::CONTACT_CONFIRMED));
}

TEST(IsClosingPhaseTest, NonClosingStates) {
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::START));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::BASELINE));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::CONTACT));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::GRASPING));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::UPLIFT));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::EVALUATE));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::SUCCESS));
  EXPECT_FALSE(KittingStateController::isClosingPhase(GraspState::FAILED));
}

// ============================================================================
// isForceRampPhase tests
// ============================================================================

TEST(IsForceRampPhaseTest, ForceRampStates) {
  EXPECT_TRUE(KittingStateController::isForceRampPhase(GraspState::GRASPING));
  EXPECT_TRUE(KittingStateController::isForceRampPhase(GraspState::UPLIFT));
  EXPECT_TRUE(KittingStateController::isForceRampPhase(GraspState::EVALUATE));
}

TEST(IsForceRampPhaseTest, NonForceRampStates) {
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::START));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::BASELINE));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::CLOSING_COMMAND));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::CLOSING));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::CONTACT_CONFIRMED));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::CONTACT));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::SUCCESS));
  EXPECT_FALSE(KittingStateController::isForceRampPhase(GraspState::FAILED));
}
