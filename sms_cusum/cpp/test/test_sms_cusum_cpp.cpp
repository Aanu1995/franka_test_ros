// Copyright (c) 2026
// Author: Aanu Olakunle
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// C++ unit tests for the SMS-CUSUM header-only library.
// Tests: AdaptiveBaseline, CUSUMDetector, SecureGraspDetector, SMSCusum state machine.

#include <cmath>
#include <cstdlib>
#include <random>

#include <gtest/gtest.h>

#include <sms_cusum/adaptive_baseline.hpp>
#include <sms_cusum/cusum.hpp>
#include <sms_cusum/secure_grasp.hpp>
#include <sms_cusum/sms_cusum.hpp>

// ============================================================================
// AdaptiveBaseline tests
// ============================================================================

TEST(AdaptiveBaseline, StartsInCollecting) {
  sms_cusum::AdaptiveBaseline bl(50, 0.01);
  EXPECT_EQ(bl.phase(), sms_cusum::BaselinePhase::COLLECTING);
  EXPECT_FALSE(bl.ready());
  EXPECT_EQ(bl.count(), 0);
}

TEST(AdaptiveBaseline, TransitionsToTrackingAfterInitSamples) {
  sms_cusum::AdaptiveBaseline bl(10, 0.01);
  for (int i = 0; i < 10; ++i) {
    bl.update(2.0);
  }
  EXPECT_EQ(bl.phase(), sms_cusum::BaselinePhase::TRACKING);
  EXPECT_TRUE(bl.ready());
  EXPECT_NEAR(bl.mean(), 2.0, 1e-9);
  EXPECT_NEAR(bl.sigma(), 0.0, 1e-9);  // Constant signal → zero variance
}

TEST(AdaptiveBaseline, MeanAndSigmaAccuracy) {
  // Feed known values: 1.0, 2.0, 3.0, 4.0, 5.0
  sms_cusum::AdaptiveBaseline bl(5, 0.01);
  for (double v : {1.0, 2.0, 3.0, 4.0, 5.0}) {
    bl.update(v);
  }
  EXPECT_NEAR(bl.mean(), 3.0, 1e-9);
  // sigma = sqrt(variance) where variance = E[x^2] - E[x]^2 = 11 - 9 = 2
  EXPECT_NEAR(bl.sigma(), std::sqrt(2.0), 1e-9);
}

TEST(AdaptiveBaseline, FreezeStopsUpdates) {
  sms_cusum::AdaptiveBaseline bl(5, 0.5);
  for (int i = 0; i < 5; ++i) bl.update(1.0);
  EXPECT_TRUE(bl.ready());
  double mu_before = bl.mean();
  bl.freeze();
  EXPECT_EQ(bl.phase(), sms_cusum::BaselinePhase::FROZEN);

  // Feed completely different values — should be ignored
  for (int i = 0; i < 100; ++i) bl.update(99.0);
  EXPECT_DOUBLE_EQ(bl.mean(), mu_before);
}

TEST(AdaptiveBaseline, UnfreezeResumesTracking) {
  sms_cusum::AdaptiveBaseline bl(5, 0.5);
  for (int i = 0; i < 5; ++i) bl.update(1.0);
  bl.freeze();
  bl.unfreeze();
  EXPECT_EQ(bl.phase(), sms_cusum::BaselinePhase::TRACKING);

  // EMA with alpha=0.5 should converge to 5.0
  for (int i = 0; i < 50; ++i) bl.update(5.0);
  EXPECT_NEAR(bl.mean(), 5.0, 0.01);
}

TEST(AdaptiveBaseline, ResetReturnsToCollecting) {
  sms_cusum::AdaptiveBaseline bl(5, 0.01);
  for (int i = 0; i < 5; ++i) bl.update(2.0);
  EXPECT_TRUE(bl.ready());
  bl.reset();
  EXPECT_EQ(bl.phase(), sms_cusum::BaselinePhase::COLLECTING);
  EXPECT_FALSE(bl.ready());
  EXPECT_EQ(bl.count(), 0);
  EXPECT_DOUBLE_EQ(bl.mean(), 0.0);
}

TEST(AdaptiveBaseline, EMATracksStepChange) {
  sms_cusum::AdaptiveBaseline bl(5, 0.1);
  for (int i = 0; i < 5; ++i) bl.update(1.0);
  EXPECT_NEAR(bl.mean(), 1.0, 1e-9);

  // Step change to 3.0 with alpha=0.1 — should converge
  for (int i = 0; i < 200; ++i) bl.update(3.0);
  EXPECT_NEAR(bl.mean(), 3.0, 0.01);
}

TEST(AdaptiveBaseline, ConfigClamping_InitSamples) {
  // init_samples < 2 should clamp to 2
  sms_cusum::AdaptiveBaseline bl(0, 0.01);
  bl.update(1.0);
  EXPECT_FALSE(bl.ready());  // Need at least 2
  bl.update(1.0);
  EXPECT_TRUE(bl.ready());   // Now at 2 (clamped minimum)
}

TEST(AdaptiveBaseline, ConfigClamping_Alpha) {
  // alpha <= 0 should clamp to 0.001
  sms_cusum::AdaptiveBaseline bl(2, -1.0);
  bl.update(1.0);
  bl.update(1.0);
  EXPECT_TRUE(bl.ready());
  // Should not crash; alpha clamped internally
}

// ============================================================================
// CUSUMDetector tests
// ============================================================================

TEST(CUSUMDetector, NoAlarmOnStableSignal) {
  sms_cusum::CusumStageConfig cfg{0.02, 0.3, 5, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  det.adapt(0.05);  // k_eff = max(0.02, 2.0*0.05) = 0.10

  // 500 samples at baseline — no alarm
  for (int i = 0; i < 500; ++i) {
    EXPECT_FALSE(det.update(1.8, 1.8));
  }
  EXPECT_NEAR(det.statistic(), 0.0, 1e-9);
}

TEST(CUSUMDetector, AlarmOnSustainedDrop) {
  sms_cusum::CusumStageConfig cfg{0.02, 0.3, 5, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  det.adapt(0.05);  // k_eff = 0.10

  double mu_0 = 1.8;
  double dropped = 1.4;  // Drop of 0.4 Nm, contribution per sample = 0.4 - 0.1 = 0.3

  bool alarmed = false;
  for (int i = 0; i < 20; ++i) {
    if (det.update(mu_0, dropped)) {
      alarmed = true;
      break;
    }
  }
  EXPECT_TRUE(alarmed);
  EXPECT_GE(det.statistic(), cfg.h);
}

TEST(CUSUMDetector, TransientDoesNotTrigger) {
  sms_cusum::CusumStageConfig cfg{0.02, 0.3, 5, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  det.adapt(0.05);

  double mu_0 = 1.8;

  // 3 samples of drop — builds some S, but streak won't reach debounce
  for (int i = 0; i < 3; ++i) {
    det.update(mu_0, 1.4);
  }

  // Return to baseline — S should decay back
  for (int i = 0; i < 200; ++i) {
    EXPECT_FALSE(det.update(mu_0, 1.8));
  }
}

TEST(CUSUMDetector, NoiseAdaptiveKEff) {
  sms_cusum::CusumStageConfig cfg{0.05, 0.3, 5, 3.0};
  sms_cusum::CUSUMDetector det(cfg);

  // Low noise
  det.adapt(0.01);  // k_eff = max(0.05, 3*0.01) = 0.05
  EXPECT_NEAR(det.k_effective(), 0.05, 1e-9);

  // High noise
  det.adapt(0.05);  // k_eff = max(0.05, 3*0.05) = 0.15
  EXPECT_NEAR(det.k_effective(), 0.15, 1e-9);
}

TEST(CUSUMDetector, ResetClearsState) {
  sms_cusum::CusumStageConfig cfg{0.02, 0.3, 5, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  det.adapt(0.05);

  // Build up some S
  for (int i = 0; i < 5; ++i) {
    det.update(1.8, 1.4);
  }
  EXPECT_GT(det.statistic(), 0.0);

  det.reset();
  EXPECT_DOUBLE_EQ(det.statistic(), 0.0);
  EXPECT_EQ(det.alarm_streak(), 0);
}

TEST(CUSUMDetector, ConfigClamping) {
  // h <= 0 should be clamped to 0.01
  sms_cusum::CusumStageConfig cfg{0.02, -1.0, 0, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  EXPECT_GT(det.config().h, 0.0);
  EXPECT_GE(det.config().debounce_count, 1);
}

TEST(CUSUMDetector, UpwardShiftNoAlarm) {
  sms_cusum::CusumStageConfig cfg{0.02, 0.3, 5, 2.0};
  sms_cusum::CUSUMDetector det(cfg);
  det.adapt(0.05);

  // Signal above baseline — should never alarm (downward detector only)
  for (int i = 0; i < 1000; ++i) {
    EXPECT_FALSE(det.update(1.8, 2.5));
  }
}

// ============================================================================
// SecureGraspDetector tests
// ============================================================================

// Helper: feed a step's worth of data to the detector
static void feedStep(sms_cusum::SecureGraspDetector& det, int step_index,
                     double mu, double noise_std, int n_samples) {
  det.begin_step(step_index);
  std::mt19937 rng(42 + step_index);
  std::normal_distribution<double> dist(mu, noise_std);
  for (int i = 0; i < n_samples; ++i) {
    det.update(dist(rng));
  }
}

TEST(SecureGrasp, DivergingSignalNeverSecure) {
  sms_cusum::SecureGraspConfig cfg{0.4, 0.12, 2, 0.14};
  sms_cusum::SecureGraspDetector det(cfg);

  double means[] = {1.0, 1.3, 1.6, 1.1, 0.8, 1.4};
  for (int i = 0; i < 6; ++i) {
    feedStep(det, i, means[i], 0.05, 50);
    auto r = det.finalize_step();
    EXPECT_FALSE(r.secure) << "Step " << i << " should not be secure";
  }
}

TEST(SecureGrasp, ConvergingSignalTriggersSecure) {
  sms_cusum::SecureGraspConfig cfg{0.4, 0.12, 2, 0.14};
  sms_cusum::SecureGraspDetector det(cfg);

  // Step 0: initialize EWMA
  feedStep(det, 0, 1.2, 0.03, 50);
  auto r0 = det.finalize_step();
  EXPECT_FALSE(r0.secure);

  // Step 1: very close to EWMA — streak=1
  feedStep(det, 1, 1.21, 0.03, 50);
  auto r1 = det.finalize_step();
  EXPECT_FALSE(r1.secure);  // Need n_confirm=2

  // Step 2: still close — streak=2 → secure
  feedStep(det, 2, 1.215, 0.03, 50);
  auto r2 = det.finalize_step();
  EXPECT_TRUE(r2.secure);
}

TEST(SecureGrasp, HighNoiseBlocksConvergence) {
  sms_cusum::SecureGraspConfig cfg{0.4, 0.12, 2, 0.14};
  sms_cusum::SecureGraspDetector det(cfg);

  // Converging means but std > std_threshold
  feedStep(det, 0, 1.2, 0.2, 50);
  det.finalize_step();
  feedStep(det, 1, 1.21, 0.2, 50);
  auto r1 = det.finalize_step();
  EXPECT_FALSE(r1.secure);  // std too high
  feedStep(det, 2, 1.215, 0.2, 50);
  auto r2 = det.finalize_step();
  EXPECT_FALSE(r2.secure);
}

TEST(SecureGrasp, ResetClearsState) {
  sms_cusum::SecureGraspConfig cfg{0.4, 0.12, 2, 0.14};
  sms_cusum::SecureGraspDetector det(cfg);

  feedStep(det, 0, 1.2, 0.03, 50);
  det.finalize_step();
  feedStep(det, 1, 1.21, 0.03, 50);
  det.finalize_step();
  feedStep(det, 2, 1.215, 0.03, 50);
  auto r = det.finalize_step();
  EXPECT_TRUE(r.secure);

  det.reset();
  EXPECT_FALSE(det.secure());
  EXPECT_EQ(det.converge_streak(), 0);
}

TEST(SecureGrasp, Step0InitializesWithoutTesting) {
  sms_cusum::SecureGraspConfig cfg{0.4, 0.12, 1, 0.14};  // n_confirm=1
  sms_cusum::SecureGraspDetector det(cfg);

  // Step 0 should initialize EWMA but NOT test convergence
  feedStep(det, 0, 1.2, 0.03, 50);
  auto r = det.finalize_step();
  EXPECT_FALSE(r.secure);  // Step 0 always false
}

// ============================================================================
// SMSCusum integration tests
// ============================================================================

TEST(SMSCusum, InitialStateIsFreeMotion) {
  sms_cusum::SMSCusum det(sms_cusum::SMSCusumConfig{});
  EXPECT_EQ(det.state(), sms_cusum::GraspState::FREE_MOTION);
  EXPECT_FALSE(det.baseline_ready());
  EXPECT_EQ(det.sample_index(), 0);
}

TEST(SMSCusum, BaselineCollectionAndTransition) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) {
    det.update(1.8);
  }
  EXPECT_TRUE(det.baseline_ready());
  EXPECT_NEAR(det.baseline().mean(), 1.8, 1e-9);
}

TEST(SMSCusum, EnterClosingChangesState) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();
  EXPECT_EQ(det.state(), sms_cusum::GraspState::CLOSING);
  EXPECT_EQ(det.baseline().phase(), sms_cusum::BaselinePhase::FROZEN);
}

TEST(SMSCusum, ContactDetectedOnDrop) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  cfg.contact_stage = {0.02, 0.3, 3, 2.0};
  sms_cusum::SMSCusum det(cfg);

  // Baseline
  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  // Sustained drop
  bool detected = false;
  for (int i = 0; i < 50; ++i) {
    auto r = det.update(1.3);
    if (r.detected && r.event.new_state == sms_cusum::GraspState::CONTACT) {
      detected = true;
      break;
    }
  }
  EXPECT_TRUE(detected);
  EXPECT_EQ(det.state(), sms_cusum::GraspState::CONTACT);
}

TEST(SMSCusum, NoFalseAlarmOnStableSignal) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  // 1000 samples at exact baseline — no detection
  for (int i = 0; i < 1000; ++i) {
    auto r = det.update(1.8);
    EXPECT_FALSE(r.detected) << "False alarm at sample " << i;
  }
  EXPECT_EQ(det.state(), sms_cusum::GraspState::CLOSING);
}

TEST(SMSCusum, NoFalseAlarmOnNoise) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 50;
  sms_cusum::SMSCusum det(cfg);

  std::mt19937 rng(123);
  std::normal_distribution<double> noise(1.8, 0.05);

  // Baseline with noise
  for (int i = 0; i < 50; ++i) det.update(noise(rng));
  det.enter_closing();

  // 1000 noisy samples around baseline — no false alarm
  for (int i = 0; i < 1000; ++i) {
    auto r = det.update(noise(rng));
    EXPECT_FALSE(r.detected) << "False alarm at sample " << i;
  }
}

TEST(SMSCusum, TransientDoesNotTriggerContact) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  cfg.contact_stage = {0.02, 0.3, 5, 2.0};
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  // 3-sample transient then back to baseline
  for (int i = 0; i < 3; ++i) det.update(1.3);
  for (int i = 0; i < 200; ++i) {
    auto r = det.update(1.8);
    EXPECT_FALSE(r.detected);
  }
}

TEST(SMSCusum, ResetReturnsFreeMotion) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();
  EXPECT_EQ(det.state(), sms_cusum::GraspState::CLOSING);

  det.reset();
  EXPECT_EQ(det.state(), sms_cusum::GraspState::FREE_MOTION);
  EXPECT_FALSE(det.baseline_ready());
  EXPECT_EQ(det.sample_index(), 0);
  EXPECT_EQ(det.event_count(), 0);
}

TEST(SMSCusum, SoftResetPreservesBaseline) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  det.soft_reset();
  EXPECT_EQ(det.state(), sms_cusum::GraspState::FREE_MOTION);
  EXPECT_TRUE(det.baseline_ready());
  EXPECT_NEAR(det.baseline().mean(), 1.8, 0.01);
}

TEST(SMSCusum, FullLifecycleToSecureGrasp) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  cfg.contact_stage = {0.02, 0.3, 3, 2.0};
  cfg.secure_grasp_stage = {0.4, 0.12, 2, 0.14};
  sms_cusum::SMSCusum det(cfg);

  // Baseline
  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  // Contact
  for (int i = 0; i < 50; ++i) {
    auto r = det.update(1.3);
    if (r.detected) break;
  }
  EXPECT_EQ(det.state(), sms_cusum::GraspState::CONTACT);

  // Grasping
  det.enter_grasping();
  EXPECT_EQ(det.state(), sms_cusum::GraspState::GRASPING);

  std::mt19937 rng(42);

  // Step 0: seed EWMA
  det.begin_grasp_step(0);
  {
    std::normal_distribution<double> d(1.2, 0.03);
    for (int i = 0; i < 50; ++i) det.update(d(rng));
  }
  auto r0 = det.finalize_grasp_step();
  EXPECT_FALSE(r0.detected);

  // Step 1: close to EWMA
  det.begin_grasp_step(1);
  {
    std::normal_distribution<double> d(1.21, 0.03);
    for (int i = 0; i < 50; ++i) det.update(d(rng));
  }
  auto r1 = det.finalize_grasp_step();
  EXPECT_FALSE(r1.detected);

  // Step 2: still close → secure
  det.begin_grasp_step(2);
  {
    std::normal_distribution<double> d(1.215, 0.03);
    for (int i = 0; i < 50; ++i) det.update(d(rng));
  }
  auto r2 = det.finalize_grasp_step();
  EXPECT_TRUE(r2.detected);
  EXPECT_EQ(det.state(), sms_cusum::GraspState::SECURE_GRASP);
}

TEST(SMSCusum, EventDiagnosticsPopulated) {
  sms_cusum::SMSCusumConfig cfg;
  cfg.baseline_init_samples = 10;
  cfg.contact_stage = {0.02, 0.3, 3, 2.0};
  sms_cusum::SMSCusum det(cfg);

  for (int i = 0; i < 10; ++i) det.update(1.8);
  det.enter_closing();

  sms_cusum::DetectionEvent evt{};
  for (int i = 0; i < 50; ++i) {
    auto r = det.update(1.3);
    if (r.detected) {
      evt = r.event;
      break;
    }
  }
  EXPECT_EQ(evt.prev_state, sms_cusum::GraspState::CLOSING);
  EXPECT_EQ(evt.new_state, sms_cusum::GraspState::CONTACT);
  EXPECT_GT(evt.cusum_statistic, 0.0);
  EXPECT_NEAR(evt.baseline_mean, 1.8, 0.01);
  EXPECT_GT(evt.k_effective, 0.0);
  EXPECT_GT(evt.sample_index, 0);
}
