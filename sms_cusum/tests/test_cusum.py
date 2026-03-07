"""Unit tests for the CUSUM detector."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest
from python.cusum import CUSUMDetector
from python.config import CusumStageConfig


class TestCUSUMDetector(unittest.TestCase):
    """Tests for CUSUMDetector with noise-adaptive allowance."""

    def test_no_alarm_on_baseline(self):
        """CUSUM should not alarm when signal matches baseline."""
        config = CusumStageConfig(k_min=0.05, h=0.8, debounce_count=5)
        cusum = CUSUMDetector(config)
        mu_0 = 1.8

        for _ in range(1000):
            result = cusum.update(mu_0, mu_0)
            self.assertFalse(result)
        self.assertAlmostEqual(cusum.statistic, 0.0)

    def test_alarm_on_step_drop(self):
        """CUSUM should alarm on a sustained step-down shift."""
        config = CusumStageConfig(k_min=0.05, h=0.8, debounce_count=5)
        cusum = CUSUMDetector(config)
        mu_0 = 1.8
        shift = 0.3  # Large drop

        alarm_fired = False
        for i in range(100):
            result = cusum.update(mu_0, mu_0 - shift)
            if result:
                alarm_fired = True
                break

        self.assertTrue(alarm_fired)
        self.assertGreaterEqual(cusum.statistic, config.h)

    def test_debounce_prevents_early_alarm(self):
        """Alarm should not fire before debounce_count consecutive samples."""
        config = CusumStageConfig(k_min=0.01, h=0.1, debounce_count=10)
        cusum = CUSUMDetector(config)
        mu_0 = 1.8
        shift = 0.5

        alarm_sample = None
        for i in range(100):
            result = cusum.update(mu_0, mu_0 - shift)
            if result:
                alarm_sample = i + 1
                break

        self.assertIsNotNone(alarm_sample)
        self.assertGreaterEqual(alarm_sample, config.debounce_count)

    def test_transient_spike_no_alarm(self):
        """A brief spike followed by return to baseline should not alarm."""
        config = CusumStageConfig(k_min=0.05, h=1.0, debounce_count=10)
        cusum = CUSUMDetector(config)
        mu_0 = 1.8

        # 3 samples of drop then back to normal
        for _ in range(3):
            cusum.update(mu_0, mu_0 - 0.5)
        for _ in range(100):
            result = cusum.update(mu_0, mu_0)
            self.assertFalse(result)

    def test_reset_clears_state(self):
        """Reset should clear accumulated CUSUM statistic."""
        config = CusumStageConfig(k_min=0.05, h=0.8, debounce_count=5)
        cusum = CUSUMDetector(config)
        mu_0 = 1.8

        for _ in range(10):
            cusum.update(mu_0, mu_0 - 0.5)

        self.assertGreater(cusum.statistic, 0)
        cusum.reset()
        self.assertEqual(cusum.statistic, 0.0)
        self.assertEqual(cusum.alarm_streak, 0)

    def test_noise_adaptive_allowance(self):
        """adapt() should scale k_effective based on sigma."""
        config = CusumStageConfig(k_min=0.05, h=0.8, debounce_count=5, noise_multiplier=3.0)
        cusum = CUSUMDetector(config)

        # Low noise: k_min dominates
        cusum.adapt(0.01)
        self.assertAlmostEqual(cusum.k_effective, 0.05)  # max(0.05, 3*0.01=0.03)

        # High noise: multiplier dominates
        cusum.adapt(0.05)
        self.assertAlmostEqual(cusum.k_effective, 0.15)  # max(0.05, 3*0.05=0.15)

    def test_small_shift_with_adapted_k(self):
        """Small shift should still be detected with low noise (small k_eff)."""
        config = CusumStageConfig(k_min=0.02, h=0.3, debounce_count=5, noise_multiplier=2.0)
        cusum = CUSUMDetector(config)
        cusum.adapt(0.005)  # Very low noise -> k_eff = max(0.02, 0.01) = 0.02
        mu_0 = 1.5
        shift = 0.08  # Small but real shift

        alarm_fired = False
        for i in range(200):
            result = cusum.update(mu_0, mu_0 - shift)
            if result:
                alarm_fired = True
                break
        self.assertTrue(alarm_fired)

    def test_large_noise_prevents_false_alarm(self):
        """High noise adaptation should prevent false alarms on noisy signal."""
        config = CusumStageConfig(k_min=0.05, h=1.0, debounce_count=10, noise_multiplier=3.0)
        cusum = CUSUMDetector(config)
        cusum.adapt(0.06)  # k_eff = max(0.05, 0.18) = 0.18

        import random
        random.seed(42)
        mu_0 = 1.8

        # Feed noise-only signal (std=0.06, same as adapted sigma)
        alarm_count = 0
        for _ in range(5000):
            x = mu_0 + random.gauss(0, 0.06)
            if cusum.update(mu_0, x):
                alarm_count += 1

        self.assertEqual(alarm_count, 0, "False alarm on noise-only signal")


class TestCUSUMDetectorEdgeCases(unittest.TestCase):
    """Edge case tests."""

    def test_exact_threshold(self):
        """Statistic exactly at h should start debounce counting."""
        config = CusumStageConfig(k_min=0.0, h=1.0, debounce_count=3)
        cusum = CUSUMDetector(config)
        # Each sample contributes (1.0 - 0.5 - 0.0) = 0.5 to S
        # Sample 1: S = 0.5 < h=1.0 -> alarm_streak = 0
        # Sample 2: S = 1.0 >= h=1.0 -> alarm_streak = 1
        cusum.update(1.0, 0.5)
        self.assertAlmostEqual(cusum.statistic, 0.5)
        self.assertEqual(cusum.alarm_streak, 0)
        cusum.update(1.0, 0.5)
        self.assertAlmostEqual(cusum.statistic, 1.0)
        self.assertEqual(cusum.alarm_streak, 1)

    def test_upward_shift_no_alarm(self):
        """Upward shift (signal above baseline) should never alarm."""
        config = CusumStageConfig(k_min=0.01, h=0.1, debounce_count=3)
        cusum = CUSUMDetector(config)
        mu_0 = 1.0

        for _ in range(1000):
            result = cusum.update(mu_0, mu_0 + 0.5)
            self.assertFalse(result)
        self.assertAlmostEqual(cusum.statistic, 0.0)


if __name__ == "__main__":
    unittest.main()
