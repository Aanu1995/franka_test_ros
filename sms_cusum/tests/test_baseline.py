"""Unit tests for the AdaptiveBaseline estimator."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math
import unittest
from python.adaptive_baseline import AdaptiveBaseline, BaselinePhase


class TestAdaptiveBaseline(unittest.TestCase):
    """Tests for AdaptiveBaseline with lifecycle management."""

    def test_initial_phase_is_collecting(self):
        """Baseline starts in COLLECTING phase."""
        bl = AdaptiveBaseline(init_samples=10)
        self.assertEqual(bl.phase, BaselinePhase.COLLECTING)
        self.assertFalse(bl.ready)

    def test_transitions_to_tracking_after_init_samples(self):
        """After init_samples, baseline should be ready and in TRACKING."""
        bl = AdaptiveBaseline(init_samples=10)
        for i in range(10):
            bl.update(1.5)
        self.assertTrue(bl.ready)
        self.assertEqual(bl.phase, BaselinePhase.TRACKING)

    def test_mean_accuracy(self):
        """Mean should be accurate after initial collection."""
        bl = AdaptiveBaseline(init_samples=100)
        for _ in range(100):
            bl.update(2.0)
        self.assertAlmostEqual(bl.mean, 2.0, places=5)

    def test_sigma_accuracy(self):
        """Sigma should reflect the actual noise standard deviation."""
        import random
        random.seed(42)
        bl = AdaptiveBaseline(init_samples=1000)
        mu = 1.8
        sigma = 0.05
        for _ in range(1000):
            bl.update(mu + random.gauss(0, sigma))
        self.assertAlmostEqual(bl.mean, mu, places=1)
        self.assertAlmostEqual(bl.sigma, sigma, delta=0.01)

    def test_freeze_stops_updates(self):
        """Frozen baseline should not change when updated."""
        bl = AdaptiveBaseline(init_samples=10)
        for _ in range(10):
            bl.update(1.0)
        bl.freeze()
        self.assertEqual(bl.phase, BaselinePhase.FROZEN)

        mu_frozen = bl.mean
        for _ in range(100):
            bl.update(5.0)  # Very different value
        self.assertAlmostEqual(bl.mean, mu_frozen)

    def test_unfreeze_resumes_tracking(self):
        """Unfreezing should allow EMA updates to resume."""
        bl = AdaptiveBaseline(init_samples=10, alpha=0.5)
        for _ in range(10):
            bl.update(1.0)
        bl.freeze()
        bl.unfreeze()
        self.assertEqual(bl.phase, BaselinePhase.TRACKING)

        # EMA with alpha=0.5 should converge quickly to new value
        for _ in range(50):
            bl.update(2.0)
        self.assertAlmostEqual(bl.mean, 2.0, places=2)

    def test_reset_returns_to_collecting(self):
        """Reset should return to COLLECTING with zeroed state."""
        bl = AdaptiveBaseline(init_samples=10)
        for _ in range(10):
            bl.update(1.5)
        self.assertTrue(bl.ready)

        bl.reset()
        self.assertFalse(bl.ready)
        self.assertEqual(bl.phase, BaselinePhase.COLLECTING)
        self.assertEqual(bl.count, 0)

    def test_snapshot_returns_mean_sigma(self):
        """snapshot() should return current (mean, sigma)."""
        bl = AdaptiveBaseline(init_samples=10)
        for _ in range(10):
            bl.update(3.0)
        mu, sigma = bl.snapshot()
        self.assertAlmostEqual(mu, 3.0)
        self.assertAlmostEqual(sigma, 0.0)

    def test_ema_tracking_converges(self):
        """EMA should converge to new signal level after unfreeze."""
        bl = AdaptiveBaseline(init_samples=10, alpha=0.1)
        for _ in range(10):
            bl.update(1.0)
        self.assertAlmostEqual(bl.mean, 1.0)

        # Now feed samples at 2.0, EMA should converge
        for _ in range(100):
            bl.update(2.0)
        self.assertAlmostEqual(bl.mean, 2.0, places=1)

    def test_not_ready_during_collection(self):
        """Should not be ready until init_samples are collected."""
        bl = AdaptiveBaseline(init_samples=50)
        for i in range(49):
            bl.update(1.0)
            self.assertFalse(bl.ready)
        bl.update(1.0)
        self.assertTrue(bl.ready)

    def test_unfreeze_from_collecting_stays_collecting(self):
        """Unfreezing from COLLECTING should not change phase."""
        bl = AdaptiveBaseline(init_samples=10)
        bl.unfreeze()  # Should be no-op since not frozen
        self.assertEqual(bl.phase, BaselinePhase.COLLECTING)


if __name__ == "__main__":
    unittest.main()
