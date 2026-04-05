"""Unit tests for the wrench slope secure grasp detector."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest
import random
from python.wrench_slope import WrenchSlopeDetector, WrenchSlopeResult
from python.config import WrenchSlopeConfig


def _make_detector(**kwargs) -> WrenchSlopeDetector:
    config = WrenchSlopeConfig(**kwargs)
    return WrenchSlopeDetector(config)


def _feed_step(det, step_index, mu, std=0.01, n_samples=100):
    """Feed one step of synthetic wrench_norm samples."""
    det.begin_step(step_index)
    rng = random.Random(42 + step_index)
    for _ in range(n_samples):
        det.update(mu + rng.gauss(0, std))
    return det.finalize_step()


class TestWrenchSlopeDetector(unittest.TestCase):

    def test_no_trigger_on_trending_signal(self):
        """Monotonically drifting wrench_norm means should NOT trigger."""
        det = _make_detector()
        means = [3.0, 2.9, 2.8, 2.7, 2.6, 2.5, 2.4, 2.3]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)

    def test_trigger_on_flat_signal(self):
        """Stable wrench_norm with flat slope should trigger after n_confirm."""
        det = _make_detector(n_confirm=3)
        # Step 0: init
        _feed_step(det, 0, 2.5, std=0.05)
        # Steps 1-3: flat
        _feed_step(det, 1, 2.51, std=0.05)
        _feed_step(det, 2, 2.49, std=0.05)
        r = _feed_step(det, 3, 2.50, std=0.05)
        self.assertTrue(det.secure)
        self.assertTrue(r.secure)

    def test_high_slope_blocks(self):
        """Even if EWMA band passes, high slope should block detection."""
        det = _make_detector(
            ewma_band_width=0.50,  # very wide band — EWMA always passes
            slope_threshold=0.01,  # very tight slope — catches any drift
            n_confirm=2,
        )
        # Gradually increasing signal — each step close to EWMA but trending
        means = [2.0, 2.03, 2.06, 2.09, 2.12, 2.15, 2.18]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.01)
        self.assertFalse(det.secure)

    def test_reset_clears_state(self):
        """reset() should clear all state."""
        det = _make_detector(n_confirm=2)
        _feed_step(det, 0, 2.5)
        _feed_step(det, 1, 2.51)
        _feed_step(det, 2, 2.50)
        self.assertTrue(det.secure)

        det.reset()
        self.assertFalse(det.secure)
        self.assertEqual(det.converge_streak, 0)
        self.assertEqual(det.step_index, 0)

    def test_secure_is_latched(self):
        """Once secure, stays secure even on subsequent finalize calls."""
        det = _make_detector(n_confirm=2)
        _feed_step(det, 0, 2.5)
        _feed_step(det, 1, 2.51)
        _feed_step(det, 2, 2.50)
        self.assertTrue(det.secure)
        # One more finalize — still secure
        r = det.finalize_step()
        self.assertTrue(r.secure)

    def test_step_zero_skips_comparison(self):
        """Step 0 only initializes EWMA, no convergence check."""
        det = _make_detector()
        det.begin_step(0)
        for _ in range(100):
            det.update(2.5)
        r = det.finalize_step()
        self.assertFalse(r.secure)
        self.assertEqual(r.converge_streak, 0)

    def test_zero_samples_returns_safe(self):
        """Finalizing a step with no samples should return not secure."""
        det = _make_detector()
        det.begin_step(0)
        r = det.finalize_step()
        self.assertFalse(r.secure)
        self.assertEqual(r.d_mu, 0.0)
        self.assertEqual(r.slope, 0.0)

    def test_large_oscillation_no_trigger(self):
        """Large oscillating means should not trigger."""
        det = _make_detector()
        means = [2.0, 2.4, 2.0, 2.4, 2.0, 2.4]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)

    def test_monotonically_increasing_no_trigger(self):
        """Monotonically increasing means should not trigger."""
        det = _make_detector()
        means = [2.0, 2.1, 2.2, 2.3, 2.4, 2.5]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)

    def test_result_has_slope_field(self):
        """WrenchSlopeResult should contain slope diagnostic."""
        det = _make_detector()
        _feed_step(det, 0, 2.5)
        r = _feed_step(det, 1, 2.51)
        self.assertIsInstance(r, WrenchSlopeResult)
        self.assertIsInstance(r.slope, float)
        self.assertIsInstance(r.d_mu, float)
        self.assertIsInstance(r.std_late, float)

    def test_slope_computed_after_min_points(self):
        """Slope check should only activate after min_slope_points steps."""
        det = _make_detector(min_slope_points=4, n_confirm=2)
        # Feed 3 flat steps — slope check not active yet (< min_slope_points)
        _feed_step(det, 0, 2.5)
        _feed_step(det, 1, 2.51)
        r = _feed_step(det, 2, 2.50)
        # With min_slope_points=4, slope not yet computed at step 2 (only 3 points)
        # EWMA passes, slope is bypassed → streak increments
        # But n_confirm=2, so it should detect by step 2
        # The point: slope gate is inactive when insufficient points
        self.assertTrue(r.secure)


class TestWrenchSlopeConfig(unittest.TestCase):

    def test_n_confirm_zero_raises(self):
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(n_confirm=0)

    def test_ewma_lambda_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(ewma_lambda=0.0)
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(ewma_lambda=1.5)

    def test_slope_window_too_small_raises(self):
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(slope_window=1)

    def test_slope_threshold_negative_raises(self):
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(slope_threshold=-0.01)

    def test_zero_thresholds_raise(self):
        """Zero-valued thresholds disable detection — must be rejected."""
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(ewma_band_width=0.0)
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(std_threshold=0.0)
        with self.assertRaises(ValueError):
            WrenchSlopeConfig(slope_threshold=0.0)


class TestUsesSlots(unittest.TestCase):

    def test_uses_slots(self):
        det = _make_detector()
        self.assertTrue(hasattr(WrenchSlopeDetector, '__slots__'))


if __name__ == "__main__":
    unittest.main()
