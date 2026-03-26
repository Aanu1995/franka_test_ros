"""Unit tests for the secure grasp convergence detector."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest
from python.secure_grasp import SecureGraspDetector, SecureGraspResult
from python.config import SecureGraspConfig


class TestSecureGraspDetector(unittest.TestCase):
    """Tests for SecureGraspDetector."""

    def _make_detector(self, **kwargs) -> SecureGraspDetector:
        config = SecureGraspConfig(**kwargs)
        return SecureGraspDetector(config)

    def _feed_step(self, det, step_index, mu, std=0.01, n_samples=100):
        """Simulate a GRASP step with given mean and noise."""
        import random
        det.begin_step(step_index)
        rng = random.Random(42 + step_index)
        for _ in range(n_samples):
            sample = mu + rng.gauss(0, std)
            det.update(sample)
        return det.finalize_step()

    def test_no_trigger_on_diverging_signal(self):
        """Diverging means across steps should never trigger secure grasp."""
        det = self._make_detector()
        det.reset()

        means = [1.0, 1.2, 1.5, 1.1, 0.8, 1.3]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            result = det.finalize_step()
            self.assertFalse(result.secure)

    def test_trigger_on_converged_signal(self):
        """Converged means should trigger after n_confirm consecutive steps."""
        det = self._make_detector(
            mean_converge_threshold=0.03,
            std_threshold=0.08,
            min_grasp_steps=1,
            n_confirm=2,
        )
        det.reset()

        # Step 0: initial (no comparison possible)
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        r0 = det.finalize_step()
        self.assertFalse(r0.secure)

        # Step 1: large shift (not converged)
        det.begin_step(1)
        for _ in range(100):
            det.update(1.2)
        r1 = det.finalize_step()
        self.assertFalse(r1.secure)
        self.assertGreater(r1.d_mu, 0.03)

        # Step 2: small shift from step 1 (streak=1)
        det.begin_step(2)
        for _ in range(100):
            det.update(1.21)
        r2 = det.finalize_step()
        self.assertFalse(r2.secure)
        self.assertEqual(r2.converge_streak, 1)

        # Step 3: small shift from step 2 (streak=2 -> SECURE)
        det.begin_step(3)
        for _ in range(100):
            det.update(1.215)
        r3 = det.finalize_step()
        self.assertTrue(r3.secure)
        self.assertEqual(r3.converge_streak, 2)

    def test_min_grasp_steps_guard(self):
        """Detection should not trigger before min_grasp_steps."""
        det = self._make_detector(min_grasp_steps=1, n_confirm=2)
        det.reset()

        # Step 0: no previous mean, so comparison skipped
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        r = det.finalize_step()
        self.assertFalse(r.secure)
        self.assertEqual(r.converge_streak, 0)
        self.assertAlmostEqual(r.d_mu, 0.0)

    def test_streak_resets_on_divergence(self):
        """Convergence streak should reset when a step diverges."""
        det = self._make_detector(n_confirm=2)
        det.reset()

        # Step 0
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        det.finalize_step()

        # Step 1: converge (streak=1)
        det.begin_step(1)
        for _ in range(100):
            det.update(1.01)
        r1 = det.finalize_step()
        self.assertEqual(r1.converge_streak, 1)

        # Step 2: diverge (streak resets to 0)
        det.begin_step(2)
        for _ in range(100):
            det.update(1.3)
        r2 = det.finalize_step()
        self.assertEqual(r2.converge_streak, 0)

        # Step 3: converge (streak=1)
        det.begin_step(3)
        for _ in range(100):
            det.update(1.31)
        r3 = det.finalize_step()
        self.assertEqual(r3.converge_streak, 1)
        self.assertFalse(r3.secure)

        # Step 4: converge (streak=2 -> SECURE)
        det.begin_step(4)
        for _ in range(100):
            det.update(1.315)
        r4 = det.finalize_step()
        self.assertTrue(r4.secure)

    def test_std_gate_blocks_noisy_signal(self):
        """High std should block detection even with converged mean."""
        det = self._make_detector(
            mean_converge_threshold=0.03,
            std_threshold=0.08,
            n_confirm=2,
        )
        det.reset()

        import random
        rng = random.Random(99)

        # Step 0
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0 + rng.gauss(0, 0.2))
        det.finalize_step()

        # Steps 1-4: same mean but very high std
        for i in range(1, 5):
            det.begin_step(i)
            for _ in range(100):
                det.update(1.0 + rng.gauss(0, 0.2))
            r = det.finalize_step()
            # std_late should be ~0.2, well above threshold 0.08
            self.assertGreater(r.std_late, 0.08)
            self.assertFalse(r.secure)

    def test_oscillatory_pattern_no_false_trigger(self):
        """Alternating means (like triangle1_t2) should not trigger."""
        det = self._make_detector(
            mean_converge_threshold=0.03,
            std_threshold=0.08,
            n_confirm=2,
        )
        det.reset()

        # Oscillating means: 1.07, 1.28, 1.28, 1.07, 1.31, 1.30, 1.07
        means = [1.07, 1.28, 1.28, 1.07, 1.31, 1.30, 1.07]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            r = det.finalize_step()

        # Should never trigger — alternating shifts > 0.03
        self.assertFalse(det.secure)

    def test_reset_clears_all_state(self):
        """Reset should clear all accumulated state."""
        det = self._make_detector(n_confirm=2)

        # Build up some state
        det.begin_step(0)
        for _ in range(50):
            det.update(1.0)
        det.finalize_step()

        det.begin_step(1)
        for _ in range(50):
            det.update(1.01)
        det.finalize_step()

        self.assertEqual(det.converge_streak, 1)

        # Reset
        det.reset()

        self.assertEqual(det.step_index, 0)
        self.assertEqual(det.converge_streak, 0)
        self.assertFalse(det.secure)

    def test_zero_samples_returns_safe_result(self):
        """finalize_step with no samples should return safe defaults."""
        det = self._make_detector()
        det.begin_step(0)
        # No update() calls
        r = det.finalize_step()
        self.assertFalse(r.secure)
        self.assertEqual(r.d_mu, 0.0)
        self.assertEqual(r.std_late, 0.0)

    def test_secure_is_latched(self):
        """Once secure, subsequent finalize_step should still return secure."""
        det = self._make_detector(n_confirm=1, min_grasp_steps=1)
        det.reset()

        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        det.finalize_step()

        det.begin_step(1)
        for _ in range(100):
            det.update(1.01)
        r1 = det.finalize_step()
        self.assertTrue(r1.secure)

        # Even without new data, should stay secure
        r2 = det.finalize_step()
        self.assertTrue(r2.secure)

    def test_thyme_like_convergence(self):
        """Simulate thyme-like profile from real data."""
        det = self._make_detector(
            mean_converge_threshold=0.03,
            std_threshold=0.08,
            min_grasp_steps=1,
            n_confirm=2,
        )
        det.reset()

        # From thyme_t2 actual data (mu_late values)
        mu_values = [0.9253, 0.9907, 0.9052, 0.9076, 0.9049]
        std_values = [0.0230, 0.0361, 0.0261, 0.0250, 0.0071]

        import random

        for i, (mu, std) in enumerate(zip(mu_values, std_values)):
            det.begin_step(i)
            rng = random.Random(i)
            for _ in range(125):
                det.update(mu + rng.gauss(0, std))
            r = det.finalize_step()

        # Should trigger at step 4 (d_mu between steps 3 and 4 is small)
        self.assertTrue(det.secure)

    def test_uses_slots(self):
        """Detector should use __slots__ for O(1) guarantees."""
        det = self._make_detector()
        self.assertTrue(hasattr(SecureGraspDetector, '__slots__'))


if __name__ == "__main__":
    unittest.main()
