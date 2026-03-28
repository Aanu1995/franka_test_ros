"""Unit tests for the secure grasp convergence detector."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest
import random
from python.secure_grasp import SecureGraspDetector, SecureGraspResult
from python.config import SecureGraspConfig


# ============================================================================
# Real trial data: step-level mean_tau_ext_norm and std_tau_ext_norm
# from all 29 trials in kitting_bags 3.
# ============================================================================
TRIAL_DATA = {
    "triangle1_t1": {"means": [1.0913, 0.9961, 1.2003, 1.2125, 1.1985, 1.2037, 1.1929], "stds": [0.1468, 0.1095, 0.1057, 0.0759, 0.0788, 0.0836, 0.0773]},
    "triangle1_t2": {"means": [1.1201, 1.2745, 1.2851, 1.1025, 1.3052, 1.3136, 1.1106], "stds": [0.1645, 0.0861, 0.0495, 0.0945, 0.0853, 0.0492, 0.1022]},
    "triangle_t1": {"means": [1.1701, 1.0664, 1.2207, 1.2122, 1.2205], "stds": [0.2021, 0.1147, 0.1099, 0.0796, 0.1032]},
    "triangle_t2": {"means": [1.2047, 1.2151, 1.2057, 1.1725], "stds": [0.1979, 0.1301, 0.1117, 0.0914]},
    "triangle_t3": {"means": [1.2299, 1.2966, 1.1569, 1.1764, 1.2417, 1.2888, 1.2783, 1.1260, 1.2991, 1.1310], "stds": [0.1740, 0.0938, 0.1381, 0.1313, 0.1211, 0.0926, 0.0855, 0.1328, 0.1021, 0.1328]},
    "triangle_t4": {"means": [1.1979, 1.2147, 1.1647, 1.2342, 1.2398], "stds": [0.1839, 0.0773, 0.1221, 0.0847, 0.0958]},
    "triangle_t5": {"means": [1.2422, 1.1701, 1.2967, 1.3221, 1.3170], "stds": [0.1866, 0.1468, 0.0801, 0.1032, 0.1004]},
    "triangle_t6": {"means": [1.2489, 1.2563, 1.2571], "stds": [0.1780, 0.1109, 0.0979]},
    "smallCircle_t1": {"means": [1.2589, 1.2618, 1.2000, 1.1801, 1.3069, 1.3036, 1.3122], "stds": [0.1605, 0.0796, 0.0889, 0.0736, 0.0978, 0.0495, 0.0624]},
    "smallCircle_t3": {"means": [1.0898, 1.0851, 1.1802, 1.1756, 1.1350, 1.1657, 1.1738, 1.1675], "stds": [0.1801, 0.1145, 0.1001, 0.0839, 0.1024, 0.0738, 0.0724, 0.0795]},
    "smallCircle_t4": {"means": [1.1795, 1.2870, 1.1884, 1.2698, 1.2782, 1.2835], "stds": [0.1698, 0.1091, 0.0971, 0.0876, 0.0823, 0.0883]},
    "smallCircle_t5": {"means": [1.1965, 1.1530, 1.2770, 1.2954, 1.2874], "stds": [0.1601, 0.1622, 0.0844, 0.0810, 0.0854]},
    "smallCircle_t6": {"means": [1.2244, 1.2954, 1.2893, 1.1610, 1.1933, 1.2804, 1.2878, 1.2846], "stds": [0.1523, 0.0979, 0.0624, 0.1011, 0.1152, 0.0905, 0.0672, 0.0670]},
    "bigCircle_t1": {"means": [1.0596, 1.1558, 1.1609, 1.1461], "stds": [0.1482, 0.0908, 0.0779, 0.0569]},
    "bigCircle_t2": {"means": [1.1937, 1.1250, 1.2554, 1.2276, 1.2640, 1.2116, 1.2391, 1.2538, 1.2481], "stds": [0.1450, 0.0942, 0.0923, 0.0635, 0.0477, 0.0580, 0.0516, 0.0389, 0.0449]},
    "bigCircle_t3": {"means": [1.1989, 1.3068, 1.2916, 1.1836, 1.1429, 1.1436], "stds": [0.1209, 0.1007, 0.0579, 0.0933, 0.0757, 0.1043]},
    "bigCircle_t4": {"means": [1.2398, 1.3180, 1.3293, 1.3100], "stds": [0.1060, 0.0936, 0.0630, 0.0633]},
    "bigCircle_t5": {"means": [1.2321, 1.3233, 1.3119, 1.2965], "stds": [0.1268, 0.0939, 0.0664, 0.0676]},
    "bigCircle_t6": {"means": [1.2462, 1.3215, 1.3263, 1.2224, 1.2589, 1.1646, 1.3225, 1.3169, 1.2534, 1.2205], "stds": [0.0995, 0.0464, 0.0896, 0.0807, 0.0533, 0.0848, 0.0834, 0.0614, 0.0710, 0.0570]},
    "thyme_t1": {"means": [0.7544, 0.7374, 0.6750, 0.6724, 0.6804], "stds": [0.1666, 0.0499, 0.0494, 0.0296, 0.0334]},
    "thyme_t2": {"means": [0.9696, 0.9913, 0.9293, 0.9206, 0.9165], "stds": [0.1741, 0.0421, 0.0566, 0.0452, 0.0396]},
    "irregularShape_t1": {"means": [0.8053, 0.8943, 0.7563, 0.8884, 0.8930, 0.8771], "stds": [0.1485, 0.1162, 0.0982, 0.1049, 0.0658, 0.0759]},
    "irregularShape_t2": {"means": [1.1865, 1.1757, 1.1776], "stds": [0.1326, 0.0990, 0.0958]},
    "irregularShape_t3": {"means": [1.2112, 1.3433, 1.3283, 1.1909, 1.1774, 1.1733, 1.1717, 1.3034, 1.1836, 1.3345, 1.3229, 1.2329, 1.2760, 1.2976], "stds": [0.1609, 0.0702, 0.0586, 0.0777, 0.1127, 0.1220, 0.1196, 0.0838, 0.1066, 0.1007, 0.0749, 0.0788, 0.0883, 0.0794]},
    "irregularShape1_t2": {"means": [1.2749, 1.4028, 1.4023, 1.3917, 1.2601, 1.4134, 1.4078, 1.4104, 1.4133, 1.3949], "stds": [0.1700, 0.1008, 0.1323, 0.1066, 0.1320, 0.1129, 0.1000, 0.0837, 0.0968, 0.0903]},
    "irregularShape1_t3": {"means": [1.3632, 1.4672, 1.3174, 1.4891, 1.4744, 1.4879], "stds": [0.1888, 0.1104, 0.0925, 0.1094, 0.0732, 0.0772]},
    "irregularShape1_t5": {"means": [1.3133, 1.2569, 1.4535, 1.4506, 1.4524], "stds": [0.1292, 0.1445, 0.1055, 0.0646, 0.0692]},
    "irregularShape1_t6": {"means": [1.4003, 1.5261, 1.5303, 1.5356], "stds": [0.1605, 0.0881, 0.0929, 0.0758]},
    "irregularShape1_t7": {"means": [1.3938, 1.4852, 1.5538, 1.5652, 1.5509], "stds": [0.1805, 0.1117, 0.0604, 0.0532, 0.0604]},
}


def _make_detector(mode="ewma", **kwargs) -> SecureGraspDetector:
    config = SecureGraspConfig(mode=mode, **kwargs)
    return SecureGraspDetector(config)


def _feed_step(det, step_index, mu, std=0.01, n_samples=100):
    """Simulate a GRASP step with given mean and noise."""
    det.begin_step(step_index)
    rng = random.Random(42 + step_index)
    for _ in range(n_samples):
        sample = mu + rng.gauss(0, std)
        det.update(sample)
    return det.finalize_step()


def _run_trial_means(det, means, stds):
    """Feed step-level means/stds through a detector.
    Returns (detected, step_detected_1indexed).
    """
    det.reset()
    for i, (mu, std) in enumerate(zip(means, stds)):
        det.begin_step(i)
        # Simulate samples with given mean/std
        rng = random.Random(42 + i)
        for _ in range(125):
            det.update(mu + rng.gauss(0, std))
        r = det.finalize_step()
        if r.secure:
            return True, i + 1
    return False, len(means)


class TestEWMAMode(unittest.TestCase):
    """Tests for EWMA band detection mode."""

    def test_no_trigger_on_diverging_signal(self):
        """Diverging means should never trigger."""
        det = _make_detector(mode="ewma")
        det.reset()
        means = [1.0, 1.2, 1.5, 1.1, 0.8, 1.3]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            result = det.finalize_step()
            self.assertFalse(result.secure)

    def test_trigger_on_converged_signal(self):
        """Converged means should trigger."""
        det = _make_detector(mode="ewma")
        det.reset()
        # Step 0: initial
        _feed_step(det, 0, 1.2, std=0.05)
        # Step 1: close
        r1 = _feed_step(det, 1, 1.21, std=0.05)
        # Step 2: close (streak=2 -> SECURE)
        r2 = _feed_step(det, 2, 1.215, std=0.05)
        self.assertTrue(det.secure)

    def test_high_noise_blocks(self):
        """High within-step std should block detection."""
        det = _make_detector(mode="ewma", std_threshold=0.14)
        det.reset()
        rng = random.Random(99)
        for i in range(5):
            det.begin_step(i)
            for _ in range(100):
                det.update(1.0 + rng.gauss(0, 0.2))
            r = det.finalize_step()
        self.assertFalse(det.secure)

    def test_reset_clears_all_state(self):
        det = _make_detector(mode="ewma", n_confirm=1)
        det.reset()
        _feed_step(det, 0, 1.0)
        _feed_step(det, 1, 1.01)
        self.assertTrue(det.secure)
        det.reset()
        self.assertFalse(det.secure)
        self.assertEqual(det.converge_streak, 0)

    def test_secure_is_latched(self):
        det = _make_detector(mode="ewma", n_confirm=1)
        det.reset()
        _feed_step(det, 0, 1.0)
        _feed_step(det, 1, 1.01)
        self.assertTrue(det.secure)
        r = det.finalize_step()
        self.assertTrue(r.secure)

    def test_step_zero_skips_comparison(self):
        det = _make_detector(mode="ewma")
        det.reset()
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        r = det.finalize_step()
        self.assertFalse(r.secure)

    def test_zero_samples_returns_safe(self):
        det = _make_detector(mode="ewma")
        det.begin_step(0)
        r = det.finalize_step()
        self.assertFalse(r.secure)


class TestSlopeMode(unittest.TestCase):
    """Tests for slope-based plateau detection mode."""

    def test_no_trigger_on_diverging_signal(self):
        det = _make_detector(mode="slope", slope_window_size=3, slope_threshold=0.03)
        det.reset()
        means = [1.0, 1.2, 1.5, 1.8, 2.1]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            r = det.finalize_step()
            self.assertFalse(r.secure)

    def test_trigger_on_flat_signal(self):
        det = _make_detector(mode="slope", slope_window_size=3, slope_threshold=0.03)
        det.reset()
        means = [1.2, 1.21, 1.19]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertTrue(det.secure)

    def test_needs_window_to_fill(self):
        det = _make_detector(mode="slope", slope_window_size=3)
        det.reset()
        _feed_step(det, 0, 1.0)
        self.assertFalse(det.secure)
        _feed_step(det, 1, 1.0)
        self.assertFalse(det.secure)

    def test_high_noise_blocks(self):
        det = _make_detector(mode="slope", std_threshold=0.14)
        det.reset()
        rng = random.Random(99)
        for i in range(5):
            det.begin_step(i)
            for _ in range(100):
                det.update(1.0 + rng.gauss(0, 0.2))
            det.finalize_step()
        self.assertFalse(det.secure)


class TestBothMode(unittest.TestCase):
    """Tests for AND-gated BOTH mode."""

    def test_needs_both_to_agree(self):
        det = _make_detector(mode="both", n_confirm=1, slope_window_size=3)
        det.reset()
        # Flat signal that should trigger both
        _feed_step(det, 0, 1.2, std=0.05)
        _feed_step(det, 1, 1.21, std=0.05)
        r = _feed_step(det, 2, 1.205, std=0.05)
        self.assertTrue(det.secure)

    def test_diverging_blocks_both(self):
        det = _make_detector(mode="both")
        det.reset()
        means = [1.0, 1.2, 1.5, 1.1, 0.8]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            det.finalize_step()
        self.assertFalse(det.secure)


class TestEWMAAllTrials(unittest.TestCase):
    """EWMA mode must detect all 29 trials from kitting_bags 3."""

    def test_ewma_detects_all_29_trials(self):
        detected = 0
        for name, data in TRIAL_DATA.items():
            det = _make_detector(mode="ewma")
            found, step = _run_trial_means(det, data["means"], data["stds"])
            if found:
                detected += 1
            else:
                print(f"  EWMA missed: {name}")
        self.assertEqual(detected, 29, f"EWMA detected {detected}/29 trials")


class TestSlopeAllTrials(unittest.TestCase):
    """Slope mode must detect all 29 trials from kitting_bags 3."""

    def test_slope_detects_all_29_trials(self):
        detected = 0
        for name, data in TRIAL_DATA.items():
            det = _make_detector(mode="slope")
            found, step = _run_trial_means(det, data["means"], data["stds"])
            if found:
                detected += 1
            else:
                print(f"  Slope missed: {name}")
        # Slope may miss highly oscillatory trials (triangle1_t2, bigCircle_t6)
        self.assertGreaterEqual(detected, 27, f"Slope detected {detected}/29 trials")


class TestBothAllTrials(unittest.TestCase):
    """Both mode should detect most trials (may be slightly more conservative)."""

    def test_both_detects_most_trials(self):
        detected = 0
        for name, data in TRIAL_DATA.items():
            det = _make_detector(mode="both")
            found, step = _run_trial_means(det, data["means"], data["stds"])
            if found:
                detected += 1
        # Both mode is AND-gated, may miss a few
        self.assertGreaterEqual(detected, 27, f"Both mode detected {detected}/29")


class TestSafetyPatterns(unittest.TestCase):
    """Ensure dangerous patterns never trigger in any mode."""

    def _assert_no_trigger(self, mode, means, stds=None):
        det = _make_detector(mode=mode)
        det.reset()
        if stds is None:
            stds = [0.05] * len(means)
        for i, (mu, std) in enumerate(zip(means, stds)):
            _feed_step(det, i, mu, std=std)
        self.assertFalse(det.secure, f"{mode} falsely triggered on {means}")

    def test_monotonically_increasing(self):
        means = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
        for mode in ("ewma", "slope", "both"):
            self._assert_no_trigger(mode, means)

    def test_large_oscillation(self):
        means = [1.0, 1.3, 1.0, 1.3, 1.0, 1.3]
        for mode in ("ewma", "slope", "both"):
            self._assert_no_trigger(mode, means)


class TestBothModePartialTrigger(unittest.TestCase):
    """AND-gate: one sub-detector fires but not the other."""

    def test_ewma_fires_slope_does_not(self):
        """EWMA converges but slope sees a trend — both mode should NOT fire."""
        det = _make_detector(mode="both", n_confirm=2, slope_window_size=3, slope_threshold=0.01)
        det.reset()
        # Upward trending means: EWMA adapts (low dev), but slope is positive
        means = [1.0, 1.05, 1.10, 1.15, 1.20]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)

    def test_slope_fires_ewma_does_not(self):
        """Slope is flat but EWMA sees large deviation — both mode should NOT fire."""
        det = _make_detector(mode="both", n_confirm=2,
                             ewma_band_width=0.01)  # very tight band
        det.reset()
        # Flat with some noise: slope triggers but EWMA band too tight
        means = [1.0, 1.05, 1.0, 1.05, 1.0]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)


class TestConfigValidation(unittest.TestCase):
    """Config validation catches invalid parameters."""

    def test_invalid_mode_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(mode="invalid")

    def test_uppercase_mode_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(mode="EWMA")

    def test_slope_window_too_small_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(slope_window_size=1)

    def test_n_confirm_zero_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(n_confirm=0)

    def test_ewma_lambda_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(ewma_lambda=0.0)
        with self.assertRaises(ValueError):
            SecureGraspConfig(ewma_lambda=1.5)


class TestSetConfig(unittest.TestCase):
    """set_config() properly propagates mode changes."""

    def test_mode_change_via_set_config(self):
        det = _make_detector(mode="ewma")
        det.reset()
        _feed_step(det, 0, 1.0)
        _feed_step(det, 1, 1.01)
        _feed_step(det, 2, 1.005)
        self.assertTrue(det.secure)

        # Switch to slope mode and reset
        new_config = SecureGraspConfig(mode="slope")
        det.set_config(new_config)
        self.assertFalse(det.secure)
        self.assertEqual(det.step_index, 0)


class TestUsesSlots(unittest.TestCase):
    def test_uses_slots(self):
        det = _make_detector()
        self.assertTrue(hasattr(SecureGraspDetector, '__slots__'))


if __name__ == "__main__":
    unittest.main()
