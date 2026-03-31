"""Unit tests for the secure grasp convergence detector (EWMA band)."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest
import random
from python.secure_grasp import SecureGraspDetector, SecureGraspResult
from python.config import SecureGraspConfig


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


def _make_detector(**kwargs) -> SecureGraspDetector:
    config = SecureGraspConfig(**kwargs)
    return SecureGraspDetector(config)


def _feed_step(det, step_index, mu, std=0.01, n_samples=100):
    det.begin_step(step_index)
    rng = random.Random(42 + step_index)
    for _ in range(n_samples):
        det.update(mu + rng.gauss(0, std))
    return det.finalize_step()


def _run_trial_means(det, means, stds):
    det.reset()
    for i, (mu, std) in enumerate(zip(means, stds)):
        det.begin_step(i)
        rng = random.Random(42 + i)
        for _ in range(125):
            det.update(mu + rng.gauss(0, std))
        r = det.finalize_step()
        if r.secure:
            return True, i + 1
    return False, len(means)


class TestEWMA(unittest.TestCase):

    def test_no_trigger_on_diverging_signal(self):
        det = _make_detector()
        det.reset()
        means = [1.0, 1.2, 1.5, 1.1, 0.8, 1.3]
        for i, mu in enumerate(means):
            det.begin_step(i)
            for _ in range(100):
                det.update(mu)
            self.assertFalse(det.finalize_step().secure)

    def test_trigger_on_converged_signal(self):
        det = _make_detector()
        det.reset()
        _feed_step(det, 0, 1.2, std=0.05)
        _feed_step(det, 1, 1.21, std=0.05)
        _feed_step(det, 2, 1.215, std=0.05)
        self.assertTrue(det.secure)

    def test_high_noise_blocks(self):
        det = _make_detector(std_threshold=0.14)
        det.reset()
        rng = random.Random(99)
        for i in range(5):
            det.begin_step(i)
            for _ in range(100):
                det.update(1.0 + rng.gauss(0, 0.2))
            det.finalize_step()
        self.assertFalse(det.secure)

    def test_reset_clears_all_state(self):
        det = _make_detector(n_confirm=1)
        det.reset()
        _feed_step(det, 0, 1.0)
        _feed_step(det, 1, 1.01)
        self.assertTrue(det.secure)
        det.reset()
        self.assertFalse(det.secure)
        self.assertEqual(det.converge_streak, 0)

    def test_secure_is_latched(self):
        det = _make_detector(n_confirm=1)
        det.reset()
        _feed_step(det, 0, 1.0)
        _feed_step(det, 1, 1.01)
        self.assertTrue(det.secure)
        self.assertTrue(det.finalize_step().secure)

    def test_step_zero_skips_comparison(self):
        det = _make_detector()
        det.reset()
        det.begin_step(0)
        for _ in range(100):
            det.update(1.0)
        r = det.finalize_step()
        self.assertFalse(r.secure)

    def test_zero_samples_returns_safe(self):
        det = _make_detector()
        det.begin_step(0)
        r = det.finalize_step()
        self.assertFalse(r.secure)

    def test_large_oscillation_no_trigger(self):
        det = _make_detector()
        det.reset()
        means = [1.0, 1.3, 1.0, 1.3, 1.0, 1.3]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)

    def test_monotonically_increasing_no_trigger(self):
        det = _make_detector()
        det.reset()
        means = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
        for i, mu in enumerate(means):
            _feed_step(det, i, mu, std=0.05)
        self.assertFalse(det.secure)


class TestAllTrials(unittest.TestCase):

    def test_detects_all_29_trials(self):
        detected = 0
        for name, data in TRIAL_DATA.items():
            det = _make_detector()
            found, step = _run_trial_means(det, data["means"], data["stds"])
            if found:
                detected += 1
        self.assertEqual(detected, 29, f"Detected {detected}/29 trials")


class TestConfigValidation(unittest.TestCase):

    def test_n_confirm_zero_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(n_confirm=0)

    def test_ewma_lambda_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            SecureGraspConfig(ewma_lambda=0.0)
        with self.assertRaises(ValueError):
            SecureGraspConfig(ewma_lambda=1.5)


class TestUsesSlots(unittest.TestCase):

    def test_uses_slots(self):
        det = _make_detector()
        self.assertTrue(hasattr(SecureGraspDetector, '__slots__'))


if __name__ == "__main__":
    unittest.main()
