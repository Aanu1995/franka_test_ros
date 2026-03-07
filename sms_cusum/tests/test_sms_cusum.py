"""Integration tests for the SMS-CUSUM state machine."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import random
import unittest
from python.sms_cusum import SMSCusum, GraspState
from python.config import SMSCusumConfig, CusumStageConfig


class TestSMSCusumContactDetection(unittest.TestCase):
    """Tests for contact detection: FREE_MOTION -> CLOSING -> CONTACT."""

    def _make_detector(self) -> SMSCusum:
        config = SMSCusumConfig(
            contact_stage=CusumStageConfig(k_min=0.05, h=0.8, debounce_count=5),
            baseline_init_samples=20,
            baseline_alpha=0.01,
        )
        return SMSCusum(config)

    def test_initial_state_is_free_motion(self):
        """Detector starts in FREE_MOTION."""
        det = self._make_detector()
        self.assertEqual(det.state, GraspState.FREE_MOTION)
        self.assertEqual(det.state_name, "FREE_MOTION")

    def test_baseline_collection(self):
        """Baseline becomes ready after init_samples."""
        det = self._make_detector()
        for _ in range(20):
            det.update(1.8)
        self.assertTrue(det.baseline_ready)
        self.assertAlmostEqual(det.baseline.mean, 1.8)

    def test_enter_closing_changes_state(self):
        """enter_closing() transitions to CLOSING."""
        det = self._make_detector()
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()
        self.assertEqual(det.state, GraspState.CLOSING)

    def test_contact_detected_on_drop(self):
        """CLOSING -> CONTACT on sustained tau_ext_norm drop."""
        det = self._make_detector()
        # Baseline
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        # Feed closing signal with drop
        event = None
        for i in range(100):
            e = det.update(1.8 - 0.3)  # 0.3 Nm drop
            if e is not None:
                event = e
                break

        self.assertIsNotNone(event, "Contact should be detected")
        self.assertEqual(event.new_state, GraspState.CONTACT)
        self.assertEqual(det.state, GraspState.CONTACT)

    def test_no_contact_on_stable_signal(self):
        """CLOSING should not detect contact when signal matches baseline."""
        det = self._make_detector()
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        for _ in range(500):
            e = det.update(1.8)
            self.assertIsNone(e)
        self.assertEqual(det.state, GraspState.CLOSING)

    def test_contact_event_has_diagnostics(self):
        """DetectionEvent should contain useful diagnostic info."""
        config = SMSCusumConfig(
            contact_stage=CusumStageConfig(k_min=0.03, h=0.5, debounce_count=5),
            baseline_init_samples=20,
        )
        det = SMSCusum(config)

        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        event = None
        for _ in range(100):
            e = det.update(1.5)  # 0.3 Nm drop
            if e:
                event = e
                break

        self.assertIsNotNone(event)
        self.assertEqual(event.prev_state, GraspState.CLOSING)
        self.assertEqual(event.new_state, GraspState.CONTACT)
        self.assertGreater(event.cusum_statistic, 0)
        self.assertAlmostEqual(event.baseline_mean, 1.8, places=1)
        self.assertGreater(event.baseline_sigma, 0)  # sigma is 0 for constant input
        self.assertGreater(event.k_effective, 0)
        self.assertIn("Contact detected", event.detail)
        self.assertEqual(len(det.events), 1)

    def test_contact_stays_in_contact(self):
        """Once CONTACT is detected, further updates remain in CONTACT."""
        det = self._make_detector()
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        # Trigger contact
        for _ in range(100):
            e = det.update(1.5)
            if e:
                break

        self.assertEqual(det.state, GraspState.CONTACT)
        # Further updates should return None (terminal state)
        for _ in range(100):
            e = det.update(1.5)
            self.assertIsNone(e)
        self.assertEqual(det.state, GraspState.CONTACT)


class TestSMSCusumNoContact(unittest.TestCase):
    """Tests for empty/no-contact scenarios (false positive prevention)."""

    def test_no_false_alarm_on_noise(self):
        """Noisy signal without real contact should not trigger alarm."""
        config = SMSCusumConfig(
            contact_stage=CusumStageConfig(k_min=0.05, h=0.8, debounce_count=10,
                                           noise_multiplier=3.0),
            baseline_init_samples=50,
        )
        det = SMSCusum(config)
        random.seed(42)
        mu = 1.5

        # Baseline
        for _ in range(50):
            det.update(mu + random.gauss(0, 0.05))

        det.enter_closing()

        # 2000 samples of noise-only (8 seconds at 250 Hz)
        false_alarms = 0
        for _ in range(2000):
            e = det.update(mu + random.gauss(0, 0.05))
            if e is not None:
                false_alarms += 1

        self.assertEqual(false_alarms, 0, "False alarm on noise-only signal")

    def test_transient_spike_no_alarm(self):
        """A brief drop followed by return to baseline should not trigger."""
        config = SMSCusumConfig(
            contact_stage=CusumStageConfig(k_min=0.05, h=1.0, debounce_count=10),
            baseline_init_samples=20,
        )
        det = SMSCusum(config)
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        # 3 samples of drop then back to normal
        for _ in range(3):
            det.update(1.3)
        for _ in range(200):
            e = det.update(1.8)
            self.assertIsNone(e)


class TestSMSCusumReset(unittest.TestCase):
    """Tests for reset and soft_reset."""

    def test_reset_returns_to_free_motion(self):
        """Full reset returns to FREE_MOTION with fresh baseline."""
        det = SMSCusum()
        for _ in range(50):
            det.update(1.8)
        det.enter_closing()
        self.assertEqual(det.state, GraspState.CLOSING)

        det.reset()
        self.assertEqual(det.state, GraspState.FREE_MOTION)
        self.assertFalse(det.baseline_ready)
        self.assertEqual(det.sample_index, 0)
        self.assertEqual(len(det.events), 0)

    def test_soft_reset_preserves_baseline(self):
        """Soft reset keeps baseline estimate, returns to FREE_MOTION."""
        det = SMSCusum(SMSCusumConfig(baseline_init_samples=20))
        for _ in range(20):
            det.update(1.8)
        det.enter_closing()

        det.soft_reset()
        self.assertEqual(det.state, GraspState.FREE_MOTION)
        self.assertTrue(det.baseline_ready)
        self.assertAlmostEqual(det.baseline.mean, 1.8, places=1)

    def test_consecutive_detections_after_soft_reset(self):
        """Soft reset enables consecutive contact detection cycles."""
        config = SMSCusumConfig(
            contact_stage=CusumStageConfig(k_min=0.03, h=0.5, debounce_count=5),
            baseline_init_samples=20,
        )
        det = SMSCusum(config)
        for _ in range(20):
            det.update(1.8)

        # First detection
        det.enter_closing()
        for _ in range(100):
            e = det.update(1.5)
            if e:
                break
        self.assertEqual(det.state, GraspState.CONTACT)

        # Soft reset and detect again
        det.soft_reset()
        self.assertEqual(det.state, GraspState.FREE_MOTION)
        # Let baseline track for a bit
        for _ in range(20):
            det.update(1.8)

        det.enter_closing()
        event2 = None
        for _ in range(100):
            e = det.update(1.5)
            if e:
                event2 = e
                break
        self.assertIsNotNone(event2, "Second contact should be detected after soft reset")


if __name__ == "__main__":
    unittest.main()
