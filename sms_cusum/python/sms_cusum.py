"""
SMS-CUSUM: Sequential Multi-State CUSUM for Contact Detection.

A novel extension of the classical CUSUM change-point detector that detects
contact state transitions from a continuous force/torque signal stream, with
noise-adaptive sensitivity and lifecycle-managed baseline estimation.

Novel contributions over standard CUSUM (Page, 1954):
    1. Noise-adaptive allowance (k_eff = max(k_min, alpha * sigma))
    2. Lifecycle-managed baseline (collect -> freeze -> detect -> recover)
    3. Automatic sensitivity scaling across objects of varying properties

Fills the gap between standard CUSUM (single change-point, known shift) and
GLR-CUSUM (unknown shift, but O(n) per sample). SMS-CUSUM handles unknown
shift magnitudes at O(1) per sample via noise-adaptive allowance.

Computational complexity: O(1) per sample, zero dynamic allocation.

References:
    Page, E.S. (1954). Biometrika, 41(1-2), 100-115.
    Lorden, G. (1971). Ann. Math. Statist., 42(6), 1897-1908.
    Moustakides, G.V. (1986). Ann. Statist., 14(4), 1379-1387.
    Basseville, M. & Nikiforov, I.V. (1993). Detection of Abrupt Changes. Prentice Hall.
    Veeravalli, V.V. & Banerjee, T. (2014). Academic Press Library in Signal Processing, Vol. 3.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from .adaptive_baseline import AdaptiveBaseline
from .config import SMSCusumConfig
from .cusum import CUSUMDetector


class GraspState(IntEnum):
    """Detected states in the SMS-CUSUM state machine.

    State graph:
      FREE_MOTION -> CLOSING -> CONTACT
    """
    FREE_MOTION = 0
    CLOSING = 1
    CONTACT = 2


_STATE_NAMES = {
    GraspState.FREE_MOTION: "FREE_MOTION",
    GraspState.CLOSING: "CLOSING",
    GraspState.CONTACT: "CONTACT",
}


@dataclass(frozen=True)
class DetectionEvent:
    """Emitted when a state transition is detected.

    Attributes
    ----------
    prev_state : GraspState
        State before the transition.
    new_state : GraspState
        State after the transition.
    sample_index : int
        Sample index (0-based) at which the transition was detected.
    cusum_statistic : float
        CUSUM statistic S_n at the moment of detection.
    baseline_mean : float
        Reference mean used by the CUSUM stage.
    baseline_sigma : float
        Noise sigma of the reference baseline.
    k_effective : float
        Effective allowance parameter used for detection.
    detail : str
        Human-readable description of the transition.
    """
    prev_state: GraspState
    new_state: GraspState
    sample_index: int
    cusum_statistic: float = 0.0
    baseline_mean: float = 0.0
    baseline_sigma: float = 0.0
    k_effective: float = 0.0
    detail: str = ""


class SMSCusum:
    """SMS-CUSUM detector for contact detection from force/torque feedback.

    Orchestrates an adaptive baseline and a CUSUM detector to identify
    contact events (torque drops) during gripper closure.

    Architecture:
      - One AdaptiveBaseline for signal characterization (mean + noise sigma)
      - One CUSUMDetector for contact detection with noise-adaptive allowance

    Usage (real-time, one sample at a time)::

        detector = SMSCusum(SMSCusumConfig())

        # Phase 1: Baseline collection (FREE_MOTION)
        for sample in baseline_data:
            detector.update(sample.tau_ext_norm)

        # Phase 2: Contact detection
        detector.enter_closing()
        for sample in closing_data:
            event = detector.update(sample.tau_ext_norm)
            if event and event.new_state == GraspState.CONTACT:
                print("Contact detected!")

        # Reset for next grasp (soft reset preserves baseline)
        detector.soft_reset()

    Parameters
    ----------
    config : SMSCusumConfig
        Full detector configuration.
    """

    def __init__(self, config: Optional[SMSCusumConfig] = None) -> None:
        self._config = config or SMSCusumConfig()

        self._state = GraspState.FREE_MOTION
        self._sample_idx: int = 0

        # Adaptive baseline for initial signal characterization
        self._baseline = AdaptiveBaseline(
            init_samples=self._config.baseline_init_samples,
            alpha=self._config.baseline_alpha,
        )

        # Contact CUSUM (CLOSING -> CONTACT)
        self._contact_cusum = CUSUMDetector(self._config.contact_stage)

        # Transition history
        self._events: list[DetectionEvent] = []

    @property
    def state(self) -> GraspState:
        """Current detected state."""
        return self._state

    @property
    def state_name(self) -> str:
        """Current state as a string."""
        return _STATE_NAMES[self._state]

    @property
    def sample_index(self) -> int:
        """Number of samples processed so far."""
        return self._sample_idx

    @property
    def baseline_ready(self) -> bool:
        """True once the initial baseline collection is complete."""
        return self._baseline.ready

    @property
    def baseline(self) -> AdaptiveBaseline:
        """Access the underlying baseline estimator."""
        return self._baseline

    @property
    def contact_cusum(self) -> CUSUMDetector:
        """Access the contact CUSUM stage (for diagnostics)."""
        return self._contact_cusum

    @property
    def events(self) -> list[DetectionEvent]:
        """History of all detected state transitions."""
        return self._events

    # ------------------------------------------------------------------
    # State transition API (called by the controlling application)
    # ------------------------------------------------------------------

    def enter_closing(self) -> None:
        """Signal that the gripper is starting to close.

        Freezes the baseline and activates the contact CUSUM stage.
        Call this when the gripper close command is issued.
        """
        self._baseline.freeze()
        self._contact_cusum.reset()
        self._contact_cusum.adapt(self._baseline.sigma)
        self._state = GraspState.CLOSING

    # ------------------------------------------------------------------
    # Core update loop: O(1) per sample
    # ------------------------------------------------------------------

    def update(
        self,
        tau_ext_norm: float,
        **kwargs,
    ) -> Optional[DetectionEvent]:
        """Process one sample. O(1) time, zero dynamic allocation.

        Parameters
        ----------
        tau_ext_norm : float
            External torque norm (primary detection signal).
            Any robot providing this scalar can use SMS-CUSUM.

        Returns
        -------
        DetectionEvent or None
            A DetectionEvent if a state transition was detected, else None.
        """
        self._sample_idx += 1

        if self._state == GraspState.FREE_MOTION:
            return self._update_free_motion(tau_ext_norm)
        elif self._state == GraspState.CLOSING:
            return self._update_closing(tau_ext_norm)
        # CONTACT: terminal state, no further transitions
        return None

    # ------------------------------------------------------------------
    # Per-state update implementations
    # ------------------------------------------------------------------

    def _update_free_motion(self, tau_ext_norm: float) -> None:
        """FREE_MOTION: collect baseline samples and track via EMA."""
        self._baseline.update(tau_ext_norm)
        return None

    def _update_closing(self, tau_ext_norm: float) -> Optional[DetectionEvent]:
        """CLOSING: run contact CUSUM against frozen baseline."""
        if not self._baseline.ready:
            return None

        alarm = self._contact_cusum.update(self._baseline.mean, tau_ext_norm)
        if alarm:
            event = DetectionEvent(
                prev_state=GraspState.CLOSING,
                new_state=GraspState.CONTACT,
                sample_index=self._sample_idx,
                cusum_statistic=self._contact_cusum.statistic,
                baseline_mean=self._baseline.mean,
                baseline_sigma=self._baseline.sigma,
                k_effective=self._contact_cusum.k_effective,
                detail=(
                    f"Contact detected: S={self._contact_cusum.statistic:.3f}, "
                    f"baseline={self._baseline.mean:.4f}, "
                    f"sigma={self._baseline.sigma:.4f}, "
                    f"k_eff={self._contact_cusum.k_effective:.4f}"
                ),
            )
            self._state = GraspState.CONTACT
            self._events.append(event)
            return event
        return None

    # ------------------------------------------------------------------
    # Reset / lifecycle
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Full reset for a new grasp cycle.

        Returns to FREE_MOTION with fresh baseline collection.
        Preserves configuration but clears all runtime state.
        """
        self._state = GraspState.FREE_MOTION
        self._sample_idx = 0
        self._baseline.reset()
        self._contact_cusum.reset()
        self._events.clear()

    def soft_reset(self) -> None:
        """Soft reset: return to FREE_MOTION, unfreeze baseline (keeps mu/sigma).

        Use this between consecutive grasp attempts on the same robot to
        avoid re-collecting the full baseline. The baseline will resume
        EMA tracking from its current estimate.
        """
        self._state = GraspState.FREE_MOTION
        self._baseline.unfreeze()
        self._contact_cusum.reset()
