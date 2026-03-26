"""
Secure grasp convergence detector for force ramp termination.

Detects when increasing grip force no longer changes the steady-state
tau_ext_norm, indicating the object is fully compressed and the grasp
is secure. Enables early termination of the force ramp, avoiding
unnecessary force on the object.

Detection criteria (AND-gated):
    1. Mean convergence: |μ_late[N] - μ_late[N-1]| < threshold
    2. Signal stability: σ_late[N] < threshold
    3. Consecutive confirmation: criteria must hold for n_confirm steps

The detector operates across force ramp steps (GRASP_1..GRASP_N), not
within a single step. Each step's HOLDING phase is split into an early
transient segment (ignored) and a late settled segment (accumulated).

Computational complexity: O(1) per sample, zero dynamic allocation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .config import SecureGraspConfig


@dataclass(frozen=True)
class SecureGraspResult:
    """Result from SecureGraspDetector.finalize_step().

    Attributes
    ----------
    secure : bool
        True if secure grasp detected (n_confirm consecutive converged steps).
    d_mu : float
        |μ_late[N] - μ_late[N-1]| for the current step.
        0.0 if no previous step to compare (step 0).
    std_late : float
        Standard deviation of tau_ext_norm in the late segment.
    converge_streak : int
        Current number of consecutive converged steps.
    """

    secure: bool
    d_mu: float
    std_late: float
    converge_streak: int


class SecureGraspDetector:
    """Detects secure grasp by tracking tau_ext_norm convergence across
    consecutive force ramp steps.

    The detector compares the settled (late-segment) mean of tau_ext_norm
    between consecutive GRASP steps. When the difference becomes
    negligibly small for n_confirm consecutive steps, the grasp is
    declared secure.

    Usage (called by the SMS-CUSUM state machine)::

        detector = SecureGraspDetector(SecureGraspConfig())

        # At force ramp start
        detector.reset()
        detector.begin_step(0)

        # During HOLDING late segment of each step
        for sample in late_holding_samples:
            detector.update(sample.tau_ext_norm)

        # At STEP_COMPLETE
        result = detector.finalize_step()
        if result.secure:
            print("Secure grasp detected!")

        # Next step
        detector.begin_step(1)
        ...

    Parameters
    ----------
    config : SecureGraspConfig
        Convergence detection configuration.
    """

    __slots__ = (
        "_config",
        "_step_index",
        "_sum",
        "_sum_sq",
        "_count",
        "_prev_mu_late",
        "_converge_streak",
        "_secure",
    )

    def __init__(self, config: Optional[SecureGraspConfig] = None) -> None:
        self._config = config or SecureGraspConfig()
        self._step_index: int = 0
        self._sum: float = 0.0
        self._sum_sq: float = 0.0
        self._count: int = 0
        self._prev_mu_late: float = 0.0
        self._converge_streak: int = 0
        self._secure: bool = False

    def reset(self) -> None:
        """Full reset for a new grasp cycle."""
        self._step_index = 0
        self._sum = 0.0
        self._sum_sq = 0.0
        self._count = 0
        self._prev_mu_late = 0.0
        self._converge_streak = 0
        self._secure = False

    def begin_step(self, step_index: int) -> None:
        """Signal start of a new GRASP step's HOLDING phase.

        Resets the running accumulators for the new step.

        Parameters
        ----------
        step_index : int
            Zero-based step index (0 = GRASP_1, 1 = GRASP_2, ...).
        """
        self._step_index = step_index
        self._sum = 0.0
        self._sum_sq = 0.0
        self._count = 0

    def update(self, tau_ext_norm: float) -> None:
        """Accumulate one sample during the late segment of HOLDING.

        O(1) time, zero allocation. Call this for each sample during
        the late (settled) portion of the HOLDING phase.

        Parameters
        ----------
        tau_ext_norm : float
            External torque norm sample.
        """
        self._sum += tau_ext_norm
        self._sum_sq += tau_ext_norm * tau_ext_norm
        self._count += 1

    def finalize_step(self) -> SecureGraspResult:
        """Finalize the current step and check for secure grasp.

        Computes the settled mean and std from accumulated samples,
        compares against the previous step's mean, and updates the
        convergence streak.

        Returns
        -------
        SecureGraspResult
            Detection result with diagnostics.
        """
        if self._secure:
            return SecureGraspResult(
                secure=True,
                d_mu=0.0,
                std_late=0.0,
                converge_streak=self._converge_streak,
            )

        if self._count == 0:
            return SecureGraspResult(
                secure=False,
                d_mu=0.0,
                std_late=0.0,
                converge_streak=self._converge_streak,
            )

        mu_late = self._sum / self._count
        variance = self._sum_sq / self._count - mu_late * mu_late
        std_late = math.sqrt(max(0.0, variance))

        d_mu = 0.0
        if self._step_index > 0:  # step 0 has no previous mu to compare
            d_mu = abs(mu_late - self._prev_mu_late)

            primary_ok = d_mu < self._config.mean_converge_threshold
            secondary_ok = std_late < self._config.std_threshold

            if primary_ok and secondary_ok:
                self._converge_streak += 1
            else:
                self._converge_streak = 0

            if self._converge_streak >= self._config.n_confirm:
                self._secure = True

        self._prev_mu_late = mu_late

        return SecureGraspResult(
            secure=self._secure,
            d_mu=d_mu,
            std_late=std_late,
            converge_streak=self._converge_streak,
        )

    @property
    def secure(self) -> bool:
        """True if secure grasp has been detected (latched)."""
        return self._secure

    @property
    def step_index(self) -> int:
        """Current step index."""
        return self._step_index

    @property
    def converge_streak(self) -> int:
        """Current consecutive converged step count."""
        return self._converge_streak

    @property
    def config(self) -> SecureGraspConfig:
        """Detector configuration."""
        return self._config
