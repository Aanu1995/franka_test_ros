"""
Secure grasp convergence detector for force ramp termination.

Detects when increasing grip force no longer changes the steady-state
tau_ext_norm, indicating the object is fully compressed and the grasp
is secure. Uses EWMA (Exponentially Weighted Moving Average) band
detection to track convergence across force ramp steps.

The detector operates across force ramp steps (GRASP_1..GRASP_N), not
within a single step. Each step's HOLDING phase is split into an early
transient segment (ignored) and a late settled segment (accumulated).

Computational complexity: O(1) per sample, O(1) per step finalization.
Zero dynamic allocation.
"""

from __future__ import annotations

import copy
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
        True if secure grasp detected.
    d_mu : float
        |mu_late - ewma| for the current step.
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
    """Detects secure grasp via EWMA band detection on tau_ext_norm.

    Tracks an exponentially weighted moving average of step-level settled
    means. Fires when new means stay within +/-band_width of the EWMA
    for n_confirm consecutive steps.

    Parameters
    ----------
    config : SecureGraspConfig
        Detection configuration.
    """

    __slots__ = (
        "_config",
        "_step_index",
        "_sum",
        "_sum_sq",
        "_count",
        "_ewma",
        "_converge_streak",
        "_secure",
    )

    def __init__(self, config: Optional[SecureGraspConfig] = None) -> None:
        self._config = config or SecureGraspConfig()
        self._step_index: int = 0
        self._sum: float = 0.0
        self._sum_sq: float = 0.0
        self._count: int = 0
        self._ewma: float = 0.0
        self._converge_streak: int = 0
        self._secure: bool = False

    def set_config(self, config: SecureGraspConfig) -> None:
        """Replace configuration and reset all state."""
        self._config = copy.copy(config)
        self.reset()

    def reset(self) -> None:
        """Full reset for a new grasp cycle."""
        self._step_index = 0
        self._sum = 0.0
        self._sum_sq = 0.0
        self._count = 0
        self._ewma = 0.0
        self._converge_streak = 0
        self._secure = False

    def begin_step(self, step_index: int) -> None:
        """Signal start of a new GRASP step's HOLDING phase.

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

        Returns
        -------
        SecureGraspResult
            Detection result with diagnostics.
        """
        if self._secure:
            return SecureGraspResult(
                secure=True, d_mu=0.0, std_late=0.0,
                converge_streak=self._converge_streak,
            )

        if self._count == 0:
            return SecureGraspResult(
                secure=False, d_mu=0.0, std_late=0.0,
                converge_streak=self._converge_streak,
            )

        mu_late = self._sum / self._count
        variance = self._sum_sq / self._count - mu_late * mu_late
        std_late = math.sqrt(max(0.0, variance))

        d_mu = 0.0
        if self._step_index == 0:
            self._ewma = mu_late
        else:
            lam = self._config.ewma_lambda
            self._ewma = lam * mu_late + (1.0 - lam) * self._ewma
            d_mu = abs(mu_late - self._ewma)

            if d_mu < self._config.ewma_band_width and std_late < self._config.std_threshold:
                self._converge_streak += 1
            else:
                self._converge_streak = 0

            if self._converge_streak >= self._config.n_confirm:
                self._secure = True

        return SecureGraspResult(
            secure=self._secure, d_mu=d_mu, std_late=std_late,
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
