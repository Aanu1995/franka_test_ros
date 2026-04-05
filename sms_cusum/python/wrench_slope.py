"""
Wrench slope secure grasp detector for force ramp termination.

Detects when wrench_norm step means have converged using two criteria:
  1. EWMA band: |mu_late - EWMA| < band_width  (local convergence)
  2. Slope check: |slope of last W means| < threshold  (no trend/drift)

Both criteria must pass for n_confirm consecutive steps before declaring
secure grasp. This eliminates false positives on heavy objects where the
signal drifts gradually but appears locally converged.

The detector operates across force ramp steps (GRASP_1..GRASP_N), not
within a single step. Each step's HOLDING phase is split into an early
transient segment (ignored) and a late settled segment (accumulated).

Computational complexity: O(1) per sample, O(W) per step finalization
where W is the slope window size (default 5). Zero dynamic allocation
in the C++ mirror (uses fixed-size buffer).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

from .config import WrenchSlopeConfig


@dataclass(frozen=True)
class WrenchSlopeResult:
    """Result from WrenchSlopeDetector.finalize_step().

    Attributes
    ----------
    secure : bool
        True if secure grasp detected.
    d_mu : float
        |mu_late - ewma| for the current step.
    slope : float
        Linear regression slope of recent step means.
    std_late : float
        Standard deviation of wrench_norm in the late segment.
    converge_streak : int
        Current number of consecutive converged steps.
    """

    secure: bool
    d_mu: float
    slope: float
    std_late: float
    converge_streak: int


def _online_slope(values: list[float]) -> float:
    """Compute slope via simple linear regression on (0,1,...,n-1) vs values.

    Returns 0.0 if fewer than 2 values.
    """
    n = len(values)
    if n < 2:
        return 0.0
    x_mean = (n - 1) / 2.0
    y_mean = sum(values) / n
    num = sum((i - x_mean) * (v - y_mean) for i, v in enumerate(values))
    den = sum((i - x_mean) ** 2 for i in range(n))
    return num / den if den > 0.0 else 0.0


class WrenchSlopeDetector:
    """Detects secure grasp via EWMA band + slope check on wrench_norm.

    Tracks an EWMA of step-level settled means and a sliding window
    of recent means for slope computation. Fires when both the EWMA
    band and slope checks pass for n_confirm consecutive steps.

    Parameters
    ----------
    config : WrenchSlopeConfig
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
        "_step_means",
    )

    def __init__(self, config: Optional[WrenchSlopeConfig] = None) -> None:
        self._config = config or WrenchSlopeConfig()
        self._step_index: int = 0
        self._sum: float = 0.0
        self._sum_sq: float = 0.0
        self._count: int = 0
        self._ewma: float = 0.0
        self._converge_streak: int = 0
        self._secure: bool = False
        self._step_means: deque = deque(maxlen=self._config.slope_window)

    def set_config(self, config: WrenchSlopeConfig) -> None:
        """Replace configuration and reset all state."""
        self._config = config
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
        self._step_means = deque(maxlen=self._config.slope_window)

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

    def update(self, wrench_norm: float) -> None:
        """Accumulate one sample during the late segment of HOLDING.

        Parameters
        ----------
        wrench_norm : float
            Wrench norm sample.
        """
        self._sum += wrench_norm
        self._sum_sq += wrench_norm * wrench_norm
        self._count += 1

    def finalize_step(self) -> WrenchSlopeResult:
        """Finalize the current step and check for secure grasp.

        Returns
        -------
        WrenchSlopeResult
            Detection result with diagnostics.
        """
        if self._secure:
            return WrenchSlopeResult(
                secure=True, d_mu=0.0, slope=0.0, std_late=0.0,
                converge_streak=self._converge_streak,
            )

        if self._count == 0:
            return WrenchSlopeResult(
                secure=False, d_mu=0.0, slope=0.0, std_late=0.0,
                converge_streak=self._converge_streak,
            )

        mu_late = self._sum / self._count
        variance = self._sum_sq / self._count - mu_late * mu_late
        std_late = math.sqrt(max(0.0, variance))

        d_mu = 0.0
        slope = 0.0

        if self._step_index == 0:
            # First step: seed EWMA, no comparison
            self._ewma = mu_late
            self._step_means.append(mu_late)
        else:
            d_mu = abs(mu_late - self._ewma)
            self._ewma = (self._config.ewma_lambda * mu_late
                          + (1.0 - self._config.ewma_lambda) * self._ewma)

            # Append to step means buffer (deque auto-trims to slope_window)
            self._step_means.append(mu_late)

            # EWMA band check
            ewma_ok = (d_mu < self._config.ewma_band_width
                       and std_late < self._config.std_threshold)

            # Slope check on last W means (only if enough points)
            if len(self._step_means) >= self._config.min_slope_points:
                slope = _online_slope(list(self._step_means))
                slope_ok = abs(slope) < self._config.slope_threshold
            else:
                slope_ok = True  # not enough data — bypass slope gate

            if ewma_ok and slope_ok:
                self._converge_streak += 1
            else:
                self._converge_streak = 0

            if self._converge_streak >= self._config.n_confirm:
                self._secure = True

        return WrenchSlopeResult(
            secure=self._secure, d_mu=d_mu, slope=slope,
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
    def config(self) -> WrenchSlopeConfig:
        """Detector configuration."""
        return self._config
