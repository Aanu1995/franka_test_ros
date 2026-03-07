"""
Adaptive baseline estimator with lifecycle management for SMS-CUSUM.

Provides a running estimate of signal mean (mu) and standard deviation (sigma)
with three operational phases:

  COLLECTING : Accumulating initial samples for a robust starting estimate.
  TRACKING   : Updating mu via Exponential Moving Average (EMA) during free-motion.
  FROZEN     : No updates; detector is in active detection mode.

The lifecycle supports context inheritance: when a CUSUM stage fires an alarm,
the baseline can be frozen, its (mu, sigma) passed as the reference for the
next stage, and later unfrozen when returning to free motion.

Computational complexity: O(1) per sample, zero dynamic allocation.
"""

from __future__ import annotations

import math
from enum import IntEnum, auto


class BaselinePhase(IntEnum):
    """Operational phase of the adaptive baseline estimator."""
    COLLECTING = auto()
    TRACKING = auto()
    FROZEN = auto()


class AdaptiveBaseline:
    """Adaptive baseline estimator using EMA with lifecycle management.

    During COLLECTING, accumulates `init_samples` observations to compute
    the initial mean and standard deviation.  Once ready, transitions to
    TRACKING and applies EMA updates:

        mu_{n+1} = alpha * x_n + (1 - alpha) * mu_n

    Can be FROZEN to prevent baseline corruption during active detection,
    and UNFROZEN to resume tracking.

    Parameters
    ----------
    init_samples : int
        Number of samples to collect before baseline is valid.
        Default 50 (0.2 s at 250 Hz).
    alpha : float
        EMA smoothing factor. Smaller = smoother, slower adaptation.
        Default 0.01 (time constant ~100 samples = 0.4 s at 250 Hz).
    """

    __slots__ = (
        "_init_samples", "_alpha", "_sum", "_sum_sq", "_count",
        "_mu", "_sigma", "_phase",
    )

    def __init__(self, init_samples: int = 50, alpha: float = 0.01) -> None:
        self._init_samples = init_samples
        self._alpha = alpha
        self._sum: float = 0.0
        self._sum_sq: float = 0.0
        self._count: int = 0
        self._mu: float = 0.0
        self._sigma: float = 0.0
        self._phase: BaselinePhase = BaselinePhase.COLLECTING

    @property
    def ready(self) -> bool:
        """True once initial collection is complete and mu/sigma are valid."""
        return self._phase != BaselinePhase.COLLECTING

    @property
    def mean(self) -> float:
        """Current estimated mean."""
        return self._mu

    @property
    def sigma(self) -> float:
        """Estimated noise standard deviation (from initial collection)."""
        return self._sigma

    @property
    def phase(self) -> BaselinePhase:
        """Current operational phase."""
        return self._phase

    @property
    def count(self) -> int:
        """Number of samples processed (during collection) or total since ready."""
        return self._count

    def freeze(self) -> None:
        """Stop updating baseline (entering detection mode).

        The current (mu, sigma) are preserved for use as CUSUM reference.
        """
        self._phase = BaselinePhase.FROZEN

    def unfreeze(self) -> None:
        """Resume EMA tracking (returning to free-motion).

        The baseline will adapt to the current signal level using EMA.
        """
        if self._phase == BaselinePhase.FROZEN:
            self._phase = BaselinePhase.TRACKING

    def reset(self) -> None:
        """Full reset for a new trial. Returns to COLLECTING phase."""
        self._sum = 0.0
        self._sum_sq = 0.0
        self._count = 0
        self._mu = 0.0
        self._sigma = 0.0
        self._phase = BaselinePhase.COLLECTING

    def update(self, x: float) -> None:
        """Process one sample. O(1) time, zero allocation.

        Behavior depends on phase:
          COLLECTING: accumulates sum/sum_sq, transitions to TRACKING when ready.
          TRACKING:   applies EMA update to mu.
          FROZEN:     no-op.

        Parameters
        ----------
        x : float
            Current observation (e.g., tau_ext_norm).
        """
        if self._phase == BaselinePhase.FROZEN:
            return

        if self._phase == BaselinePhase.COLLECTING:
            self._sum += x
            self._sum_sq += x * x
            self._count += 1
            if self._count >= self._init_samples:
                self._mu = self._sum / self._count
                variance = (self._sum_sq / self._count) - (self._mu * self._mu)
                self._sigma = math.sqrt(max(variance, 0.0))
                self._phase = BaselinePhase.TRACKING
        else:
            # TRACKING: EMA update
            self._mu = self._alpha * x + (1.0 - self._alpha) * self._mu
            self._count += 1

    def snapshot(self) -> tuple[float, float]:
        """Return current (mean, sigma) for context inheritance.

        Use this to pass the baseline statistics to the next CUSUM stage
        when a state transition is detected.

        Returns
        -------
        tuple[float, float]
            (mean, sigma) of the current baseline estimate.
        """
        return (self._mu, self._sigma)
