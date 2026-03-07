"""
CUSUM (Cumulative Sum) change-point detector with noise-adaptive allowance.

Implements the one-sided downward CUSUM for detecting negative mean shifts,
as required for contact detection (torque drop) in robotic grasping.

The classical CUSUM statistic (Page, 1954):

    S_n = max(0, S_{n-1} + (mu_0 - x_n) - k)

fires an alarm when S_n >= h. This implementation adds:
  - Debounce: alarm requires S_n >= h for `debounce_count` consecutive samples.
  - Noise-adaptive allowance: k_eff = max(k_min, noise_multiplier * sigma).

Computational complexity: O(1) per sample, zero dynamic allocation.

References:
    Page, E.S. (1954). "Continuous inspection schemes."
        Biometrika, 41(1-2), 100-115. DOI: 10.1093/biomet/41.1-2.100
    Moustakides, G.V. (1986). "Optimal stopping times for detecting
        changes in distributions." Ann. Statist., 14(4), 1379-1387.
        DOI: 10.1214/aos/1176350164
"""

from __future__ import annotations

from .config import CusumStageConfig


class CUSUMDetector:
    """One-sided downward CUSUM detector with noise-adaptive allowance.

    Detects sustained negative mean shifts (drops) in a signal relative
    to a provided reference mean mu_0. The CUSUM statistic accumulates
    evidence of a downward shift:

        S_n = max(0, S_{n-1} + (mu_0 - x_n) - k_eff)

    An alarm fires when S_n >= h for at least `debounce_count` consecutive
    samples, providing robustness against transient noise spikes.

    The noise-adaptive allowance k_eff = max(k_min, noise_multiplier * sigma)
    automatically scales sensitivity to the measured noise level, enabling
    detection across objects of varying properties without manual tuning.

    Parameters
    ----------
    config : CusumStageConfig
        Stage configuration with k_min, h, debounce_count, noise_multiplier.
    """

    __slots__ = ("_config", "_S", "_alarm_streak", "_k_eff")

    def __init__(self, config: CusumStageConfig) -> None:
        self._config = config
        self._S: float = 0.0
        self._alarm_streak: int = 0
        self._k_eff: float = config.k_min

    def reset(self) -> None:
        """Reset CUSUM state for a new detection cycle."""
        self._S = 0.0
        self._alarm_streak = 0

    def adapt(self, sigma: float) -> None:
        """Update the effective allowance based on measured noise sigma.

        Called once when entering a detection stage, using the noise
        estimate from the preceding baseline/state.

        Parameters
        ----------
        sigma : float
            Estimated standard deviation of the signal noise.
        """
        self._k_eff = max(self._config.k_min, self._config.noise_multiplier * sigma)

    def update(self, mu_0: float, x: float) -> bool:
        """Process one sample. O(1) time, zero allocation.

        Parameters
        ----------
        mu_0 : float
            Reference mean (baseline level from preceding state).
        x : float
            Current observation (e.g., tau_ext_norm).

        Returns
        -------
        bool
            True if change-point detected (alarm), False otherwise.
        """
        self._S = max(0.0, self._S + (mu_0 - x) - self._k_eff)

        if self._S >= self._config.h:
            self._alarm_streak += 1
            return self._alarm_streak >= self._config.debounce_count
        else:
            self._alarm_streak = 0
            return False

    @property
    def statistic(self) -> float:
        """Current CUSUM statistic S_n (for diagnostics/plotting)."""
        return self._S

    @property
    def alarm_streak(self) -> int:
        """Current consecutive alarm count."""
        return self._alarm_streak

    @property
    def k_effective(self) -> float:
        """Current effective allowance parameter."""
        return self._k_eff

    @property
    def config(self) -> CusumStageConfig:
        """Stage configuration."""
        return self._config
