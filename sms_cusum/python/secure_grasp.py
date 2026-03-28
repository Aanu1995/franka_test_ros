"""
Secure grasp convergence detector for force ramp termination.

Detects when increasing grip force no longer changes the steady-state
tau_ext_norm, indicating the object is fully compressed and the grasp
is secure. Enables early termination of the force ramp, avoiding
unnecessary force on the object.

Three detection modes are available:

- **EWMA band**: Tracks an exponentially weighted moving average of
  step means. Fires when new means stay within +/-band of the EWMA
  for n_confirm consecutive steps.
- **Slope**: Fits linear regression to the last W step means. Fires
  when |slope| < threshold, indicating a flat plateau.
- **Both**: AND-gated combination -- both EWMA and Slope must
  independently declare secure before the detector fires.

The detector operates across force ramp steps (GRASP_1..GRASP_N), not
within a single step. Each step's HOLDING phase is split into an early
transient segment (ignored) and a late settled segment (accumulated).

Computational complexity: O(1) per sample, O(W) per step finalization
(W = slope_window_size, typically 3). Zero dynamic allocation.
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
        Diagnostic value: |mu_late - ewma| for EWMA mode,
        |slope| for Slope mode, max of both for Both mode.
    std_late : float
        Standard deviation of tau_ext_norm in the late segment.
    converge_streak : int
        Current number of consecutive converged steps (EWMA mode).
    """

    secure: bool
    d_mu: float
    std_late: float
    converge_streak: int


class SecureGraspDetector:
    """Detects secure grasp by tracking tau_ext_norm convergence across
    consecutive force ramp steps.

    Supports three detection modes (configured via SecureGraspConfig.mode):

    - ``"ewma"``: EWMA band detection with confirmation streak.
    - ``"slope"``: Slope-based plateau detection on a sliding window.
    - ``"both"``: AND-gated -- both must agree.

    Usage (called by the SMS-CUSUM state machine)::

        detector = SecureGraspDetector(SecureGraspConfig(mode="ewma"))

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
        # EWMA state
        "_ewma",
        "_ewma_streak",
        "_ewma_secure",
        # Slope state
        "_mu_buffer",
        "_buf_head",
        "_buf_count",
        "_slope_secure",
        # Combined
        "_secure",
    )

    def __init__(self, config: Optional[SecureGraspConfig] = None) -> None:
        self._config = config or SecureGraspConfig()
        self._step_index: int = 0
        self._sum: float = 0.0
        self._sum_sq: float = 0.0
        self._count: int = 0
        # EWMA state
        self._ewma: float = 0.0
        self._ewma_streak: int = 0
        self._ewma_secure: bool = False
        # Slope state
        W = self._config.slope_window_size
        self._mu_buffer: list[float] = [0.0] * W
        self._buf_head: int = 0
        self._buf_count: int = 0
        self._slope_secure: bool = False
        # Combined
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
        self._ewma_streak = 0
        self._ewma_secure = False
        W = self._config.slope_window_size
        self._mu_buffer = [0.0] * W
        self._buf_head = 0
        self._buf_count = 0
        self._slope_secure = False
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

        O(1) time, zero allocation.

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
                secure=True,
                d_mu=0.0,
                std_late=0.0,
                converge_streak=self._ewma_streak,
            )

        if self._count == 0:
            return SecureGraspResult(
                secure=False,
                d_mu=0.0,
                std_late=0.0,
                converge_streak=self._ewma_streak,
            )

        mu_late = self._sum / self._count
        variance = self._sum_sq / self._count - mu_late * mu_late
        std_late = math.sqrt(max(0.0, variance))

        ewma_d_mu = 0.0
        slope_val = 0.0
        mode = self._config.mode

        # --- EWMA logic (runs for "ewma" and "both" modes) ---
        if mode in ("ewma", "both"):
            ewma_d_mu = self._update_ewma(mu_late, std_late)

        # --- Slope logic (runs for "slope" and "both" modes) ---
        if mode in ("slope", "both"):
            slope_val = self._update_slope(mu_late, std_late)

        # --- Combine results based on mode ---
        if mode == "ewma":
            self._secure = self._ewma_secure
        elif mode == "slope":
            self._secure = self._slope_secure
        elif mode == "both":
            self._secure = self._ewma_secure and self._slope_secure

        # Diagnostic d_mu: report most relevant value
        if mode == "ewma":
            d_mu = ewma_d_mu
        elif mode == "slope":
            d_mu = abs(slope_val)
        else:  # both
            d_mu = max(ewma_d_mu, abs(slope_val))

        return SecureGraspResult(
            secure=self._secure,
            d_mu=d_mu,
            std_late=std_late,
            converge_streak=self._ewma_streak,
        )

    def _update_ewma(self, mu_late: float, std_late: float) -> float:
        """Run EWMA band detection logic. Returns |mu_late - ewma|."""
        if self._ewma_secure:
            return 0.0

        if self._step_index == 0:
            self._ewma = mu_late
            return 0.0

        lam = self._config.ewma_lambda
        self._ewma = lam * mu_late + (1.0 - lam) * self._ewma

        dev = abs(mu_late - self._ewma)
        if dev < self._config.ewma_band_width and std_late < self._config.std_threshold:
            self._ewma_streak += 1
        else:
            self._ewma_streak = 0

        if self._ewma_streak >= self._config.n_confirm:
            self._ewma_secure = True

        return dev

    def _update_slope(self, mu_late: float, std_late: float) -> float:
        """Run slope-based plateau detection. Returns slope value."""
        if self._slope_secure:
            return 0.0

        W = self._config.slope_window_size

        # Push into circular buffer
        self._mu_buffer[self._buf_head] = mu_late
        self._buf_head = (self._buf_head + 1) % W
        self._buf_count = min(self._buf_count + 1, W)

        if self._buf_count < W or std_late >= self._config.std_threshold:
            return 0.0

        # Single-pass over the circular buffer: compute y_sum, min, max.
        y_sum = 0.0
        buf_min = self._mu_buffer[self._buf_head % W]
        buf_max = buf_min
        for i in range(W):
            val = self._mu_buffer[(self._buf_head + i) % W]
            y_sum += val
            if val < buf_min:
                buf_min = val
            if val > buf_max:
                buf_max = val

        buf_range = buf_max - buf_min
        if buf_range >= self._config.slope_max_range:
            return 0.0  # Early out: oscillation guard fails

        # Linear regression: slope = Σ((x-x̄)(y-ȳ)) / Σ((x-x̄)²)
        x_mean = (W - 1) / 2.0
        y_mean = y_sum / W
        num = 0.0
        den = 0.0
        for i in range(W):
            val = self._mu_buffer[(self._buf_head + i) % W]
            xi = i - x_mean
            num += xi * (val - y_mean)
            den += xi * xi

        slope = num / den if den > 0.0 else 0.0

        if abs(slope) < self._config.slope_threshold:
            self._slope_secure = True

        return slope

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
        """Current consecutive converged step count (EWMA mode)."""
        return self._ewma_streak

    @property
    def config(self) -> SecureGraspConfig:
        """Detector configuration."""
        return self._config
