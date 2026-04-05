"""
SMS-CUSUM configuration: parameter dataclasses for contact and secure grasp detection.

All parameters have defaults derived from empirical analysis of Franka Panda
grasping trials across 6 object types (circle4, circle6, rectangle6,
triangle, triangle4, triangle6) with baseline noise sigma 0.031-0.067 Nm
and contact drop magnitudes 0.13-0.75 Nm.

Secure grasp parameters derived from kitting_bags 3 trials across 7 object
types (27 trials, 6 objects) using EWMA band detection.
"""

from dataclasses import dataclass, field


@dataclass
class CusumStageConfig:
    """Configuration for a single CUSUM detection stage.

    Parameters
    ----------
    k_min : float
        Minimum allowance parameter. The effective allowance is
        k_eff = max(k_min, noise_multiplier * sigma), ensuring
        sensitivity even when measured noise is very low.
    h : float
        Decision threshold for the CUSUM statistic. Alarm fires
        when S_n >= h for debounce_count consecutive samples.
        Larger h = fewer false positives, more detection delay.
    debounce_count : int
        Number of consecutive samples where S_n >= h before
        declaring a state transition. At 250 Hz, 5 samples = 20 ms.
    noise_multiplier : float
        Scaling factor for adaptive allowance: k_eff = max(k_min, noise_multiplier * sigma).
        Typical range 1.5-4.0. Higher = more robust to noise, but slower to detect
        subtle shifts.
    """

    k_min: float = 0.02
    h: float = 0.3
    debounce_count: int = 5
    noise_multiplier: float = 2.0

    def __post_init__(self) -> None:
        if self.h <= 0.0:
            raise ValueError(f"CusumStageConfig.h must be > 0, got {self.h}")
        if self.debounce_count < 1:
            raise ValueError(f"CusumStageConfig.debounce_count must be >= 1, got {self.debounce_count}")
        if self.noise_multiplier < 0.0:
            raise ValueError(f"CusumStageConfig.noise_multiplier must be >= 0, got {self.noise_multiplier}")


@dataclass
class SecureGraspConfig:
    """Configuration for secure grasp convergence detection (EWMA band).

    Tracks an exponentially weighted moving average of step-level settled
    means during the force ramp. Fires when new means stay within
    +/-band_width of the EWMA for n_confirm consecutive steps.

    Parameters
    ----------
    ewma_lambda : float
        EWMA smoothing factor (0-1). Higher = more weight on current step.
    ewma_band_width : float
        Maximum |mu_late - EWMA| (Nm) for convergence.
    n_confirm : int
        Consecutive converged steps required before declaring secure.
    std_threshold : float
        Maximum sigma_late (Nm) for within-step signal stability.
    """

    ewma_lambda: float = 0.4
    ewma_band_width: float = 0.12
    n_confirm: int = 2
    std_threshold: float = 0.14

    def __post_init__(self) -> None:
        if self.n_confirm < 1:
            raise ValueError(
                f"SecureGraspConfig.n_confirm must be >= 1, "
                f"got {self.n_confirm}"
            )
        if not (0.0 < self.ewma_lambda <= 1.0):
            raise ValueError(
                f"SecureGraspConfig.ewma_lambda must be in (0, 1], "
                f"got {self.ewma_lambda}"
            )


@dataclass
class WrenchSlopeConfig:
    """Configuration for wrench slope secure grasp detector.

    Uses EWMA band detection combined with online linear regression
    slope check on wrench_norm step means. Fires when both the EWMA
    band and slope checks pass for n_confirm consecutive steps.

    Parameters
    ----------
    ewma_lambda : float
        EWMA smoothing factor (0-1). Higher = more weight on current step.
    ewma_band_width : float
        Maximum |mu_late - EWMA| (Nm) for convergence.
    slope_window : int
        Number of recent step means used for slope computation.
    slope_threshold : float
        Maximum |slope| for signal to be considered flat.
    n_confirm : int
        Consecutive converged steps required before declaring secure.
    std_threshold : float
        Maximum sigma_late (Nm) for within-step signal stability.
    min_slope_points : int
        Minimum step means accumulated before slope check activates.
    """

    ewma_lambda: float = 0.4
    ewma_band_width: float = 0.20
    slope_window: int = 5
    slope_threshold: float = 0.04
    n_confirm: int = 3
    std_threshold: float = 0.14
    min_slope_points: int = 3

    def __post_init__(self) -> None:
        if self.n_confirm < 1:
            raise ValueError(
                f"WrenchSlopeConfig.n_confirm must be >= 1, "
                f"got {self.n_confirm}"
            )
        if not (0.0 < self.ewma_lambda <= 1.0):
            raise ValueError(
                f"WrenchSlopeConfig.ewma_lambda must be in (0, 1], "
                f"got {self.ewma_lambda}"
            )
        if self.ewma_band_width <= 0.0:
            raise ValueError(
                f"WrenchSlopeConfig.ewma_band_width must be > 0, "
                f"got {self.ewma_band_width}"
            )
        if self.std_threshold <= 0.0:
            raise ValueError(
                f"WrenchSlopeConfig.std_threshold must be > 0, "
                f"got {self.std_threshold}"
            )
        if self.slope_window < 2:
            raise ValueError(
                f"WrenchSlopeConfig.slope_window must be >= 2, "
                f"got {self.slope_window}"
            )
        if self.slope_threshold <= 0.0:
            raise ValueError(
                f"WrenchSlopeConfig.slope_threshold must be > 0, "
                f"got {self.slope_threshold}"
            )
        if self.min_slope_points < 2:
            raise ValueError(
                f"WrenchSlopeConfig.min_slope_points must be >= 2, "
                f"got {self.min_slope_points}"
            )
        if self.min_slope_points > self.slope_window:
            raise ValueError(
                f"WrenchSlopeConfig.min_slope_points ({self.min_slope_points}) "
                f"must be <= slope_window ({self.slope_window})"
            )


@dataclass
class SMSCusumConfig:
    """Configuration for the SMS-CUSUM detector.

    The detector identifies contact events during gripper closure and
    secure grasp convergence during force ramp, by monitoring tau_ext_norm.

    State graph:
      FREE_MOTION -> CLOSING -> CONTACT -> GRASPING -> SECURE_GRASP

    Parameters
    ----------
    contact_stage : CusumStageConfig
        CUSUM parameters for detecting initial contact (CLOSING -> CONTACT).
    secure_grasp_stage : SecureGraspConfig
        EWMA parameters for detecting secure grasp (GRASPING -> SECURE_GRASP).
    baseline_init_samples : int
        Number of samples to collect for initial baseline mean and sigma.
        At 250 Hz, 50 samples = 0.2 s.
    baseline_alpha : float
        EMA smoothing factor for baseline tracking during FREE_MOTION.
    sample_rate : float
        Expected sample rate in Hz (for diagnostic reporting only).
    """

    contact_stage: CusumStageConfig = field(default_factory=lambda: CusumStageConfig(
        k_min=0.02, h=0.3, debounce_count=5, noise_multiplier=2.0
    ))
    secure_grasp_stage: SecureGraspConfig = field(default_factory=SecureGraspConfig)
    baseline_init_samples: int = 50
    baseline_alpha: float = 0.01
    sample_rate: float = 250.0

    def __post_init__(self) -> None:
        if self.baseline_init_samples < 2:
            raise ValueError(
                f"SMSCusumConfig.baseline_init_samples must be >= 2, "
                f"got {self.baseline_init_samples}"
            )
        if not (0.0 < self.baseline_alpha <= 1.0):
            raise ValueError(
                f"SMSCusumConfig.baseline_alpha must be in (0, 1], "
                f"got {self.baseline_alpha}"
            )
