"""
SMS-CUSUM configuration: parameter dataclasses for contact and secure grasp detection.

All parameters have defaults derived from empirical analysis of Franka Panda
grasping trials across 6 object types (circle4, circle6, rectangle6,
triangle, triangle4, triangle6) with baseline noise sigma 0.031-0.067 Nm
and contact drop magnitudes 0.13-0.75 Nm.

Secure grasp parameters derived from kitting_bags 3 trials across 7 object
types (29 trials total). Two detection modes available: EWMA band and
slope-based plateau detection.
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


@dataclass
class SecureGraspConfig:
    """Configuration for secure grasp convergence detection.

    Detects when increasing grip force no longer changes the steady-state
    tau_ext_norm, indicating the object is fully compressed and the grasp
    is secure.

    Three modes are available, selectable via the ``mode`` parameter:

    - ``"ewma"``: EWMA band detection. Tracks an exponentially weighted
      moving average of step means and checks if new means stay within
      ±band_width of the EWMA. Fast, adaptive.
    - ``"slope"``: Slope-based plateau detection. Fits a linear regression
      to the last W step means and checks if |slope| ≈ 0. Detects flat
      trends regardless of absolute level.
    - ``"both"``: AND-gated combination — both EWMA and Slope must
      independently declare secure before the detector fires.

    Parameters
    ----------
    mode : str
        Detection mode: ``"ewma"``, ``"slope"``, or ``"both"``.
    ewma_lambda : float
        EWMA smoothing factor (0–1). Higher = more weight on current step.
    ewma_band_width : float
        Maximum |μ_late - EWMA| (Nm) for EWMA convergence.
    n_confirm : int
        Consecutive converged steps required for EWMA mode.
    slope_window_size : int
        Number of recent step means used for slope regression.
    slope_threshold : float
        Maximum |slope| for plateau detection.
    slope_max_range : float
        Maximum (max - min) of values in the slope window. Guards against
        oscillating signals where slope is near zero but amplitude is large.
    std_threshold : float
        Maximum σ_late (Nm) for within-step signal stability.
        Shared across all modes.
    """

    mode: str = "ewma"
    ewma_lambda: float = 0.4
    ewma_band_width: float = 0.08
    n_confirm: int = 2
    slope_window_size: int = 3
    slope_threshold: float = 0.03
    slope_max_range: float = 0.15
    std_threshold: float = 0.14

    def __post_init__(self) -> None:
        valid_modes = ("ewma", "slope", "both")
        if self.mode not in valid_modes:
            raise ValueError(
                f"SecureGraspConfig.mode must be one of {valid_modes}, "
                f"got {self.mode!r}"
            )
        if not (2 <= self.slope_window_size <= 8):
            raise ValueError(
                f"SecureGraspConfig.slope_window_size must be in [2, 8], "
                f"got {self.slope_window_size}"
            )
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
        Tuned for detecting negative mean shifts in tau_ext_norm during
        gripper closure.
    secure_grasp_stage : SecureGraspConfig
        Parameters for detecting secure grasp convergence (GRASPING -> SECURE_GRASP).
        Monitors tau_ext_norm mean convergence across consecutive force ramp steps.
    baseline_init_samples : int
        Number of samples to collect for initial baseline mean and sigma.
        At 250 Hz, 50 samples = 0.2 s.
    baseline_alpha : float
        EMA smoothing factor for baseline tracking during FREE_MOTION.
        Smaller = smoother (slower adaptation). Time constant = 1/alpha samples.
    sample_rate : float
        Expected sample rate in Hz (for diagnostic reporting only,
        does not affect algorithm behavior).
    """

    contact_stage: CusumStageConfig = field(default_factory=lambda: CusumStageConfig(
        k_min=0.02, h=0.3, debounce_count=5, noise_multiplier=2.0
    ))
    secure_grasp_stage: SecureGraspConfig = field(default_factory=SecureGraspConfig)
    baseline_init_samples: int = 50
    baseline_alpha: float = 0.01
    sample_rate: float = 250.0
