"""
SMS-CUSUM configuration: parameter dataclasses for contact detection.

All parameters have defaults derived from empirical analysis of Franka Panda
grasping trials across 6 object types (circle4, circle6, rectangle6,
triangle, triangle4, triangle6) with baseline noise sigma 0.031-0.067 Nm
and contact drop magnitudes 0.13-0.75 Nm.
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
class SMSCusumConfig:
    """Configuration for the SMS-CUSUM contact detector.

    The detector identifies contact events during gripper closure by
    monitoring tau_ext_norm for negative mean shifts (drops).

    State graph:
      FREE_MOTION -> CLOSING -> CONTACT

    Parameters
    ----------
    contact_stage : CusumStageConfig
        CUSUM parameters for detecting initial contact (CLOSING -> CONTACT).
        Tuned for detecting negative mean shifts in tau_ext_norm during
        gripper closure.
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
    baseline_init_samples: int = 50
    baseline_alpha: float = 0.01
    sample_rate: float = 250.0
