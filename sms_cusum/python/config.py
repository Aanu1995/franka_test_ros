"""
SMS-CUSUM configuration: parameter dataclasses for contact and secure grasp detection.

All parameters have defaults derived from empirical analysis of Franka Panda
grasping trials across 6 object types (circle4, circle6, rectangle6,
triangle, triangle4, triangle6) with baseline noise sigma 0.031-0.067 Nm
and contact drop magnitudes 0.13-0.75 Nm.

Secure grasp parameters derived from kitting_bags 3 trials (triangle1, thyme)
with 4 trials showing convergence at GRASP_4-5 (9-18N).
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

    Detection is AND-gated: mean convergence AND signal stability must both
    hold for n_confirm consecutive force ramp steps.

    Parameters
    ----------
    mean_converge_threshold : float
        Maximum |μ_late[N] - μ_late[N-1]| (Nm) for convergence.
        Converged steps typically show |d_mu| < 0.016 Nm.
        Non-converged steps show |d_mu| > 0.065 Nm.
    std_threshold : float
        Maximum σ_late (Nm) for signal stability.
        Stable equilibrium: σ < 0.073 Nm. Guards against cases where
        means coincidentally match but the system is still oscillating.
    n_confirm : int
        Number of consecutive converged steps required before declaring
        secure grasp. Prevents false triggers on oscillatory signals.
    """

    mean_converge_threshold: float = 0.03
    std_threshold: float = 0.08
    n_confirm: int = 2


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
