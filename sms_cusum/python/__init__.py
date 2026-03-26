"""
SMS-CUSUM: Sequential Multi-State CUSUM for Contact and Secure Grasp Detection.

A novel extension of the classical CUSUM change-point detector (Page, 1954)
for detecting contact events and secure grasp convergence in robotic grasping
from force/torque feedback signals, with noise-adaptive sensitivity and
lifecycle-managed baseline.

Novel contributions over standard CUSUM:
1. Noise-adaptive allowance (k_eff = max(k_min, alpha * sigma))
2. Lifecycle-managed baseline (collect -> freeze -> detect -> recover)
3. Automatic sensitivity scaling across objects of varying properties
4. Secure grasp convergence detection via inter-step mean comparison

References:
    Page, E.S. (1954). Biometrika, 41(1-2), 100-115.
    Lorden, G. (1971). Ann. Math. Statist., 42(6), 1897-1908.
    Moustakides, G.V. (1986). Ann. Statist., 14(4), 1379-1387.
"""

from .config import SMSCusumConfig, CusumStageConfig, SecureGraspConfig
from .cusum import CUSUMDetector
from .adaptive_baseline import AdaptiveBaseline
from .secure_grasp import SecureGraspDetector, SecureGraspResult
from .sms_cusum import SMSCusum, GraspState, DetectionEvent, UpdateResult

__version__ = "2.0.0"
__all__ = [
    "SMSCusumConfig",
    "CusumStageConfig",
    "SecureGraspConfig",
    "CUSUMDetector",
    "AdaptiveBaseline",
    "SecureGraspDetector",
    "SecureGraspResult",
    "SMSCusum",
    "GraspState",
    "DetectionEvent",
    "UpdateResult",
]
