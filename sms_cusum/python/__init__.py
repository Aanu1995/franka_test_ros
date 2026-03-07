"""
SMS-CUSUM: Sequential Multi-State CUSUM for Contact Detection.

A novel extension of the classical CUSUM change-point detector (Page, 1954)
for detecting contact events in robotic grasping from force/torque feedback
signals, with noise-adaptive sensitivity and lifecycle-managed baseline.

Novel contributions over standard CUSUM:
1. Noise-adaptive allowance (k_eff = max(k_min, alpha * sigma))
2. Lifecycle-managed baseline (collect -> freeze -> detect -> recover)
3. Automatic sensitivity scaling across objects of varying properties

References:
    Page, E.S. (1954). Biometrika, 41(1-2), 100-115.
    Lorden, G. (1971). Ann. Math. Statist., 42(6), 1897-1908.
    Moustakides, G.V. (1986). Ann. Statist., 14(4), 1379-1387.
"""

from .config import SMSCusumConfig, CusumStageConfig
from .cusum import CUSUMDetector
from .adaptive_baseline import AdaptiveBaseline
from .sms_cusum import SMSCusum, GraspState, DetectionEvent

__version__ = "1.0.0"
__all__ = [
    "SMSCusumConfig",
    "CusumStageConfig",
    "CUSUMDetector",
    "AdaptiveBaseline",
    "SMSCusum",
    "GraspState",
    "DetectionEvent",
]
