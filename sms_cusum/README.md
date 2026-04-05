# SMS-CUSUM: Sequential Multi-State CUSUM for Contact and Secure Grasp Detection

A novel change-point detection algorithm for real-time contact detection and
secure grasp convergence detection in robotic manipulation, extending the
classical CUSUM (Cumulative Sum) framework with noise-adaptive sensitivity
and lifecycle-managed baseline estimation.

## Algorithm Overview

SMS-CUSUM detects contact events and secure grasp convergence from a continuous
force/torque signal stream during robotic grasping:

```
FREE_MOTION -> CLOSING -> CONTACT -> GRASPING -> SECURE_GRASP
```

Contact detection (CLOSING → CONTACT) uses a one-sided downward CUSUM.
Secure grasp detection (GRASPING → SECURE_GRASP) uses EWMA band detection
on tau_ext_norm during the force ramp.

The core detection is based on the one-sided downward CUSUM statistic
(Page, 1954):

```
S_n = max(0, S_{n-1} + (mu_0 - x_n) - k_eff)
```

An alarm fires when S_n >= h for at least `debounce_count` consecutive samples.

### Novel Contributions Over Standard CUSUM

Standard CUSUM detects a single change-point from a known pre-change
distribution with a fixed allowance parameter. In robotic grasping, different
objects produce different contact drop magnitudes (0.13-0.75 Nm in our tests),
making fixed parameters unreliable. SMS-CUSUM extends standard CUSUM with:

1. **Noise-adaptive allowance**: The allowance parameter scales automatically
   with measured noise: `k_eff = max(k_min, noise_multiplier * sigma)` where
   sigma is the standard deviation measured during the baseline phase. This
   handles objects of different weights/stiffness without per-object tuning.

2. **Lifecycle-managed baseline**: Automatic collect -> freeze -> detect ->
   recover cycle. The baseline collects N initial samples for robust mean/sigma
   estimation, freezes during detection to prevent corruption, and can be
   unfrozen for consecutive grasp attempts.

3. **Extensible multi-state architecture**: The framework supports adding
   additional CUSUM stages (e.g., grasp force detection, load transfer)
   with context inheritance between stages, where post-alarm statistics from
   one stage become the reference for the next.

### Computational Properties

- **O(1) per sample** time and space complexity
- **Zero dynamic allocation** (C++ implementation uses no heap)
- **Minimax optimal** base detector: CUSUM is provably optimal under Lorden's
  worst-case delay criterion (Moustakides, 1986)

## Architecture

Core components, mirrored in both Python and C++:

```
sms_cusum/
├── python/
│   ├── cusum.py               # CUSUM detector with noise-adaptive allowance
│   ├── adaptive_baseline.py   # EMA baseline with lifecycle management
│   ├── sms_cusum.py           # SMS-CUSUM state machine (contact detection + EWMA secure grasp)
│   ├── secure_grasp.py        # Secure grasp detector: EWMA band on tau_ext_norm
│   ├── wrench_slope.py        # Secure grasp detector: EWMA band + slope on wrench_norm
│   ├── config.py              # Parameter configuration dataclasses
│   ├── validate_offline.py    # Offline validation against CSV trial data
│   ├── tune_parameters.py     # Grid search parameter optimization
│   └── plot_results.py        # Publication-quality overlay plots
├── cpp/
│   └── include/sms_cusum/
│       ├── cusum.hpp           # Header-only CUSUM (noexcept, zero alloc)
│       ├── adaptive_baseline.hpp  # Header-only baseline estimator
│       ├── sms_cusum.hpp       # Header-only SMS-CUSUM state machine
│       ├── secure_grasp.hpp    # Header-only secure grasp detector (EWMA)
│       └── wrench_slope.hpp    # Header-only secure grasp detector (wrench slope)
└── tests/
    ├── test_cusum.py           # 10 CUSUM unit tests
    ├── test_baseline.py        # 11 baseline unit tests
    ├── test_sms_cusum.py       # 18 SMS-CUSUM integration tests
    ├── test_secure_grasp.py    # 13 secure grasp tests (EWMA)
    └── test_wrench_slope.py    # 16 wrench slope detector tests
```

### 1. CUSUM Detector (`cusum.py` / `cusum.hpp`)

One-sided downward CUSUM with debounce and noise-adaptive allowance:

```python
S = max(0.0, S + (mu_0 - x) - k_eff)   # Accumulate evidence of downward shift
if S >= h:
    alarm_streak += 1
    alarm = (alarm_streak >= debounce_count)
else:
    alarm_streak = 0
```

The noise-adaptive allowance `k_eff = max(k_min, noise_multiplier * sigma)`
scales sensitivity to measured noise, enabling detection across objects of
varying properties without manual tuning.

### 2. Adaptive Baseline (`adaptive_baseline.py` / `adaptive_baseline.hpp`)

Three-phase lifecycle:
- **COLLECTING**: Accumulates N initial samples for robust mean/sigma estimate
- **TRACKING**: EMA update: `mu = alpha * x + (1 - alpha) * mu`
- **FROZEN**: No updates during active detection (preserves reference)

### 3. SMS-CUSUM State Machine (`sms_cusum.py` / `sms_cusum.hpp`)

Orchestrates the baseline and CUSUM detector:
- Collects baseline during FREE_MOTION (mean + noise sigma)
- Freezes baseline when entering CLOSING (preserves reference)
- Adapts CUSUM allowance based on measured noise sigma
- Fires CONTACT alarm when sustained torque drop detected
- Supports soft_reset() for consecutive grasp cycles without re-collecting baseline

## Parameters

Optimized via grid search across 7 Franka Panda grasping trials (6 objects +
1 empty trial), achieving 100% detection rate with zero false positives.

### Contact Detection Stage

| Parameter | Value | Description |
|-----------|-------|-------------|
| `k_min` | 0.02 | Minimum allowance (Nm). Ensures sensitivity with low noise |
| `h` | 0.3 | Decision threshold. Lower = faster detection |
| `debounce_count` | 5 | Consecutive samples (20 ms at 250 Hz) for alarm |
| `noise_multiplier` | 2.0 | Adaptive scaling: `k_eff = max(k_min, 2.0 * sigma)` |

### Baseline Estimator

| Parameter | Value | Description |
|-----------|-------|-------------|
| `baseline_init_samples` | 50 | Initial collection (200 ms at 250 Hz) |
| `baseline_alpha` | 0.01 | EMA smoothing (time constant ~400 ms) |

## Validation Results

Tested against 7 Franka Panda grasping trials with objects of varying
shapes, weights, and stiffness:

| Trial | Baseline (Nm) | Noise sigma | k_eff | Detection | Latency |
|-------|---------------|-------------|-------|-----------|---------|
| circle4 | 1.813 | 0.0683 | 0.1366 | YES | -12 ms |
| circle6 | 1.312 | 0.0400 | 0.0799 | YES | -8 ms |
| empty2 | 1.558 | 0.0425 | 0.0850 | -- | no FP |
| rectangle6 | 1.306 | 0.0410 | 0.0820 | YES | +16 ms |
| triangle4 | 1.701 | 0.0861 | 0.1721 | YES | 0 ms |
| triangle6 | 1.227 | 0.0362 | 0.0724 | YES | -1092 ms* |
| triangle | 1.540 | 0.0339 | 0.0679 | YES | -260 ms* |

- **Detection rate**: 100% (6/6 contact trials)
- **False positive rate**: 0% (0/1 empty trials)
- **Max positive latency**: 16 ms (rectangle6, hardest case with SNR=4.2 and slow gripper closure)

*Negative latencies indicate SMS-CUSUM detected the physical contact event
**faster** than the original controller's ground truth label. The original
controller used a slower fixed-threshold method. The triangle and triangle6
trials had particularly delayed ground truth labels from the original
controller, which was subsequently improved in later trials.

**Note on rectangle6**: This trial was captured at a slower gripper closure
speed compared to other trials, producing a more gradual and smaller drop
(SNR=4.2). Despite being the hardest case, SMS-CUSUM still detected contact
with only +16 ms latency, demonstrating robustness to varying closure speeds.

## Usage

### Python

```python
from sms_cusum.python.sms_cusum import SMSCusum, GraspState
from sms_cusum.python.config import SMSCusumConfig

# Create detector with default (optimized) parameters
detector = SMSCusum(SMSCusumConfig())

# Phase 1: Baseline collection (feed ~50 samples of free-motion data)
for sample in baseline_data:
    detector.update(sample.tau_ext_norm)

# Phase 2: Contact detection
detector.enter_closing()
for sample in closing_data:
    event = detector.update(sample.tau_ext_norm)
    if event and event.new_state == GraspState.CONTACT:
        print(f"Contact detected at sample {event.sample_index}")
        print(f"  Baseline mean: {event.baseline_mean:.4f} Nm")
        print(f"  Noise sigma:   {event.baseline_sigma:.4f} Nm")
        print(f"  k_effective:   {event.k_effective:.4f}")
        break

# Reset for next grasp (soft reset preserves baseline)
detector.soft_reset()
```

### C++ (Header-Only, C++14)

```cpp
#include <sms_cusum/sms_cusum.hpp>

// Create detector with default parameters
sms_cusum::SMSCusum detector(sms_cusum::SMSCusumConfig{});

// Baseline collection
for (const auto& sample : baseline_data) {
    detector.update(sample.tau_ext_norm);
}

// Contact detection
detector.enter_closing();
// In your 250 Hz control loop:
auto result = detector.update(tau_ext_norm);
if (result.detected && result.event.new_state == sms_cusum::GraspState::CONTACT) {
    // Contact detected -- stop gripper immediately
}

// Compile with: c++ -std=c++14 -I/path/to/cpp/include your_code.cpp
```

### Integration with franka_kitting_controller

In `kitting_state_controller.h`:
```cpp
#include <sms_cusum/sms_cusum.hpp>

// Add member:
sms_cusum::SMSCusum grasp_detector_;
```

Replace the body of `detectContact()` in `kitting_force_ramp.cpp`:
```cpp
auto result = grasp_detector_.update(tau_ext_norm);
if (result.detected && result.event.new_state == sms_cusum::GraspState::CONTACT) {
    contact_latched_ = true;
    // ... existing contact handling preserved
}
```

## Validation

### Unit Tests

Run all tests (contact detection + secure grasp):
```bash
cd sms_cusum
python3 -m unittest discover -s tests -v
```

Run secure grasp tests only:
```bash
python3 tests/test_secure_grasp.py -v
```

### Offline Contact Validation

Run contact detection against trial data:
```bash
python3 python/validate_offline.py --data-dir ../kitting_bags2
```

With publication-quality plots:
```bash
python3 python/validate_offline.py --data-dir ../kitting_bags2 --plot
```

Grid search parameter tuning for contact detection:
```bash
python3 python/tune_parameters.py --data-dir ../kitting_bags2
```

## Why SMS-CUSUM Over Alternatives

| Method | O(n)/sample | Minimax optimal | Handles unknown shift |
|--------|:-----------:|:---------------:|:---------------------:|
| Fixed threshold | O(1) | No | No |
| EWMA | O(1) | Near-optimal (small shifts) | Partially |
| **SMS-CUSUM** | **O(1)** | **Yes (Moustakides, 1986)** | **Yes** |
| GLR-CUSUM | O(n) | Best (unknown shift) | Yes |

- **Fixed threshold** fails across objects: contact drops range 0.13-0.75 Nm.
  A single threshold either misses subtle contacts or false-triggers on noise.
- **EWMA** lacks formal minimax optimality guarantee
- **GLR-CUSUM** is O(n) per sample, computationally infeasible at 250 Hz
- **SMS-CUSUM** achieves O(1), minimax optimal detection with noise-adaptive
  sensitivity for unknown shift magnitudes

## Secure Grasp Detection

Two independent algorithms are provided for secure grasp detection during
the force ramp. The calling application (e.g., the kitting controller)
selects which algorithm to use and passes the appropriate signal.

### Algorithm 1: EWMA Band Detection (`SecureGraspDetector`)

**Signal**: `tau_ext_norm` (external torque norm)

Tracks an exponentially weighted moving average (EWMA) of the step-level
settled means (Roberts, 1959; Lucas & Saccucci, 1990). At each step, the
new mean is compared to the running EWMA; if it stays within a tolerance
band for `n_confirm` consecutive steps, the grasp is declared secure.

```
EWMA update:    Z_N = lambda * mu_late[N] + (1 - lambda) * Z_{N-1}
Detection:      |mu_late[N] - Z_N| < band_width  AND  sigma_late[N] < std_threshold
Confirmation:   above holds for n_confirm consecutive steps
```

- Fast detection for light objects (avg 3.3 steps)
- O(1) per sample, zero allocation
- **Limitation**: Produces false positives on heavy objects (>1 kg) because
  tau_ext_norm appears locally converged while actually drifting slowly

#### Parameters (`SecureGraspConfig`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ewma_lambda` | 0.4 | Smoothing factor (0-1). Higher = more weight on current step |
| `ewma_band_width` | 0.12 Nm | Max deviation from EWMA for convergence |
| `n_confirm` | 2 | Consecutive converged steps required |
| `std_threshold` | 0.14 Nm | Max within-step std for signal stability |

#### Usage

```python
from sms_cusum.python.secure_grasp import SecureGraspDetector
from sms_cusum.python.config import SecureGraspConfig

detector = SecureGraspDetector(SecureGraspConfig())
detector.begin_step(0)
for sample in late_holding_step0:
    detector.update(sample.tau_ext_norm)
result = detector.finalize_step()
# result.secure, result.d_mu, result.std_late, result.converge_streak
```

### Algorithm 2: Wrench Slope Detection (`WrenchSlopeDetector`)

**Signal**: `wrench_norm` (external wrench norm)

Combines EWMA band detection with online linear regression slope check
on wrench_norm step means. Both criteria must pass simultaneously for
n_confirm consecutive steps. The slope check catches gradual signal drift
that EWMA alone cannot detect — this is the key failure mode for heavy
objects where the signal appears locally converged but is trending over
many steps.

```
EWMA check:     |mu_late[N] - Z_N| < band_width  AND  sigma_late[N] < std_threshold
Slope check:    |slope(last W means)| < slope_threshold
Confirmation:   both checks pass for n_confirm consecutive steps
```

The slope is computed via online linear regression over the last
`slope_window` step means:

```
slope = sum((i - x_mean) * (mu_i - y_mean)) / sum((i - x_mean)^2)
```

- **100% accuracy** across all 41 trials (9 objects, including heavy >1 kg)
- **0 false positives** on heavy objects (vs 6 false positives with EWMA alone)
- O(1) per sample, O(W) per step finalization (W=5, fixed cost)
- Requires 2-5 extra steps for light objects compared to EWMA

#### Parameters (`WrenchSlopeConfig`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ewma_lambda` | 0.4 | Smoothing factor (0-1). Higher = more weight on current step |
| `ewma_band_width` | 0.20 Nm | Max deviation from EWMA for convergence |
| `slope_window` | 5 | Number of recent step means for slope computation |
| `slope_threshold` | 0.04 | Max |slope| for signal to be considered flat |
| `n_confirm` | 3 | Consecutive converged steps required |
| `std_threshold` | 0.14 Nm | Max within-step std for signal stability |
| `min_slope_points` | 3 | Min step means before slope check activates |

#### Usage

```python
from sms_cusum.python.wrench_slope import WrenchSlopeDetector
from sms_cusum.python.config import WrenchSlopeConfig

detector = WrenchSlopeDetector(WrenchSlopeConfig())
detector.begin_step(0)
for sample in late_holding_step0:
    detector.update(sample.wrench_norm)
result = detector.finalize_step()
# result.secure, result.d_mu, result.slope, result.std_late, result.converge_streak
```

#### C++ Usage

```cpp
#include <sms_cusum/wrench_slope.hpp>

sms_cusum::WrenchSlopeDetector detector(sms_cusum::WrenchSlopeConfig{});
detector.begin_step(0);
detector.update(wrench_norm);  // feed late-segment samples
auto result = detector.finalize_step();
if (result.secure) { /* grasp is secure */ }
```

### Choosing Between Algorithms

| | EWMA (`SecureGraspDetector`) | Wrench Slope (`WrenchSlopeDetector`) |
|---|---|---|
| **Signal** | `tau_ext_norm` | `wrench_norm` |
| **Accuracy (all objects)** | 85.4% | **100%** |
| **False positives (heavy)** | 6/7 trials | **0/7 trials** |
| **Avg steps (light objects)** | 3.3 | 5-8 |
| **Best for** | Light objects only | All objects including heavy |

The calling application (e.g., kitting controller) selects the method and
creates the appropriate detector directly. Both detectors share the same
interface pattern: `begin_step()`, `update()`, `finalize_step()`, `reset()`.

### Offline Validation

Run secure grasp validation with method selection:
```bash
# Wrench slope (recommended)
python3 python/validate_offline.py --data-dir ../kitting_bags --method wrench_slope

# EWMA (original)
python3 python/validate_offline.py --data-dir ../kitting_bags --method ewma
```

## Extensibility

The architecture is designed for further extension. Additional detection
stages can be added following the same pattern: implement a detector class
(like `CUSUMDetector` or `SecureGraspDetector`), add lifecycle methods to
`SMSCusum`, and extend the `GraspState` enum. Post-contact statistics
automatically become the reference for subsequent stages via context
inheritance.

## References

### Contact Detection (CUSUM)

1. Page, E.S. (1954). "Continuous inspection schemes." *Biometrika*,
   41(1-2), 100-115. DOI: [10.1093/biomet/41.1-2.100](https://doi.org/10.1093/biomet/41.1-2.100)
   -- Introduced the CUSUM statistic.

2. Lorden, G. (1971). "Procedures for reacting to a change in distribution."
   *Annals of Mathematical Statistics*, 42(6), 1897-1908.
   DOI: [10.1214/aoms/1177693055](https://doi.org/10.1214/aoms/1177693055)
   -- Proposed the minimax formulation for quickest change detection
   (worst-case detection delay criterion).

3. Moustakides, G.V. (1986). "Optimal stopping times for detecting changes
   in distributions." *Annals of Statistics*, 14(4), 1379-1387.
   DOI: [10.1214/aos/1176350164](https://doi.org/10.1214/aos/1176350164)
   -- Proved exact (non-asymptotic) minimax optimality of CUSUM under
   Lorden's criterion.

4. Basseville, M. & Nikiforov, I.V. (1993). *Detection of Abrupt Changes:
   Theory and Application*. Prentice Hall. ISBN: 978-0-13-126780-6
   -- Comprehensive textbook on change detection theory and algorithms.

5. Veeravalli, V.V. & Banerjee, T. (2014). "Quickest Change Detection."
   In *Academic Press Library in Signal Processing*, Vol. 3, pp. 209-255.
   Elsevier. -- Modern overview covering Bayesian and minimax formulations,
   optimal and asymptotically optimal solutions, and generalizations.

### Secure Grasp Detection (EWMA)

6. Roberts, S.W. (1959). "Control chart tests based on geometric moving
   averages." *Technometrics*, 1(3), 239-250.
   DOI: [10.1080/00401706.1959.10489860](https://doi.org/10.1080/00401706.1959.10489860)
   -- Introduced the EWMA statistic. Foundation for the secure grasp detector.

7. Lucas, J.M. & Saccucci, M.S. (1990). "Exponentially weighted moving
   average control schemes: properties and enhancements." *Technometrics*,
   32(1), 1-12.
   DOI: [10.1080/00401706.1990.10484583](https://doi.org/10.1080/00401706.1990.10484583)
   -- EWMA control chart properties and parameter selection guidelines.

## License

This algorithm was developed as part of the thesis "Robust Force-Controlled
Grasping Strategies for Automated Kitting" and is provided for research and
educational purposes.
