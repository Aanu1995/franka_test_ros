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
Secure grasp detection (GRASPING → SECURE_GRASP) uses inter-step mean
convergence of tau_ext_norm during the force ramp.

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

Three core components, mirrored in both Python and C++:

```
sms_cusum/
├── python/
│   ├── cusum.py               # CUSUM detector with noise-adaptive allowance
│   ├── adaptive_baseline.py   # EMA baseline with lifecycle management
│   ├── sms_cusum.py           # SMS-CUSUM state machine (core algorithm)
│   ├── secure_grasp.py        # Secure grasp detector (EWMA + Slope modes)
│   ├── config.py              # Parameter configuration dataclasses
│   ├── validate_offline.py    # Offline validation against CSV trial data
│   ├── tune_parameters.py     # Grid search parameter optimization
│   └── plot_results.py        # Publication-quality overlay plots
├── cpp/
│   └── include/sms_cusum/
│       ├── cusum.hpp           # Header-only CUSUM (noexcept, zero alloc)
│       ├── adaptive_baseline.hpp  # Header-only baseline estimator
│       ├── sms_cusum.hpp       # Header-only SMS-CUSUM state machine
│       └── secure_grasp.hpp    # Header-only secure grasp detector
└── tests/
    ├── test_cusum.py           # 10 CUSUM unit tests
    ├── test_baseline.py        # 11 baseline unit tests
    ├── test_sms_cusum.py       # 12 SMS-CUSUM integration tests
    └── test_secure_grasp.py    # 19 secure grasp tests (EWMA/Slope/Both)
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

After contact is confirmed and the force ramp begins, SMS-CUSUM detects
when the grasp has become secure — i.e., when increasing grip force no
longer changes the steady-state tau_ext_norm, indicating the object is
fully compressed and additional force is unnecessary.

### Detection Modes

Three modes are available, selectable via the `secure_grasp_mode` parameter
in the controller YAML config:

#### Mode 1: EWMA Band Detection (`"ewma"`, default)

Tracks an exponentially weighted moving average (EWMA) of the step-level
settled means. At each step, the new mean is compared to the running EWMA;
if it stays within a tolerance band for `n_confirm` consecutive steps, the
grasp is declared secure.

The EWMA is a standard signal processing technique for online mean
estimation (Roberts, 1959; Lucas & Saccucci, 1990), widely used in
statistical process control for detecting small sustained shifts.

```
EWMA update:    Z_N = lambda * mu_late[N] + (1 - lambda) * Z_{N-1}
Detection:      |mu_late[N] - Z_N| < band_width  AND  sigma_late[N] < std_threshold
Confirmation:   above holds for n_confirm consecutive steps
```

- Adapts naturally to different objects' tau_ext_norm baselines
- Recursive O(1) computation per step, zero allocation
- Empirical result: **100% detection (29/29), avg 3.2 steps**

#### Mode 2: Slope-Based Plateau Detection (`"slope"`)

Fits a linear regression to the last W step means. If |slope| is near zero
and the range of values in the window is small, the signal is flat —
indicating a plateau and a secure grasp.

This approach draws on steady-state detection methods from process control,
where a slope near zero combined with low residual variance is a standard
criterion for declaring a process has reached equilibrium (Kelly &
Hedengren, 2013).

```
Regression:     slope = sum((x_i - x_mean)(y_i - y_mean)) / sum((x_i - x_mean)^2)
Detection:      |slope| < slope_threshold  AND  range(window) < slope_max_range
                AND  sigma_late[N] < std_threshold
```

- Detects a different property than EWMA: "trend is flat" vs "value is near
  running average"
- O(W) per step finalization (W=3, negligible)
- Empirical result: **93-100% detection (27-29/29), avg 4.3 steps**

#### Mode 3: AND-Gated Combination (`"both"`)

Both EWMA and Slope must independently declare secure before the detector
fires. This is the most conservative mode — suitable when false early
termination is costlier than additional force ramp steps.

### Parameters

| Parameter | Default | Mode | Description |
|-----------|---------|------|-------------|
| `mode` | `"ewma"` | All | Detection mode: `"ewma"`, `"slope"`, or `"both"` |
| `ewma_lambda` | 0.4 | EWMA | Smoothing factor (0-1). Higher = more weight on current step |
| `ewma_band_width` | 0.08 Nm | EWMA | Max deviation from EWMA for convergence |
| `n_confirm` | 2 | EWMA | Consecutive converged steps required |
| `slope_window_size` | 3 | Slope | Number of recent step means for regression |
| `slope_threshold` | 0.03 | Slope | Max |slope| for plateau detection |
| `slope_max_range` | 0.15 Nm | Slope | Max range of window values (oscillation guard) |
| `std_threshold` | 0.14 Nm | All | Max within-step std for signal stability |

### Lifecycle

```python
from sms_cusum.python.config import SecureGraspConfig, SMSCusumConfig

# Configure mode
sg_config = SecureGraspConfig(mode="ewma")  # or "slope" or "both"
config = SMSCusumConfig(secure_grasp_stage=sg_config)
detector = SMSCusum(config)

# Phase 1: Baseline + Contact (as before)
...

# Phase 3: Secure grasp detection
detector.enter_grasping()           # Resets detector, begins step 0
for step in range(num_steps):
    # Feed late-segment samples
    for sample in late_holding:
        detector.update(sample)
    result = detector.finalize_grasp_step()
    if result.detected:
        break                       # Secure grasp — skip to UPLIFT
    detector.begin_grasp_step(step + 1)
```

### Controller Configuration

Default mode is set in `kitting_state_controller.yaml`:
```yaml
secure_grasp_mode: "ewma"    # "ewma", "slope", or "both"
```

The mode can be overridden per-trial via the `secure_grasp_mode` field
in the `KittingGripperCommand` message (empty string = use YAML default):
```bash
# Use slope mode for this trial only
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', secure_grasp_mode: 'slope'}" --once

# Use AND-gated mode for this trial only
rostopic pub /kitting_controller/state_cmd franka_kitting_controller/KittingGripperCommand \
  "{command: 'GRASPING', secure_grasp_mode: 'both'}" --once
```

### Validation

Validated against 29 trials across 7 objects (triangle1, triangle,
smallCircle, bigCircle, thyme, irregularShape, irregularShape1):

| Mode | Detection Rate | Avg Steps | Notes |
|------|---------------|-----------|-------|
| EWMA | 29/29 (100%) | 3.2 | Fastest, most adaptive |
| Slope | 27-29/29 (93-100%) | 4.3 | Misses highly oscillatory trials |
| Both | 27+/29 | 4+ | Most conservative |

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

### Secure Grasp Detection (EWMA and Slope)

6. Roberts, S.W. (1959). "Control chart tests based on geometric moving
   averages." *Technometrics*, 1(3), 239-250.
   DOI: [10.1080/00401706.1959.10489860](https://doi.org/10.1080/00401706.1959.10489860)
   -- Introduced the EWMA statistic for process monitoring. Foundation
   for the EWMA band detection mode.

7. Lucas, J.M. & Saccucci, M.S. (1990). "Exponentially weighted moving
   average control schemes: properties and enhancements." *Technometrics*,
   32(1), 1-12.
   DOI: [10.1080/00401706.1990.10484583](https://doi.org/10.1080/00401706.1990.10484583)
   -- Comprehensive analysis of EWMA control chart properties, parameter
   selection guidelines, and comparison with CUSUM. Informs the choice
   of lambda and band width parameters.

8. Kelly, J.D. & Hedengren, J.D. (2013). "A steady-state detection (SSD)
   algorithm to detect non-stationary drifts in processes." *Journal of
   Process Control*, 23(3), 326-331.
   DOI: [10.1016/j.jprocont.2012.12.001](https://doi.org/10.1016/j.jprocont.2012.12.001)
   -- Slope-near-zero combined with residual analysis as a criterion for
   steady-state detection. Informs the slope-based plateau detection mode.

9. Rhinehart, R.R. (2013). "Automated steady and transient state
   identification in noisy processes." *Proceedings of the American Control
   Conference (ACC)*, 4477-4493.
   -- Variance ratio method for online steady-state identification.
   Evaluated during algorithm selection; the core variance ratio idea
   informed the oscillation guard (slope_max_range) used in slope mode.

## License

This algorithm was developed as part of the thesis "Robust Force-Controlled
Grasping Strategies for Automated Kitting" and is provided for research and
educational purposes.
