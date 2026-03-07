# SMS-CUSUM Integration Guide for franka_kitting_controller

Step-by-step instructions for replacing the fixed-threshold contact detection
in `franka_kitting_controller` with SMS-CUSUM.

## Current Controller Architecture

The controller runs at **1 kHz** with state operations triggered at **250 Hz**
via `rate_trigger_` in the `update()` method.

**Current contact detection flow:**

```
BASELINE state           → Collect 50 samples of tau_ext_norm, compute mean
CLOSING_COMMAND state    → Wait for gripper move confirmation
CLOSING state            → Call detectContact() every 250 Hz tick
  detectContact():
    drop = cd_baseline_ - tau_ext_norm
    if drop > threshold (0.1 Nm) for debounce_time (50ms):
      → CONTACT_CONFIRMED
```

**Problem:** Fixed threshold (0.1 Nm) fails across objects with different
drop magnitudes (0.13-0.75 Nm). SMS-CUSUM replaces this with noise-adaptive
detection.

---

## Step 1: Add SMS-CUSUM Header to the Project

Copy the `cpp/include/sms_cusum/` directory into your catkin workspace:

```bash
cp -r sms_cusum/cpp/include/sms_cusum \
  franka_kitting_controller/include/franka_kitting_controller/sms_cusum
```

Or add it as an include path in `CMakeLists.txt`:

```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  # Add SMS-CUSUM headers
  ${CMAKE_CURRENT_SOURCE_DIR}/../sms_cusum/cpp/include
)
```

No `.cpp` files to compile — the library is header-only.

---

## Step 2: Add Member to kitting_state_controller.h

**File:** `include/franka_kitting_controller/kitting_state_controller.h`

Add the include at the top of the file (with the other includes):

```cpp
#include <sms_cusum/sms_cusum.hpp>
```

Add the detector as a member variable. Place it near the existing contact
detection state variables (around line 240, near `cd_baseline_sum_`,
`cd_baseline_count_`, etc.):

```cpp
// --- SMS-CUSUM contact detector (replaces fixed-threshold detection) ---
sms_cusum::SMSCusum sms_detector_{sms_cusum::SMSCusumConfig{}};
```

The default `SMSCusumConfig{}` uses the optimized parameters from grid search
validation (k_min=0.02, h=0.3, debounce=5, noise_mult=2.0). You can also
configure it explicitly:

```cpp
sms_cusum::SMSCusumConfig sms_config;
sms_config.contact_stage = {0.02, 0.3, 5, 2.0};  // k_min, h, debounce, noise_mult
sms_config.baseline_init_samples = 50;             // 0.2s at 250 Hz
sms_config.baseline_alpha = 0.01;                  // EMA smoothing
sms_cusum::SMSCusum sms_detector_{sms_config};
```

---

## Step 3: Feed Baseline Samples in BASELINE State

**File:** `src/kitting_state_controller.cpp`, inside `update()` method

**Replace lines 635-648** (the existing baseline collection block):

```cpp
// BEFORE (existing code):
{
  GraspState bl_state = current_state_.load(std::memory_order_relaxed);
  if (bl_state == GraspState::BASELINE && !cd_baseline_ready_ &&
      !downlift_active_.load(std::memory_order_relaxed)) {
    cd_baseline_sum_ += tau_ext_norm;
    cd_baseline_count_++;
    if (cd_baseline_count_ >= kContactBaselineSamples) {
      cd_baseline_ = cd_baseline_sum_ / cd_baseline_count_;
      cd_baseline_ready_ = true;
      ROS_INFO("  [BASELINE]  tau_ext_norm baseline ready: %.3f Nm  (from %d samples, 0.2s)",
               cd_baseline_, cd_baseline_count_);
    }
  }
}
```

```cpp
// AFTER (SMS-CUSUM replacement):
{
  GraspState bl_state = current_state_.load(std::memory_order_relaxed);
  if (bl_state == GraspState::BASELINE &&
      !downlift_active_.load(std::memory_order_relaxed)) {
    // Feed tau_ext_norm to SMS-CUSUM baseline estimator.
    // During FREE_MOTION state, the detector collects init_samples (50)
    // to compute mean and noise sigma, then tracks via EMA.
    sms_detector_.update(tau_ext_norm);

    // Keep the existing cd_baseline_ready_ flag for compatibility
    // with other parts of the controller that check it.
    if (!cd_baseline_ready_ && sms_detector_.baseline_ready()) {
      cd_baseline_ready_ = true;
      cd_baseline_ = sms_detector_.baseline().mean();
      cd_baseline_count_ = sms_detector_.config().baseline_init_samples;
      ROS_INFO("  [BASELINE]  SMS-CUSUM baseline ready: mu=%.3f Nm, sigma=%.4f Nm  "
               "(from %d samples, 0.2s)",
               sms_detector_.baseline().mean(),
               sms_detector_.baseline().sigma(),
               sms_detector_.config().baseline_init_samples);
    }
  }
}
```

**What happens here:**
- SMS-CUSUM's `AdaptiveBaseline` collects 50 samples (COLLECTING phase)
- It computes both **mean** and **noise sigma** (the existing code only computes mean)
- After 50 samples, the baseline transitions to TRACKING phase (EMA updates)
- The noise sigma is used later to auto-scale detection sensitivity

---

## Step 4: Call enter_closing() When Entering CLOSING State

**File:** `src/kitting_force_ramp.cpp`, inside `runClosingTransitions()`

**At the transition from CLOSING_COMMAND to CLOSING** (around line 56, where
the state changes to CLOSING), add:

```cpp
// Transition CLOSING_COMMAND → CLOSING now that gripper is confirmed moving
if (state == GraspState::CLOSING_COMMAND) {
  current_state_.store(GraspState::CLOSING, std::memory_order_relaxed);
  state = GraspState::CLOSING;
  publishStateLabel("CLOSING");
  logStateTransition("CLOSING", "Gripper move confirmed — closing");

  // >>> ADD THIS: Activate SMS-CUSUM contact detection <<<
  if (sms_detector_.baseline_ready()) {
    sms_detector_.enter_closing();
    ROS_INFO("  [CLOSING]  SMS-CUSUM activated: k_eff=%.4f  baseline=%.3f  sigma=%.4f",
             sms_detector_.contact_cusum().k_effective(),
             sms_detector_.baseline().mean(),
             sms_detector_.baseline().sigma());
  }
}
```

**What happens here:**
- `enter_closing()` freezes the baseline (prevents corruption during detection)
- It adapts the CUSUM allowance: `k_eff = max(0.02, 2.0 * sigma)`
- The detector is now monitoring for a sustained torque drop

---

## Step 5: Replace detectContact() Body

**File:** `src/kitting_force_ramp.cpp`

**Replace the body of `detectContact()`** (lines 100-145):

```cpp
void KittingStateController::detectContact(const ros::Time& time,
                                            const GripperData& gripper_snapshot,
                                            double tau_ext_norm) {
  bool gripper_data_valid = (gripper_snapshot.stamp != ros::Time(0)) &&
                            ((time - gripper_snapshot.stamp).toSec() < 0.5);
  if (!gripper_data_valid) {
    return;
  }

  // Fallback: if baseline not ready, collect now (same as original)
  if (!sms_detector_.baseline_ready()) {
    sms_detector_.update(tau_ext_norm);
    if (sms_detector_.baseline_ready() && !cd_baseline_ready_) {
      cd_baseline_ready_ = true;
      cd_baseline_ = sms_detector_.baseline().mean();
      ROS_WARN("  [CONTACT]  SMS-CUSUM baseline ready (fallback): mu=%.3f Nm, sigma=%.4f Nm",
               sms_detector_.baseline().mean(), sms_detector_.baseline().sigma());
      // Now activate detection
      sms_detector_.enter_closing();
    }
    return;
  }

  // --- SMS-CUSUM contact detection (replaces fixed threshold + debounce) ---
  auto result = sms_detector_.update(tau_ext_norm);

  if (result.detected &&
      result.event.new_state == sms_cusum::GraspState::CONTACT) {
    contact_latched_ = true;

    current_state_.store(GraspState::CONTACT_CONFIRMED, std::memory_order_relaxed);
    publishStateLabel("CONTACT_CONFIRMED");
    logStateTransition("CONTACT_CONFIRMED", "SMS-CUSUM contact detected — gripper stopping");

    requestGripperStop("Contact");

    ROS_INFO("  [CONTACT_CONFIRMED]  SMS-CUSUM detection: "
            "S=%.3f  baseline=%.3f  sigma=%.4f  k_eff=%.4f  tau=%.3f  w=%.4f",
            result.event.cusum_statistic,
            result.event.baseline_mean,
            result.event.baseline_sigma,
            result.event.k_effective,
            tau_ext_norm,
            gripper_snapshot.width);
  }
}
```

**What this does:**
- Replaces `drop > threshold` with CUSUM statistic accumulation
- The CUSUM accumulates evidence: `S = max(0, S + (baseline - tau) - k_eff)`
- Fires when `S >= h` for `debounce_count` consecutive samples
- `k_eff` automatically scales to noise level, so no per-object threshold tuning needed

---

## Step 6: Reset Between Grasp Attempts

**Where the controller resets for a new grasp cycle** (when transitioning
back to BASELINE state or after FAILED/SUCCESS), add:

```cpp
// Soft reset: preserves baseline mu/sigma, avoids re-collecting
sms_detector_.soft_reset();
cd_baseline_ready_ = false;  // Reset compatibility flag if needed
```

Or for a full reset (e.g., after robot repositioning):

```cpp
sms_detector_.reset();  // Full reset: fresh baseline collection
cd_baseline_ready_ = false;
cd_baseline_sum_ = 0.0;
cd_baseline_count_ = 0;
```

**When to use which:**
- `soft_reset()`: Between consecutive grasps on the **same robot position**.
  Preserves baseline estimate, resumes EMA tracking. Faster recovery.
- `reset()`: After robot **repositioning** or **configuration change**.
  Re-collects baseline from scratch.

---

## Step 7: Remove Old Detection Variables (Optional Cleanup)

Once SMS-CUSUM is working, you can optionally remove the old fixed-threshold
variables from `kitting_state_controller.h`:

```cpp
// These can be removed (SMS-CUSUM handles everything internally):
// double cd_baseline_sum_{0.0};     // Replaced by AdaptiveBaseline
// int    cd_baseline_count_{0};     // Replaced by AdaptiveBaseline
// double cd_baseline_{0.0};         // Replaced by sms_detector_.baseline().mean()
// DebounceState gripper_debounce_;  // Replaced by CUSUM debounce

// Keep cd_baseline_ready_ if other parts of the controller check it.
// Keep rt_contact_torque_thresh_ and rt_contact_debounce_time_ if you
// want to keep the YAML interface for fallback.
```

---

## Summary: What Changes Where

| File | What Changes |
|------|-------------|
| `kitting_state_controller.h` | Add `#include`, add `sms_cusum::SMSCusum sms_detector_` member |
| `kitting_state_controller.cpp` | Replace baseline collection block in `update()` (lines 635-648) |
| `kitting_force_ramp.cpp` | Add `enter_closing()` call at CLOSING transition (~line 56) |
| `kitting_force_ramp.cpp` | Replace `detectContact()` body (lines 100-145) |
| Reset locations | Add `sms_detector_.soft_reset()` or `sms_detector_.reset()` |

**Total: ~40 lines changed, ~30 lines added.** The existing state machine
(CONTACT_CONFIRMED -> CONTACT -> GRASPING -> UPLIFT -> EVALUATE etc.) is
completely preserved. Only the detection kernel is replaced.

---

## Signal Flow Diagram

```
                    250 Hz update() loop
                           |
                    [BASELINE state]
                           |
                    sms_detector_.update(tau_ext_norm)
                           |
                    AdaptiveBaseline collects 50 samples
                    Computes: mean (mu) + noise sigma
                           |
                    [CLOSING_COMMAND → CLOSING]
                           |
                    sms_detector_.enter_closing()
                      - Freezes baseline (mu, sigma preserved)
                      - Adapts k_eff = max(0.02, 2.0 * sigma)
                      - Resets CUSUM statistic S = 0
                           |
                    [CLOSING state, each 250 Hz tick]
                           |
                    sms_detector_.update(tau_ext_norm)
                      - S = max(0, S + (mu - tau) - k_eff)
                      - If S >= h for 5 consecutive samples:
                        → CONTACT detected!
                           |
                    [CONTACT_CONFIRMED → CONTACT → GRASPING → ...]
                    (existing state machine continues unchanged)
```

---

## Diagnostic Logging

SMS-CUSUM provides rich diagnostic info in every `DetectionEvent`:

| Field | Description | Example |
|-------|-------------|---------|
| `cusum_statistic` | CUSUM S_n at alarm | 1.680 |
| `baseline_mean` | Frozen baseline mu | 1.813 Nm |
| `baseline_sigma` | Measured noise sigma | 0.068 Nm |
| `k_effective` | Adapted allowance | 0.137 |
| `sample_index` | Sample count at detection | 2003 |

You can also access the CUSUM statistic during CLOSING for real-time
monitoring (e.g., for plotting or debugging):

```cpp
double S = sms_detector_.contact_cusum().statistic();
double k = sms_detector_.contact_cusum().k_effective();
```

---

## Tuning (If Needed)

The default parameters are optimized for the Franka Panda with objects ranging
from 0.13-0.75 Nm contact drops. If you need to adjust:

| Parameter | Effect of Increasing | Default |
|-----------|---------------------|---------|
| `k_min` | Less sensitive to small drops | 0.02 |
| `h` | Slower detection, fewer false positives | 0.3 |
| `debounce_count` | More robust to transients, adds latency | 5 (20ms) |
| `noise_multiplier` | More noise-tolerant, slower on subtle drops | 2.0 |
| `baseline_init_samples` | More robust baseline, longer startup | 50 (200ms) |

Use the Python grid search for systematic tuning:
```bash
cd sms_cusum
python3 python/tune_parameters.py --data-dir ../kitting_bags2
```
