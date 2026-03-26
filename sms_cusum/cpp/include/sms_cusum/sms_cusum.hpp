/**
 * @file sms_cusum.hpp
 * @brief SMS-CUSUM: Sequential Multi-State CUSUM for Contact and Secure Grasp Detection.
 *
 * A novel extension of the classical CUSUM change-point detector that detects
 * contact events from a continuous force/torque signal stream, with
 * noise-adaptive sensitivity and lifecycle-managed baseline estimation.
 *
 * Novel contributions over standard CUSUM (Page, 1954):
 *     1. Noise-adaptive allowance (k_eff = max(k_min, alpha * sigma))
 *     2. Lifecycle-managed baseline (collect -> freeze -> detect -> recover)
 *     3. Automatic sensitivity scaling across objects of varying properties
 *
 * Computational complexity: O(1) per sample, zero dynamic allocation.
 * Requires C++14 or later.
 *
 * References:
 *     Page, E.S. (1954). Biometrika, 41(1-2), 100-115.
 *     Lorden, G. (1971). Ann. Math. Statist., 42(6), 1897-1908.
 *     Moustakides, G.V. (1986). Ann. Statist., 14(4), 1379-1387.
 *     Basseville, M. & Nikiforov, I.V. (1993). Detection of Abrupt Changes. Prentice Hall.
 *     Veeravalli, V.V. & Banerjee, T. (2014). Academic Press Library in Signal Processing, Vol. 3.
 */

#pragma once

#include <cstdint>    // uint8_t, int32_t
#include <cstdio>     // std::snprintf

#include "adaptive_baseline.hpp"
#include "cusum.hpp"
#include "secure_grasp.hpp"

namespace sms_cusum {

// ============================================================================
// Grasp state enumeration
// ============================================================================

/**
 * @brief Detected states in the SMS-CUSUM state machine.
 *
 * State graph:
 *   FREE_MOTION -> CLOSING -> CONTACT -> GRASPING -> SECURE_GRASP
 */
enum class GraspState : uint8_t {
    FREE_MOTION  = 0,
    CLOSING      = 1,
    CONTACT      = 2,
    GRASPING     = 3,
    SECURE_GRASP = 4
};

/**
 * @brief Convert GraspState to a human-readable C-string.
 */
inline const char* grasp_state_name(GraspState s) noexcept {
    switch (s) {
        case GraspState::FREE_MOTION:  return "FREE_MOTION";
        case GraspState::CLOSING:      return "CLOSING";
        case GraspState::CONTACT:      return "CONTACT";
        case GraspState::GRASPING:     return "GRASPING";
        case GraspState::SECURE_GRASP: return "SECURE_GRASP";
        default:                       return "UNKNOWN";
    }
}

// ============================================================================
// Detection event
// ============================================================================

/**
 * @brief Emitted when a state transition is detected.
 *
 * Contains all diagnostic information about the detected transition,
 * including CUSUM statistics and baseline parameters at the moment
 * of detection.
 */
struct DetectionEvent {
    GraspState prev_state;        ///< State before the transition.
    GraspState new_state;         ///< State after the transition.
    int32_t    sample_index;      ///< Sample index (0-based) at detection.
    double     cusum_statistic;   ///< CUSUM statistic S_n at detection.
    double     baseline_mean;     ///< Reference mean used by the CUSUM stage.
    double     baseline_sigma;    ///< Noise sigma of the reference baseline.
    double     k_effective;       ///< Effective allowance parameter used for detection.

    /**
     * @brief Fixed-size buffer for human-readable transition description.
     */
    char detail[256];
};

/**
 * @brief Return type from SMSCusum::update() (C++14-compatible, no std::optional).
 *
 * Check `detected` before accessing `event`. If `detected` is false,
 * the `event` field is uninitialized.
 */
struct UpdateResult {
    bool           detected;  ///< True if a state transition was detected.
    DetectionEvent event;     ///< Valid only when detected == true.
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for the SMS-CUSUM detector.
 *
 * State graph:
 *   FREE_MOTION -> CLOSING -> CONTACT -> GRASPING -> SECURE_GRASP
 */
struct SMSCusumConfig {
    CusumStageConfig  contact_stage       = {0.02, 0.3, 5, 2.0};
    SecureGraspConfig secure_grasp_stage  = {};
    int32_t baseline_init_samples         = 50;
    double  baseline_alpha                = 0.01;
    double  sample_rate                   = 250.0;
};

// ============================================================================
// SMS-CUSUM detector
// ============================================================================

/**
 * @brief SMS-CUSUM detector for contact detection from force/torque feedback.
 *
 * Orchestrates an adaptive baseline and a CUSUM detector to identify
 * contact events (torque drops) during gripper closure.
 *
 * Usage:
 * @code
 *     sms_cusum::SMSCusum detector(sms_cusum::SMSCusumConfig{});
 *
 *     // Phase 1: Baseline collection (FREE_MOTION)
 *     for (auto& sample : baseline_data) {
 *         detector.update(sample.tau_ext_norm);
 *     }
 *
 *     // Phase 2: Contact detection
 *     detector.enter_closing();
 *     auto result = detector.update(tau_ext_norm);
 *     if (result.detected && result.event.new_state == GraspState::CONTACT) {
 *         // Contact detected!
 *     }
 * @endcode
 *
 * Real-time safe: O(1) per sample, zero dynamic allocation, all hot-path
 * methods noexcept. Requires C++14 or later.
 */
class SMSCusum {
public:
    explicit SMSCusum(const SMSCusumConfig& config = SMSCusumConfig{}) noexcept
        : config_{config}
        , state_{GraspState::FREE_MOTION}
        , sample_idx_{0}
        , baseline_{config.baseline_init_samples, config.baseline_alpha}
        , contact_cusum_{config.contact_stage}
        , secure_grasp_{config.secure_grasp_stage}
        , event_count_{0}
    {}

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    GraspState state() const noexcept { return state_; }
    const char* state_name() const noexcept { return grasp_state_name(state_); }
    int32_t sample_index() const noexcept { return sample_idx_; }
    bool baseline_ready() const noexcept { return baseline_.ready(); }
    const AdaptiveBaseline& baseline() const noexcept { return baseline_; }
    AdaptiveBaseline& baseline() noexcept { return baseline_; }
    const CUSUMDetector& contact_cusum() const noexcept { return contact_cusum_; }
    const SecureGraspDetector& secure_grasp_detector() const noexcept { return secure_grasp_; }
    int32_t event_count() const noexcept { return event_count_; }
    const DetectionEvent& event(int32_t i) const noexcept { return events_[i]; }
    const SMSCusumConfig& config() const noexcept { return config_; }

    // ------------------------------------------------------------------
    // State transition API
    // ------------------------------------------------------------------

    /**
     * @brief Signal that the gripper is starting to close.
     *
     * Freezes the baseline and activates the contact CUSUM stage.
     */
    void enter_closing() noexcept {
        baseline_.freeze();
        contact_cusum_.reset();
        contact_cusum_.adapt(baseline_.sigma());
        state_ = GraspState::CLOSING;
    }

    /**
     * @brief Signal that the force ramp is starting.
     *
     * Call after CONTACT is confirmed and the first grasp command
     * is about to be sent.
     */
    void enter_grasping() noexcept {
        secure_grasp_.reset();
        secure_grasp_.begin_step(0);
        state_ = GraspState::GRASPING;
    }

    /**
     * @brief Signal start of a new GRASP_N HOLDING phase.
     * @param step_index Zero-based step index.
     */
    void begin_grasp_step(int32_t step_index) noexcept {
        secure_grasp_.begin_step(step_index);
    }

    /**
     * @brief Signal end of GRASP_N HOLDING phase. Check for secure grasp.
     * @return UpdateResult with detected=true if secure grasp was detected.
     */
    UpdateResult finalize_grasp_step() noexcept {
        auto sg = secure_grasp_.finalize_step();

        UpdateResult result = {};
        result.detected = false;

        if (sg.secure && state_ == GraspState::GRASPING) {
            result.detected = true;
            result.event.prev_state      = GraspState::GRASPING;
            result.event.new_state       = GraspState::SECURE_GRASP;
            result.event.sample_index    = sample_idx_;
            result.event.cusum_statistic = static_cast<double>(sg.converge_streak);
            result.event.baseline_mean   = sg.d_mu;
            result.event.baseline_sigma  = sg.std_late;
            result.event.k_effective     = 0.0;

            std::snprintf(result.event.detail, sizeof(result.event.detail),
                "Secure grasp: d_mu=%.4f, std=%.4f, streak=%d",
                sg.d_mu, sg.std_late, sg.converge_streak);

            state_ = GraspState::SECURE_GRASP;
            record_event(result.event);
        }

        return result;
    }

    // ------------------------------------------------------------------
    // Core update loop: O(1) per sample
    // ------------------------------------------------------------------

    /**
     * @brief Process one sample. O(1) time, zero dynamic allocation.
     *
     * @param tau_ext_norm External torque norm (primary detection signal).
     * @return UpdateResult with detected=true if a state transition occurred.
     */
    UpdateResult update(double tau_ext_norm) noexcept {
        ++sample_idx_;

        UpdateResult result = {};
        result.detected = false;

        switch (state_) {
            case GraspState::FREE_MOTION:
                baseline_.update(tau_ext_norm);
                break;

            case GraspState::CLOSING: {
                if (!baseline_.ready()) {
                    break;
                }

                bool alarm = contact_cusum_.update(baseline_.mean(), tau_ext_norm);
                if (alarm) {
                    result.detected = true;
                    result.event.prev_state      = GraspState::CLOSING;
                    result.event.new_state       = GraspState::CONTACT;
                    result.event.sample_index    = sample_idx_;
                    result.event.cusum_statistic = contact_cusum_.statistic();
                    result.event.baseline_mean   = baseline_.mean();
                    result.event.baseline_sigma  = baseline_.sigma();
                    result.event.k_effective     = contact_cusum_.k_effective();

                    std::snprintf(result.event.detail, sizeof(result.event.detail),
                        "Contact detected: S=%.3f, baseline=%.4f, sigma=%.4f, k_eff=%.4f",
                        contact_cusum_.statistic(),
                        baseline_.mean(),
                        baseline_.sigma(),
                        contact_cusum_.k_effective());

                    state_ = GraspState::CONTACT;
                    record_event(result.event);
                }
                break;
            }

            case GraspState::GRASPING:
                // Accumulate samples for secure grasp detection.
                // Detection happens via finalize_grasp_step().
                secure_grasp_.update(tau_ext_norm);
                break;

            case GraspState::CONTACT:
            case GraspState::SECURE_GRASP:
                // Waiting for lifecycle calls, no per-sample transitions
                break;

            default:
                break;
        }

        return result;
    }

    // ------------------------------------------------------------------
    // Reset / lifecycle
    // ------------------------------------------------------------------

    void reset() noexcept {
        state_      = GraspState::FREE_MOTION;
        sample_idx_ = 0;
        baseline_.reset();
        contact_cusum_.reset();
        secure_grasp_.reset();
        event_count_ = 0;
    }

    void soft_reset() noexcept {
        state_ = GraspState::FREE_MOTION;
        baseline_.unfreeze();
        contact_cusum_.reset();
        secure_grasp_.reset();
    }

private:
    void record_event(const DetectionEvent& event) noexcept {
        if (event_count_ < kMaxEvents) {
            events_[event_count_] = event;
            ++event_count_;
        }
    }

    static const int32_t kMaxEvents = 16;

    SMSCusumConfig       config_;
    GraspState           state_;
    int32_t              sample_idx_;
    AdaptiveBaseline     baseline_;
    CUSUMDetector        contact_cusum_;
    SecureGraspDetector  secure_grasp_;
    DetectionEvent       events_[kMaxEvents];
    int32_t              event_count_;
};

}  // namespace sms_cusum
