/**
 * @file secure_grasp.hpp
 * @brief Secure grasp convergence detector for force ramp termination.
 *
 * Detects when increasing grip force no longer changes the steady-state
 * tau_ext_norm, indicating the object is fully compressed and the grasp
 * is secure. Enables early termination of the force ramp.
 *
 * Detection criteria (AND-gated):
 *     1. Mean convergence: |mu_late[N] - mu_late[N-1]| < threshold
 *     2. Signal stability: sigma_late[N] < threshold
 *     3. Consecutive confirmation: criteria hold for n_confirm steps
 *
 * Computational complexity: O(1) per sample, zero dynamic allocation.
 * Requires C++14 or later.
 */

#pragma once

#include <algorithm>  // std::max
#include <cmath>      // std::sqrt, std::fabs
#include <cstdint>    // int32_t

namespace sms_cusum {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for secure grasp convergence detection.
 *
 * @param mean_converge_threshold  Max |d_mu| (Nm) for convergence.
 * @param std_threshold            Max sigma_late (Nm) for stability.
 * @param n_confirm                Consecutive converged steps required.
 */
struct SecureGraspConfig {
    double  mean_converge_threshold = 0.03;
    double  std_threshold           = 0.08;
    int32_t n_confirm               = 2;
};

// ============================================================================
// Result
// ============================================================================

/**
 * @brief Result from SecureGraspDetector::finalize_step().
 */
struct SecureGraspResult {
    bool    secure;           ///< True if secure grasp detected.
    double  d_mu;             ///< |mu_late[N] - mu_late[N-1]|.
    double  std_late;         ///< Std of tau_ext_norm in late segment.
    int32_t converge_streak;  ///< Current consecutive converged steps.
};

// ============================================================================
// Detector
// ============================================================================

/**
 * @brief Detects secure grasp by tracking tau_ext_norm convergence across
 * consecutive force ramp steps.
 *
 * Real-time safe: O(1) per sample, zero dynamic allocation, all methods noexcept.
 */
class SecureGraspDetector {
public:
    explicit SecureGraspDetector(const SecureGraspConfig& config = SecureGraspConfig{}) noexcept
        : config_{config}
        , step_index_{0}
        , sum_{0.0}
        , sum_sq_{0.0}
        , count_{0}
        , prev_mu_late_{0.0}
        , converge_streak_{0}
        , secure_{false}
    {}

    /**
     * @brief Full reset for a new grasp cycle.
     */
    void reset() noexcept {
        step_index_ = 0;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
        prev_mu_late_ = 0.0;
        converge_streak_ = 0;
        secure_ = false;
    }

    /**
     * @brief Signal start of a new GRASP step's HOLDING phase.
     * @param step_index Zero-based step index.
     */
    void begin_step(int32_t step_index) noexcept {
        step_index_ = step_index;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
    }

    /**
     * @brief Accumulate one sample during the late segment of HOLDING.
     * @param tau_ext_norm External torque norm sample.
     */
    void update(double tau_ext_norm) noexcept {
        sum_ += tau_ext_norm;
        sum_sq_ += tau_ext_norm * tau_ext_norm;
        ++count_;
    }

    /**
     * @brief Finalize the current step and check for secure grasp.
     * @return SecureGraspResult with detection status and diagnostics.
     */
    SecureGraspResult finalize_step() noexcept {
        SecureGraspResult result;

        if (secure_) {
            result.secure = true;
            result.d_mu = 0.0;
            result.std_late = 0.0;
            result.converge_streak = converge_streak_;
            return result;
        }

        if (count_ == 0) {
            result.secure = false;
            result.d_mu = 0.0;
            result.std_late = 0.0;
            result.converge_streak = converge_streak_;
            return result;
        }

        double mu_late = sum_ / count_;
        double variance = sum_sq_ / count_ - mu_late * mu_late;
        double std_late = std::sqrt(std::max(0.0, variance));

        double d_mu = 0.0;
        if (step_index_ > 0) {  // step 0 has no previous mu to compare
            d_mu = std::fabs(mu_late - prev_mu_late_);

            bool primary_ok = d_mu < config_.mean_converge_threshold;
            bool secondary_ok = std_late < config_.std_threshold;

            if (primary_ok && secondary_ok) {
                ++converge_streak_;
            } else {
                converge_streak_ = 0;
            }

            if (converge_streak_ >= config_.n_confirm) {
                secure_ = true;
            }
        }

        prev_mu_late_ = mu_late;

        result.secure = secure_;
        result.d_mu = d_mu;
        result.std_late = std_late;
        result.converge_streak = converge_streak_;
        return result;
    }

    // Accessors
    [[nodiscard]] bool secure() const noexcept { return secure_; }
    [[nodiscard]] int32_t step_index() const noexcept { return step_index_; }
    [[nodiscard]] int32_t converge_streak() const noexcept { return converge_streak_; }
    [[nodiscard]] const SecureGraspConfig& config() const noexcept { return config_; }

private:
    SecureGraspConfig config_;
    int32_t step_index_;
    double  sum_;
    double  sum_sq_;
    int32_t count_;
    double  prev_mu_late_;
    int32_t converge_streak_;
    bool    secure_;
};

}  // namespace sms_cusum
