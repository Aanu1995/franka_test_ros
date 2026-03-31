/**
 * @file secure_grasp.hpp
 * @brief Secure grasp convergence detector for force ramp termination.
 *
 * Detects when increasing grip force no longer changes the steady-state
 * tau_ext_norm using EWMA (Exponentially Weighted Moving Average) band
 * detection. Enables early termination of the force ramp.
 *
 * Computational complexity: O(1) per sample, O(1) per step finalization.
 * Zero dynamic allocation. Requires C++14 or later.
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
 * @brief Configuration for secure grasp convergence detection (EWMA band).
 */
struct SecureGraspConfig {
    double  ewma_lambda             = 0.4;
    double  ewma_band_width         = 0.08;
    int32_t n_confirm               = 2;
    double  std_threshold           = 0.14;
};

// ============================================================================
// Result
// ============================================================================

struct SecureGraspResult {
    bool    secure;
    double  d_mu;
    double  std_late;
    int32_t converge_streak;
};

// ============================================================================
// Detector
// ============================================================================

class SecureGraspDetector {
public:
    explicit SecureGraspDetector(const SecureGraspConfig& config = SecureGraspConfig{}) noexcept
        : config_{config}
    {
        reset();
    }

    void set_config(const SecureGraspConfig& config) noexcept {
        config_ = config;
        if (config_.n_confirm < 1) config_.n_confirm = 1;
        if (config_.ewma_lambda <= 0.0) config_.ewma_lambda = 0.01;
        if (config_.ewma_lambda > 1.0) config_.ewma_lambda = 1.0;
        reset();
    }

    void reset() noexcept {
        step_index_ = 0;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
        ewma_ = 0.0;
        ewma_streak_ = 0;
        secure_ = false;
    }

    void begin_step(int32_t step_index) noexcept {
        step_index_ = step_index;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
    }

    void update(double tau_ext_norm) noexcept {
        sum_ += tau_ext_norm;
        sum_sq_ += tau_ext_norm * tau_ext_norm;
        ++count_;
    }

    SecureGraspResult finalize_step() noexcept {
        if (secure_) {
            return {true, 0.0, 0.0, ewma_streak_};
        }
        if (count_ == 0) {
            return {false, 0.0, 0.0, ewma_streak_};
        }

        double mu_late = sum_ / count_;
        double variance = sum_sq_ / count_ - mu_late * mu_late;
        double std_late = std::sqrt(std::max(0.0, variance));

        double d_mu = 0.0;
        if (step_index_ == 0) {
            ewma_ = mu_late;
        } else {
            ewma_ = config_.ewma_lambda * mu_late + (1.0 - config_.ewma_lambda) * ewma_;
            d_mu = std::fabs(mu_late - ewma_);

            if (d_mu < config_.ewma_band_width && std_late < config_.std_threshold) {
                ++ewma_streak_;
            } else {
                ewma_streak_ = 0;
            }

            if (ewma_streak_ >= config_.n_confirm) {
                secure_ = true;
            }
        }

        return {secure_, d_mu, std_late, ewma_streak_};
    }

    [[nodiscard]] bool secure() const noexcept { return secure_; }
    [[nodiscard]] int32_t step_index() const noexcept { return step_index_; }
    [[nodiscard]] int32_t converge_streak() const noexcept { return ewma_streak_; }
    [[nodiscard]] const SecureGraspConfig& config() const noexcept { return config_; }

private:
    SecureGraspConfig config_;
    int32_t step_index_{0};
    double  sum_{0.0};
    double  sum_sq_{0.0};
    int32_t count_{0};
    double  ewma_{0.0};
    int32_t ewma_streak_{0};
    bool    secure_{false};
};

}  // namespace sms_cusum
