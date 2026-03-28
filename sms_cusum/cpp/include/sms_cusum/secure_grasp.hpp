/**
 * @file secure_grasp.hpp
 * @brief Secure grasp convergence detector for force ramp termination.
 *
 * Detects when increasing grip force no longer changes the steady-state
 * tau_ext_norm, indicating the object is fully compressed and the grasp
 * is secure. Enables early termination of the force ramp.
 *
 * Three detection modes are available:
 *
 *   - EWMA: Tracks exponentially weighted moving average of step means.
 *     Fires when new means stay within +/-band of the EWMA for n_confirm
 *     consecutive steps.
 *   - SLOPE: Fits linear regression to the last W step means. Fires
 *     when |slope| < threshold.
 *   - BOTH: AND-gated -- both EWMA and SLOPE must independently declare
 *     secure before the detector fires.
 *
 * Computational complexity: O(1) per sample, O(W) per step finalization.
 * Zero dynamic allocation. Requires C++14 or later.
 */

#pragma once

#include <algorithm>  // std::max, std::min
#include <array>      // std::array
#include <cmath>      // std::sqrt, std::fabs
#include <cstdint>    // int32_t, uint8_t

namespace sms_cusum {

// ============================================================================
// Mode enum
// ============================================================================

enum class SecureGraspMode : uint8_t {
    EWMA  = 0,
    SLOPE = 1,
    BOTH  = 2,
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for secure grasp convergence detection.
 *
 * @param mode               Detection mode (EWMA, SLOPE, or BOTH).
 * @param ewma_lambda        EWMA smoothing factor (0-1).
 * @param ewma_band_width    Max |mu - ewma| (Nm) for EWMA convergence.
 * @param n_confirm          Consecutive converged steps for EWMA mode.
 * @param slope_window_size  Number of step means for slope regression.
 * @param slope_threshold    Max |slope| for plateau detection.
 * @param std_threshold      Max sigma_late (Nm) for within-step stability.
 */
struct SecureGraspConfig {
    SecureGraspMode mode            = SecureGraspMode::EWMA;
    double  ewma_lambda             = 0.4;
    double  ewma_band_width         = 0.08;
    int32_t n_confirm               = 2;
    int32_t slope_window_size       = 3;
    double  slope_threshold         = 0.03;
    double  slope_max_range         = 0.15;
    double  std_threshold           = 0.14;
};

// ============================================================================
// Result
// ============================================================================

/**
 * @brief Result from SecureGraspDetector::finalize_step().
 */
struct SecureGraspResult {
    bool    secure;           ///< True if secure grasp detected.
    double  d_mu;             ///< Diagnostic: |mu-ewma| or |slope|.
    double  std_late;         ///< Std of tau_ext_norm in late segment.
    int32_t converge_streak;  ///< EWMA consecutive converged steps.
};

// ============================================================================
// Detector
// ============================================================================

/// Maximum slope window size (compile-time buffer limit).
static constexpr int32_t kMaxSlopeWindow = 8;

/**
 * @brief Detects secure grasp by tracking tau_ext_norm convergence across
 * consecutive force ramp steps.
 *
 * Real-time safe: O(1) per sample, O(W) per finalize, zero dynamic allocation.
 */
class SecureGraspDetector {
public:
    explicit SecureGraspDetector(const SecureGraspConfig& config = SecureGraspConfig{}) noexcept
        : config_{config}
    {
        reset();
    }

    /**
     * @brief Replace configuration and reset all state.
     */
    void set_config(const SecureGraspConfig& config) noexcept {
        config_ = config;
        if (config_.slope_window_size < 2) config_.slope_window_size = 2;
        if (config_.slope_window_size > kMaxSlopeWindow) config_.slope_window_size = kMaxSlopeWindow;
        if (config_.n_confirm < 1) config_.n_confirm = 1;
        if (config_.ewma_lambda <= 0.0) config_.ewma_lambda = 0.01;
        if (config_.ewma_lambda > 1.0) config_.ewma_lambda = 1.0;
        reset();
    }

    /**
     * @brief Full reset for a new grasp cycle.
     */
    void reset() noexcept {
        step_index_ = 0;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
        ewma_ = 0.0;
        ewma_streak_ = 0;
        ewma_secure_ = false;
        mu_buffer_.fill(0.0);
        buf_head_ = 0;
        buf_count_ = 0;
        slope_secure_ = false;
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
        if (secure_) {
            return {true, 0.0, 0.0, ewma_streak_};
        }

        if (count_ == 0) {
            return {false, 0.0, 0.0, ewma_streak_};
        }

        double mu_late = sum_ / count_;
        double variance = sum_sq_ / count_ - mu_late * mu_late;
        double std_late = std::sqrt(std::max(0.0, variance));

        double ewma_d_mu = 0.0;
        double slope_val = 0.0;

        auto mode = config_.mode;

        if (mode == SecureGraspMode::EWMA || mode == SecureGraspMode::BOTH) {
            ewma_d_mu = update_ewma(mu_late, std_late);
        }
        if (mode == SecureGraspMode::SLOPE || mode == SecureGraspMode::BOTH) {
            slope_val = update_slope(mu_late, std_late);
        }

        if (mode == SecureGraspMode::EWMA) {
            secure_ = ewma_secure_;
        } else if (mode == SecureGraspMode::SLOPE) {
            secure_ = slope_secure_;
        } else {  // BOTH
            secure_ = ewma_secure_ && slope_secure_;
        }

        double d_mu = 0.0;
        if (mode == SecureGraspMode::EWMA) {
            d_mu = ewma_d_mu;
        } else if (mode == SecureGraspMode::SLOPE) {
            d_mu = std::fabs(slope_val);
        } else {
            d_mu = std::max(ewma_d_mu, std::fabs(slope_val));
        }

        return {secure_, d_mu, std_late, ewma_streak_};
    }

    // Accessors
    [[nodiscard]] bool secure() const noexcept { return secure_; }
    [[nodiscard]] int32_t step_index() const noexcept { return step_index_; }
    [[nodiscard]] int32_t converge_streak() const noexcept { return ewma_streak_; }
    [[nodiscard]] const SecureGraspConfig& config() const noexcept { return config_; }

private:
    double update_ewma(double mu_late, double std_late) noexcept {
        if (ewma_secure_) return 0.0;

        if (step_index_ == 0) {
            ewma_ = mu_late;
            return 0.0;
        }

        ewma_ = config_.ewma_lambda * mu_late + (1.0 - config_.ewma_lambda) * ewma_;
        double dev = std::fabs(mu_late - ewma_);

        if (dev < config_.ewma_band_width && std_late < config_.std_threshold) {
            ++ewma_streak_;
        } else {
            ewma_streak_ = 0;
        }

        if (ewma_streak_ >= config_.n_confirm) {
            ewma_secure_ = true;
        }

        return dev;
    }

    double update_slope(double mu_late, double std_late) noexcept {
        if (slope_secure_) return 0.0;

        int32_t W = std::min(config_.slope_window_size, kMaxSlopeWindow);

        // Push into circular buffer
        mu_buffer_[buf_head_] = mu_late;
        buf_head_ = (buf_head_ + 1) % W;
        buf_count_ = std::min(buf_count_ + 1, W);

        if (buf_count_ < W || std_late >= config_.std_threshold) {
            return 0.0;
        }

        // Read buffer in chronological order and compute slope
        double x_mean = (W - 1) / 2.0;
        double y_sum = 0.0;
        for (int32_t i = 0; i < W; ++i) {
            y_sum += mu_buffer_[(buf_head_ - W + i + W) % W];
        }
        double y_mean = y_sum / W;

        double num = 0.0;
        double den = 0.0;
        for (int32_t i = 0; i < W; ++i) {
            double xi = i - x_mean;
            double yi = mu_buffer_[(buf_head_ - W + i + W) % W] - y_mean;
            num += xi * yi;
            den += xi * xi;
        }

        double slope = (den > 0.0) ? (num / den) : 0.0;

        double buf_min = mu_buffer_[(buf_head_ - W + W) % W];
        double buf_max = buf_min;
        for (int32_t i = 1; i < W; ++i) {
            double val = mu_buffer_[(buf_head_ - W + i + W) % W];
            if (val < buf_min) buf_min = val;
            if (val > buf_max) buf_max = val;
        }

        if (std::fabs(slope) < config_.slope_threshold
                && (buf_max - buf_min) < config_.slope_max_range) {
            slope_secure_ = true;
        }

        return slope;
    }

    SecureGraspConfig config_;
    int32_t step_index_{0};
    double  sum_{0.0};
    double  sum_sq_{0.0};
    int32_t count_{0};
    // EWMA state
    double  ewma_{0.0};
    int32_t ewma_streak_{0};
    bool    ewma_secure_{false};
    // Slope state
    std::array<double, kMaxSlopeWindow> mu_buffer_{};
    int32_t buf_head_{0};
    int32_t buf_count_{0};
    bool    slope_secure_{false};
    // Combined
    bool    secure_{false};
};

}  // namespace sms_cusum
