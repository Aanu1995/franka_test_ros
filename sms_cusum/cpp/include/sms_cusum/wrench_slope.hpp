/**
 * @file wrench_slope.hpp
 * @brief Wrench slope secure grasp detector for force ramp termination.
 *
 * Detects when wrench_norm step means have converged using two criteria:
 *   1. EWMA band: |mu_late - EWMA| < band_width  (local convergence)
 *   2. Slope check: |slope of last W means| < threshold  (no trend/drift)
 *
 * Both criteria must pass for n_confirm consecutive steps before declaring
 * secure grasp. This eliminates false positives on heavy objects where the
 * signal drifts gradually but appears locally converged.
 *
 * Computational complexity: O(1) per sample, O(W) per step finalization.
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

struct WrenchSlopeConfig {
    double  ewma_lambda         = 0.4;
    double  ewma_band_width     = 0.20;
    int32_t slope_window        = 5;
    double  slope_threshold     = 0.04;
    int32_t n_confirm           = 3;
    double  std_threshold       = 0.14;
    int32_t min_slope_points    = 3;
};

// ============================================================================
// Result
// ============================================================================

struct WrenchSlopeResult {
    bool    secure;
    double  d_mu;
    double  slope;
    double  std_late;
    int32_t converge_streak;
};

// ============================================================================
// Detector
// ============================================================================

class WrenchSlopeDetector {
public:
    explicit WrenchSlopeDetector(const WrenchSlopeConfig& config = WrenchSlopeConfig{}) noexcept {
        set_config(config);
    }

    void set_config(const WrenchSlopeConfig& config) noexcept {
        config_ = config;
        if (config_.n_confirm < 1) config_.n_confirm = 1;
        if (config_.ewma_lambda <= 0.0) config_.ewma_lambda = 0.01;
        if (config_.ewma_lambda > 1.0) config_.ewma_lambda = 1.0;
        if (config_.ewma_band_width <= 0.0) config_.ewma_band_width = 0.001;
        if (config_.slope_threshold <= 0.0) config_.slope_threshold = 0.001;
        if (config_.std_threshold <= 0.0) config_.std_threshold = 0.001;
        if (config_.slope_window < 2) config_.slope_window = 2;
        if (config_.slope_window > kMaxSlopeWindow) config_.slope_window = kMaxSlopeWindow;
        if (config_.min_slope_points < 2) config_.min_slope_points = 2;
        if (config_.min_slope_points > config_.slope_window) {
            config_.min_slope_points = config_.slope_window;
        }
        reset();
    }

    void reset() noexcept {
        step_index_ = 0;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
        ewma_ = 0.0;
        converge_streak_ = 0;
        secure_ = false;
        step_means_count_ = 0;
    }

    void begin_step(int32_t step_index) noexcept {
        step_index_ = step_index;
        sum_ = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
    }

    void update(double wrench_norm) noexcept {
        sum_ += wrench_norm;
        sum_sq_ += wrench_norm * wrench_norm;
        ++count_;
    }

    WrenchSlopeResult finalize_step() noexcept {
        if (secure_) {
            return {true, 0.0, 0.0, 0.0, converge_streak_};
        }
        if (count_ == 0) {
            return {false, 0.0, 0.0, 0.0, converge_streak_};
        }

        double mu_late = sum_ / count_;
        double variance = sum_sq_ / count_ - mu_late * mu_late;
        double std_late = std::sqrt(std::max(0.0, variance));

        double d_mu = 0.0;
        double slope = 0.0;

        if (step_index_ == 0) {
            ewma_ = mu_late;
            push_step_mean(mu_late);
        } else {
            d_mu = std::fabs(mu_late - ewma_);
            ewma_ = config_.ewma_lambda * mu_late + (1.0 - config_.ewma_lambda) * ewma_;

            push_step_mean(mu_late);

            // EWMA band check
            bool ewma_ok = (d_mu < config_.ewma_band_width) && (std_late < config_.std_threshold);

            // Slope check
            bool slope_ok = true;
            if (step_means_count_ >= config_.min_slope_points) {
                slope = compute_slope();
                slope_ok = std::fabs(slope) < config_.slope_threshold;
            }

            if (ewma_ok && slope_ok) {
                ++converge_streak_;
            } else {
                converge_streak_ = 0;
            }

            if (converge_streak_ >= config_.n_confirm) {
                secure_ = true;
            }
        }

        return {secure_, d_mu, slope, std_late, converge_streak_};
    }

    [[nodiscard]] bool secure() const noexcept { return secure_; }
    [[nodiscard]] int32_t step_index() const noexcept { return step_index_; }
    [[nodiscard]] int32_t converge_streak() const noexcept { return converge_streak_; }
    [[nodiscard]] const WrenchSlopeConfig& config() const noexcept { return config_; }

private:
    static const int32_t kMaxSlopeWindow = 16;

    void push_step_mean(double value) noexcept {
        if (step_means_count_ < kMaxSlopeWindow) {
            step_means_[step_means_count_] = value;
            ++step_means_count_;
        } else {
            // Shift left by 1 to make room
            for (int32_t i = 0; i < kMaxSlopeWindow - 1; ++i) {
                step_means_[i] = step_means_[i + 1];
            }
            step_means_[kMaxSlopeWindow - 1] = value;
        }
    }

    double compute_slope() const noexcept {
        int32_t n = std::min(step_means_count_, config_.slope_window);
        if (n < 2) return 0.0;

        // Use the last n entries
        int32_t start = step_means_count_ - n;
        double x_mean = (n - 1) / 2.0;
        double y_sum = 0.0;
        for (int32_t i = 0; i < n; ++i) {
            y_sum += step_means_[start + i];
        }
        double y_mean = y_sum / n;

        double num = 0.0;
        double den = 0.0;
        for (int32_t i = 0; i < n; ++i) {
            double dx = i - x_mean;
            num += dx * (step_means_[start + i] - y_mean);
            den += dx * dx;
        }
        return (den > 0.0) ? (num / den) : 0.0;
    }

    WrenchSlopeConfig config_;
    int32_t step_index_{0};
    double  sum_{0.0};
    double  sum_sq_{0.0};
    int32_t count_{0};
    double  ewma_{0.0};
    int32_t converge_streak_{0};
    bool    secure_{false};
    double  step_means_[kMaxSlopeWindow]{};
    int32_t step_means_count_{0};
};

}  // namespace sms_cusum
