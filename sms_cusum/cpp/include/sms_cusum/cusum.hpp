/**
 * @file cusum.hpp
 * @brief CUSUM (Cumulative Sum) change-point detector with noise-adaptive allowance.
 *
 * Implements the one-sided downward CUSUM for detecting negative mean shifts,
 * as required for contact detection (torque drop) in robotic grasping.
 *
 * The classical CUSUM statistic (Page, 1954):
 *
 *     S_n = max(0, S_{n-1} + (mu_0 - x_n) - k)
 *
 * fires an alarm when S_n >= h. This implementation adds:
 *   - Debounce: alarm requires S_n >= h for `debounce_count` consecutive samples.
 *   - Noise-adaptive allowance: k_eff = max(k_min, noise_multiplier * sigma).
 *
 * Computational complexity: O(1) per sample, zero dynamic allocation.
 *
 * References:
 *     Page, E.S. (1954). "Continuous inspection schemes."
 *         Biometrika, 41(1-2), 100-115. DOI: 10.1093/biomet/41.1-2.100
 *     Moustakides, G.V. (1986). "Optimal stopping times for detecting
 *         changes in distributions." Ann. Statist., 14(4), 1379-1387.
 *         DOI: 10.1214/aos/1176350164
 */

#pragma once

#include <algorithm>  // std::max
#include <cstdint>    // int32_t

namespace sms_cusum {

/**
 * @brief Configuration for a single CUSUM detection stage.
 *
 * @param k_min           Minimum allowance parameter. The effective allowance is
 *                        k_eff = max(k_min, noise_multiplier * sigma), ensuring
 *                        sensitivity even when measured noise is very low.
 * @param h               Decision threshold for the CUSUM statistic. Alarm fires
 *                        when S_n >= h for debounce_count consecutive samples.
 *                        Larger h = fewer false positives, more detection delay.
 * @param debounce_count  Number of consecutive samples where S_n >= h before
 *                        declaring a state transition. At 250 Hz, 10 samples = 40 ms.
 * @param noise_multiplier Scaling factor for adaptive allowance:
 *                        k_eff = max(k_min, noise_multiplier * sigma).
 *                        Typical range 1.5-4.0. Higher = more robust to noise,
 *                        but slower to detect subtle shifts.
 */
struct CusumStageConfig {
    double k_min           = 0.02;
    double h               = 0.3;
    int32_t debounce_count = 5;
    double noise_multiplier = 2.0;
};

/**
 * @brief One-sided downward CUSUM detector with noise-adaptive allowance.
 *
 * Detects sustained negative mean shifts (drops) in a signal relative
 * to a provided reference mean mu_0. The CUSUM statistic accumulates
 * evidence of a downward shift:
 *
 *     S_n = max(0, S_{n-1} + (mu_0 - x_n) - k_eff)
 *
 * An alarm fires when S_n >= h for at least `debounce_count` consecutive
 * samples, providing robustness against transient noise spikes.
 *
 * The noise-adaptive allowance k_eff = max(k_min, noise_multiplier * sigma)
 * automatically scales sensitivity to the measured noise level, enabling
 * detection across objects of varying properties without manual tuning.
 *
 * Real-time safe: O(1) per sample, zero dynamic allocation, all methods noexcept.
 */
class CUSUMDetector {
public:
    /**
     * @brief Construct a CUSUM detector from stage configuration parameters.
     *
     * @param k_min           Minimum allowance parameter.
     * @param h               Decision threshold for the CUSUM statistic.
     * @param debounce_count  Consecutive alarm samples required before detection.
     * @param noise_multiplier Scaling factor for adaptive allowance.
     */
    explicit CUSUMDetector(
        double k_min           = 0.02,
        double h               = 0.3,
        int32_t debounce_count = 5,
        double noise_multiplier = 2.0
    ) noexcept
        : config_{k_min, h, debounce_count, noise_multiplier}
        , S_{0.0}
        , alarm_streak_{0}
        , k_eff_{k_min}
    {}

    /**
     * @brief Construct a CUSUM detector from a CusumStageConfig struct.
     *
     * @param config Stage configuration.
     */
    explicit CUSUMDetector(const CusumStageConfig& config) noexcept
        : config_{config}
        , S_{0.0}
        , alarm_streak_{0}
        , k_eff_{config.k_min}
    {}

    /**
     * @brief Reset CUSUM state for a new detection cycle.
     *
     * Clears the statistic S_n and alarm streak counter but preserves
     * configuration and the current effective allowance.
     */
    void reset() noexcept {
        S_ = 0.0;
        alarm_streak_ = 0;
    }

    /**
     * @brief Update the effective allowance based on measured noise sigma.
     *
     * Called once when entering a detection stage, using the noise
     * estimate from the preceding baseline/state.
     *
     * @param sigma Estimated standard deviation of the signal noise.
     */
    void adapt(double sigma) noexcept {
        k_eff_ = std::max(config_.k_min, config_.noise_multiplier * sigma);
    }

    /**
     * @brief Process one sample. O(1) time, zero allocation.
     *
     * @param mu_0 Reference mean (baseline level from preceding state).
     * @param x    Current observation (e.g., tau_ext_norm).
     * @return true if change-point detected (alarm), false otherwise.
     */
    bool update(double mu_0, double x) noexcept {
        S_ = std::max(0.0, S_ + (mu_0 - x) - k_eff_);

        if (S_ >= config_.h) {
            ++alarm_streak_;
            return alarm_streak_ >= config_.debounce_count;
        } else {
            alarm_streak_ = 0;
            return false;
        }
    }

    /** @brief Current CUSUM statistic S_n (for diagnostics/plotting). */
    [[nodiscard]] double statistic() const noexcept { return S_; }

    /** @brief Current consecutive alarm count. */
    [[nodiscard]] int32_t alarm_streak() const noexcept { return alarm_streak_; }

    /** @brief Current effective allowance parameter. */
    [[nodiscard]] double k_effective() const noexcept { return k_eff_; }

    /** @brief Stage configuration (read-only). */
    [[nodiscard]] const CusumStageConfig& config() const noexcept { return config_; }

private:
    CusumStageConfig config_;
    double           S_;
    int32_t          alarm_streak_;
    double           k_eff_;
};

}  // namespace sms_cusum
