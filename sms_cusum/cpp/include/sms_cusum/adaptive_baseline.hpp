/**
 * @file adaptive_baseline.hpp
 * @brief Adaptive baseline estimator with lifecycle management for SMS-CUSUM.
 *
 * Provides a running estimate of signal mean (mu) and standard deviation (sigma)
 * with three operational phases:
 *
 *   COLLECTING : Accumulating initial samples for a robust starting estimate.
 *   TRACKING   : Updating mu via Exponential Moving Average (EMA) during free-motion.
 *   FROZEN     : No updates; detector is in active detection mode.
 *
 * The lifecycle supports context inheritance: when a CUSUM stage fires an alarm,
 * the baseline can be frozen, its (mu, sigma) passed as the reference for the
 * next stage, and later unfrozen when returning to free motion.
 *
 * Computational complexity: O(1) per sample, zero dynamic allocation.
 */

#pragma once

#include <algorithm>  // std::max
#include <cmath>      // std::sqrt
#include <cstdint>    // int32_t
#include <utility>    // std::pair

namespace sms_cusum {

/**
 * @brief Operational phase of the adaptive baseline estimator.
 */
enum class BaselinePhase : uint8_t {
    COLLECTING = 0,  ///< Accumulating initial samples for robust estimate.
    TRACKING   = 1,  ///< EMA updates to mu during free-motion.
    FROZEN     = 2   ///< No updates; in active detection mode.
};

/**
 * @brief Adaptive baseline estimator using EMA with lifecycle management.
 *
 * During COLLECTING, accumulates `init_samples` observations to compute
 * the initial mean and standard deviation.  Once ready, transitions to
 * TRACKING and applies EMA updates:
 *
 *     mu_{n+1} = alpha * x_n + (1 - alpha) * mu_n
 *
 * Can be FROZEN to prevent baseline corruption during active detection,
 * and UNFROZEN to resume tracking.
 *
 * Real-time safe: O(1) per sample, zero dynamic allocation, all methods noexcept.
 */
class AdaptiveBaseline {
public:
    /**
     * @brief Construct an adaptive baseline estimator.
     *
     * @param init_samples Number of samples to collect before baseline is valid.
     *                     Default 50 (0.2 s at 250 Hz).
     * @param alpha        EMA smoothing factor. Smaller = smoother, slower adaptation.
     *                     Default 0.01 (time constant ~100 samples = 0.4 s at 250 Hz).
     */
    explicit AdaptiveBaseline(int32_t init_samples = 50, double alpha = 0.01) noexcept
        : init_samples_{init_samples < 2 ? 2 : init_samples}
        , alpha_{(alpha <= 0.0) ? 0.001 : (alpha > 1.0 ? 1.0 : alpha)}
        , sum_{0.0}
        , sum_sq_{0.0}
        , count_{0}
        , mu_{0.0}
        , sigma_{0.0}
        , phase_{BaselinePhase::COLLECTING}
    {}

    /**
     * @brief True once initial collection is complete and mu/sigma are valid.
     */
    [[nodiscard]] bool ready() const noexcept {
        return phase_ != BaselinePhase::COLLECTING;
    }

    /** @brief Current estimated mean. */
    [[nodiscard]] double mean() const noexcept { return mu_; }

    /** @brief Estimated noise standard deviation (from initial collection). */
    [[nodiscard]] double sigma() const noexcept { return sigma_; }

    /** @brief Current operational phase. */
    [[nodiscard]] BaselinePhase phase() const noexcept { return phase_; }

    /** @brief Number of samples processed (during collection) or total since ready. */
    [[nodiscard]] int32_t count() const noexcept { return count_; }

    /**
     * @brief Stop updating baseline (entering detection mode).
     *
     * The current (mu, sigma) are preserved for use as CUSUM reference.
     */
    void freeze() noexcept {
        phase_ = BaselinePhase::FROZEN;
    }

    /**
     * @brief Resume EMA tracking (returning to free-motion).
     *
     * The baseline will adapt to the current signal level using EMA.
     * Only transitions from FROZEN; no-op in other phases.
     */
    void unfreeze() noexcept {
        if (phase_ == BaselinePhase::FROZEN) {
            phase_ = BaselinePhase::TRACKING;
        }
    }

    /**
     * @brief Full reset for a new trial. Returns to COLLECTING phase.
     */
    void reset() noexcept {
        sum_   = 0.0;
        sum_sq_ = 0.0;
        count_ = 0;
        mu_    = 0.0;
        sigma_ = 0.0;
        phase_ = BaselinePhase::COLLECTING;
    }

    /**
     * @brief Process one sample. O(1) time, zero allocation.
     *
     * Behavior depends on phase:
     *   COLLECTING: accumulates sum/sum_sq, transitions to TRACKING when ready.
     *   TRACKING:   applies EMA update to mu.
     *   FROZEN:     no-op.
     *
     * @param x Current observation (e.g., tau_ext_norm).
     */
    void update(double x) noexcept {
        if (phase_ == BaselinePhase::FROZEN) {
            return;
        }

        if (phase_ == BaselinePhase::COLLECTING) {
            sum_   += x;
            sum_sq_ += x * x;
            ++count_;
            if (count_ >= init_samples_) {
                mu_ = sum_ / static_cast<double>(count_);
                double variance = (sum_sq_ / static_cast<double>(count_)) - (mu_ * mu_);
                sigma_ = std::sqrt(std::max(variance, 0.0));
                phase_ = BaselinePhase::TRACKING;
            }
        } else {
            // TRACKING: EMA update of mean and sigma
            mu_ = alpha_ * x + (1.0 - alpha_) * mu_;
            double diff = x - mu_;
            double new_var = alpha_ * (diff * diff) + (1.0 - alpha_) * (sigma_ * sigma_);
            sigma_ = std::sqrt(std::max(new_var, 0.0));
            ++count_;
        }
    }

    /**
     * @brief Return current (mean, sigma) for context inheritance.
     *
     * Use this to pass the baseline statistics to the next CUSUM stage
     * when a state transition is detected.
     *
     * @return std::pair<double, double> (mean, sigma) of the current baseline estimate.
     */
    [[nodiscard]] std::pair<double, double> snapshot() const noexcept {
        return {mu_, sigma_};
    }

private:
    int32_t       init_samples_;
    double        alpha_;
    double        sum_;
    double        sum_sq_;
    int32_t       count_;
    double        mu_;
    double        sigma_;
    BaselinePhase phase_;
};

}  // namespace sms_cusum
