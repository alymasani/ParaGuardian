/**
 ******************************************************************************
 * @file    RunningStats.hpp
 * @brief   Streaming (O(1) memory) statistics used by the stationary detector
 *          and the calibrators: Welford mean/variance and per-segment means.
 *
 * WHY WELFORD?
 *   The naive formula var = E[x^2] - E[x]^2 subtracts two large, nearly equal
 *   numbers. For a barometer (x ~ 101325 Pa, spread ~ 1 Pa) in float that
 *   cancellation destroys every significant digit of the variance. Welford's
 *   update keeps a running mean and the sum of squared deviations from it:
 *       n    = n + 1
 *       d    = x - mean
 *       mean = mean + d / n
 *       M2   = M2 + d * (x - mean)        (uses the NEW mean)
 *       variance = M2 / (n - 1)           (sample variance)
 *   which only ever works with deviations. Accumulators are double: the
 *   STM32H743 FPU (fpv5-d16) executes double add/mul/div in hardware.
 *
 * WHY SEGMENT MEANS?
 *   A slow, steady change (a turning wearer, a warming gyro, a module carried
 *   upstairs) can have a small standard deviation inside a 5 s window but a
 *   clear trend. Splitting the window into 1 s segments and comparing the
 *   largest and smallest segment mean detects the trend with 8 small slots
 *   instead of storing 500 samples.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_RUNNING_STATS_HPP
#define PARAGUARDIAN_PROCESSING_RUNNING_STATS_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "Processing/TimeBase.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

class RunningStats {
public:
    void reset()
    {
        n_ = 0u;
        mean_ = 0.0;
        m2_ = 0.0;
    }
    void add(double x)
    {
        ++n_;
        const double d = x - mean_;
        mean_ += d / static_cast<double>(n_);
        m2_ += d * (x - mean_);
    }
    uint32_t count() const { return n_; }
    double mean() const { return mean_; }
    /// Sample variance (n - 1); 0 for fewer than 2 samples.
    double variance() const { return (n_ > 1u) ? m2_ / static_cast<double>(n_ - 1u) : 0.0; }
    double stddev() const { return std::sqrt(variance()); }

private:
    uint32_t n_ = 0u;
    double mean_ = 0.0;
    double m2_ = 0.0;
};

class RunningStats3 {
public:
    void reset()
    {
        x_.reset();
        y_.reset();
        z_.reset();
    }
    void add(const Vec3f& v)
    {
        x_.add(v.x);
        y_.add(v.y);
        z_.add(v.z);
    }
    uint32_t count() const { return x_.count(); }
    Vec3f mean() const
    {
        return Vec3f{static_cast<float>(x_.mean()), static_cast<float>(y_.mean()), static_cast<float>(z_.mean())};
    }
    Vec3f stddev() const
    {
        return Vec3f{static_cast<float>(x_.stddev()), static_cast<float>(y_.stddev()),
                     static_cast<float>(z_.stddev())};
    }
    float maxStddev() const
    {
        const Vec3f s = stddev();
        float m = s.x;
        m = (s.y > m) ? s.y : m;
        return (s.z > m) ? s.z : m;
    }

private:
    RunningStats x_, y_, z_;
};

/**
 * Mean of a scalar per fixed-length time segment, since a start time.
 * Segment k covers [start + k*segmentMs, start + (k+1)*segmentMs). Samples
 * past the last slot are added to the last slot (so a window longer than
 * kMaxSegments * segmentMs still works, with a coarser final segment).
 */
class SegmentMeans {
public:
    static constexpr size_t kMaxSegments = 8u;

    void reset(uint32_t startMs, uint32_t segmentMs)
    {
        startMs_ = startMs;
        segmentMs_ = (segmentMs == 0u) ? 1u : segmentMs;
        for (size_t i = 0; i < kMaxSegments; ++i) {
            sum_[i] = 0.0;
            count_[i] = 0u;
        }
    }
    void add(uint32_t tMs, double v)
    {
        const uint32_t elapsed = timebase::ageMs(startMs_, tMs);
        size_t k = static_cast<size_t>(elapsed / segmentMs_);
        if (k >= kMaxSegments) {
            k = kMaxSegments - 1u;
        }
        sum_[k] += v;
        ++count_[k];
    }
    /// Largest minus smallest segment mean over segments that contain at least @p minCount samples.
    double spread(uint32_t minCount = 1u) const
    {
        bool any = false;
        double lo = 0.0;
        double hi = 0.0;
        for (size_t i = 0; i < kMaxSegments; ++i) {
            if (count_[i] < minCount || count_[i] == 0u) {
                continue;
            }
            const double m = sum_[i] / static_cast<double>(count_[i]);
            if (!any) {
                lo = hi = m;
                any = true;
            } else {
                lo = (m < lo) ? m : lo;
                hi = (m > hi) ? m : hi;
            }
        }
        return any ? (hi - lo) : 0.0;
    }

private:
    uint32_t startMs_ = 0u;
    uint32_t segmentMs_ = 1000u;
    double sum_[kMaxSegments] = {};
    uint32_t count_[kMaxSegments] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_RUNNING_STATS_HPP
