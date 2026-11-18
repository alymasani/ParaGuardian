/**
 ******************************************************************************
 * @file    StationaryDetector.cpp
 * @brief   See StationaryDetector.hpp.
 ******************************************************************************
 */
#include "Processing/StationaryDetector.hpp"

#include <cmath>

#include "Processing/TimeBase.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

void StationaryDetector::reset()
{
    report_ = StationaryReport{};
    report_.status = StationaryStatus::InsufficientData;
    restartWindow();
}

void StationaryDetector::restartWindow()
{
    haveSample_ = false;
    gyroStats_.reset();
    accelNormStats_.reset();
    report_.samples = 0u;
    report_.elapsedMs = 0u;
}

uint32_t StationaryDetector::minSamples() const
{
    const uint32_t period = (cfg_.nominalPeriodMs == 0u) ? 1u : cfg_.nominalPeriodMs;
    const float expected = static_cast<float>(cfg_.windowMs / period);
    const uint32_t n = static_cast<uint32_t>(expected * cfg_.minSampleFraction);
    return (n < 2u) ? 2u : n;
}

void StationaryDetector::motion(MotionCause cause)
{
    report_.status = StationaryStatus::Moving;
    report_.lastCause = cause;
    if (report_.motionRestarts != UINT32_MAX) {
        ++report_.motionRestarts;
    }
    restartWindow();
}

bool StationaryDetector::windowStatisticsQuiet(MotionCause& cause)
{
    const uint32_t period = (cfg_.nominalPeriodMs == 0u) ? 1u : cfg_.nominalPeriodMs;
    // Ignore nearly empty segments (e.g. the last partial one): their mean is noisy.
    uint32_t segMin = (cfg_.segmentMs / period) / 4u;
    segMin = (segMin == 0u) ? 1u : segMin;

    const Vec3f gstd = gyroStats_.stddev();
    report_.gyroMean_radps = gyroStats_.mean();
    report_.gyroStd_radps = gstd;
    report_.accelNormMean_mps2 = static_cast<float>(accelNormStats_.mean());
    report_.accelNormStd_mps2 = static_cast<float>(accelNormStats_.stddev());
    const double sx = gyroSegX_.spread(segMin);
    const double sy = gyroSegY_.spread(segMin);
    const double sz = gyroSegZ_.spread(segMin);
    double gs = (sx > sy) ? sx : sy;
    gs = (sz > gs) ? sz : gs;
    report_.gyroSegmentSpread_radps = static_cast<float>(gs);
    const double ax = accelSegX_.spread(segMin);
    const double ay = accelSegY_.spread(segMin);
    const double az = accelSegZ_.spread(segMin);
    double as = (ax > ay) ? ax : ay;
    as = (az > as) ? az : as;
    report_.accelSegmentSpread_mps2 = static_cast<float>(as);

    if (gyroStats_.maxStddev() > cfg_.gyroStdMax_radps) {
        cause = MotionCause::GyroNoise;
        return false;
    }
    if (report_.accelNormStd_mps2 > cfg_.accelStdMax_mps2) {
        cause = MotionCause::AccelNoise;
        return false;
    }
    if (report_.gyroSegmentSpread_radps > cfg_.gyroSegmentSpreadMax_radps) {
        cause = MotionCause::GyroDrift;
        return false;
    }
    if (report_.accelSegmentSpread_mps2 > cfg_.accelSegmentSpreadMax_mps2) {
        cause = MotionCause::AccelDrift;
        return false;
    }
    return true;
}

StationaryStatus StationaryDetector::add(uint32_t timestampMs, const Vec3f& gyro, const Vec3f& accel)
{
    if (!vec::isFinite(gyro) || !vec::isFinite(accel)) {
        return report_.status;  // caller should have validated; never accumulate NaN
    }

    if (haveSample_) {
        const int32_t dt = timebase::diffMs(timestampMs, lastSampleMs_);
        if (dt <= 0) {
            return report_.status;  // not newer than the previous sample: ignore
        }
        if (static_cast<uint32_t>(dt) > cfg_.maxGapMs) {
            // We did not observe the device during the gap: start over.
            if (report_.gapRestarts != UINT32_MAX) {
                ++report_.gapRestarts;
            }
            report_.lastCause = MotionCause::DataGap;
            restartWindow();
        }
    }

    // ---- 1. instantaneous limits ----
    const float gyroMag = vec::norm(gyro);
    const float accelMag = vec::norm(accel);
    if (gyroMag > cfg_.gyroMagnitudeMax_radps) {
        motion(MotionCause::GyroMagnitude);
        return report_.status;
    }
    if (std::fabs(accelMag - units::kStandardGravity_mps2) > cfg_.accelMagnitudeTolerance_mps2) {
        motion(MotionCause::AccelMagnitude);
        return report_.status;
    }

    // ---- 2. accumulate ----
    if (!haveSample_) {
        haveSample_ = true;
        report_.windowStartMs = timestampMs;
        gyroSegX_.reset(timestampMs, cfg_.segmentMs);
        gyroSegY_.reset(timestampMs, cfg_.segmentMs);
        gyroSegZ_.reset(timestampMs, cfg_.segmentMs);
        accelSegX_.reset(timestampMs, cfg_.segmentMs);
        accelSegY_.reset(timestampMs, cfg_.segmentMs);
        accelSegZ_.reset(timestampMs, cfg_.segmentMs);
    }
    lastSampleMs_ = timestampMs;
    gyroStats_.add(gyro);
    accelNormStats_.add(accelMag);
    gyroSegX_.add(timestampMs, gyro.x);
    gyroSegY_.add(timestampMs, gyro.y);
    gyroSegZ_.add(timestampMs, gyro.z);
    accelSegX_.add(timestampMs, accel.x);
    accelSegY_.add(timestampMs, accel.y);
    accelSegZ_.add(timestampMs, accel.z);
    report_.samples = gyroStats_.count();
    report_.elapsedMs = timebase::ageMs(report_.windowStartMs, timestampMs);

    // ---- 3./4. window complete? ----
    if (report_.elapsedMs < cfg_.windowMs || report_.samples < minSamples()) {
        report_.status = StationaryStatus::InsufficientData;
        return report_.status;
    }
    MotionCause cause = MotionCause::None;
    if (!windowStatisticsQuiet(cause)) {
        motion(cause);
        return report_.status;
    }
    report_.status = StationaryStatus::Stationary;
    return report_.status;
}

}  // namespace pg
