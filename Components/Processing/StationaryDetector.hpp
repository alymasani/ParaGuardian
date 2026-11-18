/**
 ******************************************************************************
 * @file    StationaryDetector.hpp
 * @brief   Decides whether ONE IMU has been continuously still for a full
 *          observation window (start-up calibration gate).
 *
 * A window is accepted only if ALL of these hold:
 *   1. every sample in it passed the instantaneous limits
 *        |w| <= gyroMagnitudeMax           (no clear rotation)
 *        | |a| - g | <= accelTolerance       (only gravity is sensed)
 *   2. no gap between consecutive samples exceeded maxGapMs
 *   3. it spans at least windowMs and contains >= minSamples samples
 *   4. its statistics are quiet:
 *        per-axis gyro standard deviation   <= gyroStdMax
 *        standard deviation of |a|          <= accelStdMax
 *        spread of 1 s segment means, gyro per axis   <= gyro drift limit
 *        spread of 1 s segment means, accel PER AXIS   <= accel drift limit
 *   The per-axis accelerometer drift test is what catches a SLOW CONSTANT
 *   ROTATION: such a rotation passes every gyro test (a constant rate looks
 *   exactly like a bias) and leaves |a| = g, but gravity's DIRECTION in the
 *   sensor axes turns, so the accel components change (0.05 rad/s for 5 s
 *   moves them by ~2.4 m/s^2; even 0.002 rad/s gives ~0.1 m/s^2).
 *
 * Any violation RESTARTS the window at the next sample. So a short
 * motionless moment inside an otherwise moving period can never be accepted:
 * the window must be still from its first to its last sample.
 *
 * Feed only samples the SampleValidator accepted with BOTH channels usable,
 * in the IMU's own (sensor) axes - stillness does not depend on orientation.
 * The altitude/pressure is NOT used: a constant altitude does not mean the
 * wearer is still.
 *
 * Memory: 4 Welford accumulators + 6 segment tables ~ 0.7 KB. O(1) per sample.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_STATIONARY_DETECTOR_HPP
#define PARAGUARDIAN_PROCESSING_STATIONARY_DETECTOR_HPP

#include <cstdint>

#include "Processing/ProcessingConfig.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Processing/RunningStats.hpp"

namespace pg {

/// Why the last window was restarted.
enum class MotionCause : uint8_t {
    None = 0,
    GyroMagnitude,     ///< instantaneous |w| too large
    AccelMagnitude,    ///< instantaneous | |a| - g | too large
    GyroNoise,         ///< window gyro std too large
    AccelNoise,        ///< window |a| std too large
    GyroDrift,         ///< gyro segment means moved
    AccelDrift,        ///< accel per-axis segment means moved (rotation or translation)
    DataGap,           ///< samples missing for longer than maxGapMs
};

struct StationaryReport {
    StationaryStatus status;
    MotionCause lastCause;
    uint32_t windowStartMs;
    uint32_t elapsedMs;
    uint32_t samples;
    Vec3f gyroMean_radps;
    Vec3f gyroStd_radps;
    float accelNormMean_mps2;
    float accelNormStd_mps2;
    float gyroSegmentSpread_radps;   ///< largest per-axis spread
    float accelSegmentSpread_mps2;   ///< largest per-axis spread
    uint32_t motionRestarts;         ///< windows restarted because of motion
    uint32_t gapRestarts;            ///< windows restarted because of missing data
};

class StationaryDetector {
public:
    explicit StationaryDetector(const StationaryConfig& cfg) : cfg_(cfg) { reset(); }

    /// Clears everything, including the restart counters.
    void reset();
    /// Starts a fresh window at the next sample (keeps the counters).
    void restartWindow();

    /**
     * Adds one validated sample (sensor axes).
     * @return the status after this sample.
     */
    StationaryStatus add(uint32_t timestampMs, const Vec3f& gyro_radps, const Vec3f& accel_mps2);

    StationaryStatus status() const { return report_.status; }
    const StationaryReport& report() const { return report_; }
    /// Minimum samples a window must hold (windowMs / nominalPeriodMs * minSampleFraction).
    uint32_t minSamples() const;
    const StationaryConfig& config() const { return cfg_; }

private:
    void motion(MotionCause cause);
    bool windowStatisticsQuiet(MotionCause& cause);

    StationaryConfig cfg_;
    StationaryReport report_{};
    bool haveSample_ = false;
    uint32_t lastSampleMs_ = 0u;
    RunningStats3 gyroStats_{};
    RunningStats accelNormStats_{};
    SegmentMeans gyroSegX_{}, gyroSegY_{}, gyroSegZ_{};
    SegmentMeans accelSegX_{}, accelSegY_{}, accelSegZ_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_STATIONARY_DETECTOR_HPP
