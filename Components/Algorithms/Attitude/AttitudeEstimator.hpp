/**
 ******************************************************************************
 * @file    AttitudeEstimator.hpp
 * @brief   Madgwick attitude estimation with initialisation, timing, gating,
 *          recovery, magnetometer policy and gravity compensation.
 *
 * PIPELINE POSITION
 *   ProcessedSensorFrame --attitudeInputFromFrame()--> AttitudeInput
 *     --update()--> AttitudeEstimate (quaternion + linear acceleration)
 *
 * TIMING POLICY (asynchronous sensors)
 *   * The GYROSCOPE drives the filter: a step happens only when the gyro
 *     timestamp is NEWER than the last integrated one. A republished frame
 *     (same timestamp) or an older sample is never integrated twice.
 *   * dt = (gyro timestamp - previous gyro timestamp), wrap-safe, in ms.
 *     dt > maxIntegrationDtMs (50 ms): the interval is NOT integrated (the
 *     rates inside it are unknown); the attitude is held, marked Degraded
 *     ("gap"), and must be re-confirmed by an accepted accelerometer correction
 *     or the recovery rule. No gyro for > maxGapMs (500 ms): Invalid,
 *     re-levelled from gravity when quasi-static data return.
 *   * The accelerometer and magnetometer are used at a gyro step if they are
 *     fresh (age limits); the freshest sample is used AS IS - it is not
 *     interpolated to the gyro instant (at 100 Hz the IMU accel and gyro come
 *     from the same read; the 50 Hz magnetometer can be up to ~20 ms older,
 *     which is negligible for heading at paragliding turn rates).
 *
 * Memory: one instance 424 bytes on the STM32 (measured), no heap, bounded constant work per update.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_ESTIMATOR_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_ESTIMATOR_HPP

#include <cstdint>

#include "Algorithms/Attitude/AttitudeConfig.hpp"
#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Algorithms/Attitude/GravityCompensator.hpp"
#include "Algorithms/Attitude/MadgwickFilter.hpp"

namespace pg {

class AttitudeEstimator {
public:
    explicit AttitudeEstimator(const AttitudeConfig& cfg = AttitudeConfig{});

    /// Back to Uninitialized; counters cleared.
    void reset();

    /// Processes one input (call once per processed frame). Bounded, allocation-free.
    void update(const AttitudeInput& in);

    /// The estimate produced by the last update().
    const AttitudeEstimate& latest() const { return est_; }
    const AttitudeConfig& config() const { return cfg_; }
    bool configValid() const { return cfgValid_; }

private:
    void stepInit(const AttitudeInput& in, bool gyroOk, bool accelOk);
    void stepTracking(const AttitudeInput& in, bool gyroOk, bool accelOk, bool magFresh);
    void level(uint32_t gyroTs);
    void invalidate();
    const Vec3f* decideAccel(const AttitudeInput& in, bool accelOk, Vec3f& aUnit, bool& recoveryGain);
    const Vec3f* decideMag(const AttitudeInput& in, bool magFresh, bool accelAccepted, Vec3f& mUnit, bool& recoveryGain);
    void updateQuiet(const AttitudeInput& in, bool accelOk);
    void finalize(const AttitudeInput& in, bool accelOk);
    bool tracking() const;

    AttitudeConfig cfg_;
    bool cfgValid_;
    MadgwickFilter filter_{};
    GravityCompensator gravity_;
    AttitudeEstimate est_{};
    AttitudeState state_ = AttitudeState::Uninitialized;

    // gyro time base
    uint32_t lastGyroTs_ = 0u;          ///< last integrated (or levelling) gyro timestamp

    // levelling window
    bool initActive_ = false;
    uint32_t initStartMs_ = 0u;
    uint32_t initLastMs_ = 0u;
    uint32_t initCount_ = 0u;
    double initSum_[3] = {0.0, 0.0, 0.0};
    bool hadOrientation_ = false;        ///< an orientation existed before (re-init keeps its yaw)
    float previousYaw_ = 0.0f;

    // accelerometer correction bookkeeping
    uint32_t lastAccelAcceptedMs_ = 0u;
    bool recovering_ = false;
    bool gapPending_ = false;
    bool quietActive_ = false;
    uint32_t quietSinceMs_ = 0u;

    // magnetometer bookkeeping
    bool magRefValid_ = false;
    float magRefField_uT_ = 0.0f;
    float magRefInclination_ = 0.0f;     ///< rad, negative = field points down
    uint32_t magAcquireCount_ = 0u;       ///< counts DISTINCT magnetometer samples (50 Hz vs 100 Hz gyro)
    float magAcquireField_uT_ = 0.0f;
    uint32_t magAcquireLastTs_ = 0u;
    bool everMagAccepted_ = false;
    uint32_t lastMagAcceptedMs_ = 0u;
    bool lastMagCalibrated_ = false;
    bool magDisagreeActive_ = false;
    uint32_t magDisagreeSinceMs_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_ESTIMATOR_HPP
