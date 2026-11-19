/**
 ******************************************************************************
 * @file    SafetyInput.hpp
 * @brief   The bounded input of the flight / landing safety layer, and the
 *          adapter that fills it from the real pipeline outputs
 *          (ProcessedSensorFrame + AttitudeEstimate + NavigationEstimate).
 *
 * Only the fields the three algorithms use are copied (~150 bytes instead of
 * ~1.4 KB of frame + estimates). Every source keeps ITS OWN timestamp and
 * validity: inputs are asynchronous and are never assumed simultaneous.
 * Invalid numbers stay invalid (flags), they are never replaced by 0.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_INPUT_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_INPUT_HPP

#include <cmath>
#include <cstdint>
#include <type_traits>

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Processing/ProcessedSensorFrame.hpp"

namespace pg {

struct SafetyNavInput {
    NavState state;
    NavEstimatorId estimator;
    uint32_t timestampMs;          ///< filter time of the estimate
    uint32_t sequence;
    float altitude_m;              ///< relative to the barometric reference (NOT above ground)
    float verticalVelocity_mps;    ///< positive up
    float verticalAccel_mps2;
    float altitudeSigma_m;
    float velocitySigma_mps;
    bool altitudeValid;
    bool velocityValid;
    bool accelerationValid;
    uint32_t qualityFlags;         ///< navflag::* (incl. ModelPlaceholder)
};

struct SafetyAttitudeInput {
    Quaternion q;                  ///< q_ENU_from_body
    bool orientationValid;
    AttitudeState state;
    uint32_t timestampMs;          ///< orientation time (last integrated gyro sample)
    uint32_t qualityFlags;         ///< attitude::*
    float linearAccelNorm_mps2;    ///< |a_ENU| (gravity removed), for motion detection
    bool linearAccelValid;
    uint32_t linearAccelTimestampMs;
};

struct SafetyImuInput {
    float rateNorm_radps;          ///< |angular rate| (bias-corrected body rate)
    uint32_t timestampMs;
    bool valid;
};

struct SafetyLidarInput {
    float distance_m;              ///< slant range along the beam
    uint16_t signalStrength;
    uint32_t timestampMs;
    bool valid;                    ///< accepted by preprocessing and fresh
    uint32_t lastRejectReasons;    ///< reject::* of the newest rejected sample (0 = none)
    uint32_t lastRejectTimestampMs;
};

struct SafetyGnssInput {
    float groundSpeed_mps;         ///< horizontal speed from velocityNorth/East
    float speedAccuracy_mps;
    uint32_t timestampMs;
    GnssFixType fixType;
    uint8_t numSatellites;
    bool valid;
};

struct SafetyInput {
    uint32_t nowMs;
    SafetyNavInput nav;
    SafetyAttitudeInput attitude;
    SafetyImuInput imu;
    SafetyLidarInput lidar;
    SafetyGnssInput gnss;
    uint32_t frameQualityFlags;    ///< quality::* of the frame (MountingUnverified etc.)
};

static_assert(std::is_trivially_copyable<SafetyInput>::value, "safety input must be memcpy-safe");

inline SafetyInput safetyInputFrom(const ProcessedSensorFrame& f, const AttitudeEstimate& a, const NavigationEstimate& n)
{
    SafetyInput in{};
    in.nowMs = f.timestampMs;

    in.nav.state = n.state;
    in.nav.estimator = n.estimator;
    in.nav.timestampMs = n.timestampMs;
    in.nav.sequence = n.sequence;
    in.nav.altitude_m = n.altitude_m;
    in.nav.verticalVelocity_mps = n.verticalVelocity_mps;
    in.nav.verticalAccel_mps2 = n.verticalAccel_mps2;
    in.nav.altitudeSigma_m = n.altitudeSigma_m;
    in.nav.velocitySigma_mps = n.velocitySigma_mps;
    in.nav.altitudeValid = n.altitudeValid;
    in.nav.velocityValid = n.velocityValid;
    in.nav.accelerationValid = n.accelerationValid;
    in.nav.qualityFlags = n.qualityFlags;

    in.attitude.q = a.q;
    in.attitude.orientationValid = a.orientationValid;
    in.attitude.state = a.state;
    in.attitude.timestampMs = a.timestampMs;
    in.attitude.qualityFlags = a.qualityFlags;
    const Vec3f& la = a.linearAccel.enu_mps2;
    in.attitude.linearAccelValid = a.linearAccel.valid;
    in.attitude.linearAccelNorm_mps2 = std::sqrt(la.x * la.x + la.y * la.y + la.z * la.z);
    in.attitude.linearAccelTimestampMs = a.linearAccel.timestampMs;

    const Vec3f& w = f.angularRate_radps.value;
    in.imu.valid = f.angularRate_radps.valid;
    in.imu.rateNorm_radps = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    in.imu.timestampMs = f.angularRate_radps.timestampMs;

    in.lidar.distance_m = f.lidar.distance_m;
    in.lidar.signalStrength = f.lidar.signalStrength;
    in.lidar.timestampMs = f.lidar.timestampMs;
    in.lidar.valid = f.lidar.valid;
    in.lidar.lastRejectReasons = f.lidar.lastRejectReasons;
    in.lidar.lastRejectTimestampMs = f.lidar.lastRejectTimestampMs;

    in.gnss.valid = f.gnss.valid;
    in.gnss.groundSpeed_mps = std::sqrt(f.gnss.velocityNorth_mps * f.gnss.velocityNorth_mps +
                                        f.gnss.velocityEast_mps * f.gnss.velocityEast_mps);
    in.gnss.speedAccuracy_mps = f.gnss.speedAccuracy_mps;
    in.gnss.timestampMs = f.gnss.timestampMs;
    in.gnss.fixType = f.gnss.fixType;
    in.gnss.numSatellites = f.gnss.numSatellites;

    in.frameQualityFlags = f.qualityFlags;
    return in;
}

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_INPUT_HPP
