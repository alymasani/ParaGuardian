/**
 ******************************************************************************
 * @file    GroundClearanceTypes.hpp
 * @brief   Output and configuration of the LiDAR GroundClearanceEstimator.
 *
 * WHAT "CLEARANCE" MEANS HERE
 *   The TF02-Pro measures a SLANT range r along its beam. With the beam
 *   direction d (unit vector) rotated into ENU by the attitude quaternion,
 *       clearance = -r * d_ENU.z
 *   is the vertical distance from the sensor down to the ILLUMINATED POINT,
 *   which equals the height above ground beneath the pilot ONLY if the
 *   terrain is locally horizontal. On a slope, or with a step / building /
 *   vegetation, the beam hits terrain ahead of or beside the pilot: the
 *   number is then the height above THAT point. The estimator always sets
 *   clearflag::FlatTerrainAssumed to say so.
 *
 *   This is NOT the UKF altitude (relative to the barometric reference) and
 *   NOT GNSS altitude (sea level). The three are never interchanged.
 *
 * MOUNTING: the beam direction in body axes is configuration. The default
 * (body +z = down when level, FRD) is a PLACEHOLDER: no hardware exists, so
 * every output carries clearflag::MountingUnverified until mountingVerified
 * is set from a real check (HARDWARE_INTEGRATION_TODO.md section 15).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_TYPES_HPP

#include <cstdint>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Why a clearance is or is not available (one value; modifiers are in clearflag).
enum class ClearanceStatus : uint8_t {
    Valid = 0,
    NoLidar,            ///< no LiDAR sample has ever been accepted or rejected
    Stale,              ///< newest sample older than maxAgeMs (dropout)
    BeyondRange,        ///< recent samples report no return within range: clearance > lowerBound_m (if geometry OK)
    WeakSignal,         ///< recent samples rejected for weak / saturated signal: no number
    InvalidMeasurement, ///< other rejection (non-finite, timestamp, ...)
    AttitudeInvalid,    ///< no valid orientation (or orientation lost after a gyro gap)
    AttitudeStale,      ///< attitude and LiDAR times too far apart to project
    BeamGeometry,       ///< beam more than maxBeamTiltDeg from straight down (bank / pitch / mounting)
};

namespace clearflag {
enum : uint32_t {
    MountingUnverified  = 1u << 0,   ///< beam direction is a placeholder (always, until verified)
    FlatTerrainAssumed  = 1u << 1,   ///< clearance = height above the illuminated point (always set when valid)
    AttitudeDegraded    = 1u << 2,   ///< attitude Degraded / Recovering: larger tilt uncertainty
    StepChange          = 1u << 3,   ///< this sample jumped vs the prediction (terrain step, obstacle, or outlier)
    NotNew              = 1u << 4,   ///< no new LiDAR sample this update (output carried, ageing)
    RateReset           = 1u << 5,   ///< closure-rate history restarted this update
    HighTilt            = 1u << 6,   ///< valid, but tilt above highTiltDeg (beam hits terrain far off)
    ImuMountingUnverified = 1u << 7, ///< inherited: IMU mountings (hence attitude axes) unverified
};
}  // namespace clearflag

struct GroundClearanceConfig {
    Vec3f beamBody{0.0f, 0.0f, 1.0f};   ///< beam direction in body FRD (normalised internally); PLACEHOLDER
    bool mountingVerified = false;
    float minRange_m = 0.1f;            ///< TF02-Pro manual: 0.1-40 m (90 % reflectivity)
    float maxRange_m = 40.0f;           ///< must match Tf02Config::maxRange_m (driver's out-of-range limit)
    float maxBeamTiltDeg = 40.0f;       ///< beyond this the illuminated point is > 0.84 r away horizontally
    float highTiltDeg = 20.0f;
    uint32_t maxAgeMs = 60u;            ///< LiDAR sample older than this at processing time: Stale
    uint32_t maxAttitudeSkewMs = 40u;   ///< |t_attitude - t_lidar| above: AttitudeStale
    uint32_t rejectHoldMs = 300u;       ///< a rejection this recent explains an invalid LiDAR (BeyondRange / WeakSignal)
    // uncertainty (1 sigma) - TF02-Pro manual: +-6 cm below 6 m, 1 % above
    float rangeSigmaNear_m = 0.06f;
    float rangeSigmaFraction = 0.01f;
    float attitudeSigmaDeg = 2.0f;      ///< tilt uncertainty with attitude Tracking (simulation-based)
    float degradedAttitudeSigmaDeg = 6.0f;
    float mountingSigmaDeg = 5.0f;      ///< added while the mounting is unverified
    // discontinuities (kept as valid data, flagged, rate history restarted)
    float stepThreshold_m = 1.5f;       ///< 25 sigma of the near-range noise (6 cm)
    float stepFraction = 0.10f;         ///< step if |jump| > max(stepThreshold_m, stepFraction * clearance): 10 sigma of the 1 % far noise
    // closure rate: least-squares slope over a short window
    uint32_t rateWindowMs = 400u;
    uint32_t rateMinSpanMs = 200u;
    uint32_t rateMinSamples = 6u;
    float rateMaxResidual_m = 0.5f;     ///< RMS fit residual above: rate invalid (spiky / curved)
    // time to contact
    float ttcMinClosure_mps = 0.3f;     ///< closure below this: no finite TTC
    float ttcMax_s = 60.0f;             ///< longer TTCs are reported invalid (meaningless)

    bool isValid() const
    {
        const float n2 = beamBody.x * beamBody.x + beamBody.y * beamBody.y + beamBody.z * beamBody.z;
        return n2 > 0.25f && maxRange_m > minRange_m && minRange_m >= 0.0f && maxBeamTiltDeg > 0.0f &&
               maxBeamTiltDeg < 80.0f && highTiltDeg <= maxBeamTiltDeg && maxAgeMs > 0u && rateMinSamples >= 3u &&
               rateMinSpanMs > 0u && rateWindowMs >= rateMinSpanMs && ttcMinClosure_mps > 0.0f;
    }

    static constexpr uint32_t kRateCapacity = 64u;   ///< samples kept for the rate fit (100 Hz x 400 ms = 40)
};

struct GroundClearance {
    uint32_t timestampMs;         ///< LiDAR sample time of the values below
    uint32_t ageMs;               ///< at the update time
    ClearanceStatus status;
    uint32_t flags;               ///< clearflag::*

    float slantRange_m;           ///< raw range (NaN if none)
    Vec3f beamEnu;                ///< beam unit vector in ENU (NaN if no attitude)
    float projectionFactor;       ///< -beamEnu.z = cos(tilt from straight down) (NaN if none)
    float tiltDeg;

    bool clearanceValid;
    float clearance_m;            ///< vertical distance to the illuminated point (NaN if invalid)
    float clearanceSigma_m;       ///< range + attitude (+ mounting) uncertainty; NOT terrain slope
    float lowerBound_m;           ///< BeyondRange only: clearance is at least this (NaN otherwise)

    bool closureRateValid;
    float closureRate_mps;        ///< POSITIVE = clearance decreasing (approaching terrain); NaN if invalid
    float closureRateSigma_mps;
    uint32_t rateLatencyMs;       ///< ~ half the fitted window
    uint8_t rateSamples;

    bool ttcValid;
    float timeToContact_s;        ///< clearance / closure (constant closure); NaN if invalid

    // lifetime counters (saturating)
    uint32_t samples;             ///< new accepted LiDAR samples processed
    uint32_t validSamples;
    uint32_t rejectedGeometry;
    uint32_t rejectedAttitude;
    uint32_t staleUpdates;
    uint32_t beyondRangeUpdates;
    uint32_t weakSignalUpdates;
    uint32_t stepChanges;
    uint32_t duplicates;          ///< updates without a new sample
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_TYPES_HPP
