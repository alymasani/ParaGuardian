/**
 ******************************************************************************
 * @file    AttitudeTypes.hpp
 * @brief   Input and output types of the attitude subsystem.
 *
 * AttitudeInput   what the estimator consumes (built from a
 *                 ProcessedSensorFrame by attitudeInputFromFrame(), or
 *                 directly by the simulation tests).
 * AttitudeEstimate what it produces. RULE (same as ProcessedSensorFrame): a
 *                 number means nothing unless its validity flag says so.
 *                 Finite is not the same as valid.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_TYPES_HPP

#include <cstdint>
#include <type_traits>

#include "Algorithms/Attitude/Quaternion.hpp"
#include "Processing/ProcessingTypes.hpp"

namespace pg {

/// Estimator state machine (see MADGWICK_ATTITUDE_DESIGN.md section 7).
enum class AttitudeState : uint8_t {
    Uninitialized = 0,  ///< no usable (calibrated) gyro + accelerometer yet: no orientation
    Initializing,       ///< collecting a quasi-static window to level from gravity
    Tracking6Axis,      ///< gyro + accepted accelerometer corrections; heading not magnetically corrected
    Tracking9Axis,      ///< as above + accepted magnetometer corrections (see HeadingReference for trust)
    Degraded,           ///< orientation propagated, but corrections absent for long / after a gap / recovering
    Invalid,            ///< orientation lost (long gap, numerical failure): output invalid, re-initialising
};

/// How much the heading (yaw) can be trusted. Roll/pitch do not depend on this.
enum class HeadingReference : uint8_t {
    Unreferenced = 0,      ///< 6-axis: yaw is relative to an ARBITRARY start (0 = "pseudo-north"); drifts
    MagneticUnvalidated,   ///< magnetometer used WITHOUT hard/soft-iron calibration (experimental)
    MagneticReferenced,    ///< calibrated magnetometer accepted recently: yaw = MAGNETIC heading
    MagneticLost,          ///< was referenced, no accepted magnetometer for headingHoldMs: drifting
};

/// What happened to the gyro sample of this update.
enum class GyroIntegration : uint8_t {
    None = 0,      ///< not tracking (or no gyro data at all)
    Integrated,    ///< integrated over dt
    NotNew,        ///< same timestamp as the last integrated sample (republished frame): skipped
    OutOfOrder,    ///< older than the last integrated sample: skipped
    GapSkipped,    ///< dt > maxIntegrationDtMs: interval NOT integrated (rates inside unknown)
    Unavailable,   ///< gyro invalid / stale / uncalibrated
};

/// Outcome of an accelerometer or magnetometer correction attempt.
enum class CorrectionStatus : uint8_t {
    NotAttempted = 0,     ///< no gyro step happened (nothing to correct)
    Accepted,
    AcceptedRecovery,     ///< accepted by the recovery rule although outside the normal gate
    Unavailable,          ///< no valid / fresh measurement
    Disabled,             ///< policy forbids it (e.g. uncalibrated magnetometer)
    RejectedMagnitude,    ///< |f| far from g, or |m| implausible / changed
    RejectedDirection,    ///< angle gate (tilt for accel, heading for mag)
    RejectedRotation,     ///< accel only: turning faster than accelRejectionRate_radps (turn gate)
    RejectedInclination,  ///< magnetic dip angle changed (magnetic disturbance)
    NoGravityReference,   ///< mag only: skipped because the accelerometer was not accepted
    Acquiring,            ///< mag only: collecting consistent samples to acquire the heading
};

namespace attitude {
/// AttitudeEstimate::qualityFlags bits. Set = something the consumer must know.
enum : uint32_t {
    NotInitialized          = 1u << 0,   ///< no valid orientation
    GyroUnavailable         = 1u << 1,
    GyroNotCalibrated       = 1u << 2,   ///< preprocessing has no gyro bias yet
    AccelUnavailable        = 1u << 3,
    AccelRejected           = 1u << 4,   ///< this update's accel correction was rejected
    AccelRejectedProlonged  = 1u << 5,   ///< no accepted correction for degradedAfterMs
    Recovering              = 1u << 6,   ///< recovery gain active
    GapDetected             = 1u << 7,   ///< a gyro gap occurred and no correction has confirmed the attitude since
    HeadingUnreferenced     = 1u << 8,   ///< heading != MagneticReferenced (yaw not absolute)
    MagUncalibrated         = 1u << 9,   ///< magnetometer used without iron calibration
    MagRejected             = 1u << 10,  ///< this update's magnetometer correction was rejected
    MountingUnverified      = 1u << 11,  ///< inherited from preprocessing (identity placeholders)
    AccelNotCalibrated      = 1u << 12,  ///< inherited: accel bias/scale not estimated
    ImuRedundancyDegraded   = 1u << 13,  ///< gyro or accel vote was Degraded
    NumericalFault          = 1u << 14,  ///< a non-finite / degenerate value was caught this update
    LinearAccelUnreliable   = 1u << 15,  ///< linear acceleration computed but attitude not fully trusted
    InvalidConfig           = 1u << 16,  ///< AttitudeConfig failed isValid(): estimator disabled
};
}  // namespace attitude

/// One update's worth of input. Body frame FRD, SI units, MCU-millisecond timestamps.
struct AttitudeInput {
    uint32_t nowMs;

    Vec3f gyro_radps;
    uint32_t gyroTimestampMs;
    bool gyroValid;
    bool gyroCalibrated;
    uint8_t gyroSourceMask;              ///< bit i = IMU i contributed
    RedundancyStatus gyroStatus;

    Vec3f accel_mps2;                    ///< specific force (gravity NOT removed)
    uint32_t accelTimestampMs;
    bool accelValid;
    uint8_t accelSourceMask;
    RedundancyStatus accelStatus;

    Vec3f mag_uT;
    uint32_t magTimestampMs;
    bool magValid;
    bool magCalibrated;                  ///< hard/soft-iron calibration applied

    bool mountingVerified;
    bool accelCalibrated;
};

/// Linear (gravity-free) acceleration in ENU. See GravityCompensator.
struct LinearAcceleration {
    Vec3f enu_mps2;
    float vertical_mps2;     ///< = enu_mps2.z, positive UP
    uint32_t timestampMs;    ///< accelerometer sample time
    bool valid;              ///< computed from a valid attitude and a valid accelerometer sample
    bool reliable;           ///< valid AND attitude in a Tracking state (not Degraded/Initializing)
};

/// Rotation convention carried in the output so a consumer cannot misread it.
enum class RotationConvention : uint8_t {
    EnuFromFrdBody = 0,  ///< q rotates body FRD vectors into ENU: v_ENU = q (x) v_body (x) q*
};

struct AttitudeEstimate {
    uint32_t sequence;               ///< increments per update
    uint32_t timestampMs;            ///< time the orientation refers to (last integrated gyro sample)
    uint32_t computedAtMs;           ///< MCU time of the update

    Quaternion q;                    ///< q_ENU_from_body, scalar first. AUTHORITATIVE orientation.
    RotationConvention convention;
    bool orientationValid;           ///< q is a usable orientation (any state except Uninitialized/Initializing/Invalid)

    /// Display only: degrees, aerospace Z-Y-X of the FRD body relative to NED
    /// (roll right-down +, pitch nose-up +, yaw clockwise from north, -180..180).
    EulerAngles eulerDeg;
    bool yawReferenced;              ///< yaw is an absolute (magnetic) heading

    LinearAcceleration linearAccel;

    AttitudeState state;
    HeadingReference heading;

    uint32_t dtMs;                   ///< integrated interval of this update (0 if none)
    GyroIntegration gyroIntegration;
    CorrectionStatus accelCorrection;
    CorrectionStatus magCorrection;
    float accelErrorDeg;             ///< angle between measured and predicted "up" (diagnostic; -1 = n/a)
    float magHeadingErrorDeg;        ///< heading disagreement (diagnostic; 0 when n/a)
    float betaUsed_radps;            ///< gain of this update (0 = no correction)

    uint32_t gyroAgeMs;              ///< at nowMs; UINT32_MAX = no sample
    uint32_t accelAgeMs;
    uint32_t magAgeMs;
    uint32_t sinceAccelAcceptedMs;   ///< UINT32_MAX = never
    uint32_t sinceMagAcceptedMs;

    uint8_t gyroSourceMask;
    uint8_t accelSourceMask;
    bool magUsed;

    uint32_t qualityFlags;           ///< attitude::* bits

    // Lifetime counters (saturating)
    uint32_t initializations;        ///< successful levellings (first one + every re-initialisation)
    uint32_t invalidations;          ///< orientation discarded (gap / numerical failure)
    uint32_t recoveries;             ///< recovery episodes entered
    uint32_t gaps;                   ///< gaps > maxIntegrationDtMs
    uint32_t notNewSamples;          ///< repeated gyro timestamps skipped
    uint32_t outOfOrderSamples;
    uint32_t accelRejections;
    uint32_t magRejections;
    uint32_t numericalFaults;
    uint32_t headingAcquisitions;

    bool has(uint32_t flags) const { return (qualityFlags & flags) != 0u; }
};

static_assert(std::is_trivially_copyable<AttitudeEstimate>::value, "estimate must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_TYPES_HPP
