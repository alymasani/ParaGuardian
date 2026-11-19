/**
 ******************************************************************************
 * @file    ProcessedSensorFrame.hpp
 * @brief   Output of the preprocessing layer: calibrated, body-frame,
 *          cross-checked measurements WITH their quality metadata.
 *
 * RULE: a number in this struct means nothing unless its `valid` flag is
 * true. Missing data is never represented as 0; it is represented as
 * valid = false (and a quality flag says why).
 *
 * WHAT IT IS (and is not):
 *   specificForce_mps2  body FRD, accelerometer specific force (gravity NOT
 *                       removed - that needs attitude: see
 *                       AttitudeEstimate::linearAccel). At rest, level: (0, 0, -9.81).
 *   angularRate_radps   body FRD, gyro bias removed.
 *   baroAltitude_m      height relative to where the barometer reference was
 *                       taken, positive UP, ISA approximation; drifts with weather.
 *   mag.fieldBody_uT    body FRD, NOT hard/soft-iron calibrated.
 *   gnss                pass-through of the latest valid fix (MCU arrival time
 *                       + receiver time-of-week, never mixed).
 *   lidar               pass-through of the latest valid range.
 *
 * Consumers:
 *   Madgwick: angularRate (+ gyroDtMs), specificForce, optional mag.
 *   UKF (VerticalNavigationEstimator, via navigationInputFrom()): baroAltitude
 *             (+ per-barometer timestamps / references), gnss vertical
 *             velocity + speedAccuracy; vertical acceleration comes from
 *             the AttitudeEstimate. lidar: future landing package.
 *
 * Plain data: trivially copyable, fixed size, safe to copy into a mailbox/queue.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_PROCESSED_SENSOR_FRAME_HPP
#define PARAGUARDIAN_PROCESSING_PROCESSED_SENSOR_FRAME_HPP

#include <cstdint>
#include <type_traits>

#include "Processing/ProcessingTypes.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Frame-level quality flags (bit mask). Set = something the consumer must know.
namespace quality {
enum : uint32_t {
    MountingUnverified     = 1u << 0,   ///< sensor->body rotations are placeholders (identity)
    AccelNotCalibrated     = 1u << 1,   ///< accel bias/scale not estimated (factory sensitivity only)
    GyroNotCalibrated      = 1u << 2,   ///< no IMU has a valid gyro bias -> no angular rate output
    CalibrationInProgress  = 1u << 3,
    SamplesLost            = 1u << 4,   ///< sequence gaps observed since the previous frame
    SamplesRejected        = 1u << 5,   ///< validation rejected samples since the previous frame
    ImuDegraded            = 1u << 6,
    ImuAmbiguous           = 1u << 7,
    ImuUnavailable         = 1u << 8,   ///< no fresh IMU at all (critical)
    BaroDegraded           = 1u << 9,
    BaroAmbiguous          = 1u << 10,
    BaroUnavailable        = 1u << 11,  ///< no usable barometric altitude (critical)
    BaroNoReference        = 1u << 12,  ///< no barometer has a valid reference pressure
    BaroSourceChanged      = 1u << 13,  ///< contributing barometers differ from the previous frame
    MagUnavailable         = 1u << 14,
    MagNotIronCalibrated   = 1u << 15,  ///< always set in this milestone
    GnssUnavailable        = 1u << 16,
    LidarUnavailable       = 1u << 17,
    NoNewImuData           = 1u << 18,  ///< no IMU sample arrived since the previous frame
};
}  // namespace quality

/// A redundancy-checked vector (body frame).
struct VectorEstimate {
    Vec3f value;
    uint32_t timestampMs;        ///< newest contributing sample (MCU ms, read-completion time)
    uint32_t ageMs;              ///< frame time - timestampMs
    uint32_t sampleSpreadMs;     ///< newest - oldest contributing sample
    float maxDisagreement;       ///< largest pairwise |difference| among candidates
    RedundancyStatus status;
    uint8_t candidateMask;       ///< bit i = IMU i fresh & usable
    uint8_t contributingMask;    ///< bit i = IMU i is in `value`
    uint8_t excludedMask;        ///< bit i = IMU i persistently excluded / re-qualifying
    uint8_t outlierMask;         ///< bit i = IMU i was this frame's outlier
    bool partialAgreement;       ///< median of 3 used because only 2 of 3 pairs agreed
    bool valid;
};

/// A redundancy-checked scalar (barometric altitude).
struct ScalarEstimate {
    float value;
    uint32_t timestampMs;
    uint32_t ageMs;
    uint32_t sampleSpreadMs;
    float maxDisagreement;
    RedundancyStatus status;
    uint8_t candidateMask;
    uint8_t contributingMask;
    uint8_t excludedMask;
    bool valid;
};

/// One IMU's individual, processed reading (for diagnostics and custom consumers).
struct ImuChannelInfo {
    Vec3f accelBody_mps2;        ///< aligned, factory-calibrated
    Vec3f gyroBody_radps;        ///< aligned, bias-corrected if gyroCalibration == Valid
    uint32_t timestampMs;
    uint32_t ageMs;
    uint32_t lastRejectReasons;  ///< reject::* bits of the last rejected/partially rejected sample
    CalibrationState gyroCalibration;
    bool present;                ///< at least one accepted sample has been received
    bool fresh;                  ///< age <= maxAge
    bool accelUsable;
    bool gyroUsable;
    // ---- impact evidence since the PREVIOUS frame (crash detection, before any smoothing) ----
    // A frame normally carries one sample per IMU, but if processing was delayed several
    // samples may have been ingested; these keep the largest of them instead of only the
    // newest. Saturated / out-of-range samples ARE included (their value is a lower bound).
    float peakAccelNorm_mps2;    ///< largest |specific force| (sensor-calibrated, gravity NOT removed)
    float peakGyroNorm_radps;    ///< largest |angular rate| (bias-corrected if calibrated)
    uint32_t peakTimestampMs;    ///< timestamp of the sample with the largest |specific force|
    uint16_t samplesSinceFrame;  ///< accepted samples since the previous frame (0 = peaks not meaningful)
    bool accelSaturatedSinceFrame;  ///< a sample hit the accel full scale (value clipped)
    bool gyroSaturatedSinceFrame;   ///< a sample hit the gyro full scale (value clipped)
};

struct BaroChannelInfo {
    float pressure_pa;
    float relativeAltitude_m;
    float referencePressure_pa;
    uint32_t timestampMs;
    uint32_t ageMs;
    uint32_t lastRejectReasons;
    CalibrationState referenceState;
    bool present;
    bool fresh;
    bool altitudeValid;          ///< fresh && reference valid && finite
};

struct MagInfo {
    Vec3f fieldBody_uT;
    uint32_t timestampMs;
    uint32_t ageMs;
    bool valid;                  ///< fresh and accepted
    bool ironCalibrated;         ///< false: no hard/soft-iron calibration exists yet
};

struct GnssInfo {
    uint32_t timestampMs;        ///< MCU time the solution was parsed (TimeDomain::McuMillis)
    uint32_t ageMs;
    uint32_t timeOfWeek_ms;      ///< receiver time (TimeDomain::GnssTimeOfWeek) - NOT comparable with MCU time
    int32_t latitude_1e7deg;
    int32_t longitude_1e7deg;
    float heightMsl_m;
    float velocityNorth_mps;
    float velocityEast_mps;
    float verticalVelocity_mps;  ///< positive up
    float horizontalAccuracy_m;
    float verticalAccuracy_m;
    float speedAccuracy_mps;     ///< NAV-PVT sAcc (receiver's speed accuracy estimate)
    uint8_t numSatellites;
    GnssFixType fixType;
    bool valid;
};

struct LidarInfo {
    float distance_m;            ///< along the sensor boresight (not yet height above ground)
    uint16_t signalStrength;
    uint32_t timestampMs;
    uint32_t ageMs;
    bool valid;
    /// reject::* bits of the newest REJECTED sample (0 = none yet), e.g.
    /// LidarOutOfRange (no return within the driver's max range) or
    /// LidarWeakSignal (strength < 60 or saturated). Lets consumers tell
    /// "beyond range" from "sensor silent". Pass-through only.
    uint32_t lastRejectReasons;
    uint32_t lastRejectTimestampMs;
};

struct ProcessedSensorFrame {
    uint32_t sequence;           ///< increments per frame; gaps = frames a reader missed
    uint32_t timestampMs;        ///< MCU time the frame was built
    BodyFrame bodyFrame;         ///< FRD
    TimeDomain timeDomain;       ///< McuMillis for every *timestampMs field except gnss.timeOfWeek_ms

    VectorEstimate specificForce_mps2;
    VectorEstimate angularRate_radps;
    uint32_t gyroDtMs;           ///< time between this and the previous angular-rate output
    bool gyroDtValid;            ///< both outputs valid, new data, 0 < dt <= limit

    ScalarEstimate baroAltitude_m;

    ImuChannelInfo imu[kImuCount];
    BaroChannelInfo baro[kBaroCount];
    MagInfo mag;
    GnssInfo gnss;
    LidarInfo lidar;

    CalibrationState calibrationState;
    CalibrationFailure lastCalibrationFailure;
    uint32_t calibrationSessionId;

    uint32_t qualityFlags;               ///< quality::* bits
    uint32_t samplesLostSinceLastFrame;  ///< sequence-gap total since previous frame
    uint32_t samplesRejectedSinceLastFrame;

    bool has(uint32_t flags) const { return (qualityFlags & flags) != 0u; }
};

static_assert(std::is_trivially_copyable<ProcessedSensorFrame>::value, "frame must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_PROCESSED_SENSOR_FRAME_HPP
