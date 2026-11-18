/**
 ******************************************************************************
 * @file    ProcessingTypes.hpp
 * @brief   Shared enums, flags and small helpers of the preprocessing layer.
 *
 * Rejection reasons and quality flags are BIT MASKS rather than one boolean so
 * a consumer (and a person in the debugger) can see every reason a sample was
 * rejected at once, e.g. "stale AND saturated".
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_PROCESSING_TYPES_HPP
#define PARAGUARDIAN_PROCESSING_PROCESSING_TYPES_HPP

#include <cstddef>
#include <cstdint>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

constexpr size_t kSensorCount = static_cast<size_t>(SensorId::Count);
constexpr size_t kImuCount = 3u;
constexpr size_t kBaroCount = 2u;

constexpr size_t sensorIndex(SensorId id) { return static_cast<size_t>(id); }
constexpr bool isImu(SensorId id) { return id == SensorId::Imu0 || id == SensorId::Imu1 || id == SensorId::Imu2; }
constexpr bool isBaro(SensorId id) { return id == SensorId::Baro0 || id == SensorId::Baro1; }
/// 0..2 for Imu0..Imu2 (only meaningful when isImu(id)).
constexpr size_t imuIndex(SensorId id) { return static_cast<size_t>(id) - static_cast<size_t>(SensorId::Imu0); }
/// 0..1 for Baro0..Baro1 (only meaningful when isBaro(id)).
constexpr size_t baroIndex(SensorId id) { return static_cast<size_t>(id) - static_cast<size_t>(SensorId::Baro0); }
constexpr SensorId imuId(size_t i) { return static_cast<SensorId>(static_cast<size_t>(SensorId::Imu0) + i); }
constexpr SensorId baroId(size_t i) { return static_cast<SensorId>(static_cast<size_t>(SensorId::Baro0) + i); }

/**
 * Why a sample (or one channel of it) was not accepted. Several may be set.
 * "Whole-sample" reasons reject every channel; channel reasons only that channel.
 */
namespace reject {
enum : uint32_t {
    None = 0u,
    // ---- whole-sample reasons ----
    NoSample            = 1u << 0,   ///< message type None / null header
    UnknownSensor       = 1u << 1,   ///< id out of range or id does not match message type
    FlagInvalid         = 1u << 2,   ///< driver set header.valid = false
    NonFinite           = 1u << 3,   ///< NaN or Inf in a field that is used
    Stale               = 1u << 4,   ///< older than the per-sensor maximum age
    TimestampFuture     = 1u << 5,   ///< timestamp ahead of "now" by more than the tolerance
    TimestampBackwards  = 1u << 6,   ///< newer sequence but older timestamp than the last accepted sample
    Duplicate           = 1u << 7,   ///< same sequence number as the last accepted sample
    OutOfOrder          = 1u << 8,   ///< sequence number older than the last accepted sample
    TemperatureRange    = 1u << 9,   ///< die temperature outside the operating range
    // ---- IMU channel reasons ----
    AccelSaturated      = 1u << 10,
    GyroSaturated       = 1u << 11,
    AccelOutOfRange     = 1u << 12,  ///< |component| above the configured full scale
    GyroOutOfRange      = 1u << 13,
    AccelNotNew         = 1u << 14,  ///< status register said "no new accel data" (repeat value)
    GyroNotNew          = 1u << 15,
    // ---- other sensor reasons ----
    PressureRange       = 1u << 16,  ///< <= 0 Pa or outside the BMP585 operating range
    MagSaturated        = 1u << 17,
    MagOutOfRange       = 1u << 18,
    LidarOutOfRange     = 1u << 19,
    LidarWeakSignal     = 1u << 20,
    GnssNoFix           = 1u << 21,
    GnssBadAccuracy     = 1u << 22,  ///< negative / non-finite accuracy estimates
};

constexpr uint32_t kWholeSampleMask = NoSample | UnknownSensor | FlagInvalid | NonFinite | Stale | TimestampFuture |
                                      TimestampBackwards | Duplicate | OutOfOrder | TemperatureRange | PressureRange |
                                      MagSaturated | MagOutOfRange | LidarOutOfRange | LidarWeakSignal | GnssNoFix |
                                      GnssBadAccuracy;
constexpr uint32_t kAccelMask = AccelSaturated | AccelOutOfRange | AccelNotNew;
constexpr uint32_t kGyroMask = GyroSaturated | GyroOutOfRange | GyroNotNew;
constexpr unsigned kReasonBits = 23u;
}  // namespace reject

/// Health of a redundant group (3 IMUs, 2 barometers).
enum class RedundancyStatus : uint8_t {
    Unavailable = 0,  ///< no usable sensor: no output
    Healthy,          ///< every configured sensor contributed and all agree
    Degraded,         ///< output produced, but with reduced redundancy or partial agreement
    Ambiguous,        ///< usable sensors disagree and the faulty one cannot be identified: no output
};

/// Calibration state of one sensor (gyro bias, barometer reference).
enum class CalibrationState : uint8_t {
    NotCalibrated = 0,  ///< never calibrated (or invalidated)
    Collecting,         ///< a calibration session is gathering data
    Valid,              ///< a calibration result is in use
    Failed,             ///< the last session could not calibrate this sensor
};

/// Why a sensor's calibration failed.
enum class CalibrationFailure : uint8_t {
    None = 0,
    InsufficientSamples,   ///< too few valid samples in the window (missing / intermittent sensor)
    TooNoisy,              ///< standard deviation above the limit
    Drifting,              ///< segment means moved more than allowed (slow motion / warm-up / weather)
    NoStationaryImu,       ///< no IMU confirmed the device was still: nothing may be calibrated
    TimedOut,              ///< session exceeded its maximum duration (wearer never still long enough)
    InvalidValue,          ///< estimate non-finite or physically impossible (e.g. pressure <= 0)
};

/// Output of the stationary detector.
enum class StationaryStatus : uint8_t {
    InsufficientData = 0,  ///< window not yet long enough / not enough samples
    Moving,                ///< the latest sample (or window statistics) showed motion
    Stationary,            ///< a full, continuous, still window has been observed
};

/// Clock domain of a time value. MCU and GNSS time are NEVER mixed without an explicit mapping.
enum class TimeDomain : uint8_t {
    McuMillis = 0,      ///< HAL_GetTick() milliseconds since boot (wraps after ~49.7 days)
    GnssTimeOfWeek,     ///< receiver iTOW, ms since the start of the GPS week
};

/// Body frame convention used by every processed vector.
enum class BodyFrame : uint8_t {
    FrdBody = 0,  ///< X forward, Y right, Z down (right-handed). See SENSOR_PREPROCESSING_DESIGN.md.
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_PROCESSING_TYPES_HPP
