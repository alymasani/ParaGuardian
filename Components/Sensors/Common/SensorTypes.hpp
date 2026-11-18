/**
 ******************************************************************************
 * @file    SensorTypes.hpp
 * @brief   Standardised, SI-unit sensor samples shared by drivers, the
 *          acquisition task and (later) the algorithms.
 *
 * UNIT CONVENTION (every float in this file):
 *   acceleration     m/s^2        angular rate   rad/s
 *   pressure         Pa           temperature    degrees Celsius
 *   magnetic field   microtesla   distance       metres
 *   velocity         m/s          time stamps    milliseconds since boot (uint32, wraps ~49.7 days)
 *
 * AXIS CONVENTION:
 *   Every vector is in the SENSOR'S OWN axes exactly as printed in its
 *   datasheet (right-handed X/Y/Z of the chip package). Rotating into the
 *   paraglider/body frame needs the PCB placement, which is not known yet;
 *   it will be done once, in one place, by the BSP/algorithm layer.
 *
 * All structs are plain data (trivially copyable) so they can be copied
 * byte-for-byte into an RTOS queue.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_COMMON_SENSOR_TYPES_HPP
#define PARAGUARDIAN_SENSORS_COMMON_SENSOR_TYPES_HPP

#include <cstdint>
#include <type_traits>

namespace pg {

namespace units {
constexpr float kStandardGravity_mps2 = 9.80665f;           // 1 g, exact by definition
constexpr float kDegToRad = 0.017453292519943295f;           // pi / 180
constexpr float kGaussToMicroTesla = 100.0f;                 // 1 G = 1e-4 T = 100 uT
}  // namespace units

struct Vec3f {
    float x;
    float y;
    float z;
};

/// Logical identity of each physical sensor on the board.
enum class SensorId : uint8_t {
    Imu0 = 0,
    Imu1,
    Imu2,
    Baro0,
    Baro1,
    Mag0,
    Gnss0,
    Lidar0,
    Count
};

/// Common header carried by every sample.
struct SampleHeader {
    SensorId id;
    uint32_t timestampMs;  ///< Time the read completed (IClock::millis()).
    uint32_t sequence;     ///< Increments per published sample of this sensor; gaps = lost samples.
    bool valid;            ///< False if the reading must not be used at all.
};

struct ImuSample {
    SampleHeader header;
    Vec3f accel_mps2;
    Vec3f gyro_radps;
    float temperature_c;
    bool accelFresh;        ///< Status register said "new accel data" (XLDA).
    bool gyroFresh;         ///< Status register said "new gyro data" (GDA).
    bool accelSaturated;    ///< At least one accel axis at the int16 limit (+32767 / -32768).
    bool gyroSaturated;     ///< At least one gyro axis at the int16 limit.
};

struct BaroSample {
    SampleHeader header;
    float pressure_pa;
    float temperature_c;
    bool pressureInRange;   ///< Within the sensor's specified operating range.
};

struct MagSample {
    SampleHeader header;
    Vec3f field_uT;
    bool saturated;         ///< At least one axis at 0 or full scale (outside +-8 G).
    bool offsetCancelled;   ///< True when produced by a SET/RESET pair (bridge offset removed).
};

enum class GnssFixType : uint8_t {
    NoFix = 0,
    DeadReckoningOnly = 1,
    Fix2D = 2,
    Fix3D = 3,
    GnssPlusDeadReckoning = 4,
    TimeOnly = 5,
};

struct GnssSample {
    SampleHeader header;
    // Position: kept as the receiver's integer 1e-7 degree units. A float has
    // only ~7 significant digits (~0.4 m resolution at 50 deg), so we do NOT
    // convert to float here. Use toDegrees() (double) when needed.
    int32_t latitude_1e7deg;
    int32_t longitude_1e7deg;
    float heightMsl_m;
    float heightEllipsoid_m;
    float velocityNorth_mps;
    float velocityEast_mps;
    float verticalVelocity_mps;   ///< Positive UP (= -velD from the receiver).
    float groundSpeed_mps;
    float horizontalAccuracy_m;   ///< Receiver's own 1-sigma-ish estimate (hAcc).
    float verticalAccuracy_m;     ///< vAcc
    float speedAccuracy_mps;      ///< sAcc
    float pDop;
    uint32_t gnssTimeOfWeek_ms;   ///< iTOW
    uint16_t utcYear;
    uint8_t utcMonth, utcDay, utcHour, utcMinute, utcSecond;
    uint8_t numSatellites;
    GnssFixType fixType;
    bool gnssFixOk;               ///< flags.gnssFixOK: fix within DOP/accuracy masks.
    bool positionValid;           ///< gnssFixOk && 3D-capable fix && !invalidLlh
    bool heightValid;             ///< positionValid && fix includes height (3D)
    bool utcDateValid;
    bool utcTimeValid;
};

inline double toDegrees(int32_t value_1e7deg) { return static_cast<double>(value_1e7deg) * 1e-7; }

struct LidarSample {
    SampleHeader header;
    float distance_m;
    uint16_t signalStrength;     ///< Raw "Strength" field (unitless, 0..65535).
    float temperature_c;
    bool outOfRange;             ///< Distance beyond configured max range (or device sentinel).
    bool weakSignal;             ///< Strength below the manufacturer's reliability threshold.
};

static_assert(std::is_trivially_copyable<ImuSample>::value, "samples must be POD for RTOS queues");
static_assert(std::is_trivially_copyable<GnssSample>::value, "samples must be POD for RTOS queues");

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_COMMON_SENSOR_TYPES_HPP
