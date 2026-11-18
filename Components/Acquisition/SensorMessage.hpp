/**
 ******************************************************************************
 * @file    SensorMessage.hpp
 * @brief   One tagged message that can carry any sensor sample through a queue.
 *
 * A FreeRTOS queue copies a fixed number of bytes per item, so every item
 * must have the same size. A tagged union does exactly that: "type" says
 * which member of the union is meaningful. All members are plain structs,
 * so the whole message is trivially copyable (memcpy-safe).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SENSOR_MESSAGE_HPP
#define PARAGUARDIAN_ACQUISITION_SENSOR_MESSAGE_HPP

#include <cstdint>
#include <type_traits>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

enum class SensorMessageType : uint8_t { None = 0, Imu, Baro, Mag, Gnss, Lidar };

struct SensorMessage {
    SensorMessageType type;
    union {
        ImuSample imu;
        BaroSample baro;
        MagSample mag;
        GnssSample gnss;
        LidarSample lidar;
    };

    static SensorMessage fromImu(const ImuSample& s) { SensorMessage m{}; m.type = SensorMessageType::Imu; m.imu = s; return m; }
    static SensorMessage fromBaro(const BaroSample& s) { SensorMessage m{}; m.type = SensorMessageType::Baro; m.baro = s; return m; }
    static SensorMessage fromMag(const MagSample& s) { SensorMessage m{}; m.type = SensorMessageType::Mag; m.mag = s; return m; }
    static SensorMessage fromGnss(const GnssSample& s) { SensorMessage m{}; m.type = SensorMessageType::Gnss; m.gnss = s; return m; }
    static SensorMessage fromLidar(const LidarSample& s) { SensorMessage m{}; m.type = SensorMessageType::Lidar; m.lidar = s; return m; }
};

static_assert(std::is_trivially_copyable<SensorMessage>::value, "SensorMessage must be memcpy-safe");

/// Header of whichever sample the message carries, or nullptr for SensorMessageType::None.
inline const SampleHeader* headerOf(const SensorMessage& m)
{
    switch (m.type) {
        case SensorMessageType::Imu:   return &m.imu.header;
        case SensorMessageType::Baro:  return &m.baro.header;
        case SensorMessageType::Mag:   return &m.mag.header;
        case SensorMessageType::Gnss:  return &m.gnss.header;
        case SensorMessageType::Lidar: return &m.lidar.header;
        case SensorMessageType::None:  break;
    }
    return nullptr;
}

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SENSOR_MESSAGE_HPP
