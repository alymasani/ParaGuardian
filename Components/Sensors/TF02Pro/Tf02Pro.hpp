/**
 ******************************************************************************
 * @file    Tf02Pro.hpp
 * @brief   Benewake TF02-Pro LiDAR: UART frame parser + measurement validity.
 *
 * Standard 9-byte output frame (TF02-Pro product manual, "Data output format"):
 *
 *   byte 0   0x59        frame header
 *   byte 1   0x59        frame header
 *   byte 2   Dist_L      distance low byte   (default unit: cm)
 *   byte 3   Dist_H      distance high byte
 *   byte 4   Strength_L  signal strength low byte
 *   byte 5   Strength_H  signal strength high byte
 *   byte 6   Temp_L      chip temperature low byte
 *   byte 7   Temp_H      chip temperature high byte
 *   byte 8   Checksum    low 8 bits of (byte0 + ... + byte7)
 *
 * Temperature in degC = raw / 8 - 256.
 * Reliability (manual): readings with Strength < 60, or Strength = 65535
 * (over-exposure), are not reliable.
 *
 * VERIFY ON HARDWARE: default baud 115200 8N1, 100 Hz frame rate, cm unit,
 * and the out-of-range sentinel value reported by your module's firmware.
 *
 * This class does NOT touch the UART and does NOT implement landing guidance.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_TF02PRO_HPP
#define PARAGUARDIAN_SENSORS_TF02PRO_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

namespace tf02 {
constexpr uint8_t HEADER = 0x59;
constexpr size_t FRAME_LEN = 9u;
constexpr uint16_t MIN_RELIABLE_STRENGTH = 60u;
constexpr uint16_t OVEREXPOSED_STRENGTH = 65535u;

struct RawFrame {
    uint16_t distance;
    uint16_t strength;
    uint16_t temperatureRaw;
};

struct ParserStats {
    uint32_t framesOk = 0;
    uint32_t checksumErrors = 0;
    uint32_t discardedBytes = 0;
};

/// Streaming parser with resynchronisation inside a corrupted 9-byte window.
class Parser {
public:
    /// Returns true when @p byte completes a valid frame (written to @p out).
    bool feed(uint8_t byte, RawFrame& out);
    void reset();
    const ParserStats& stats() const { return stats_; }

private:
    void resyncAfterBadFrame();

    uint8_t buf_[FRAME_LEN] = {};
    size_t count_ = 0u;
    ParserStats stats_{};
};

float rawToTemperatureC(uint16_t raw);
}  // namespace tf02

enum class Tf02DistanceUnit : uint8_t { Centimetre, Millimetre };

struct Tf02ProConfig {
    Tf02DistanceUnit unit = Tf02DistanceUnit::Centimetre;
    float maxRange_m = 40.0f;       ///< Manual: 0.1..40 m at 90 % reflectivity.
    uint32_t staleTimeoutMs = 100u; ///< 10 missed frames at the default 100 Hz.
};

class Tf02Pro {
public:
    explicit Tf02Pro(const Tf02ProConfig& config = Tf02ProConfig{}, SensorId id = SensorId::Lidar0)
        : config_(config), id_(id) {}

    /// Feeds UART bytes; returns the number of new measurements decoded.
    size_t processBytes(const uint8_t* data, size_t len, uint32_t nowMs);

    /// Marks the latest sample invalid if no frame arrived within the stale timeout.
    void poll(uint32_t nowMs);

    bool hasFreshMeasurement(uint32_t nowMs) const;
    const LidarSample& latest() const { return latest_; }

    /// Pure conversion: raw frame -> LidarSample (no timestamp/sequence).
    static void frameToSample(const tf02::RawFrame& frame, const Tf02ProConfig& config, LidarSample& out);

    const tf02::ParserStats& parserStats() const { return parser_.stats(); }
    const SensorHealth& health() const { return health_; }
    SensorId id() const { return id_; }

private:
    Tf02ProConfig config_;
    SensorId id_;
    tf02::Parser parser_{};
    LidarSample latest_{};
    bool haveMeasurement_ = false;
    uint32_t lastMeasurementMs_ = 0;
    uint32_t sequence_ = 0;
    SensorHealth health_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_TF02PRO_HPP
