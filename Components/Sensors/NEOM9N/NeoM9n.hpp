/**
 ******************************************************************************
 * @file    NeoM9n.hpp
 * @brief   u-blox NEO-M9N GNSS receiver (MIKROE-3922 "GNSS 7 Click") driver:
 *          UBX-NAV-PVT decoding, configuration frames, stale-data detection.
 *
 * This class does NOT touch the UART. The board/task layer reads bytes from
 * the UART receive buffer and hands them to processBytes(). That keeps the
 * protocol logic testable on a laptop.
 *
 * Reference: u-blox "NEO-M9N / u-blox M9 SPG 4.04 Interface Description"
 * (UBX-19035940): UBX-NAV-PVT (0x01 0x07, 92 bytes), UBX-CFG-VALSET (0x06 0x8A),
 * UBX-ACK-ACK / ACK-NAK (0x05 0x01 / 0x05 0x00).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_NEOM9N_HPP
#define PARAGUARDIAN_SENSORS_NEOM9N_HPP

#include <cstddef>
#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/Status.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/Common/SensorTypes.hpp"
#include "Sensors/NEOM9N/UbxParser.hpp"

namespace pg {

namespace ubx {
constexpr uint8_t CLASS_NAV = 0x01;
constexpr uint8_t ID_NAV_PVT = 0x07;
constexpr uint16_t LEN_NAV_PVT = 92u;
constexpr uint8_t CLASS_ACK = 0x05;
constexpr uint8_t ID_ACK_NAK = 0x00;
constexpr uint8_t ID_ACK_ACK = 0x01;
constexpr uint8_t CLASS_CFG = 0x06;
constexpr uint8_t ID_CFG_VALSET = 0x8A;

// Configuration keys (u-blox M9 SPG 4.04 Interface Description, section 6.9).
// VERIFY against the interface description for your receiver firmware before
// relying on them in flight. Bits 28..30 of each key give the value size.
constexpr uint32_t KEY_CFG_UART1OUTPROT_UBX = 0x10740001u;       // L  (1 byte)
constexpr uint32_t KEY_CFG_UART1OUTPROT_NMEA = 0x10740002u;      // L  (1 byte)
constexpr uint32_t KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007u; // U1 (output rate per nav solution)
constexpr uint32_t KEY_CFG_RATE_MEAS = 0x30210001u;              // U2 (measurement period, ms)

constexpr uint8_t VALSET_LAYER_RAM = 0x01;
constexpr uint8_t VALSET_LAYER_BBR = 0x02;
constexpr uint8_t VALSET_LAYER_FLASH = 0x04;

/// Decoded UBX-NAV-PVT, in the receiver's own integer units.
struct NavPvt {
    uint32_t iTOW_ms;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;       // bit0 validDate, bit1 validTime, bit2 fullyResolved
    uint32_t tAcc_ns;
    int32_t nano_ns;
    uint8_t fixType;
    uint8_t flags;       // bit0 gnssFixOK
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon_1e7deg;
    int32_t lat_1e7deg;
    int32_t height_mm;   // above ellipsoid
    int32_t hMSL_mm;     // above mean sea level
    uint32_t hAcc_mm;
    uint32_t vAcc_mm;
    int32_t velN_mmps;
    int32_t velE_mmps;
    int32_t velD_mmps;   // positive DOWN
    int32_t gSpeed_mmps;
    int32_t headMot_1e5deg;
    uint32_t sAcc_mmps;
    uint32_t headAcc_1e5deg;
    uint16_t pDOP_001;
    uint16_t flags3;     // bit0 invalidLlh
};

/// Decodes a NAV-PVT frame. Returns false if class/id/length do not match.
bool decodeNavPvt(const Frame& frame, NavPvt& out);

/// Converts NAV-PVT to the standard SI GnssSample (validity rules documented in NeoM9n.cpp).
void navPvtToSample(const NavPvt& pvt, GnssSample& out);

struct ConfigItem {
    uint32_t key;
    uint32_t value;  ///< Only the low N bytes are sent, N given by the key's size field.
};

/// Value size in bytes encoded in bits 28..30 of a configuration key (1 bit is sent as 1 byte).
size_t keyValueSize(uint32_t key);

/**
 * Builds a complete UBX-CFG-VALSET frame (version 0) into @p dst.
 * @return frame length, or 0 if it does not fit / a key has an unsupported size.
 */
size_t buildValSet(const ConfigItem* items, size_t count, uint8_t layers, uint8_t* dst, size_t dstCapacity);
}  // namespace ubx

enum class GnssConfigAck : uint8_t { None, Ack, Nak };

class NeoM9n {
public:
    explicit NeoM9n(SensorId id = SensorId::Gnss0, uint32_t staleTimeoutMs = 2000u)
        : id_(id), staleTimeoutMs_(staleTimeoutMs) {}

    /**
     * Feeds received UART bytes. Each complete NAV-PVT frame updates latest().
     * @return number of new NAV-PVT solutions decoded from these bytes.
     */
    size_t processBytes(const uint8_t* data, size_t len, uint32_t nowMs);

    /// Call periodically: abandons stalled partial frames.
    void poll(uint32_t nowMs);

    /// True if a NAV-PVT has been received within the stale timeout.
    bool hasFreshSolution(uint32_t nowMs) const;

    /// Last decoded solution (header.valid=false until one arrives).
    const GnssSample& latest() const { return latest_; }

    /// Result of the last ACK/NAK received for a CFG message.
    GnssConfigAck lastConfigAck() const { return lastAck_; }
    void clearConfigAck() { lastAck_ = GnssConfigAck::None; }

    /**
     * Builds the VALSET frame that enables UBX output on UART1 and NAV-PVT at
     * one message per navigation solution with the given measurement period.
     * Send the returned bytes with the UART writer, then watch lastConfigAck().
     */
    static size_t buildEnableNavPvtFrame(uint16_t measurementPeriodMs, bool disableNmea, uint8_t* dst,
                                         size_t dstCapacity);

    const ubx::ParserStats& parserStats() const { return parser_.stats(); }
    const SensorHealth& health() const { return health_; }
    SensorId id() const { return id_; }

private:
    void handleFrame(const ubx::Frame& frame, uint32_t nowMs);

    SensorId id_;
    uint32_t staleTimeoutMs_;
    ubx::Parser parser_{};
    GnssSample latest_{};
    bool haveSolution_ = false;
    uint32_t lastSolutionMs_ = 0;
    uint32_t sequence_ = 0;
    GnssConfigAck lastAck_ = GnssConfigAck::None;
    SensorHealth health_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_NEOM9N_HPP
