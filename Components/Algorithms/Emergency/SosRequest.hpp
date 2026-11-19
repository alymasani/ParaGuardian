/**
 ******************************************************************************
 * @file    SosRequest.hpp
 * @brief   Versioned SOS request (firmware -> phone app) and its bounded
 *          binary encoding.
 *
 * WHY A NEW FORMAT NEXT TO SosMessage.hpp: the existing SosPayload /
 * formatSosRecord() (Communications/Bluetooth) has no version, no incident id
 * (so no idempotency) and no validity / age information. It is kept
 * unchanged; toLegacySosPayload() maps a request onto it for compatibility.
 *
 * CONTENT RULES
 *   - Position = the LAST VALID GNSS fix with its AGE, never invented, never
 *     replaced by a guess. No fix ever: sosvalid::Position is clear and the
 *     coordinates are 0 (meaningless; the flag says so).
 *   - heightMsl = GNSS height above mean sea level. relativeAltitude = the
 *     barometric UKF altitude relative to its reference (NOT MSL, NOT above
 *     ground). They are separate fields and never substituted.
 *   - No personal data (name, phone numbers): those live in the phone app.
 *
 * ENCODING: kSosFrameBytes, little-endian, fixed layout, CRC-32 (IEEE) at the
 * end. Version byte first so a phone app can reject versions it does not know.
 * The phone deduplicates by (device, incidentId); revision orders updates of
 * the same incident.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Communications/Bluetooth/SosMessage.hpp"

namespace pg {

constexpr uint8_t kSosProtocolVersion = 1u;
constexpr size_t kSosFrameBytes = 64u;   ///< incl. the 4-byte CRC

enum class SosRequestKind : uint8_t { Invalid = 0, Alert = 1, Cancel = 2 };

namespace sosvalid {
/// SosRequest::validity: which fields carry data.
enum : uint16_t {
    Position          = 1u << 0,   ///< latitude / longitude / accuracy / fix age valid
    HeightMsl         = 1u << 1,   ///< GNSS height above MSL valid
    RelativeAltitude  = 1u << 2,   ///< barometric relative altitude valid
    VerticalSpeed     = 1u << 3,
    Impact            = 1u << 4,   ///< impact summary valid (automatic incidents)
    ImpactSaturated   = 1u << 5,   ///< impact peak is a LOWER bound (IMU full scale reached)
    Battery           = 1u << 6,
    PositionStale     = 1u << 7,   ///< the fix is older than kSosStaleFixMs
    IncidentIdVolatile= 1u << 8,   ///< incident ids are not persistent (may repeat after a reset)
    Restored          = 1u << 9,   ///< re-issued after a reset
};
}  // namespace sosvalid
constexpr uint32_t kSosStaleFixMs = 60000u;

struct SosRequest {
    uint8_t version;              ///< kSosProtocolVersion
    SosRequestKind kind;
    IncidentTrigger trigger;
    EmergencyState state;         ///< firmware state when built
    uint32_t incidentId;
    uint16_t revision;            ///< 1, 2, ... per incident
    uint16_t validity;            ///< sosvalid::*
    uint32_t createdUptimeMs;     ///< MCU time the request (revision) was built
    uint32_t eventUptimeMs;       ///< MCU time of the incident (impact / manual command)
    int32_t latitude_1e7deg;
    int32_t longitude_1e7deg;
    int32_t heightMsl_dm;
    uint16_t horizontalAccuracy_dm;
    uint8_t fixType;
    uint8_t numSatellites;
    uint32_t fixAgeMs;            ///< createdUptimeMs - fix time (UINT32_MAX = no fix)
    int32_t relativeAltitude_dm;
    int16_t verticalSpeed_cmps;   ///< positive up
    uint16_t peakAccel_cg;        ///< corroborated impact peak, 0.01 g units
    uint16_t peakRate_dps;
    uint8_t impactClass;          ///< pg::ImpactClass
    uint8_t impacts;
    uint32_t crashEvidence;       ///< crashev::* bits
    uint16_t batteryMillivolts;   ///< 0 = unknown
    uint32_t deviceHealthFlags;   ///< ProcessedSensorFrame quality::* of the newest frame (low 16 bits encoded)
};
static_assert(std::is_trivially_copyable<SosRequest>::value, "request must be memcpy-safe");

/// CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF). Bitwise: small, no table.
uint32_t crc32Ieee(const uint8_t* data, size_t len);

/// Encodes into exactly kSosFrameBytes. Returns the length, or 0 if @p cap is too small / invalid kind.
size_t encodeSosRequest(const SosRequest& r, uint8_t* dst, size_t cap);
/// Decodes and checks version + CRC. Returns false on any mismatch (out untouched on failure).
bool decodeSosRequest(const uint8_t* src, size_t len, SosRequest& out);

/// Maps onto the existing (unversioned) SosPayload of Communications/Bluetooth/SosMessage.hpp.
SosPayload toLegacySosPayload(const SosRequest& r);

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_HPP
