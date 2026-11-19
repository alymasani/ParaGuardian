/**
 ******************************************************************************
 * @file    SosRequest.cpp
 * @brief   See SosRequest.hpp. Fixed layout (offsets in bytes):
 *
 *   0 version  1 kind  2 trigger  3 state  4 incidentId(4)  8 revision(2)
 *  10 validity(2)  12 createdUptimeMs(4)  16 eventUptimeMs(4)  20 lat(4)
 *  24 lon(4)  28 heightMsl_dm(4)  32 hAcc_dm(2)  34 fixType  35 numSv
 *  36 fixAgeMs(4)  40 relAlt_dm(4)  44 vz_cmps(2)  46 peakAccel_cg(2)
 *  48 peakRate_dps(2)  50 impactClass  51 impacts  52 crashEvidence(4)
 *  56 battery_mV(2)  58 healthFlags low 16 bits(2)  60 CRC-32 over [0, 60)(4)
 *  = 64 bytes. (quality:: bits 0..15 cover every sensor-group flag the phone
 *  needs; the full 32-bit word stays in the firmware's health snapshot.)
 ******************************************************************************
 */
#include "Algorithms/Emergency/SosRequest.hpp"

namespace pg {

namespace {
constexpr size_t kCrcOffset = kSosFrameBytes - 4u;   // 60: CRC over bytes [0, 60)
static_assert(kCrcOffset == 60u, "layout");

void put8(uint8_t* p, size_t o, uint8_t v) { p[o] = v; }
void put16(uint8_t* p, size_t o, uint16_t v)
{
    p[o] = static_cast<uint8_t>(v);
    p[o + 1u] = static_cast<uint8_t>(v >> 8);
}
void put32(uint8_t* p, size_t o, uint32_t v)
{
    for (size_t i = 0; i < 4u; ++i) p[o + i] = static_cast<uint8_t>(v >> (8u * i));
}
uint16_t get16(const uint8_t* p, size_t o)
{
    return static_cast<uint16_t>(p[o] | (static_cast<uint16_t>(p[o + 1u]) << 8));
}
uint32_t get32(const uint8_t* p, size_t o)
{
    uint32_t v = 0u;
    for (size_t i = 0; i < 4u; ++i) v |= static_cast<uint32_t>(p[o + i]) << (8u * i);
    return v;
}
}  // namespace

uint32_t crc32Ieee(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xFFFFFFFFu;
}

size_t encodeSosRequest(const SosRequest& r, uint8_t* dst, size_t cap)
{
    if (dst == nullptr || cap < kSosFrameBytes) return 0u;
    if (r.kind != SosRequestKind::Alert && r.kind != SosRequestKind::Cancel) return 0u;
    for (size_t i = 0; i < kSosFrameBytes; ++i) dst[i] = 0u;
    put8(dst, 0, r.version);
    put8(dst, 1, static_cast<uint8_t>(r.kind));
    put8(dst, 2, static_cast<uint8_t>(r.trigger));
    put8(dst, 3, static_cast<uint8_t>(r.state));
    put32(dst, 4, r.incidentId);
    put16(dst, 8, r.revision);
    put16(dst, 10, r.validity);
    put32(dst, 12, r.createdUptimeMs);
    put32(dst, 16, r.eventUptimeMs);
    put32(dst, 20, static_cast<uint32_t>(r.latitude_1e7deg));
    put32(dst, 24, static_cast<uint32_t>(r.longitude_1e7deg));
    put32(dst, 28, static_cast<uint32_t>(r.heightMsl_dm));
    put16(dst, 32, r.horizontalAccuracy_dm);
    put8(dst, 34, r.fixType);
    put8(dst, 35, r.numSatellites);
    put32(dst, 36, r.fixAgeMs);
    put32(dst, 40, static_cast<uint32_t>(r.relativeAltitude_dm));
    put16(dst, 44, static_cast<uint16_t>(r.verticalSpeed_cmps));
    put16(dst, 46, r.peakAccel_cg);
    put16(dst, 48, r.peakRate_dps);
    put8(dst, 50, r.impactClass);
    put8(dst, 51, r.impacts);
    put32(dst, 52, r.crashEvidence);
    put16(dst, 56, r.batteryMillivolts);
    put16(dst, 58, static_cast<uint16_t>(r.deviceHealthFlags & 0xFFFFu));   // low 16 quality bits
    put32(dst, kCrcOffset, crc32Ieee(dst, kCrcOffset));
    return kSosFrameBytes;
}

bool decodeSosRequest(const uint8_t* src, size_t len, SosRequest& out)
{
    if (src == nullptr || len != kSosFrameBytes) return false;
    if (get32(src, kCrcOffset) != crc32Ieee(src, kCrcOffset)) return false;
    if (src[0] != kSosProtocolVersion) return false;
    if (src[1] != static_cast<uint8_t>(SosRequestKind::Alert) && src[1] != static_cast<uint8_t>(SosRequestKind::Cancel))
        return false;
    SosRequest r{};
    r.version = src[0];
    r.kind = static_cast<SosRequestKind>(src[1]);
    r.trigger = static_cast<IncidentTrigger>(src[2]);
    r.state = static_cast<EmergencyState>(src[3]);
    r.incidentId = get32(src, 4);
    r.revision = get16(src, 8);
    r.validity = get16(src, 10);
    r.createdUptimeMs = get32(src, 12);
    r.eventUptimeMs = get32(src, 16);
    r.latitude_1e7deg = static_cast<int32_t>(get32(src, 20));
    r.longitude_1e7deg = static_cast<int32_t>(get32(src, 24));
    r.heightMsl_dm = static_cast<int32_t>(get32(src, 28));
    r.horizontalAccuracy_dm = get16(src, 32);
    r.fixType = src[34];
    r.numSatellites = src[35];
    r.fixAgeMs = get32(src, 36);
    r.relativeAltitude_dm = static_cast<int32_t>(get32(src, 40));
    r.verticalSpeed_cmps = static_cast<int16_t>(get16(src, 44));
    r.peakAccel_cg = get16(src, 46);
    r.peakRate_dps = get16(src, 48);
    r.impactClass = src[50];
    r.impacts = src[51];
    r.crashEvidence = get32(src, 52);
    r.batteryMillivolts = get16(src, 56);
    r.deviceHealthFlags = get16(src, 58);
    out = r;
    return true;
}

SosPayload toLegacySosPayload(const SosRequest& r)
{
    SosPayload p{};
    switch (r.trigger) {
        case IncidentTrigger::Manual: p.reason = SosReason::ManualTrigger; break;
        case IncidentTrigger::AutomaticCrashSuspected:
        case IncidentTrigger::DetectorSilence:
            p.reason = ((r.crashEvidence & (1u << 16)) != 0u) ? SosReason::NoMotionAfterImpact : SosReason::ImpactDetected;
            break;
        default: p.reason = SosReason::Unknown; break;
    }
    p.positionValid = (r.validity & sosvalid::Position) != 0u;
    p.latitude_1e7deg = p.positionValid ? r.latitude_1e7deg : 0;
    p.longitude_1e7deg = p.positionValid ? r.longitude_1e7deg : 0;
    p.altitudeMsl_dm = (r.validity & sosvalid::HeightMsl) != 0u ? r.heightMsl_dm : 0;   // MSL only, never the baro altitude
    p.batteryMillivolts = (r.validity & sosvalid::Battery) != 0u ? r.batteryMillivolts : 0u;
    p.uptimeSeconds = r.createdUptimeMs / 1000u;
    return p;
}

}  // namespace pg
