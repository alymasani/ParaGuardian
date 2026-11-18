/**
 ******************************************************************************
 * @file    NeoM9n.cpp
 * @brief   NAV-PVT decoding, CFG-VALSET building and stream handling.
 ******************************************************************************
 */
#include "Sensors/NEOM9N/NeoM9n.hpp"

#include "Common/ByteOrder.hpp"

namespace pg {
namespace ubx {

bool decodeNavPvt(const Frame& frame, NavPvt& out)
{
    if (frame.msgClass != CLASS_NAV || frame.msgId != ID_NAV_PVT || frame.length != LEN_NAV_PVT ||
        frame.payload == nullptr) {
        return false;
    }
    const uint8_t* p = frame.payload;
    // Byte offsets from the NAV-PVT payload table in the interface description.
    out.iTOW_ms = bytes::le_u32(p + 0);
    out.year = bytes::le_u16(p + 4);
    out.month = p[6];
    out.day = p[7];
    out.hour = p[8];
    out.min = p[9];
    out.sec = p[10];
    out.valid = p[11];
    out.tAcc_ns = bytes::le_u32(p + 12);
    out.nano_ns = bytes::le_i32(p + 16);
    out.fixType = p[20];
    out.flags = p[21];
    out.flags2 = p[22];
    out.numSV = p[23];
    out.lon_1e7deg = bytes::le_i32(p + 24);
    out.lat_1e7deg = bytes::le_i32(p + 28);
    out.height_mm = bytes::le_i32(p + 32);
    out.hMSL_mm = bytes::le_i32(p + 36);
    out.hAcc_mm = bytes::le_u32(p + 40);
    out.vAcc_mm = bytes::le_u32(p + 44);
    out.velN_mmps = bytes::le_i32(p + 48);
    out.velE_mmps = bytes::le_i32(p + 52);
    out.velD_mmps = bytes::le_i32(p + 56);
    out.gSpeed_mmps = bytes::le_i32(p + 60);
    out.headMot_1e5deg = bytes::le_i32(p + 64);
    out.sAcc_mmps = bytes::le_u32(p + 68);
    out.headAcc_1e5deg = bytes::le_u32(p + 72);
    out.pDOP_001 = bytes::le_u16(p + 76);
    out.flags3 = bytes::le_u16(p + 78);
    return true;
}

void navPvtToSample(const NavPvt& pvt, GnssSample& out)
{
    const bool gnssFixOk = (pvt.flags & 0x01u) != 0u;
    const bool invalidLlh = (pvt.flags3 & 0x0001u) != 0u;
    const GnssFixType fix = (pvt.fixType <= 5u) ? static_cast<GnssFixType>(pvt.fixType) : GnssFixType::NoFix;
    const bool has3d = (fix == GnssFixType::Fix3D) || (fix == GnssFixType::GnssPlusDeadReckoning);
    const bool has2dOr3d = has3d || (fix == GnssFixType::Fix2D);

    out.latitude_1e7deg = pvt.lat_1e7deg;
    out.longitude_1e7deg = pvt.lon_1e7deg;
    out.heightMsl_m = static_cast<float>(pvt.hMSL_mm) * 1e-3f;
    out.heightEllipsoid_m = static_cast<float>(pvt.height_mm) * 1e-3f;
    out.velocityNorth_mps = static_cast<float>(pvt.velN_mmps) * 1e-3f;
    out.velocityEast_mps = static_cast<float>(pvt.velE_mmps) * 1e-3f;
    out.verticalVelocity_mps = -static_cast<float>(pvt.velD_mmps) * 1e-3f;  // NED down -> up
    out.groundSpeed_mps = static_cast<float>(pvt.gSpeed_mmps) * 1e-3f;
    out.horizontalAccuracy_m = static_cast<float>(pvt.hAcc_mm) * 1e-3f;
    out.verticalAccuracy_m = static_cast<float>(pvt.vAcc_mm) * 1e-3f;
    out.speedAccuracy_mps = static_cast<float>(pvt.sAcc_mmps) * 1e-3f;
    out.pDop = static_cast<float>(pvt.pDOP_001) * 0.01f;
    out.gnssTimeOfWeek_ms = pvt.iTOW_ms;
    out.utcYear = pvt.year;
    out.utcMonth = pvt.month;
    out.utcDay = pvt.day;
    out.utcHour = pvt.hour;
    out.utcMinute = pvt.min;
    out.utcSecond = pvt.sec;
    out.numSatellites = pvt.numSV;
    out.fixType = fix;
    out.gnssFixOk = gnssFixOk;
    // Validity rules: only trust position when the receiver itself says the
    // fix is OK (gnssFixOK), the fix type actually provides position (2D/3D),
    // and the lat/lon/height are not flagged invalid. Height needs a 3D fix.
    out.positionValid = gnssFixOk && has2dOr3d && !invalidLlh;
    out.heightValid = gnssFixOk && has3d && !invalidLlh;
    out.utcDateValid = (pvt.valid & 0x01u) != 0u;
    out.utcTimeValid = (pvt.valid & 0x02u) != 0u;
    out.header.valid = out.positionValid;
}

size_t keyValueSize(uint32_t key)
{
    switch ((key >> 28) & 0x7u) {
        case 0x1u: return 1u;  // one bit, transmitted as one byte
        case 0x2u: return 1u;
        case 0x3u: return 2u;
        case 0x4u: return 4u;
        default: return 0u;    // 8-byte values not needed / unsupported here
    }
}

size_t buildValSet(const ConfigItem* items, size_t count, uint8_t layers, uint8_t* dst, size_t dstCapacity)
{
    if (items == nullptr || count == 0u) {
        return 0u;
    }
    // VALSET payload: version(1)=0, layers(1), reserved(2), then key(4 LE) + value(N LE) pairs.
    uint8_t payload[64] = {};
    size_t len = 0u;
    payload[len++] = 0x00u;
    payload[len++] = layers;
    payload[len++] = 0x00u;
    payload[len++] = 0x00u;
    for (size_t i = 0; i < count; ++i) {
        const size_t valueSize = keyValueSize(items[i].key);
        if (valueSize == 0u || len + 4u + valueSize > sizeof(payload)) {
            return 0u;
        }
        bytes::put_le_u32(&payload[len], items[i].key);
        len += 4u;
        for (size_t b = 0; b < valueSize; ++b) {
            payload[len++] = static_cast<uint8_t>((items[i].value >> (8u * b)) & 0xFFu);
        }
    }
    return buildFrame(CLASS_CFG, ID_CFG_VALSET, payload, static_cast<uint16_t>(len), dst, dstCapacity);
}

}  // namespace ubx

// ---------------------------------------------------------------------------
size_t NeoM9n::processBytes(const uint8_t* data, size_t len, uint32_t nowMs)
{
    if (data == nullptr) {
        return 0u;
    }
    const uint32_t ckBefore = parser_.stats().checksumErrors;
    size_t solutions = 0u;
    for (size_t i = 0; i < len; ++i) {
        ubx::Frame frame{};
        if (parser_.feed(data[i], nowMs, frame)) {
            const uint32_t before = sequence_;
            handleFrame(frame, nowMs);
            if (sequence_ != before) {
                ++solutions;
            }
        }
    }
    // Account checksum failures as invalid data on the health record.
    for (uint32_t k = ckBefore; k < parser_.stats().checksumErrors; ++k) {
        health_.recordFailure(Status::ChecksumError);
    }
    return solutions;
}

void NeoM9n::handleFrame(const ubx::Frame& frame, uint32_t nowMs)
{
    if (frame.msgClass == ubx::CLASS_NAV && frame.msgId == ubx::ID_NAV_PVT) {
        ubx::NavPvt pvt{};
        if (!ubx::decodeNavPvt(frame, pvt)) {
            health_.recordFailure(Status::InvalidData);  // wrong length for NAV-PVT
            return;
        }
        ubx::navPvtToSample(pvt, latest_);
        latest_.header.id = id_;
        latest_.header.timestampMs = nowMs;
        latest_.header.sequence = ++sequence_;
        haveSolution_ = true;
        lastSolutionMs_ = nowMs;
        health_.initialized = true;  // receiver is talking UBX to us
        health_.recordSuccess(nowMs);
        return;
    }
    if (frame.msgClass == ubx::CLASS_ACK && frame.length == 2u && frame.payload[0] == ubx::CLASS_CFG) {
        lastAck_ = (frame.msgId == ubx::ID_ACK_ACK) ? GnssConfigAck::Ack : GnssConfigAck::Nak;
    }
}

void NeoM9n::poll(uint32_t nowMs)
{
    parser_.checkTimeout(nowMs);
    if (haveSolution_ && !hasFreshSolution(nowMs) && latest_.header.valid) {
        latest_.header.valid = false;  // stale data must not be used
        health_.recordFailure(Status::Timeout);
    }
}

bool NeoM9n::hasFreshSolution(uint32_t nowMs) const
{
    return haveSolution_ && (nowMs - lastSolutionMs_) <= staleTimeoutMs_;
}

size_t NeoM9n::buildEnableNavPvtFrame(uint16_t measurementPeriodMs, bool disableNmea, uint8_t* dst,
                                      size_t dstCapacity)
{
    if (measurementPeriodMs < 40u) {  // NEO-M9N datasheet: max navigation update rate 25 Hz (= 40 ms)
        return 0u;
    }
    const ubx::ConfigItem items[] = {
        {ubx::KEY_CFG_UART1OUTPROT_UBX, 1u},
        {ubx::KEY_CFG_UART1OUTPROT_NMEA, disableNmea ? 0u : 1u},
        {ubx::KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1u},
        {ubx::KEY_CFG_RATE_MEAS, measurementPeriodMs},
    };
    return ubx::buildValSet(items, sizeof(items) / sizeof(items[0]), ubx::VALSET_LAYER_RAM, dst, dstCapacity);
}

}  // namespace pg
