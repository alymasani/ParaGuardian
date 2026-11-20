/**
 ******************************************************************************
 * @file    LogRecords.cpp
 * @brief   Log record framing (see LogRecords.hpp).
 ******************************************************************************
 */
#include "Storage/LogRecords.hpp"

#include "Algorithms/Emergency/SosRequest.hpp"   // crc32Ieee

namespace pg {

namespace {
void put32(uint8_t* d, uint32_t v)
{
    for (size_t i = 0; i < 4u; ++i) d[i] = static_cast<uint8_t>(v >> (8u * i));
}
uint32_t get32(const uint8_t* s)
{
    uint32_t v = 0u;
    for (size_t i = 0; i < 4u; ++i) v |= static_cast<uint32_t>(s[i]) << (8u * i);
    return v;
}
}  // namespace

size_t encodeLogRecord(const LogEntry& e, uint32_t sequence, uint8_t* out, size_t cap)
{
    const size_t total = kLogHeaderBytes + e.length + kLogCrcBytes;
    if (out == nullptr || e.length > kLogMaxPayload || e.type == LogType::Invalid ||
        static_cast<uint8_t>(e.type) >= static_cast<uint8_t>(LogType::Count) || cap < total) {
        return 0u;
    }
    out[0] = kLogSync;
    out[1] = static_cast<uint8_t>(e.type);
    out[2] = e.length;
    out[3] = kLogSchemaVersion;
    put32(&out[4], sequence);
    put32(&out[8], e.timestampMs);
    std::memcpy(&out[kLogHeaderBytes], e.payload, e.length);
    put32(&out[kLogHeaderBytes + e.length], crc32Ieee(out, kLogHeaderBytes + e.length));
    return total;
}

size_t decodeLogRecord(const uint8_t* in, size_t avail, LogEntry& e, uint32_t& sequence)
{
    if (in == nullptr || avail < kLogHeaderBytes + kLogCrcBytes || in[0] != kLogSync) return 0u;
    const size_t len = in[2];
    const size_t total = kLogHeaderBytes + len + kLogCrcBytes;
    if (len > kLogMaxPayload || total > avail || in[3] != kLogSchemaVersion || in[1] == 0u ||
        in[1] >= static_cast<uint8_t>(LogType::Count)) {
        return 0u;
    }
    if (get32(&in[kLogHeaderBytes + len]) != crc32Ieee(in, kLogHeaderBytes + len)) return 0u;
    e.type = static_cast<LogType>(in[1]);
    e.length = static_cast<uint8_t>(len);
    sequence = get32(&in[4]);
    e.timestampMs = get32(&in[8]);
    std::memcpy(e.payload, &in[kLogHeaderBytes], len);
    return total;
}

}  // namespace pg
