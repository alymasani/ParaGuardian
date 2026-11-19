/**
 ******************************************************************************
 * @file    IEmergencyStore.hpp
 * @brief   Minimal incident state that should survive an MCU reset, its
 *          integrity-checked encoding, and the storage interface.
 *
 * STATUS: ONLY THE INTERFACE AND A HOST MOCK EXIST. The firmware uses
 * NullEmergencyStore (available() == false), so today the firmware CANNOT
 * continue an SOS across a reset: RAM does not survive power loss, and the
 * W25Q128 flash wiring / write timing are unverified. A flash backend
 * belongs to the logging milestone and must write asynchronously (never from
 * the processing path; a sector erase takes tens of ms).
 *
 * Record (kEmergencyRecordBytes = 32): magic "PGER", version, state, trigger,
 * stage, nextIncidentId, incidentId, requestRevision, reportedStages, flags,
 * cancelNotice, reserved, CRC-32.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_IEMERGENCY_STORE_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_IEMERGENCY_STORE_HPP

#include <cstddef>
#include <cstdint>

#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Emergency/SosRequest.hpp"   // crc32Ieee

namespace pg {

constexpr size_t kEmergencyRecordBytes = 32u;
constexpr uint32_t kEmergencyRecordMagic = 0x52454750u;   // "PGER" little-endian
constexpr uint8_t kEmergencyRecordVersion = 1u;

struct EmergencyPersistRecord {
    uint32_t nextIncidentId;
    uint32_t incidentId;
    EmergencyState state;
    IncidentTrigger trigger;
    DeliveryStage stage;
    uint16_t requestRevision;
    uint16_t reportedStages;
    bool requestIssued;
    bool requestLeftDevice;
    bool cancelledAfterSend;
    CancelNotice cancelNotice;
};

inline size_t encodeEmergencyRecord(const EmergencyPersistRecord& r, uint8_t* d, size_t cap)
{
    if (d == nullptr || cap < kEmergencyRecordBytes) return 0u;
    for (size_t i = 0; i < kEmergencyRecordBytes; ++i) d[i] = 0u;
    auto put32 = [d](size_t o, uint32_t v) {
        for (size_t i = 0; i < 4u; ++i) d[o + i] = static_cast<uint8_t>(v >> (8u * i));
    };
    put32(0, kEmergencyRecordMagic);
    d[4] = kEmergencyRecordVersion;
    d[5] = static_cast<uint8_t>(r.state);
    d[6] = static_cast<uint8_t>(r.trigger);
    d[7] = static_cast<uint8_t>(r.stage);
    put32(8, r.nextIncidentId);
    put32(12, r.incidentId);
    d[16] = static_cast<uint8_t>(r.requestRevision);
    d[17] = static_cast<uint8_t>(r.requestRevision >> 8);
    d[18] = static_cast<uint8_t>(r.reportedStages);
    d[19] = static_cast<uint8_t>(r.reportedStages >> 8);
    d[20] = static_cast<uint8_t>((r.requestIssued ? 1u : 0u) | (r.requestLeftDevice ? 2u : 0u) |
                                 (r.cancelledAfterSend ? 4u : 0u));
    d[21] = static_cast<uint8_t>(r.cancelNotice);
    put32(28, crc32Ieee(d, 28u));
    return kEmergencyRecordBytes;
}

inline bool decodeEmergencyRecord(const uint8_t* s, size_t len, EmergencyPersistRecord& out)
{
    if (s == nullptr || len != kEmergencyRecordBytes) return false;
    auto get32 = [s](size_t o) {
        uint32_t v = 0u;
        for (size_t i = 0; i < 4u; ++i) v |= static_cast<uint32_t>(s[o + i]) << (8u * i);
        return v;
    };
    if (get32(0) != kEmergencyRecordMagic || s[4] != kEmergencyRecordVersion) return false;
    if (get32(28) != crc32Ieee(s, 28u)) return false;
    if (s[5] > static_cast<uint8_t>(EmergencyState::Unavailable) || s[6] > static_cast<uint8_t>(IncidentTrigger::Restored) ||
        s[7] > static_cast<uint8_t>(DeliveryStage::DeliveryConfirmed) || s[21] > static_cast<uint8_t>(CancelNotice::Failed)) {
        return false;
    }
    EmergencyPersistRecord r{};
    r.state = static_cast<EmergencyState>(s[5]);
    r.trigger = static_cast<IncidentTrigger>(s[6]);
    r.stage = static_cast<DeliveryStage>(s[7]);
    r.nextIncidentId = get32(8);
    r.incidentId = get32(12);
    r.requestRevision = static_cast<uint16_t>(s[16] | (s[17] << 8));
    r.reportedStages = static_cast<uint16_t>(s[18] | (s[19] << 8));
    r.requestIssued = (s[20] & 1u) != 0u;
    r.requestLeftDevice = (s[20] & 2u) != 0u;
    r.cancelledAfterSend = (s[20] & 4u) != 0u;
    r.cancelNotice = static_cast<CancelNotice>(s[21]);
    out = r;
    return true;
}

/// Byte storage for one record. Implementations must not block the caller for long.
class IEmergencyStore {
public:
    virtual bool available() const = 0;
    virtual bool load(uint8_t* dst, size_t len) = 0;
    virtual bool save(const uint8_t* src, size_t len) = 0;

protected:
    ~IEmergencyStore() = default;
};

/// Firmware default: no persistence (reported as such in EmergencyStatus / g_systemHealth).
class NullEmergencyStore final : public IEmergencyStore {
public:
    bool available() const override { return false; }
    bool load(uint8_t*, size_t) override { return false; }
    bool save(const uint8_t*, size_t) override { return false; }
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_IEMERGENCY_STORE_HPP
