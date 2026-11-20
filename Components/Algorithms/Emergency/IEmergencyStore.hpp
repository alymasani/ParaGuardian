/**
 ******************************************************************************
 * @file    IEmergencyStore.hpp
 * @brief   Minimal incident state that must survive an MCU reset, its
 *          integrity-checked encoding, and the (asynchronous) storage
 *          interface.
 *
 * DURABILITY IS EXPLICIT (Milestone 8). save() never blocks; it returns
 *   Rejected   the store cannot take the record (unavailable / bad length)
 *   Queued     accepted into RAM; NOT yet on flash (a reset now may lose it)
 *   Committed  durably committed before save() returned (RAM test doubles only)
 * plus a TICKET. The record is durable only once committedTicket() has
 * reached that ticket. Tickets increase by one per save(); a later ticket
 * supersedes earlier ones (the store may coalesce: only the newest pending
 * record is written, which contains everything the older ones did).
 *
 * Firmware backend: Storage/JournaledEmergencyStore (W25Q128 journal written
 * by StorageTask). Host tests: Tests/mocks/MockEmergencyIo.hpp.
 *
 * Record (kEmergencyRecordBytes = 32, version 2): magic "PGER", version,
 * state, trigger, stage, nextIncidentId, incidentId, requestRevision,
 * reportedStages, flags, cancelNotice, countdownRestarts, identityFlags,
 * reserved, CRC-32. Version 1 records (Milestone 6) still decode.
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
constexpr uint8_t kEmergencyRecordVersion = 2u;
constexpr uint8_t kEmergencyRecordVersionV1 = 1u;

namespace identityflag {
enum : uint8_t {
    Uncertain = 1u << 0,   ///< ids were (re)started without a trustworthy record (blank / corrupt journal)
    Exhausted = 1u << 1,   ///< the 32-bit id space was used up and wrapped (device must be re-provisioned)
};
}

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
    uint8_t countdownRestarts;   ///< resets that restarted the SAME countdown (reboot-loop guard)
    uint8_t identityFlags;       ///< identityflag::*
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
    d[22] = r.countdownRestarts;
    d[23] = r.identityFlags;
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
    if (get32(0) != kEmergencyRecordMagic) return false;
    if (s[4] != kEmergencyRecordVersion && s[4] != kEmergencyRecordVersionV1) return false;
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
    if (s[4] == kEmergencyRecordVersion) {   // v1 wrote zeros here
        r.countdownRestarts = s[22];
        r.identityFlags = s[23];
    }
    out = r;
    return true;
}

enum class PersistAck : uint8_t {
    Rejected = 0,   ///< not accepted (store unavailable / invalid record)
    Queued,         ///< accepted into RAM, not durable yet: poll committedTicket()
    Committed,      ///< durable before save() returned (synchronous RAM doubles)
};

/// What the store found when it was opened (reported, never hidden).
enum class StoreMount : uint8_t {
    Unavailable = 0,   ///< no medium (not configured / flash not responding)
    Blank,             ///< medium erased: first use (ids legitimately start at 1)
    Recovered,         ///< a committed record was found
    Corrupt,           ///< data present but no valid committed record: identity uncertain
};

/// Byte storage for one record. Implementations must never block the caller.
class IEmergencyStore {
public:
    virtual bool available() const = 0;
    virtual StoreMount mountState() const { return available() ? StoreMount::Recovered : StoreMount::Unavailable; }
    /// Last COMMITTED record (false = none / unavailable).
    virtual bool load(uint8_t* dst, size_t len) = 0;
    /// An intact record NEWER than the committed one whose commit marker is missing
    /// (power lost between programming the body and the commit word). Never used as
    /// state; used only to keep incident ids monotonic. Default: none.
    virtual bool loadUncommitted(uint8_t* dst, size_t len)
    {
        (void)dst;
        (void)len;
        return false;
    }
    virtual PersistAck save(const uint8_t* src, size_t len, uint32_t& ticket) = 0;
    /// Highest ticket durably committed (0 = none). Wrap-safe comparison by the caller.
    virtual uint32_t committedTicket() const = 0;
    /// Highest ticket whose commit attempts were exhausted (0 = none). A later commit clears the condition.
    virtual uint32_t failedTicket() const { return 0u; }

protected:
    ~IEmergencyStore() = default;
};

/// Firmware fallback: no persistence (reported as such in EmergencyStatus / g_systemHealth).
class NullEmergencyStore final : public IEmergencyStore {
public:
    bool available() const override { return false; }
    bool load(uint8_t*, size_t) override { return false; }
    PersistAck save(const uint8_t*, size_t, uint32_t& ticket) override
    {
        ticket = 0u;
        return PersistAck::Rejected;
    }
    uint32_t committedTicket() const override { return 0u; }
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_IEMERGENCY_STORE_HPP
