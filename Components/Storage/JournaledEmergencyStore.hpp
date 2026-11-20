/**
 ******************************************************************************
 * @file    JournaledEmergencyStore.hpp
 * @brief   IEmergencyStore backed by the EmergencyJournal, written
 *          asynchronously by the storage task.
 *
 *   EmergencyAlertTask                      StorageTask (low priority)
 *   save(record) -> copy into pending_  ->  takePending() -> journal.commit()
 *                   returns Queued+ticket    reportCommitted(ticket) / reportFailed(ticket)
 *   committedTicket() <------------------------------------------'
 *
 * - save() NEVER touches flash and never waits: a 32-byte copy under a
 *   critical section. The emergency task therefore never blocks on an erase.
 * - COALESCING: only the newest pending record is kept. It contains every
 *   older transition's information, so committing it commits them all
 *   (committedTicket jumps to its ticket). Nothing is silently lost: a
 *   superseded record is counted in stats().coalesced.
 * - "Durable" means committedTicket() >= ticket. Queued is never reported
 *   as saved (PersistAck::Queued).
 * - available() is false until the storage task has mounted the journal; the
 *   emergency task waits (bounded) for mounted() before restore().
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_JOURNALED_EMERGENCY_STORE_HPP
#define PARAGUARDIAN_STORAGE_JOURNALED_EMERGENCY_STORE_HPP

#include <cstdint>
#include <cstring>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Emergency/IEmergencyStore.hpp"
#include "Storage/EmergencyJournal.hpp"

namespace pg {

struct EmergencyStoreStats {
    uint32_t saves;
    uint32_t coalesced;        ///< pending record replaced by a newer one before it was written
    uint32_t commits;
    uint32_t failures;         ///< reportFailed() calls (commit attempts exhausted for a ticket)
    uint32_t rejected;
};

class JournaledEmergencyStore final : public IEmergencyStore {
public:
    static_assert(kEmergencyRecordBytes == journal::kPayloadBytes, "record must fill one journal payload");

    explicit JournaledEmergencyStore(ICriticalSection& lock) : lock_(lock) {}

    // ---- emergency task side (IEmergencyStore) ----
    bool available() const override
    {
        Guard g(lock_);
        return mounted_ && mount_ != StoreMount::Unavailable;
    }
    StoreMount mountState() const override
    {
        Guard g(lock_);
        return mount_;
    }
    bool load(uint8_t* dst, size_t len) override { return copyOut(haveLatest_, latest_, dst, len); }
    bool loadUncommitted(uint8_t* dst, size_t len) override { return copyOut(haveUnc_, unc_, dst, len); }
    PersistAck save(const uint8_t* src, size_t len, uint32_t& ticket) override
    {
        Guard g(lock_);
        ticket = 0u;
        if (!mounted_ || mount_ == StoreMount::Unavailable || src == nullptr || len != kEmergencyRecordBytes) {
            inc(stats_.rejected);
            return PersistAck::Rejected;
        }
        if (hasPending_) inc(stats_.coalesced);
        std::memcpy(pending_, src, len);
        hasPending_ = true;
        ticket = ++ticket_ == 0u ? ++ticket_ : ticket_;   // 0 means "none"
        pendingTicket_ = ticket;
        inc(stats_.saves);
        return PersistAck::Queued;
    }
    uint32_t committedTicket() const override
    {
        Guard g(lock_);
        return committed_;
    }
    uint32_t failedTicket() const override
    {
        Guard g(lock_);
        return failed_;
    }

    // ---- storage task side ----
    /// Publishes the mount result (also when Unavailable) and the recovered records.
    void publishMount(const EmergencyJournal& j)
    {
        Guard g(lock_);
        switch (j.mountState()) {
            case JournalMount::Blank: mount_ = StoreMount::Blank; break;
            case JournalMount::Recovered: mount_ = StoreMount::Recovered; break;
            case JournalMount::Corrupt: mount_ = StoreMount::Corrupt; break;
            default: mount_ = StoreMount::Unavailable; break;
        }
        haveLatest_ = j.latest(latest_, sizeof(latest_));
        haveUnc_ = j.uncommitted(unc_, sizeof(unc_));
        mounted_ = true;
    }
    bool mounted() const
    {
        Guard g(lock_);
        return mounted_;
    }
    bool hasPending() const
    {
        Guard g(lock_);
        return hasPending_;
    }
    /// Copies the newest pending record (it stays pending until reportCommitted() with its ticket).
    bool takePending(uint8_t* dst, uint32_t& ticket) const
    {
        Guard g(lock_);
        if (!hasPending_) return false;
        std::memcpy(dst, pending_, kEmergencyRecordBytes);
        ticket = pendingTicket_;
        return true;
    }
    void reportCommitted(uint32_t ticket)
    {
        Guard g(lock_);
        committed_ = ticket;
        if (ticket == pendingTicket_) hasPending_ = false;   // a newer save() keeps it pending
        inc(stats_.commits);
    }
    void reportFailed(uint32_t ticket)
    {
        Guard g(lock_);
        failed_ = ticket;
        inc(stats_.failures);
    }
    EmergencyStoreStats stats() const
    {
        Guard g(lock_);
        return stats_;
    }

private:
    struct Guard {
        explicit Guard(ICriticalSection& l) : l_(l) { l_.enter(); }
        ~Guard() { l_.exit(); }
        ICriticalSection& l_;
    };
    static void inc(uint32_t& c)
    {
        if (c != UINT32_MAX) ++c;
    }
    bool copyOut(const bool& have, const uint8_t* src, uint8_t* dst, size_t len) const
    {
        Guard g(lock_);
        if (!have || dst == nullptr || len != kEmergencyRecordBytes) return false;
        std::memcpy(dst, src, len);
        return true;
    }

    ICriticalSection& lock_;
    bool mounted_ = false;
    StoreMount mount_ = StoreMount::Unavailable;
    bool haveLatest_ = false;
    bool haveUnc_ = false;
    uint8_t latest_[kEmergencyRecordBytes] = {};
    uint8_t unc_[kEmergencyRecordBytes] = {};
    bool hasPending_ = false;
    uint8_t pending_[kEmergencyRecordBytes] = {};
    uint32_t pendingTicket_ = 0u;
    uint32_t ticket_ = 0u;
    uint32_t committed_ = 0u;
    uint32_t failed_ = 0u;
    EmergencyStoreStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_JOURNALED_EMERGENCY_STORE_HPP
