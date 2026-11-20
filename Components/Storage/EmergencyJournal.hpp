/**
 ******************************************************************************
 * @file    EmergencyJournal.hpp
 * @brief   Power-loss-safe, append-only journal of 32-byte emergency records
 *          on SPI NOR flash (two or more 4 KiB sectors used as a ring).
 *
 * SLOT (64 bytes, 4 per 256-byte page, never crosses a page):
 *    0  magic "PGJ1"           u32   (0x314A4750, little endian)
 *    4  layout version         u8    (1)
 *    5  record type            u8    (1 = emergency state)
 *    6  payload length         u8    (32)
 *    7  reserved               u8    (0)
 *    8  sequence               u32   (generation: 1, 2, ...; 0 and 0xFFFFFFFF never used)
 *   12  boot count             u32   (recovery metadata: mounts that found this journal)
 *   16  payload               32 B   (EmergencyPersistRecord, with its own CRC)
 *   48  writer uptime ms       u32   (recovery metadata, MCU clock)
 *   52  reserved               u32   (0)
 *   56  CRC-32 of bytes 0..55  u32   (accidental corruption only; NOT authentication)
 *   60  commit word            u32   (0x3CC3A55A, programmed LAST)
 *
 * WHY THIS IS SAFE AGAINST POWER LOSS
 *   - Write order: program bytes 0..59, read back and compare, THEN program the
 *     commit word and read it back. A slot counts only if magic, version, CRC
 *     AND the exact commit word are all present. A cut during the body leaves
 *     random 0/1 bits: CRC or commit fails. A cut during the commit word leaves
 *     some of its bits still 1: the exact value fails. Either way the previous
 *     record remains the newest valid one.
 *   - Append only: a committed slot is never reprogrammed.
 *   - Rotation: when the active sector is full the NEXT sector is erased and
 *     written. The newest committed record is always in the ACTIVE sector, so
 *     the erase can never destroy the only valid copy. A cut during that erase
 *     leaves the next sector half-erased: mount ignores its garbage (CRC) and
 *     erases it again before use.
 *   - Mount picks the highest sequence among committed slots; new slots use a
 *     sequence above every intact slot ever seen (also uncommitted ones).
 *   - An intact slot WITHOUT a commit word (cut between body and commit) is
 *     reported separately (uncommitted()): never used as state, only to keep
 *     incident ids monotonic.
 *
 * WEAR: 64 slots per sector -> one erase per 64 commits per sector. With
 * 100,000 erase cycles (W25Q128JV datasheet minimum) and 2 sectors the
 * journal survives ~12.8 million commits (~10 per incident + 1 per boot).
 *
 * BOUNDED RAM: two 32-byte payload copies + a 64-byte slot buffer.
 * NOT THREAD-SAFE: only the StorageTask calls it (one owner).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_EMERGENCY_JOURNAL_HPP
#define PARAGUARDIAN_STORAGE_EMERGENCY_JOURNAL_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"
#include "Storage/NorFlash.hpp"

namespace pg {

namespace journal {
constexpr size_t kSlotBytes = 64u;
constexpr size_t kPayloadBytes = 32u;
constexpr size_t kBodyBytes = 60u;            ///< everything before the commit word
constexpr size_t kCrcOffset = 56u;
constexpr size_t kCommitOffset = 60u;
constexpr uint32_t kMagic = 0x314A4750u;      ///< "PGJ1"
constexpr uint8_t kLayoutVersion = 1u;
constexpr uint8_t kTypeEmergency = 1u;
constexpr uint32_t kCommitWord = 0x3CC3A55Au;
constexpr uint32_t kMaxSequence = 0xFFFFFF00u; ///< beyond this the journal refuses (never reached in practice)
}  // namespace journal

enum class JournalMount : uint8_t {
    NotMounted = 0,
    Unavailable,   ///< flash not ready / unreadable
    Blank,         ///< every slot erased (first use)
    Recovered,     ///< a committed record was found
    Corrupt,       ///< non-erased data but no committed record
};

struct JournalStats {
    uint32_t mounts;
    uint32_t commits;
    uint32_t commitFailures;     ///< any commit() that did not commit
    uint32_t verifyFailures;     ///< read-back mismatch (slot burned, next slot used)
    uint32_t burnedSlots;
    uint32_t erases;
    uint32_t eraseFailures;      ///< erase error or not blank afterwards
    uint32_t readFailures;
    uint32_t validAtMount;       ///< committed slots seen by the last mount
    uint32_t tornAtMount;        ///< non-erased, invalid slots seen by the last mount
    uint32_t uncommittedAtMount; ///< intact slots without commit word
};

class EmergencyJournal {
public:
    /// @param region at least 2 sectors; base sector-aligned.
    EmergencyJournal(INorFlash& flash, const FlashRegion& region);

    /// Scans every slot (bounded: sectors x 64 reads of 64 B). Call once, from the storage task.
    JournalMount mount();

    /// Appends one record and makes it the newest committed one. One or two page programs,
    /// plus (every 64 commits) one sector erase if maintain() did not pre-erase.
    Status commit(const uint8_t* payload, size_t len, uint32_t uptimeMs);

    /// Pre-erases the next sector once the active one is half full (idle-time work, so a
    /// later commit does not also pay for an erase). Returns Ok when there is nothing to do.
    Status maintain();
    bool maintenanceDue() const;

    bool latest(uint8_t* out, size_t len) const;        ///< newest committed payload
    bool uncommitted(uint8_t* out, size_t len) const;   ///< newer intact payload without commit word
    JournalMount mountState() const { return mount_; }
    uint32_t sequence() const { return seq_; }
    uint32_t bootCount() const { return boot_; }
    const JournalStats& stats() const { return stats_; }
    uint32_t slotsPerSector() const { return slotsPerSector_; }
    uint32_t activeSector() const { return cursorSector_; }
    uint32_t cursorSlot() const { return cursorSlot_; }

    /// Encodes a slot (exposed for tests and the host decoder).
    static void encodeSlot(uint8_t* slot, uint32_t seq, uint32_t boot, const uint8_t* payload, uint32_t uptimeMs);
    /// 0 = not intact, 1 = intact without commit, 2 = committed.
    static int classifySlot(const uint8_t* slot, uint32_t& seq);

private:
    uint32_t slotAddress(uint32_t sector, uint32_t slot) const;
    Status eraseAndCheck(uint32_t sector);
    Status prepareSlot();

    INorFlash& flash_;
    FlashRegion region_;
    uint32_t slotsPerSector_ = 0u;
    JournalMount mount_ = JournalMount::NotMounted;
    uint32_t seq_ = 0u;
    uint32_t boot_ = 0u;
    uint32_t cursorSector_ = 0u;
    uint32_t cursorSlot_ = 0u;
    uint32_t latestSector_ = UINT32_MAX;
    bool nextErased_ = false;
    bool haveLatest_ = false;
    bool haveUncommitted_ = false;
    uint8_t latest_[journal::kPayloadBytes] = {};
    uint8_t uncommitted_[journal::kPayloadBytes] = {};
    uint8_t slot_[journal::kSlotBytes] = {};
    JournalStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_EMERGENCY_JOURNAL_HPP
