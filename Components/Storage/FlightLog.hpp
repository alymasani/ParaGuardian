/**
 ******************************************************************************
 * @file    FlightLog.hpp
 * @brief   Rotating, power-loss-tolerant record log on SPI NOR (one per
 *          partition: telemetry "flight log" and the separate "critical log").
 *
 * LAYOUT: every sector starts with a 16-byte header
 *    0 magic "PGLS" u32 | 4 version u8 | 5 region tag u8 | 6 reserved u16 |
 *    8 sector sequence u32 | 12 CRC-32 of bytes 0..11
 * followed by records (LogRecords.hpp) packed into 256-byte pages; a record
 * never crosses a page, unused page tail stays 0xFF.
 *
 * WRITING: append() fills a RAM copy of the current page; flush() programs
 * only the bytes added since the last flush (NOR allows programming the
 * still-erased tail of a page). The storage task flushes when a page fills,
 * after every critical record, and at least once a second for telemetry, so
 * a power cut loses at most the unflushed records (<= 1 s of telemetry,
 * at most the record being written for the critical log).
 *
 * ROTATION / RETENTION: when the head sector is full, the next sector in the
 * ring is erased and gets header sequence + 1. OverwriteOldest (default)
 * keeps the newest ~region size of data; StopWhenFull refuses new records
 * (counted) instead of erasing old ones.
 *
 * RECOVERY (mount): the head is the sector with the highest valid header
 * sequence; inside it, records are parsed page by page. A torn record (CRC
 * fails / garbage) ends that page; writing resumes at the first page whose
 * remaining bytes are all erased. Every committed record before the cut is
 * recovered; the record being programmed at the cut is lost (detected by CRC).
 * A sector whose erase was interrupted has no valid header and is ignored,
 * then erased again when the ring reaches it.
 *
 * Only the StorageTask calls this class (single owner, not thread-safe).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_FLIGHT_LOG_HPP
#define PARAGUARDIAN_STORAGE_FLIGHT_LOG_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"
#include "Storage/LogRecords.hpp"
#include "Storage/NorFlash.hpp"

namespace pg {

namespace logfmt {
constexpr uint32_t kSectorMagic = 0x534C4750u;   ///< "PGLS"
constexpr uint8_t kSectorVersion = 1u;
constexpr size_t kSectorHeaderBytes = 16u;
constexpr size_t kPageBytes = 256u;
constexpr uint8_t kTagFlight = 1u;
constexpr uint8_t kTagCritical = 2u;
}  // namespace logfmt

enum class LogMount : uint8_t { NotMounted = 0, Unavailable, Blank, Recovered, Corrupt };
enum class LogFullPolicy : uint8_t { OverwriteOldest = 0, StopWhenFull };

struct FlightLogStats {
    uint32_t recordsAppended;
    uint32_t bytesProgrammed;
    uint32_t programs;
    uint32_t sectorsErased;
    uint32_t rotations;
    uint32_t programFailures;
    uint32_t eraseFailures;
    uint32_t recordsLost;        ///< appended but discarded by a failed program
    uint32_t droppedFull;        ///< StopWhenFull: refused
    uint32_t recoveredRecords;   ///< valid records in the head sector at mount
    uint32_t tornAtMount;        ///< pages ended by an invalid frame at mount
    uint32_t invalidSectors;     ///< sectors without a valid header at mount
};

class FlightLog {
public:
    FlightLog(INorFlash& flash, const FlashRegion& region, uint8_t tag, LogFullPolicy policy);

    LogMount mount();
    /// Buffers one record (programs the previous page / rotates when needed).
    Status append(const LogEntry& e);
    /// Programs the bytes appended since the last flush (no-op if none).
    Status flush();
    bool dirty() const { return pageOff_ != flushedOff_; }

    LogMount mountState() const { return mount_; }
    uint32_t nextSequence() const { return nextSeq_; }
    uint32_t headSector() const { return sector_; }
    uint32_t headSequence() const { return sectorSeq_; }
    bool full() const { return full_; }
    const FlightLogStats& stats() const { return stats_; }

    /// Reader: visits every valid record, oldest sector first. Returns the number visited.
    using Visitor = void (*)(void* ctx, const LogEntry& e, uint32_t sequence);
    static size_t scan(INorFlash& flash, const FlashRegion& region, uint8_t tag, Visitor visit, void* ctx);
    static bool readSectorHeader(INorFlash& flash, uint32_t address, uint8_t tag, uint32_t& seq);

private:
    uint32_t sectorAddress(uint32_t s) const { return region_.base + s * partition::kSector; }
    Status openSector(uint32_t s, uint32_t seq);
    Status advancePage();
    void resumeInHead();

    INorFlash& flash_;
    FlashRegion region_;
    uint8_t tag_;
    LogFullPolicy policy_;
    LogMount mount_ = LogMount::NotMounted;
    bool full_ = false;
    bool needOpen_ = true;        ///< no writable head sector yet (blank / corrupt / unavailable at mount)
    uint32_t sector_ = 0u;
    uint32_t sectorSeq_ = 0u;
    uint32_t page_ = 0u;          ///< page index inside the sector
    size_t pageOff_ = 0u;
    size_t flushedOff_ = 0u;
    uint32_t pageRecords_ = 0u;   ///< records in the unflushed part of the page
    uint32_t nextSeq_ = 1u;
    uint8_t pageBuf_[logfmt::kPageBytes] = {};
    FlightLogStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_FLIGHT_LOG_HPP
