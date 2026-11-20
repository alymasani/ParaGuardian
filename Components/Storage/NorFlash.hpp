/**
 ******************************************************************************
 * @file    NorFlash.hpp
 * @brief   Minimal SPI-NOR interface used by the emergency journal and the
 *          flight log, the W25Q128 adapter, the "not configured" backend,
 *          and the external-flash partition plan.
 *
 * NOR RULES every user of this interface relies on:
 *   - erased bytes read 0xFF; program() can only clear bits (new = old AND data)
 *   - program() must stay inside ONE page (the W25Q wraps inside the page otherwise)
 *   - eraseSector() sets a whole 4 KiB sector back to 0xFF and is slow
 *     (W25Q128JV tSE typ 45 ms, max 400 ms) -> only the StorageTask calls it
 *   - an interrupted program / erase leaves UNDEFINED bits: readers must
 *     tolerate any content (CRC + commit markers, see EmergencyJournal.hpp)
 *
 * The callers are synchronous: one call = one bounded flash operation (one
 * page program or one sector erase, each with the driver's busy timeout).
 * Only the low-priority StorageTask calls them, so the time they take never
 * delays acquisition, processing or the emergency task.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_NOR_FLASH_HPP
#define PARAGUARDIAN_STORAGE_NOR_FLASH_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"
#include "Storage/W25Q128/W25q128.hpp"

namespace pg {

struct NorGeometry {
    uint32_t pageSize;
    uint32_t sectorSize;
    uint32_t capacity;
};

class INorFlash {
public:
    virtual Status read(uint32_t address, uint8_t* dst, size_t len) = 0;
    /// Program inside one page (NOR AND semantics). The target bits must be erased.
    virtual Status program(uint32_t address, const uint8_t* src, size_t len) = 0;
    /// Erase the sector-aligned sector at @p address.
    virtual Status eraseSector(uint32_t address) = 0;
    virtual bool ready() const = 0;
    virtual NorGeometry geometry() const = 0;

protected:
    ~INorFlash() = default;
};

/// A contiguous run of whole sectors.
struct FlashRegion {
    uint32_t base;      ///< sector-aligned
    uint32_t sectors;
};

/**
 * External flash partition plan (W25Q128JV, 16 MiB = 4096 x 4 KiB sectors).
 * Inspected 2026-11: no firmware code reads or writes the external flash yet
 * (the driver exists but nothing is wired), so nothing on the chip is in use.
 * The first 64 KiB are left unused for future provisioning data (device
 * identity / keys); emergency data is kept apart from bulk telemetry so a
 * full or corrupted flight log can never touch it.
 */
namespace partition {
constexpr uint32_t kSector = w25q::SECTOR_SIZE;
constexpr FlashRegion kReserved{0x000000u, 16u};          ///< 64 KiB: future provisioning (untouched)
constexpr FlashRegion kEmergencyJournal{0x010000u, 2u};   ///< 8 KiB: two-sector journal (A/B)
constexpr FlashRegion kCriticalLog{0x012000u, 14u};       ///< 56 KiB: emergency / SOS / boot / fault history
constexpr FlashRegion kFlightLog{0x020000u, 4064u};       ///< ~15.9 MiB: telemetry ring (rotates)
constexpr uint32_t end(const FlashRegion& r) { return r.base + r.sectors * kSector; }
static_assert(end(kReserved) == kEmergencyJournal.base, "partition gap / overlap");
static_assert(end(kEmergencyJournal) == kCriticalLog.base, "partition gap / overlap");
static_assert(end(kCriticalLog) == kFlightLog.base, "partition gap / overlap");
static_assert(end(kFlightLog) == w25q::CAPACITY_BYTES, "partition must end at the chip end");
}  // namespace partition

/// Adapter: the real W25Q128 driver (portable; the SPI device underneath is the BSP's).
class W25qNorFlash final : public INorFlash {
public:
    explicit W25qNorFlash(W25q128& chip) : chip_(chip) {}
    Status read(uint32_t address, uint8_t* dst, size_t len) override { return chip_.read(address, dst, len); }
    Status program(uint32_t address, const uint8_t* src, size_t len) override
    {
        return chip_.pageProgram(address, src, len);
    }
    Status eraseSector(uint32_t address) override { return chip_.eraseSector(address); }
    bool ready() const override { return chip_.isInitialized(); }
    NorGeometry geometry() const override
    {
        return NorGeometry{w25q::PAGE_SIZE, w25q::SECTOR_SIZE, w25q::CAPACITY_BYTES};
    }

private:
    W25q128& chip_;
};

/// Firmware default until the flash SPI instance / chip select are known (no schematic).
class UnconfiguredNorFlash final : public INorFlash {
public:
    Status read(uint32_t, uint8_t*, size_t) override { return Status::NotInitialized; }
    Status program(uint32_t, const uint8_t*, size_t) override { return Status::NotInitialized; }
    Status eraseSector(uint32_t) override { return Status::NotInitialized; }
    bool ready() const override { return false; }
    NorGeometry geometry() const override
    {
        return NorGeometry{w25q::PAGE_SIZE, w25q::SECTOR_SIZE, w25q::CAPACITY_BYTES};
    }
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_NOR_FLASH_HPP
