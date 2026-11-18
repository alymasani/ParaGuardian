/**
 ******************************************************************************
 * @file    W25q128.hpp
 * @brief   Winbond W25Q128JV 128 Mbit (16 MiB) SPI NOR flash driver.
 *
 * Source: Winbond W25Q128JV datasheet (Rev. I or later): instruction set
 * table, status register 1 (BUSY bit 0, WEL bit 1), AC characteristics
 * (tPP, tSE, tBE1, tRES1).
 *
 * Geometry (verified for W25Q128JV):
 *   capacity 16,777,216 bytes = 65,536 pages x 256 bytes
 *                             = 4,096 sectors x 4 KiB
 *                             = 256 blocks x 64 KiB
 * Addressing: 24-bit, sent most-significant byte first.
 *
 * NOR flash rules the driver enforces:
 *   - Programming can only change 1 -> 0. Erase (sector/block) sets bytes to 0xFF.
 *   - Page Program wraps around INSIDE a 256-byte page if it crosses the page
 *     end, silently corrupting data. pageProgram() therefore rejects any write
 *     that would cross a page boundary; write() splits writes at boundaries.
 *   - Every program/erase must be preceded by Write Enable (06h).
 *
 * The driver NEVER erases anything during init().
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_W25Q128_HPP
#define PARAGUARDIAN_STORAGE_W25Q128_HPP

#include <cstddef>
#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/Status.hpp"

namespace pg {

namespace w25q {
constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
constexpr uint8_t CMD_WRITE_DISABLE = 0x04;
constexpr uint8_t CMD_READ_STATUS1 = 0x05;
constexpr uint8_t CMD_READ_STATUS2 = 0x35;
constexpr uint8_t CMD_READ_STATUS3 = 0x15;
constexpr uint8_t CMD_READ_DATA = 0x03;      // up to 50 MHz, no dummy byte
constexpr uint8_t CMD_PAGE_PROGRAM = 0x02;
constexpr uint8_t CMD_SECTOR_ERASE_4K = 0x20;
constexpr uint8_t CMD_BLOCK_ERASE_64K = 0xD8;
constexpr uint8_t CMD_JEDEC_ID = 0x9F;
constexpr uint8_t CMD_RELEASE_POWER_DOWN = 0xAB;
constexpr uint8_t CMD_ENABLE_RESET = 0x66;
constexpr uint8_t CMD_RESET_DEVICE = 0x99;

constexpr uint8_t SR1_BUSY = 1u << 0;
constexpr uint8_t SR1_WEL = 1u << 1;
constexpr uint8_t SR1_PROTECT_MASK = 0x7Cu;   // BP0..BP2, TB, SEC (bits 2..6)

constexpr uint8_t MANUFACTURER_WINBOND = 0xEF;
constexpr uint8_t MEMORY_TYPE_SPI = 0x40;     // W25Q128JV-IQ/JQ
constexpr uint8_t MEMORY_TYPE_DTR = 0x70;     // W25Q128JV-IM/JM
constexpr uint8_t CAPACITY_128MBIT = 0x18;

constexpr uint32_t CAPACITY_BYTES = 16u * 1024u * 1024u;
constexpr uint32_t PAGE_SIZE = 256u;
constexpr uint32_t SECTOR_SIZE = 4096u;
constexpr uint32_t BLOCK_SIZE_64K = 65536u;

// Maximum times from the datasheet AC table, plus margin.
constexpr uint32_t PAGE_PROGRAM_TIMEOUT_MS = 5u;      // tPP max 3 ms
constexpr uint32_t SECTOR_ERASE_TIMEOUT_MS = 500u;    // tSE max 400 ms
constexpr uint32_t BLOCK_ERASE_TIMEOUT_MS = 2500u;    // tBE2 (64 KB) max 2000 ms
constexpr uint32_t WRITE_ENABLE_TIMEOUT_MS = 2u;
}  // namespace w25q

struct JedecId {
    uint8_t manufacturer;
    uint8_t memoryType;
    uint8_t capacity;
};

struct FlashStats {
    uint32_t pagesProgrammed = 0;
    uint32_t sectorsErased = 0;
    uint32_t busyTimeouts = 0;
    uint32_t busErrors = 0;
    uint32_t verifyFailures = 0;
};

class W25q128 {
public:
    W25q128(ISpiDevice& spi, IClock& clock) : spi_(spi), clock_(clock) {}

    /// Wake from power-down, read and check the JEDEC ID, check not busy. No erase.
    Status init();

    Status readJedecId(JedecId& id);
    Status readStatus1(uint8_t& sr1);

    /// Read any number of bytes from any address (wraps not allowed: must stay inside the chip).
    Status read(uint32_t address, uint8_t* dst, size_t len);

    /// Program within ONE page. Rejects writes that cross a 256-byte page boundary.
    Status pageProgram(uint32_t address, const uint8_t* src, size_t len);

    /**
     * Program any length, split at page boundaries. Target area must already be
     * erased (0xFF). If @p verify is true each page is read back and compared.
     */
    Status write(uint32_t address, const uint8_t* src, size_t len, bool verify = false);

    /// Erase the 4 KiB sector containing @p address. @p address must be sector-aligned.
    Status eraseSector(uint32_t address);

    /// Erase the 64 KiB block at @p address (must be 64 KiB aligned).
    Status eraseBlock64k(uint32_t address);

    /// Poll BUSY until clear or @p timeoutMs elapses.
    Status waitUntilReady(uint32_t timeoutMs);

    bool isInitialized() const { return initialized_; }
    const FlashStats& stats() const { return stats_; }

    /// True if [address, address+len) lies inside the chip (overflow-safe).
    static bool rangeIsValid(uint32_t address, size_t len);

private:
    Status writeEnable();
    Status eraseCommand(uint8_t command, uint32_t address, uint32_t alignment, uint32_t timeoutMs);
    Status countBus(Status s);

    ISpiDevice& spi_;
    IClock& clock_;
    bool initialized_ = false;
    FlashStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_W25Q128_HPP
