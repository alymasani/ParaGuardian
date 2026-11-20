/**
 ******************************************************************************
 * @file    FakeNorFlash.hpp
 * @brief   NOR flash model for storage tests, with fault injection.
 *
 * Models: 0xFF erase state, program = AND (bits only 1 -> 0), page wrap
 * rejection, per-sector erase counts (wear), simulated busy time (advances a
 * virtual clock: page program 1 ms, sector erase 45 ms typical), and:
 *   - power cut after N more programmed BYTES: the byte being programmed gets
 *     a random subset of its 0-bits, the rest of the operation is lost, and
 *     every later call fails until reboot() (models the MCU losing power)
 *   - power cut during an erase: the sector gets a mix of erased and old bytes
 *   - transient program / erase / read failures, permanent unavailability,
 *     write timeout, silent bit flips (corruption at rest)
 * Deterministic: a fixed-seed LCG drives every "random" choice.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_FAKE_NOR_FLASH_HPP
#define PARAGUARDIAN_TESTS_FAKE_NOR_FLASH_HPP

#include <cstdint>
#include <cstring>
#include <vector>

#include "Common/BusInterfaces.hpp"
#include "Storage/NorFlash.hpp"

class FakeNorFlash final : public pg::INorFlash, public pg::IClock {
public:
    explicit FakeNorFlash(uint32_t capacity = pg::w25q::CAPACITY_BYTES)
        : mem(capacity, 0xFF), eraseCount(capacity / pg::w25q::SECTOR_SIZE, 0u)
    {
    }

    std::vector<uint8_t> mem;
    std::vector<uint32_t> eraseCount;
    bool present = true;          ///< false = flash permanently unavailable
    bool powerLost = false;
    long cutAfterBytes = -1;      ///< >= 0: power is lost after this many more bytes are programmed
    bool cutNextErase = false;
    int failPrograms = 0;         ///< next N program() calls fail (bus error, nothing written)
    int failErases = 0;
    int failReads = 0;
    bool timeoutWrites = false;   ///< program/erase report Timeout (busy never clears)
    uint32_t programs = 0, erases = 0, reads = 0, bytesProgrammed = 0;
    uint32_t nowMs = 0;           ///< virtual clock advanced by flash busy time
    uint32_t programMs = 1, eraseMs = 45;
    uint32_t seed = 12345u;

    void reboot() { powerLost = false; cutAfterBytes = -1; cutNextErase = false; }
    void flip(uint32_t addr, uint8_t mask) { mem[addr] ^= mask; }

    // IClock
    uint32_t millis() override { return nowMs; }
    void delayMs(uint32_t ms) override { nowMs += ms; }

    // INorFlash
    bool ready() const override { return present && !powerLost; }
    pg::NorGeometry geometry() const override
    {
        return pg::NorGeometry{pg::w25q::PAGE_SIZE, pg::w25q::SECTOR_SIZE, static_cast<uint32_t>(mem.size())};
    }
    pg::Status read(uint32_t a, uint8_t* dst, size_t len) override
    {
        if (!ready()) return pg::Status::NotInitialized;
        if (failReads > 0) { --failReads; return pg::Status::BusError; }
        if (a + len > mem.size()) return pg::Status::OutOfRange;
        ++reads;
        std::memcpy(dst, &mem[a], len);
        return pg::Status::Ok;
    }
    pg::Status program(uint32_t a, const uint8_t* src, size_t len) override
    {
        if (!ready()) return pg::Status::NotInitialized;
        if (len == 0 || a + len > mem.size()) return pg::Status::OutOfRange;
        if ((a % pg::w25q::PAGE_SIZE) + len > pg::w25q::PAGE_SIZE) return pg::Status::InvalidArgument;
        if (failPrograms > 0) { --failPrograms; return pg::Status::BusError; }
        if (timeoutWrites) { nowMs += 5; return pg::Status::Timeout; }
        ++programs;
        nowMs += programMs;
        for (size_t i = 0; i < len; ++i) {
            if (cutAfterBytes == 0) {
                // the interrupted byte: only some of its 0-bits made it
                mem[a + i] &= static_cast<uint8_t>(src[i] | (rnd() & 0xFFu));
                powerLost = true;
                return pg::Status::Timeout;
            }
            mem[a + i] &= src[i];
            ++bytesProgrammed;
            if (cutAfterBytes > 0) --cutAfterBytes;
        }
        return pg::Status::Ok;
    }
    pg::Status eraseSector(uint32_t a) override
    {
        if (!ready()) return pg::Status::NotInitialized;
        if (a % pg::w25q::SECTOR_SIZE != 0 || a >= mem.size()) return pg::Status::InvalidArgument;
        if (failErases > 0) { --failErases; return pg::Status::BusError; }
        if (timeoutWrites) { nowMs += 500; return pg::Status::Timeout; }
        ++erases;
        ++eraseCount[a / pg::w25q::SECTOR_SIZE];
        nowMs += eraseMs;
        if (cutNextErase) {
            for (uint32_t i = 0; i < pg::w25q::SECTOR_SIZE; ++i) {
                if (rnd() & 1u) mem[a + i] = 0xFF;   // partially erased: some bytes keep old data
            }
            powerLost = true;
            return pg::Status::Timeout;
        }
        std::memset(&mem[a], 0xFF, pg::w25q::SECTOR_SIZE);
        return pg::Status::Ok;
    }

private:
    uint32_t rnd()
    {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    }
};

#endif  // PARAGUARDIAN_TESTS_FAKE_NOR_FLASH_HPP
