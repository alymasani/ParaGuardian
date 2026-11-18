/**
 ******************************************************************************
 * @file    FakeW25q.hpp
 * @brief   Behavioural model of a W25Q128JV on SPI, for host tests.
 *
 * Models the parts of the real chip that matter to the driver:
 *  - JEDEC ID (configurable), status register 1 BUSY/WEL bits
 *  - WEL must be set by 06h before program/erase, and is cleared after
 *  - Page Program uses NOR semantics (new = old AND data) and, exactly like the
 *    real chip, WRAPS to the start of the same 256-byte page if data runs past
 *    the page end (that is the bug pageProgram() guards against)
 *  - Sector erase sets 4 KiB to 0xFF; programming/erasing makes the chip busy
 *    for a few status reads
 *  - Write protection (WEL never sets), stuck-busy and bus-error injection
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_FAKE_W25Q_HPP
#define PARAGUARDIAN_TESTS_FAKE_W25Q_HPP

#include <cstdint>
#include <vector>

#include "Common/BusInterfaces.hpp"

class FakeW25q final : public pg::ISpiDevice {
public:
    static constexpr uint32_t kSize = 16u * 1024u * 1024u;

    FakeW25q() : mem(kSize, 0xFF) {}

    std::vector<uint8_t> mem;
    uint8_t jedec[3] = {0xEF, 0x40, 0x18};
    bool wel = false;
    int busyReads = 0;          // remaining status reads that report BUSY
    int busyReadsPerOp = 2;
    bool stuckBusy = false;
    bool writeProtected = false;
    bool failNext = false;
    std::vector<uint8_t> commands;               // opcode of every transaction
    struct Program { uint32_t addr; size_t len; };
    std::vector<Program> programs;               // every 0x02 issued
    std::vector<uint32_t> erases;                // every 0x20 address

    pg::Status writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override
    {
        if (failNext) { failNext = false; return pg::Status::BusError; }
        if (txLen == 0) return pg::Status::InvalidArgument;
        commands.push_back(tx[0]);
        switch (tx[0]) {
            case 0x9F:
                for (size_t i = 0; i < rxLen; ++i) rx[i] = (i < 3) ? jedec[i] : 0;
                return pg::Status::Ok;
            case 0x05: {
                bool busy = stuckBusy || busyReads > 0;
                if (busyReads > 0) --busyReads;
                for (size_t i = 0; i < rxLen; ++i) rx[i] = static_cast<uint8_t>((busy ? 0x01 : 0) | (wel ? 0x02 : 0));
                return pg::Status::Ok;
            }
            case 0x03: {
                if (txLen != 4) return pg::Status::InvalidArgument;
                uint32_t a = addr(tx);
                for (size_t i = 0; i < rxLen; ++i) rx[i] = mem[(a + i) % kSize];
                return pg::Status::Ok;
            }
            default:
                return pg::Status::InvalidArgument;
        }
    }

    pg::Status writeThenWrite(const uint8_t* h, size_t hLen, const uint8_t* p, size_t pLen) override
    {
        if (failNext) { failNext = false; return pg::Status::BusError; }
        if (hLen == 0) return pg::Status::InvalidArgument;
        commands.push_back(h[0]);
        switch (h[0]) {
            case 0xAB: return pg::Status::Ok;
            case 0x06: if (!writeProtected) wel = true; return pg::Status::Ok;
            case 0x04: wel = false; return pg::Status::Ok;
            case 0x02: {
                if (hLen != 4) return pg::Status::InvalidArgument;
                if (!wel) return pg::Status::Ok;  // real chip silently ignores
                const uint32_t a = addr(h);
                programs.push_back({a, pLen});
                const uint32_t pageBase = a & ~0xFFu;
                uint32_t offset = a & 0xFFu;
                for (size_t i = 0; i < pLen; ++i) {
                    mem[pageBase + offset] &= p[i];  // NOR: can only clear bits
                    offset = (offset + 1u) & 0xFFu;   // WRAP inside the page
                }
                wel = false;
                busyReads = busyReadsPerOp;
                return pg::Status::Ok;
            }
            case 0x20: case 0xD8: {
                if (hLen != 4) return pg::Status::InvalidArgument;
                if (!wel) return pg::Status::Ok;
                const uint32_t size = (h[0] == 0x20) ? 4096u : 65536u;
                const uint32_t a = addr(h) & ~(size - 1u);
                erases.push_back(a);
                for (uint32_t i = 0; i < size; ++i) mem[a + i] = 0xFF;
                wel = false;
                busyReads = busyReadsPerOp;
                return pg::Status::Ok;
            }
            default:
                return pg::Status::InvalidArgument;
        }
    }

    bool sawCommand(uint8_t op) const
    {
        for (uint8_t c : commands) if (c == op) return true;
        return false;
    }

private:
    static uint32_t addr(const uint8_t* h)
    {
        return (static_cast<uint32_t>(h[1]) << 16) | (static_cast<uint32_t>(h[2]) << 8) | h[3];
    }
};

#endif  // PARAGUARDIAN_TESTS_FAKE_W25Q_HPP
