/**
 ******************************************************************************
 * @file    FakeRegisterDevice.hpp
 * @brief   Simulated register-bank sensor reachable over SPI or I2C.
 *
 * - 128 registers (addresses 0x00..0x7F).
 * - SPI protocol: first byte = address | 0x80 for read, address for write
 *   (the convention of LSM6DSRX / BMP585 / MMC5983MA). The fake CHECKS the
 *   read/write bit, so a driver that sent the wrong command byte would fail.
 * - Burst reads auto-increment the address.
 * - Every write is logged (register, value) in order.
 * - Hooks (onWrite/onRead) let a test emulate chip behaviour, e.g. a reset
 *   bit that clears itself, or a "measurement done" flag.
 * - Fault injection: fail the Nth transaction with a chosen Status,
 *   or simulate a floating bus (every read returns 0xFF).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_FAKE_REGISTER_DEVICE_HPP
#define PARAGUARDIAN_TESTS_FAKE_REGISTER_DEVICE_HPP

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "Common/BusInterfaces.hpp"

class FakeRegisterDevice {
public:
    uint8_t regs[128] = {};
    std::vector<std::pair<uint8_t, uint8_t>> writes;  // (register, value) in order
    std::vector<uint8_t> readStarts;                  // first register of each read transaction
    std::function<void(uint8_t reg, uint8_t value)> onWrite;
    std::function<void(uint8_t reg)> onRead;          // called BEFORE data is returned
    int transactions = 0;
    int failAtTransaction = -1;                       // 1-based; -1 = never
    pg::Status failStatus = pg::Status::BusError;
    bool floatingBus = false;
    int protocolErrors = 0;                           // wrong R/W bit seen

    pg::Status doRead(uint8_t reg, uint8_t* rx, size_t len)
    {
        if (shouldFail()) return failStatus;
        readStarts.push_back(reg);
        if (onRead) onRead(reg);
        for (size_t i = 0; i < len; ++i) {
            rx[i] = floatingBus ? 0xFF : regs[(reg + i) & 0x7F];
        }
        return pg::Status::Ok;
    }

    pg::Status doWrite(uint8_t reg, uint8_t value)
    {
        if (shouldFail()) return failStatus;
        writes.push_back({reg, value});
        regs[reg & 0x7F] = value;
        if (onWrite) onWrite(reg, value);
        return pg::Status::Ok;
    }

    /// Value of the last write to @p reg, or -1 if never written.
    int lastWrite(uint8_t reg) const
    {
        for (auto it = writes.rbegin(); it != writes.rend(); ++it) {
            if (it->first == reg) return it->second;
        }
        return -1;
    }

private:
    bool shouldFail()
    {
        ++transactions;
        return failAtTransaction > 0 && transactions == failAtTransaction;
    }
};

/// SPI front-end for FakeRegisterDevice.
class FakeSpiRegisterDevice final : public pg::ISpiDevice {
public:
    explicit FakeSpiRegisterDevice(FakeRegisterDevice& dev) : dev_(dev) {}

    pg::Status writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override
    {
        if (txLen != 1 || rx == nullptr || rxLen == 0) return pg::Status::InvalidArgument;
        if ((tx[0] & 0x80) == 0) {
            ++dev_.protocolErrors;  // a read must have bit 7 set
            return pg::Status::BusError;
        }
        return dev_.doRead(static_cast<uint8_t>(tx[0] & 0x7F), rx, rxLen);
    }

    pg::Status writeThenWrite(const uint8_t* header, size_t headerLen, const uint8_t* payload,
                              size_t payloadLen) override
    {
        if (headerLen != 2 || payloadLen != 0 || payload != nullptr) return pg::Status::InvalidArgument;
        if ((header[0] & 0x80) != 0) {
            ++dev_.protocolErrors;  // a write must have bit 7 clear
            return pg::Status::BusError;
        }
        return dev_.doWrite(header[0], header[1]);
    }

private:
    FakeRegisterDevice& dev_;
};

/// I2C front-end: device answers only at its own 7-bit address (else NACK -> BusError).
class FakeI2cBus final : public pg::II2cBus {
public:
    FakeI2cBus(FakeRegisterDevice& dev, uint8_t address7) : dev_(dev), address7_(address7) {}

    pg::Status write(uint8_t address7, const uint8_t* data, size_t len) override
    {
        if (address7 != address7_) return pg::Status::BusError;  // NACK
        if (len != 2) return pg::Status::InvalidArgument;
        return dev_.doWrite(data[0], data[1]);
    }

    pg::Status writeThenRead(uint8_t address7, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override
    {
        if (address7 != address7_) return pg::Status::BusError;  // NACK
        if (txLen != 1) return pg::Status::InvalidArgument;
        return dev_.doRead(tx[0], rx, rxLen);
    }

private:
    FakeRegisterDevice& dev_;
    uint8_t address7_;
};

#endif  // PARAGUARDIAN_TESTS_FAKE_REGISTER_DEVICE_HPP
