/**
 ******************************************************************************
 * @file    RegisterBus.hpp
 * @brief   "Read/write register N" on top of SPI or I2C.
 *
 * Most sensors (LSM6DSRX, BMP585, MMC5983MA) are a bank of 8-bit registers.
 * The only difference between talking to them over SPI and over I2C is how
 * the register address is sent:
 *
 *   SPI : first byte = register address with bit 7 = 1 for READ, 0 for WRITE.
 *   I2C : write the register address, then repeated-start and read.
 *
 * IRegisterBus hides that difference so each sensor driver is written once and
 * works on either bus. The board code decides which one to construct.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_REGISTER_BUS_HPP
#define PARAGUARDIAN_COMMON_REGISTER_BUS_HPP

#include <cstddef>
#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/Status.hpp"

namespace pg {

class IRegisterBus {
public:
    /// Burst-read @p len consecutive registers starting at @p reg into @p dst.
    virtual Status readRegisters(uint8_t reg, uint8_t* dst, size_t len) = 0;

    /// Write one register.
    virtual Status writeRegister(uint8_t reg, uint8_t value) = 0;

    /// Convenience: read exactly one register.
    Status readRegister(uint8_t reg, uint8_t& value) { return readRegisters(reg, &value, 1u); }

protected:
    ~IRegisterBus() = default;
};

/**
 * SPI register access.
 * The command byte is  (reg & addressMask) | readFlag   for reads
 *                      (reg & addressMask) | writeFlag  for writes.
 * Defaults (0x80 read flag, 0x7F address mask) match LSM6DSRX, BMP585 and
 * MMC5983MA, which all use "bit 7 = 1 means read".
 */
class SpiRegisterBus final : public IRegisterBus {
public:
    struct Protocol {
        uint8_t readFlag;
        uint8_t writeFlag;
        uint8_t addressMask;
    };
    static constexpr Protocol kBit7ReadProtocol{0x80u, 0x00u, 0x7Fu};

    explicit SpiRegisterBus(ISpiDevice& spi, Protocol protocol = kBit7ReadProtocol)
        : spi_(spi), protocol_(protocol) {}

    Status readRegisters(uint8_t reg, uint8_t* dst, size_t len) override;
    Status writeRegister(uint8_t reg, uint8_t value) override;

private:
    ISpiDevice& spi_;
    Protocol protocol_;
};

/// I2C register access for a device at a fixed 7-bit address.
class I2cRegisterBus final : public IRegisterBus {
public:
    I2cRegisterBus(II2cBus& bus, uint8_t address7) : bus_(bus), address7_(address7) {}

    Status readRegisters(uint8_t reg, uint8_t* dst, size_t len) override;
    Status writeRegister(uint8_t reg, uint8_t value) override;

    uint8_t address() const { return address7_; }

private:
    II2cBus& bus_;
    uint8_t address7_;
};

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_REGISTER_BUS_HPP
