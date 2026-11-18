/**
 ******************************************************************************
 * @file    RegisterBus.cpp
 * @brief   SPI / I2C implementations of IRegisterBus.
 ******************************************************************************
 */
#include "Common/RegisterBus.hpp"

namespace pg {

Status SpiRegisterBus::readRegisters(uint8_t reg, uint8_t* dst, size_t len)
{
    if (dst == nullptr || len == 0u) {
        return Status::InvalidArgument;
    }
    // Example (LSM6DSRX WHO_AM_I = 0x0F):  (0x0F & 0x7F) | 0x80 = 0x8F.
    const uint8_t command = static_cast<uint8_t>((reg & protocol_.addressMask) | protocol_.readFlag);
    return spi_.writeThenRead(&command, 1u, dst, len);
}

Status SpiRegisterBus::writeRegister(uint8_t reg, uint8_t value)
{
    const uint8_t frame[2] = {
        static_cast<uint8_t>((reg & protocol_.addressMask) | protocol_.writeFlag),
        value,
    };
    return spi_.writeThenWrite(frame, sizeof(frame), nullptr, 0u);
}

Status I2cRegisterBus::readRegisters(uint8_t reg, uint8_t* dst, size_t len)
{
    if (dst == nullptr || len == 0u) {
        return Status::InvalidArgument;
    }
    return bus_.writeThenRead(address7_, &reg, 1u, dst, len);
}

Status I2cRegisterBus::writeRegister(uint8_t reg, uint8_t value)
{
    const uint8_t frame[2] = {reg, value};
    return bus_.write(address7_, frame, sizeof(frame));
}

}  // namespace pg
