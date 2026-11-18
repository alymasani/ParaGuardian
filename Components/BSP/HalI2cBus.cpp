/**
 ******************************************************************************
 * @file    HalI2cBus.cpp
 * @brief   See HalI2cBus.hpp.
 ******************************************************************************
 */
#include "BSP/HalI2cBus.hpp"

#if defined(HAL_I2C_MODULE_ENABLED)

namespace pg {
namespace bsp {

namespace {
Status fromHal(HAL_StatusTypeDef s)
{
    switch (s) {
        case HAL_OK:      return Status::Ok;
        case HAL_TIMEOUT: return Status::Timeout;
        case HAL_BUSY:    return Status::Busy;
        case HAL_ERROR:   // includes NACK (device absent / wrong address)
        default:          return Status::BusError;
    }
}
}  // namespace

Status HalI2cBus::write(uint8_t address7, const uint8_t* data, size_t len)
{
    if (hi2c_ == nullptr || data == nullptr || len == 0u || len > UINT16_MAX || address7 > 0x7Fu) {
        return Status::InvalidArgument;
    }
    return fromHal(HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(address7 << 1), const_cast<uint8_t*>(data),
                                           static_cast<uint16_t>(len), timeoutMs_));
}

Status HalI2cBus::writeThenRead(uint8_t address7, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen)
{
    if (hi2c_ == nullptr || tx == nullptr || txLen == 0u || rx == nullptr || rxLen == 0u || rxLen > UINT16_MAX ||
        address7 > 0x7Fu) {
        return Status::InvalidArgument;
    }
    const uint16_t addr8 = static_cast<uint16_t>(address7 << 1);
    if (txLen == 1u || txLen == 2u) {
        const uint16_t memAddr = (txLen == 1u) ? tx[0] : static_cast<uint16_t>((tx[0] << 8) | tx[1]);
        const uint16_t memSize = (txLen == 1u) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
        return fromHal(HAL_I2C_Mem_Read(hi2c_, addr8, memAddr, memSize, rx, static_cast<uint16_t>(rxLen), timeoutMs_));
    }
    if (txLen > UINT16_MAX) {
        return Status::InvalidArgument;
    }
    HAL_StatusTypeDef hs =
        HAL_I2C_Master_Transmit(hi2c_, addr8, const_cast<uint8_t*>(tx), static_cast<uint16_t>(txLen), timeoutMs_);
    if (hs == HAL_OK) {
        hs = HAL_I2C_Master_Receive(hi2c_, addr8, rx, static_cast<uint16_t>(rxLen), timeoutMs_);
    }
    return fromHal(hs);
}

}  // namespace bsp
}  // namespace pg

#endif  // HAL_I2C_MODULE_ENABLED
