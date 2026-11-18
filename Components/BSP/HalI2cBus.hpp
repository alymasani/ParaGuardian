/**
 ******************************************************************************
 * @file    HalI2cBus.hpp
 * @brief   II2cBus implemented with STM32 HAL I2C (blocking, with timeout).
 *
 * HAL_I2C_MODULE_ENABLED is currently defined by CubeMX's default hal_conf
 * even though no I2C instance is configured, so this compiles; it is simply
 * not instantiated until the board defines which I2C peripheral is used.
 *
 * HAL I2C functions take an 8-bit address = 7-bit address << 1.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_BSP_HAL_I2C_BUS_HPP
#define PARAGUARDIAN_BSP_HAL_I2C_BUS_HPP

#include "main.h"

#if defined(HAL_I2C_MODULE_ENABLED)

#include <cstdint>

#include "Common/BusInterfaces.hpp"

namespace pg {
namespace bsp {

class HalI2cBus final : public II2cBus {
public:
    HalI2cBus(I2C_HandleTypeDef* hi2c, uint32_t timeoutMs) : hi2c_(hi2c), timeoutMs_(timeoutMs) {}

    Status write(uint8_t address7, const uint8_t* data, size_t len) override;

    /**
     * For 1- or 2-byte register addresses this uses HAL_I2C_Mem_Read, which
     * produces a true REPEATED START. Longer write parts fall back to
     * transmit + STOP + receive (documented limitation).
     */
    Status writeThenRead(uint8_t address7, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override;

private:
    I2C_HandleTypeDef* hi2c_;
    uint32_t timeoutMs_;
};

}  // namespace bsp
}  // namespace pg

#endif  // HAL_I2C_MODULE_ENABLED
#endif  // PARAGUARDIAN_BSP_HAL_I2C_BUS_HPP
