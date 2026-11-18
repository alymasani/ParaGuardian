/**
 ******************************************************************************
 * @file    Bmp585Registers.hpp
 * @brief   BMP585 register map (subset used by the driver).
 *
 * Source: Bosch BMP585 datasheet (BST-BMP585-DS003) register map, and the
 * official Bosch "BMP5 Sensor API" (github.com/boschsensortec/BMP5_SensorAPI,
 * bmp5_defs.h). BMP581 and BMP585 share this register map; they differ in
 * CHIP_ID (BMP581 = 0x50, BMP585 = 0x51) and in package/application.
 *
 * VERIFY ON HARDWARE: CHIP_ID 0x51 for BMP585 is taken from the Bosch API
 * (BMP5_CHIP_ID_SEC); confirm against the datasheet revision you are using.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_BMP585_REGISTERS_HPP
#define PARAGUARDIAN_SENSORS_BMP585_REGISTERS_HPP

#include <cstdint>

namespace pg {
namespace bmp585 {

constexpr uint8_t REG_CHIP_ID = 0x01;
constexpr uint8_t REG_REV_ID = 0x02;
constexpr uint8_t REG_TEMP_DATA_XLSB = 0x1D;   // 0x1D..0x1F temperature, 0x20..0x22 pressure
constexpr uint8_t REG_INT_STATUS = 0x27;       // clear-on-read
constexpr uint8_t REG_STATUS = 0x28;
constexpr uint8_t REG_DSP_CONFIG = 0x30;
constexpr uint8_t REG_DSP_IIR = 0x31;
constexpr uint8_t REG_OSR_CONFIG = 0x36;
constexpr uint8_t REG_ODR_CONFIG = 0x37;
constexpr uint8_t REG_OSR_EFF = 0x38;
constexpr uint8_t REG_CMD = 0x7E;

constexpr uint8_t CHIP_ID_BMP585 = 0x51;
constexpr uint8_t CHIP_ID_BMP581 = 0x50;  // same register map, different part -> rejected
constexpr uint8_t CMD_SOFT_RESET = 0xB6;

// INT_STATUS bits
constexpr uint8_t INT_STATUS_DRDY = 1u << 0;
constexpr uint8_t INT_STATUS_POR = 1u << 4;     // power-on-reset / soft-reset complete

// STATUS bits
constexpr uint8_t STATUS_CORE_RDY = 1u << 0;
constexpr uint8_t STATUS_NVM_RDY = 1u << 1;
constexpr uint8_t STATUS_NVM_ERR = 1u << 2;
constexpr uint8_t STATUS_NVM_CMD_ERR = 1u << 3;

// DSP_CONFIG bits: route the IIR-filtered value into the data registers ("shadow")
constexpr uint8_t DSP_IIR_FLUSH_FORCED_EN = 1u << 2;
constexpr uint8_t DSP_SHDW_SEL_IIR_T = 1u << 3;
constexpr uint8_t DSP_SHDW_SEL_IIR_P = 1u << 5;

// OSR_CONFIG: osr_t[2:0], osr_p[5:3], press_en[6]
constexpr uint8_t OSR_PRESS_EN = 1u << 6;

// ODR_CONFIG: pwr_mode[1:0], odr[6:2], deep_dis[7]
constexpr uint8_t ODR_DEEP_DISABLE = 1u << 7;
constexpr uint8_t ODR_PWR_MODE_MASK = 0x03u;

// OSR_EFF: odr_is_valid[7]
constexpr uint8_t OSR_EFF_ODR_IS_VALID = 1u << 7;

/// Soft reset / power-up time (datasheet t_soft_res / t_pup): 2 ms.
constexpr uint32_t SOFT_RESET_DELAY_MS = 2u;

/// Conversion: temperature 24-bit signed, 1/65536 degC per LSB.
constexpr float TEMP_LSB_PER_DEGC = 65536.0f;
/// Conversion: pressure 24-bit unsigned, 1/64 Pa per LSB.
constexpr float PRESS_LSB_PER_PA = 64.0f;

/// Specified operating pressure range (300..1250 hPa) used for plausibility.
constexpr float PRESSURE_MIN_PA = 30000.0f;
constexpr float PRESSURE_MAX_PA = 125000.0f;

}  // namespace bmp585
}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_BMP585_REGISTERS_HPP
