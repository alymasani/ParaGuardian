/**
 ******************************************************************************
 * @file    Lsm6dsrxRegisters.hpp
 * @brief   LSM6DSRX register addresses and bit fields used by the driver.
 *
 * Source: ST LSM6DSRX datasheet (DS13633), section "Register mapping" and
 * "Register description", cross-checked against ST's official C driver
 * (STMicroelectronics/lsm6dsrx-pid, lsm6dsrx_reg.h).
 * Only the registers the driver actually uses are listed.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_LSM6DSRX_REGISTERS_HPP
#define PARAGUARDIAN_SENSORS_LSM6DSRX_REGISTERS_HPP

#include <cstdint>

namespace pg {
namespace lsm6dsrx {

// ---- Register addresses -------------------------------------------------
constexpr uint8_t REG_WHO_AM_I = 0x0F;   // read-only, always 0x6B
constexpr uint8_t REG_CTRL1_XL = 0x10;   // accel ODR_XL[7:4] FS_XL[3:2] LPF2_XL_EN[1]
constexpr uint8_t REG_CTRL2_G = 0x11;    // gyro  ODR_G[7:4]  FS_G[3:2] FS_125[1] FS_4000[0]
constexpr uint8_t REG_CTRL3_C = 0x12;    // BOOT[7] BDU[6] H_LACTIVE[5] PP_OD[4] SIM[3] IF_INC[2] - SW_RESET[0]
constexpr uint8_t REG_STATUS = 0x1E;     // - - - - - TDA[2] GDA[1] XLDA[0]
constexpr uint8_t REG_OUT_TEMP_L = 0x20; // 0x20..0x21 temperature, 0x22..0x27 gyro XYZ, 0x28..0x2D accel XYZ

// ---- Constants -----------------------------------------------------------
constexpr uint8_t WHO_AM_I_VALUE = 0x6B;

constexpr uint8_t CTRL3_SW_RESET = 1u << 0;
constexpr uint8_t CTRL3_IF_INC = 1u << 2;   // auto-increment register address in burst reads
constexpr uint8_t CTRL3_BDU = 1u << 6;      // block data update: LSB/MSB of one sample never mixed

constexpr uint8_t STATUS_XLDA = 1u << 0;    // new accelerometer data available
constexpr uint8_t STATUS_GDA = 1u << 1;     // new gyroscope data available
constexpr uint8_t STATUS_TDA = 1u << 2;     // new temperature data available

constexpr uint8_t CTRL2_FS_4000 = 1u << 0;
constexpr uint8_t CTRL2_FS_125 = 1u << 1;

/// Burst length of temperature (2) + gyro (6) + accel (6) output registers.
constexpr uint8_t OUTPUT_BLOCK_LEN = 14u;

/// Temperature: 256 LSB per degC, raw 0 corresponds to 25 degC.
constexpr float TEMP_LSB_PER_DEGC = 256.0f;
constexpr float TEMP_OFFSET_DEGC = 25.0f;

}  // namespace lsm6dsrx
}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_LSM6DSRX_REGISTERS_HPP
