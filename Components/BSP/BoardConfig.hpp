/**
 ******************************************************************************
 * @file    BoardConfig.hpp
 * @brief   The ONE place where drivers are connected to real STM32 peripherals.
 *
 * Today the PCB schematic does not exist, so no SPI/I2C/UART instance, chip
 * select pin or I2C address is known. registerSensors() therefore registers
 * nothing and the acquisition task runs an empty loop.
 *
 * When the schematic exists (see Docs/HARDWARE_INTEGRATION_TODO.md):
 *   1. Enable the peripherals in CubeMX and regenerate.
 *   2. In BoardConfig.cpp create static HalSpiDevice / HalI2cBus / HalUartPort
 *      objects with the generated handles (e.g. &hspi1) and CS pins.
 *   3. Create the register buses, drivers and acquisition sources on top.
 *   4. Register the sources with the acquisition task.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_BSP_BOARD_CONFIG_HPP
#define PARAGUARDIAN_BSP_BOARD_CONFIG_HPP

#include <cstddef>

class SensorAcquisitionTask;

namespace pg {
namespace board {

/// Registers every wired sensor with the acquisition task. Returns how many.
size_t registerSensors(SensorAcquisitionTask& task);

}  // namespace board
}  // namespace pg

#endif  // PARAGUARDIAN_BSP_BOARD_CONFIG_HPP
