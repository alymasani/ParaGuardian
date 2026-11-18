/**
 ******************************************************************************
 * @file    HalSpiDevice.hpp
 * @brief   ISpiDevice implemented with STM32 HAL SPI + a GPIO chip-select.
 *
 * Compiled only when CubeMX has enabled an SPI peripheral
 * (HAL_SPI_MODULE_ENABLED in Inc/stm32h7xx_hal_conf.h). Today the .ioc has no
 * SPI, so this adapter is compiled out and nothing references it.
 *
 * Blocking (polling) HAL calls with a timeout are used first because they are
 * simple and easy to debug. A sensor register read is a few microseconds at
 * multi-MHz SPI clocks. DMA can replace this later without touching drivers.
 *
 * Optional bus lock: when several tasks share one SPI bus, pass a SoarOS
 * Mutex; each transaction then holds the mutex for its duration.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_BSP_HAL_SPI_DEVICE_HPP
#define PARAGUARDIAN_BSP_HAL_SPI_DEVICE_HPP

#include "main.h"

#if defined(HAL_SPI_MODULE_ENABLED)

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Mutex.hpp"  // SoarOS

namespace pg {
namespace bsp {

class HalSpiDevice final : public ISpiDevice {
public:
    /**
     * @param hspi       CubeMX-generated handle, e.g. &hspi1 (owned by main.c, lives forever)
     * @param csPort     GPIO port of this device's chip-select (from the schematic)
     * @param csPin      GPIO pin mask of the chip-select (e.g. GPIO_PIN_4)
     * @param timeoutMs  HAL timeout per transfer
     * @param busLock    optional mutex shared by all devices on the same bus
     */
    HalSpiDevice(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint32_t timeoutMs,
                 Mutex* busLock = nullptr)
        : hspi_(hspi), csPort_(csPort), csPin_(csPin), timeoutMs_(timeoutMs), busLock_(busLock) {}

    Status writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override;
    Status writeThenWrite(const uint8_t* header, size_t headerLen, const uint8_t* payload,
                          size_t payloadLen) override;

    /// Converts a HAL status into a ParaGuardian Status.
    static Status fromHal(HAL_StatusTypeDef s);

private:
    void select() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET); }   // CS active-low
    void deselect() { HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET); }
    bool lock();
    void unlock();

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
    uint32_t timeoutMs_;
    Mutex* busLock_;
};

}  // namespace bsp
}  // namespace pg

#endif  // HAL_SPI_MODULE_ENABLED
#endif  // PARAGUARDIAN_BSP_HAL_SPI_DEVICE_HPP
