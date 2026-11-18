/**
 ******************************************************************************
 * @file    RtosClock.hpp
 * @brief   IClock implementation for the STM32 + FreeRTOS firmware.
 *
 * millis():  HAL_GetTick() (TIM6, 1 ms), monotonic from HAL_Init(). Keeps
 *            advancing before the scheduler starts thanks to
 *            Components/SysCore/HalTickFallback.cpp.
 * delayMs(): vTaskDelay() once the scheduler runs, so other tasks keep running
 *            while a driver waits for a sensor. Before the scheduler starts it
 *            uses HAL_Delay(), which is bounded for the same reason.
 *            (Drivers are normally initialised inside tasks.)
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_BSP_RTOS_CLOCK_HPP
#define PARAGUARDIAN_BSP_RTOS_CLOCK_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"

namespace pg {
namespace bsp {

class RtosClock final : public IClock {
public:
    uint32_t millis() override;
    void delayMs(uint32_t ms) override;
};

/// The single clock instance used by all drivers in the firmware.
RtosClock& systemClock();

}  // namespace bsp
}  // namespace pg

#endif  // PARAGUARDIAN_BSP_RTOS_CLOCK_HPP
