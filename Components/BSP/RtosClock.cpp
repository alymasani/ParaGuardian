/**
 ******************************************************************************
 * @file    RtosClock.cpp
 * @brief   See RtosClock.hpp.
 *
 * Hardening (Audit A, see Docs/FIRMWARE_HARDENING_WALKTHROUGH.txt):
 *  - millis() now uses HAL_GetTick() everywhere. Before, it used
 *    xTaskGetTickCount(), which is frozen at 0 before the scheduler starts, so
 *    a driver timeout loop  (millis() - start) >= timeout  could never end if a
 *    driver were used before osKernelStart(). HAL_GetTick() keeps advancing
 *    there thanks to Components/SysCore/HalTickFallback.cpp, and it is one
 *    monotonic time base from boot (no jump at scheduler start).
 *  - The pre-scheduler delay now uses HAL_Delay() (bounded for the same
 *    reason). The previous DWT cycle-counter busy-wait was removed: on
 *    Cortex-M7 parts the DWT may need its Lock Access Register unlocked before
 *    CYCCNT counts without a debugger, in which case the old loop would never
 *    have ended.
 ******************************************************************************
 */
#include "BSP/RtosClock.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

namespace pg {
namespace bsp {

uint32_t RtosClock::millis()
{
    // TIM6 HAL tick: 1 kHz (uwTickFreq = 1), counts from HAL_Init().
    return HAL_GetTick();
}

void RtosClock::delayMs(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // pdMS_TO_TICKS rounds down; vTaskDelay(n) waits between n-1 and n
        // tick periods, so add one tick to guarantee AT LEAST ms milliseconds.
        vTaskDelay(pdMS_TO_TICKS(ms) + 1u);
    } else {
        // Before the scheduler (or with it suspended): bounded busy-wait on the
        // HAL tick, which advances even while TIM6 is masked (HalTickFallback).
        // Requires HAL_Init() to have run (true for any code inside main()).
        HAL_Delay(ms);
    }
}

RtosClock& systemClock()
{
    static RtosClock clock;
    return clock;
}

}  // namespace bsp
}  // namespace pg
