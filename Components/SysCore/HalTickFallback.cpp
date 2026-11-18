/**
 ******************************************************************************
 * @file    HalTickFallback.cpp
 * @brief   Keeps HAL_GetTick() advancing while the TIM6 time-base interrupt
 *          is masked (in particular: everything that runs in main() before
 *          osKernelStart()).
 *
 * THE PROBLEM (Docs/FIRMWARE_RELIABILITY_AUDIT.md, Audit A)
 *   SoarOS constructs the global Global::vaListMutex before main(). Its
 *   constructor calls xSemaphoreCreateMutex(), which enters and exits FreeRTOS
 *   critical sections. Before the scheduler starts, the FreeRTOS Cortex-M port
 *   deliberately leaves BASEPRI raised (uxCriticalNesting starts at 0xAAAAAAAA,
 *   so the exit never reaches 0). BASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY
 *   (5) blocks the TIM6 interrupt (priority 15), so the HAL tick (uwTick) does
 *   not advance until the scheduler starts. Every HAL timeout used in main()
 *   becomes infinite and HAL_Delay() would never return.
 *
 * THE FIX
 *   HAL_GetTick() is a __weak function in stm32h7xx_hal.c; this strong
 *   definition replaces it. When (and only when) the TIM6 interrupt cannot run
 *   because of PRIMASK/BASEPRI, it polls TIM6's update flag itself: if the
 *   1 ms update event has happened, it clears the flag and calls HAL_IncTick(),
 *   exactly what the interrupt would have done.
 *   - No interrupt mask is changed.
 *   - uwTick stays the single, monotonic time base (no jump when the scheduler
 *     starts and the real interrupt takes over).
 *   - The interrupt cannot run concurrently with the poll (it is masked), so
 *     there is no double counting. If the flag was cleared here while the NVIC
 *     still has the interrupt pending, the ISR later finds UIF = 0 and does
 *     nothing (HAL_TIM_IRQHandler checks the flag).
 *   - The tick can run SLOW (one increment per poll at most, and only while
 *     somebody polls), never fast, so timeouts are at least as long as asked
 *     and always finite.
 *
 * This file must match the CubeMX time base: it references htim6 from
 * Src/stm32h7xx_hal_timebase_tim.c, so if the time base is ever changed to
 * another timer the link fails loudly instead of silently polling the wrong one.
 ******************************************************************************
 */
#include "main.h"

#include "Common/InterruptMask.hpp"

extern TIM_HandleTypeDef htim6;  // defined by CubeMX in stm32h7xx_hal_timebase_tim.c

namespace pg {
// Number of ticks produced by polling instead of by the interrupt. Non-zero on
// hardware proves the masking analysis; shown in g_systemHealth.
volatile uint32_t g_tickFallbackIncrements = 0u;
}  // namespace pg

extern "C" uint32_t HAL_GetTick(void)
{
    TIM_TypeDef* const tim = htim6.Instance;
    if (tim != nullptr && (tim->CR1 & TIM_CR1_CEN) != 0u &&
        pg::interruptBlockedByMask(__get_PRIMASK(), __get_BASEPRI(), NVIC_GetPriority(TIM6_DAC_IRQn),
                                   __NVIC_PRIO_BITS) &&
        (tim->SR & TIM_SR_UIF) != 0u) {
        // TIMx_SR bits are "rc_w0": writing 0 clears a bit, writing 1 leaves it
        // unchanged. ~TIM_SR_UIF therefore clears UIF and nothing else.
        tim->SR = ~TIM_SR_UIF;
        HAL_IncTick();
        pg::g_tickFallbackIncrements = pg::g_tickFallbackIncrements + 1u;
    }
    return uwTick;
}
