/**
 ******************************************************************************
 * @file    FaultHooks.cpp
 * @brief   See FaultHooks.h.
 *
 * What survives a reset:
 *   - RCC->RSR reset flags (hardware): e.g. SFTRSTF = software reset (stack
 *     overflow hook, SOAR_ASSERT, NVIC_SystemReset), IWDG1RSTF = watchdog,
 *     BORRSTF = brown-out, PINRSTF = reset pin, PORRSTF = power-on.
 *     Read at boot into g_systemHealth.resetFlags.
 * What does NOT survive:
 *   - g_systemHealth itself (normal .bss, zeroed at reset), so the overflowing
 *     task's name is only visible if a debugger halts before the reset.
 *   - Persisting the detailed cause needs RTC backup registers or a flash
 *     log; deliberately not added yet (see the audit, "residual risks").
 *     A .noinit RAM section was rejected because the STM32H7 SRAM ECC can
 *     flag reads of never-written RAM after power-up.
 ******************************************************************************
 */
#include "SysCore/FaultHooks.h"

#include "main.h"
#include "Tasks/SystemHealth.hpp"

extern "C" void paraguardian_on_stack_overflow(TaskHandle_t /*task*/, const char* taskName)
{
    taskDISABLE_INTERRUPTS();
    pg::SystemHealthSnapshot& h = pg::g_systemHealth;
    ++h.stackOverflowCount;
    // Bounded copy of the task name (configMAX_TASK_NAME_LEN includes the NUL).
    size_t i = 0u;
    if (taskName != nullptr) {
        for (; i + 1u < sizeof(h.lastStackOverflowTask) && taskName[i] != '\0'; ++i) {
            h.lastStackOverflowTask[i] = taskName[i];
        }
    }
    h.lastStackOverflowTask[i] = '\0';

    // A BKPT instruction without a debugger escalates to HardFault, so only
    // halt when a debugger is actually attached (C_DEBUGEN set).
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u) {
        __BKPT(0);
    }
    // The overflowed stack may have corrupted other memory: do not continue.
    NVIC_SystemReset();
}

extern "C" void paraguardian_on_malloc_failed(void)
{
    ++pg::g_systemHealth.mallocFailedCount;
}

extern "C" void paraguardian_capture_reset_cause(void)
{
    pg::g_systemHealth.resetFlags = RCC->RSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();  // RCC->RSR |= RMVF
}
