/**
 ******************************************************************************
 * @file    RtosCriticalSection.hpp
 * @brief   ICriticalSection implemented with FreeRTOS task-level critical
 *          sections (raises BASEPRI to configMAX_SYSCALL_INTERRUPT_PRIORITY).
 *
 * TASK CONTEXT ONLY. Keep the protected work tiny (a ~100-byte copy), because
 * interrupts at priority 5..15 (including the TIM6 HAL tick) wait meanwhile.
 * Nesting is handled by FreeRTOS (uxCriticalNesting).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_RTOS_CRITICAL_SECTION_HPP
#define PARAGUARDIAN_TASKS_RTOS_CRITICAL_SECTION_HPP

#include "FreeRTOS.h"
#include "task.h"

#include "Acquisition/LatestSampleMailbox.hpp"

namespace pg {

class RtosCriticalSection final : public ICriticalSection {
public:
    void enter() override { taskENTER_CRITICAL(); }
    void exit() override { taskEXIT_CRITICAL(); }
};

}  // namespace pg

#endif  // PARAGUARDIAN_TASKS_RTOS_CRITICAL_SECTION_HPP
