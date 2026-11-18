/**
 ******************************************************************************
 * @file    FaultHooks.h
 * @brief   C-callable fault forwarders, called from the FreeRTOS hook
 *          functions that CubeMX generates in Src/freertos.c (USER CODE 4/5),
 *          plus boot-time reset-cause capture.
 *
 * Enabled by the .ioc settings FREERTOS.configCHECK_FOR_STACK_OVERFLOW=2 and
 * FREERTOS.configUSE_MALLOC_FAILED_HOOK=1 (names verified by letting CubeMX
 * 6.18.1 regenerate a copy of the project, see FIRMWARE_RELIABILITY_AUDIT.md).
 * The hooks themselves are NOT defined in ParaGuardian code, so there is no
 * duplicate-symbol conflict with the CubeMX-generated definitions.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_FAULT_HOOKS_H
#define PARAGUARDIAN_FAULT_HOOKS_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Stack overflow detected by the kernel (called from the context switch, i.e.
 * interrupt context with interrupts masked). Records the task name in
 * g_systemHealth, halts at a breakpoint if a debugger is attached, otherwise
 * resets the MCU. Never returns. No blocking, no heap, no printing.
 */
void paraguardian_on_stack_overflow(TaskHandle_t task, const char* taskName);

/** pvPortMalloc() returned NULL. Counts it; execution continues. */
void paraguardian_on_malloc_failed(void);

/**
 * Copies the RCC reset flags (RCC->RSR) into g_systemHealth.resetFlags and
 * clears them so the next boot reports only its own cause. Called once at
 * start-up by paraguardian_create_tasks().
 */
void paraguardian_capture_reset_cause(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAGUARDIAN_FAULT_HOOKS_H */
