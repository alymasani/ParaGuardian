/**
 ******************************************************************************
 * @file    SoarOSCmsisV2Compat.h
 * @brief   Makes the unmodified SoarOS (Cube++) sources compile on a CubeMX
 *          project that uses FreeRTOS with the CMSIS-RTOS **V2** wrapper.
 *
 * HOW IT IS USED
 *   This header is FORCE-INCLUDED into every C++ file by the CubeIDE setting
 *   "MCU/MPU G++ Compiler > Include files (-include)". The compiler behaves as
 *   if each .cpp file started with  #include "SoarOSCmsisV2Compat.h".
 *   It is NOT applied to C files, so CubeMX-generated C code is untouched.
 *
 * WHY IT IS NEEDED (the two concrete V1 -> V2 differences SoarOS hits)
 *   1. With CMSIS-RTOS V1, ST's cmsis_os.h itself included FreeRTOS.h,
 *      task.h, queue.h, semphr.h and timers.h. SoarOS relies on that: e.g.
 *      Mutex.hpp includes only "cmsis_os.h" and then uses SemaphoreHandle_t,
 *      Task.hpp uses TaskHandle_t. The V2 cmsis_os.h only includes
 *      cmsis_os2.h, so those native FreeRTOS types are undeclared.
 *   2. SoarOS CubeUtils.hpp converts ms <-> ticks with the V1 macro
 *      osKernelSysTickFrequency. V2 has no such macro (it has the function
 *      osKernelGetTickFreq()). SoarOS uses it in constexpr initialisers, so
 *      a compile-time constant is required: configTICK_RATE_HZ, which is
 *      exactly what the macro meant in V1.
 *
 * WHAT IT DELIBERATELY DOES NOT DO
 *   It does not alias V1 functions (osThreadCreate, osMessageGet, ...) to V2
 *   ones. SoarOS does not call those; it calls native FreeRTOS functions
 *   (xTaskCreate, xQueueSend, xSemaphoreTake, xTimerCreate), which behave the
 *   same under V1 and V2 because both wrappers sit on the same kernel.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SOAROS_CMSIS_V2_COMPAT_H
#define PARAGUARDIAN_SOAROS_CMSIS_V2_COMPAT_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "cmsis_os.h"   /* CMSIS-RTOS V2 header (defines osCMSIS 0x20001) */

#if !defined(osCMSIS) || (osCMSIS < 0x20000U)
#error "SoarOSCmsisV2Compat.h is only for CMSIS-RTOS V2 projects"
#endif

#ifndef osKernelSysTickFrequency
/* V1 meaning: kernel tick frequency in Hz. Used by SoarOS MS_TO_TICKS / TICKS_TO_MS. */
#define osKernelSysTickFrequency configTICK_RATE_HZ
#endif

#ifdef __cplusplus
/* SoarOS computes ticks = ms * (osKernelSysTickFrequency / 1000) with INTEGER
 * division. That is only exact when the tick rate is a multiple of 1000 Hz;
 * at e.g. 100 Hz it would silently become ms * 0. Fail the build instead. */
static_assert((configTICK_RATE_HZ % 1000U) == 0U,
              "SoarOS MS_TO_TICKS requires configTICK_RATE_HZ to be a multiple of 1000 Hz");
#endif

#endif /* PARAGUARDIAN_SOAROS_CMSIS_V2_COMPAT_H */
