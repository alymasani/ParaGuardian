/**
 ******************************************************************************
 * @file    TaskConfig.hpp
 * @brief   Priorities, stack sizes and periods of every ParaGuardian task.
 *
 * PRIORITIES (CMSIS-RTOS V2 detail)
 *   With the V2 wrapper, FreeRTOS is configured with configMAX_PRIORITIES = 56
 *   and the CMSIS osPriority_t values ARE the FreeRTOS priority numbers
 *   (osPriorityNormal = 24, osPriorityHigh = 40, ...). Under V1 there were only
 *   7 priorities and SoarOS projects used raw numbers like 2 or 3.
 *   We express our priorities with the osPriority_t names so both the CMSIS
 *   API (defaultTask) and native FreeRTOS calls (our tasks) use one scale.
 *   NOTE: the FreeRTOS timer service task runs at configTIMER_TASK_PRIORITY = 2,
 *   i.e. BELOW every task here. SoarOS Timer callbacks would only run when
 *   all ParaGuardian tasks are blocked. ParaGuardian does not use software timers.
 *
 * STACK SIZES are in WORDS (StackType_t = 4 bytes on Cortex-M), because
 * xTaskCreateStatic() takes words. CMSIS V2 osThreadAttr_t::stack_size is in
 * BYTES (CubeMX wrote "128 * 4" for defaultTask). Do not mix them up.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_TASK_CONFIG_HPP
#define PARAGUARDIAN_TASKS_TASK_CONFIG_HPP

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os2.h"

namespace TaskConfig {

// ---- Sensor acquisition: one task services every sensor --------------------
constexpr UBaseType_t ACQUISITION_PRIORITY = static_cast<UBaseType_t>(osPriorityHigh);  // 40
constexpr uint32_t ACQUISITION_STACK_WORDS = 1024u;   // 4 KiB (driver call chains + 64-byte UART chunks)
constexpr uint32_t ACQUISITION_PERIOD_MS = 10u;       // 100 Hz base loop
constexpr size_t MAX_ACQUISITION_SOURCES = 10u;       // 3 IMU + 2 baro + mag + GNSS + LiDAR + spare
constexpr uint8_t INIT_ATTEMPTS_PER_CYCLE = 1u;       // driver init() calls allowed per 10 ms cycle

// ---- Sensor preprocessing (validation, calibration, alignment, redundancy) -------
// Priority: BELOW acquisition (so sampling is never delayed by processing) and
// ABOVE health (diagnostics may wait). It therefore runs right after each
// acquisition cycle blocks, and finds that cycle's whole burst in its queue.
constexpr UBaseType_t PROCESSING_PRIORITY = static_cast<UBaseType_t>(osPriorityAboveNormal);  // 32
// 4 KiB: pipeline calls are shallow (<= ~6 frames, doubles, one pow()); the
// pipeline state and the output frame are class members, not stack variables.
// Confirm with the Health task's stack high-water mark on hardware.
constexpr uint32_t PROCESSING_STACK_WORDS = 1024u;
// Queue depth: ~575 msg/s produced (RTOS_TIMING_ANALYSIS.md section 7) = ~6 per
// 10 ms cycle, bursts of ~10. 64 x 88 B = 5.6 KiB holds ~110 ms of data, so the
// processing task can be delayed ~10 acquisition cycles before DropOldest acts.
constexpr size_t PROCESSING_QUEUE_DEPTH = 64u;
// If no sample arrives for this long, a frame is still produced so consumers
// and the health monitor SEE that the sensors are missing (no silent stall).
constexpr uint32_t PROCESSING_IDLE_TIMEOUT_MS = 50u;

// ---- Health monitor -----------------------------------------------------------
constexpr UBaseType_t HEALTH_PRIORITY = static_cast<UBaseType_t>(osPriorityBelowNormal);  // 16
constexpr uint32_t HEALTH_STACK_WORDS = 512u;         // 2 KiB
constexpr uint32_t HEALTH_PERIOD_MS = 1000u;
constexpr uint16_t HEALTH_QUEUE_DEPTH = 4u;           // SoarOS Command queue (heap: 4 x 12 B + overhead)
constexpr uint32_t LOW_STACK_WARNING_WORDS = 64u;     // warn when < 256 bytes of stack never used

// ---- Sensor sample queues (for future consumer tasks) ---------------------------
constexpr size_t SAMPLE_QUEUE_DEPTH = 32u;

static_assert(ACQUISITION_PRIORITY < configMAX_PRIORITIES, "priority out of range");
static_assert(HEALTH_PRIORITY < configMAX_PRIORITIES, "priority out of range");
static_assert(PROCESSING_PRIORITY < ACQUISITION_PRIORITY, "processing must never preempt acquisition");
static_assert(PROCESSING_PRIORITY > HEALTH_PRIORITY, "processing should run before diagnostics");
static_assert(PROCESSING_STACK_WORDS >= configMINIMAL_STACK_SIZE, "stack too small");
static_assert(ACQUISITION_STACK_WORDS >= configMINIMAL_STACK_SIZE, "stack too small");
static_assert(HEALTH_STACK_WORDS >= configMINIMAL_STACK_SIZE, "stack too small");
static_assert((ACQUISITION_PERIOD_MS * configTICK_RATE_HZ) % 1000u == 0u, "period must be a whole number of ticks");

}  // namespace TaskConfig

#endif  // PARAGUARDIAN_TASKS_TASK_CONFIG_HPP
