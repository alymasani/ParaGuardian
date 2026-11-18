/**
 ******************************************************************************
 * @file    SystemDefines.hpp
 * @brief   Project configuration that SoarOS (Cube++) requires every
 *          application to provide. SoarOS headers include it by bare name.
 *
 * Modelled on the team's DAQBoard Components/SystemDefines.hpp and the
 * Cube++ README "Codebase Setup" section.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SYSTEM_DEFINES_HPP
#define PARAGUARDIAN_SYSTEM_DEFINES_HPP

/* Debug output -------------------------------------------------------------
 * The PCB does not exist yet, so no UART pins are verified for a debug port.
 * Following the Cube++ README, the debug UART is disabled: SOAR_PRINT becomes
 * a no-op and SOAR_ASSERT resets the MCU without printing. Use the SWD
 * debugger (breakpoints, Live Expressions on g_systemHealth) instead.
 * To enable later: configure a UART in CubeMX with the LL driver, create a
 * UARTDriver for it, point DEFAULT_DEBUG_UART_DRIVER at it, remove this define,
 * and start CubeTask in Components/SysCore/RunInterface.cpp.
 */
#define DISABLE_DEBUG

/* System wide includes ----------------------------------------------------*/
#include "main.h"            // HAL + device header (USART_TypeDef etc.)
#include "CubeDefines.hpp"   // SOAR_ASSERT, SOAR_PRINT, soar_malloc
#include "UARTDriver.hpp"    // SoarOS class, header only (UARTDriver.cpp is excluded from the build)

/* Cube++ required configuration -------------------------------------------*/
constexpr UARTDriver* const DEFAULT_DEBUG_UART_DRIVER = nullptr;  // no debug UART (see DISABLE_DEBUG)

/// Global command categories used by SoarOS Command objects.
enum GLOBAL_COMMANDS : uint8_t {
    COMMAND_NONE = 0,       // No command, packet can be ignored
    TASK_SPECIFIC_COMMAND,  // Task-specific command in taskCommand
    DATA_COMMAND,           // Data for a task; target in taskCommand
    DATA_BROKER_COMMAND,    // Reserved for SoarOS DataBroker compatibility (not used by ParaGuardian)
};

/* Task parameters required by SoarOS CubeTask (compiled, but NOT started
 * while DISABLE_DEBUG is defined). Same values as the DAQ board. */
constexpr uint8_t UART_TASK_RTOS_PRIORITY = 2;
constexpr uint8_t UART_TASK_QUEUE_DEPTH_OBJS = 10;
constexpr uint16_t UART_TASK_STACK_DEPTH_WORDS = 512;

/* ParaGuardian task parameters live in Components/Tasks/TaskConfig.hpp. */

#endif  // PARAGUARDIAN_SYSTEM_DEFINES_HPP
