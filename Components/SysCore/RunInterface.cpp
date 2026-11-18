/**
 ******************************************************************************
 * @file    RunInterface.cpp
 * @brief   C entry points called from Src/main.c (see RunInterface.h).
 *
 * Difference from the SoarOS/DAQ "run_interface()" pattern:
 *   SoarOS projects on CMSIS V1 call run_interface() in USER CODE 2, and
 *   run_main() itself calls osKernelStart(); the generated scheduler start is
 *   hidden behind "#if 0 ... #endif". That skips osKernelInitialize(), which
 *   CMSIS V2 requires before any RTOS object is created and before
 *   osKernelStart() (V2's osKernelStart() refuses to run unless the kernel
 *   state is "ready", set by osKernelInitialize()).
 *   ParaGuardian therefore keeps the generated V2 sequence intact and only
 *   inserts task creation between osKernelInitialize() and osKernelStart().
 ******************************************************************************
 */
#include "SysCore/RunInterface.h"

#include "SystemDefines.hpp"
#include "BSP/BoardConfig.hpp"
#include "SysCore/FaultHooks.h"
#include "Tasks/HealthMonitorTask.hpp"
#include "Tasks/SensorAcquisitionTask.hpp"
#include "Tasks/SensorProcessingTask.hpp"
#include "Tasks/SystemHealth.hpp"

namespace {
bool s_tasksCreated = false;
}

extern "C" void paraguardian_create_tasks(void)
{
    // Guard: creating the tasks twice would create two threads on the same
    // static stack. Ignore a second call instead.
    if (s_tasksCreated) {
        ++pg::g_systemHealth.startupErrors;
        return;
    }
    s_tasksCreated = true;

    // Record why we (re)booted before anything else can reset again.
    paraguardian_capture_reset_cause();

    // Must run after osKernelInitialize() and before osKernelStart().
    SOAR_ASSERT(osKernelGetState() == osKernelReady, "paraguardian_create_tasks: kernel not in Ready state");

    // 1. Sensor acquisition: the board layer registers whatever sensors are
    //    wired (none until the schematic exists), then the thread is created.
    SensorAcquisitionTask& acquisition = SensorAcquisitionTask::Inst();
    pg::board::registerSensors(acquisition);
    acquisition.InitTask();

    // 2. Sensor preprocessing: subscribes its own queue (DropOldest) to the
    //    acquisition hub, then creates its thread (lower priority than acquisition).
    SensorProcessingTask& processing = SensorProcessingTask::Inst();
    processing.InitTask(acquisition.hub());

    // 3. Health monitor (also watches its own stack).
    HealthMonitorTask& health = HealthMonitorTask::Inst();
    health.watchTask(acquisition.handle(), "SensorAcq");
    health.watchTask(processing.handle(), "SensorProc");
    health.InitTask();
}

extern "C" void paraguardian_default_task(void)
{
    // The CubeMX defaultTask was created with osThreadNew() (CMSIS V2), so it
    // is ended with the matching V2 call. osThreadExit() never returns.
    osThreadExit();
}
