/**
 ******************************************************************************
 * @file    SensorProcessingTask.hpp
 * @brief   RTOS wrapper around pg::PreprocessingPipeline.
 *
 * DATA FLOW
 *   SensorAcquisitionTask --hub.publish()--> RtosSampleQueue<64> (DropOldest)
 *        --popWait--> PreprocessingPipeline::ingest() (every queued message)
 *        --buildFrame--> ProcessedFrameMailbox (latest frame, readers poll)
 *        --attitudeInputFromFrame--> AttitudeEstimator (Madgwick, once per frame)
 *        --> AttitudeMailbox (latest AttitudeEstimate, readers poll)
 *        --navigationInputFrom(frame, attitude)--> VerticalNavigationEstimator
 *          (UKF, x = [h, vz, az], once per frame; assimilates only NEW samples)
 *        --> NavigationMailbox (latest NavigationEstimate, readers poll)
 *        --safetyInputFrom(frame, attitude, navigation)--> SafetyMonitor
 *          (LiDAR ground clearance -> flight phase -> landing warnings)
 *        --> SafetySnapshotMailbox (current phase / warnings, readers poll)
 *        --> SafetyEventQueue (transitions only, bounded, priority-aware;
 *            consumed by EmergencyAlertTask's SafetyEventDispatcher)
 *        --crashInputFrom(frame, attitude, snapshot, navigation)--> CrashDetector
 *          (per-IMU peaks BEFORE voting / Madgwick / UKF smoothing; events into the
 *          same SafetyEventQueue) --> CrashReportMailbox (status + incident context)
 *
 * WHY NAVIGATION ALSO RUNS HERE: same reasons as attitude. It needs the
 * attitude of the SAME frame (gravity-compensated vertical acceleration),
 * costs a few predict/update steps of a 3-state UKF (estimated well under
 * 0.1 ms at 64 MHz, VERTICAL_UKF_DESIGN.md section 12) and never blocks.
 * The process model object (navModel_) lives here too; replacing the model
 * means changing that one member's type.
 *
 * WHY ATTITUDE RUNS HERE (not in its own task)
 *   It consumes exactly one ProcessedSensorFrame per wake, costs a few
 *   hundred floating-point operations (~10-20 us estimated at 64 MHz, see
 *   MADGWICK_ATTITUDE_DESIGN.md section 11), and needs no blocking. A
 *   separate task would add a queue, a stack and a second copy of every
 *   frame for no benefit, and would see frames LATER. Data ownership stays
 *   simple: this task is the only writer of both mailboxes. The estimator
 *   integrates a gyro sample only when its timestamp is new, so idle-timeout
 *   frames (no new IMU data) never integrate twice.
 *
 * WHY A SEPARATE TASK
 *   The acquisition task must keep its 10 ms rhythm. Validation, calibration,
 *   alignment, voting and the barometric pow() are done here, at a LOWER
 *   priority, so they can never delay a sensor read. The hub's publish() only
 *   copies 88 bytes into this task's queue and never blocks.
 *
 * QUEUE POLICY: DropOldest - for navigation the newest data matter; anything
 *   discarded is counted by the hub (overwritesFor) and shows up as sequence
 *   gaps (frame.samplesLostSinceLastFrame / quality::SamplesLost).
 *
 * The loop blocks in popWait() (no busy-waiting). It wakes when data arrives
 * or after PROCESSING_IDLE_TIMEOUT_MS, then drains at most the queue capacity
 * (bounded work per wake) and publishes one frame.
 *
 * Static allocation only (xTaskCreateStatic, xQueueCreateStatic).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP
#define PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP

#include <atomic>
#include <cstdint>

#include "Task.hpp"   // SoarOS

#include "Acquisition/SensorDataHub.hpp"
#include "Algorithms/Attitude/AttitudeEstimator.hpp"
#include "Algorithms/Attitude/AttitudeMailbox.hpp"
#include "Algorithms/Crash/CrashDetector.hpp"
#include "Algorithms/Crash/CrashInputAdapter.hpp"
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"
#include "Algorithms/Navigation/NavigationMailbox.hpp"
#include "Algorithms/Navigation/VerticalNavigationEstimator.hpp"
#include "Algorithms/Safety/SafetyEventQueue.hpp"
#include "Algorithms/Safety/SafetyMonitor.hpp"
#include "Algorithms/Safety/SafetySnapshotMailbox.hpp"
#include "Processing/PreprocessingPipeline.hpp"
#include "Processing/ProcessedFrameMailbox.hpp"
#include "Tasks/RtosCriticalSection.hpp"
#include "Tasks/RtosSampleQueue.hpp"
#include "Tasks/TaskConfig.hpp"

class SensorProcessingTask : public Task {
public:
    struct Stats {
        uint32_t batches;               ///< loop iterations (one frame each)
        uint32_t messages;              ///< messages taken from the queue
        uint32_t idleTimeouts;          ///< wakes with no data (sensors silent)
        uint32_t maxBatch;              ///< most messages drained in one wake
        uint32_t lastProcessingTimeMs;  ///< ingest + buildFrame of the last batch (1 ms resolution)
        uint32_t maxProcessingTimeMs;
    };

    static SensorProcessingTask& Inst()
    {
        static SensorProcessingTask inst;
        return inst;
    }

    /**
     * Creates the queue, subscribes it to @p hub (DropOldest) and creates the
     * thread. Call once, after osKernelInitialize(), before osKernelStart().
     */
    void InitTask(pg::SensorDataHub& hub);
    void InitTask() override;   // not used: the hub is required (asserts)

    /// Asks for a new calibration session (any task; takes effect at the next wake).
    void requestCalibration() { calibrationRequested_.store(true); }

    /// Newest processed frame (thread-safe copy).
    const pg::ProcessedFrameMailbox& frames() const { return frames_; }
    /// Newest attitude estimate (thread-safe copy). Check orientationValid / qualityFlags.
    const pg::AttitudeMailbox& attitude() const { return attitudeOut_; }
    /// Newest vertical navigation estimate (thread-safe copy). Check state / *Valid flags.
    const pg::NavigationMailbox& navigation() const { return navigationOut_; }
    /// Newest flight / landing safety snapshot (thread-safe copy).
    const pg::SafetySnapshotMailbox& safety() const { return safetyOut_; }
    /// Safety events (transitions, incl. crash events). ONE consumer pops them: EmergencyAlertTask.
    pg::SafetyEventQueue<TaskConfig::SAFETY_EVENT_QUEUE_DEPTH>& safetyEvents() { return safetyEvents_; }
    const pg::SafetyEventQueue<TaskConfig::SAFETY_EVENT_QUEUE_DEPTH>& safetyEvents() const { return safetyEvents_; }
    /// Newest crash detector status + incident context (thread-safe copy). Read by EmergencyAlertTask.
    const pg::CrashReportMailbox& crashReports() const { return crashOut_; }
    /// Counter snapshot (each field one aligned 32-bit word).
    Stats stats() const;
    TaskHandle_t handle() const { return rtTaskHandle; }
    const pg::ISampleQueue& queue() const { return queue_; }

protected:
    static void RunTask(void* pvParams) { SensorProcessingTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    SensorProcessingTask();
    SensorProcessingTask(const SensorProcessingTask&) = delete;
    SensorProcessingTask& operator=(const SensorProcessingTask&) = delete;

    pg::RtosSampleQueue<TaskConfig::PROCESSING_QUEUE_DEPTH> queue_{};
    pg::PreprocessingPipeline pipeline_;
    pg::RtosCriticalSection frameLock_{};
    pg::ProcessedFrameMailbox frames_;
    pg::ProcessedSensorFrame work_{};          // built here, then copied into frames_
    pg::AttitudeEstimator attitude_;           // portable Madgwick estimator (~0.4 KB, no heap)
    pg::RtosCriticalSection attitudeLock_{};
    pg::AttitudeMailbox attitudeOut_;
    pg::ConstantAccelerationModel navModel_;   // TEMPORARY placeholder process model (replaceable)
    pg::VerticalNavigationEstimator navigation_;   // UKF (~1 KB, no heap); references navModel_
    pg::RtosCriticalSection navigationLock_{};
    pg::NavigationMailbox navigationOut_;
    pg::SafetyMonitor safety_;                  // ground clearance + flight phase + landing warnings (no heap)
    pg::RtosCriticalSection safetyLock_{};
    pg::SafetySnapshotMailbox safetyOut_;
    pg::RtosCriticalSection safetyEventLock_{};
    pg::SafetyEventQueue<TaskConfig::SAFETY_EVENT_QUEUE_DEPTH> safetyEvents_;
    pg::CrashDetector crash_;                   // possible impact / suspected crash (~1.3 KB, no heap)
    pg::IncidentContext incident_{};            // last valid fix / navigation for an SOS request
    pg::CrashReport crashReport_{};             // built here (not on the stack), then copied into crashOut_
    pg::RtosCriticalSection crashLock_{};
    pg::CrashReportMailbox crashOut_;
    std::atomic<bool> calibrationRequested_{false};
    Stats stats_{};

    StackType_t stack_[TaskConfig::PROCESSING_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

#endif  // PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP
