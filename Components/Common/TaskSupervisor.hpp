/**
 ******************************************************************************
 * @file    TaskSupervisor.hpp
 * @brief   Portable task-heartbeat supervision and watchdog POLICY (no
 *          hardware watchdog is enabled; see MILESTONE8_HARDWARE_BRINGUP.md).
 *
 * Each supervised task exposes a progress counter that it already increments
 * (acquisition cycles, processing batches, emergency wakes, storage steps).
 * The health task samples the counters; a counter that does not move for
 * the task's timeout means the task is STALLED (blocked, starved, looping
 * without reaching its progress point).
 *
 * ACTIONS (worst over all tasks):
 *   Report         a non-critical task is late / stalled (diagnostics, SystemFault alert)
 *   Degrade        storage stalled: emergency state no longer becomes durable (alert, keep running)
 *   RequestReset   a CRITICAL task (acquisition, emergency) is stalled. A reset is the only
 *                  recovery for a stuck emergency task, BUT it must not erase an unresolved
 *                  emergency: while an incident is open and its newest state is not yet
 *                  committed, the reset is DEFERRED for up to resetDeferMs (the storage task
 *                  gets the chance to commit), then requested anyway (a stuck SOS path is worse).
 * feedWatchdog() is the single answer the hardware watchdog layer would use.
 * Bounded: fixed table, O(tasks) per evaluate().
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_TASK_SUPERVISOR_HPP
#define PARAGUARDIAN_COMMON_TASK_SUPERVISOR_HPP

#include <cstdint>

namespace pg {

enum class TaskCriticality : uint8_t { Diagnostic = 0, Storage, Critical };
enum class TaskLiveness : uint8_t { Unknown = 0, Alive, Late, Stalled };
enum class SupervisorAction : uint8_t { None = 0, Report, Degrade, RequestReset };

struct SupervisorContext {
    bool emergencyOpen;        ///< an incident is not Idle / closed
    bool emergencyDurable;     ///< its newest state is committed (PersistState::Committed)
};

struct SupervisorVerdict {
    SupervisorAction action;
    uint8_t stalledMask;       ///< bit i = task i stalled
    uint8_t lateMask;
    bool resetDeferred;        ///< RequestReset withheld while the emergency state is not durable
    uint32_t stallEvents;      ///< lifetime transitions into Stalled
};

class TaskSupervisor {
public:
    static constexpr uint8_t kMaxTasks = 8u;

    explicit TaskSupervisor(uint32_t resetDeferMs = 5000u) : resetDeferMs_(resetDeferMs) {}

    /// Returns the task index, or -1 if the table is full / the timeout is 0.
    int add(const char* name, uint32_t timeoutMs, TaskCriticality criticality);
    void observe(int index, uint32_t progressCounter, uint32_t nowMs);
    SupervisorVerdict evaluate(uint32_t nowMs, const SupervisorContext& ctx);

    bool feedWatchdog() const { return verdict_.action != SupervisorAction::RequestReset; }
    TaskLiveness liveness(int index) const;
    const char* name(int index) const;
    uint8_t count() const { return count_; }

private:
    struct Entry {
        const char* name;
        uint32_t timeoutMs;
        TaskCriticality criticality;
        bool seen;
        uint32_t counter;
        uint32_t lastProgressMs;
        TaskLiveness liveness;
    };
    Entry tasks_[kMaxTasks] = {};
    uint8_t count_ = 0u;
    uint32_t resetDeferMs_;
    bool deferring_ = false;
    uint32_t deferStartMs_ = 0u;
    SupervisorVerdict verdict_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_TASK_SUPERVISOR_HPP
