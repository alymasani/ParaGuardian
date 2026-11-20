/**
 ******************************************************************************
 * @file    TaskSupervisor.cpp
 * @brief   See TaskSupervisor.hpp.
 ******************************************************************************
 */
#include "Common/TaskSupervisor.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

int TaskSupervisor::add(const char* name, uint32_t timeoutMs, TaskCriticality criticality)
{
    if (count_ >= kMaxTasks || timeoutMs == 0u) return -1;
    tasks_[count_] = Entry{name, timeoutMs, criticality, false, 0u, 0u, TaskLiveness::Unknown};
    return count_++;
}

void TaskSupervisor::observe(int index, uint32_t progressCounter, uint32_t nowMs)
{
    if (index < 0 || index >= count_) return;
    Entry& e = tasks_[index];
    if (!e.seen || progressCounter != e.counter) {
        e.seen = true;
        e.counter = progressCounter;
        e.lastProgressMs = nowMs;
    }
}

SupervisorVerdict TaskSupervisor::evaluate(uint32_t nowMs, const SupervisorContext& ctx)
{
    SupervisorAction worst = SupervisorAction::None;
    verdict_.stalledMask = verdict_.lateMask = 0u;
    for (uint8_t i = 0; i < count_; ++i) {
        Entry& e = tasks_[i];
        if (!e.seen) {
            e.liveness = TaskLiveness::Unknown;
            continue;
        }
        const uint32_t idle = timebase::ageMs(e.lastProgressMs, nowMs);
        TaskLiveness l = TaskLiveness::Alive;
        if (idle >= e.timeoutMs) l = TaskLiveness::Stalled;
        else if (idle >= e.timeoutMs / 2u) l = TaskLiveness::Late;
        if (l == TaskLiveness::Stalled && e.liveness != TaskLiveness::Stalled && verdict_.stallEvents != UINT32_MAX) {
            ++verdict_.stallEvents;
        }
        e.liveness = l;
        SupervisorAction a = SupervisorAction::None;
        if (l == TaskLiveness::Late) {
            verdict_.lateMask = static_cast<uint8_t>(verdict_.lateMask | (1u << i));
            a = SupervisorAction::Report;
        } else if (l == TaskLiveness::Stalled) {
            verdict_.stalledMask = static_cast<uint8_t>(verdict_.stalledMask | (1u << i));
            a = e.criticality == TaskCriticality::Critical ? SupervisorAction::RequestReset
                : e.criticality == TaskCriticality::Storage ? SupervisorAction::Degrade
                                                             : SupervisorAction::Report;
        }
        if (static_cast<uint8_t>(a) > static_cast<uint8_t>(worst)) worst = a;
    }
    verdict_.resetDeferred = false;
    if (worst == SupervisorAction::RequestReset && ctx.emergencyOpen && !ctx.emergencyDurable) {
        if (!deferring_) {
            deferring_ = true;
            deferStartMs_ = nowMs;
        }
        if (timebase::ageMs(deferStartMs_, nowMs) < resetDeferMs_) {
            worst = SupervisorAction::Degrade;   // give the storage task time to commit the incident
            verdict_.resetDeferred = true;
        }
    } else if (worst != SupervisorAction::RequestReset) {
        deferring_ = false;
    }
    verdict_.action = worst;
    return verdict_;
}

TaskLiveness TaskSupervisor::liveness(int index) const
{
    return (index >= 0 && index < count_) ? tasks_[index].liveness : TaskLiveness::Unknown;
}

const char* TaskSupervisor::name(int index) const
{
    return (index >= 0 && index < count_) ? tasks_[index].name : "";
}

}  // namespace pg
