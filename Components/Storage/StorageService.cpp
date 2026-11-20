/**
 ******************************************************************************
 * @file    StorageService.cpp
 * @brief   See StorageService.hpp.
 ******************************************************************************
 */
#include "Storage/StorageService.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
void inc(uint32_t& c, uint32_t by = 1u)
{
    c = (c > UINT32_MAX - by) ? UINT32_MAX : c + by;
}
bool reached(uint32_t now, uint32_t t) { return timebase::diffMs(now, t) >= 0; }
}  // namespace

StorageService::StorageService(INorFlash& flash, JournaledEmergencyStore& store, ILogSource& critical,
                               ILogSource& telemetry, const StorageServiceConfig& cfg, IClock* clock)
    : flash_(flash),
      store_(store),
      criticalQ_(critical),
      telemetryQ_(telemetry),
      cfg_(cfg),
      clock_(clock),
      journal_(flash, partition::kEmergencyJournal),
      critical_(flash, partition::kCriticalLog, logfmt::kTagCritical, LogFullPolicy::OverwriteOldest),
      flight_(flash, partition::kFlightLog, logfmt::kTagFlight, LogFullPolicy::OverwriteOldest)
{
}

void StorageService::start(uint32_t nowMs)
{
    started_ = true;
    st_.emergencyDegraded = !flash_.ready();   // reported until a commit succeeds
    journal_.mount();
    store_.publishMount(journal_);   // also when Unavailable: the emergency task must not wait forever
    const LogMount c = critical_.mount();
    const LogMount f = flight_.mount();
    logsMounted_ = c != LogMount::Unavailable && f != LogMount::Unavailable;
    lastMountMs_ = nowMs;
    lastTelemetryFlushMs_ = nowMs;
    if (journal_.mountState() == JournalMount::Corrupt) recordFault(1u, Status::ChecksumError, 0u, nowMs);
    if (c == LogMount::Corrupt) recordFault(2u, Status::ChecksumError, 0u, nowMs);
    if (f == LogMount::Corrupt) recordFault(3u, Status::ChecksumError, 0u, nowMs);
    st_.lastUnit = static_cast<uint32_t>(StorageUnit::Mount);
    syncCounters();
}

void StorageService::recordFault(uint8_t component, Status status, uint32_t detail, uint32_t nowMs)
{
    inc(faultsLogged_);
    LogPayload p;
    p.put8(component);
    p.put8(static_cast<uint8_t>(status));
    p.put32(detail);
    p.put32(faultsLogged_);
    LogEntry e{};
    e.type = LogType::StorageFault;
    e.length = static_cast<uint8_t>(p.size());
    e.timestampMs = nowMs;
    for (size_t i = 0; i < p.size(); ++i) e.payload[i] = p.data()[i];
    if (isOk(critical_.append(e))) critical_.flush();
}

bool StorageService::commitEmergency(uint32_t nowMs)
{
    uint32_t ticket = 0u;
    if (!store_.takePending(scratch_, ticket)) {
        waiting_ = false;
        return false;
    }
    if (!waiting_) {
        waiting_ = true;
        pendingSinceMs_ = nowMs;
    }
    if (ticket != attemptTicket_) {   // a newer record: fresh attempt budget, no backoff
        attemptTicket_ = ticket;
        attempts_ = 0u;
        retryAtMs_ = nowMs;
    }
    if (!reached(nowMs, retryAtMs_)) return false;   // backing off: let lower-priority work run
    inc(st_.emergencyAttempts);
    const Status s = journal_.commit(scratch_, sizeof(scratch_), nowMs);
    if (isOk(s)) {
        store_.reportCommitted(ticket);
        const uint32_t waited = timebase::ageMs(pendingSinceMs_, nowMs);
        if (waited > st_.maxEmergencyWaitMs) st_.maxEmergencyWaitMs = waited;
        st_.emergencyDegraded = false;
        backoffMs_ = 0u;
        attempts_ = 0u;
        waiting_ = store_.hasPending();
        if (waiting_) pendingSinceMs_ = nowMs;
        return true;
    }
    ++attempts_;
    if (attempts_ >= cfg_.attemptsPerTicket) {
        store_.reportFailed(ticket);
        st_.emergencyDegraded = true;
        backoffMs_ = backoffMs_ == 0u ? cfg_.retryBaseMs
                                      : (backoffMs_ >= cfg_.retryMaxMs / 2u ? cfg_.retryMaxMs : backoffMs_ * 2u);
        retryAtMs_ = nowMs + backoffMs_;
        attempts_ = 0u;
        recordFault(1u, s, ticket, nowMs);
    }
    return true;
}

bool StorageService::drain(ILogSource& q, FlightLog& log, bool critical, uint32_t nowMs)
{
    if (q.size() == 0u) return false;
    LogEntry e{};
    for (uint8_t i = 0; i < cfg_.recordsPerStep && q.peek(e); ++i) {
        const Status s = log.append(e);
        if (s == Status::Busy || s == Status::NotInitialized) break;   // rotation retry later / no medium
        q.pop();   // appended, or refused for good (full / invalid): never retried in a loop
        if (!isOk(s)) {
            inc(st_.logFailures);
            break;
        }
        inc(critical ? st_.criticalRecords : st_.telemetryRecords);
    }
    if (critical) {
        if (!isOk(log.flush())) inc(st_.logFailures);
    } else if (log.dirty() && timebase::ageMs(lastTelemetryFlushMs_, nowMs) >= cfg_.telemetryFlushMs) {
        if (!isOk(log.flush())) inc(st_.logFailures);
        lastTelemetryFlushMs_ = nowMs;
    }
    return true;
}

bool StorageService::step(uint32_t nowMs)
{
    if (!started_) start(nowMs);
    inc(st_.steps);
    const uint32_t t0 = clock_ != nullptr ? clock_->millis() : 0u;
    StorageUnit unit = StorageUnit::None;

    if (commitEmergency(nowMs)) {
        unit = StorageUnit::EmergencyCommit;
    } else if (!logsMounted_ && timebase::ageMs(lastMountMs_, nowMs) >= cfg_.remountMs) {
        lastMountMs_ = nowMs;
        logsMounted_ = critical_.mount() != LogMount::Unavailable && flight_.mount() != LogMount::Unavailable;
        unit = StorageUnit::Mount;
    } else if (drain(criticalQ_, critical_, true, nowMs)) {
        unit = StorageUnit::CriticalLog;
    } else if (drain(telemetryQ_, flight_, false, nowMs)) {
        unit = StorageUnit::TelemetryLog;
    } else if (flight_.dirty() && timebase::ageMs(lastTelemetryFlushMs_, nowMs) >= cfg_.telemetryFlushMs) {
        if (!isOk(flight_.flush())) inc(st_.logFailures);
        lastTelemetryFlushMs_ = nowMs;
        unit = StorageUnit::TelemetryLog;
    } else if (journal_.maintenanceDue()) {
        if (!isOk(journal_.maintain())) recordFault(1u, Status::VerifyFailed, journal_.activeSector(), nowMs);
        unit = StorageUnit::Maintenance;
    }
    if (clock_ != nullptr) {
        const uint32_t took = timebase::ageMs(t0, clock_->millis());
        if (took > st_.maxUnitMs) st_.maxUnitMs = took;
    }
    st_.lastUnit = static_cast<uint32_t>(unit);
    syncCounters();
    if (unit == StorageUnit::None) return false;
    const bool emergencyReady = store_.hasPending() && reached(nowMs, retryAtMs_);
    return emergencyReady || criticalQ_.size() != 0u || telemetryQ_.size() != 0u;
}

void StorageService::flushAll()
{
    critical_.flush();
    flight_.flush();
}

void StorageService::syncCounters()
{
    st_.journalMount = static_cast<uint8_t>(journal_.mountState());
    st_.criticalMount = static_cast<uint8_t>(critical_.mountState());
    st_.flightMount = static_cast<uint8_t>(flight_.mountState());
    st_.journalSequence = journal_.sequence();
    st_.bootCount = journal_.bootCount();
    st_.emergencyCommits = journal_.stats().commits;
    st_.emergencyCommitFailures = journal_.stats().commitFailures;
    st_.recordsLost = critical_.stats().recordsLost + flight_.stats().recordsLost;
}

}  // namespace pg
