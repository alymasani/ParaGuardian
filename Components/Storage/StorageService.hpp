/**
 ******************************************************************************
 * @file    StorageService.hpp
 * @brief   Portable scheduler of every external-flash operation: emergency
 *          journal commits, critical log, flight (telemetry) log, journal
 *          maintenance. Run by the low-priority StorageTask; host-testable
 *          with a fake NOR flash.
 *
 * ONE UNIT OF WORK PER step(), IN STRICT PRIORITY ORDER:
 *   1. emergency record pending   -> journal commit (retries, backoff, failure report)
 *   2. critical log entries       -> append + flush immediately
 *   3. telemetry log entries      -> append; flush when a page fills / every telemetryFlushMs
 *   4. idle maintenance           -> journal pre-erase
 * The emergency record is checked before EVERY unit, so it waits for at most
 * one unit already in progress (worst case one sector erase, W25Q128JV tSE
 * max 400 ms, plus one page program). Telemetry can never delay it longer,
 * however much is queued.
 *
 * "SAVED" means committed and read back (JournaledEmergencyStore tickets),
 * never "queued". Failed commits are retried with backoff forever; the
 * manager reports PersistenceDegraded meanwhile.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_STORAGE_SERVICE_HPP
#define PARAGUARDIAN_STORAGE_STORAGE_SERVICE_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Storage/EmergencyJournal.hpp"
#include "Storage/FlightLog.hpp"
#include "Storage/JournaledEmergencyStore.hpp"
#include "Storage/LogRecords.hpp"

namespace pg {

struct StorageServiceConfig {
    uint8_t attemptsPerTicket = 3u;      ///< immediate commit attempts (each on a fresh slot)
    uint32_t retryBaseMs = 200u;         ///< then back off: 200 ms, 400 ms, ... up to retryMaxMs
    uint32_t retryMaxMs = 10000u;
    uint32_t telemetryFlushMs = 1000u;   ///< max telemetry data at risk on a power cut
    uint32_t remountMs = 5000u;          ///< flash unavailable at boot: retry the logs this often
    uint8_t recordsPerStep = 8u;         ///< log entries appended per unit (bounded work)
};

enum class StorageUnit : uint8_t { None = 0, Mount, EmergencyCommit, CriticalLog, TelemetryLog, Maintenance };

struct StorageStatus {
    uint8_t journalMount;          ///< pg::JournalMount
    uint8_t criticalMount;         ///< pg::LogMount
    uint8_t flightMount;           ///< pg::LogMount
    bool emergencyDegraded;        ///< last emergency commit failed (retrying)
    uint32_t journalSequence;
    uint32_t bootCount;
    uint32_t emergencyCommits;
    uint32_t emergencyCommitFailures;
    uint32_t emergencyAttempts;
    uint32_t criticalRecords;
    uint32_t telemetryRecords;
    uint32_t logFailures;          ///< append / flush / erase failures (both logs)
    uint32_t recordsLost;          ///< appended but lost to a failed program (both logs)
    uint32_t steps;
    uint32_t lastUnit;             ///< pg::StorageUnit
    uint32_t maxEmergencyWaitMs;   ///< record pending -> committed, worst seen (service view)
    uint32_t maxUnitMs;            ///< longest single unit (flash time)
};

class StorageService {
public:
    StorageService(INorFlash& flash, JournaledEmergencyStore& store, ILogSource& critical, ILogSource& telemetry,
                   const StorageServiceConfig& cfg = StorageServiceConfig{}, IClock* clock = nullptr);

    /// Mounts the journal (published to the store even when unavailable) and both logs.
    void start(uint32_t nowMs);
    /// One bounded unit of work. Returns true if more work is ready right now.
    bool step(uint32_t nowMs);
    /// Forces the telemetry page out (e.g. before a planned shutdown).
    void flushAll();

    StorageStatus status() const { return st_; }
    const EmergencyJournal& journal() const { return journal_; }
    const FlightLog& criticalLog() const { return critical_; }
    const FlightLog& flightLog() const { return flight_; }
    /// Appends a storage-fault record directly to the critical log (storage task only).
    void recordFault(uint8_t component, Status status, uint32_t detail, uint32_t nowMs);

private:
    bool commitEmergency(uint32_t nowMs);
    bool drain(ILogSource& q, FlightLog& log, bool critical, uint32_t nowMs);
    void syncCounters();

    INorFlash& flash_;
    JournaledEmergencyStore& store_;
    ILogSource& criticalQ_;
    ILogSource& telemetryQ_;
    StorageServiceConfig cfg_;
    IClock* clock_;
    EmergencyJournal journal_;
    FlightLog critical_;
    FlightLog flight_;
    StorageStatus st_{};
    bool started_ = false;
    bool logsMounted_ = false;
    uint32_t lastMountMs_ = 0u;
    uint32_t attempts_ = 0u;          ///< attempts for the current ticket
    uint32_t attemptTicket_ = 0u;
    uint32_t backoffMs_ = 0u;
    uint32_t retryAtMs_ = 0u;
    bool waiting_ = false;            ///< an emergency record is pending (since pendingSinceMs_)
    uint32_t pendingSinceMs_ = 0u;
    uint32_t lastTelemetryFlushMs_ = 0u;
    uint32_t faultsLogged_ = 0u;
    uint8_t scratch_[kEmergencyRecordBytes] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_STORAGE_SERVICE_HPP
