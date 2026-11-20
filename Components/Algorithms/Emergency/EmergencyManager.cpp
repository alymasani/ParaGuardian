/**
 ******************************************************************************
 * @file    EmergencyManager.cpp
 * @brief   See EmergencyManager.hpp and Docs/CRASH_EMERGENCY_DESIGN.md
 *          sections 7-10.
 ******************************************************************************
 */
#include "Algorithms/Emergency/EmergencyManager.hpp"

#include <cmath>
#include <limits>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
/// A countdown evaluated later than this after its deadline is flagged CountdownOverrun.
constexpr uint32_t kOverrunToleranceMs = 100u;
/// Barometric altitude / vertical speed older than this are not put into an SOS request.
constexpr uint32_t kNavMaxAgeMs = 5000u;
/// A battery reading older than this is not put into an SOS request.
constexpr uint32_t kBatteryMaxAgeMs = 30000u;

void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
uint32_t elapsed(uint32_t now, uint32_t since) { return timebase::ageMs(since, now); }
/// now has reached / passed t (wrap-safe).
bool reached(uint32_t now, uint32_t t) { return timebase::diffMs(now, t) >= 0; }

int32_t toScaled(float v, float scale, int32_t lo, int32_t hi)
{
    const float x = v * scale;
    if (!std::isfinite(x)) return 0;
    if (x <= static_cast<float>(lo)) return lo;
    if (x >= static_cast<float>(hi)) return hi;
    return static_cast<int32_t>(x >= 0.0f ? x + 0.5f : x - 0.5f);
}
}  // namespace

EmergencyManager::EmergencyManager(const EmergencyConfig& cfg, ISosTransport& transport, IEmergencyStore* store)
    : cfg_(cfg), valid_(cfg.isValid()), transport_(transport), store_(store)
{
    st_.state = valid_ ? EmergencyState::Idle : EmergencyState::Unavailable;
    st_.flags = emflag::Unvalidated | emflag::PersistenceUnavailable | emflag::IncidentIdVolatile;
    impact_.peakAccel_mps2 = impact_.peakRate_radps = kNaN;
}

// ---------------------------------------------------------------------------
void EmergencyManager::restore(uint32_t nowMs)
{
    nowCache_ = nowMs;
    persistent_ = store_ != nullptr && store_->available();
    if (!persistent_) {
        st_.persist = PersistState::Disabled;
        st_.nextIncidentId = nextIncidentId_;
        return;   // flags already say: not persistent, ids volatile
    }
    st_.flags &= ~static_cast<uint32_t>(emflag::PersistenceUnavailable | emflag::IncidentIdVolatile);
    st_.persist = PersistState::Committed;

    uint8_t buf[kEmergencyRecordBytes] = {};
    EmergencyPersistRecord r{};
    const bool have = store_->load(buf, sizeof(buf)) && decodeEmergencyRecord(buf, sizeof(buf), r);

    // ---- incident identity: never hand out an id a previous boot may have used ----
    uint32_t next = 1u;
    auto atLeast = [&next](uint32_t nextId, uint32_t usedId) {
        if (nextId > next) next = nextId;
        if (usedId != 0u && usedId != UINT32_MAX && usedId + 1u > next) next = usedId + 1u;
    };
    if (have) {
        atLeast(r.nextIncidentId, r.incidentId);
        identityFlags_ = r.identityFlags;
        committedNextId_ = next;
    }
    EmergencyPersistRecord u{};
    uint8_t ubuf[kEmergencyRecordBytes] = {};
    if (store_->loadUncommitted(ubuf, sizeof(ubuf)) && decodeEmergencyRecord(ubuf, sizeof(ubuf), u)) {
        atLeast(u.nextIncidentId, u.incidentId);   // torn commit: its ids may have been shown
    }
    if (store_->mountState() == StoreMount::Corrupt && !have) {
        identityFlags_ |= identityflag::Uncertain;   // previous ids unknown: say so in every request
    }
    nextIncidentId_ = next;
    if ((identityFlags_ & identityflag::Uncertain) != 0u) st_.flags |= emflag::IdentityUncertain;
    if ((identityFlags_ & identityflag::Exhausted) != 0u) st_.flags |= emflag::IdentityExhausted;

    if (!have || !valid_ || r.incidentId == 0u || r.state == EmergencyState::Idle ||
        r.state == EmergencyState::Unavailable) {
        persist();   // boot record: commits the identity floor before any incident can use it
        st_.nextIncidentId = nextIncidentId_;
        return;
    }

    // An incident was open at the reset: continue it under the SAME incident id.
    st_.incidentId = r.incidentId;
    st_.trigger = r.trigger;
    st_.requestRevision = r.requestRevision;
    st_.stage = r.stage;
    st_.reportedStages = r.reportedStages;
    st_.requestIssued = r.requestIssued;
    st_.requestLeftDevice = r.requestLeftDevice;
    st_.cancelledAfterSend = r.cancelledAfterSend;
    st_.cancelNotice = r.cancelNotice;
    st_.flags |= emflag::RestoredAfterReset;
    eventMs_ = nowMs;
    holdStartMs_ = nowMs;
    switch (r.state) {
        case EmergencyState::Countdown: {
            // The MCU clock does not run while powered off: the time spent in reset is UNKNOWN.
            // Policy: restart the FULL countdown (the pilot can still cancel a false alarm), but
            // only maxCountdownRestarts times: a reset loop must not postpone help forever.
            st_.countdownRestarts = static_cast<uint8_t>(r.countdownRestarts == UINT8_MAX ? UINT8_MAX
                                                                                          : r.countdownRestarts + 1u);
            if (st_.countdownRestarts > cfg_.maxCountdownRestarts) {
                st_.flags |= emflag::RepeatedResetEscalated;
                createRequest(nowMs);   // persists
                break;
            }
            const uint32_t total = r.trigger == IncidentTrigger::Manual ? cfg_.manualCountdownMs : cfg_.autoCountdownMs;
            st_.countdownTotalMs = total;
            countdownDeadlineMs_ = nowMs + total;
            enter(EmergencyState::Countdown, nowMs);
            emit(SafetyEventType::EmergencyCountdown, SafetyTransition::Raised, SafetySeverity::Critical,
                 SafetySeverity::Info, nowMs);
            break;
        }
        case EmergencyState::SosPending:
        case EmergencyState::AwaitingPhoneAck:
        case EmergencyState::AwaitingDelivery:
        case EmergencyState::DeliveryUnknown:
            // Re-send (idempotent: same incident id, next revision). The phone / backend
            // de-duplicate on (device, incident id); a newer revision only updates content.
            buildRequest(SosRequestKind::Alert, nowMs);
            st_.attempts = 0u;
            st_.cycles = 1u;
            nextAttemptMs_ = nowMs;
            enter(EmergencyState::SosPending, nowMs);
            break;
        case EmergencyState::Cancelled:
            enter(EmergencyState::Cancelled, nowMs);
            if (st_.cancelNotice == CancelNotice::Pending || st_.cancelNotice == CancelNotice::Sent) {
                buildRequest(SosRequestKind::Cancel, nowMs);
                st_.cancelNotice = CancelNotice::Pending;
                st_.attempts = 0u;
                nextAttemptMs_ = nowMs;
            }
            break;
        default:   // Delivered, DeliveryFailed: terminal, kept until a ResetIncident
            if (st_.requestIssued) buildRequest(SosRequestKind::Alert, nowMs);
            enter(r.state, nowMs);
            break;
    }
    persist();
    st_.nextIncidentId = nextIncidentId_;
}

bool EmergencyManager::identityDurable(uint32_t incidentId) const
{
    return persistent_ && incidentId != 0u && committedNextId_ > incidentId;
}

void EmergencyManager::observeBattery(uint16_t millivolts, bool valid, uint32_t nowMs)
{
    batteryValid_ = valid && millivolts != 0u;
    batteryMv_ = batteryValid_ ? millivolts : 0u;
    batteryMs_ = nowMs;
    st_.batteryMillivolts = batteryMv_;
}

// ---------------------------------------------------------------------------
void EmergencyManager::enter(EmergencyState s, uint32_t nowMs)
{
    st_.state = s;
    st_.stateSinceMs = nowMs;
}

void EmergencyManager::emit(SafetyEventType type, SafetyTransition tr, SafetySeverity sev, SafetySeverity prev,
                            uint32_t nowMs)
{
    SafetyEvent e{};
    e.timestampMs = nowMs;
    e.episodeId = st_.incidentId;
    e.type = type;
    e.severity = sev;
    e.previousSeverity = prev;
    e.transition = tr;
    e.source = SafetySource::Emergency;
    e.phase = FlightPhase::Unknown;   // the emergency task does not track the flight phase
    e.qualityFlags = safetyq::Unvalidated;
    e.verticalVelocity_mps = ctx_.navVelocityValid ? ctx_.navVerticalVelocity_mps : kNaN;
    e.clearance_m = kNaN;
    e.closureRate_mps = kNaN;
    e.timeToContact_s = kNaN;
    emitter_.emit(e);
}

void EmergencyManager::persist()
{
    if (!persistent_) return;
    EmergencyPersistRecord r{};
    r.nextIncidentId = nextIncidentId_;
    r.incidentId = st_.incidentId;
    r.state = st_.state;
    r.trigger = st_.trigger;
    r.stage = st_.stage;
    r.requestRevision = st_.requestRevision;
    r.reportedStages = st_.reportedStages;
    r.requestIssued = st_.requestIssued;
    r.requestLeftDevice = st_.requestLeftDevice;
    r.cancelledAfterSend = st_.cancelledAfterSend;
    r.cancelNotice = st_.cancelNotice;
    r.countdownRestarts = st_.countdownRestarts;
    r.identityFlags = identityFlags_;
    uint8_t buf[kEmergencyRecordBytes];
    const size_t n = encodeEmergencyRecord(r, buf, sizeof(buf));
    uint32_t ticket = 0u;
    const PersistAck ack = n != 0u ? store_->save(buf, n, ticket) : PersistAck::Rejected;
    switch (ack) {
        case PersistAck::Committed:
            inc(st_.persistenceWrites);
            committedNextId_ = nextIncidentId_;
            persistOutstanding_ = false;
            pendingTicket_ = ticket;
            st_.persist = PersistState::Committed;
            st_.flags &= ~static_cast<uint32_t>(emflag::PersistencePending | emflag::PersistenceDegraded);
            break;
        case PersistAck::Queued:
            if (!persistOutstanding_) persistQueuedMs_ = nowCache_;
            persistOutstanding_ = true;
            pendingTicket_ = ticket;
            savedNextIdTicket_ = ticket;
            savedNextId_ = nextIncidentId_;
            if (st_.persist != PersistState::Failed) st_.persist = PersistState::Pending;
            st_.flags |= emflag::PersistencePending;
            break;
        case PersistAck::Rejected:
        default:
            inc(st_.persistenceFailures);
            st_.persist = PersistState::Failed;
            st_.flags |= emflag::PersistenceDegraded;
            break;
    }
}

void EmergencyManager::pollPersistence(uint32_t nowMs)
{
    if (!persistent_ || !persistOutstanding_) return;
    const uint32_t committed = store_->committedTicket();
    if (committed != 0u && timebase::diffMs(committed, savedNextIdTicket_) >= 0 && savedNextIdTicket_ != 0u) {
        if (savedNextId_ > committedNextId_) committedNextId_ = savedNextId_;
    }
    if (committed != 0u && timebase::diffMs(committed, pendingTicket_) >= 0) {
        persistOutstanding_ = false;
        inc(st_.persistenceWrites);
        const uint32_t latency = elapsed(nowMs, persistQueuedMs_);
        if (latency > st_.persistLatencyMaxMs) st_.persistLatencyMaxMs = latency;
        st_.persist = PersistState::Committed;
        st_.flags &= ~static_cast<uint32_t>(emflag::PersistencePending | emflag::PersistenceDegraded);
        return;
    }
    const uint32_t failed = store_->failedTicket();
    if (failed != 0u && timebase::diffMs(failed, pendingTicket_) >= 0 && st_.persist != PersistState::Failed) {
        inc(st_.persistenceFailures);
        st_.persist = PersistState::Failed;   // the store keeps retrying; a later commit clears this
        st_.flags |= emflag::PersistenceDegraded;
    }
}

void EmergencyManager::markStage(DeliveryStage stage, uint16_t bit)
{
    st_.reportedStages = static_cast<uint16_t>(st_.reportedStages | bit);
    if (stage > st_.stage) st_.stage = stage;
}

uint32_t EmergencyManager::backoffMs(uint8_t failedAttempts) const
{
    uint32_t d = cfg_.retryBaseMs;
    for (uint8_t i = 1u; i < failedAttempts && d < cfg_.retryMaxMs; ++i) d = d > cfg_.retryMaxMs / 2u ? cfg_.retryMaxMs : d * 2u;
    return d > cfg_.retryMaxMs ? cfg_.retryMaxMs : d;
}

bool EmergencyManager::awaitingAck() const
{
    return st_.state == EmergencyState::AwaitingPhoneAck ||
           (st_.state == EmergencyState::Cancelled && st_.cancelNotice == CancelNotice::Sent);
}

bool EmergencyManager::sendingActive() const
{
    return st_.state == EmergencyState::SosPending || awaitingAck() ||
           (st_.state == EmergencyState::Cancelled && st_.cancelNotice == CancelNotice::Pending);
}

PendingSosRecord EmergencyManager::pendingRecord() const
{
    PendingSosRecord r{};
    r.active = st_.requestIssued && st_.state != EmergencyState::Idle &&
               !(st_.state == EmergencyState::Cancelled && !st_.cancelledAfterSend);
    r.request = req_;
    r.stage = st_.stage;
    r.reportedStages = st_.reportedStages;
    r.state = st_.state;
    return r;
}

// ---------------------------------------------------------------------------
void EmergencyManager::openIncident(IncidentTrigger trigger, uint32_t crashEpisode, uint32_t nowMs)
{
    st_.incidentId = nextIncidentId_;
    if (nextIncidentId_ >= UINT32_MAX - 1u) {
        // 32-bit id space used up (unreachable in practice: ~4e9 incidents). Wrapping would
        // repeat ids the phone / backend already know, so every later request says so.
        identityFlags_ |= identityflag::Exhausted;
        st_.flags |= emflag::IdentityExhausted;
        nextIncidentId_ = 1u;
    } else {
        ++nextIncidentId_;   // 0 is never an incident id
    }
    st_.nextIncidentId = nextIncidentId_;
    st_.countdownRestarts = 0u;
    st_.trigger = trigger;
    st_.crashEpisodeId = crashEpisode;
    st_.requestRevision = 0u;
    st_.stage = DeliveryStage::None;
    st_.reportedStages = 0u;
    st_.failure = FailureReason::None;
    st_.cancelReason = CancelReason::None;
    st_.cancelNotice = CancelNotice::None;
    st_.requestIssued = st_.requestLeftDevice = st_.cancelledAfterSend = false;
    st_.attempts = st_.cycles = 0u;
    st_.additionalImpacts = 0u;
    st_.countdownOverrunMs = 0u;
    st_.flags &= ~static_cast<uint32_t>(emflag::RestoredAfterReset | emflag::ProvenanceIncomplete |
                                        emflag::ManuallyConfirmed | emflag::CountdownOverrun |
                                        emflag::IdentityUnconfirmed | emflag::RepeatedResetEscalated);
    req_ = SosRequest{};
    eventMs_ = nowMs;
    if (trigger == IncidentTrigger::Manual) {
        impact_ = ImpactSummary{};
        impact_.peakAccel_mps2 = impact_.peakRate_radps = kNaN;
        crashEvidence_ = 0u;
    } else if (impact_.timestampMs != 0u && timebase::diffMs(nowMs, impact_.timestampMs) >= 0) {
        eventMs_ = impact_.timestampMs;
    }
    inc(st_.incidents);
    if (trigger == IncidentTrigger::AutomaticCrashSuspected) inc(st_.automaticIncidents);

    const uint32_t total = trigger == IncidentTrigger::Manual ? cfg_.manualCountdownMs : cfg_.autoCountdownMs;
    st_.countdownTotalMs = total;
    if (total == 0u) {
        createRequest(nowMs);
        return;
    }
    countdownDeadlineMs_ = nowMs + total;   // wraps correctly; compared with diffMs
    enter(EmergencyState::Countdown, nowMs);
    emit(SafetyEventType::EmergencyCountdown, SafetyTransition::Raised, SafetySeverity::Critical, SafetySeverity::Info,
         nowMs);
    persist();
}

void EmergencyManager::expireCountdown(uint32_t nowMs, bool lateIsOverrun)
{
    const uint32_t late = elapsed(nowMs, countdownDeadlineMs_);
    st_.countdownOverrunMs = late;
    if (lateIsOverrun && late > kOverrunToleranceMs) st_.flags |= emflag::CountdownOverrun;
    emit(SafetyEventType::EmergencyCountdown, SafetyTransition::Cleared, SafetySeverity::Critical,
         SafetySeverity::Critical, nowMs);
    createRequest(nowMs);
}

void EmergencyManager::buildRequest(SosRequestKind kind, uint32_t nowMs)
{
    SosRequest& r = req_;
    r = SosRequest{};
    r.version = kSosProtocolVersion;
    r.kind = kind;
    r.trigger = st_.trigger;
    r.state = st_.state;
    r.incidentId = st_.incidentId;
    st_.requestRevision = st_.requestRevision == UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(st_.requestRevision + 1u);
    r.revision = st_.requestRevision;
    r.createdUptimeMs = nowMs;
    r.eventUptimeMs = eventMs_;
    uint16_t v = 0u;
    if (ctx_.gnssFixValid) {   // the LAST VALID fix, with its age; never invented
        v |= sosvalid::Position;
        r.latitude_1e7deg = ctx_.latitude_1e7deg;
        r.longitude_1e7deg = ctx_.longitude_1e7deg;
        r.horizontalAccuracy_dm = static_cast<uint16_t>(toScaled(ctx_.horizontalAccuracy_m, 10.0f, 0, 65535));
        r.fixType = ctx_.fixType;
        r.numSatellites = ctx_.numSatellites;
        r.fixAgeMs = elapsed(nowMs, ctx_.fixTimestampMs);
        if (r.fixAgeMs > kSosStaleFixMs) v |= sosvalid::PositionStale;
        if (std::isfinite(ctx_.heightMsl_m)) {
            v |= sosvalid::HeightMsl;
            r.heightMsl_dm = toScaled(ctx_.heightMsl_m, 10.0f, INT32_MIN + 1, INT32_MAX);
        }
    } else {
        r.fixAgeMs = UINT32_MAX;
    }
    // Navigation values are sent only while recent (no age field for them in v1).
    const bool navRecent = elapsed(nowMs, ctx_.navTimestampMs) <= kNavMaxAgeMs;
    if (ctx_.navAltitudeValid && navRecent) {
        v |= sosvalid::RelativeAltitude;   // barometric, relative: a separate field from MSL
        r.relativeAltitude_dm = toScaled(ctx_.navAltitude_m, 10.0f, INT32_MIN + 1, INT32_MAX);
    }
    if (ctx_.navVelocityValid && navRecent) {
        v |= sosvalid::VerticalSpeed;
        r.verticalSpeed_cmps = static_cast<int16_t>(toScaled(ctx_.navVerticalVelocity_mps, 100.0f, -32767, 32767));
    }
    if (st_.trigger != IncidentTrigger::Manual && std::isfinite(impact_.peakAccel_mps2)) {
        v |= sosvalid::Impact;
        r.peakAccel_cg = static_cast<uint16_t>(toScaled(impact_.peakAccel_mps2 / 9.80665f, 100.0f, 0, 65535));
        r.peakRate_dps = static_cast<uint16_t>(toScaled(impact_.peakRate_radps, 57.2957795f, 0, 65535));
        r.impactClass = static_cast<uint8_t>(impact_.cls);
        r.impacts = impact_.impacts;
        r.crashEvidence = crashEvidence_;
        if (impact_.saturated) v |= sosvalid::ImpactSaturated;
    }
    if (!persistent_ || identityFlags_ != 0u || (st_.flags & emflag::IdentityUnconfirmed) != 0u) {
        v |= sosvalid::IncidentIdVolatile;
    }
    if ((st_.flags & emflag::RestoredAfterReset) != 0u) v |= sosvalid::Restored;
    if (batteryValid_ && elapsed(nowMs, batteryMs_) <= kBatteryMaxAgeMs) {
        v |= sosvalid::Battery;
        r.batteryMillivolts = batteryMv_;
    } else {
        r.batteryMillivolts = 0u;   // unknown: Battery bit clear (never invented)
    }
    r.deviceHealthFlags = ctx_.frameQualityFlags;
    r.validity = v;
}

void EmergencyManager::createRequest(uint32_t nowMs)
{
    st_.requestIssued = true;
    holdStartMs_ = nowMs;
    enter(EmergencyState::SosPending, nowMs);
    buildRequest(SosRequestKind::Alert, nowMs);
    markStage(DeliveryStage::Created, stagebit::Created);
    st_.attempts = 0u;
    st_.cycles = 1u;
    nextAttemptMs_ = nowMs;
    inc(st_.sosRequestsCreated);
    emit(SafetyEventType::EmergencyRequest, SafetyTransition::Notice, SafetySeverity::Critical, SafetySeverity::Info,
         nowMs);
    persist();
}

void EmergencyManager::cancel(CancelReason reason, uint32_t nowMs)
{
    const bool wasCountdown = st_.state == EmergencyState::Countdown;
    st_.cancelReason = reason;
    inc(st_.cancellations);
    setTransportUnavailable(false, nowMs);
    if (wasCountdown) {
        emit(SafetyEventType::EmergencyCountdown, SafetyTransition::Cleared, SafetySeverity::Critical,
             SafetySeverity::Critical, nowMs);
    }
    if (st_.requestLeftDevice) {
        // The request may already have reached the phone, and the phone may already have
        // alerted someone. Cancellation cannot withdraw that: a cancellation NOTICE is sent
        // with the same retry machinery, and the status says the alert may be out.
        st_.cancelledAfterSend = true;
        inc(st_.cancellationsAfterSend);
        st_.cancelNotice = CancelNotice::Pending;
        enter(EmergencyState::Cancelled, nowMs);
        buildRequest(SosRequestKind::Cancel, nowMs);
        st_.attempts = 0u;
        nextAttemptMs_ = nowMs;
    } else {
        st_.cancelNotice = CancelNotice::None;   // nothing left the device: fully withdrawn
        enter(EmergencyState::Cancelled, nowMs);
    }
    emit(SafetyEventType::EmergencyCancelled, SafetyTransition::Notice, SafetySeverity::Advisory, SafetySeverity::Info,
         nowMs);
    persist();
}

void EmergencyManager::resumeSending(uint32_t nowMs)
{
    // Same incident, fresh content (newer position), next revision, new attempt cycle.
    buildRequest(SosRequestKind::Alert, nowMs);
    st_.attempts = 0u;
    if (st_.cycles != UINT8_MAX) ++st_.cycles;
    st_.failure = FailureReason::None;
    nextAttemptMs_ = nowMs;
    enter(EmergencyState::SosPending, nowMs);
    persist();
}

void EmergencyManager::setTransportUnavailable(bool unavailable, uint32_t nowMs)
{
    if (unavailable == transportUnavailableRaised_) return;
    transportUnavailableRaised_ = unavailable;
    if (unavailable) {
        st_.flags |= emflag::TransportUnavailable;
        emit(SafetyEventType::SosTransportUnavailable, SafetyTransition::Raised, SafetySeverity::Warning,
             SafetySeverity::Info, nowMs);
    } else {
        st_.flags &= ~static_cast<uint32_t>(emflag::TransportUnavailable);
        emit(SafetyEventType::SosTransportUnavailable, SafetyTransition::Cleared, SafetySeverity::Warning,
             SafetySeverity::Warning, nowMs);
    }
}

// ---------------------------------------------------------------------------
void EmergencyManager::observeCrash(const CrashReport& report, uint32_t nowMs)
{
    nowCache_ = nowMs;
    if (!valid_) return;
    const CrashStatus& c = report.status;
    ctx_ = report.context;
    lastReportMs_ = report.publishedMs;
    if (!haveCrash_) {
        haveCrash_ = true;
        // A suspicion already active when the manager starts is acted on; older ones are not.
        lastSerial_ = (c.state == CrashState::CrashSuspected && c.suspicionSerial != 0u) ? c.suspicionSerial - 1u
                                                                                        : c.suspicionSerial;
        lastCleared_ = c.suspicionsCleared;
    }

    // Supervision: a strong in-flight impact still under observation.
    const bool observing = c.state == CrashState::ImpactCandidate || c.state == CrashState::PostImpactObservation;
    strongPending_ = observing && c.impact.cls == ImpactClass::Strong && (c.evidence & crashev::AirborneAtImpact) != 0u;
    if (strongPending_) strongEpisode_ = c.episodeId;

    if (c.suspicionSerial != lastSerial_ && !timebase::isAfter(c.suspicionSerial, lastSerial_)) {
        // The counter went BACKWARDS: the detector was re-created (e.g. processing restarted).
        // Re-synchronise; an old count is not a new suspicion.
        lastSerial_ = c.suspicionSerial;
        lastCleared_ = c.suspicionsCleared;
    }
    if (c.suspicionSerial != lastSerial_) {
        lastSerial_ = c.suspicionSerial;
        impact_ = c.impact;
        crashEvidence_ = c.evidence;
        if (st_.state == EmergencyState::Idle || st_.state == EmergencyState::Cancelled) {
            openIncident(IncidentTrigger::AutomaticCrashSuspected, c.episodeId, nowMs);
        } else if (st_.state != EmergencyState::Unavailable && st_.additionalImpacts != UINT8_MAX) {
            ++st_.additionalImpacts;   // same incident: repeated observations never open a second one
        }
    }
    if (c.suspicionsCleared != lastCleared_) {
        lastCleared_ = c.suspicionsCleared;
        if (cfg_.autoCancelOnSuspicionCleared && st_.state == EmergencyState::Countdown &&
            st_.trigger == IncidentTrigger::AutomaticCrashSuspected && st_.crashEpisodeId == c.episodeId &&
            (st_.flags & emflag::ManuallyConfirmed) == 0u) {
            cancel(CancelReason::SuspicionCleared, nowMs);
        }
    }
}

CommandResult EmergencyManager::handleCommand(const EmergencyCommand& c, uint32_t nowMs)
{
    nowCache_ = nowMs;
    if (!isWellFormed(c)) {
        inc(st_.commandsRejected);
        return CommandResult::Malformed;
    }
    if (!valid_) return CommandResult::NotAllowed;
    const uint8_t src = static_cast<uint8_t>(c.source);
    if (seqSeen_[src] && lastSeq_[src] == c.sequence) {
        inc(st_.commandsDuplicate);
        return CommandResult::Duplicate;
    }
    seqSeen_[src] = true;
    lastSeq_[src] = c.sequence;

    const bool open = st_.state != EmergencyState::Idle;
    switch (c.type) {
        case EmergencyCommandType::ManualSos:
            inc(st_.manualSosCommands);
            if (c.incidentId != 0u && (!open || c.incidentId != st_.incidentId)) {
                inc(st_.commandsWrongIncident);
                return CommandResult::WrongIncident;
            }
            switch (st_.state) {
                case EmergencyState::Idle:
                case EmergencyState::Cancelled:   // a finished incident: this is a new one
                    eventMs_ = c.issuedMs;
                    openIncident(IncidentTrigger::Manual, 0u, nowMs);
                    return CommandResult::Accepted;
                case EmergencyState::Countdown:   // the pilot confirms: escalate now
                    st_.flags |= emflag::ManuallyConfirmed;
                    expireCountdown(nowMs, false);
                    return CommandResult::Accepted;
                case EmergencyState::Delivered:
                case EmergencyState::DeliveryUnknown:
                case EmergencyState::DeliveryFailed:   // send again: same incident, fresh position
                    resumeSending(nowMs);
                    return CommandResult::Accepted;
                default:
                    return CommandResult::NoEffect;   // already being sent (idempotent)
            }

        case EmergencyCommandType::CancelSos:
            if (!open || c.incidentId != st_.incidentId) {
                inc(st_.commandsWrongIncident);   // never cancels a different incident
                return CommandResult::WrongIncident;
            }
            if (st_.state == EmergencyState::Cancelled) return CommandResult::NoEffect;
            if (st_.state == EmergencyState::Countdown && !reached(c.issuedMs, countdownDeadlineMs_ + 1u)) {
                cancel(CancelReason::Command, nowMs);   // issued at or before the deadline: in time
                return CommandResult::Accepted;
            }
            if (st_.state == EmergencyState::Countdown) expireCountdown(nowMs, true);   // too late: request exists
            cancel(CancelReason::Command, nowMs);
            return CommandResult::Accepted;

        case EmergencyCommandType::AcknowledgeAlert:
            return CommandResult::ForwardToAlerts;

        case EmergencyCommandType::RequestStatus:
            return CommandResult::Accepted;

        case EmergencyCommandType::ResetIncident:
            if (!open || c.incidentId != st_.incidentId) {
                inc(st_.commandsWrongIncident);
                return CommandResult::WrongIncident;
            }
            if (st_.state == EmergencyState::Delivered || st_.state == EmergencyState::DeliveryUnknown ||
                st_.state == EmergencyState::DeliveryFailed || st_.state == EmergencyState::Cancelled) {
                setTransportUnavailable(false, nowMs);
                enter(EmergencyState::Idle, nowMs);
                persist();
                return CommandResult::Accepted;
            }
            inc(st_.commandsNotAllowed);   // countdown / sending: use CancelSos
            return CommandResult::NotAllowed;

        default:
            inc(st_.commandsRejected);
            return CommandResult::Malformed;
    }
}

void EmergencyManager::handleReport(const TransportReport& r, uint32_t nowMs)
{
    nowCache_ = nowMs;
    if (!valid_) return;
    if (r.kind == TransportReportKind::None || r.kind >= TransportReportKind::Count) {
        inc(st_.invalidReports);
        return;
    }
    if (r.kind == TransportReportKind::LinkUp) {
        st_.transportLinkUp = true;
        if (st_.state == EmergencyState::SosPending ||
            (st_.state == EmergencyState::Cancelled && st_.cancelNotice == CancelNotice::Pending)) {
            nextAttemptMs_ = nowMs;   // do not wait for the next link poll
        }
        const bool unresolved = st_.state == EmergencyState::AwaitingDelivery ||
            st_.state == EmergencyState::DeliveryUnknown ||
            (st_.state == EmergencyState::DeliveryFailed && st_.failure == FailureReason::RetriesExhausted);
        if (unresolved && st_.cycles < cfg_.maxCycles) {
            resumeSending(nowMs);   // retrieve the phone's durable outcome under a new revision
        }
        return;
    }
    if (r.kind == TransportReportKind::LinkDown) {
        st_.transportLinkUp = false;
        if (awaitingAck()) {   // the attempt in flight is lost; resend when the link returns
            attemptFailed(nowMs);
        }
        return;
    }

    // Everything else refers to ONE incident and ONE request revision.
    if (st_.incidentId == 0u || r.incidentId != st_.incidentId || st_.state == EmergencyState::Idle) {
        inc(st_.wrongIncidentReports);
        return;
    }
    if (!st_.requestIssued || r.revision == 0u || r.revision > st_.requestRevision) {
        inc(st_.invalidReports);   // no request yet, or a revision that was never sent
        return;
    }

    const bool cancelled = st_.state == EmergencyState::Cancelled;
    switch (r.kind) {
        case TransportReportKind::WriteCompleted:
            if ((st_.reportedStages & stagebit::TransferredToPhone) != 0u && !awaitingAck()) {
                inc(st_.duplicateAcks);
                return;
            }
            markStage(DeliveryStage::TransferredToPhone, stagebit::TransferredToPhone);
            break;

        case TransportReportKind::WriteFailed:
        case TransportReportKind::PhoneRejected:
            if (!awaitingAck()) {
                inc(st_.duplicateAcks);   // not for the attempt in flight
                return;
            }
            attemptFailed(nowMs);
            return;

        case TransportReportKind::PhoneAccepted: {
            const bool newAttempt = st_.state == EmergencyState::SosPending ||
                                    st_.state == EmergencyState::AwaitingPhoneAck ||
                                    (st_.state == EmergencyState::DeliveryFailed &&
                                     st_.failure == FailureReason::RetriesExhausted);
            if ((st_.reportedStages & stagebit::AcceptedByPhone) != 0u && !newAttempt) {
                inc(st_.duplicateAcks);
                return;
            }
            markStage(DeliveryStage::AcceptedByPhone, stagebit::AcceptedByPhone);
            if (!cancelled && newAttempt) {
                acceptedMs_ = nowMs;
                st_.failure = FailureReason::None;
                setTransportUnavailable(false, nowMs);
                enter(EmergencyState::AwaitingDelivery, nowMs);
                emit(SafetyEventType::SosPhoneAccepted, SafetyTransition::Notice, SafetySeverity::Advisory,
                     SafetySeverity::Info, nowMs);
            }
            break;
        }

        case TransportReportKind::PhoneSendAttempted:
            if ((st_.reportedStages & stagebit::PhoneSendAttempted) != 0u) {
                inc(st_.duplicateAcks);
                return;
            }
            markStage(DeliveryStage::PhoneSendAttempted, stagebit::PhoneSendAttempted);
            if ((st_.reportedStages & stagebit::AcceptedByPhone) == 0u) st_.flags |= emflag::ProvenanceIncomplete;
            if (!cancelled && (st_.state == EmergencyState::SosPending || st_.state == EmergencyState::AwaitingPhoneAck)) {
                acceptedMs_ = nowMs;   // implies acceptance, but the bit stays unset (provenance)
                enter(EmergencyState::AwaitingDelivery, nowMs);
            }
            break;

        case TransportReportKind::DeliveryConfirmed: {
            const bool known = (st_.reportedStages & stagebit::DeliveryConfirmed) != 0u;
            if (known && (st_.state == EmergencyState::Delivered || cancelled)) {
                inc(st_.duplicateAcks);
                return;
            }
            markStage(DeliveryStage::DeliveryConfirmed, stagebit::DeliveryConfirmed);
            if ((st_.reportedStages & stagebit::AcceptedByPhone) == 0u) st_.flags |= emflag::ProvenanceIncomplete;
            if (!cancelled && st_.state != EmergencyState::Delivered) {
                if (!known) inc(st_.deliveriesConfirmed);
                setTransportUnavailable(false, nowMs);
                enter(EmergencyState::Delivered, nowMs);
                emit(SafetyEventType::SosDeliveryConfirmed, SafetyTransition::Notice, SafetySeverity::Advisory,
                     SafetySeverity::Info, nowMs);
            }
            break;
        }

        case TransportReportKind::DeliveryFailed: {
            const bool known = (st_.reportedStages & stagebit::DeliveryFailed) != 0u;
            if (known && (st_.state == EmergencyState::DeliveryFailed || cancelled)) {
                inc(st_.duplicateAcks);
                return;
            }
            if (st_.state == EmergencyState::Delivered) {
                inc(st_.invalidReports);   // contradicts a reported confirmation: kept, counted
                return;
            }
            st_.reportedStages = static_cast<uint16_t>(st_.reportedStages | stagebit::DeliveryFailed);
            if (!cancelled) {
                st_.failure = FailureReason::PhoneReportedFailure;
                if (!known) inc(st_.deliveriesFailed);
                enter(EmergencyState::DeliveryFailed, nowMs);
                emit(SafetyEventType::SosDeliveryFailed, SafetyTransition::Notice, SafetySeverity::Critical,
                     SafetySeverity::Info, nowMs);
            }
            break;
        }

        case TransportReportKind::DeliveryUnknown:
            if (st_.state != EmergencyState::AwaitingDelivery) return;
            inc(st_.deliveriesUnknown);
            enter(EmergencyState::DeliveryUnknown, nowMs);
            emit(SafetyEventType::SosDeliveryUnknown, SafetyTransition::Notice, SafetySeverity::Warning,
                 SafetySeverity::Info, nowMs);
            break;

        case TransportReportKind::CancelAccepted:
            if (!cancelled || (st_.reportedStages & stagebit::CancelAccepted) != 0u) {
                inc(st_.duplicateAcks);
                return;
            }
            st_.reportedStages = static_cast<uint16_t>(st_.reportedStages | stagebit::CancelAccepted);
            st_.cancelNotice = CancelNotice::Accepted;
            break;

        default:
            inc(st_.invalidReports);
            return;
    }
    persist();
}

void EmergencyManager::attemptFailed(uint32_t nowMs)
{
    inc(st_.transportFailures);
    const bool notice = st_.state == EmergencyState::Cancelled;
    if (st_.attempts >= cfg_.maxAttempts) {
        if (notice) {
            st_.cancelNotice = CancelNotice::Failed;
        } else {
            st_.failure = FailureReason::RetriesExhausted;
            inc(st_.deliveriesFailed);
            enter(EmergencyState::DeliveryFailed, nowMs);
            emit(SafetyEventType::SosDeliveryFailed, SafetyTransition::Notice, SafetySeverity::Critical,
                 SafetySeverity::Info, nowMs);
        }
        persist();
        return;
    }
    nextAttemptMs_ = nowMs + backoffMs(st_.attempts);
    if (notice) st_.cancelNotice = CancelNotice::Pending;
    else enter(EmergencyState::SosPending, nowMs);
}

void EmergencyManager::serviceTransport(uint32_t nowMs)
{
    if (!sendingActive()) return;
    if (awaitingAck()) {
        if (reached(nowMs, ackDeadlineMs_)) attemptFailed(nowMs);   // no acceptance in time
        return;
    }
    if (!reached(nowMs, nextAttemptMs_)) return;
    if (!transport_.linkUp()) {
        // Waiting for a link is NOT an attempt: the request stays pending indefinitely.
        setTransportUnavailable(true, nowMs);
        nextAttemptMs_ = nowMs + cfg_.linkPollMs;
        return;
    }
    // (Exhausted id space: ids repeat by definition and every request says so; waiting is pointless.)
    if (!st_.requestLeftDevice && persistent_ && (identityFlags_ & identityflag::Exhausted) == 0u &&
        !identityDurable(st_.incidentId)) {
        // First transmission of this incident: wait (bounded) until its id is durable, so a
        // reset cannot give the same id to a different incident. Never withhold help: after
        // identityHoldMs the request goes out marked IncidentIdVolatile.
        if (elapsed(nowMs, holdStartMs_) < cfg_.identityHoldMs && st_.persist != PersistState::Failed) return;
        st_.flags |= emflag::IdentityUnconfirmed;
        req_.validity = static_cast<uint16_t>(req_.validity | sosvalid::IncidentIdVolatile);
    }
    req_.state = st_.state;
    switch (transport_.submit(req_, nowMs)) {
        case TransportSubmitResult::Accepted:
            if (st_.attempts != UINT8_MAX) ++st_.attempts;
            inc(st_.transportAttempts);
            st_.requestLeftDevice = true;
            setTransportUnavailable(false, nowMs);
            if (st_.state == EmergencyState::Cancelled) {
                st_.cancelNotice = CancelNotice::Sent;
            } else {
                markStage(DeliveryStage::QueuedForTransport, stagebit::QueuedForTransport);
                enter(EmergencyState::AwaitingPhoneAck, nowMs);
            }
            ackDeadlineMs_ = nowMs + cfg_.ackTimeoutMs;
            persist();
            break;
        case TransportSubmitResult::Busy:
            nextAttemptMs_ = nowMs + cfg_.busyRetryMs;
            break;
        case TransportSubmitResult::LinkDown:
        case TransportSubmitResult::Unavailable:
            setTransportUnavailable(true, nowMs);
            nextAttemptMs_ = nowMs + cfg_.linkPollMs;
            break;
        case TransportSubmitResult::Invalid:
        default:
            if (st_.attempts != UINT8_MAX) ++st_.attempts;   // counts: a rejected request must not loop forever
            attemptFailed(nowMs);
            break;
    }
}

const EmergencyStatus& EmergencyManager::tick(uint32_t nowMs)
{
    if (st_.sequence != UINT32_MAX) ++st_.sequence;
    st_.timestampMs = nowMs;
    nowCache_ = nowMs;
    pollPersistence(nowMs);
    if (!valid_) {
        st_.state = EmergencyState::Unavailable;
        return st_;
    }

    // Explicit policy: strong in-flight impact under observation, then the detector went silent.
    if (cfg_.escalateOnDetectorSilence && strongPending_ && haveCrash_ && st_.state == EmergencyState::Idle &&
        strongEpisode_ != silenceHandledEpisode_ && elapsed(nowMs, lastReportMs_) >= cfg_.detectorSilenceMs) {
        silenceHandledEpisode_ = strongEpisode_;
        inc(st_.detectorSilenceIncidents);
        openIncident(IncidentTrigger::DetectorSilence, strongEpisode_, nowMs);
    }

    switch (st_.state) {
        case EmergencyState::Countdown:
            // Expires strictly AFTER the deadline: a cancel issued at the deadline is still in time.
            if (reached(nowMs, countdownDeadlineMs_ + 1u)) expireCountdown(nowMs, true);
            break;
        case EmergencyState::AwaitingDelivery:
            if (elapsed(nowMs, acceptedMs_) >= cfg_.deliveryTimeoutMs) {
                inc(st_.deliveriesUnknown);   // honest: accepted, outcome not reported
                enter(EmergencyState::DeliveryUnknown, nowMs);
                emit(SafetyEventType::SosDeliveryUnknown, SafetyTransition::Notice, SafetySeverity::Warning,
                     SafetySeverity::Info, nowMs);
                persist();
            }
            break;
        case EmergencyState::Cancelled:
            if ((st_.cancelNotice == CancelNotice::None || st_.cancelNotice == CancelNotice::Accepted ||
                 st_.cancelNotice == CancelNotice::Failed) &&
                elapsed(nowMs, st_.stateSinceMs) >= cfg_.cancelRearmMs) {
                enter(EmergencyState::Idle, nowMs);   // re-armed for a new incident
                persist();
            }
            break;
        default:
            break;
    }
    serviceTransport(nowMs);

    st_.countdownRemainingMs = 0u;
    if (st_.state == EmergencyState::Countdown) {
        const int32_t left = timebase::diffMs(countdownDeadlineMs_, nowMs);
        st_.countdownRemainingMs = left > 0 ? static_cast<uint32_t>(left) : 0u;
    }
    st_.nextAttemptInMs = 0u;
    if (sendingActive() && !awaitingAck()) {
        const int32_t left = timebase::diffMs(nextAttemptMs_, nowMs);
        st_.nextAttemptInMs = left > 0 ? static_cast<uint32_t>(left) : 0u;
    }
    st_.nextIncidentId = nextIncidentId_;
    return st_;
}

}  // namespace pg
