/**
 ******************************************************************************
 * @file    SafetyEventDispatcher.cpp
 * @brief   See SafetyEventDispatcher.hpp and CRASH_EMERGENCY_DESIGN.md section 11.
 ******************************************************************************
 */
#include "Algorithms/Safety/SafetyEventDispatcher.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
uint32_t elapsed(uint32_t now, uint32_t since) { return timebase::ageMs(since, now); }
constexpr uint32_t kStaleEpisode = 0xFFFFFFFFu;
constexpr AlertKind kLandingKinds[] = {AlertKind::SystemDegraded, AlertKind::LowClearance, AlertKind::DescentWarning,
                                       AlertKind::ClosureWarning, AlertKind::HazardousApproach};
}  // namespace

SafetyEventDispatcher::SafetyEventDispatcher(const DispatcherConfig& cfg, IAudioAlert* audio, IStatusLed* led)
    : cfg_(cfg), audio_(audio), led_(led)
{
}

void SafetyEventDispatcher::set(AlertKind k, bool active, SafetySeverity sev, uint32_t episode, uint32_t nowMs)
{
    Alert& a = alerts_[static_cast<uint8_t>(k)];
    if (!active || sev == SafetySeverity::Info) {
        a = Alert{};   // inactive: the next activation sounds again
        return;
    }
    if (!a.active || a.episode != episode) {
        a.sinceMs = nowMs;
        a.acknowledged = false;
    } else if (sev > a.severity) {
        a.acknowledged = false;   // escalation always sounds
    }
    a.active = true;
    a.severity = sev;
    a.episode = episode;
}

bool SafetyEventDispatcher::trackGap(const SafetyEvent& e)
{
    const uint8_t i = e.source == SafetySource::Emergency ? 1u : 0u;
    if (haveEventId_[i] && !timebase::isAfter(e.eventId, lastEventId_[i])) {
        inc(st_.duplicateEvents);   // replayed / older event: already handled
        return false;
    }
    if (haveEventId_[i] && e.eventId != lastEventId_[i] + 1u) {
        const uint32_t missed = e.eventId - lastEventId_[i] - 1u;
        st_.eventGaps = (UINT32_MAX - st_.eventGaps < missed) ? UINT32_MAX : st_.eventGaps + missed;
    }
    lastEventId_[i] = e.eventId;
    haveEventId_[i] = true;
    return true;
}

void SafetyEventDispatcher::consume(const SafetyEvent& e, uint32_t nowMs)
{
    if (!trackGap(e)) return;
    inc(st_.eventsConsumed);
    const int32_t lat = timebase::diffMs(nowMs, e.timestampMs);
    st_.lastLatencyMs = lat > 0 ? static_cast<uint32_t>(lat) : 0u;
    if (st_.lastLatencyMs > st_.maxLatencyMs) st_.maxLatencyMs = st_.lastLatencyMs;

    const AlertKind k = alertKindForLanding(e.type);
    if (k != AlertKind::None) {
        const uint8_t i = static_cast<uint8_t>(k);
        if (e.transition == SafetyTransition::Raised || e.transition == SafetyTransition::Escalated) {
            ++edgeSerial_[i];
            Alert& a = alerts_[i];
            const SafetySeverity sev = (a.active && a.severity > e.severity) ? a.severity : e.severity;
            set(k, true, sev, a.active ? a.episode : e.episodeId, nowMs);
            a.acknowledged = false;
            a.transient = true;   // announced even if it clears before the next snapshot
            a.transientUntilMs = nowMs + cfg_.transientHoldMs;
        }
        return;
    }
    switch (e.type) {
        case SafetyEventType::CrashSuspected:
            if (e.transition == SafetyTransition::Raised) ++edgeSerial_[static_cast<uint8_t>(AlertKind::CrashSuspected)];
            break;
        case SafetyEventType::EmergencyRequest:
            ++edgeSerial_[static_cast<uint8_t>(AlertKind::SosActive)];
            break;
        case SafetyEventType::SosDeliveryFailed:
        case SafetyEventType::SosDeliveryUnknown:
            ++edgeSerial_[static_cast<uint8_t>(AlertKind::SosCommFailure)];
            break;
        case SafetyEventType::SosPhoneAccepted:
        case SafetyEventType::SosDeliveryConfirmed:
            ++edgeSerial_[static_cast<uint8_t>(AlertKind::SosAccepted)];
            break;
        default:
            break;   // phase notices, impact candidates: no pilot alert
    }
}

void SafetyEventDispatcher::acknowledge(uint32_t /*nowMs*/)
{
    if (st_.audioAlert == AlertKind::None) return;
    Alert& a = alerts_[static_cast<uint8_t>(st_.audioAlert)];
    if (a.active && alertStyleFor(st_.audioAlert, a.severity).acknowledgeable) {
        a.acknowledged = true;
        inc(st_.acknowledgements);
    }
}

void SafetyEventDispatcher::setSystemInputs(const SystemAlertInputs& in, uint32_t nowMs)
{
    if (in.phoneLinkUp) linkEverUp_ = true;
    if (!haveSys_ || in.phoneLinkUp != sys_.phoneLinkUp) linkDownSinceMs_ = nowMs;
    sys_ = in;
    haveSys_ = true;
}

const DispatchStatus& SafetyEventDispatcher::update(uint32_t nowMs, const DispatcherInputs& in)
{
    if (st_.sequence != UINT32_MAX) ++st_.sequence;
    st_.timestampMs = nowMs;

    // ---- landing / system alerts from the safety snapshot ----
    const bool fresh = in.safety != nullptr && elapsed(nowMs, in.safety->timestampMs) <= cfg_.snapshotMaxAgeMs &&
                       timebase::diffMs(in.safety->timestampMs, nowMs) <= 1000;   // not from the future
    if (fresh) {
        lastSafetyChangeMs_ = nowMs;
        safetyEverSeen_ = true;
        const SafetySnapshot& s = *in.safety;
        SafetySeverity sev[kAlertKindCount] = {};
        bool act[kAlertKindCount] = {};
        uint32_t epi[kAlertKindCount] = {};
        for (uint8_t w = 0; w < kLandingWarningCount; ++w) {
            const WarningState& ws = s.warnings[w];
            if (!ws.active) continue;
            static const SafetyEventType kTypes[kLandingWarningCount] = {
                SafetyEventType::ExcessiveDescent, SafetyEventType::LowGroundClearance,
                SafetyEventType::RapidGroundClosure, SafetyEventType::HazardousApproach,
                SafetyEventType::TerrainClearanceUnavailable, SafetyEventType::NavigationDegraded};
            const uint8_t k = static_cast<uint8_t>(alertKindForLanding(kTypes[w]));
            act[k] = true;
            if (ws.severity > sev[k]) sev[k] = ws.severity;
            epi[k] = epi[k] == 0u ? ws.episodeId : (epi[k] ^ (ws.episodeId << 8));
        }
        for (AlertKind kind : kLandingKinds) {
            const uint8_t k = static_cast<uint8_t>(kind);
            Alert& a = alerts_[k];
            if (act[k]) {
                set(kind, true, sev[k], epi[k], nowMs);
                a.transient = false;
            } else if (a.active && a.transient && timebase::diffMs(a.transientUntilMs, nowMs) > 0) {
                // keep: a warning that already cleared is still announced once
                if (!a.transientCounted) {
                    a.transientCounted = true;
                    inc(st_.transientAlerts);
                }
            } else {
                set(kind, false, SafetySeverity::Info, 0u, nowMs);
            }
        }
    } else if (!safetyEverSeen_ || elapsed(nowMs, lastSafetyChangeMs_) > cfg_.staleHoldMs) {
        // Processing silent: landing alerts can no longer be trusted. Say so instead.
        for (AlertKind kind : kLandingKinds) set(kind, false, SafetySeverity::Info, 0u, nowMs);
        if (safetyEverSeen_) set(AlertKind::SystemDegraded, true, SafetySeverity::Advisory, kStaleEpisode, nowMs);
    }   // else: hold the last landing alerts for staleHoldMs
    st_.safetyStale = !fresh;

    // ---- emergency (from its status; never depends on sensor frames or on events) ----
    const EmergencyStatus* em = in.emergency;
    const EmergencyState es = em != nullptr ? em->state : EmergencyState::Unavailable;
    const uint32_t inc0 = em != nullptr ? em->incidentId : 0u;
    set(AlertKind::SosCountdown, es == EmergencyState::Countdown, SafetySeverity::Critical, inc0, nowMs);
    const bool sending = es == EmergencyState::SosPending || es == EmergencyState::AwaitingPhoneAck ||
                         es == EmergencyState::AwaitingDelivery;
    set(AlertKind::SosActive, sending, SafetySeverity::Critical, inc0, nowMs);
    const bool transportDown = sending && (em->flags & emflag::TransportUnavailable) != 0u;
    SafetySeverity commSev = SafetySeverity::Info;
    if (es == EmergencyState::DeliveryFailed) commSev = SafetySeverity::Critical;
    else if (es == EmergencyState::DeliveryUnknown || transportDown) commSev = SafetySeverity::Warning;
    set(AlertKind::SosCommFailure, commSev != SafetySeverity::Info, commSev, inc0, nowMs);
    const bool accepted = es == EmergencyState::Delivered && elapsed(nowMs, em->stateSinceMs) < cfg_.acceptedShowMs;
    set(AlertKind::SosAccepted, accepted, SafetySeverity::Advisory, inc0, nowMs);

    // ---- crash suspicion (only while no incident handles it, e.g. manager not running yet) ----
    const bool incidentOpen = es != EmergencyState::Idle && es != EmergencyState::Unavailable &&
                              es != EmergencyState::Cancelled;
    const bool crashFresh = in.crash != nullptr && in.crash->state == CrashState::CrashSuspected;
    set(AlertKind::CrashSuspected, crashFresh && !incidentOpen, SafetySeverity::Critical,
        crashFresh ? in.crash->episodeId : 0u, nowMs);
    st_.crashStale = in.crash == nullptr;

    // ---- system alerts (Milestone 8): battery, storage / supervisor faults, phone link ----
    if (haveSys_) {
        const SafetySeverity batt = sys_.batteryLevel == 3u   ? SafetySeverity::Warning
                                    : sys_.batteryLevel == 2u ? SafetySeverity::Caution
                                                              : SafetySeverity::Info;
        set(AlertKind::LowBattery, batt != SafetySeverity::Info, batt, sys_.batteryLevel, nowMs);
        set(AlertKind::SystemFault, sys_.storageDegraded || sys_.supervisorFault, SafetySeverity::Advisory,
            (sys_.storageDegraded ? 1u : 0u) | (sys_.supervisorFault ? 2u : 0u), nowMs);
        const bool lost = sys_.phoneLinkKnown && linkEverUp_ && !sys_.phoneLinkUp && sys_.inFlight &&
                          elapsed(nowMs, linkDownSinceMs_) >= cfg_.phoneLossMs && !incidentOpen;
        set(AlertKind::PhoneDisconnected, lost, SafetySeverity::Advisory, linkDownSinceMs_, nowMs);
    }

    // ---- selection: highest priority; ties -> most recent ----
    AlertKind audioK = AlertKind::None, ledK = AlertKind::None;
    uint8_t audioP = 0u, ledP = 0u;
    uint32_t audioSince = 0u, ledSince = 0u;
    st_.activeMask = 0u;
    for (uint8_t k = 1u; k < kAlertKindCount; ++k) {
        const Alert& a = alerts_[k];
        st_.severity[k] = a.active ? a.severity : SafetySeverity::Info;
        if (!a.active) continue;
        st_.activeMask = static_cast<uint16_t>(st_.activeMask | (1u << k));
        const AlertKind kind = static_cast<AlertKind>(k);
        const AlertStyle s = alertStyleFor(kind, a.severity);
        if (s.priority == 0u) continue;
        if (s.priority > ledP || (s.priority == ledP && timebase::isAfter(a.sinceMs, ledSince))) {
            ledK = kind;
            ledP = s.priority;
            ledSince = a.sinceMs;
        }
        const bool silenced = a.acknowledged && s.acknowledgeable;
        if (!silenced && s.audio != AudioPatternId::None &&
            (s.priority > audioP || (s.priority == audioP && timebase::isAfter(a.sinceMs, audioSince)))) {
            audioK = kind;
            audioP = s.priority;
            audioSince = a.sinceMs;
        }
    }

    // ---- audio ----
    AudioPatternId pattern = AudioPatternId::None;
    uint32_t key = 0u;
    if (audioK != AlertKind::None) {
        const uint8_t k = static_cast<uint8_t>(audioK);
        const Alert& a = alerts_[k];
        pattern = alertStyleFor(audioK, a.severity).audio;
        if (audioK == AlertKind::SosCountdown && em != nullptr && em->countdownRemainingMs <= cfg_.countdownFinalMs) {
            pattern = AudioPatternId::CountdownFinal;
        }
        key = (static_cast<uint32_t>(k) << 24) ^ (static_cast<uint32_t>(a.severity) << 20) ^ a.episode ^
              (edgeSerial_[k] << 12);
    }
    if (audioK != lastAudioAlert_ && audioK != AlertKind::None && lastAudioAlert_ != AlertKind::None &&
        audioP > lastAudioPriority_) {
        inc(st_.preemptions);
    }
    lastAudioAlert_ = audioK;
    lastAudioPriority_ = audioP;
    audio_.setDesired(pattern, audioP, key);
    audio_.update(nowMs);

    // ---- LED (normal status pattern when nothing is active) ----
    if (ledK != AlertKind::None) {
        const AlertStyle s = alertStyleFor(ledK, alerts_[static_cast<uint8_t>(ledK)].severity);
        led_.setDesired(s.led, s.color);
    } else {
        led_.setDesired(LedPattern::SlowBlink, LedColor::Green);
    }
    led_.update(nowMs);

    st_.audioAlert = audioK;
    st_.ledAlert = ledK;
    st_.topPriority = ledP;
    st_.audioPattern = audio_.current();
    st_.ledPattern = led_.pattern();
    st_.ledColor = led_.color();
    st_.audioState = audio_.state();
    st_.ledState = led_.state();
    st_.audioRequests = audio_.requests();
    st_.audioFailures = audio_.failures();
    st_.ledChanges = led_.changes();
    return st_;
}

}  // namespace pg
