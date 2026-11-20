/**
 ******************************************************************************
 * @file    FlightRecorder.cpp
 * @brief   See FlightRecorder.hpp.
 ******************************************************************************
 */
#include "Storage/FlightRecorder.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

LogProfilePeriods logProfilePeriods(LogProfile p)
{
    switch (p) {
        case LogProfile::Minimal: return LogProfilePeriods{0u, 0u, 60000u};
        case LogProfile::Detailed: return LogProfilePeriods{100u, 1000u, 5000u};
        case LogProfile::Emergency: return LogProfilePeriods{200u, 1000u, 5000u};
        case LogProfile::Normal:
        default: return LogProfilePeriods{1000u, 5000u, 10000u};
    }
}

FlightRecorder::FlightRecorder(ILogSink& critical, ILogSink& telemetry, LogProfile profile)
    : critical_(critical), telemetry_(telemetry), profile_(profile)
{
}

void FlightRecorder::push(bool critical, LogType t, uint32_t nowMs, const LogPayload& p)
{
    const bool ok = (critical ? critical_ : telemetry_).push(t, nowMs, p);
    if (!ok) {
        uint32_t& c = critical ? criticalRefused_ : telemetryRefused_;
        if (c != UINT32_MAX) ++c;
    }
}

bool FlightRecorder::due(uint32_t& last, bool& started, uint32_t period, uint32_t nowMs)
{
    if (period == 0u) return false;
    if (started && timebase::ageMs(last, nowMs) < period) return false;
    started = true;
    last = nowMs;
    return true;
}

void FlightRecorder::setProfile(LogProfile p, uint32_t nowMs)
{
    if (p == profile_) return;
    profile_ = p;
    LogPayload q;
    q.put8(static_cast<uint8_t>(p));
    q.put8(0u);   // reason: configuration
    push(false, LogType::Profile, nowMs, q);
}

void FlightRecorder::logBoot(uint32_t resetFlags, uint32_t bootCount, uint8_t journalMount, uint8_t criticalMount,
                             uint8_t flightMount, uint32_t nowMs)
{
    LogPayload p;
    p.put32(resetFlags);
    p.put32(bootCount);
    p.put8(journalMount);
    p.put8(criticalMount);
    p.put8(flightMount);
    push(true, LogType::Boot, nowMs, p);
}

void FlightRecorder::logEvent(const SafetyEvent& e, uint32_t nowMs)
{
    LogPayload p;
    p.put32(e.eventId);
    p.put8(static_cast<uint8_t>(e.type));
    p.put8(static_cast<uint8_t>(e.severity));
    p.put8(static_cast<uint8_t>(e.previousSeverity));
    p.put8(static_cast<uint8_t>(e.transition));
    p.put8(static_cast<uint8_t>(e.source));
    p.put8(static_cast<uint8_t>(e.phase));
    p.put32(e.episodeId);
    p.putScaled(e.verticalVelocity_mps, 1000.0f);
    p.putScaled(e.clearance_m, 100.0f);
    const bool critical = e.source == SafetySource::Emergency || e.source == SafetySource::CrashDetector ||
                          e.severity >= SafetySeverity::Warning || e.type == SafetyEventType::FlightPhaseChanged;
    push(critical, LogType::SafetyEvent, nowMs, p);
}

void FlightRecorder::logEmergency(const EmergencyStatus& s, uint32_t nowMs)
{
    const uint8_t state = static_cast<uint8_t>(s.state);
    const bool active = s.state != EmergencyState::Idle && s.state != EmergencyState::Unavailable;
    emergencyActive_ = active;
    if (haveEmergency_ && state == lastState_ && static_cast<uint8_t>(s.stage) == lastStage_ &&
        static_cast<uint8_t>(s.cancelNotice) == lastNotice_ && s.requestRevision == lastRevision_ &&
        s.incidentId == lastIncident_ && static_cast<uint8_t>(s.persist) == lastPersist_) {
        return;
    }
    haveEmergency_ = true;
    lastState_ = state;
    lastStage_ = static_cast<uint8_t>(s.stage);
    lastNotice_ = static_cast<uint8_t>(s.cancelNotice);
    lastRevision_ = s.requestRevision;
    lastIncident_ = s.incidentId;
    lastPersist_ = static_cast<uint8_t>(s.persist);
    LogPayload p;
    p.put8(state);
    p.put8(static_cast<uint8_t>(s.trigger));
    p.put8(static_cast<uint8_t>(s.stage));
    p.put8(static_cast<uint8_t>(s.cancelNotice));
    p.put32(s.incidentId);
    p.put16(s.requestRevision);
    p.put16(s.reportedStages);
    p.put32(s.flags);
    p.put8(static_cast<uint8_t>(s.persist));
    p.put8(s.attempts);
    push(true, LogType::Emergency, nowMs, p);
}

void FlightRecorder::logPower(const PowerLogSummary& q, uint32_t nowMs)
{
    if (havePower_ && q.batteryState == lastPower_.batteryState && q.flags == lastPower_.flags &&
        q.rail == lastPower_.rail && q.amplifier == lastPower_.amplifier && q.lidarPower == lastPower_.lidarPower &&
        q.sources == lastPower_.sources) {
        return;   // millivolt changes alone are telemetry noise: logged with the next state change
    }
    havePower_ = true;
    lastPower_ = q;
    LogPayload p;
    p.put8(q.batteryState);
    p.put16(q.millivolts);
    p.put8(q.flags);
    p.put8(q.rail);
    p.put8(q.amplifier);
    p.put8(q.lidarPower);
    p.put8(q.sources);
    push(true, LogType::Power, nowMs, p);
}

void FlightRecorder::logNavigation(const NavigationEstimate& n, uint32_t nowMs)
{
    if (!due(navLast_, navStarted_, logProfilePeriods(profile()).navigationMs, nowMs)) return;
    LogPayload p;
    p.put8(static_cast<uint8_t>(n.state));
    p.putScaled(n.altitudeValid ? n.altitude_m : NAN, 100.0f);
    p.putScaled(n.velocityValid ? n.verticalVelocity_mps : NAN, 1000.0f);
    p.putScaled(n.accelerationValid ? n.verticalAccel_mps2 : NAN, 1000.0f);
    p.putScaled(n.altitudeSigma_m, 100.0f);
    p.put32(n.qualityFlags);
    push(false, LogType::Navigation, nowMs, p);
}

void FlightRecorder::logQuality(const SensorQualitySummary& q, uint32_t nowMs)
{
    if (!due(qualityLast_, qualityStarted_, logProfilePeriods(profile()).qualityMs, nowMs)) return;
    LogPayload p;
    p.put32(q.qualityFlags);
    p.put8(q.accelStatus);
    p.put8(q.gyroStatus);
    p.put8(q.baroStatus);
    p.put8(q.accelMask);
    p.put8(q.gyroMask);
    p.put8(q.baroMask);
    p.put16(q.sourcesHealthyMask);
    push(false, LogType::SensorQuality, nowMs, p);
}

void FlightRecorder::logCrash(const CrashStatus& c, uint32_t nowMs)
{
    const uint8_t state = static_cast<uint8_t>(c.state);
    if (haveCrash_ && state == crashState_ && c.impactCandidates == crashCandidates_) return;
    haveCrash_ = true;
    crashState_ = state;
    crashCandidates_ = c.impactCandidates;
    LogPayload p;
    p.put8(state);
    p.put8(static_cast<uint8_t>(c.decision));
    p.put8(static_cast<uint8_t>(c.impact.cls));
    p.put32(c.evidence);
    p.putScaled(c.impactCandidates != 0u ? c.impact.peakAccel_mps2 / 9.80665f : NAN, 100.0f);
    p.putScaled(c.impactCandidates != 0u ? c.impact.peakRate_radps * 57.2957795f : NAN, 1.0f);
    p.put32(c.impactCandidates);
    p.put32(c.suspicions);
    push(c.state == CrashState::CrashSuspected, LogType::CrashSummary, nowMs, p);
}

void FlightRecorder::logHealth(const HealthSummary& h, uint32_t nowMs)
{
    if (!due(healthLast_, healthStarted_, logProfilePeriods(profile()).healthMs, nowMs)) return;
    LogPayload p;
    p.put32(h.freeHeap);
    p.put32(h.minHeap);
    p.put32(h.processingMaxMs);
    p.put32(h.emergencyLateWakes);
    p.put32(h.stackMinWords);
    p.put32(h.faults);
    push(false, LogType::Health, nowMs, p);
}

}  // namespace pg
