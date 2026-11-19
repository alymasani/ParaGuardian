/**
 ******************************************************************************
 * @file    test_alert_dispatch.cpp
 * @brief   SafetyEventDispatcher + AudioAlertScheduler + LedPatternScheduler:
 *          alert scenarios 47-59 (CRASH_EMERGENCY_VALIDATION_REPORT.md
 *          section 6). Audio and LED are recording mocks: nothing here says a
 *          speaker or LED works.
 ******************************************************************************
 */
#include <cstdio>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/MockEmergencyIo.hpp"

#include "Algorithms/Safety/SafetyEventDispatcher.hpp"
#include "Algorithms/Safety/SafetyEventEmitter.hpp"
#include "Algorithms/Safety/SafetyEventQueue.hpp"
#include "Processing/TimeBase.hpp"

using namespace pg;

namespace {

SafetySnapshot snap(uint32_t now)
{
    SafetySnapshot s{};
    s.timestampMs = now;
    s.phase = FlightPhase::Flying;
    return s;
}
void warn(SafetySnapshot& s, LandingWarning w, SafetySeverity sev, uint32_t episode)
{
    WarningState& ws = s.warnings[static_cast<uint8_t>(w)];
    ws.active = true;
    ws.severity = sev;
    ws.episodeId = episode;
    s.activeWarningMask = static_cast<uint8_t>(s.activeWarningMask | (1u << static_cast<uint8_t>(w)));
}
EmergencyStatus emergency(EmergencyState st, uint32_t incident = 1u, uint32_t remaining = 50000u)
{
    EmergencyStatus e{};
    e.state = st;
    e.incidentId = incident;
    e.countdownRemainingMs = st == EmergencyState::Countdown ? remaining : 0u;
    return e;
}
SafetyEvent event(uint32_t id, SafetyEventType t, SafetyTransition tr, SafetySeverity sev, uint32_t ms,
                  SafetySource src = SafetySource::LandingSafety)
{
    SafetyEvent e{};
    e.eventId = id;
    e.type = t;
    e.transition = tr;
    e.severity = sev;
    e.timestampMs = ms;
    e.source = src;
    e.episodeId = 1u;
    return e;
}

struct Rig {
    mock::AudioAlert audio;
    mock::StatusLed led;
    SafetyEventDispatcher d{DispatcherConfig{}, &audio, &led};
    EmergencyStatus em = emergency(EmergencyState::Idle);
    const DispatchStatus& step(uint32_t now, const SafetySnapshot* s)
    {
        return d.update(now, DispatcherInputs{s, nullptr, &em});
    }
    /// Runs `ms` of updates every 20 ms with a fixed snapshot (re-stamped each update).
    void runFor(uint32_t& now, uint32_t ms, SafetySnapshot* s)
    {
        for (uint32_t end = now + ms; timebase::diffMs(end, now) > 0; now += 20u) {
            if (s != nullptr) s->timestampMs = now;
            step(now, s);
        }
    }
};

}  // namespace

TEST(alert_47_descent_caution_and_clearance_advisory_patterns)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::LowGroundClearance, SafetySeverity::Advisory, 5u);
    r.runFor(t, 30000u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::LowClearance);
    CHECK_EQ(r.audio.count(AlertTone::Advisory), 1u);   // advisory: one chime, not repeated
    CHECK(r.led.pattern == LedPattern::SlowBlink && r.led.color == LedColor::Amber);

    Rig q;
    t = 1000u;
    SafetySnapshot c = snap(t);
    warn(c, LandingWarning::ExcessiveDescent, SafetySeverity::Caution, 6u);
    q.runFor(t, 30000u, &c);
    CHECK(q.d.status().audioAlert == AlertKind::DescentWarning);
    CHECK(q.d.status().audioPattern == AudioPatternId::CautionDouble);
    std::printf("    [metric] caution 30 s: %zu beeps (2 per 10 s group)\n", q.audio.count(AlertTone::Caution));
    CHECK(q.audio.count(AlertTone::Caution) >= 6u && q.audio.count(AlertTone::Caution) <= 8u);
    CHECK(q.led.pattern == LedPattern::DoubleBlink);
}

TEST(alert_48_critical_terrain_preempts_advisory_immediately)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::LowGroundClearance, SafetySeverity::Advisory, 5u);
    r.runFor(t, 1000u, &s);
    warn(s, LandingWarning::HazardousApproach, SafetySeverity::Critical, 6u);
    s.timestampMs = t;
    const size_t before = r.audio.requests.size();
    r.step(t, &s);   // same update
    CHECK(r.d.status().audioAlert == AlertKind::HazardousApproach);
    CHECK(r.d.status().audioPattern == AudioPatternId::CriticalFast);
    CHECK(r.audio.requests.size() == before + 1u);
    CHECK(r.audio.requests.back().tone == AlertTone::Critical);
    CHECK(r.audio.requests.back().priority == AlertPriority::Critical || r.audio.requests.back().priority == AlertPriority::High);
    CHECK_EQ(r.d.status().preemptions, 1u);
    CHECK(r.led.pattern == LedPattern::FastBlink && r.led.color == LedColor::Red);
    CHECK((r.d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::LowClearance))) != 0u);   // still known
}

TEST(alert_49_sos_countdown_preempts_landing_warning_and_resumes_it)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::HazardousApproach, SafetySeverity::Critical, 6u);
    r.runFor(t, 500u, &s);
    r.em = emergency(EmergencyState::Countdown, 1u, 50000u);
    r.step(t, &s);
    CHECK(r.d.status().audioAlert == AlertKind::SosCountdown);
    CHECK(r.d.status().audioPattern == AudioPatternId::CountdownTick);
    CHECK(r.led.pattern == LedPattern::SosBlink && r.led.color == LedColor::Red);
    r.em.countdownRemainingMs = 8000u;
    t += 20u;
    s.timestampMs = t;
    r.step(t, &s);
    CHECK(r.d.status().audioPattern == AudioPatternId::CountdownFinal);   // faster in the last seconds
    // the countdown alert cannot be acknowledged away
    r.d.acknowledge(t);
    t += 20u;
    s.timestampMs = t;
    r.step(t, &s);
    CHECK(r.d.status().audioAlert == AlertKind::SosCountdown);
    CHECK_EQ(r.d.status().acknowledgements, 0u);
    // cancelled: the landing warning (still active) takes the audio back
    r.em = emergency(EmergencyState::Cancelled);
    t += 20u;
    s.timestampMs = t;
    r.step(t, &s);
    CHECK(r.d.status().audioAlert == AlertKind::HazardousApproach);
}

TEST(alert_50_repeated_and_duplicate_events_are_deduplicated)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::ExcessiveDescent, SafetySeverity::Warning, 7u);
    const SafetyEvent e = event(10u, SafetyEventType::ExcessiveDescent, SafetyTransition::Raised, SafetySeverity::Warning, t);
    r.d.consume(e, t);
    r.runFor(t, 2000u, &s);
    const size_t n = r.audio.requests.size();
    // the same event delivered again (replay) and the same state for 2 s: no restart
    r.d.consume(e, t);
    r.d.consume(e, t);
    r.runFor(t, 1000u, &s);
    CHECK_EQ(r.d.status().duplicateEvents, 2u);
    CHECK_EQ(r.d.status().eventsConsumed, 1u);
    // WarningTriple: 3 beeps / 3 s -> in the extra second at most the rest of one group
    CHECK(r.audio.requests.size() <= n + 3u);
    CHECK(r.audio.cancels.empty());
}

TEST(alert_51_52_clear_and_overlapping_warnings)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::ExcessiveDescent, SafetySeverity::Warning, 7u);
    warn(s, LandingWarning::RapidGroundClosure, SafetySeverity::Caution, 8u);
    r.runFor(t, 1000u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::DescentWarning);   // Warning band outranks Caution
    // descent clears: closure caution takes over (restarted)
    s.warnings[static_cast<uint8_t>(LandingWarning::ExcessiveDescent)] = WarningState{};
    r.runFor(t, 100u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::ClosureWarning);
    CHECK(r.d.status().audioPattern == AudioPatternId::CautionDouble);
    // everything clears: silent, LED back to the normal status pattern
    s = snap(t);
    r.runFor(t, 100u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::None);
    CHECK(r.d.status().activeMask == 0u);
    CHECK(r.led.pattern == LedPattern::SlowBlink && r.led.color == LedColor::Green);
}

TEST(alert_53_54_59_queue_overflow_keeps_critical_and_emergency_state)
{
    mock::NoLock lock;
    SafetyEventQueue<16> q(lock);
    SafetyEventEmitter em;
    em.addSink(&q);
    // a slow consumer: 60 advisory events, then one critical, nothing consumed in between
    for (int i = 0; i < 60; ++i) {
        SafetyEvent e = event(0u, SafetyEventType::LowGroundClearance,
                              (i % 2) ? SafetyTransition::Cleared : SafetyTransition::Raised, SafetySeverity::Advisory,
                              1000u + static_cast<uint32_t>(i));
        em.emit(e);
    }
    em.emit(event(0u, SafetyEventType::HazardousApproach, SafetyTransition::Raised, SafetySeverity::Critical, 2000u));
    CHECK_EQ(q.size(), 16u);
    CHECK_EQ(q.stats().droppedCritical, 0u);
    Rig r;
    // meanwhile an SOS is being sent: its alert comes from the status, not from events
    r.em = emergency(EmergencyState::AwaitingPhoneAck, 3u);
    SafetyEvent e{};
    bool sawCritical = false;
    while (q.pop(e)) {
        sawCritical = sawCritical || e.severity == SafetySeverity::Critical;
        r.d.consume(e, 2100u);
    }
    CHECK(sawCritical);
    CHECK(r.d.status().eventGaps > 0u);   // lost advisories are visible as id gaps
    SafetySnapshot s = snap(2100u);
    warn(s, LandingWarning::HazardousApproach, SafetySeverity::Critical, 9u);
    r.step(2100u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::SosActive);   // SOS outranks every landing warning
    CHECK(r.led.pattern == LedPattern::SosBlink && r.led.color == LedColor::Amber);
    std::printf("    [metric] overflow: %u events consumed, %u ids lost (counted), critical kept\n",
                static_cast<unsigned>(r.d.status().eventsConsumed), static_cast<unsigned>(r.d.status().eventGaps));
}

TEST(alert_55_audio_unavailable_does_not_block_led)
{
    Rig r;
    r.audio.fail = true;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::HazardousApproach, SafetySeverity::Critical, 6u);
    r.runFor(t, 5000u, &s);
    CHECK(r.d.status().audioState == OutputState::Unavailable);
    CHECK(r.d.status().audioFailures >= 4u && r.d.status().audioFailures <= 6u);   // retried ~1/s, not every update
    CHECK(r.led.pattern == LedPattern::FastBlink);
    CHECK(r.d.status().ledState == OutputState::Active);
    // no backend at all (firmware today): unavailable, never "playing"
    SafetyEventDispatcher none(DispatcherConfig{}, nullptr, nullptr);
    none.update(t, DispatcherInputs{&s, nullptr, nullptr});
    CHECK(none.status().audioState == OutputState::Unavailable);
    CHECK(none.status().ledState == OutputState::Unavailable);
    UnconfiguredAudioAlert ua;
    UnconfiguredStatusLed ul;
    SafetyEventDispatcher fw(DispatcherConfig{}, &ua, &ul);
    fw.update(t, DispatcherInputs{&s, nullptr, nullptr});
    CHECK(fw.status().audioState == OutputState::Unavailable);
    CHECK(fw.status().ledState == OutputState::Unavailable);
}

TEST(alert_56_led_unavailable_does_not_block_audio)
{
    Rig r;
    r.led.avail = false;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::HazardousApproach, SafetySeverity::Critical, 6u);
    r.runFor(t, 1000u, &s);
    CHECK(r.d.status().ledState == OutputState::Unavailable);
    CHECK_EQ(r.led.sets, 0u);
    CHECK(r.audio.count(AlertTone::Critical) >= 3u);
}

TEST(alert_57_continues_without_sensor_frames)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::ExcessiveDescent, SafetySeverity::Warning, 7u);
    r.runFor(t, 1000u, &s);
    // processing stops: the snapshot is never refreshed again; an SOS countdown runs
    r.em = emergency(EmergencyState::Countdown, 2u, 40000u);
    const uint32_t stopped = t;
    r.runFor(t, 2000u, nullptr);
    CHECK(r.d.status().audioAlert == AlertKind::SosCountdown);
    CHECK(r.d.status().safetyStale);
    CHECK((r.d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::DescentWarning))) != 0u);   // held
    const size_t ticks0 = r.audio.count(AlertTone::Countdown);
    r.runFor(t, 3000u, nullptr);
    CHECK(r.audio.count(AlertTone::Countdown) >= ticks0 + 2u);   // countdown beeps keep coming
    CHECK((r.d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::DescentWarning))) == 0u);   // released
    CHECK((r.d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::SystemDegraded))) != 0u);   // says so
    std::printf("    [metric] safety data stale %u ms: landing alerts released, SystemDegraded shown\n",
                static_cast<unsigned>(t - stopped));
}

TEST(alert_58_audio_pattern_timing_across_wraparound)
{
    mock::AudioAlert a;
    AudioAlertScheduler sch(&a);
    sch.setDesired(AudioPatternId::CountdownTick, 100u, 1u);
    std::vector<uint32_t> at;
    uint32_t t = 0xFFFFFFFFu - 3500u;
    for (int i = 0; i < 400; ++i, t += 20u) {
        const size_t n = a.requests.size();
        sch.update(t);
        if (a.requests.size() != n) at.push_back(t);
    }
    CHECK(at.size() >= 7u);
    for (size_t i = 1; i < at.size(); ++i) {
        const uint32_t d = at[i] - at[i - 1u];
        CHECK(d >= 1000u && d <= 1020u);   // 1 beep per second, before and after the wrap
    }
    // a scheduler gap of 5 s: the next beep plays once, no burst of missed beeps
    const size_t n0 = a.requests.size();
    sch.update(t + 5000u);
    sch.update(t + 5020u);
    CHECK_EQ(a.requests.size(), n0 + 1u);
}

TEST(alert_acknowledge_silences_until_escalation)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    warn(s, LandingWarning::ExcessiveDescent, SafetySeverity::Warning, 7u);
    r.runFor(t, 500u, &s);
    r.d.acknowledge(t);
    r.runFor(t, 100u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::None);
    CHECK_EQ(r.d.status().acknowledgements, 1u);
    CHECK(r.led.pattern == LedPattern::FastBlink);   // the LED still shows it
    const size_t n = r.audio.requests.size();
    r.runFor(t, 5000u, &s);
    CHECK_EQ(r.audio.requests.size(), n);
    warn(s, LandingWarning::ExcessiveDescent, SafetySeverity::Critical, 7u);   // escalation
    r.runFor(t, 100u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::DescentWarning);
    CHECK(r.audio.requests.size() > n);
}

TEST(alert_transient_warning_is_still_announced)
{
    Rig r;
    uint32_t t = 1000u;
    SafetySnapshot s = snap(t);
    r.runFor(t, 200u, &s);
    // Raised and Cleared between two snapshots: the snapshot never shows it
    r.d.consume(event(1u, SafetyEventType::RapidGroundClosure, SafetyTransition::Raised, SafetySeverity::Critical, t), t);
    r.d.consume(event(2u, SafetyEventType::RapidGroundClosure, SafetyTransition::Cleared, SafetySeverity::Critical, t), t);
    r.runFor(t, 100u, &s);
    CHECK(r.audio.count(AlertTone::Critical) >= 1u);
    CHECK_EQ(r.d.status().transientAlerts, 1u);
    r.runFor(t, 3000u, &s);
    CHECK(r.d.status().audioAlert == AlertKind::None);   // then released
}

TEST(alert_led_reference_timing)
{
    CHECK(LedPatternScheduler::levelAt(LedPattern::SlowBlink, 100u));
    CHECK(!LedPatternScheduler::levelAt(LedPattern::SlowBlink, 600u));
    CHECK(LedPatternScheduler::levelAt(LedPattern::FastBlink, 50u));
    CHECK(!LedPatternScheduler::levelAt(LedPattern::FastBlink, 150u));
    CHECK(LedPatternScheduler::levelAt(LedPattern::DoubleBlink, 250u));
    CHECK(!LedPatternScheduler::levelAt(LedPattern::DoubleBlink, 150u));
    // SOS: dot at 0, dash at 1050..1500, period 3600 ms
    CHECK(LedPatternScheduler::levelAt(LedPattern::SosBlink, 0u));
    CHECK(LedPatternScheduler::levelAt(LedPattern::SosBlink, 1200u));
    CHECK(LedPatternScheduler::levelAt(LedPattern::SosBlink, 3600u));
    CHECK(!LedPatternScheduler::levelAt(LedPattern::SosBlink, 3500u));
    CHECK(!LedPatternScheduler::levelAt(LedPattern::Off, 0u));
}
