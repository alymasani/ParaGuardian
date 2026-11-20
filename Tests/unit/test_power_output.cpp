/**
 ******************************************************************************
 * @file    test_power_output.cpp
 * @brief   Milestone 8: PowerManager (battery / charging / rails), PCM tone
 *          output with amplifier gating, GPIO LED backend, system alerts in
 *          the dispatcher, LSM6DSRX FIFO decoding / drain, TaskSupervisor.
 ******************************************************************************
 */
#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/MockEmergencyIo.hpp"
#include "Algorithms/Safety/SafetyEventDispatcher.hpp"
#include "Common/TaskSupervisor.hpp"
#include "Output/Audio/PcmToneOutput.hpp"
#include "Output/LED/GpioStatusLed.hpp"
#include "Power/PowerManager.hpp"
#include "Sensors/LSM6DSRX/Lsm6dsrxFifo.hpp"

using namespace pg;

namespace {
struct Switch final : IPowerSwitch {
    bool isConfigured = true;
    bool on = false;
    bool fail = false;
    Signal good = Signal::Unknown;
    std::vector<bool> history;
    Status set(bool v) override
    {
        if (fail) return Status::BusError;
        on = v;
        history.push_back(v);
        return Status::Ok;
    }
    bool configured() const override { return isConfigured; }
    Signal powerGood() const override { return good; }
};

BatteryConfig battery()
{
    BatteryConfig b{};
    b.lowMv = 3500u;       // test thresholds only: real values need the chosen battery's data
    b.criticalMv = 3300u;
    b.hysteresisMv = 50u;
    b.confirmSamples = 3u;
    return b;
}

PowerInputs mv(uint16_t v, uint32_t t, Signal chg = Signal::Inactive, Signal done = Signal::Inactive)
{
    PowerInputs in{};
    in.sampleMs = t;
    in.batteryMillivolts = v;
    in.batteryValid = v != 0u;
    in.charging = chg;
    in.chargeComplete = done;
    return in;
}
}  // namespace

TEST(power_unconfigured_thresholds_and_missing_data_are_never_healthy)
{
    PowerManager pm(BatteryConfig{}, PowerPolicyConfig{}, nullptr, nullptr);
    pm.start(0u);
    const PowerSnapshot& s = pm.update(mv(4100u, 0u), PowerContext{}, 0u);
    CHECK(s.level == BatteryLevel::Unknown);
    CHECK((s.flags & powerflag::ThresholdsUnconfigured) != 0u);
    CHECK(s.rail5v == RailState::Unconfigured);
    CHECK_EQ(s.stateOfChargePercent, 255u);   // voltage is not a state of charge

    PowerManager p2(battery(), PowerPolicyConfig{}, nullptr, nullptr);
    p2.start(0u);
    for (uint32_t t = 0; t < 10000u; t += 1000u) p2.update(mv(0u, t), PowerContext{}, t);
    CHECK(p2.snapshot().level == BatteryLevel::Unknown);
    CHECK((p2.snapshot().flags & powerflag::StaleMeasurement) != 0u);
}

TEST(power_levels_with_confirmation_and_hysteresis)
{
    PowerManager pm(battery(), PowerPolicyConfig{}, nullptr, nullptr);
    pm.start(0u);
    uint32_t t = 0;
    auto feed = [&](uint16_t v, int n) {
        for (int i = 0; i < n; ++i, t += 1000u) pm.update(mv(v, t), PowerContext{}, t);
    };
    feed(3900u, 3);
    CHECK(pm.snapshot().level == BatteryLevel::Healthy);
    feed(3480u, 2);                                 // one load dip is not enough
    CHECK(pm.snapshot().level == BatteryLevel::Healthy);
    feed(3480u, 1);
    CHECK(pm.snapshot().level == BatteryLevel::Low);
    feed(3520u, 5);                                 // inside the hysteresis band: stays Low
    CHECK(pm.snapshot().level == BatteryLevel::Low);
    feed(3290u, 3);
    CHECK(pm.snapshot().level == BatteryLevel::Critical);
    feed(3560u, 3);                                 // above low + hysteresis but charger lifting? no: not charging
    CHECK(pm.snapshot().level != BatteryLevel::Critical);
}

TEST(power_charging_states_conflict_and_level_held_while_charging)
{
    PowerManager pm(battery(), PowerPolicyConfig{}, nullptr, nullptr);
    pm.start(0u);
    for (uint32_t t = 0; t < 3000u; t += 1000u) pm.update(mv(3250u, t), PowerContext{}, t);
    CHECK(pm.snapshot().level == BatteryLevel::Critical);
    for (uint32_t t = 3000u; t < 9000u; t += 1000u) pm.update(mv(4150u, t, Signal::Active), PowerContext{}, t);
    CHECK(pm.snapshot().charge == ChargeState::Charging);
    CHECK(pm.snapshot().supply == SupplyState::Present);
    CHECK(pm.snapshot().level == BatteryLevel::Critical);   // charger voltage is not the cell's
    CHECK((pm.snapshot().flags & powerflag::LevelHeldWhileCharging) != 0u);
    pm.update(mv(4180u, 9000u, Signal::Inactive, Signal::Active), PowerContext{}, 9000u);
    CHECK(pm.snapshot().charge == ChargeState::Complete);
    pm.update(mv(4180u, 10000u, Signal::Active, Signal::Active), PowerContext{}, 10000u);
    CHECK(pm.snapshot().charge == ChargeState::Fault);
    CHECK((pm.snapshot().flags & powerflag::ChargerConflict) != 0u);
    PowerInputs u = mv(4000u, 11000u, Signal::Unknown, Signal::Unknown);
    pm.update(u, PowerContext{}, 11000u);
    CHECK(pm.snapshot().charge == ChargeState::Unknown);
    CHECK(pm.snapshot().supply == SupplyState::Unknown);
    u.supplyPresent = Signal::Inactive;
    pm.update(u, PowerContext{}, 12000u);
    CHECK(pm.snapshot().supply == SupplyState::Disconnected);
}

TEST(power_rail_and_amplifier_sequencing_hold_and_faults)
{
    Switch rail, amp;
    PowerPolicyConfig pol{};
    pol.lidarOnByDefault = false;   // isolate the audio path
    RailConfig rc{};
    rc.settleMs = 20u;
    PowerManager pm(battery(), pol, &rail, &amp, rc, rc);
    pm.start(0u);
    CHECK(!rail.on && !amp.on);                     // explicit safe default: off
    PowerContext ctx{};
    ctx.audioRequested = true;
    pm.update(mv(3900u, 0u), ctx, 0u);
    CHECK(rail.on);
    CHECK(!amp.on);                                 // amplifier waits for a settled rail
    pm.update(mv(3900u, 20u), ctx, 20u);            // rail settled -> amplifier enabled
    CHECK(amp.on);
    CHECK(!pm.snapshot().audioReady);               // amplifier still settling
    pm.update(mv(3900u, 40u), ctx, 40u);
    CHECK(pm.snapshot().audioReady);
    ctx.audioRequested = false;
    pm.update(mv(3900u, 500u), ctx, 500u);
    CHECK(amp.on);                                  // hold time: no toggling between beeps
    pm.update(mv(3900u, 2000u), ctx, 2000u);
    CHECK(!amp.on);
    const size_t toggles = amp.history.size();
    CHECK(toggles <= 3u);

    rail.good = Signal::Inactive;                   // power-good feedback says the rail failed
    ctx.audioRequested = true;
    for (uint32_t t = 2100u; t < 2400u; t += 20u) pm.update(mv(3900u, t), ctx, t);
    CHECK(pm.snapshot().rail5v == RailState::Fault);
    CHECK(!amp.on);                                 // never enabled on a failed rail
    CHECK(!pm.snapshot().audioReady);
    CHECK(pm.snapshot().railFaults >= 1u);
}

TEST(power_lidar_policy_is_conservative_and_emergency_never_sheds)
{
    Switch rail, amp;
    PowerPolicyConfig pol{};
    pol.allowLidarShedOnGround = true;
    PowerManager pm(battery(), pol, &rail, &amp);
    pm.start(0u);
    PowerContext ctx{};
    ctx.inFlight = true;
    for (uint32_t t = 0; t < 4000u; t += 1000u) pm.update(mv(3200u, t), ctx, t);
    CHECK(pm.snapshot().level == BatteryLevel::Critical);
    CHECK(pm.snapshot().lidarPowerRequested);       // in flight: never shed
    ctx.inFlight = false;
    ctx.emergencyActive = true;
    pm.update(mv(3200u, 5000u), ctx, 5000u);
    CHECK(pm.snapshot().lidarPowerRequested);       // open emergency: never shed
    CHECK(pm.audioAllowed(10u, true));              // and every alert may sound
    ctx.emergencyActive = false;
    pm.update(mv(3200u, 6000u), ctx, 6000u);
    CHECK(!pm.snapshot().lidarPowerRequested);      // ground, critical, explicitly allowed
    CHECK(!pm.audioAllowed(45u, false));
    CHECK(pm.audioAllowed(90u, false));
    PowerManager def(battery(), PowerPolicyConfig{}, &rail, &amp);   // default policy keeps it on
    def.start(0u);
    for (uint32_t t = 0; t < 4000u; t += 1000u) def.update(mv(3200u, t), PowerContext{}, t);
    CHECK(def.snapshot().lidarPowerRequested);
}

// ---------------------------------------------------------------------------
TEST(pcm_tone_frequency_duration_ramp_and_volume)
{
    mock::NoLock lock;
    PcmToneOutput out(lock);
    out.setAmplifierReady(true, 1u);
    CHECK(isOk(out.request(AlertRequest{AlertTone::Critical, AlertPriority::Critical, 100u, 50u})));
    std::vector<int16_t> buf(2000);
    out.render(buf.data(), buf.size());
    // 100 ms at 16 kHz = 1600 samples, then silence
    size_t lastNonZero = 0, crossings = 0;
    int peak = 0;
    for (size_t i = 0; i < buf.size(); ++i) {
        if (buf[i] != 0) lastNonZero = i;
        if (i > 0 && (buf[i - 1] < 0) != (buf[i] < 0) && i < 1600u) ++crossings;
        peak = std::max(peak, std::abs(static_cast<int>(buf[i])));
    }
    CHECK(lastNonZero < 1600u && lastNonZero > 1590u);
    const double hz = crossings / 2.0 / 0.1;
    std::printf("    [audio] Critical tone: %.0f Hz measured (table %u Hz), peak %d (50%% of cap %u)\n", hz,
                toneFrequencyHz(AlertTone::Critical), peak, out.config().maxAmplitude);
    CHECK(std::fabs(hz - 2000.0) < 30.0);
    CHECK(peak <= 6000 && peak >= 5900);
    CHECK(std::abs(static_cast<int>(buf[0])) < 200);   // attack ramp: no click
    CHECK(!out.isPlaying());
    CHECK_EQ(out.stats().tonesAudible, 1u);
}

TEST(pcm_tone_waits_for_amplifier_then_drops_honestly)
{
    mock::NoLock lock;
    PcmToneOutput out(lock);
    CHECK(isOk(out.request(AlertRequest{AlertTone::Warning, AlertPriority::High, 200u, 80u})));
    CHECK(out.wantsAmplifier());
    int16_t buf[256];
    out.service(1000u);
    out.render(buf, 256);
    for (int16_t v : buf) CHECK_EQ(v, 0);           // amp off: silence, not "played"
    out.setAmplifierReady(true, 1100u);             // amp came up within the start delay
    out.render(buf, 256);
    CHECK(out.isPlaying());
    CHECK_EQ(out.stats().tonesStarted, 1u);
    out.setAmplifierReady(false, 1150u);            // amplifier lost mid-tone
    out.render(buf, 256);
    CHECK(!out.isPlaying());
    CHECK_EQ(out.stats().droppedNoAmplifier, 1u);
    CHECK_EQ(out.stats().tonesAudible, 0u);
    out.request(AlertRequest{AlertTone::Advisory, AlertPriority::Low, 100u, 50u});
    out.service(2000u);
    out.service(2300u);                             // > maxStartDelayMs, amp still off
    CHECK(!out.wantsAmplifier());
    CHECK_EQ(out.stats().droppedNoAmplifier, 2u);
}

TEST(pcm_tone_preemption_releases_then_starts_new_tone)
{
    mock::NoLock lock;
    PcmToneOutput out(lock);
    out.setAmplifierReady(true, 1u);
    out.request(AlertRequest{AlertTone::Advisory, AlertPriority::Low, 0u, 50u});   // until cancelled
    std::vector<int16_t> buf(400);
    out.render(buf.data(), buf.size());
    out.request(AlertRequest{AlertTone::Emergency, AlertPriority::Critical, 50u, 100u});
    out.render(buf.data(), buf.size());             // release of the old tone (ramp), then ends
    out.render(buf.data(), buf.size());             // new tone starts
    CHECK(out.isPlaying());
    CHECK_EQ(out.stats().preempted, 1u);
    CHECK_EQ(out.stats().tonesStarted, 2u);
    out.cancel(AlertTone::Emergency);
    out.render(buf.data(), buf.size());
    CHECK(!out.isPlaying());
}

// ---------------------------------------------------------------------------
namespace {
struct Pin final : IDigitalOutput {
    bool level = false;
    int writes = 0;
    void write(bool on) override { level = on; ++writes; }
};
}  // namespace

TEST(gpio_led_follows_reference_timing_colour_and_only_writes_changes)
{
    Pin r, g;
    GpioStatusLed led(&r, &g, nullptr);
    CHECK(led.available());
    led.setPattern(LedPattern::SlowBlink, LedColor::Amber);
    int mismatches = 0;
    for (uint32_t t = 1000u; t < 5000u; t += 10u) {
        led.tick(t);
        const bool ref = LedPatternScheduler::levelAt(LedPattern::SlowBlink, t - 1000u);
        mismatches += (r.level != ref || g.level != ref) ? 1 : 0;
    }
    CHECK_EQ(mismatches, 0);
    CHECK(r.writes < 20);                            // edges only, not every tick
    led.setPattern(LedPattern::SolidOn, LedColor::Blue);   // blue not wired -> first output
    led.tick(6000u);
    CHECK(r.level);
    CHECK(!g.level);
    GpioStatusLed none(nullptr, nullptr, nullptr);
    CHECK(!none.available());
    none.tick(0u);
}

// ---------------------------------------------------------------------------
TEST(alerts_low_battery_fault_and_phone_loss_priorities)
{
    mock::AudioAlert audio;
    mock::StatusLed led;
    SafetyEventDispatcher d(DispatcherConfig{}, &audio, &led);
    EmergencyStatus em{};
    em.state = EmergencyState::Idle;
    SystemAlertInputs sys{};
    sys.batteryLevel = 2u;
    sys.phoneLinkKnown = true;
    sys.phoneLinkUp = true;
    sys.inFlight = true;
    d.setSystemInputs(sys, 0u);
    d.update(0u, DispatcherInputs{nullptr, nullptr, &em});
    CHECK(d.status().audioAlert == AlertKind::LowBattery);
    sys.phoneLinkUp = false;
    d.setSystemInputs(sys, 1000u);
    d.update(5000u, DispatcherInputs{nullptr, nullptr, &em});
    CHECK((d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::PhoneDisconnected))) == 0u);
    d.update(11500u, DispatcherInputs{nullptr, nullptr, &em});
    CHECK((d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::PhoneDisconnected))) != 0u);
    CHECK(d.status().audioAlert == AlertKind::LowBattery);   // 45 > 30
    sys.batteryLevel = 3u;
    sys.storageDegraded = true;
    d.setSystemInputs(sys, 12000u);
    d.update(12000u, DispatcherInputs{nullptr, nullptr, &em});
    CHECK(d.status().audioAlert == AlertKind::LowBattery);
    CHECK(d.status().severity[static_cast<uint8_t>(AlertKind::LowBattery)] == SafetySeverity::Warning);
    CHECK((d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::SystemFault))) != 0u);
    // an SOS countdown still outranks everything (critical battery during SOS)
    em.state = EmergencyState::Countdown;
    em.incidentId = 1u;
    em.countdownRemainingMs = 30000u;
    d.update(13000u, DispatcherInputs{nullptr, nullptr, &em});
    CHECK(d.status().audioAlert == AlertKind::SosCountdown);
    CHECK((d.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::PhoneDisconnected))) == 0u);
    // without setSystemInputs nothing new appears (existing behaviour preserved)
    SafetyEventDispatcher plain(DispatcherConfig{}, &audio, &led);
    EmergencyStatus idle{};
    plain.update(0u, DispatcherInputs{nullptr, nullptr, &idle});
    CHECK(plain.status().audioAlert == AlertKind::None);
}

// ---------------------------------------------------------------------------
namespace {
/// Register bus model of the LSM6DSRX FIFO: a queue of 7-byte words behind 0x78.
struct FifoBus final : IRegisterBus {
    std::deque<std::vector<uint8_t>> words;
    uint8_t status2 = 0u;
    uint8_t ctrl3 = 0u, ctrl4 = 0u;
    int failAfterReads = -1;
    Status readRegisters(uint8_t reg, uint8_t* dst, size_t len) override
    {
        if (failAfterReads == 0) return Status::BusError;
        if (failAfterReads > 0) --failAfterReads;
        if (reg == lsm6fifo::REG_FIFO_STATUS1 && len == 2u) {
            dst[0] = static_cast<uint8_t>(words.size() & 0xFFu);
            dst[1] = static_cast<uint8_t>(status2 | ((words.size() >> 8) & 0x3u));
            return Status::Ok;
        }
        if (reg == lsm6fifo::REG_FIFO_DATA_OUT_TAG && len == 7u && !words.empty()) {
            for (size_t i = 0; i < 7u; ++i) dst[i] = words.front()[i];
            words.pop_front();
            return Status::Ok;
        }
        return Status::InvalidArgument;
    }
    Status writeRegister(uint8_t reg, uint8_t v) override
    {
        if (reg == lsm6fifo::REG_FIFO_CTRL3) ctrl3 = v;
        if (reg == lsm6fifo::REG_FIFO_CTRL4) ctrl4 = v;
        return Status::Ok;
    }
    void push(uint8_t tag, int16_t x, int16_t y, int16_t z)
    {
        words.push_back({static_cast<uint8_t>(tag << 3), static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8),
                         static_cast<uint8_t>(y), static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(z),
                         static_cast<uint8_t>(z >> 8)});
    }
};
}  // namespace

TEST(imu_fifo_captures_a_peak_that_polling_would_miss)
{
    FifoBus bus;
    Lsm6dsrxFifo fifo(bus, Lsm6dsrxAccelRange::G16, Lsm6dsrxGyroRange::Dps2000);
    FifoBatch b{};
    CHECK(fifo.drain(0u, b) == Status::NotInitialized);
    CHECK(isOk(fifo.configure(FifoConfig{})));
    CHECK_EQ(bus.ctrl4, lsm6fifo::MODE_CONTINUOUS);
    CHECK_EQ(bus.ctrl3, static_cast<uint8_t>(0x66u));   // 416 Hz both
    // 10 ms at 416 Hz: 4 accel + 4 gyro samples; one 2.4 ms impact spike in the middle.
    // A 100 Hz poll reads only the LAST accel sample (1 g).
    const int16_t oneG = static_cast<int16_t>(1000.0f / 0.488f);
    bus.push(lsm6fifo::TAG_ACCEL, 0, 0, oneG);
    bus.push(lsm6fifo::TAG_GYRO, 10, 0, 0);
    bus.push(lsm6fifo::TAG_ACCEL, 20000, 0, oneG);   // ~9.8 g spike
    bus.push(lsm6fifo::TAG_GYRO, 0, 15000, 0);
    bus.push(lsm6fifo::TAG_ACCEL, 0, 0, oneG);
    bus.push(lsm6fifo::TAG_ACCEL, 0, 0, oneG);
    CHECK(isOk(fifo.drain(10u, b)));
    std::printf("    [fifo] peak %.1f m/s^2 (poll would see %.1f), %u accel / %u gyro samples\n",
                static_cast<double>(b.peakAccel_mps2), 9.81, b.accelSamples, b.gyroSamples);
    CHECK_EQ(b.accelSamples, 4u);
    CHECK_EQ(b.gyroSamples, 2u);
    CHECK(b.peakAccel_mps2 > 90.0f && b.peakAccel_mps2 < 100.0f);
    CHECK(std::isfinite(b.peakRate_radps));
    CHECK(!b.overflow && b.wordsLeft == 0u);
}

TEST(imu_fifo_overflow_saturation_bound_and_partial_read)
{
    FifoBus bus;
    Lsm6dsrxFifo fifo(bus, Lsm6dsrxAccelRange::G16, Lsm6dsrxGyroRange::Dps2000);
    FifoConfig cfg{};
    cfg.maxWordsPerDrain = 4u;
    fifo.configure(cfg);
    for (int i = 0; i < 10; ++i) bus.push(lsm6fifo::TAG_ACCEL, INT16_MAX, 0, 0);
    bus.push(0x1Fu, 1, 2, 3);   // unknown tag (corruption / unsupported batching)
    bus.status2 = lsm6fifo::STATUS2_OVR;
    FifoBatch b{};
    CHECK(isOk(fifo.drain(0u, b)));
    CHECK(b.overflow);                   // samples were lost: reported, never hidden
    CHECK(b.accelSaturated);             // full scale reached: peak is a lower bound
    CHECK_EQ(b.wordsRead, 4u);           // bounded: other sensors are not starved
    CHECK_EQ(b.wordsLeft, 7u);
    bus.status2 = 0u;
    bus.failAfterReads = 3;              // status + 2 words, then a bus error
    CHECK(!isOk(fifo.drain(10u, b)));
    CHECK_EQ(b.wordsRead, 2u);
    CHECK_EQ(fifo.partialReads(), 1u);
    bus.failAfterReads = -1;
    fifo.drain(20u, b);
    fifo.drain(30u, b);
    CHECK_EQ(b.unknownWords, 1u);
    CHECK(std::isnan(b.peakRate_radps));   // no gyro sample: NaN, not 0
    CHECK(isOk(fifo.disable()));
    CHECK_EQ(bus.ctrl4, lsm6fifo::MODE_BYPASS);
}

// ---------------------------------------------------------------------------
TEST(supervisor_stall_detection_and_emergency_safe_reset_deferral)
{
    TaskSupervisor sup(5000u);
    const int acq = sup.add("SensorAcq", 200u, TaskCriticality::Critical);
    const int sto = sup.add("Storage", 2000u, TaskCriticality::Storage);
    const int hlt = sup.add("Health", 3000u, TaskCriticality::Diagnostic);
    CHECK(acq == 0 && sto == 1 && hlt == 2);
    SupervisorContext ctx{false, true};
    uint32_t c = 0;
    for (uint32_t t = 0; t < 1000u; t += 100u) {
        sup.observe(acq, ++c, t);
        sup.observe(sto, c, t);
        sup.observe(hlt, c, t);
        CHECK(sup.evaluate(t, ctx).action == SupervisorAction::None);
    }
    // storage stalls: degrade, never reset for logging
    for (uint32_t t = 1000u; t < 4000u; t += 100u) sup.observe(acq, ++c, t);
    SupervisorVerdict v = sup.evaluate(4000u, ctx);
    CHECK(v.action == SupervisorAction::Degrade);
    CHECK(sup.feedWatchdog());
    // acquisition stalls while an emergency is open and NOT durable: reset deferred ...
    ctx = SupervisorContext{true, false};
    v = sup.evaluate(4300u, ctx);
    CHECK(v.resetDeferred);
    CHECK(sup.feedWatchdog());
    v = sup.evaluate(8000u, ctx);
    CHECK(v.resetDeferred);
    v = sup.evaluate(9400u, ctx);       // ... but not forever: a stuck SOS path is worse
    CHECK(v.action == SupervisorAction::RequestReset);
    CHECK(!sup.feedWatchdog());
    // with the emergency state durable the reset is immediate (restore() continues the SOS)
    TaskSupervisor s2;
    const int em = s2.add("EmergencyAlert", 200u, TaskCriticality::Critical);
    s2.observe(em, 1u, 0u);
    CHECK(s2.evaluate(300u, SupervisorContext{true, true}).action == SupervisorAction::RequestReset);
    CHECK(s2.liveness(em) == TaskLiveness::Stalled);
    CHECK_EQ(s2.add("x", 0u, TaskCriticality::Diagnostic), -1);
}
