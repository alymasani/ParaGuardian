/**
 * Regression tests for the reliability audit (Docs/FIRMWARE_RELIABILITY_AUDIT.md).
 *
 * Timing tests use ONE MockClock shared by every driver and the scheduler.
 * The fake clock only advances when code calls delayMs(), so the measured
 * "cycle time" is exactly the time the code chose to WAIT (reset waits,
 * polling sleeps) - the part that can break a 10 ms deadline. Bus transfer
 * time is not modelled here; RTOS_TIMING_ANALYSIS.md estimates it separately.
 */
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"
#include "mocks/FakeUart.hpp"
#include "mocks/MockClock.hpp"

#include "Acquisition/AcquisitionScheduler.hpp"
#include "Acquisition/AcquisitionSources.hpp"
#include "Acquisition/LatestSampleMailbox.hpp"
#include "Acquisition/SampleFreshness.hpp"
#include "Acquisition/SampleQueue.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "Common/ByteRingBuffer.hpp"
#include "Common/InterruptMask.hpp"
#include "Common/RegisterBus.hpp"

using namespace pg;

namespace {

struct CollectingSink final : public ISampleSink {
    std::vector<SensorMessage> got;
    void publish(const SensorMessage& m) override { got.push_back(m); }
    size_t count(SensorMessageType t) const
    {
        size_t n = 0;
        for (const auto& m : got) n += (m.type == t) ? 1u : 0u;
        return n;
    }
};

/// LSM6DSRX fake on a shared clock. resetReads < 0 = SW_RESET never clears.
struct ImuDev {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    Lsm6dsrx imu;
    ImuDev(MockClock& clock, SensorId id, bool present = true, bool resetHangs = false) : imu(bus, clock, id)
    {
        dev.regs[0x0F] = present ? 0x6B : 0x00;
        dev.regs[0x2D] = 0x40;  // AZ = 0x4000: plausible non-zero data
        dev.onRead = [this, resetHangs](uint8_t reg) {
            if (reg == 0x12 && !resetHangs) dev.regs[0x12] &= static_cast<uint8_t>(~0x01);
        };
    }
};

struct BaroDev {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    Bmp585 baro;
    BaroDev(MockClock& clock, SensorId id, bool present = true) : baro(bus, clock, id)
    {
        dev.regs[0x01] = present ? 0x51 : 0x00;
        dev.regs[0x28] = 0x02;
        dev.regs[0x27] = 0x10;
        dev.regs[0x38] = 0x80;
        dev.regs[0x20] = 0x40; dev.regs[0x21] = 0xF3; dev.regs[0x22] = 0x62;  // 101325 Pa
    }
};

/// MMC5983MA fake: a measurement completes `conversionMs` after TM_M is
/// written (time-based, like the real chip: 8 ms at 100 Hz bandwidth).
struct MagDev {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    MockClock& clock;
    Mmc5983ma mag;
    uint32_t conversionMs = 8u;
    bool converting = false;
    uint32_t startedMs = 0u;
    explicit MagDev(MockClock& c) : clock(c), mag(bus, c)
    {
        dev.regs[0x2F] = 0x30;
        // 131072 + 8192 counts on X (+50 uT): X0 = 0x88
        dev.regs[0x00] = 0x88; dev.regs[0x02] = 0x80; dev.regs[0x04] = 0x80;
        dev.onWrite = [this](uint8_t reg, uint8_t v) {
            if (reg == 0x09 && (v & 0x01)) { converting = true; startedMs = clock.now; }
            if (reg == 0x08) dev.regs[0x08] = static_cast<uint8_t>(dev.regs[0x08] & ~v);
        };
        dev.onRead = [this](uint8_t reg) {
            if (reg == 0x08 && converting && clock.now - startedMs >= conversionMs) {
                dev.regs[0x08] |= 0x01;
                converting = false;
            }
        };
    }
};

/// Source that always blocks for a fixed time (models a slow bus transaction).
struct BlockingSource final : public IAcquisitionSource {
    using IAcquisitionSource::service;
    MockClock& clock;
    uint32_t blockMs;
    SensorHealth h{};
    BlockingSource(MockClock& c, uint32_t ms) : clock(c), blockMs(ms) {}
    const char* name() const override { return "BLOCK"; }
    void service(uint32_t, ISampleSink&, CycleBudget&) override { clock.delayMs(blockMs); }
    const SensorHealth& health() const override { return h; }
    uint32_t initFailures() const override { return 0u; }
};

SensorMessage imuMsg(SensorId id, uint32_t seq, uint32_t t)
{
    ImuSample s{};
    s.header = SampleHeader{id, t, seq, true};
    return SensorMessage::fromImu(s);
}

/// Lock for host concurrency tests.
struct MutexLock final : public ICriticalSection {
    std::mutex m;
    void enter() override { m.lock(); }
    void exit() override { m.unlock(); }
};

}  // namespace

// ---------------------------------------------------------------------------
// Audit A: interrupt masking rule used by the HAL tick fallback
// ---------------------------------------------------------------------------
TEST(audit_a_basepri_masks_tim6_before_scheduler)
{
    // Pre-scheduler state measured from the FreeRTOS port: BASEPRI = 5 << 4 = 0x50.
    CHECK(interruptBlockedByMask(0u, 0x50u, 15u, 4u));    // TIM6 tick (prio 15): BLOCKED
    CHECK(interruptBlockedByMask(0u, 0x50u, 5u, 4u));     // prio 5 == BASEPRI: blocked
    CHECK(!interruptBlockedByMask(0u, 0x50u, 4u, 4u));    // prio 4 is more urgent: runs
    CHECK(!interruptBlockedByMask(0u, 0x00u, 15u, 4u));   // after scheduler start: BASEPRI 0
    CHECK(interruptBlockedByMask(1u, 0x00u, 0u, 4u));     // PRIMASK blocks everything
    CHECK(!interruptBlockedByMask(0u, 0xF0u, 14u, 4u));   // BASEPRI 15 lets 14 through
}

// ---------------------------------------------------------------------------
// Audit B: blocking behaviour in the acquisition path
// ---------------------------------------------------------------------------
TEST(audit_b_old_blocking_mag_measure_consumes_cycle)
{
    // Reproduces the original problem: measure() sleeps until the conversion is done.
    MockClock clock;
    MagDev m(clock);
    m.conversionMs = 8u;  // 100 Hz-bandwidth conversion
    CHECK(isOk(m.mag.init(Mmc5983maConfig{})));
    const uint32_t start = clock.now;
    MagSample s{};
    CHECK(isOk(m.mag.measure(s)));
    CHECK(clock.now - start >= 8u);  // 80 % of a 10 ms cycle spent waiting
}

TEST(audit_b_mag_source_is_non_blocking_two_phase)
{
    MockClock clock;
    MagDev m(clock);
    m.conversionMs = 8u;   // done before the next 10 ms cycle
    MagSource src("MAG", m.mag, Mmc5983maConfig{}, 20u);
    CollectingSink sink;
    src.service(0, sink);  // first cycle: the source's one-time init (10 ms power-on wait)
    CHECK(m.mag.isInitialized());
    uint32_t waitedInCycles = 0;
    for (uint32_t t = 10; t < 210; t += 10) {
        clock.now = t;
        const uint32_t before = clock.totalDelayMs;
        src.service(t, sink);
        waitedInCycles += clock.totalDelayMs - before;
    }
    CHECK_EQ(waitedInCycles, 0u);                 // never sleeps inside a cycle
    // Period 20 ms: trigger at t, collect at t+10, trigger again at t+20
    // -> 10 samples in 200 ms, with no waiting inside any cycle.
    CHECK_EQ(sink.count(SensorMessageType::Mag), 10u);
    CHECK_NEAR(sink.got.back().mag.field_uT.x, 50.0, 1e-3);
    CHECK(src.health().isHealthy());
}

TEST(audit_b_mag_nonblocking_timeout_detected)
{
    MockClock clock;
    MagDev m(clock);
    CHECK(isOk(m.mag.init(Mmc5983maConfig{})));
    m.conversionMs = 1000000u;  // conversion never finishes
    CHECK(isOk(m.mag.startMeasurement()));
    CHECK(m.mag.startMeasurement() == Status::Busy);  // one at a time
    MagSample s{};
    CHECK(m.mag.readMeasurementIfReady(s) == Status::NotReady);
    clock.now += m.mag.measurementTimeoutMs();
    CHECK(m.mag.readMeasurementIfReady(s) == Status::Timeout);
    CHECK(!m.mag.measurementPending());
    CHECK_EQ(m.mag.health().timeoutCount, 1u);
}

TEST(audit_b_mag_late_poll_of_finished_conversion_is_not_a_timeout)
{
    // An overrunning cycle may poll 25 ms after the trigger (> 18 ms timeout).
    // The conversion finished long ago, so the data must be accepted.
    MockClock clock;
    MagDev m(clock);
    CHECK(isOk(m.mag.init(Mmc5983maConfig{})));
    CHECK(isOk(m.mag.startMeasurement()));
    clock.now += 25u;
    MagSample s{};
    CHECK(isOk(m.mag.readMeasurementIfReady(s)));
    CHECK(s.header.valid);
    CHECK_EQ(m.mag.health().timeoutCount, 0u);
}

TEST(audit_b_simultaneous_init_failures_are_spread_across_cycles)
{
    // Three IMUs whose reset never completes: each init() waits the full 50 ms.
    MockClock clock;
    ImuDev a(clock, SensorId::Imu0, true, true), b(clock, SensorId::Imu1, true, true),
        c(clock, SensorId::Imu2, true, true);
    ImuSource sa("IMU0", a.imu, Lsm6dsrxConfig{}, 10u), sb("IMU1", b.imu, Lsm6dsrxConfig{}, 10u),
        sc("IMU2", c.imu, Lsm6dsrxConfig{}, 10u);
    AcquisitionScheduler sched(clock, 10u, 1u);
    CHECK(sched.add(&sa) && sched.add(&sb) && sched.add(&sc));
    CollectingSink sink;
    const uint32_t first = sched.runCycle(sink);
    CHECK(first <= Lsm6dsrx::kResetTimeoutMs + 3u);   // ONE init (~50 ms), not three (~150 ms)
    CHECK_EQ(sched.stats().initAttemptsDeferred, 2u);
    sched.runCycle(sink);
    sched.runCycle(sink);
    CHECK_EQ(sa.initFailures() + sb.initFailures() + sc.initFailures(), 3u);  // each tried once
}

TEST(audit_b_without_budget_init_waits_stack_up)
{
    // Same scenario with an unlimited budget = the original behaviour.
    MockClock clock;
    ImuDev a(clock, SensorId::Imu0, true, true), b(clock, SensorId::Imu1, true, true),
        c(clock, SensorId::Imu2, true, true);
    ImuSource sa("IMU0", a.imu, Lsm6dsrxConfig{}, 10u), sb("IMU1", b.imu, Lsm6dsrxConfig{}, 10u),
        sc("IMU2", c.imu, Lsm6dsrxConfig{}, 10u);
    AcquisitionScheduler sched(clock, 10u, 3u);
    sched.add(&sa); sched.add(&sb); sched.add(&sc);
    CollectingSink sink;
    CHECK(sched.runCycle(sink) >= 3u * Lsm6dsrx::kResetTimeoutMs);
}

TEST(audit_b_two_barometers_fail_together_retry_with_backoff)
{
    MockClock clock;
    BaroDev b0(clock, SensorId::Baro0, false), b1(clock, SensorId::Baro1, false);
    BaroSource s0("BARO0", b0.baro, Bmp585Config{}, 20u), s1("BARO1", b1.baro, Bmp585Config{}, 20u);
    AcquisitionScheduler sched(clock, 10u, 1u);
    sched.add(&s0); sched.add(&s1);
    CollectingSink sink;
    // Drive the loop for 40 s of virtual time, 10 ms per cycle.
    std::vector<uint32_t> attempts0;
    uint32_t last = 0;
    for (uint32_t t = 0; t < 40000; t += 10) {
        clock.now = t;
        sched.runCycle(sink);
        if (s0.initFailures() != last) { attempts0.push_back(t); last = s0.initFailures(); }
    }
    // Retry gaps grow 1 s, 2 s, 4 s, then stay at the 8 s cap.
    CHECK(attempts0.size() >= 6u);
    CHECK_NEAR(attempts0[1] - attempts0[0], 1000, 20);
    CHECK_NEAR(attempts0[2] - attempts0[1], 2000, 20);
    CHECK_NEAR(attempts0[3] - attempts0[2], 4000, 20);
    CHECK_NEAR(attempts0[4] - attempts0[3], 8000, 20);
    CHECK_NEAR(attempts0[5] - attempts0[4], 8000, 20);
    CHECK(s1.initFailures() >= 6u);  // the second barometer is not starved
    CHECK_EQ(sched.stats().cycleOverruns, 0u);  // failing fast never blows the cycle
}

TEST(audit_b_dead_sensor_after_init_triggers_reinit_and_recovers)
{
    MockClock clock;
    ImuDev a(clock, SensorId::Imu0);
    ImuSource src("IMU0", a.imu, Lsm6dsrxConfig{}, 10u);
    CollectingSink sink;
    src.service(0, sink);
    CHECK_EQ(sink.got.size(), 1u);
    a.dev.floatingBus = true;  // connector falls off
    uint32_t t = 10;
    for (; t <= 100; t += 10) src.service(t, sink);  // 10 failed reads
    CHECK_EQ(src.reinitCount(), 1u);
    CHECK(src.needsInit());
    // Detection: health turns unhealthy after 3 consecutive failures (= 30 ms at 100 Hz).
    CHECK(!src.health().isHealthy());
    a.dev.floatingBus = false;  // connector back
    src.service(t, sink);       // immediate re-init attempt succeeds and reads
    CHECK(!src.needsInit());
    CHECK_EQ(sink.got.size(), 2u);
    CHECK(src.health().isHealthy());
}

TEST(audit_b_health_detection_time_three_cycles)
{
    MockClock clock;
    ImuDev a(clock, SensorId::Imu0);
    ImuSource src("IMU0", a.imu, Lsm6dsrxConfig{}, 10u);
    CollectingSink sink;
    src.service(0, sink);
    a.dev.floatingBus = true;
    src.service(10, sink);
    src.service(20, sink);
    CHECK(src.health().isHealthy());   // 2 failures: still tolerated
    src.service(30, sink);
    CHECK(!src.health().isHealthy());  // 3rd failure: flagged 30 ms after the fault
}

TEST(audit_b_full_sensor_set_normal_cycle_has_no_waits)
{
    MockClock clock;
    ImuDev i0(clock, SensorId::Imu0), i1(clock, SensorId::Imu1), i2(clock, SensorId::Imu2);
    BaroDev b0(clock, SensorId::Baro0), b1(clock, SensorId::Baro1);
    MagDev m(clock);
    NeoM9n gnss;
    Tf02Pro lidar;
    FakeByteSource gnssRx, lidarRx;
    ImuSource si0("IMU0", i0.imu, Lsm6dsrxConfig{}, 10u), si1("IMU1", i1.imu, Lsm6dsrxConfig{}, 10u),
        si2("IMU2", i2.imu, Lsm6dsrxConfig{}, 10u);
    BaroSource sb0("BARO0", b0.baro, Bmp585Config{}, 20u), sb1("BARO1", b1.baro, Bmp585Config{}, 20u);
    MagSource sm("MAG", m.mag, Mmc5983maConfig{}, 20u);
    GnssSource sg("GNSS", gnss, gnssRx);
    LidarSource sl("LIDAR", lidar, lidarRx);
    AcquisitionScheduler sched(clock, 10u, 1u);
    IAcquisitionSource* all[] = {&si0, &si1, &si2, &sb0, &sb1, &sm, &sg, &sl};
    for (auto* s : all) CHECK(sched.add(s));
    CollectingSink sink;
    // Start-up: one init per cycle -> 6 polled sources need 6 cycles.
    for (int k = 0; k < 6; ++k) { sched.runCycle(sink); clock.now += 10; }
    const uint32_t maxStartup = sched.stats().maxCycleTimeMs;
    CHECK(maxStartup <= 13u);  // worst start-up cycle = one MMC5983MA init (10 ms + 1 ms SET + tick)
    // Steady state: one second.
    const uint32_t overrunsBefore = sched.stats().cycleOverruns;
    for (int k = 0; k < 100; ++k) {
        lidarRx.add(std::vector<uint8_t>(115, 0x00));  // 1 cycle of 115200-baud LiDAR traffic
        const uint32_t before = clock.now;
        sched.runCycle(sink);
        CHECK_EQ(clock.now - before, 0u);  // steady-state cycles never sleep
        clock.now += 10;
    }
    CHECK_EQ(sched.stats().cycleOverruns, overrunsBefore);
    CHECK_EQ(lidarRx.pending.size(), 0u);  // drained every cycle, no backlog
    CHECK(sink.count(SensorMessageType::Imu) >= 290u);  // 3 IMUs x ~100 reads
    CHECK(sink.count(SensorMessageType::Baro) >= 98u);  // 2 baros x ~50
    CHECK(sink.count(SensorMessageType::Mag) >= 40u);   // ~50 at 20 ms (two-phase)
}

TEST(audit_b_overrun_is_observable)
{
    MockClock clock;
    BlockingSource slow(clock, 15u);  // e.g. an I2C transaction stuck in HAL's 25 ms busy wait
    AcquisitionScheduler sched(clock, 10u);
    sched.add(&slow);
    CollectingSink sink;
    sched.runCycle(sink);
    CHECK_EQ(sched.stats().cycleOverruns, 1u);
    CHECK_EQ(sched.stats().maxCycleTimeMs, 15u);
    CHECK_EQ(sched.stats().lastCycleTimeMs, 15u);
}

TEST(audit_b_gnss_burst_during_heavy_activity_drains_bounded)
{
    MockClock clock;
    NeoM9n gnss;
    FakeByteSource rx;
    GnssSource src("GNSS", gnss, rx);
    CollectingSink sink;
    rx.add(std::vector<uint8_t>(1500, 0x00));  // backlog after a stall
    int cycles = 0;
    while (!rx.pending.empty() && cycles < 10) {
        src.service(clock.now, sink);
        ++cycles;
        clock.now += 10;
    }
    CHECK_EQ(cycles, 3);  // 512 B per cycle: backlog cleared in 3 cycles, no single long cycle
}

// ---------------------------------------------------------------------------
// Audit C: overflow policies, freshness, sequence gaps
// ---------------------------------------------------------------------------
TEST(audit_c_drop_oldest_keeps_newest_in_order)
{
    SensorDataHub hub;
    BoundedSampleQueue<4> nav;
    BoundedSampleQueue<4> log;
    CHECK(hub.subscribe(&nav, OverflowPolicy::DropOldest));
    CHECK(hub.subscribe(&log, OverflowPolicy::DropNewest));
    for (uint32_t i = 1; i <= 10; ++i) hub.publish(imuMsg(SensorId::Imu0, i, i * 10));
    // Navigation queue holds the 4 NEWEST, still oldest-first.
    SensorMessage m{};
    for (uint32_t expect = 7; expect <= 10; ++expect) {
        CHECK(nav.tryPop(m));
        CHECK_EQ(m.imu.header.sequence, expect);
    }
    CHECK_EQ(hub.overwritesFor(0), 6u);
    CHECK_EQ(hub.dropsFor(0), 0u);
    // Logging queue holds the 4 OLDEST (unbroken prefix) and reports 6 drops.
    for (uint32_t expect = 1; expect <= 4; ++expect) {
        CHECK(log.tryPop(m));
        CHECK_EQ(m.imu.header.sequence, expect);
    }
    CHECK_EQ(hub.dropsFor(1), 6u);
    CHECK_EQ(hub.overwritesFor(1), 0u);
    CHECK_EQ(hub.totalDrops(), 6u);
    CHECK_EQ(hub.totalOverwrites(), 6u);
}

TEST(audit_c_slow_consumer_sees_fresh_data_with_drop_oldest)
{
    // Producer: IMU every 10 ms. Consumer: wakes every 100 ms and takes ONE sample.
    SensorDataHub hub;
    BoundedSampleQueue<8> navOld, navNew;
    hub.subscribe(&navOld, OverflowPolicy::DropNewest);
    hub.subscribe(&navNew, OverflowPolicy::DropOldest);
    uint32_t seq = 0;
    uint32_t worstAgeOld = 0, worstAgeNew = 0;
    for (uint32_t t = 0; t <= 2000; t += 10) {
        hub.publish(imuMsg(SensorId::Imu0, ++seq, t));
        if (t % 100 == 0 && t > 0) {
            SensorMessage a{}, b{};
            CHECK(navOld.tryPop(a));
            CHECK(navNew.tryPop(b));
            const uint32_t ageA = sampleAgeMs(a.imu.header, t), ageB = sampleAgeMs(b.imu.header, t);
            worstAgeOld = ageA > worstAgeOld ? ageA : worstAgeOld;
            worstAgeNew = ageB > worstAgeNew ? ageB : worstAgeNew;
        }
    }
    // Drop-newest: the 8-deep queue becomes a backlog drained one item per 100 ms,
    // so served samples are ~8 x 100 ms old. Drop-oldest: within 8 x 10 ms.
    CHECK(worstAgeOld >= 700u);
    CHECK(worstAgeNew <= 70u);
}

TEST(audit_c_mailbox_latest_per_sensor_and_staleness)
{
    NoCriticalSection lock;
    LatestSampleMailbox box(lock);
    SensorDataHub hub;
    CHECK(hub.subscribeMailbox(&box));
    CHECK(!hub.subscribeMailbox(&box));  // duplicate rejected
    SensorMessage m{};
    CHECK(!box.read(SensorId::Baro0, m));  // never stored
    hub.publish(imuMsg(SensorId::Imu0, 1, 100));
    hub.publish(imuMsg(SensorId::Imu0, 2, 110));
    BaroSample b{};
    b.header = SampleHeader{SensorId::Baro0, 105, 7, true};
    hub.publish(SensorMessage::fromBaro(b));
    CHECK(box.read(SensorId::Imu0, m));
    CHECK_EQ(m.imu.header.sequence, 2u);  // overwritten with the newest
    CHECK(box.read(SensorId::Baro0, m));
    CHECK_EQ(m.baro.header.sequence, 7u);
    CHECK_EQ(box.writes(SensorId::Imu0), 2u);
    // Freshness: the baro sample is 15 ms old at t=120, 495 ms old at t=600.
    CHECK(isFresh(m.baro.header, 120, 50));
    CHECK(!isFresh(m.baro.header, 600, 50));
    m.baro.header.valid = false;
    CHECK(!isFresh(m.baro.header, 120, 50));  // invalid samples are never "fresh"
}

TEST(audit_c_freshness_across_counter_wrap)
{
    SampleHeader h{SensorId::Imu0, 0xFFFFFFF0u, 1, true};  // 16 ms before the 32-bit wrap
    CHECK_EQ(sampleAgeMs(h, 0x00000010u), 32u);
    CHECK(isFresh(h, 0x00000010u, 50));
}

TEST(audit_c_sequence_gaps_mixed_rates)
{
    // IMU at 100 Hz and baro at 50 Hz into a 3-deep drop-oldest queue; consumer
    // drains every 50 ms. The tracker must count exactly the samples it missed.
    SensorDataHub hub;
    BoundedSampleQueue<3> q;
    hub.subscribe(&q, OverflowPolicy::DropOldest);
    SequenceTracker tracker;
    uint32_t imuSeq = 0, baroSeq = 0;
    for (uint32_t t = 0; t < 1000; t += 10) {
        hub.publish(imuMsg(SensorId::Imu0, ++imuSeq, t));
        if (t % 20 == 0) {
            BaroSample b{};
            b.header = SampleHeader{SensorId::Baro0, t, ++baroSeq, true};
            hub.publish(SensorMessage::fromBaro(b));
        }
        if (t % 50 == 40) {
            SensorMessage m{};
            while (q.tryPop(m)) tracker.observe(m);
        }
    }
    const uint32_t delivered = tracker.received(SensorId::Imu0) + tracker.received(SensorId::Baro0);
    // Everything published is either received, counted lost, or never seen
    // because it was overwritten before the consumer's first read.
    CHECK(tracker.lost(SensorId::Imu0) > 0u);
    CHECK(tracker.received(SensorId::Imu0) + tracker.lost(SensorId::Imu0) <= imuSeq);
    CHECK_EQ(delivered + hub.overwritesFor(0) + q.size(), imuSeq + baroSeq);
    CHECK_EQ(tracker.outOfOrder(SensorId::Imu0), 0u);
}

TEST(audit_c_sequence_tracker_duplicates_and_gaps)
{
    SequenceTracker t;
    CHECK_EQ(t.observe(imuMsg(SensorId::Imu1, 5, 0)), 0u);   // first sample: no gap
    CHECK_EQ(t.observe(imuMsg(SensorId::Imu1, 6, 0)), 0u);
    CHECK_EQ(t.observe(imuMsg(SensorId::Imu1, 10, 0)), 3u);  // 7, 8, 9 missing
    CHECK_EQ(t.observe(imuMsg(SensorId::Imu1, 10, 0)), 0u);  // duplicate
    CHECK_EQ(t.lost(SensorId::Imu1), 3u);
    CHECK_EQ(t.outOfOrder(SensorId::Imu1), 1u);
    CHECK_EQ(t.lost(SensorId::Imu0), 0u);                     // per-sensor bookkeeping
}

TEST(audit_c_rtos_style_queue_never_blocks_publisher)
{
    // A zero-capacity-left queue with DropNewest: publish returns immediately and counts.
    SensorDataHub hub;
    BoundedSampleQueue<1> q;
    hub.subscribe(&q);
    for (int i = 0; i < 1000; ++i) hub.publish(imuMsg(SensorId::Imu0, static_cast<uint32_t>(i + 1), 0));
    CHECK_EQ(hub.dropsFor(0), 999u);
    CHECK_EQ(q.size(), 1u);
}

// ---------------------------------------------------------------------------
// Concurrency (real threads; run with SANITIZE=thread to check for races)
// ---------------------------------------------------------------------------
TEST(concurrency_ring_buffer_spsc_order_and_accounting)
{
    ByteRingBuffer<256> rb;
    constexpr uint32_t kBytes = 200000;
    std::atomic<bool> done{false};
    std::vector<uint8_t> received;
    received.reserve(kBytes);
    std::thread consumer([&] {
        uint8_t buf[64];
        for (;;) {
            const size_t n = rb.pop(buf, sizeof(buf));
            received.insert(received.end(), buf, buf + n);
            if (n == 0 && done.load(std::memory_order_acquire) && rb.size() == 0) break;
        }
    });
    uint32_t accepted = 0;
    std::vector<uint8_t> acceptedBytes;
    acceptedBytes.reserve(kBytes);
    for (uint32_t i = 0; i < kBytes; ++i) {
        const uint8_t b = static_cast<uint8_t>(i * 31u);
        if (rb.push(b)) { ++accepted; acceptedBytes.push_back(b); }
    }
    done.store(true, std::memory_order_release);
    consumer.join();
    CHECK_EQ(accepted + rb.overflowCount(), kBytes);       // every byte accounted for
    CHECK_EQ(received.size(), static_cast<size_t>(accepted));
    CHECK(received == acceptedBytes);                       // order preserved, nothing corrupted
}

TEST(concurrency_mailbox_reads_are_never_torn)
{
    MutexLock lock;
    LatestSampleMailbox box(lock);
    std::atomic<bool> stop{false};
    std::atomic<uint32_t> torn{0}, reads{0};
    std::thread reader([&] {
        SensorMessage m{};
        while (!stop.load()) {
            if (box.read(SensorId::Imu0, m)) {
                ++reads;
                // Writer keeps every field equal to the sequence number.
                const uint32_t s = m.imu.header.sequence;
                if (m.imu.header.timestampMs != s || m.imu.accel_mps2.x != static_cast<float>(s)) ++torn;
            }
        }
    });
    for (uint32_t s = 1; s <= 50000; ++s) {
        ImuSample i{};
        i.header = SampleHeader{SensorId::Imu0, s, s, true};
        i.accel_mps2.x = static_cast<float>(s);
        box.store(SensorMessage::fromImu(i));
    }
    stop.store(true);
    reader.join();
    CHECK_EQ(torn.load(), 0u);
    CHECK(reads.load() > 0u);
    CHECK_EQ(box.writes(SensorId::Imu0), 50000u);
}
