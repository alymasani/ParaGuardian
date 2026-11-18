/**
 * Acquisition-layer tests: hub overflow policy, init retry, per-source rates,
 * UART-stream sources. Uses the same fakes as the driver tests.
 */
#include <vector>

#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"
#include "mocks/FakeUart.hpp"
#include "mocks/MockClock.hpp"

#include "Acquisition/AcquisitionSources.hpp"
#include "Acquisition/SampleQueue.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "Common/RegisterBus.hpp"

using namespace pg;

namespace {
/// Collects published messages for inspection.
struct CollectingSink final : public ISampleSink {
    std::vector<SensorMessage> got;
    void publish(const SensorMessage& m) override { got.push_back(m); }
};

struct ImuRig {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    MockClock clock;
    Lsm6dsrx imu{bus, clock, SensorId::Imu0};
    ImuRig()
    {
        dev.regs[0x0F] = 0x6B;
        dev.onRead = [this](uint8_t reg) { if (reg == 0x12) dev.regs[0x12] &= static_cast<uint8_t>(~0x01); };
        dev.regs[0x2C] = 0x00;  // AZ = 0x4000 -> non-zero data
        dev.regs[0x2D] = 0x40;
    }
};
}  // namespace

TEST(hub_drop_newest_per_subscriber)
{
    SensorDataHub hub;
    BoundedSampleQueue<2> small;
    BoundedSampleQueue<8> big;
    CHECK(hub.subscribe(&small));
    CHECK(hub.subscribe(&big));
    CHECK(!hub.subscribe(&small));   // duplicate
    CHECK(!hub.subscribe(nullptr));
    for (uint32_t i = 1; i <= 5; ++i) {
        BaroSample b{};
        b.header.sequence = i;
        hub.publish(SensorMessage::fromBaro(b));
    }
    CHECK_EQ(hub.publishedCount(), 5u);
    CHECK_EQ(hub.dropsFor(0), 3u);
    CHECK_EQ(hub.dropsFor(1), 0u);
    CHECK_EQ(hub.totalDrops(), 3u);
    SensorMessage m{};
    CHECK(small.tryPop(m));
    CHECK_EQ(m.baro.header.sequence, 1u);  // oldest kept, newest dropped
    CHECK(small.tryPop(m));
    CHECK_EQ(m.baro.header.sequence, 2u);
    CHECK(!small.tryPop(m));
    CHECK_EQ(big.size(), 5u);
}

TEST(hub_subscriber_limit)
{
    SensorDataHub hub;
    BoundedSampleQueue<1> q[5];
    for (int i = 0; i < 4; ++i) CHECK(hub.subscribe(&q[i]));
    CHECK(!hub.subscribe(&q[4]));
}

TEST(sensor_message_tagging)
{
    LidarSample l{};
    l.distance_m = 3.5f;
    const SensorMessage m = SensorMessage::fromLidar(l);
    CHECK(m.type == SensorMessageType::Lidar);
    CHECK_NEAR(m.lidar.distance_m, 3.5, 1e-6);
}

TEST(imu_source_retries_init_at_interval)
{
    ImuRig r;
    r.dev.regs[0x0F] = 0x00;  // sensor not answering correctly yet
    ImuSource src("IMU0", r.imu, Lsm6dsrxConfig{}, 10u);  // default RetryPolicy: first retry after 1000 ms
    CollectingSink sink;
    src.service(0, sink);
    CHECK_EQ(src.initFailures(), 1u);
    CHECK(src.lastInitStatus() == Status::WrongDeviceId);
    src.service(500, sink);            // too early to retry
    CHECK_EQ(src.initFailures(), 1u);
    r.dev.regs[0x0F] = 0x6B;           // sensor appears
    src.service(1000, sink);           // retry succeeds and reads immediately
    CHECK_EQ(src.initFailures(), 1u);
    CHECK(r.imu.isInitialized());
    CHECK_EQ(sink.got.size(), 1u);
    CHECK(sink.got[0].type == SensorMessageType::Imu);
}

TEST(polled_source_respects_its_own_period)
{
    ImuRig r;
    ImuSource src("IMU0", r.imu, Lsm6dsrxConfig{}, 20u);  // 50 Hz source
    CollectingSink sink;
    for (uint32_t t = 0; t < 100; t += 10) src.service(t, sink);  // 100 Hz loop
    CHECK_EQ(sink.got.size(), 5u);  // t = 0, 20, 40, 60, 80
}

TEST(polled_source_does_not_burst_after_stall)
{
    ImuRig r;
    ImuSource src("IMU0", r.imu, Lsm6dsrxConfig{}, 10u);
    CollectingSink sink;
    src.service(0, sink);
    src.service(500, sink);  // loop stalled for 500 ms
    src.service(505, sink);  // must NOT replay 50 missed reads
    src.service(510, sink);
    CHECK_EQ(sink.got.size(), 3u);  // t = 0, 500, 510
}

TEST(polled_source_read_failure_not_published)
{
    ImuRig r;
    ImuSource src("IMU0", r.imu, Lsm6dsrxConfig{}, 10u);
    CollectingSink sink;
    src.service(0, sink);
    CHECK_EQ(sink.got.size(), 1u);
    r.dev.floatingBus = true;
    src.service(10, sink);
    CHECK_EQ(sink.got.size(), 1u);
    CHECK_EQ(src.health().invalidDataCount, 1u);
}

TEST(lidar_source_publishes_newest_frame_per_cycle)
{
    Tf02Pro lidar;
    FakeByteSource rx;
    LidarSource src("LIDAR", lidar, rx);
    CollectingSink sink;
    const std::vector<uint8_t> f1 = {0x59, 0x59, 0x64, 0x00, 0xE8, 0x03, 0xC8, 0x08, 0x00};  // 100 cm
    std::vector<uint8_t> a = f1;
    unsigned sum = 0;
    for (int i = 0; i < 8; ++i) sum += a[i];
    a[8] = static_cast<uint8_t>(sum);
    std::vector<uint8_t> b = a;
    b[2] = 0xC8;  // 200 cm
    sum = 0;
    for (int i = 0; i < 8; ++i) sum += b[i];
    b[8] = static_cast<uint8_t>(sum);
    rx.add(a);
    rx.add(b);
    src.service(0, sink);
    CHECK_EQ(sink.got.size(), 1u);
    CHECK_NEAR(sink.got[0].lidar.distance_m, 2.0, 1e-6);
    src.service(10, sink);  // no new bytes -> nothing published
    CHECK_EQ(sink.got.size(), 1u);
}

TEST(gnss_source_drains_bounded_amount_per_cycle)
{
    NeoM9n gnss;
    FakeByteSource rx;
    GnssSource src("GNSS", gnss, rx);
    CollectingSink sink;
    rx.add(std::vector<uint8_t>(2000, 0x00));  // a burst of junk bytes
    src.service(0, sink);
    CHECK_EQ(rx.pending.size(), 2000u - 512u);  // at most 8 x 64 bytes per cycle
    CHECK(sink.got.empty());
}
