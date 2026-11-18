/**
 * TF02-Pro tests. Frame: 59 59 DistL DistH StrL StrH TempL TempH CS,
 * CS = low byte of the sum of the first 8 bytes. Temp degC = raw/8 - 256.
 * Hand-computed frame: distance 1234 cm, strength 1000, temperature 25 degC
 *   raw temp = (25 + 256) * 8 = 2248 = 0x08C8
 *   sum = 59+59+D2+04+E8+03+C8+08 = 835 = 0x343 -> CS = 0x43
 */
#include <vector>

#include "TestFramework.hpp"

#include "Sensors/TF02Pro/Tf02Pro.hpp"

using namespace pg;

namespace {
const std::vector<uint8_t> kFrame = {0x59, 0x59, 0xD2, 0x04, 0xE8, 0x03, 0xC8, 0x08, 0x43};

std::vector<uint8_t> makeFrame(uint16_t dist, uint16_t strength, uint16_t tempRaw)
{
    std::vector<uint8_t> f = {0x59, 0x59, static_cast<uint8_t>(dist & 0xFF), static_cast<uint8_t>(dist >> 8),
                              static_cast<uint8_t>(strength & 0xFF), static_cast<uint8_t>(strength >> 8),
                              static_cast<uint8_t>(tempRaw & 0xFF), static_cast<uint8_t>(tempRaw >> 8)};
    unsigned sum = 0;
    for (uint8_t b : f) sum += b;
    f.push_back(static_cast<uint8_t>(sum & 0xFF));
    return f;
}
}  // namespace

TEST(tf02_valid_frame_decoded)
{
    Tf02Pro lidar;
    CHECK_EQ(lidar.processBytes(kFrame.data(), kFrame.size(), 42), 1u);
    const LidarSample& s = lidar.latest();
    CHECK_NEAR(s.distance_m, 12.34, 1e-5);
    CHECK_EQ(s.signalStrength, 1000);
    CHECK_NEAR(s.temperature_c, 25.0, 1e-6);
    CHECK(s.header.valid && !s.weakSignal && !s.outOfRange);
    CHECK_EQ(s.header.timestampMs, 42u);
}

TEST(tf02_millimetre_unit)
{
    Tf02ProConfig cfg;
    cfg.unit = Tf02DistanceUnit::Millimetre;
    Tf02Pro lidar(cfg);
    lidar.processBytes(kFrame.data(), kFrame.size(), 0);
    CHECK_NEAR(lidar.latest().distance_m, 1.234, 1e-6);
}

TEST(tf02_bad_checksum_rejected)
{
    Tf02Pro lidar;
    std::vector<uint8_t> bad = kFrame;
    bad[8] = 0x44;
    CHECK_EQ(lidar.processBytes(bad.data(), bad.size(), 0), 0u);
    CHECK_EQ(lidar.parserStats().checksumErrors, 1u);
    CHECK(!lidar.latest().header.valid);
}

TEST(tf02_garbage_then_valid_frame)
{
    Tf02Pro lidar;
    std::vector<uint8_t> s = {0x00, 0x13, 0x59, 0x07, 0xFF};  // noise incl. a lone 0x59
    s.insert(s.end(), kFrame.begin(), kFrame.end());
    CHECK_EQ(lidar.processBytes(s.data(), s.size(), 0), 1u);
    CHECK_NEAR(lidar.latest().distance_m, 12.34, 1e-5);
}

TEST(tf02_resync_inside_corrupted_window)
{
    // A truncated frame (3 bytes) is immediately followed by a real frame.
    // The 9-byte window [59 59 12 | 59 59 D2 04 E8 03] fails its checksum and
    // the parser must find the real header at offset 3 without losing the frame.
    Tf02Pro lidar;
    std::vector<uint8_t> s = {0x59, 0x59, 0x12};
    s.insert(s.end(), kFrame.begin(), kFrame.end());
    CHECK_EQ(lidar.processBytes(s.data(), s.size(), 0), 1u);
    CHECK_NEAR(lidar.latest().distance_m, 12.34, 1e-5);
    CHECK_EQ(lidar.parserStats().checksumErrors, 1u);
}

TEST(tf02_frame_split_across_calls)
{
    Tf02Pro lidar;
    CHECK_EQ(lidar.processBytes(kFrame.data(), 4, 0), 0u);
    CHECK_EQ(lidar.processBytes(kFrame.data() + 4, 5, 0), 1u);
}

TEST(tf02_weak_and_overexposed_signal_invalid)
{
    Tf02Pro lidar;
    auto weak = makeFrame(500, 59, 2248);
    lidar.processBytes(weak.data(), weak.size(), 0);
    CHECK(lidar.latest().weakSignal);
    CHECK(!lidar.latest().header.valid);
    auto ok = makeFrame(500, 60, 2248);  // threshold is inclusive at 60
    lidar.processBytes(ok.data(), ok.size(), 0);
    CHECK(lidar.latest().header.valid);
    auto over = makeFrame(500, 65535, 2248);
    lidar.processBytes(over.data(), over.size(), 0);
    CHECK(lidar.latest().weakSignal);
}

TEST(tf02_out_of_range)
{
    Tf02Pro lidar;
    auto far = makeFrame(4500, 1000, 2248);  // 45 m > 40 m
    lidar.processBytes(far.data(), far.size(), 0);
    CHECK(lidar.latest().outOfRange);
    CHECK(!lidar.latest().header.valid);
}

TEST(tf02_stale_timeout)
{
    Tf02Pro lidar;
    lidar.processBytes(kFrame.data(), kFrame.size(), 1000);
    lidar.poll(1100);
    CHECK(lidar.latest().header.valid);
    lidar.poll(1101);
    CHECK(!lidar.latest().header.valid);
    CHECK_EQ(lidar.health().timeoutCount, 1u);
}

TEST(tf02_many_frames_in_one_buffer)
{
    Tf02Pro lidar;
    std::vector<uint8_t> s;
    for (int i = 0; i < 10; ++i) s.insert(s.end(), kFrame.begin(), kFrame.end());
    CHECK_EQ(lidar.processBytes(s.data(), s.size(), 0), 10u);
    CHECK_EQ(lidar.latest().header.sequence, 10u);
}
