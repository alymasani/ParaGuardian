/**
 * UBX / NEO-M9N tests.
 * Independent references:
 *  - Published u-blox example frames whose checksums are well known:
 *      CFG-RATE 100 ms: B5 62 06 08 06 00 64 00 01 00 01 00 7A 12
 *      MON-VER poll   : B5 62 0A 04 00 00 0E 34
 *  - NAV-PVT field offsets from the u-blox M9 interface description, filled by
 *    hand below (the test writes bytes at offsets; it does not call the decoder
 *    to build its input).
 */
#include <vector>

#include "TestFramework.hpp"

#include "Sensors/NEOM9N/NeoM9n.hpp"
#include "Sensors/NEOM9N/UbxParser.hpp"

using namespace pg;

namespace {
const std::vector<uint8_t> kCfgRate = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,
                                       0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const std::vector<uint8_t> kMonVerPoll = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

/// Independent Fletcher-8 used only to finish hand-made test frames.
std::vector<uint8_t> frameWithChecksum(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f = {0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF),
                              static_cast<uint8_t>(payload.size() >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < f.size(); ++i) { a = static_cast<uint8_t>(a + f[i]); b = static_cast<uint8_t>(b + a); }
    f.push_back(a);
    f.push_back(b);
    return f;
}

void put32(std::vector<uint8_t>& p, size_t off, int32_t v)
{
    const uint32_t u = static_cast<uint32_t>(v);
    for (int i = 0; i < 4; ++i) p[off + i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
}

/// NAV-PVT payload: 51.0447331 N, -114.0718831 E, hMSL 1045.123 m, climbing 1.5 m/s.
std::vector<uint8_t> navPvtPayload(uint8_t fixType, uint8_t flags, uint16_t flags3 = 0)
{
    std::vector<uint8_t> p(92, 0);
    put32(p, 0, 345678000);       // iTOW
    p[4] = 0xEA; p[5] = 0x07;     // year 2026
    p[6] = 11; p[7] = 14; p[8] = 9; p[9] = 30; p[10] = 15;
    p[11] = 0x07;                 // validDate|validTime|fullyResolved
    p[20] = fixType;
    p[21] = flags;
    p[23] = 12;                   // numSV
    put32(p, 24, -1140718831);    // lon
    put32(p, 28, 510447331);      // lat
    put32(p, 32, 1028000);        // height above ellipsoid (mm)
    put32(p, 36, 1045123);        // hMSL (mm)
    put32(p, 40, 2500);           // hAcc
    put32(p, 44, 3800);           // vAcc
    put32(p, 48, 1200);           // velN
    put32(p, 52, -300);           // velE
    put32(p, 56, -1500);          // velD (negative = going UP)
    put32(p, 60, 1237);           // gSpeed
    put32(p, 68, 400);            // sAcc
    p[76] = 0x8C; p[77] = 0x00;   // pDOP 140 -> 1.40
    p[78] = static_cast<uint8_t>(flags3 & 0xFF);
    p[79] = static_cast<uint8_t>(flags3 >> 8);
    return p;
}

size_t feedAll(ubx::Parser& parser, const std::vector<uint8_t>& bytes, ubx::Frame& last, uint32_t now = 0)
{
    size_t frames = 0;
    for (uint8_t b : bytes) {
        if (parser.feed(b, now, last)) ++frames;
    }
    return frames;
}
}  // namespace

TEST(ubx_checksum_matches_published_frames)
{
    uint8_t a = 0, b = 0;
    ubx::checksum(&kCfgRate[2], kCfgRate.size() - 4, a, b);
    CHECK_EQ(a, 0x7A);
    CHECK_EQ(b, 0x12);
    ubx::checksum(&kMonVerPoll[2], 4, a, b);
    CHECK_EQ(a, 0x0E);
    CHECK_EQ(b, 0x34);
}

TEST(ubx_build_frame_reproduces_published_frame)
{
    const uint8_t payload[6] = {0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
    uint8_t out[32] = {};
    const size_t n = ubx::buildFrame(0x06, 0x08, payload, 6, out, sizeof(out));
    CHECK_EQ(n, kCfgRate.size());
    for (size_t i = 0; i < n; ++i) CHECK_EQ(out[i], kCfgRate[i]);
    CHECK_EQ(ubx::buildFrame(0x06, 0x08, payload, 6, out, 10), 0u);  // too small -> 0, no overflow
}

TEST(ubx_parser_skips_nmea_and_parses_frame)
{
    ubx::Parser parser;
    std::vector<uint8_t> stream;
    const char* nmea = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (const char* c = nmea; *c; ++c) stream.push_back(static_cast<uint8_t>(*c));
    stream.insert(stream.end(), kCfgRate.begin(), kCfgRate.end());
    ubx::Frame f{};
    CHECK_EQ(feedAll(parser, stream, f), 1u);
    CHECK_EQ(f.msgClass, 0x06);
    CHECK_EQ(f.msgId, 0x08);
    CHECK_EQ(f.length, 6);
    CHECK_EQ(f.payload[0], 0x64);
    CHECK(parser.stats().discardedBytes >= 60u);
}

TEST(ubx_parser_rejects_corrupt_checksum_then_recovers)
{
    ubx::Parser parser;
    std::vector<uint8_t> bad = kCfgRate;
    bad[7] ^= 0x01;  // flip a payload bit
    ubx::Frame f{};
    CHECK_EQ(feedAll(parser, bad, f), 0u);
    CHECK_EQ(parser.stats().checksumErrors, 1u);
    CHECK_EQ(feedAll(parser, kMonVerPoll, f), 1u);
    CHECK_EQ(f.msgClass, 0x0A);
}

TEST(ubx_parser_partial_frame_timeout_then_recovery)
{
    ubx::Parser parser;
    ubx::Frame f{};
    // First 5 bytes of a frame, then the stream stalls.
    for (size_t i = 0; i < 5; ++i) parser.feed(kCfgRate[i], 1000, f);
    CHECK(parser.inFrame());
    parser.checkTimeout(1050, 100);  // 50 ms: still waiting
    CHECK(parser.inFrame());
    parser.checkTimeout(1200, 100);  // 200 ms: abandon
    CHECK(!parser.inFrame());
    CHECK_EQ(parser.stats().timeoutResets, 1u);
    CHECK_EQ(feedAll(parser, kMonVerPoll, f, 1201), 1u);
}

TEST(ubx_parser_truncated_frame_recovers_within_two_frames)
{
    // A frame cut short is followed directly by complete frames (no stall):
    // the parser eats bytes of the next frame as payload, fails the checksum,
    // and must be back in sync for a later frame.
    ubx::Parser parser;
    std::vector<uint8_t> stream(kCfgRate.begin(), kCfgRate.begin() + 9);
    for (int i = 0; i < 3; ++i) stream.insert(stream.end(), kMonVerPoll.begin(), kMonVerPoll.end());
    ubx::Frame f{};
    const size_t frames = feedAll(parser, stream, f);
    CHECK(frames >= 1u);
    CHECK(parser.stats().checksumErrors >= 1u);
}

TEST(ubx_parser_oversize_frame_does_not_overflow)
{
    ubx::Parser parser;
    std::vector<uint8_t> payload(1000, 0xAA);  // > MAX_PAYLOAD (256)
    std::vector<uint8_t> big = frameWithChecksum(0x0A, 0x31, payload);
    ubx::Frame f{};
    CHECK_EQ(feedAll(parser, big, f), 0u);
    CHECK_EQ(parser.stats().oversizeFrames, 1u);
    CHECK_EQ(feedAll(parser, kMonVerPoll, f), 1u);  // still in sync
}

TEST(ubx_parser_zero_length_frame)
{
    ubx::Parser parser;
    ubx::Frame f{};
    CHECK_EQ(feedAll(parser, kMonVerPoll, f), 1u);
    CHECK_EQ(f.length, 0);
}

TEST(nav_pvt_decoded_to_si_units)
{
    NeoM9n gnss;
    const std::vector<uint8_t> frame = frameWithChecksum(0x01, 0x07, navPvtPayload(3, 0x01));
    CHECK_EQ(gnss.processBytes(frame.data(), frame.size(), 2000), 1u);
    const GnssSample& s = gnss.latest();
    CHECK(s.header.valid);
    CHECK(s.positionValid && s.heightValid);
    CHECK_EQ(s.latitude_1e7deg, 510447331);
    CHECK_EQ(s.longitude_1e7deg, -1140718831);
    CHECK_NEAR(toDegrees(s.latitude_1e7deg), 51.0447331, 1e-9);
    CHECK_NEAR(s.heightMsl_m, 1045.123, 1e-3);
    CHECK_NEAR(s.heightEllipsoid_m, 1028.0, 1e-3);
    CHECK_NEAR(s.verticalVelocity_mps, 1.5, 1e-6);  // velD -1500 mm/s -> +1.5 m/s up
    CHECK_NEAR(s.velocityNorth_mps, 1.2, 1e-6);
    CHECK_NEAR(s.velocityEast_mps, -0.3, 1e-6);
    CHECK_NEAR(s.horizontalAccuracy_m, 2.5, 1e-6);
    CHECK_NEAR(s.verticalAccuracy_m, 3.8, 1e-6);
    CHECK_NEAR(s.speedAccuracy_mps, 0.4, 1e-6);
    CHECK_NEAR(s.pDop, 1.40, 1e-5);
    CHECK_EQ(s.numSatellites, 12);
    CHECK_EQ(s.utcYear, 2026);
    CHECK_EQ(s.utcSecond, 15);
    CHECK(s.utcDateValid && s.utcTimeValid);
    CHECK(s.fixType == GnssFixType::Fix3D);
    CHECK_EQ(s.header.timestampMs, 2000u);
}

TEST(nav_pvt_validity_rules)
{
    NeoM9n gnss;
    // 2D fix: position valid, height NOT valid
    auto f2d = frameWithChecksum(0x01, 0x07, navPvtPayload(2, 0x01));
    gnss.processBytes(f2d.data(), f2d.size(), 0);
    CHECK(gnss.latest().positionValid);
    CHECK(!gnss.latest().heightValid);
    // 3D fix but gnssFixOK = 0: nothing valid
    auto fNotOk = frameWithChecksum(0x01, 0x07, navPvtPayload(3, 0x00));
    gnss.processBytes(fNotOk.data(), fNotOk.size(), 0);
    CHECK(!gnss.latest().positionValid);
    CHECK(!gnss.latest().header.valid);
    // invalidLlh flag set
    auto fLlh = frameWithChecksum(0x01, 0x07, navPvtPayload(3, 0x01, 0x0001));
    gnss.processBytes(fLlh.data(), fLlh.size(), 0);
    CHECK(!gnss.latest().positionValid);
    // time-only fix
    auto fTime = frameWithChecksum(0x01, 0x07, navPvtPayload(5, 0x01));
    gnss.processBytes(fTime.data(), fTime.size(), 0);
    CHECK(!gnss.latest().positionValid);
}

TEST(nav_pvt_wrong_length_rejected)
{
    NeoM9n gnss;
    std::vector<uint8_t> p = navPvtPayload(3, 0x01);
    p.resize(84);  // older protocol length
    auto f = frameWithChecksum(0x01, 0x07, p);
    CHECK_EQ(gnss.processBytes(f.data(), f.size(), 0), 0u);
    CHECK(!gnss.latest().header.valid);
    CHECK_EQ(gnss.health().invalidDataCount, 1u);
}

TEST(gnss_stale_solution_invalidated)
{
    NeoM9n gnss(SensorId::Gnss0, 2000u);
    auto f = frameWithChecksum(0x01, 0x07, navPvtPayload(3, 0x01));
    gnss.processBytes(f.data(), f.size(), 10000);
    gnss.poll(11999);
    CHECK(gnss.latest().header.valid);
    CHECK(gnss.hasFreshSolution(12000));
    gnss.poll(12001);
    CHECK(!gnss.latest().header.valid);
    CHECK(!gnss.hasFreshSolution(12001));
    CHECK_EQ(gnss.health().timeoutCount, 1u);
}

TEST(gnss_checksum_errors_counted_in_health)
{
    NeoM9n gnss;
    auto f = frameWithChecksum(0x01, 0x07, navPvtPayload(3, 0x01));
    f[30] ^= 0xFF;
    CHECK_EQ(gnss.processBytes(f.data(), f.size(), 0), 0u);
    CHECK_EQ(gnss.health().invalidDataCount, 1u);
    CHECK_EQ(gnss.parserStats().checksumErrors, 1u);
}

TEST(valset_single_key_matches_hand_built_frame)
{
    // payload: version 0, layers RAM(1), reserved 0 0, key 0x20910007 LE, value U1 = 1
    // checksum over 06 8A 09 00 00 01 00 00 07 00 91 20 01 computed independently: 0x53 0x48
    const ubx::ConfigItem item{ubx::KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1u};
    uint8_t out[64] = {};
    const size_t n = ubx::buildValSet(&item, 1, ubx::VALSET_LAYER_RAM, out, sizeof(out));
    const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00,
                                0x07, 0x00, 0x91, 0x20, 0x01, 0x53, 0x48};
    CHECK_EQ(n, sizeof(expected));
    for (size_t i = 0; i < n; ++i) CHECK_EQ(out[i], expected[i]);
}

TEST(valset_value_sizes_from_key_ids)
{
    CHECK_EQ(ubx::keyValueSize(ubx::KEY_CFG_UART1OUTPROT_UBX), 1u);  // L
    CHECK_EQ(ubx::keyValueSize(ubx::KEY_CFG_RATE_MEAS), 2u);         // U2
    CHECK_EQ(ubx::keyValueSize(0x40520001u), 4u);                    // U4 (baud rate key)
    CHECK_EQ(ubx::keyValueSize(0x50000000u), 0u);                    // 8-byte: unsupported here
    uint8_t out[64] = {};
    CHECK_EQ(NeoM9n::buildEnableNavPvtFrame(20u, true, out, sizeof(out)), 0u);  // > 25 Hz rejected
    const size_t n = NeoM9n::buildEnableNavPvtFrame(100u, true, out, sizeof(out));
    // payload = 4 + (4+1) + (4+1) + (4+1) + (4+2) = 25 bytes -> frame 33
    CHECK_EQ(n, 33u);
    // Round-trip through the parser proves framing + checksum are self-consistent.
    ubx::Parser parser;
    ubx::Frame f{};
    size_t frames = 0;
    for (size_t i = 0; i < n; ++i) frames += parser.feed(out[i], 0, f) ? 1u : 0u;
    CHECK_EQ(frames, 1u);
    CHECK_EQ(f.payload[f.length - 2], 100);  // CFG-RATE-MEAS = 100 ms, little-endian
    CHECK_EQ(f.payload[f.length - 1], 0);
}

TEST(gnss_ack_and_nak_tracked)
{
    NeoM9n gnss;
    auto ack = frameWithChecksum(0x05, 0x01, {0x06, 0x8A});
    gnss.processBytes(ack.data(), ack.size(), 0);
    CHECK(gnss.lastConfigAck() == GnssConfigAck::Ack);
    auto nak = frameWithChecksum(0x05, 0x00, {0x06, 0x8A});
    gnss.processBytes(nak.data(), nak.size(), 0);
    CHECK(gnss.lastConfigAck() == GnssConfigAck::Nak);
}
