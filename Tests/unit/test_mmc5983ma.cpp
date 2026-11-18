/**
 * MMC5983MA tests. From the datasheet: PRODUCT_ID 0x2F = 0x30; 18-bit output,
 * null field = 131072 counts, 16384 counts/G; CTRL0 0x09 bits TM_M=0x01,
 * TM_T=0x02, SET=0x08, RESET=0x10, Auto_SR_en=0x20; CTRL1 0x0A SW_RST=0x80;
 * STATUS 0x08 Meas_M_Done=0x01, Meas_T_Done=0x02; Tout: -75 degC + 0.8 degC/LSB.
 */
#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"
#include "mocks/MockClock.hpp"

#include "Common/RegisterBus.hpp"
#include "Sensors/MMC5983MA/Mmc5983ma.hpp"

using namespace pg;

namespace {
void encode18(uint8_t regs[7], uint32_t x, uint32_t y, uint32_t z)
{
    regs[0] = static_cast<uint8_t>(x >> 10);
    regs[1] = static_cast<uint8_t>((x >> 2) & 0xFF);
    regs[2] = static_cast<uint8_t>(y >> 10);
    regs[3] = static_cast<uint8_t>((y >> 2) & 0xFF);
    regs[4] = static_cast<uint8_t>(z >> 10);
    regs[5] = static_cast<uint8_t>((z >> 2) & 0xFF);
    regs[6] = static_cast<uint8_t>(((x & 3) << 6) | ((y & 3) << 4) | ((z & 3) << 2));
}

/// Fake chip: measurement completes as soon as TM_M/TM_T is written. Output
/// depends on whether the last SET or RESET pulse was applied:
///   after SET:   reading = null + H + offset
///   after RESET: reading = null - H + offset
struct MagFixture {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    MockClock clock;
    Mmc5983ma mag{bus, clock};
    int32_t H[3] = {0, 0, 0};
    int32_t offset[3] = {0, 0, 0};
    bool setState = true;
    bool neverDone = false;

    MagFixture()
    {
        dev.regs[0x2F] = 0x30;
        dev.onWrite = [this](uint8_t reg, uint8_t v) {
            if (reg == 0x09) {
                if (v & 0x08) setState = true;
                if (v & 0x10) setState = false;
                if ((v & 0x01) && !neverDone) {
                    const int sign = setState ? 1 : -1;
                    uint8_t r[7];
                    encode18(r, 131072u + sign * H[0] + offset[0], 131072u + sign * H[1] + offset[1],
                             131072u + sign * H[2] + offset[2]);
                    for (int i = 0; i < 7; ++i) dev.regs[i] = r[i];
                    dev.regs[0x08] |= 0x01;
                }
                if ((v & 0x02) && !neverDone) dev.regs[0x08] |= 0x02;
            }
            if (reg == 0x08) dev.regs[0x08] = static_cast<uint8_t>(dev.regs[0x08] & ~v);  // write-1-to-clear
        };
    }
};
}  // namespace

TEST(mmc5983ma_decode_18bit_hand_computed)
{
    // x = 147456 = 131072 + 16384 (+1 G): X0=0x90, X1=0x00, low bits 00
    // y = 122880 = 131072 - 8192 (-0.5 G): Y0=0x78, Y1=0x00
    // z = 131073 (+1 count): Z0=0x80, Z1=0x00, Z[1:0]=01 -> XYZ2 = 0b0000'0100
    const uint8_t regs[7] = {0x90, 0x00, 0x78, 0x00, 0x80, 0x00, 0x04};
    const Mmc5983maRaw raw = Mmc5983ma::decode18Bit(regs);
    CHECK_EQ(raw.x, 147456u);
    CHECK_EQ(raw.y, 122880u);
    CHECK_EQ(raw.z, 131073u);
    CHECK_NEAR(Mmc5983ma::countsToMicroTesla(raw.x), 100.0, 1e-4);   // 1 G = 100 uT
    CHECK_NEAR(Mmc5983ma::countsToMicroTesla(raw.y), -50.0, 1e-4);
    CHECK_NEAR(Mmc5983ma::countsToMicroTesla(raw.z), 100.0 / 16384.0, 1e-6);
}

TEST(mmc5983ma_low_bits_of_each_axis_are_separated)
{
    // XYZ2 = 11 10 01 00 -> X low=3, Y low=2, Z low=1
    const uint8_t regs[7] = {0, 0, 0, 0, 0, 0, 0xE4};
    const Mmc5983maRaw raw = Mmc5983ma::decode18Bit(regs);
    CHECK_EQ(raw.x, 3u);
    CHECK_EQ(raw.y, 2u);
    CHECK_EQ(raw.z, 1u);
}

TEST(mmc5983ma_init_sequence)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    CHECK_EQ(f.dev.writes[0].first, 0x0A);   // SW_RST first
    CHECK_EQ(f.dev.writes[0].second, 0x80);
    CHECK(f.clock.totalDelayMs >= 10u);      // power-on time
    CHECK_EQ(f.dev.writes[1].second, 0x00);  // CTRL1 bandwidth 100 Hz
    CHECK_EQ(f.dev.writes[2].first, 0x09);   // SET pulse with Auto_SR kept
    CHECK_EQ(f.dev.writes[2].second, 0x28);
    CHECK_EQ(f.dev.lastWrite(0x09), 0x20);
}

TEST(mmc5983ma_wrong_product_id)
{
    MagFixture f;
    f.dev.regs[0x2F] = 0x00;
    CHECK(f.mag.init(Mmc5983maConfig{}) == Status::WrongDeviceId);
}

TEST(mmc5983ma_rejects_invalid_continuous_combination)
{
    MagFixture f;
    Mmc5983maConfig cfg{};
    cfg.continuousRate = Mmc5983maContinuousRate::Hz1000;  // needs BW 800 Hz
    CHECK(f.mag.init(cfg) == Status::InvalidArgument);
}

TEST(mmc5983ma_measure_converts)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    f.H[0] = 8192;    // +0.5 G = +50 uT
    f.H[1] = -16384;  // -1 G
    f.H[2] = 0;
    MagSample s{};
    CHECK(isOk(f.mag.measure(s)));
    CHECK_NEAR(s.field_uT.x, 50.0, 1e-3);
    CHECK_NEAR(s.field_uT.y, -100.0, 1e-3);
    CHECK_NEAR(s.field_uT.z, 0.0, 1e-3);
    CHECK(s.header.valid && !s.saturated && !s.offsetCancelled);
}

TEST(mmc5983ma_measurement_timeout)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    f.neverDone = true;
    const uint32_t start = f.clock.now;
    MagSample s{};
    CHECK(f.mag.measure(s) == Status::Timeout);
    CHECK(f.clock.now - start <= 20u);  // bounded by the bandwidth-based timeout
    CHECK(!s.header.valid);
}

TEST(mmc5983ma_saturation_marks_invalid)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    f.H[0] = -131072;  // reading 0 counts = negative full scale
    MagSample s{};
    CHECK(isOk(f.mag.measure(s)));
    CHECK(s.saturated);
    CHECK(!s.header.valid);
}

TEST(mmc5983ma_set_reset_removes_offset)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    f.H[0] = 4096;      // +0.25 G = 25 uT true field
    f.H[1] = -2048;     // -12.5 uT
    f.H[2] = 0;
    f.offset[0] = 1000; // large bridge offsets that a single reading would include
    f.offset[1] = -700;
    f.offset[2] = 300;
    MagSample single{};
    CHECK(isOk(f.mag.measure(single)));
    CHECK(std::fabs(single.field_uT.x - 25.0) > 5.0);  // offset visible in a plain reading
    MagSample s{};
    CHECK(isOk(f.mag.measureOffsetCancelled(s)));
    CHECK(s.offsetCancelled);
    CHECK_NEAR(s.field_uT.x, 25.0, 1e-3);
    CHECK_NEAR(s.field_uT.y, -12.5, 1e-3);
    CHECK_NEAR(s.field_uT.z, 0.0, 1e-3);
    // Auto_SR restored afterwards
    CHECK_EQ(f.dev.lastWrite(0x09) & 0x20, 0x20);
}

TEST(mmc5983ma_temperature)
{
    MagFixture f;
    CHECK(isOk(f.mag.init(Mmc5983maConfig{})));
    f.dev.regs[0x07] = 125;  // -75 + 0.8*125 = 25 degC
    float t = 0;
    CHECK(isOk(f.mag.measureTemperature(t)));
    CHECK_NEAR(t, 25.0, 1e-4);
}
