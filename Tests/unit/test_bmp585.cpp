/**
 * BMP585 tests. Register map / constants from the Bosch BMP585 datasheet and
 * BMP5 Sensor API: CHIP_ID 0x01 = 0x51, STATUS 0x28 (nvm_rdy bit1, nvm_err bit2),
 * INT_STATUS 0x27 (POR bit4), DSP_CONFIG 0x30, DSP_IIR 0x31, OSR_CONFIG 0x36,
 * ODR_CONFIG 0x37, OSR_EFF 0x38 (odr_is_valid bit7), CMD 0x7E (soft reset 0xB6).
 * Conversions: P[Pa] = raw/64, T[degC] = raw(signed 24)/65536.
 */
#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"
#include "mocks/MockClock.hpp"

#include "Common/RegisterBus.hpp"
#include "Sensors/BMP585/Bmp585.hpp"

using namespace pg;

namespace {
struct BaroFixture {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    MockClock clock;
    Bmp585 baro{bus, clock, SensorId::Baro1};

    BaroFixture()
    {
        dev.regs[0x01] = 0x51;  // BMP585
        dev.regs[0x28] = 0x02;  // nvm_rdy
        dev.regs[0x27] = 0x10;  // POR complete
        dev.regs[0x38] = 0x80;  // odr_is_valid
    }
    void setData(uint32_t tempRaw24, uint32_t pressRaw24)
    {
        dev.regs[0x1D] = tempRaw24 & 0xFF;
        dev.regs[0x1E] = (tempRaw24 >> 8) & 0xFF;
        dev.regs[0x1F] = (tempRaw24 >> 16) & 0xFF;
        dev.regs[0x20] = pressRaw24 & 0xFF;
        dev.regs[0x21] = (pressRaw24 >> 8) & 0xFF;
        dev.regs[0x22] = (pressRaw24 >> 16) & 0xFF;
    }
};
}  // namespace

TEST(bmp585_register_encodings)
{
    // OSR: press_en(0x40) | X16(4)<<3 (0x20) | X2(1) = 0x61
    CHECK_EQ(Bmp585::encodeOsrConfig(Bmp585Oversampling::X16, Bmp585Oversampling::X2), 0x61);
    // ODR: deep_dis(0x80) | 50 Hz code 0x0F << 2 (0x3C) | normal(1) = 0xBD
    CHECK_EQ(Bmp585::encodeOdrConfig(Bmp585Odr::Hz50, Bmp585PowerMode::Normal), 0xBD);
    CHECK_EQ(Bmp585::encodeOdrConfig(Bmp585Odr::Hz50, Bmp585PowerMode::Standby), 0xBC);
    // IIR: press coef3(2)<<3 | temp bypass(0) = 0x10
    CHECK_EQ(Bmp585::encodeDspIir(Bmp585IirFilter::Coef3, Bmp585IirFilter::Bypass), 0x10);
}

TEST(bmp585_init_sequence)
{
    BaroFixture f;
    CHECK(isOk(f.baro.init(Bmp585Config{})));
    CHECK(f.baro.isInitialized());
    CHECK_EQ(f.dev.protocolErrors, 0);
    // First write must be the soft reset command, followed by >= 2 ms wait.
    CHECK_EQ(f.dev.writes[0].first, 0x7E);
    CHECK_EQ(f.dev.writes[0].second, 0xB6);
    CHECK(f.clock.totalDelayMs >= 2u);
    // Standby before configuration, then normal mode last.
    CHECK_EQ(f.dev.writes[1].first, 0x37);
    CHECK_EQ(f.dev.writes[1].second, 0xBC);
    CHECK_EQ(f.dev.lastWrite(0x30), 0x2C);  // shdw_sel_iir_t|shdw_sel_iir_p|iir_flush_forced_en
    CHECK_EQ(f.dev.lastWrite(0x31), 0x10);
    CHECK_EQ(f.dev.lastWrite(0x36), 0x61);
    CHECK_EQ(f.dev.writes.back().first, 0x37);
    CHECK_EQ(f.dev.writes.back().second, 0xBD);
}

TEST(bmp585_rejects_bmp581_chip_id)
{
    BaroFixture f;
    f.dev.regs[0x01] = 0x50;
    CHECK(f.baro.init(Bmp585Config{}) == Status::WrongDeviceId);
    CHECK(!f.baro.isInitialized());
}

TEST(bmp585_nvm_error_and_missing_por_are_not_ready)
{
    BaroFixture a;
    a.dev.regs[0x28] = 0x02 | 0x04;  // nvm_rdy + nvm_err
    CHECK(a.baro.init(Bmp585Config{}) == Status::NotReady);
    BaroFixture b;
    b.dev.regs[0x27] = 0x00;  // no POR flag
    CHECK(b.baro.init(Bmp585Config{}) == Status::NotReady);
}

TEST(bmp585_invalid_osr_odr_combination_detected)
{
    BaroFixture f;
    f.dev.regs[0x38] = 0x00;  // chip says ODR not achievable with this OSR
    CHECK(f.baro.init(Bmp585Config{}) == Status::VerifyFailed);
}

TEST(bmp585_converts_pressure_and_temperature)
{
    BaroFixture f;
    CHECK(isOk(f.baro.init(Bmp585Config{})));
    // 101325 Pa * 64 = 6484800 = 0x62F340 ; 25.5 degC * 65536 = 0x198000
    f.setData(0x198000, 0x62F340);
    f.clock.now = 777;
    BaroSample s{};
    CHECK(isOk(f.baro.read(s)));
    CHECK_NEAR(s.pressure_pa, 101325.0, 0.01);
    CHECK_NEAR(s.temperature_c, 25.5, 1e-6);
    CHECK(s.pressureInRange && s.header.valid);
    CHECK_EQ(s.header.timestampMs, 777u);
    CHECK(s.header.id == SensorId::Baro1);
}

TEST(bmp585_negative_temperature_sign_extended)
{
    BaroFixture f;
    CHECK(isOk(f.baro.init(Bmp585Config{})));
    // -10.25 degC * 65536 = -671744 -> 24-bit two's complement 0xF5C000
    f.setData(0xF5C000, 0x62F340);
    BaroSample s{};
    CHECK(isOk(f.baro.read(s)));
    CHECK_NEAR(s.temperature_c, -10.25, 1e-6);
}

TEST(bmp585_zero_pressure_not_ready_and_out_of_range)
{
    BaroFixture f;
    CHECK(isOk(f.baro.init(Bmp585Config{})));
    f.setData(0x198000, 0);
    BaroSample s{};
    CHECK(f.baro.read(s) == Status::NotReady);
    CHECK(!s.header.valid);
    f.setData(0x198000, 0x138800);  // 20000 Pa, below 300 hPa operating range
    CHECK(f.baro.read(s) == Status::OutOfRange);
    CHECK(!s.header.valid);
    CHECK_NEAR(s.pressure_pa, 20000.0, 0.01);
}

TEST(bmp585_forced_measurement_waits_for_standby)
{
    BaroFixture f;
    Bmp585Config cfg{};
    cfg.mode = Bmp585PowerMode::Forced;
    CHECK(isOk(f.baro.init(cfg)));
    int readsUntilDone = 3;
    f.dev.onRead = [&](uint8_t reg) {
        if (reg == 0x37 && (f.dev.regs[0x37] & 0x03) == 0x02 && --readsUntilDone == 0) {
            f.dev.regs[0x37] = static_cast<uint8_t>(f.dev.regs[0x37] & ~0x03);  // back to standby
        }
    };
    f.setData(0x198000, 0x62F340);
    BaroSample s{};
    CHECK(isOk(f.baro.measureForced(s)));
    CHECK_NEAR(s.pressure_pa, 101325.0, 0.01);

    // Chip never returns to standby -> bounded timeout.
    f.dev.onRead = nullptr;
    CHECK(f.baro.measureForced(s, 20u) == Status::Timeout);
}

TEST(bmp585_two_instances_independent)
{
    BaroFixture a, b;
    b.dev.regs[0x01] = 0x00;  // second sensor not answering correctly
    CHECK(isOk(a.baro.init(Bmp585Config{})));
    CHECK(b.baro.init(Bmp585Config{}) == Status::WrongDeviceId);
    CHECK(a.baro.isInitialized());
    CHECK(!b.baro.isInitialized());
}
