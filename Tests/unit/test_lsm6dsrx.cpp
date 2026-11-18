/**
 * LSM6DSRX tests. Register addresses/values come from the datasheet:
 *   WHO_AM_I 0x0F = 0x6B; CTRL1_XL 0x10; CTRL2_G 0x11; CTRL3_C 0x12 (SW_RESET bit0,
 *   IF_INC bit2, BDU bit6); STATUS 0x1E; OUT_TEMP_L 0x20 .. OUTZ_H_A 0x2D.
 * Sensitivities: +-16 g = 0.488 mg/LSB, 2000 dps = 70 mdps/LSB, temp 256 LSB/degC @ 25 degC.
 */
#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"
#include "mocks/MockClock.hpp"

#include "Common/RegisterBus.hpp"
#include "Sensors/LSM6DSRX/Lsm6dsrx.hpp"

using namespace pg;

namespace {

/// Emulates the chip: WHO_AM_I, SW_RESET self-clearing after `resetReads` reads.
struct ImuFixture {
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi{dev};
    SpiRegisterBus bus{spi};
    MockClock clock;
    Lsm6dsrx imu{bus, clock, SensorId::Imu1};
    int resetReadsRemaining = 0;

    explicit ImuFixture(int resetReads = 2)
    {
        dev.regs[0x0F] = 0x6B;
        dev.regs[0x12] = 0x04;  // CTRL3_C reset value (IF_INC=1)
        dev.onWrite = [this, resetReads](uint8_t reg, uint8_t value) {
            if (reg == 0x12 && (value & 0x01)) resetReadsRemaining = resetReads;
        };
        dev.onRead = [this](uint8_t reg) {
            if (reg == 0x12 && resetReadsRemaining >= 0 && (dev.regs[0x12] & 0x01)) {
                if (resetReadsRemaining == 0) dev.regs[0x12] = 0x04;  // reset complete
                else --resetReadsRemaining;
            }
        };
    }

    void setOutputs(int16_t temp, int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az,
                    uint8_t status = 0x07)
    {
        const int16_t v[7] = {temp, gx, gy, gz, ax, ay, az};
        for (int i = 0; i < 7; ++i) {
            dev.regs[0x20 + 2 * i] = static_cast<uint8_t>(static_cast<uint16_t>(v[i]) & 0xFF);
            dev.regs[0x21 + 2 * i] = static_cast<uint8_t>(static_cast<uint16_t>(v[i]) >> 8);
        }
        dev.regs[0x1E] = status;
    }
};

}  // namespace

TEST(lsm6dsrx_encoding_matches_datasheet_bitfields)
{
    // 416 Hz = 0110, +-16 g = 01  ->  0110 01 00 = 0x64
    CHECK_EQ(Lsm6dsrx::encodeCtrl1Xl(Lsm6dsrxOdr::Hz416, Lsm6dsrxAccelRange::G16), 0x64);
    // 104 Hz = 0100, +-2 g = 00   ->  0x40
    CHECK_EQ(Lsm6dsrx::encodeCtrl1Xl(Lsm6dsrxOdr::Hz104, Lsm6dsrxAccelRange::G2), 0x40);
    // 416 Hz, 2000 dps (FS_G=11)  ->  0110 11 0 0 = 0x6C
    CHECK_EQ(Lsm6dsrx::encodeCtrl2G(Lsm6dsrxOdr::Hz416, Lsm6dsrxGyroRange::Dps2000), 0x6C);
    // 4000 dps uses FS_4000 (bit0); 125 dps uses FS_125 (bit1)
    CHECK_EQ(Lsm6dsrx::encodeCtrl2G(Lsm6dsrxOdr::Hz208, Lsm6dsrxGyroRange::Dps4000), 0x51);
    CHECK_EQ(Lsm6dsrx::encodeCtrl2G(Lsm6dsrxOdr::Hz208, Lsm6dsrxGyroRange::Dps125), 0x52);
}

TEST(lsm6dsrx_init_success_writes_and_verifies)
{
    ImuFixture f;
    CHECK(isOk(f.imu.init(Lsm6dsrxConfig{})));
    CHECK(f.imu.isInitialized());
    CHECK_EQ(f.dev.protocolErrors, 0);
    // Order: SW_RESET, then CTRL3 (BDU|IF_INC = 0x44), CTRL1_XL, CTRL2_G
    CHECK(f.dev.writes.size() >= 4u);
    CHECK_EQ(f.dev.writes[0].first, 0x12);
    CHECK_EQ(f.dev.writes[0].second, 0x01);
    CHECK_EQ(f.dev.lastWrite(0x12), 0x44);
    CHECK_EQ(f.dev.lastWrite(0x10), 0x64);
    CHECK_EQ(f.dev.lastWrite(0x11), 0x6C);
    CHECK(f.clock.totalDelayMs >= 1u);  // waited for the reset
}

TEST(lsm6dsrx_wrong_who_am_i_rejected_without_writes)
{
    ImuFixture f;
    f.dev.regs[0x0F] = 0x6C;  // e.g. a different ST part
    CHECK(f.imu.init(Lsm6dsrxConfig{}) == Status::WrongDeviceId);
    CHECK(f.dev.writes.empty());
    CHECK(!f.imu.isInitialized());
    ImuSample s{};
    CHECK(f.imu.read(s) == Status::NotInitialized);
    CHECK(!s.header.valid);
}

TEST(lsm6dsrx_reset_never_completes_times_out)
{
    ImuFixture f(1000000);  // reset bit never clears in time
    CHECK(f.imu.init(Lsm6dsrxConfig{}) == Status::Timeout);
    CHECK(f.clock.now >= Lsm6dsrx::kResetTimeoutMs);
    CHECK(f.clock.now <= Lsm6dsrx::kResetTimeoutMs + 2u);  // bounded wait
    CHECK_EQ(f.imu.health().timeoutCount, 1u);
}

TEST(lsm6dsrx_bus_error_during_init_propagates)
{
    ImuFixture f;
    f.dev.failAtTransaction = 2;  // the SW_RESET write fails
    CHECK(f.imu.init(Lsm6dsrxConfig{}) == Status::BusError);
    CHECK(!f.imu.isInitialized());
    CHECK_EQ(f.imu.health().busErrorCount, 1u);
}

TEST(lsm6dsrx_verify_failure_detected)
{
    ImuFixture f;
    auto chain = f.dev.onWrite;
    f.dev.onWrite = [&f, chain](uint8_t reg, uint8_t value) {
        chain(reg, value);
        if (reg == 0x10) f.dev.regs[0x10] = 0x00;  // device "ignores" CTRL1_XL
    };
    CHECK(f.imu.init(Lsm6dsrxConfig{}) == Status::VerifyFailed);
}

TEST(lsm6dsrx_read_converts_to_si_units)
{
    ImuFixture f;
    CHECK(isOk(f.imu.init(Lsm6dsrxConfig{})));  // +-16 g, 2000 dps
    // temp 512 -> 512/256 + 25 = 27 degC
    // gyro X 1000 -> 70 dps = 1.2217305 rad/s; gyro Y -1000 -> -1.2217305
    // accel Z 2048 -> 2048 * 0.488 mg = 0.999424 g = 9.8010014 m/s^2; accel X -2048 -> -9.8010014
    f.setOutputs(512, 1000, -1000, 0, -2048, 0, 2048);
    f.clock.now = 5000;
    ImuSample s{};
    CHECK(isOk(f.imu.read(s)));
    CHECK(s.header.valid);
    CHECK(s.header.id == SensorId::Imu1);
    CHECK_EQ(s.header.timestampMs, 5000u);
    CHECK_EQ(s.header.sequence, 1u);
    CHECK_NEAR(s.temperature_c, 27.0, 1e-6);
    CHECK_NEAR(s.gyro_radps.x, 1.2217305, 1e-5);
    CHECK_NEAR(s.gyro_radps.y, -1.2217305, 1e-5);
    CHECK_NEAR(s.accel_mps2.z, 9.8010014, 1e-4);
    CHECK_NEAR(s.accel_mps2.x, -9.8010014, 1e-4);
    CHECK(s.accelFresh && s.gyroFresh);
    CHECK(!s.accelSaturated && !s.gyroSaturated);
}

TEST(lsm6dsrx_negative_temperature_and_status_bits)
{
    ImuFixture f;
    CHECK(isOk(f.imu.init(Lsm6dsrxConfig{})));
    f.setOutputs(-256 * 30, 1, 1, 1, 1, 1, 1, 0x01);  // -30 degC offset -> -5 degC; only XLDA
    ImuSample s{};
    CHECK(isOk(f.imu.read(s)));
    CHECK_NEAR(s.temperature_c, -5.0, 1e-6);
    CHECK(s.accelFresh);
    CHECK(!s.gyroFresh);
}

TEST(lsm6dsrx_saturation_flags)
{
    ImuFixture f;
    CHECK(isOk(f.imu.init(Lsm6dsrxConfig{})));
    f.setOutputs(0, -32768, 0, 0, 32767, 0, 0);
    ImuSample s{};
    CHECK(isOk(f.imu.read(s)));
    CHECK(s.accelSaturated);
    CHECK(s.gyroSaturated);
}

TEST(lsm6dsrx_floating_bus_is_invalid_data)
{
    ImuFixture f;
    CHECK(isOk(f.imu.init(Lsm6dsrxConfig{})));
    f.dev.floatingBus = true;
    ImuSample s{};
    CHECK(f.imu.read(s) == Status::InvalidData);
    CHECK(!s.header.valid);
    CHECK_EQ(f.imu.health().invalidDataCount, 1u);
}

TEST(lsm6dsrx_three_instances_are_independent)
{
    ImuFixture a, b, c;
    Lsm6dsrxConfig ca{};
    Lsm6dsrxConfig cb{};
    cb.accelRange = Lsm6dsrxAccelRange::G2;  // 0.061 mg/LSB
    Lsm6dsrxConfig cc{};
    cc.gyroRange = Lsm6dsrxGyroRange::Dps250;
    CHECK(isOk(a.imu.init(ca)));
    CHECK(isOk(b.imu.init(cb)));
    c.dev.regs[0x0F] = 0x00;  // third IMU absent
    CHECK(c.imu.init(cc) == Status::WrongDeviceId);

    a.setOutputs(0, 0, 0, 0, 0, 0, 1000);
    b.setOutputs(0, 0, 0, 0, 0, 0, 1000);
    ImuSample sa{}, sb{};
    CHECK(isOk(a.imu.read(sa)));
    CHECK(isOk(b.imu.read(sb)));
    // 1000 LSB: a = 0.488 g = 4.7856 m/s^2, b = 0.061 g = 0.59821 m/s^2
    CHECK_NEAR(sa.accel_mps2.z, 1000 * 0.488e-3 * 9.80665, 1e-4);
    CHECK_NEAR(sb.accel_mps2.z, 1000 * 0.061e-3 * 9.80665, 1e-5);
    CHECK(!c.imu.isInitialized());
    CHECK(a.imu.isInitialized() && b.imu.isInitialized());
}

TEST(lsm6dsrx_works_over_i2c)
{
    FakeRegisterDevice dev;
    dev.regs[0x0F] = 0x6B;
    // Reset completes immediately on the next read.
    dev.onRead = [&dev](uint8_t reg) { if (reg == 0x12) dev.regs[0x12] &= static_cast<uint8_t>(~0x01); };
    FakeI2cBus i2c(dev, 0x6A);
    I2cRegisterBus bus(i2c, 0x6A);
    MockClock clock;
    Lsm6dsrx imu(bus, clock);
    CHECK(isOk(imu.init(Lsm6dsrxConfig{})));
    CHECK_EQ(dev.lastWrite(0x10), 0x64);
}
