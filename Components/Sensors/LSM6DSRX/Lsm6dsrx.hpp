/**
 ******************************************************************************
 * @file    Lsm6dsrx.hpp
 * @brief   Driver for the ST LSM6DSRX 6-axis IMU (accelerometer + gyroscope).
 *
 * One class, many instances: ParaGuardian has three IMUs, so the board code
 * creates three Lsm6dsrx objects, each with its own IRegisterBus (its own
 * chip-select or I2C address). Nothing is static or global, so the three
 * instances never share state.
 *
 * Typical use (inside a task, after the scheduler has started):
 *     pg::Lsm6dsrx imu(regBus, clock);
 *     if (pg::isOk(imu.init(config))) { imu.read(sample); }
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_LSM6DSRX_HPP
#define PARAGUARDIAN_SENSORS_LSM6DSRX_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/RegisterBus.hpp"
#include "Common/Status.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Output data rate codes (4-bit field ODR_XL / ODR_G). Values are the register codes.
enum class Lsm6dsrxOdr : uint8_t {
    PowerDown = 0x0,
    Hz12_5 = 0x1,
    Hz26 = 0x2,
    Hz52 = 0x3,
    Hz104 = 0x4,
    Hz208 = 0x5,
    Hz416 = 0x6,
    Hz833 = 0x7,
    Hz1666 = 0x8,
    Hz3333 = 0x9,
    Hz6666 = 0xA,
};

/// Accelerometer full scale. NOTE the non-monotonic register encoding (01 = 16 g).
enum class Lsm6dsrxAccelRange : uint8_t {
    G2 = 0x0,
    G16 = 0x1,
    G4 = 0x2,
    G8 = 0x3,
};

/// Gyroscope full scale (logical values; the driver converts to FS_G/FS_125/FS_4000 bits).
enum class Lsm6dsrxGyroRange : uint8_t {
    Dps125,
    Dps250,
    Dps500,
    Dps1000,
    Dps2000,
    Dps4000,
};

struct Lsm6dsrxConfig {
    Lsm6dsrxOdr accelOdr = Lsm6dsrxOdr::Hz416;
    Lsm6dsrxAccelRange accelRange = Lsm6dsrxAccelRange::G16;
    Lsm6dsrxOdr gyroOdr = Lsm6dsrxOdr::Hz416;
    Lsm6dsrxGyroRange gyroRange = Lsm6dsrxGyroRange::Dps2000;
};

/// Raw register values, useful for tests and debugging.
struct Lsm6dsrxRaw {
    int16_t temperature;
    int16_t gyro[3];
    int16_t accel[3];
    uint8_t status;
};

class Lsm6dsrx {
public:
    Lsm6dsrx(IRegisterBus& bus, IClock& clock, SensorId id = SensorId::Imu0)
        : bus_(bus), clock_(clock), id_(id) {}

    /**
     * Identify, reset, configure and verify the device.
     * @return Ok, WrongDeviceId, Timeout (reset never finished), VerifyFailed
     *         (register read-back mismatch) or a bus error.
     */
    Status init(const Lsm6dsrxConfig& config);

    /// Reads WHO_AM_I into @p value (does not require init()).
    Status readWhoAmI(uint8_t& value);

    /// Reads the STATUS register (data-ready bits).
    Status readStatus(uint8_t& status);

    /// Reads all 14 output bytes in one burst and returns them undecoded.
    Status readRaw(Lsm6dsrxRaw& raw);

    /**
     * Reads one complete sample and converts it to SI units.
     * Fills @p out even when data is not fresh (accelFresh/gyroFresh tell you).
     * @return Ok, NotInitialized, InvalidData (bus returned all 0x00 or all 0xFF),
     *         or a bus error. On error @p out.header.valid is false.
     */
    Status read(ImuSample& out);

    // ---- Unit conversion helpers (public + static so tests can check them) ----
    static float accelSensitivity_mg_per_lsb(Lsm6dsrxAccelRange range);
    static float gyroSensitivity_mdps_per_lsb(Lsm6dsrxGyroRange range);
    static float rawToTemperatureC(int16_t raw);
    static uint8_t encodeCtrl1Xl(Lsm6dsrxOdr odr, Lsm6dsrxAccelRange range);
    static uint8_t encodeCtrl2G(Lsm6dsrxOdr odr, Lsm6dsrxGyroRange range);

    const SensorHealth& health() const { return health_; }
    bool isInitialized() const { return health_.initialized; }
    SensorId id() const { return id_; }

    /// Reset/boot polling budget.
    static constexpr uint32_t kResetTimeoutMs = 50u;

private:
    Status track(Status s);

    IRegisterBus& bus_;
    IClock& clock_;
    SensorId id_;
    Lsm6dsrxConfig config_{};
    float accelScale_mps2_per_lsb_ = 0.0f;
    float gyroScale_radps_per_lsb_ = 0.0f;
    uint32_t sequence_ = 0u;
    SensorHealth health_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_LSM6DSRX_HPP
