/**
 ******************************************************************************
 * @file    Bmp585.hpp
 * @brief   Driver for the Bosch BMP585 barometric pressure sensor.
 *
 * Two instances are used (two barometers). Each has its own IRegisterBus and
 * its own configuration, so they can run at different rates / filters.
 *
 * Output: pressure in pascals and temperature in degrees Celsius. The BMP585
 * compensates internally (no calibration coefficients to read, unlike BMP280).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_BMP585_HPP
#define PARAGUARDIAN_SENSORS_BMP585_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/RegisterBus.hpp"
#include "Common/Status.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Oversampling: more samples averaged inside the chip = less noise, slower.
enum class Bmp585Oversampling : uint8_t { X1 = 0, X2, X4, X8, X16, X32, X64, X128 };

/// IIR low-pass filter coefficient (0 = bypass). Smooths pressure noise / gusts.
enum class Bmp585IirFilter : uint8_t { Bypass = 0, Coef1, Coef3, Coef7, Coef15, Coef31, Coef63, Coef127 };

enum class Bmp585PowerMode : uint8_t { Standby = 0, Normal = 1, Forced = 2, Continuous = 3 };

/// Output data rate register codes (5-bit). Only commonly useful values are named.
enum class Bmp585Odr : uint8_t {
    Hz240 = 0x00,
    Hz160 = 0x04,
    Hz120 = 0x08,
    Hz100 = 0x0A,
    Hz80 = 0x0C,
    Hz60 = 0x0E,
    Hz50 = 0x0F,
    Hz40 = 0x11,
    Hz30 = 0x13,
    Hz25 = 0x14,
    Hz20 = 0x15,
    Hz10 = 0x17,
    Hz5 = 0x18,
    Hz1 = 0x1C,
};

struct Bmp585Config {
    Bmp585Oversampling pressureOsr = Bmp585Oversampling::X16;
    Bmp585Oversampling temperatureOsr = Bmp585Oversampling::X2;
    Bmp585IirFilter pressureIir = Bmp585IirFilter::Coef3;
    Bmp585IirFilter temperatureIir = Bmp585IirFilter::Bypass;
    Bmp585Odr odr = Bmp585Odr::Hz50;
    Bmp585PowerMode mode = Bmp585PowerMode::Normal;
};

class Bmp585 {
public:
    Bmp585(IRegisterBus& bus, IClock& clock, SensorId id = SensorId::Baro0)
        : bus_(bus), clock_(clock), id_(id) {}

    /**
     * Soft reset, identify, power-up check, configure (in standby), verify,
     * then enter the configured power mode.
     * @return Ok, WrongDeviceId, NotReady (NVM not ready / NVM error / no POR flag),
     *         VerifyFailed (read-back mismatch or chip says ODR/OSR combination invalid),
     *         or a bus error.
     */
    Status init(const Bmp585Config& config);

    Status readChipId(uint8_t& id);

    /// Reads the latest compensated pressure and temperature.
    Status read(BaroSample& out);

    /**
     * Forced mode: starts one measurement and waits (polls) until the chip
     * returns to standby, then reads it. @p timeoutMs bounds the wait.
     */
    Status measureForced(BaroSample& out, uint32_t timeoutMs = 100u);

    // Pure conversion helpers (tested independently).
    static float rawToPressurePa(uint32_t raw24);
    static float rawToTemperatureC(int32_t raw24signed);
    static uint8_t encodeOsrConfig(Bmp585Oversampling p, Bmp585Oversampling t);
    static uint8_t encodeOdrConfig(Bmp585Odr odr, Bmp585PowerMode mode);
    static uint8_t encodeDspIir(Bmp585IirFilter p, Bmp585IirFilter t);

    const SensorHealth& health() const { return health_; }
    bool isInitialized() const { return health_.initialized; }
    SensorId id() const { return id_; }

private:
    Status setPowerMode(Bmp585PowerMode mode);
    Status track(Status s);

    IRegisterBus& bus_;
    IClock& clock_;
    SensorId id_;
    Bmp585Config config_{};
    uint32_t sequence_ = 0u;
    SensorHealth health_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_BMP585_HPP
