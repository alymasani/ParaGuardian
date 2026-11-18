/**
 ******************************************************************************
 * @file    Mmc5983ma.hpp
 * @brief   Driver for the MEMSIC MMC5983MA 3-axis magnetometer.
 *
 * Source: MEMSIC MMC5983MA datasheet Rev A (register map section 8, "Internal
 * Control 0..3", 18-bit output format, SET/RESET description).
 *
 * IMPORTANT DEVICE QUIRK: Internal Control registers 0..3 are WRITE-ONLY.
 * Reading them back returns 0, so configuration cannot be verified by
 * read-back. The driver keeps a "shadow" copy of the bits that must persist
 * (e.g. Auto_SR_en) and includes them in every write to Internal Control 0.
 *
 * Output: microtesla in the sensor's own axes. 1 gauss = 100 uT.
 * Hard-/soft-iron calibration (environment dependent) is NOT done here.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_MMC5983MA_HPP
#define PARAGUARDIAN_SENSORS_MMC5983MA_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/RegisterBus.hpp"
#include "Common/Status.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

namespace mmc5983ma {
constexpr uint8_t REG_XOUT0 = 0x00;       // X[17:10]; 0x00..0x06 = X0,X1,Y0,Y1,Z0,Z1,XYZ2
constexpr uint8_t REG_TOUT = 0x07;        // temperature, unsigned 8-bit
constexpr uint8_t REG_STATUS = 0x08;      // Meas_M_Done[0] Meas_T_Done[1] OTP_Read_Done[4]
constexpr uint8_t REG_CTRL0 = 0x09;       // TM_M[0] TM_T[1] INT_meas_done_en[2] SET[3] RESET[4] Auto_SR_en[5] OTP_Read[6]
constexpr uint8_t REG_CTRL1 = 0x0A;       // BW[1:0] X_inhibit[2] YZ_inhibit[4:3] SW_RST[7]
constexpr uint8_t REG_CTRL2 = 0x0B;       // Cm_freq[2:0] Cmm_en[3] Prd_set[6:4] En_prd_set[7]
constexpr uint8_t REG_CTRL3 = 0x0C;
constexpr uint8_t REG_PRODUCT_ID = 0x2F;

constexpr uint8_t PRODUCT_ID_VALUE = 0x30;

constexpr uint8_t STATUS_MEAS_M_DONE = 1u << 0;
constexpr uint8_t STATUS_MEAS_T_DONE = 1u << 1;

constexpr uint8_t CTRL0_TM_M = 1u << 0;
constexpr uint8_t CTRL0_TM_T = 1u << 1;
constexpr uint8_t CTRL0_SET = 1u << 3;
constexpr uint8_t CTRL0_RESET = 1u << 4;
constexpr uint8_t CTRL0_AUTO_SR_EN = 1u << 5;

constexpr uint8_t CTRL1_SW_RST = 1u << 7;
constexpr uint8_t CTRL2_CMM_EN = 1u << 3;

/// 18-bit output: zero field = 2^17 = 131072 counts, 16384 counts per gauss (+-8 G range).
constexpr uint32_t NULL_FIELD_18BIT = 131072u;
constexpr float COUNTS_PER_GAUSS_18BIT = 16384.0f;
constexpr uint32_t MAX_18BIT = 262143u;

/// Temperature: unsigned, 0 = -75 degC, 0.8 degC per LSB (approximate, uncalibrated).
constexpr float TEMP_OFFSET_DEGC = -75.0f;
constexpr float TEMP_DEGC_PER_LSB = 0.8f;

constexpr uint32_t POWER_ON_TIME_MS = 10u;  // after power-up or SW_RST
constexpr uint32_t SET_RESET_TIME_MS = 1u;  // generous; datasheet pulse is ~500 ns
}  // namespace mmc5983ma

/// Filter bandwidth = measurement time. Lower bandwidth = less noise, longer measurement.
enum class Mmc5983maBandwidth : uint8_t {
    Hz100 = 0x0,  // 8 ms measurement
    Hz200 = 0x1,  // 4 ms
    Hz400 = 0x2,  // 2 ms
    Hz800 = 0x3,  // 0.5 ms
};

/// Continuous-measurement frequency code (Cm_freq). Off = on-demand only.
enum class Mmc5983maContinuousRate : uint8_t {
    Off = 0x0,
    Hz1 = 0x1,
    Hz10 = 0x2,
    Hz20 = 0x3,
    Hz50 = 0x4,
    Hz100 = 0x5,
    Hz200 = 0x6,   // requires BW = 200 Hz or higher
    Hz1000 = 0x7,  // requires BW = 800 Hz
};

struct Mmc5983maConfig {
    Mmc5983maBandwidth bandwidth = Mmc5983maBandwidth::Hz100;
    Mmc5983maContinuousRate continuousRate = Mmc5983maContinuousRate::Off;
    bool autoSetReset = true;  ///< Chip pulses SET before each measurement (reduces offset drift).
};

struct Mmc5983maRaw {
    uint32_t x;  ///< 18-bit unsigned counts
    uint32_t y;
    uint32_t z;
};

class Mmc5983ma {
public:
    Mmc5983ma(IRegisterBus& bus, IClock& clock, SensorId id = SensorId::Mag0)
        : bus_(bus), clock_(clock), id_(id) {}

    /// Software reset, identify (PRODUCT_ID = 0x30), configure, initial SET pulse.
    Status init(const Mmc5983maConfig& config);

    Status readProductId(uint8_t& id);

    /// On-demand measurement: trigger, wait for Meas_M_Done (bounded), read, convert.
    Status measure(MagSample& out);

    /// Continuous mode: read the latest result if Meas_M_Done is set, else NotReady.
    Status readLatest(MagSample& out);

    /**
     * NON-BLOCKING on-demand measurement, part 1: clears the done flag and
     * triggers TM_M, then returns immediately (two register writes).
     * @return Ok, NotInitialized, Busy (a measurement is already pending) or a bus error.
     */
    Status startMeasurement();

    /**
     * NON-BLOCKING part 2: one STATUS read. If Meas_M_Done is set, reads and
     * converts the result (Ok). If not yet done returns NotReady, unless the
     * measurement has been pending longer than measurementTimeoutMs(), in
     * which case it gives up and returns Timeout (counted in health).
     * Returns NotReady without bus traffic if no measurement is pending.
     */
    Status readMeasurementIfReady(MagSample& out);

    bool measurementPending() const { return measurementPending_; }

    /// Upper bound for one measurement at the configured bandwidth (ms).
    uint32_t measurementTimeoutMs() const;

    /**
     * SET -> measure -> RESET -> measure, then field = (Mset - Mreset)/2.
     * Removes the bridge offset (and its temperature drift).
     * Only valid in on-demand mode (continuousRate == Off).
     */
    Status measureOffsetCancelled(MagSample& out);

    /// Triggers a temperature measurement and returns degC (uncalibrated, +-several degC).
    Status measureTemperature(float& temperatureC);

    Status pulseSet();
    Status pulseReset();

    // Pure decoding helpers.
    static Mmc5983maRaw decode18Bit(const uint8_t regs[7]);
    static float countsToMicroTesla(uint32_t counts18);
    static bool isSaturated(uint32_t counts18);
    static float rawToTemperatureC(uint8_t raw);

    const SensorHealth& health() const { return health_; }
    bool isInitialized() const { return health_.initialized; }
    SensorId id() const { return id_; }

private:
    Status triggerAndWait(uint8_t triggerBit, uint8_t doneBit, uint32_t timeoutMs);
    Status readRawBlock(Mmc5983maRaw& raw);
    void fillSample(MagSample& out, const Mmc5983maRaw& raw, bool offsetCancelled);
    Status track(Status s);

    IRegisterBus& bus_;
    IClock& clock_;
    SensorId id_;
    Mmc5983maConfig config_{};
    uint8_t ctrl0Persistent_ = 0u;  ///< Shadow of bits that must be kept in every CTRL0 write.
    bool measurementPending_ = false;
    uint32_t measurementStartMs_ = 0u;
    uint32_t sequence_ = 0u;
    SensorHealth health_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_MMC5983MA_HPP
