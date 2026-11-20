/**
 ******************************************************************************
 * @file    Lsm6dsrxFifo.hpp
 * @brief   LSM6DSRX FIFO configuration, tagged-word decoding and bounded
 *          drain with impact-peak extraction (Milestone 8, NOT ENABLED in
 *          the firmware yet).
 *
 * WHY: the acquisition loop polls each IMU every 10 ms (100 Hz) while the
 * IMUs run at 416 Hz (Lsm6dsrxConfig defaults), so ~3 of every 4 samples are
 * never read. A short impact peak (a few ms) can fall between two polls. In
 * continuous FIFO mode the IMU queues every sample; draining it each poll
 * lets the crash path see the true peak of the interval.
 *
 * REGISTER MAP USED (ST LSM6DSRX datasheet, "Register mapping" and "FIFO"
 * sections; must be re-checked against the exact datasheet revision during
 * bring-up, see Docs/MILESTONE8_HARDWARE_BRINGUP.md):
 *   FIFO_CTRL3 0x09  BDR_GY[7:4] | BDR_XL[3:0]  (batch data rates, ODR-like codes)
 *   FIFO_CTRL4 0x0A  FIFO_MODE[2:0]             (000 bypass, 110 continuous)
 *   FIFO_STATUS1 0x3A DIFF_FIFO[7:0]            (unread words)
 *   FIFO_STATUS2 0x3B DIFF_FIFO[9:8] bits 1:0, FIFO_OVR_LATCHED bit 3,
 *                     FIFO_FULL_IA bit 5, FIFO_OVR_IA bit 6, FIFO_WTM_IA bit 7
 *   FIFO_DATA_OUT_TAG 0x78 TAG_SENSOR[7:3] | TAG_CNT[2:1] | TAG_PARITY[0]
 *   FIFO_DATA_OUT_X_L..Z_H 0x79..0x7E           (one 7-byte word per sample)
 *   Tags: 0x01 gyroscope, 0x02 accelerometer, 0x03 temperature, 0x04 timestamp,
 *         0x05 configuration change (others: not used here, counted as unknown)
 *
 * TIMESTAMPS: FIFO words carry no time unless timestamp batching is enabled;
 * a drained batch is stamped with the drain time and its sample count, and the
 * sample spacing is the configured batch rate. This is enough for a PEAK over
 * the interval, not for sub-sample impact timing.
 * BOUNDED: drain() reads at most maxWords words (7 bytes each) per call, so
 * one IMU cannot starve the other sensors; leftover words are reported.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_LSM6DSRX_FIFO_HPP
#define PARAGUARDIAN_SENSORS_LSM6DSRX_FIFO_HPP

#include <cstddef>
#include <cstdint>

#include "Common/RegisterBus.hpp"
#include "Common/Status.hpp"
#include "Sensors/LSM6DSRX/Lsm6dsrx.hpp"

namespace pg {

namespace lsm6fifo {
constexpr uint8_t REG_FIFO_CTRL3 = 0x09;
constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;
constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;
constexpr uint8_t REG_FIFO_DATA_OUT_TAG = 0x78;
constexpr uint8_t MODE_BYPASS = 0x0;
constexpr uint8_t MODE_CONTINUOUS = 0x6;
constexpr uint8_t STATUS2_OVR_LATCHED = 1u << 3;
constexpr uint8_t STATUS2_FULL = 1u << 5;
constexpr uint8_t STATUS2_OVR = 1u << 6;
constexpr uint8_t TAG_GYRO = 0x01;
constexpr uint8_t TAG_ACCEL = 0x02;
constexpr uint8_t TAG_TEMPERATURE = 0x03;
constexpr uint8_t TAG_TIMESTAMP = 0x04;
constexpr uint8_t TAG_CONFIG_CHANGE = 0x05;
constexpr size_t kWordBytes = 7u;
}  // namespace lsm6fifo

enum class FifoTag : uint8_t { Unknown = 0, Gyro, Accel, Temperature, Timestamp, ConfigChange };

struct FifoWord {
    FifoTag tag;
    uint8_t rawTag;
    int16_t x, y, z;
};

/// Decodes one 7-byte FIFO word (no bus access).
FifoWord decodeFifoWord(const uint8_t* w);

struct FifoBatch {
    uint32_t timestampMs;        ///< drain time (MCU clock)
    uint16_t accelSamples;
    uint16_t gyroSamples;
    uint16_t wordsRead;
    uint16_t wordsLeft;          ///< still in the FIFO after this bounded drain
    uint16_t unknownWords;       ///< unexpected tags (corruption / unsupported batching)
    bool overflow;               ///< the FIFO overran since the last drain: samples were LOST
    bool accelSaturated;         ///< an axis hit the int16 limit (full scale reached)
    bool gyroSaturated;
    float peakAccel_mps2;        ///< max |specific force| over the batch (NaN if no accel sample)
    float peakRate_radps;        ///< max |angular rate| over the batch (NaN if no gyro sample)
};

struct FifoConfig {
    Lsm6dsrxOdr accelBatchRate = Lsm6dsrxOdr::Hz416;
    Lsm6dsrxOdr gyroBatchRate = Lsm6dsrxOdr::Hz416;
    uint16_t maxWordsPerDrain = 16u;   ///< 16 x 7 B = 112 B per call (one 10 ms poll at 416 Hz ~ 8-9 words)
};

class Lsm6dsrxFifo {
public:
    Lsm6dsrxFifo(IRegisterBus& bus, Lsm6dsrxAccelRange accelRange, Lsm6dsrxGyroRange gyroRange)
        : bus_(bus),
          accelScale_(Lsm6dsrx::accelSensitivity_mg_per_lsb(accelRange) * 9.80665e-3f),
          gyroScale_(Lsm6dsrx::gyroSensitivity_mdps_per_lsb(gyroRange) * 1.74532925e-5f)
    {
    }

    /// Enables continuous mode with the given batch rates (call after Lsm6dsrx::init()).
    Status configure(const FifoConfig& cfg);
    /// Returns the FIFO to bypass mode (the polling path keeps working unchanged).
    Status disable();
    /// Reads the unread-word count and at most cfg.maxWordsPerDrain words.
    Status drain(uint32_t nowMs, FifoBatch& out);

    uint32_t overflows() const { return overflows_; }
    uint32_t partialReads() const { return partialReads_; }

private:
    IRegisterBus& bus_;
    float accelScale_;
    float gyroScale_;
    uint16_t maxWords_ = 16u;
    bool configured_ = false;
    uint32_t overflows_ = 0u;
    uint32_t partialReads_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_LSM6DSRX_FIFO_HPP
