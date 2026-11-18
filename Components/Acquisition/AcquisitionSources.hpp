/**
 ******************************************************************************
 * @file    AcquisitionSources.hpp
 * @brief   Adapters that connect each driver to the acquisition loop.
 *
 * The acquisition task does not know about IMUs or barometers. It owns a list
 * of IAcquisitionSource pointers and, every cycle, calls service() on each.
 * Each adapter decides whether its sensor is due (its own period), reads it,
 * and publishes a SensorMessage.
 *
 * Timing rules (hardened, see Docs/RTOS_TIMING_ANALYSIS.md):
 *  - Driver init() can block for several ms (reset/power-up waits). The
 *    CycleBudget passed by the scheduler allows only a limited number of init
 *    attempts per cycle (default 1); other sources wait for a later cycle, so
 *    several failing sensors cannot stack their init delays into one cycle.
 *  - A failed init is retried with exponential backoff
 *    (initial 1 s, doubling, capped at 8 s by default).
 *  - A sensor that was working but then fails reinitAfterConsecutiveFailures
 *    reads in a row is re-initialised (e.g. it browned out and lost its config).
 *  - Normal reads never block for a sensor-defined wait: the magnetometer uses
 *    a two-phase trigger / collect cycle instead of its blocking measure().
 *
 * Everything here is portable (no FreeRTOS), so it is unit-tested on a laptop.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SOURCES_HPP
#define PARAGUARDIAN_ACQUISITION_SOURCES_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/SensorDataHub.hpp"
#include "Common/BusInterfaces.hpp"
#include "Common/Status.hpp"
#include "Sensors/BMP585/Bmp585.hpp"
#include "Sensors/Common/SensorHealth.hpp"
#include "Sensors/LSM6DSRX/Lsm6dsrx.hpp"
#include "Sensors/MMC5983MA/Mmc5983ma.hpp"
#include "Sensors/NEOM9N/NeoM9n.hpp"
#include "Sensors/TF02Pro/Tf02Pro.hpp"

namespace pg {

/// Per-cycle allowance handed to every source by the scheduler.
struct CycleBudget {
    uint8_t initAttemptsRemaining = 1u;  ///< driver init() calls still allowed this cycle
    uint32_t initAttemptsDeferred = 0u;  ///< sources that wanted to init but had to wait
};

class IAcquisitionSource {
public:
    virtual const char* name() const = 0;
    /// Do whatever is due at @p nowMs, publish to @p sink, respect @p budget. Must not block for long.
    virtual void service(uint32_t nowMs, ISampleSink& sink, CycleBudget& budget) = 0;
    /// Convenience overload (one init attempt allowed). Kept for compatibility.
    void service(uint32_t nowMs, ISampleSink& sink)
    {
        CycleBudget budget{};
        service(nowMs, sink, budget);
    }
    virtual const SensorHealth& health() const = 0;
    /// Number of failed init attempts so far.
    virtual uint32_t initFailures() const = 0;
    /// Number of times a working sensor was re-initialised after repeated failures.
    virtual uint32_t reinitCount() const { return 0u; }

protected:
    ~IAcquisitionSource() = default;
};

struct RetryPolicy {
    uint32_t initialRetryMs = 1000u;              ///< wait after the first failed init
    uint32_t maxRetryMs = 8000u;                  ///< backoff cap
    uint16_t reinitAfterConsecutiveFailures = 10u;///< read failures in a row before re-init
};

/// Shared scheduling / retry logic for register-polled sensors (IMU, baro, mag).
class PolledSourceBase : public IAcquisitionSource {
public:
    using IAcquisitionSource::service;
    const char* name() const override { return name_; }
    uint32_t initFailures() const override { return initFailures_; }
    uint32_t reinitCount() const override { return reinitCount_; }
    Status lastInitStatus() const { return lastInitStatus_; }
    uint32_t currentRetryIntervalMs() const { return retryMs_; }
    bool needsInit() const { return needsInit_; }
    void service(uint32_t nowMs, ISampleSink& sink, CycleBudget& budget) override;

protected:
    PolledSourceBase(const char* name, uint32_t periodMs, const RetryPolicy& policy)
        : name_(name), periodMs_(periodMs), policy_(policy), retryMs_(policy.initialRetryMs) {}
    ~PolledSourceBase() = default;

    virtual Status begin() = 0;
    /// Called when the source is due. Return the read status (Ok, or the failure).
    virtual Status readAndPublish(ISampleSink& sink) = 0;
    /// Two-phase sources: is a started measurement still waiting to be collected?
    virtual bool hasPendingWork() const { return false; }
    /// Called every cycle while hasPendingWork(). Return Busy while still waiting.
    virtual Status continuePending(ISampleSink& /*sink*/) { return Status::Ok; }

private:
    void account(Status s);

    const char* name_;
    uint32_t periodMs_;
    RetryPolicy policy_;
    uint32_t retryMs_;
    bool needsInit_ = true;
    bool attempted_ = false;
    uint32_t lastAttemptMs_ = 0u;
    bool scheduled_ = false;
    uint32_t nextDueMs_ = 0u;
    uint32_t initFailures_ = 0u;
    uint32_t reinitCount_ = 0u;
    uint16_t consecutiveReadFailures_ = 0u;
    Status lastInitStatus_ = Status::NotInitialized;
};

class ImuSource final : public PolledSourceBase {
public:
    ImuSource(const char* name, Lsm6dsrx& imu, const Lsm6dsrxConfig& cfg, uint32_t periodMs,
              const RetryPolicy& policy = RetryPolicy{})
        : PolledSourceBase(name, periodMs, policy), imu_(imu), cfg_(cfg) {}
    const SensorHealth& health() const override { return imu_.health(); }

private:
    Status begin() override { return imu_.init(cfg_); }
    Status readAndPublish(ISampleSink& sink) override;
    Lsm6dsrx& imu_;
    Lsm6dsrxConfig cfg_;
};

class BaroSource final : public PolledSourceBase {
public:
    BaroSource(const char* name, Bmp585& baro, const Bmp585Config& cfg, uint32_t periodMs,
               const RetryPolicy& policy = RetryPolicy{})
        : PolledSourceBase(name, periodMs, policy), baro_(baro), cfg_(cfg) {}
    const SensorHealth& health() const override { return baro_.health(); }

private:
    Status begin() override { return baro_.init(cfg_); }
    Status readAndPublish(ISampleSink& sink) override;
    Bmp585& baro_;
    Bmp585Config cfg_;
};

/**
 * Magnetometer: when due, TRIGGERS a measurement and returns; the result is
 * collected on a later cycle (8 ms at 100 Hz bandwidth). Its configured
 * period must therefore be longer than one measurement (e.g. >= 20 ms).
 */
class MagSource final : public PolledSourceBase {
public:
    MagSource(const char* name, Mmc5983ma& mag, const Mmc5983maConfig& cfg, uint32_t periodMs,
              const RetryPolicy& policy = RetryPolicy{})
        : PolledSourceBase(name, periodMs, policy), mag_(mag), cfg_(cfg) {}
    const SensorHealth& health() const override { return mag_.health(); }

private:
    Status begin() override { return mag_.init(cfg_); }
    Status readAndPublish(ISampleSink& sink) override;
    bool hasPendingWork() const override { return mag_.measurementPending(); }
    Status continuePending(ISampleSink& sink) override;
    Mmc5983ma& mag_;
    Mmc5983maConfig cfg_;
};

/// Drains a UART receive buffer into the GNSS driver; publishes each new NAV-PVT.
class GnssSource final : public IAcquisitionSource {
public:
    using IAcquisitionSource::service;
    GnssSource(const char* name, NeoM9n& gnss, IByteSource& rx) : name_(name), gnss_(gnss), rx_(rx) {}
    const char* name() const override { return name_; }
    void service(uint32_t nowMs, ISampleSink& sink, CycleBudget& budget) override;
    const SensorHealth& health() const override { return gnss_.health(); }
    uint32_t initFailures() const override { return 0u; }

    static constexpr size_t kChunkBytes = 64u;
    static constexpr int kMaxChunksPerCycle = 8;  ///< 512 bytes per cycle at most

private:
    const char* name_;
    NeoM9n& gnss_;
    IByteSource& rx_;
};

/// Drains a UART receive buffer into the LiDAR parser; publishes the newest frame per cycle.
class LidarSource final : public IAcquisitionSource {
public:
    using IAcquisitionSource::service;
    LidarSource(const char* name, Tf02Pro& lidar, IByteSource& rx) : name_(name), lidar_(lidar), rx_(rx) {}
    const char* name() const override { return name_; }
    void service(uint32_t nowMs, ISampleSink& sink, CycleBudget& budget) override;
    const SensorHealth& health() const override { return lidar_.health(); }
    uint32_t initFailures() const override { return 0u; }

private:
    const char* name_;
    Tf02Pro& lidar_;
    IByteSource& rx_;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SOURCES_HPP
