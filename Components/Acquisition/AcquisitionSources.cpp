/**
 ******************************************************************************
 * @file    AcquisitionSources.cpp
 * @brief   See AcquisitionSources.hpp.
 ******************************************************************************
 */
#include "Acquisition/AcquisitionSources.hpp"

namespace pg {

void PolledSourceBase::account(Status s)
{
    // OutOfRange = sensor alive but measuring something implausible (sample
    // still published, flagged invalid). Busy = two-phase read still waiting.
    if (isOk(s) || s == Status::OutOfRange || s == Status::Busy) {
        consecutiveReadFailures_ = 0u;
        return;
    }
    if (consecutiveReadFailures_ != UINT16_MAX) {
        ++consecutiveReadFailures_;
    }
    if (policy_.reinitAfterConsecutiveFailures != 0u &&
        consecutiveReadFailures_ >= policy_.reinitAfterConsecutiveFailures) {
        // Treat the sensor as lost: go back to the init path (immediately
        // eligible, then exponential backoff if it keeps failing).
        needsInit_ = true;
        attempted_ = false;
        retryMs_ = policy_.initialRetryMs;
        consecutiveReadFailures_ = 0u;
        if (reinitCount_ != UINT32_MAX) {
            ++reinitCount_;
        }
    }
}

void PolledSourceBase::service(uint32_t nowMs, ISampleSink& sink, CycleBudget& budget)
{
    if (needsInit_) {
        if (attempted_ && (nowMs - lastAttemptMs_) < retryMs_) {
            return;  // backing off
        }
        if (budget.initAttemptsRemaining == 0u) {
            ++budget.initAttemptsDeferred;  // someone else used this cycle's init slot
            return;
        }
        --budget.initAttemptsRemaining;
        if (attempted_) {
            // This is a retry: double the wait for the NEXT one (capped).
            retryMs_ = (retryMs_ >= policy_.maxRetryMs / 2u) ? policy_.maxRetryMs : retryMs_ * 2u;
        }
        attempted_ = true;
        lastAttemptMs_ = nowMs;
        lastInitStatus_ = begin();
        if (!isOk(lastInitStatus_)) {
            if (initFailures_ != UINT32_MAX) {
                ++initFailures_;
            }
            return;
        }
        needsInit_ = false;
        retryMs_ = policy_.initialRetryMs;
        consecutiveReadFailures_ = 0u;
        scheduled_ = false;  // start the read schedule fresh after a (re)init
    }

    if (hasPendingWork()) {
        account(continuePending(sink));
        return;  // never start a new read while one is outstanding
    }

    if (!scheduled_) {
        scheduled_ = true;
        nextDueMs_ = nowMs;
    }
    // Signed difference handles uint32 wrap-around: due when now >= nextDue.
    if (static_cast<int32_t>(nowMs - nextDueMs_) < 0) {
        return;
    }
    nextDueMs_ += periodMs_;
    // If we fell more than one period behind, do not try to "catch up" with a burst.
    if (static_cast<int32_t>(nowMs - nextDueMs_) >= 0) {
        nextDueMs_ = nowMs + periodMs_;
    }
    account(readAndPublish(sink));
}

Status ImuSource::readAndPublish(ISampleSink& sink)
{
    ImuSample s{};
    const Status st = imu_.read(s);
    if (isOk(st)) {
        sink.publish(SensorMessage::fromImu(s));
    }
    return st;
}

Status BaroSource::readAndPublish(ISampleSink& sink)
{
    BaroSample s{};
    const Status st = baro_.read(s);
    // OutOfRange samples are still published (header.valid = false) so the
    // consumer can see the sensor is alive but reporting implausible values.
    if (isOk(st) || st == Status::OutOfRange) {
        sink.publish(SensorMessage::fromBaro(s));
    }
    return st;
}

Status MagSource::readAndPublish(ISampleSink& /*sink*/)
{
    // Phase 1: trigger only (two register writes); collected in continuePending().
    return mag_.startMeasurement();
}

Status MagSource::continuePending(ISampleSink& sink)
{
    MagSample s{};
    const Status st = mag_.readMeasurementIfReady(s);
    if (st == Status::NotReady) {
        return Status::Busy;  // still converting, try again next cycle
    }
    if (isOk(st)) {
        sink.publish(SensorMessage::fromMag(s));
    }
    return st;
}

void GnssSource::service(uint32_t nowMs, ISampleSink& sink, CycleBudget& /*budget*/)
{
    uint8_t chunk[kChunkBytes];
    size_t n = 0u;
    // Bounded: at most kMaxChunksPerCycle chunks per cycle so one burst cannot
    // starve the other sensors; the rest stays in the ring buffer for next cycle.
    for (int i = 0; i < kMaxChunksPerCycle && (n = rx_.read(chunk, sizeof(chunk))) > 0u; ++i) {
        if (gnss_.processBytes(chunk, n, nowMs) > 0u) {
            sink.publish(SensorMessage::fromGnss(gnss_.latest()));
        }
    }
    gnss_.poll(nowMs);
}

void LidarSource::service(uint32_t nowMs, ISampleSink& sink, CycleBudget& /*budget*/)
{
    uint8_t chunk[64];
    size_t n = 0u;
    size_t produced = 0u;
    for (int i = 0; i < 8 && (n = rx_.read(chunk, sizeof(chunk))) > 0u; ++i) {
        produced += lidar_.processBytes(chunk, n, nowMs);
    }
    // At 100 Hz several frames may arrive per cycle; publish only the newest.
    if (produced > 0u) {
        sink.publish(SensorMessage::fromLidar(lidar_.latest()));
    }
    lidar_.poll(nowMs);
}

}  // namespace pg
