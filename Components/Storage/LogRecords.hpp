/**
 ******************************************************************************
 * @file    LogRecords.hpp
 * @brief   Binary log record schema (version 1): types, explicit
 *          little-endian payload encoding, record framing, and the bounded
 *          producer queue between the RTOS tasks and the StorageTask.
 *
 * RECORD FRAME (on flash; never crosses a 256-byte page):
 *    0  sync          u8   0xA5
 *    1  type          u8   LogType
 *    2  payload len   u8   0..kMaxPayload
 *    3  schema        u8   kLogSchemaVersion
 *    4  sequence      u32  monotonic per log (survives rotation and resets)
 *    8  timestamp ms  u32  MCU uptime (wraps after 49.7 days; the reader unwraps per boot)
 *   12  payload       len bytes, explicitly encoded (no struct copies, no padding)
 *   12+len CRC-32 of bytes 0..11+len
 * Total = 16 + len bytes. Bytes after the last record of a page are 0xFF.
 *
 * Payload fields are written with LogPayload::put*() in a fixed order per type
 * (the order is the schema; Tools/pglog/pglog_decode.py mirrors it). Invalid
 * values are written as explicit sentinels (INT32_MIN), never left undefined.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_LOG_RECORDS_HPP
#define PARAGUARDIAN_STORAGE_LOG_RECORDS_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection

namespace pg {

constexpr uint8_t kLogSync = 0xA5u;
constexpr uint8_t kLogSchemaVersion = 1u;
constexpr size_t kLogHeaderBytes = 12u;
constexpr size_t kLogCrcBytes = 4u;
constexpr size_t kLogMaxPayload = 48u;
constexpr size_t kLogMaxRecord = kLogHeaderBytes + kLogMaxPayload + kLogCrcBytes;   // 64

enum class LogType : uint8_t {
    Invalid = 0,
    Boot = 1,              ///< resetFlags u32, bootCount u32, journalMount u8, criticalMount u8, flightMount u8
    SensorQuality = 2,     ///< qualityFlags u32, accel/gyro/baro status u8 x3, masks u8 x3, sourcesHealthy u16
    Navigation = 3,        ///< state u8, alt_cm i32, vz_mmps i32, az_mmps2 i32, altSigma_cm i32, flags u32
    SafetyEvent = 4,       ///< eventId u32, type u8, severity u8, previous u8, transition u8, source u8, phase u8, episode u32, vz_mmps i32, clearance_cm i32
    CrashSummary = 5,      ///< state u8, decision u8, class u8, evidence u32, peak_cg i32, rate_dps i32, candidates u32, suspicions u32
    Emergency = 6,         ///< state u8, trigger u8, stage u8, cancelNotice u8, incident u32, revision u16, reportedStages u16, flags u32, persist u8, attempts u8
    StorageFault = 7,      ///< component u8, status u8, detail u32, count u32
    Health = 8,            ///< freeHeap u32, minHeap u32, procMaxMs u32, emergencyLateWakes u32, stackMinWords u32, faults u32
    Power = 9,             ///< battery state u8, mv u16, flags u8, rail u8, amp u8, lidarPower u8, sources u8
    Profile = 10,          ///< profile u8, reason u8
    Count
};

/// Explicit little-endian payload builder (bounded; overflow is detected, not written).
class LogPayload {
public:
    void put8(uint8_t v) { putRaw(&v, 1u); }
    void put16(uint16_t v)
    {
        const uint8_t b[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
        putRaw(b, 2u);
    }
    void put32(uint32_t v)
    {
        const uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v >> 16),
                              static_cast<uint8_t>(v >> 24)};
        putRaw(b, 4u);
    }
    void putI32(int32_t v) { put32(static_cast<uint32_t>(v)); }
    /// value * scale rounded; NaN / out of range -> INT32_MIN (explicit "invalid").
    void putScaled(float value, float scale)
    {
        const float x = value * scale;
        putI32((x > -2.0e9f && x < 2.0e9f) ? static_cast<int32_t>(x >= 0.0f ? x + 0.5f : x - 0.5f) : INT32_MIN);
    }
    const uint8_t* data() const { return buf_; }
    size_t size() const { return len_; }
    bool overflow() const { return overflow_; }

private:
    void putRaw(const uint8_t* p, size_t n)
    {
        if (len_ + n > kLogMaxPayload) {
            overflow_ = true;
            return;
        }
        std::memcpy(&buf_[len_], p, n);
        len_ += n;
    }
    uint8_t buf_[kLogMaxPayload] = {};
    size_t len_ = 0u;
    bool overflow_ = false;
};

/// One queued entry (the sequence number is assigned by the writer, in write order).
struct LogEntry {
    LogType type;
    uint8_t length;
    uint32_t timestampMs;
    uint8_t payload[kLogMaxPayload];
};

/// Encodes a frame; returns its length (0 = invalid entry).
size_t encodeLogRecord(const LogEntry& e, uint32_t sequence, uint8_t* out, size_t cap);
/// Decodes one frame at @p in. Returns its length, 0 if no valid frame starts here.
size_t decodeLogRecord(const uint8_t* in, size_t avail, LogEntry& e, uint32_t& sequence);

/// Consumer view of a log queue (the StorageTask drains it).
class ILogSource {
public:
    virtual bool peek(LogEntry& out) const = 0;
    virtual void pop() = 0;
    virtual size_t size() const = 0;

protected:
    ~ILogSource() = default;
};

/// Producer view (tasks / FlightRecorder push into it; never blocks).
class ILogSink {
public:
    virtual bool push(LogType type, uint32_t timestampMs, const LogPayload& p) = 0;

protected:
    ~ILogSink() = default;
};

struct LogQueueStats {
    uint32_t pushed;
    uint32_t dropped;     ///< queue full: the NEW entry was dropped (counted, never silent)
    uint32_t highWater;
};

/**
 * Bounded multi-producer / single-consumer queue of LogEntry (fixed slots,
 * no heap). push() never blocks: if full, the new entry is dropped and
 * counted. Separate queues for telemetry and critical records mean telemetry
 * bursts can never push out an emergency record.
 */
template <size_t N>
class LogQueue final : public ILogSource, public ILogSink {
public:
    explicit LogQueue(ICriticalSection& lock) : lock_(lock) {}

    bool push(LogType type, uint32_t timestampMs, const LogPayload& p) override
    {
        if (p.overflow() || type == LogType::Invalid) return false;
        lock_.enter();
        bool ok = false;
        if (count_ < N) {
            LogEntry& e = slots_[(head_ + count_) % N];
            e.type = type;
            e.length = static_cast<uint8_t>(p.size());
            e.timestampMs = timestampMs;
            std::memcpy(e.payload, p.data(), p.size());
            ++count_;
            ok = true;
            if (stats_.pushed != UINT32_MAX) ++stats_.pushed;
            if (count_ > stats_.highWater) stats_.highWater = static_cast<uint32_t>(count_);
        } else if (stats_.dropped != UINT32_MAX) {
            ++stats_.dropped;
        }
        lock_.exit();
        return ok;
    }
    bool peek(LogEntry& out) const override
    {
        lock_.enter();
        const bool ok = count_ > 0u;
        if (ok) out = slots_[head_];
        lock_.exit();
        return ok;
    }
    void pop() override
    {
        lock_.enter();
        if (count_ > 0u) {
            head_ = (head_ + 1u) % N;
            --count_;
        }
        lock_.exit();
    }
    size_t size() const override
    {
        lock_.enter();
        const size_t n = count_;
        lock_.exit();
        return n;
    }
    LogQueueStats stats() const
    {
        lock_.enter();
        const LogQueueStats s = stats_;
        lock_.exit();
        return s;
    }
    static constexpr size_t capacity() { return N; }

private:
    ICriticalSection& lock_;
    LogEntry slots_[N] = {};
    size_t head_ = 0u;
    size_t count_ = 0u;
    LogQueueStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_LOG_RECORDS_HPP
