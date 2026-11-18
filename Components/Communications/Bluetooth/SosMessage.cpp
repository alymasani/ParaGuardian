/**
 ******************************************************************************
 * @file    SosMessage.cpp
 * @brief   Bounded SOS record formatter (no printf, no heap).
 ******************************************************************************
 */
#include "Communications/Bluetooth/SosMessage.hpp"

namespace pg {
namespace {

/// Small bounded string builder. Every append checks remaining capacity.
class Writer {
public:
    Writer(char* dst, size_t cap) : dst_(dst), cap_(cap) {}

    void put(char c)
    {
        if (len_ + 1u < cap_) {  // keep one byte for the terminating NUL
            dst_[len_++] = c;
        } else {
            overflow_ = true;
        }
    }

    void putText(const char* s)
    {
        while (*s != '\0') {
            put(*s++);
        }
    }

    void putInt(int64_t v)
    {
        char tmp[21];
        size_t n = 0u;
        const bool negative = v < 0;
        uint64_t mag = negative ? static_cast<uint64_t>(-(v + 1)) + 1u : static_cast<uint64_t>(v);
        do {
            tmp[n++] = static_cast<char>('0' + (mag % 10u));
            mag /= 10u;
        } while (mag != 0u);
        if (negative) {
            put('-');
        }
        while (n > 0u) {
            put(tmp[--n]);
        }
    }

    size_t finish()
    {
        if (cap_ == 0u) {
            return 0u;
        }
        dst_[len_] = '\0';
        return overflow_ ? 0u : len_;
    }

private:
    char* dst_;
    size_t cap_;
    size_t len_ = 0u;
    bool overflow_ = false;
};

}  // namespace

size_t formatSosRecord(const SosPayload& payload, char* dst, size_t capacity)
{
    if (dst == nullptr || capacity == 0u) {
        return 0u;
    }
    Writer w(dst, capacity);
    w.putText("SOS,");
    w.putInt(static_cast<int64_t>(payload.reason));
    w.put(',');
    w.putInt(payload.positionValid ? 1 : 0);
    w.put(',');
    w.putInt(payload.latitude_1e7deg);
    w.put(',');
    w.putInt(payload.longitude_1e7deg);
    w.put(',');
    w.putInt(payload.altitudeMsl_dm);
    w.put(',');
    w.putInt(payload.batteryMillivolts);
    w.put(',');
    w.putInt(payload.uptimeSeconds);
    return w.finish();
}

}  // namespace pg
