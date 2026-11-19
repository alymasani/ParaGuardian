#include "Communications/Bluetooth/BleProtocol.hpp"

#include <algorithm>
#include <cstring>

#include "Algorithms/Emergency/SosRequest.hpp"

namespace pg::ble {

void store16(uint8_t* out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8u);
}

void store32(uint8_t* out, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8u * i));
}

uint16_t load16(const uint8_t* in)
{
    return static_cast<uint16_t>(in[0] | (static_cast<uint16_t>(in[1]) << 8u));
}

uint32_t load32(const uint8_t* in)
{
    uint32_t value = 0;
    for (size_t i = 0; i < 4; ++i) value |= static_cast<uint32_t>(in[i]) << (8u * i);
    return value;
}

static bool validKind(uint8_t kind)
{
    return kind >= static_cast<uint8_t>(Kind::Sos) && kind <= static_cast<uint8_t>(Kind::Telemetry);
}

size_t encode(const Frame& frame, Bytes& out)
{
    const auto kind = static_cast<uint8_t>(frame.kind);
    if (!validKind(kind) || frame.sequence == 0 || frame.length > kMaxPayload) return 0;
    out[0] = kVersion;
    out[1] = kind;
    store32(&out[2], frame.sequence);
    store32(&out[6], frame.incident);
    store16(&out[10], frame.length);
    std::memcpy(&out[kHeader], frame.payload.data(), frame.length);
    const size_t protectedSize = kHeader + frame.length;
    store32(&out[protectedSize], crc32Ieee(out.data(), protectedSize));
    return protectedSize + 4;
}

size_t fragment(const Bytes& frame, size_t length, size_t offset, size_t att, Packet& out)
{
    if (length < kOverhead || length > kMaxFrame || offset >= length || att < kDefaultAtt) return 0;
    const size_t count = std::min(length - offset, att - kFragmentHeader);
    store32(out.data(), load32(&frame[2]));
    out[4] = static_cast<uint8_t>(offset);
    out[5] = static_cast<uint8_t>(length);
    std::memcpy(&out[kFragmentHeader], &frame[offset], count);
    return kFragmentHeader + count;
}

bool Reassembly::expire(uint32_t now)
{
    if (used_ == 0 || now - started_ < kFragmentTimeoutMs) return false;
    used_ = 0;
    return true;
}

Receive Reassembly::push(const uint8_t* packet, size_t size, uint32_t now, Frame& out)
{
    const bool expired = expire(now);
    if (packet == nullptr || size <= kFragmentHeader || size > kMaxPacket) return Receive::Invalid;
    const uint32_t sequence = load32(packet);
    const size_t offset = packet[4];
    const size_t total = packet[5];
    const size_t count = size - kFragmentHeader;
    if (sequence == 0 || total < kOverhead || total > kMaxFrame || offset + count > total) return Receive::Invalid;
    if (sequence <= lastComplete_) return Receive::Duplicate;
    if (offset == 0 && (used_ == 0 || sequence > sequence_)) {
        sequence_ = sequence;
        started_ = now;
        total_ = total;
        used_ = 0;
    }
    if (sequence != sequence_ || total != total_ || offset > used_ || (offset != 0 && used_ == 0))
        return expired ? Receive::Timeout : Receive::Invalid;
    if (offset < used_) {
        return offset + count <= used_ && std::memcmp(&bytes_[offset], packet + kFragmentHeader, count) == 0
            ? Receive::Duplicate : Receive::Invalid;
    }

    std::memcpy(&bytes_[offset], packet + kFragmentHeader, count);
    used_ += count;
    if (used_ != total_) return Receive::Partial;
    used_ = 0;
    if (bytes_[0] != kVersion) return Receive::Version;
    const size_t length = load16(&bytes_[10]);
    if (!validKind(bytes_[1]) || load32(&bytes_[2]) != sequence || length + kOverhead != total ||
        load32(&bytes_[total - 4]) != crc32Ieee(bytes_.data(), total - 4)) return Receive::Invalid;

    lastComplete_ = sequence;
    out.kind = static_cast<Kind>(bytes_[1]);
    out.sequence = sequence;
    out.incident = load32(&bytes_[6]);
    out.length = static_cast<uint8_t>(length);
    std::memcpy(out.payload.data(), &bytes_[kHeader], length);
    return Receive::Complete;
}

}  // namespace pg::ble
