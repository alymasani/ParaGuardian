#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace pg::ble {

constexpr uint8_t kVersion = 1;
constexpr size_t kHeader = 12;
constexpr size_t kOverhead = kHeader + 4;
constexpr size_t kFragmentHeader = 6;
constexpr size_t kMaxPayload = 96;
constexpr size_t kMaxFrame = kOverhead + kMaxPayload;
constexpr size_t kMaxPacket = kMaxFrame + kFragmentHeader;
constexpr size_t kDefaultAtt = 20;
constexpr uint32_t kFragmentTimeoutMs = 2000;

enum class Kind : uint8_t { Sos = 1, Report, Command, Status, Telemetry };

struct Frame {
    uint32_t sequence = 0;
    uint32_t incident = 0;
    Kind kind = Kind::Sos;
    uint8_t length = 0;
    std::array<uint8_t, kMaxPayload> payload{};
};

using Bytes = std::array<uint8_t, kMaxFrame>;
using Packet = std::array<uint8_t, kMaxPacket>;

void store16(uint8_t* out, uint16_t value);
void store32(uint8_t* out, uint32_t value);
uint16_t load16(const uint8_t* in);
uint32_t load32(const uint8_t* in);

size_t encode(const Frame& frame, Bytes& out);
size_t fragment(const Bytes& frame, size_t length, size_t offset, size_t att, Packet& out);

enum class Receive { Partial, Complete, Duplicate, Invalid, Version, Timeout };

class Reassembly {
public:
    Receive push(const uint8_t* packet, size_t size, uint32_t now, Frame& out);
    bool expire(uint32_t now);
    void reset() { *this = Reassembly{}; }

private:
    Bytes bytes_{};
    uint32_t sequence_ = 0;
    uint32_t lastComplete_ = 0;
    uint32_t started_ = 0;
    size_t used_ = 0;
    size_t total_ = 0;
};

}  // namespace pg::ble
