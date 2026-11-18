/**
 ******************************************************************************
 * @file    UbxParser.hpp
 * @brief   Byte-at-a-time parser for the u-blox UBX binary protocol.
 *
 * UBX frame (u-blox M9 SPG Interface Description, section "UBX protocol"):
 *
 *   0xB5 0x62 | CLASS | ID | LEN_L LEN_H | PAYLOAD (LEN bytes) | CK_A CK_B
 *
 * The checksum is the 8-bit Fletcher algorithm over CLASS, ID, LEN and PAYLOAD:
 *   for each byte b:  CK_A = CK_A + b;  CK_B = CK_B + CK_A;   (both mod 256)
 *
 * The receiver also sends NMEA text on the same UART by default. Bytes that
 * are not part of a UBX frame are skipped and counted (discardedBytes).
 *
 * Bounded memory: the payload buffer has a fixed capacity. A frame whose
 * declared length exceeds it is skipped (counted in oversizeFrames) WITHOUT
 * writing past the buffer.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_NEOM9N_UBX_PARSER_HPP
#define PARAGUARDIAN_SENSORS_NEOM9N_UBX_PARSER_HPP

#include <cstddef>
#include <cstdint>

namespace pg {
namespace ubx {

constexpr uint8_t SYNC_1 = 0xB5;
constexpr uint8_t SYNC_2 = 0x62;
constexpr size_t HEADER_LEN = 6u;   // sync(2) + class + id + length(2)
constexpr size_t CHECKSUM_LEN = 2u;

/// Largest payload kept. NAV-PVT is 92 bytes; 256 leaves room for MON-VER etc.
constexpr size_t MAX_PAYLOAD = 256u;

/// A complete, checksum-verified frame. payload points into the parser's buffer
/// and is only valid until the next call to UbxParser::feed().
struct Frame {
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t length;
    const uint8_t* payload;
};

struct ParserStats {
    uint32_t framesOk = 0;
    uint32_t checksumErrors = 0;
    uint32_t oversizeFrames = 0;
    uint32_t discardedBytes = 0;   // bytes outside any UBX frame (NMEA, noise)
    uint32_t timeoutResets = 0;    // partial frames abandoned because the stream stalled
};

/// Fletcher-8 checksum as defined by u-blox. Exposed for building outgoing frames.
void checksum(const uint8_t* data, size_t len, uint8_t& ckA, uint8_t& ckB);

class Parser {
public:
    /**
     * Feeds one received byte. Returns true exactly when this byte completed
     * a valid frame; @p out is then filled. @p nowMs is used for the
     * inter-byte timeout (pass the same clock you use elsewhere).
     */
    bool feed(uint8_t byte, uint32_t nowMs, Frame& out);

    /// Abandons a partial frame if no byte arrived for @p timeoutMs.
    void checkTimeout(uint32_t nowMs, uint32_t timeoutMs = 100u);

    void reset();
    const ParserStats& stats() const { return stats_; }
    bool inFrame() const { return state_ != State::Sync1; }

private:
    enum class State : uint8_t { Sync1, Sync2, Class, Id, LenLow, LenHigh, Payload, CkA, CkB };

    void restart() { state_ = State::Sync1; }

    State state_ = State::Sync1;
    uint8_t class_ = 0;
    uint8_t id_ = 0;
    uint16_t length_ = 0;
    uint16_t index_ = 0;
    bool oversize_ = false;   // payload is being skipped, not stored
    uint8_t ckA_ = 0;
    uint8_t ckB_ = 0;
    uint8_t rxCkA_ = 0;
    uint32_t lastByteMs_ = 0;
    uint8_t payload_[MAX_PAYLOAD] = {};
    ParserStats stats_{};
};

/**
 * Builds a complete UBX frame (sync, header, payload, checksum) into @p dst.
 * @return number of bytes written, or 0 if @p dstCapacity is too small.
 */
size_t buildFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t payloadLen, uint8_t* dst,
                  size_t dstCapacity);

}  // namespace ubx
}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_NEOM9N_UBX_PARSER_HPP
