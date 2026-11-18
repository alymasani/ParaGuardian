/**
 ******************************************************************************
 * @file    UbxParser.cpp
 * @brief   UBX framing state machine. See UbxParser.hpp.
 ******************************************************************************
 */
#include "Sensors/NEOM9N/UbxParser.hpp"

#include "Common/ByteOrder.hpp"

namespace pg {
namespace ubx {

void checksum(const uint8_t* data, size_t len, uint8_t& ckA, uint8_t& ckB)
{
    uint8_t a = 0u;
    uint8_t b = 0u;
    for (size_t i = 0; i < len; ++i) {
        a = static_cast<uint8_t>(a + data[i]);
        b = static_cast<uint8_t>(b + a);
    }
    ckA = a;
    ckB = b;
}

void Parser::reset()
{
    restart();
    stats_ = ParserStats{};
}

void Parser::checkTimeout(uint32_t nowMs, uint32_t timeoutMs)
{
    if (state_ != State::Sync1 && (nowMs - lastByteMs_) > timeoutMs) {
        ++stats_.timeoutResets;
        restart();
    }
}

bool Parser::feed(uint8_t byte, uint32_t nowMs, Frame& out)
{
    lastByteMs_ = nowMs;

    // Running Fletcher checksum over class..payload.
    auto accumulate = [this](uint8_t b) {
        ckA_ = static_cast<uint8_t>(ckA_ + b);
        ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
    };

    switch (state_) {
        case State::Sync1:
            if (byte == SYNC_1) {
                state_ = State::Sync2;
            } else {
                ++stats_.discardedBytes;
            }
            return false;

        case State::Sync2:
            if (byte == SYNC_2) {
                state_ = State::Class;
                ckA_ = 0u;
                ckB_ = 0u;
            } else if (byte == SYNC_1) {
                ++stats_.discardedBytes;  // previous 0xB5 was noise; this one may be the real start
            } else {
                stats_.discardedBytes += 2u;
                restart();
            }
            return false;

        case State::Class:
            class_ = byte;
            accumulate(byte);
            state_ = State::Id;
            return false;

        case State::Id:
            id_ = byte;
            accumulate(byte);
            state_ = State::LenLow;
            return false;

        case State::LenLow:
            length_ = byte;
            accumulate(byte);
            state_ = State::LenHigh;
            return false;

        case State::LenHigh:
            length_ = static_cast<uint16_t>(length_ | (static_cast<uint16_t>(byte) << 8));
            accumulate(byte);
            index_ = 0u;
            oversize_ = (length_ > MAX_PAYLOAD);
            state_ = (length_ == 0u) ? State::CkA : State::Payload;
            return false;

        case State::Payload:
            if (!oversize_) {
                payload_[index_] = byte;  // index_ < length_ <= MAX_PAYLOAD, always in bounds
            }
            accumulate(byte);
            ++index_;
            if (index_ >= length_) {
                state_ = State::CkA;
            }
            return false;

        case State::CkA:
            rxCkA_ = byte;
            state_ = State::CkB;
            return false;

        case State::CkB: {
            const bool checksumOk = (rxCkA_ == ckA_) && (byte == ckB_);
            restart();
            if (!checksumOk) {
                ++stats_.checksumErrors;
                return false;
            }
            if (oversize_) {
                ++stats_.oversizeFrames;  // valid but too large to keep
                return false;
            }
            ++stats_.framesOk;
            out.msgClass = class_;
            out.msgId = id_;
            out.length = length_;
            out.payload = payload_;
            return true;
        }
    }
    restart();
    return false;
}

size_t buildFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t payloadLen, uint8_t* dst,
                  size_t dstCapacity)
{
    const size_t total = HEADER_LEN + payloadLen + CHECKSUM_LEN;
    if (dst == nullptr || dstCapacity < total || (payloadLen > 0u && payload == nullptr)) {
        return 0u;
    }
    dst[0] = SYNC_1;
    dst[1] = SYNC_2;
    dst[2] = msgClass;
    dst[3] = msgId;
    bytes::put_le_u16(&dst[4], payloadLen);
    for (uint16_t i = 0; i < payloadLen; ++i) {
        dst[HEADER_LEN + i] = payload[i];
    }
    uint8_t a = 0u;
    uint8_t b = 0u;
    checksum(&dst[2], 4u + payloadLen, a, b);  // class, id, len(2), payload
    dst[HEADER_LEN + payloadLen] = a;
    dst[HEADER_LEN + payloadLen + 1u] = b;
    return total;
}

}  // namespace ubx
}  // namespace pg
