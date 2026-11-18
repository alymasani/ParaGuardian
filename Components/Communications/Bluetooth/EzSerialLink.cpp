/**
 ******************************************************************************
 * @file    EzSerialLink.cpp
 * @brief   Line framing / classification for EZ-Serial text mode. See header.
 ******************************************************************************
 */
#include "Communications/Bluetooth/EzSerialLink.hpp"

namespace pg {
namespace ezs {

void parseLine(char* line, Packet& out)
{
    out.line = line;
    out.fieldCount = 0u;
    out.kind = PacketKind::Other;
    if (line == nullptr) {
        return;
    }
    char* cursor = nullptr;
    if (line[0] == '@' && (line[1] == 'R' || line[1] == 'E') && line[2] == ',') {
        out.kind = (line[1] == 'R') ? PacketKind::Response : PacketKind::Event;
        cursor = &line[3];
    } else {
        return;  // keep 'Other' lines whole, unsplit
    }
    // Split remaining text at commas, in place.
    out.fields[out.fieldCount++] = cursor;
    for (char* p = cursor; *p != '\0'; ++p) {
        if (*p == ',') {
            *p = '\0';
            if (out.fieldCount < MAX_FIELDS) {
                out.fields[out.fieldCount++] = p + 1;
            }
        }
    }
}

}  // namespace ezs

Status EzSerialLink::sendCommand(const char* commandText, uint32_t nowMs)
{
    if (awaitingResponse_) {
        return Status::Busy;
    }
    if (commandText == nullptr || commandText[0] == '\0') {
        return Status::InvalidArgument;
    }
    char buffer[ezs::MAX_COMMAND + 2u];
    size_t len = 0u;
    for (; commandText[len] != '\0'; ++len) {
        const char c = commandText[len];
        if (len >= ezs::MAX_COMMAND || c < 0x20 || c > 0x7E) {
            return Status::InvalidArgument;  // too long or contains CR/LF/control chars
        }
        buffer[len] = c;
    }
    buffer[len++] = '\r';
    buffer[len++] = '\n';
    const Status s = tx_.write(reinterpret_cast<const uint8_t*>(buffer), len);
    if (isOk(s)) {
        awaitingResponse_ = true;
        commandSentMs_ = nowMs;
        ++stats_.commandsSent;
    }
    return s;
}

size_t EzSerialLink::processBytes(const uint8_t* data, size_t len, uint32_t nowMs)
{
    if (data == nullptr) {
        return 0u;
    }
    size_t lines = 0u;
    for (size_t i = 0; i < len; ++i) {
        const char c = static_cast<char>(data[i]);
        if (c == '\n') {
            if (discardingLine_) {
                discardingLine_ = false;  // end of the overlong line: resume normal framing
            } else {
                line_[lineLen_] = '\0';
                handleLine(nowMs);
                ++lines;
            }
            lineLen_ = 0u;
            continue;
        }
        if (c == '\r' || discardingLine_) {
            continue;
        }
        if (lineLen_ >= ezs::MAX_LINE) {
            ++stats_.overlongLines;
            discardingLine_ = true;
            lineLen_ = 0u;
            continue;
        }
        line_[lineLen_++] = c;
    }
    return lines;
}

void EzSerialLink::handleLine(uint32_t nowMs)
{
    (void)nowMs;
    if (lineLen_ == 0u) {
        return;  // blank line
    }
    ++stats_.linesReceived;
    ezs::Packet packet{};
    ezs::parseLine(line_, packet);
    switch (packet.kind) {
        case ezs::PacketKind::Response:
            ++stats_.responses;
            awaitingResponse_ = false;
            consecutiveTimeouts_ = 0u;
            break;
        case ezs::PacketKind::Event:
            ++stats_.events;
            break;
        case ezs::PacketKind::Other:
            ++stats_.otherLines;
            break;
    }
    if (state_ == BleLinkState::Unknown || state_ == BleLinkState::Fault) {
        state_ = BleLinkState::ModuleActive;
    }
    if (listener_ != nullptr) {
        listener_->onPacket(packet);
    }
}

void EzSerialLink::poll(uint32_t nowMs)
{
    if (awaitingResponse_ && (nowMs - commandSentMs_) >= responseTimeoutMs_) {
        awaitingResponse_ = false;
        ++stats_.responseTimeouts;
        if (consecutiveTimeouts_ < UINT8_MAX) {
            ++consecutiveTimeouts_;
        }
        if (consecutiveTimeouts_ >= 3u) {
            state_ = BleLinkState::Fault;
        }
    }
}

}  // namespace pg
