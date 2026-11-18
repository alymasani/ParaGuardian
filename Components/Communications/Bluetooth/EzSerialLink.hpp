/**
 ******************************************************************************
 * @file    EzSerialLink.hpp
 * @brief   Transport + framing layer for an Infineon CYW20822 module running
 *          EZ-Serial firmware in TEXT mode.
 *
 * WHAT IS IMPLEMENTED (protocol-agnostic, safe to use now):
 *   - Line framing: EZ-Serial text mode exchanges ASCII lines terminated by
 *     CR LF. Bytes are accumulated in a bounded buffer until '\n'.
 *   - Overlong-line protection (line discarded, counted, never overflows).
 *   - Packet classification by the leading characters of a line:
 *       '@' 'R' ','  -> response to a command
 *       '@' 'E' ','  -> asynchronous event
 *     and splitting of the remaining comma-separated fields.
 *   - Command transmission of CALLER-SUPPLIED text with one outstanding
 *     command at a time and a response timeout.
 *   - Event callbacks and a link-state field driven by the application.
 *
 * WHAT IS DELIBERATELY NOT IMPLEMENTED:
 *   - No command names, event names, result codes or binary-mode packets are
 *     hard-coded. The EZ-Serial command reference for the exact CYW20822
 *     firmware build used on the PCB has not been verified, and inventing
 *     protocol strings is not acceptable for a safety device.
 *     -> Docs/HARDWARE_INTEGRATION_TODO.md lists what must be confirmed.
 *   - The "@R"/"@E" prefixes follow the EZ-Serial text-mode format described
 *     in Infineon's EZ-Serial user guides for earlier modules; they MUST be
 *     re-checked against the CYW20822 EZ-Serial documentation.
 *   - This layer can only say a command was handed to the module and whether
 *     a response line arrived. It can NEVER say an SOS was delivered.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMS_EZSERIAL_LINK_HPP
#define PARAGUARDIAN_COMMS_EZSERIAL_LINK_HPP

#include <cstddef>
#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/Status.hpp"

namespace pg {

namespace ezs {
constexpr size_t MAX_LINE = 160u;     // bounded receive line buffer (bytes, excl. terminator)
constexpr size_t MAX_FIELDS = 12u;    // comma-separated fields kept per line
constexpr size_t MAX_COMMAND = 128u;  // largest command line we will transmit

enum class PacketKind : uint8_t { Response, Event, Other };

/// A parsed line. Field pointers point into the link's line buffer and are
/// only valid during the callback that receives them.
struct Packet {
    PacketKind kind;
    const char* line;               ///< whole line, NUL-terminated, CR/LF removed
    size_t fieldCount;              ///< number of comma-separated fields AFTER the "@R"/"@E" tag
    const char* fields[MAX_FIELDS]; ///< each NUL-terminated
};

/// Splits @p line (modified in place: commas become NUL) into a Packet.
void parseLine(char* line, Packet& out);
}  // namespace ezs

enum class BleLinkState : uint8_t {
    Unknown = 0,     ///< nothing heard from the module yet
    ModuleActive,    ///< at least one valid line received
    Connected,       ///< set by the application's event interpreter
    Disconnected,    ///< set by the application's event interpreter
    Fault,           ///< repeated timeouts / overflow
};

struct EzSerialStats {
    uint32_t linesReceived = 0;
    uint32_t responses = 0;
    uint32_t events = 0;
    uint32_t otherLines = 0;
    uint32_t overlongLines = 0;
    uint32_t responseTimeouts = 0;
    uint32_t commandsSent = 0;
};

/// Receives parsed packets. The application decides what event names mean.
class IEzSerialListener {
public:
    virtual void onPacket(const ezs::Packet& packet) = 0;

protected:
    ~IEzSerialListener() = default;
};

class EzSerialLink {
public:
    EzSerialLink(IByteWriter& tx, uint32_t responseTimeoutMs = 1000u)
        : tx_(tx), responseTimeoutMs_(responseTimeoutMs) {}

    void setListener(IEzSerialListener* listener) { listener_ = listener; }

    /**
     * Sends one caller-supplied command line (CR LF appended).
     * @return Busy if a previous command is still waiting for its response,
     *         InvalidArgument for empty/oversized/non-printable text, or the
     *         UART writer's status.
     */
    Status sendCommand(const char* commandText, uint32_t nowMs);

    /// Feeds received bytes. Returns number of complete lines processed.
    size_t processBytes(const uint8_t* data, size_t len, uint32_t nowMs);

    /// Detects response timeouts. Call periodically.
    void poll(uint32_t nowMs);

    bool awaitingResponse() const { return awaitingResponse_; }
    BleLinkState state() const { return state_; }
    /// Application sets Connected/Disconnected from events it has verified.
    void setState(BleLinkState s) { state_ = s; }
    const EzSerialStats& stats() const { return stats_; }

private:
    void handleLine(uint32_t nowMs);

    IByteWriter& tx_;
    IEzSerialListener* listener_ = nullptr;
    uint32_t responseTimeoutMs_;
    char line_[ezs::MAX_LINE + 1u] = {};
    size_t lineLen_ = 0u;
    bool discardingLine_ = false;
    bool awaitingResponse_ = false;
    uint32_t commandSentMs_ = 0u;
    uint8_t consecutiveTimeouts_ = 0u;
    BleLinkState state_ = BleLinkState::Unknown;
    EzSerialStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_COMMS_EZSERIAL_LINK_HPP
