/**
 ******************************************************************************
 * @file    EmergencyCommand.hpp
 * @brief   Abstract emergency commands (manual SOS, cancel, acknowledge,
 *          status, reset), transport / phone reports, and the bounded inbox
 *          that carries both into the emergency task.
 *
 * No buttons are assumed to exist. A command can come from a future hardware
 * input, the phone app over BLE, or a diagnostic / test interface; each
 * source numbers its own commands (sequence) so a repeated delivery of the
 * same command is recognised and ignored.
 *
 * A command must be built with makeEmergencyCommand() (it sets the integrity
 * word). A zeroed, truncated or corrupted command is rejected: a malformed
 * command can never create an SOS.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_COMMAND_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_COMMAND_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection

namespace pg {

enum class EmergencyCommandType : uint8_t {
    None = 0,
    ManualSos,          ///< incidentId 0 = "open / escalate"; or the current incident's id
    CancelSos,          ///< incidentId REQUIRED; cancels only that incident
    AcknowledgeAlert,   ///< silences the current audible alert episode (not the SOS countdown)
    RequestStatus,      ///< asks for a status refresh (the status is published every wake anyway)
    ResetIncident,      ///< incidentId REQUIRED; closes a finished incident and re-arms
    Count
};

enum class CommandSource : uint8_t { Unknown = 0, HardwareInput, PhoneApp, Diagnostic, Test, Count };
constexpr uint8_t kCommandSourceCount = static_cast<uint8_t>(CommandSource::Count);

constexpr uint32_t kEmergencyCommandMagic = 0x50474543u;   // "PGEC"

struct EmergencyCommand {
    EmergencyCommandType type;
    CommandSource source;
    uint16_t sequence;     ///< per source; the same (source, sequence) twice = duplicate
    uint32_t incidentId;
    uint32_t issuedMs;     ///< MCU time the command was received
    uint32_t check;        ///< kEmergencyCommandMagic ^ fields (integrity)
};
static_assert(std::is_trivially_copyable<EmergencyCommand>::value, "command must be memcpy-safe");

inline uint32_t emergencyCommandCheck(const EmergencyCommand& c)
{
    return kEmergencyCommandMagic ^ (static_cast<uint32_t>(c.type) << 24) ^ (static_cast<uint32_t>(c.source) << 16) ^
           c.sequence ^ (c.incidentId * 2654435761u);
}

inline EmergencyCommand makeEmergencyCommand(EmergencyCommandType type, CommandSource source, uint16_t sequence,
                                             uint32_t incidentId, uint32_t issuedMs)
{
    EmergencyCommand c{type, source, sequence, incidentId, issuedMs, 0u};
    c.check = emergencyCommandCheck(c);
    return c;
}

/// Structural validity only (the manager decides whether it is allowed now).
inline bool isWellFormed(const EmergencyCommand& c)
{
    if (c.check != emergencyCommandCheck(c)) return false;
    if (c.type == EmergencyCommandType::None || c.type >= EmergencyCommandType::Count) return false;
    if (c.source == CommandSource::Unknown || c.source >= CommandSource::Count) return false;
    if ((c.type == EmergencyCommandType::CancelSos || c.type == EmergencyCommandType::ResetIncident) &&
        c.incidentId == 0u) {
        return false;   // never "cancel whatever is active": cancellation is incident-specific
    }
    return true;
}

/// What the transport (future BLE task) and the phone app report back.
enum class TransportReportKind : uint8_t {
    None = 0,
    LinkUp,              ///< transport / phone connection available (incidentId may be 0)
    LinkDown,
    WriteCompleted,      ///< BLE write of this request finished (transport level ONLY)
    WriteFailed,
    PhoneAccepted,       ///< phone app parsed and accepted the request
    PhoneRejected,       ///< phone app could not accept it (malformed / version)
    PhoneSendAttempted,  ///< phone app started its outbound send
    DeliveryConfirmed,   ///< phone app reports a delivery confirmation
    DeliveryFailed,      ///< phone app reports its outbound send failed
    CancelAccepted,      ///< phone app accepted a cancellation notice
    DeliveryUnknown,     ///< phone app has no reliable delivery evidence
    Count
};

struct TransportReport {
    TransportReportKind kind;
    uint16_t revision;     ///< request revision the report refers to
    uint32_t incidentId;
    uint32_t receivedMs;
};
static_assert(std::is_trivially_copyable<TransportReport>::value, "report must be memcpy-safe");

/**
 * Bounded multi-producer / single-consumer inbox (fixed memory, locked).
 * Full: the NEW item is dropped and counted (a command is never silently
 * replaced by a later one; the sender can retry).
 */
template <typename T, size_t N>
class BoundedInbox {
    static_assert(N >= 1u, "inbox too small");

public:
    explicit BoundedInbox(ICriticalSection& lock) : lock_(lock) {}

    bool push(const T& item)
    {
        lock_.enter();
        const bool ok = count_ < N;
        if (ok) {
            buf_[(head_ + count_) % N] = item;
            ++count_;
            if (count_ > highWater_) highWater_ = static_cast<uint32_t>(count_);
        } else if (dropped_ != UINT32_MAX) {
            ++dropped_;
        }
        lock_.exit();
        return ok;
    }

    bool pop(T& out)
    {
        lock_.enter();
        const bool have = count_ != 0u;
        if (have) {
            out = buf_[head_];
            head_ = (head_ + 1u) % N;
            --count_;
        }
        lock_.exit();
        return have;
    }

    size_t size() const
    {
        lock_.enter();
        const size_t n = count_;
        lock_.exit();
        return n;
    }
    uint32_t dropped() const
    {
        lock_.enter();
        const uint32_t d = dropped_;
        lock_.exit();
        return d;
    }
    uint32_t highWater() const
    {
        lock_.enter();
        const uint32_t h = highWater_;
        lock_.exit();
        return h;
    }
    static constexpr size_t capacity() { return N; }

private:
    ICriticalSection& lock_;
    T buf_[N] = {};
    size_t head_ = 0u;
    size_t count_ = 0u;
    uint32_t dropped_ = 0u;
    uint32_t highWater_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_COMMAND_HPP
