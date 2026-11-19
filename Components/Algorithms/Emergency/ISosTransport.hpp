/**
 ******************************************************************************
 * @file    ISosTransport.hpp
 * @brief   What the EmergencyManager needs from a transport towards the
 *          companion phone (future: BLE through the CYW20822 / EZ-Serial).
 *
 * submit() must NOT block: it copies the request into the transport's own
 * buffer and returns. Everything that happens later (write completed, phone
 * accepted, delivery outcome, link changes) comes back asynchronously as
 * TransportReports (EmergencyCommand.hpp) through the emergency task's inbox.
 *
 * Milestone 7's BLE bridge implements this boundary. The shipping task uses
 * an unconfigured packet port until CYW20822 transport and trust provisioning
 * are verified. Host tests use a simulated packet port.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_ISOS_TRANSPORT_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_ISOS_TRANSPORT_HPP

#include <cstdint>

#include "Algorithms/Emergency/SosRequest.hpp"

namespace pg {

enum class TransportSubmitResult : uint8_t {
    Accepted = 0,   ///< copied into the transport's buffer (NOT sent, NOT received)
    Busy,           ///< a previous transfer is still in progress: try again soon
    LinkDown,       ///< no connection to a phone
    Unavailable,    ///< transport not configured / faulted
    Invalid,        ///< request rejected by the transport (e.g. cannot encode)
};

class ISosTransport {
public:
    virtual bool linkUp() const = 0;
    virtual TransportSubmitResult submit(const SosRequest& request, uint32_t nowMs) = 0;

protected:
    ~ISosTransport() = default;
};

/// Firmware default until a verified BLE transport exists: honestly unavailable.
class UnconfiguredSosTransport final : public ISosTransport {
public:
    bool linkUp() const override { return false; }
    TransportSubmitResult submit(const SosRequest&, uint32_t) override { return TransportSubmitResult::Unavailable; }
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_ISOS_TRANSPORT_HPP
