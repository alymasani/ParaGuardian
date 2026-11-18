/**
 ******************************************************************************
 * @file    SosMessage.hpp
 * @brief   Future SOS data interface (application level, transport independent).
 *
 * This defines WHAT an SOS report contains and a bounded text encoding of it.
 * It deliberately has no "delivered" state: the firmware can only know that
 * the message was handed to the Bluetooth module. Whether a phone received
 * it, and whether anyone was alerted, cannot be known from this side.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMS_SOS_MESSAGE_HPP
#define PARAGUARDIAN_COMMS_SOS_MESSAGE_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

enum class SosReason : uint8_t { Unknown = 0, ManualTrigger, ImpactDetected, NoMotionAfterImpact, Test };

struct SosPayload {
    SosReason reason;
    bool positionValid;
    int32_t latitude_1e7deg;
    int32_t longitude_1e7deg;
    int32_t altitudeMsl_dm;     ///< decimetres, avoids floats in the wire format
    uint16_t batteryMillivolts; ///< 0 = unknown
    uint32_t uptimeSeconds;
};

/// What the firmware can honestly report about an SOS attempt.
enum class SosTransmitState : uint8_t {
    Idle,
    Queued,               ///< accepted by the SOS channel, not yet given to the module
    HandedToModule,       ///< bytes written to the BLE module; delivery UNKNOWN
    ModuleRejected,       ///< module returned an error response
    TransportUnavailable, ///< no link / module fault
};

/// Interface a future SOS sender must implement.
class ISosChannel {
public:
    virtual Status submit(const SosPayload& payload) = 0;
    virtual SosTransmitState state() const = 0;

protected:
    ~ISosChannel() = default;
};

/**
 * Formats the payload as a compact ASCII record, e.g.
 *   "SOS,1,1,511234567,-1141234567,10450,3700,123"
 * (reason, positionValid, lat_1e7, lon_1e7, alt_dm, battery_mV, uptime_s).
 * The format is ParaGuardian's own, not part of EZ-Serial.
 * @return characters written (excluding NUL) or 0 if @p capacity is too small.
 */
size_t formatSosRecord(const SosPayload& payload, char* dst, size_t capacity);

}  // namespace pg

#endif  // PARAGUARDIAN_COMMS_SOS_MESSAGE_HPP
