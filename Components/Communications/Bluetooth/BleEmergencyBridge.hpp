#pragma once

#include "Algorithms/Emergency/EmergencyManager.hpp"
#include "Communications/Bluetooth/BleProtocol.hpp"

namespace pg::ble {

enum class Trust { Disconnected, Unverified, Associated, Simulation };
struct Session {
    Trust trust = Trust::Disconnected;
    uint32_t generation = 0;
    size_t attPayload = kDefaultAtt;
};

enum class Send { Queued, Busy, Failed };

/// Owned by the emergency task. Methods must be bounded and non-blocking.
/// The driver authenticates BOTH directions before publishing Associated.
/// On disconnect it must discard old-session RX/TX and change generation before
/// publishing a new session. send() copies the packet or returns Busy/Failed.
class PacketPort {
public:
    virtual Session session() const = 0;
    virtual size_t receive(Packet& packet) = 0;
    virtual Send send(Kind kind, const uint8_t* packet, size_t size) = 0;

protected:
    ~PacketPort() = default;
};

class UnconfiguredPacketPort final : public PacketPort {
public:
    Session session() const override { return {}; }
    size_t receive(Packet&) override { return 0; }
    Send send(Kind, const uint8_t*, size_t) override { return Send::Failed; }
};

struct Diagnostics {
    uint32_t connects = 0, disconnects = 0, unverifiedSessions = 0;
    uint32_t rx = 0, tx = 0, invalid = 0, versions = 0, duplicate = 0, incomplete = 0;
    uint32_t busy = 0, errors = 0, sos = 0, reports = 0, commands = 0, rejected = 0;
};

struct Telemetry {
    uint32_t quality = 0;
    int32_t altitudeDm = 0, speedCmps = 0;
    uint16_t batteryMv = 0;
    uint8_t flight = 0, navigation = 0, warnings = 0, validity = 0;
};

class EmergencyBridge final : public ISosTransport {
public:
    explicit EmergencyBridge(PacketPort& port, bool simulation = false) : port_(port), simulation_(simulation) {}

    bool linkUp() const override { return connected_; }
    TransportSubmitResult submit(const SosRequest& request, uint32_t now) override;
    void receive(EmergencyManager& manager, uint32_t now);
    void transmit(EmergencyManager& manager, const Telemetry& telemetry, uint32_t now);
    bool takeAlertAcknowledgment() { const bool value = alertAck_; alertAck_ = false; return value; }
    const Diagnostics& diagnostics() const { return stats_; }

private:
    bool begin(Frame& frame, uint32_t now);
    void accept(const Frame& frame, EmergencyManager& manager, uint32_t now);
    void publish(const EmergencyStatus& status, const Telemetry& telemetry, uint32_t now);

    PacketPort& port_;
    const bool simulation_;
    Session session_{};
    bool connected_ = false, alertAck_ = false;
    uint32_t nextSequence_ = 0, lastStatusAt_ = 0, started_ = 0;
    uint16_t commandSequence_ = 0;
    uint32_t txIncident_ = 0, sentIncident_ = 0;
    uint16_t txRevision_ = 0, sentRevision_ = 0;
    SosRequestKind txRequestKind_ = SosRequestKind::Invalid, sentKind_ = SosRequestKind::Invalid;
    EmergencyState publishedState_ = EmergencyState::Unavailable;
    size_t length_ = 0, offset_ = 0;
    Kind kind_ = Kind::Status;
    Bytes tx_{};
    Packet packet_{};
    Frame scratch_{};
    Reassembly reassembly_{};
    Diagnostics stats_{};
};

}  // namespace pg::ble
