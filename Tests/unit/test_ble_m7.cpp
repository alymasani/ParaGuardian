#include <fstream>
#include <map>
#include <string>

#include "TestFramework.hpp"
#include "mocks/MockBlePort.hpp"

using namespace pg;
using namespace pg::ble;

namespace {

using PacketList = std::vector<mock::BlePort::Bytes>;

std::map<std::string, mock::BlePort::Bytes> vectors()
{
    std::ifstream input("vectors/ble_v1.txt");
    std::map<std::string, mock::BlePort::Bytes> result;
    std::string name, hex;
    while (input >> name >> hex) {
        for (size_t i = 0; i < hex.size(); i += 2)
            result[name].push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return result;
}

Frame sample()
{
    Frame frame{};
    frame.kind = Kind::Sos;
    frame.sequence = 0x11223344;
    frame.incident = 0x01020304;
    frame.length = 64;
    return frame;
}

Frame requestFixture()
{
    auto frame = sample();
    const auto raw = vectors().at("sos");
    std::copy(raw.begin(), raw.end(), frame.payload.begin());
    return frame;
}

PacketList packets(const Frame& frame, size_t att)
{
    mock::BlePort port;
    port.current.attPayload = att;
    port.queue(frame);
    return PacketList(port.input.begin(), port.input.end());
}

Frame command(uint32_t sequence, EmergencyCommandType type, uint32_t incident = 0)
{
    Frame frame{};
    frame.kind = Kind::Command;
    frame.sequence = sequence;
    frame.incident = incident;
    frame.length = 1;
    frame.payload[0] = static_cast<uint8_t>(type);
    return frame;
}

Frame report(uint32_t sequence, TransportReportKind kind, uint32_t id, uint16_t revision)
{
    Frame frame{};
    frame.kind = Kind::Report;
    frame.sequence = sequence;
    frame.incident = id;
    frame.length = 3;
    frame.payload[0] = static_cast<uint8_t>(kind);
    store16(&frame.payload[1], revision);
    return frame;
}

struct Fixture {
    mock::BlePort port;
    EmergencyBridge bridge;
    EmergencyConfig config;
    EmergencyManager manager;

    static EmergencyConfig settings()
    {
        EmergencyConfig result{};
        result.manualCountdownMs = 0;
        result.autoCountdownMs = 200;
        result.ackTimeoutMs = 1000;
        result.retryBaseMs = 100;
        result.maxAttempts = 2;
        result.maxCycles = 2;
        return result;
    }

    explicit Fixture(bool simulation = true)
        : bridge(port, simulation), config(settings()), manager(config, bridge) {}

    void wake(uint32_t now)
    {
        bridge.receive(manager, now);
        manager.tick(now);
        bridge.transmit(manager, {}, now);
    }

    void manual(uint32_t now)
    {
        manager.handleCommand(makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::Test, 1, 0, now), now);
        wake(now);
    }

    void drain(uint32_t now)
    {
        for (uint32_t i = 0; i < 8; ++i) wake(now + i * 20);
    }

    void suspect(uint32_t now)
    {
        CrashReport r{};
        r.status.state = CrashState::CrashSuspected;
        r.status.suspicionSerial = 1;
        r.status.episodeId = 1;
        r.publishedMs = now;
        manager.observeCrash(r, now);
        wake(now);
    }
};

}  // namespace

TEST(ble_m7_shared_alert_ack_and_cancel_vectors)
{
    const auto reference = vectors();
    CHECK_EQ(reference.size(), 14u);
    auto frame = requestFixture();
    SosRequest sos{};
    CHECK(decodeSosRequest(frame.payload.data(), frame.length, sos));
    CHECK_EQ(sos.latitude_1e7deg, 513000000);
    CHECK_EQ(sos.longitude_1e7deg, -1153000000);
    CHECK_EQ(sos.verticalSpeed_cmps, -112);
    CHECK_EQ(encodeSosRequest(sos, frame.payload.data(), frame.payload.size()), 64u);

    const auto assertVector = [&](const char* name, const Frame& item) {
        Bytes encoded{};
        const auto size = encode(item, encoded);
        const auto& expected = reference.at(name);
        CHECK_EQ(size, expected.size());
        CHECK(std::equal(expected.begin(), expected.end(), encoded.begin()));
        const auto parts = packets(item, 20);
        for (size_t i = 0; i < parts.size(); ++i)
            CHECK(parts[i] == reference.at(std::string(name) + "_att20_" + std::to_string(i)));
    };
    assertVector("alert", frame);
    assertVector("accepted", report(1, TransportReportKind::PhoneAccepted, frame.incident, 9));
    assertVector("cancel_command", command(2, EmergencyCommandType::CancelSos, frame.incident));
}

TEST(ble_m7_mtu_reassembly_replay_and_timestamp_wrap)
{
    const auto frame = requestFixture();
    for (size_t att : {20u, 41u, 244u}) {
        const auto parts = packets(frame, att);
        Reassembly rx;
        Frame parsed{};
        uint32_t now = UINT32_MAX - 30u;
        for (size_t i = 0; i < parts.size(); ++i) {
            const auto& packet = parts[i];
            const auto result = rx.push(packet.data(), packet.size(), now, parsed);
            CHECK(result == (i + 1 == parts.size() ? Receive::Complete : Receive::Partial));
            CHECK(rx.push(packet.data(), packet.size(), now, parsed) == Receive::Duplicate);
            now += 20;
        }
        CHECK_EQ(parsed.incident, frame.incident);
        CHECK(parsed.payload == frame.payload);
    }
}

TEST(ble_m7_invalid_reordered_corrupted_and_interrupted_frames)
{
    const auto parts = packets(requestFixture(), 20);
    Reassembly rx;
    Frame out{};
    const auto feed = [&](const mock::BlePort::Bytes& part, uint32_t time) {
        return rx.push(part.data(), part.size(), time, out);
    };
    CHECK(feed(parts[2], 100) == Receive::Invalid);
    CHECK(feed(parts[0], UINT32_MAX - 100) == Receive::Partial);
    CHECK(feed(parts[1], 2000) == Receive::Timeout);
    CHECK(feed(parts[0], 3000) == Receive::Partial);
    CHECK(feed(parts[2], 3001) == Receive::Invalid);
    rx.reset();
    CHECK(feed(parts[1], 4000) == Receive::Invalid);
    auto corrupt = parts;
    corrupt.back().back() ^= 1;
    for (size_t i = 0; i < corrupt.size(); ++i)
        CHECK(feed(corrupt[i], 4100) == (i + 1 == corrupt.size() ? Receive::Invalid : Receive::Partial));
    auto downgrade = parts;
    downgrade[0][6] = 0;
    for (size_t i = 0; i < downgrade.size(); ++i)
        CHECK(feed(downgrade[i], 4200) == (i + 1 == downgrade.size() ? Receive::Version : Receive::Partial));
    uint8_t ignored[kMaxPacket + 1]{};
    CHECK(rx.push(ignored, sizeof(ignored), 0, out) == Receive::Invalid);
    auto wrongLength = parts[0];
    wrongLength[5] = 255;
    CHECK(feed(wrongLength, 5000) == Receive::Invalid);
}

TEST(ble_m7_untrusted_link_cannot_cancel_or_forge_outcomes)
{
    Fixture rig(false);
    rig.port.current.trust = Trust::Unverified;
    rig.suspect(100);
    const auto incident = rig.manager.status().incidentId;
    rig.port.queue(command(1, EmergencyCommandType::CancelSos, incident));
    rig.wake(120);
    CHECK(rig.manager.status().state == EmergencyState::Countdown);
    rig.port.transition(Trust::Simulation);
    rig.port.queue(command(2, EmergencyCommandType::CancelSos, incident));
    rig.wake(140);
    CHECK(rig.manager.status().state == EmergencyState::Countdown);
    rig.wake(301);
    CHECK(rig.manager.status().state == EmergencyState::SosPending);
    rig.port.transition(Trust::Associated);
    rig.drain(400);
    rig.port.transition(Trust::Unverified);
    rig.port.queue(report(1, TransportReportKind::PhoneAccepted, incident, 1));
    rig.port.queue(report(2, TransportReportKind::DeliveryConfirmed, incident, 1));
    rig.wake(650);
    CHECK(rig.manager.status().state == EmergencyState::SosPending);
    CHECK_EQ(rig.bridge.diagnostics().reports, 0u);
    CHECK_EQ(rig.bridge.diagnostics().unverifiedSessions, 3u);
}

TEST(ble_m7_cancel_only_current_countdown_and_monotonic_command_order)
{
    Fixture rig;
    rig.suspect(100);
    const auto incident = rig.manager.status().incidentId;
    rig.port.queue(command(1, EmergencyCommandType::CancelSos, incident + 1));
    rig.wake(120);
    CHECK(rig.manager.status().state == EmergencyState::Countdown);
    rig.port.queue(command(3, EmergencyCommandType::RequestStatus));
    rig.port.queue(command(2, EmergencyCommandType::CancelSos, incident));
    rig.wake(140);
    CHECK(rig.manager.status().state == EmergencyState::Countdown);
    rig.port.queue(command(4, EmergencyCommandType::CancelSos, incident));
    rig.port.queue(command(4, EmergencyCommandType::CancelSos, incident));
    rig.wake(160);
    CHECK(rig.manager.status().state == EmergencyState::Cancelled);
    CHECK_EQ(rig.manager.status().cancellations, 1u);
    CHECK(rig.bridge.diagnostics().duplicate >= 2u);
}

TEST(ble_m7_correlates_reports_to_transmitted_incident_and_revision)
{
    Fixture rig;
    rig.manual(100);
    rig.port.queue(report(1, TransportReportKind::DeliveryConfirmed, 1, 1));
    rig.wake(120);
    CHECK_EQ(rig.bridge.diagnostics().rejected, 1u);
    rig.drain(140);
    rig.port.queue(report(2, TransportReportKind::PhoneAccepted, 2, 1));
    rig.port.queue(report(3, TransportReportKind::PhoneAccepted, 1, 2));
    rig.wake(350);
    CHECK_EQ(rig.bridge.diagnostics().rejected, 3u);
    rig.port.queue(report(4, TransportReportKind::PhoneAccepted, 1, 1));
    rig.port.queue(report(4, TransportReportKind::PhoneAccepted, 1, 1));
    rig.wake(370);
    CHECK(rig.manager.status().state == EmergencyState::AwaitingDelivery);
    rig.port.queue(report(5, TransportReportKind::DeliveryUnknown, 1, 1));
    rig.wake(390);
    CHECK(rig.manager.status().state == EmergencyState::DeliveryUnknown);
}

TEST(ble_m7_backpressure_is_bounded_and_sos_survives_disconnect_and_exhaustion)
{
    Fixture rig;
    rig.port.capacity = 1;
    rig.manual(100);
    rig.drain(120);
    CHECK_EQ(rig.port.output.size(), 1u);
    CHECK(rig.bridge.diagnostics().busy >= 1u);
    CHECK(rig.manager.pendingRecord().active);
    rig.port.transition(Trust::Disconnected);
    rig.wake(400);
    rig.port.transition(Trust::Simulation);
    rig.wake(500);
    CHECK_EQ(rig.manager.status().attempts, 2u);
    rig.port.transition(Trust::Disconnected);
    rig.wake(600);
    CHECK(rig.manager.status().failure == FailureReason::RetriesExhausted);
    CHECK(rig.manager.pendingRecord().active);
    rig.port.transition(Trust::Simulation);
    rig.wake(700);
    CHECK_EQ(rig.manager.status().cycles, 2u);
    CHECK_EQ(rig.manager.status().incidentId, 1u);
    CHECK_EQ(rig.manager.status().requestRevision, 2u);
}

TEST(ble_m7_repeated_ordered_packet_stress)
{
    Reassembly receiver;
    Frame out{};
    uint32_t now = UINT32_MAX - 4000u;
    for (uint32_t sequence = 1; sequence <= 2000; ++sequence) {
        auto frame = sample();
        frame.sequence = sequence;
        const auto parts = packets(frame, sequence % 2 == 0 ? 20 : 118);
        for (size_t i = 0; i < parts.size(); ++i) {
            const auto& part = parts[i];
            CHECK(receiver.push(part.data(), part.size(), now++, out) ==
                (i + 1 == parts.size() ? Receive::Complete : Receive::Partial));
        }
        CHECK_EQ(out.sequence, sequence);
    }
}

TEST(ble_m7_unconfigured_port_preserves_in_memory_sos_but_reboot_loses_it)
{
    UnconfiguredPacketPort port;
    EmergencyBridge lowLevel(port);
    EmergencyManager first(Fixture::settings(), lowLevel);
    first.handleCommand(makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::Test, 1, 0, 1), 1);
    for (uint32_t now = 1; now < 100000; now += 20) {
        lowLevel.receive(first, now);
        first.tick(now);
        lowLevel.transmit(first, {}, now);
    }
    CHECK(first.pendingRecord().active);
    CHECK_EQ(first.status().attempts, 0u);
    CHECK((first.status().flags & emflag::TransportUnavailable) != 0);
    EmergencyManager rebooted(Fixture::settings(), lowLevel);
    rebooted.restore(0);
    CHECK(!rebooted.pendingRecord().active);
}

TEST(ble_m7_reconnect_after_acceptance_recovers_final_stage_without_a_new_incident)
{
    Fixture rig;
    rig.manual(100);
    rig.drain(120);
    rig.port.queue(report(1, TransportReportKind::PhoneAccepted, 1, 1));
    rig.wake(300);
    CHECK(rig.manager.status().state == EmergencyState::AwaitingDelivery);
    rig.port.queue(report(2, TransportReportKind::DeliveryConfirmed, 1, 1));
    rig.wake(320);
    CHECK(rig.manager.status().state == EmergencyState::AwaitingDelivery);
    rig.port.transition(Trust::Disconnected);
    rig.wake(340);
    rig.port.transition(Trust::Simulation);
    rig.drain(360);
    CHECK_EQ(rig.manager.status().requestRevision, 2u);
    rig.port.queue(report(1, TransportReportKind::PhoneAccepted, 1, 2));
    rig.port.queue(report(2, TransportReportKind::PhoneSendAttempted, 1, 2));
    rig.port.queue(report(3, TransportReportKind::DeliveryConfirmed, 1, 2));
    rig.drain(520);
    CHECK(rig.manager.status().state == EmergencyState::Delivered);
    CHECK_EQ(rig.manager.status().incidents, 1u);
}
