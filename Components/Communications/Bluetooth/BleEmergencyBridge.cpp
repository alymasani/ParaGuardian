#include "Communications/Bluetooth/BleEmergencyBridge.hpp"

namespace pg::ble {

bool EmergencyBridge::begin(Frame& frame, uint32_t now)
{
    if (nextSequence_ == UINT32_MAX) return false;   // reconnect before reusing a sequence
    frame.sequence = ++nextSequence_;
    length_ = encode(frame, tx_);
    offset_ = 0;
    kind_ = frame.kind;
    started_ = now;
    return length_ != 0;
}

TransportSubmitResult EmergencyBridge::submit(const SosRequest& request, uint32_t now)
{
    if (!connected_) return TransportSubmitResult::LinkDown;
    if (length_ != 0 && kind_ == Kind::Sos) return TransportSubmitResult::Busy;
    scratch_.kind = Kind::Sos;
    scratch_.incident = request.incidentId;
    scratch_.length = static_cast<uint8_t>(
        encodeSosRequest(request, scratch_.payload.data(), scratch_.payload.size()));
    if (request.incidentId == 0 || request.revision == 0 || scratch_.length == 0 || !begin(scratch_, now))
        return TransportSubmitResult::Invalid;
    txIncident_ = request.incidentId;
    txRevision_ = request.revision;
    txRequestKind_ = request.kind;
    ++stats_.sos;
    return TransportSubmitResult::Accepted;
}

void EmergencyBridge::accept(const Frame& frame, EmergencyManager& manager, uint32_t now)
{
    const auto& status = manager.status();
    if (frame.kind == Kind::Report && frame.length == 3 && frame.incident != 0 &&
        frame.incident == status.incidentId && frame.incident == sentIncident_) {
        const auto report = static_cast<TransportReportKind>(frame.payload[0]);
        const uint16_t revision = load16(&frame.payload[1]);
        const bool alertReport = report == TransportReportKind::PhoneAccepted ||
            report == TransportReportKind::PhoneRejected || report == TransportReportKind::PhoneSendAttempted ||
            report == TransportReportKind::DeliveryConfirmed || report == TransportReportKind::DeliveryFailed ||
            report == TransportReportKind::DeliveryUnknown;
        const bool accepted = (status.reportedStages & stagebit::AcceptedByPhone) != 0u;
        const bool attempted = (status.reportedStages & stagebit::PhoneSendAttempted) != 0u;
        const bool provenance = report == TransportReportKind::PhoneAccepted || report == TransportReportKind::PhoneRejected ||
            (accepted && (report != TransportReportKind::DeliveryConfirmed || attempted));
        if (revision == sentRevision_ && revision == status.requestRevision &&
            ((sentKind_ == SosRequestKind::Alert && alertReport && provenance) ||
             (sentKind_ == SosRequestKind::Cancel && report == TransportReportKind::CancelAccepted))) {
            manager.handleReport({report, revision, frame.incident, now}, now);
            ++stats_.reports;
            return;
        }
    }
    if (frame.kind == Kind::Command && frame.length == 1) {
        const auto type = static_cast<EmergencyCommandType>(frame.payload[0]);
        const bool current = frame.incident != 0 && frame.incident == status.incidentId;
        const bool allowed =
            (type == EmergencyCommandType::ManualSos && (current || frame.incident == 0)) ||
            (type == EmergencyCommandType::CancelSos && current && status.state == EmergencyState::Countdown) ||
            (type == EmergencyCommandType::RequestStatus && frame.incident == 0) ||
            (type == EmergencyCommandType::AcknowledgeAlert && (current || frame.incident == 0));
        if (allowed) {
            const auto result = manager.handleCommand(
                makeEmergencyCommand(type, CommandSource::PhoneApp, ++commandSequence_, frame.incident, now), now);
            if (result == CommandResult::ForwardToAlerts) alertAck_ = true;
            publishedState_ = EmergencyState::Unavailable;   // promptly report the actual resulting state
            ++stats_.commands;
            return;
        }
    }
    ++stats_.rejected;
}

void EmergencyBridge::receive(EmergencyManager& manager, uint32_t now)
{
    const Session session = port_.session();
    if (session.generation != session_.generation || session.trust != session_.trust) {
        if (connected_) {
            ++stats_.disconnects;
            manager.handleReport({TransportReportKind::LinkDown, 0, 0, now}, now);
        }
        reassembly_.reset();
        length_ = 0;
        nextSequence_ = 0;
        connected_ = session.generation != 0 && session.attPayload >= kDefaultAtt &&
            (simulation_ ? session.trust == Trust::Simulation : session.trust == Trust::Associated);
        if (connected_) {
            ++stats_.connects;
            publishedState_ = EmergencyState::Unavailable;
            lastStatusAt_ = now - 500;
            manager.handleReport({TransportReportKind::LinkUp, 0, 0, now}, now);
        } else if (session.trust != Trust::Disconnected) {
            ++stats_.unverifiedSessions;
        }
    }
    session_ = session;
    if (!connected_) return;
    if (reassembly_.expire(now)) ++stats_.incomplete;
    for (size_t i = 0; i < 4; ++i) {
        const size_t size = port_.receive(packet_);
        if (size == 0) break;
        ++stats_.rx;
        const auto result = size <= session_.attPayload
            ? reassembly_.push(packet_.data(), size, now, scratch_) : Receive::Invalid;
        switch (result) {
            case Receive::Complete: accept(scratch_, manager, now); break;
            case Receive::Duplicate: ++stats_.duplicate; break;
            case Receive::Version: ++stats_.versions; break;
            case Receive::Timeout: ++stats_.incomplete; break;
            case Receive::Invalid: ++stats_.invalid; break;
            case Receive::Partial: break;
        }
    }
}

void EmergencyBridge::publish(const EmergencyStatus& status, const Telemetry& telemetry, uint32_t now)
{
    scratch_.kind = Kind::Status;
    scratch_.incident = status.incidentId;
    scratch_.length = 40;
    auto* p = scratch_.payload.data();
    p[0] = static_cast<uint8_t>(status.state);
    p[1] = static_cast<uint8_t>(status.trigger);
    p[2] = static_cast<uint8_t>(status.stage);
    p[3] = static_cast<uint8_t>(status.failure);
    p[4] = status.attempts;
    p[5] = status.cycles;
    store16(&p[6], status.reportedStages);
    store32(&p[8], status.countdownRemainingMs);
    store32(&p[12], status.nextAttemptInMs);
    store32(&p[16], status.flags);
    p[20] = static_cast<uint8_t>(status.cancelNotice);
    p[21] = telemetry.flight;
    p[22] = telemetry.navigation;
    p[23] = telemetry.warnings;
    p[24] = telemetry.validity;
    p[25] = 0;
    store16(&p[26], telemetry.batteryMv);
    store32(&p[28], static_cast<uint32_t>(telemetry.altitudeDm));
    store32(&p[32], static_cast<uint32_t>(telemetry.speedCmps));
    store32(&p[36], telemetry.quality);
    begin(scratch_, now);
    publishedState_ = status.state;
    lastStatusAt_ = now;
}

void EmergencyBridge::transmit(EmergencyManager& manager, const Telemetry& telemetry, uint32_t now)
{
    if (!connected_) return;
    const auto& status = manager.status();
    if (length_ != 0 && kind_ == Kind::Sos &&
        (status.incidentId != txIncident_ || status.requestRevision != txRevision_ ||
         (status.state == EmergencyState::Cancelled && txRequestKind_ == SosRequestKind::Alert))) length_ = 0;
    if (length_ == 0 && (publishedState_ != status.state || now - lastStatusAt_ >= 500))
        publish(status, telemetry, now);
    if (length_ == 0) return;

    const size_t count = fragment(tx_, length_, offset_, session_.attPayload, packet_);
    const Send result = count == 0 || now - started_ >= kFragmentTimeoutMs
        ? Send::Failed : port_.send(kind_, packet_.data(), count);
    if (result == Send::Busy) {
        ++stats_.busy;
        return;
    }
    if (result == Send::Failed) {
        ++stats_.errors;
        length_ = 0;
        if (kind_ == Kind::Sos)
            manager.handleReport({TransportReportKind::WriteFailed, txRevision_, txIncident_, now}, now);
        return;
    }
    ++stats_.tx;
    offset_ += count - kFragmentHeader;
    if (offset_ != length_) return;
    length_ = 0;
    if (kind_ == Kind::Sos) {
        sentIncident_ = txIncident_;
        sentRevision_ = txRevision_;
        sentKind_ = txRequestKind_;
        // Driver queue completion is not proof that the phone received anything.
    }
}

}  // namespace pg::ble
