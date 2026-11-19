#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include "Communications/Bluetooth/BleEmergencyBridge.hpp"

namespace pg::mock {

class BlePort final : public ble::PacketPort {
public:
    using Bytes = std::vector<uint8_t>;
    ble::Session current{ble::Trust::Simulation, 1, 20};
    ble::Send result = ble::Send::Queued;
    size_t capacity = 32;
    std::deque<Bytes> input, output;

    ble::Session session() const override { return current; }
    void transition(ble::Trust trust)
    {
        current.trust = trust;
        ++current.generation;
        input.clear();
        output.clear();
    }
    size_t receive(ble::Packet& packet) override
    {
        if (input.empty()) return 0;
        const auto size = input.front().size();
        if (size <= packet.size()) std::copy(input.front().begin(), input.front().end(), packet.begin());
        input.pop_front();
        return size;
    }
    ble::Send send(ble::Kind, const uint8_t* packet, size_t size) override
    {
        if (result != ble::Send::Queued) return result;
        if (output.size() >= capacity) return ble::Send::Busy;
        output.emplace_back(packet, packet + size);
        return ble::Send::Queued;
    }

    void queue(const ble::Frame& frame)
    {
        ble::Bytes encoded{};
        ble::Packet packet{};
        const auto length = ble::encode(frame, encoded);
        for (size_t offset = 0; offset < length;) {
            const auto size = ble::fragment(encoded, length, offset, current.attPayload, packet);
            if (size == 0) return;
            input.emplace_back(packet.begin(), packet.begin() + static_cast<ptrdiff_t>(size));
            offset += size - ble::kFragmentHeader;
        }
    }
};

}  // namespace pg::mock
