#include <iomanip>
#include <iostream>
#include <string>

#include "mocks/MockBlePort.hpp"
#include "sim/CrashSimulator.hpp"

using namespace pg;

namespace {

struct Device {
    mock::BlePort port;
    ble::EmergencyBridge bridge{port, true};
    EmergencyManager manager{EmergencyConfig{}, bridge};
    uint32_t now = 0;
    bool flying = false, countdown = false;

    void tick()
    {
        bridge.receive(manager, now);
        manager.tick(now);
        bridge.transmit(manager, {}, now);
    }

    bool runFlight()
    {
        csim::Profile profile;
        profile.crashT = 40.0;
        profile.crashVz = -9.0;
        profile.postTiltDeg = 80.0;
        auto scenario = csim::baseFlight("BLE crash simulation", profile, 140);
        scenario.fs.gnss = false;
        csim::impactAtContact(scenario, 9.0, 40, 600);
        csim::Runner runner(scenario, 77);
        runner.each = [this](double, const CrashInput&, const CrashStatus& status, const SafetySnapshot& snapshot,
                             const fsim::Truth&, const SafetyInput& input) {
            flying |= snapshot.phase == FlightPhase::Flying;
            if (bridge.diagnostics().sos != 0 && !port.output.empty()) return;
            port.output.clear();   // status before the first SOS is read via the precondition flags
            now = input.nowMs;
            manager.observeCrash({status, {}, now}, now);
            tick();
            countdown |= manager.status().state == EmergencyState::Countdown;
        };
        runner.run();
        for (size_t i = 0; i < 6; ++i) { now += 20; tick(); }
        return flying && countdown && runner.suspicions > 0 && manager.status().attempts == 1;
    }

    void dump()
    {
        for (const auto& packet : port.output) {
            for (uint8_t byte : packet) std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
            std::cout << '\n';
        }
        port.output.clear();
        std::cout << "END" << std::endl;
    }

    void reconnect()
    {
        port.transition(ble::Trust::Disconnected);
        now += 20;
        tick();
        port.transition(ble::Trust::Simulation);
        for (size_t i = 0; i < 8; ++i) { now += 20; tick(); }
    }
};

}  // namespace

int main()
{
    Device device;
    if (!device.runFlight()) return 1;
    std::cout << "FLIGHT COUNTDOWN SOS" << std::endl;
    device.dump();
    for (std::string line; std::getline(std::cin, line);) {
        if (line == "reconnect") {
            device.reconnect();
            device.dump();
        } else if (line == "state") {
            const auto& s = device.manager.status();
            std::cout << std::dec << unsigned(s.state) << ' ' << s.incidentId << ' ' << s.reportedStages << std::endl;
        } else {
            if (line.empty() || line.size() % 2 != 0 || line.size() > 2 * ble::kMaxPacket) return 2;
            mock::BlePort::Bytes packet;
            for (size_t i = 0; i < line.size(); i += 2)
                packet.push_back(static_cast<uint8_t>(std::stoul(line.substr(i, 2), nullptr, 16)));
            device.port.input.push_back(packet);
            device.now += 20;
            device.tick();
        }
    }
}
