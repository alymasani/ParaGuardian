/**
 ******************************************************************************
 * @file    SafetyMonitor.hpp
 * @brief   Runs the flight / landing safety layer once per processing frame.
 *
 * ORDER (no circular dependency, no stale state of this update):
 *   1. GroundClearanceEstimator  (LiDAR + attitude of THIS frame)
 *   2. FlightStateDetector       (navigation + clearance from step 1 + GNSS + IMU motion)
 *   3. LandingSafetyAnalyzer     (phase from step 2, clearance from step 1, navigation)
 *   4. phase-change Notice events, SafetySnapshot
 * Clearance does not depend on the phase; the phase does not depend on
 * warnings. Every value carries its own timestamp; nothing is interpolated.
 *
 * Portable (no HAL / FreeRTOS), fixed memory, bounded work per update.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_MONITOR_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_MONITOR_HPP

#include "Algorithms/Flight/FlightStateDetector.hpp"
#include "Algorithms/Landing/GroundClearanceEstimator.hpp"
#include "Algorithms/Landing/LandingSafetyAnalyzer.hpp"
#include "Algorithms/Safety/SafetyEventEmitter.hpp"
#include "Algorithms/Safety/SafetyInput.hpp"

namespace pg {

struct SafetyConfig {
    GroundClearanceConfig clearance{};
    FlightStateConfig flight{};
    LandingSafetyConfig landing{};
    bool isValid() const { return clearance.isValid() && flight.isValid() && landing.isValid(); }
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(const SafetyConfig& cfg = SafetyConfig{});

    bool configValid() const { return configValid_; }
    /// Adds an event sink (e.g. a SafetyEventQueue). Call before the scheduler starts.
    bool addSink(ISafetyEventSink* sink) { return emitter_.addSink(sink); }

    const SafetySnapshot& update(const SafetyInput& in);
    const SafetySnapshot& latest() const { return snap_; }

    const GroundClearance& clearance() const { return clearance_.latest(); }
    const FlightStateStatus& flightState() const { return flight_.latest(); }
    const SafetyEventEmitter& emitter() const { return emitter_; }
    /// The same emitter for other producers of THIS task context (the CrashDetector),
    /// so every event of SensorProcessingTask is in one event-id sequence.
    SafetyEventEmitter& eventEmitter() { return emitter_; }

private:
    bool configValid_;
    GroundClearanceEstimator clearance_;
    FlightStateDetector flight_;
    LandingSafetyAnalyzer landing_;
    SafetyEventEmitter emitter_;
    SafetySnapshot snap_{};
    FlightPhase lastPhase_ = FlightPhase::Unknown;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_MONITOR_HPP
