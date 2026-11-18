/**
 ******************************************************************************
 * @file    IPowerMonitor.hpp
 * @brief   Hardware-independent battery / power diagnostics interface.
 *
 * Battery chemistry, the ADC channel, the voltage-divider ratio and any
 * fuel-gauge IC are unknown until the schematic exists. The implementation
 * will convert an ADC reading through the divider ratio into millivolts.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_POWER_MONITOR_HPP
#define PARAGUARDIAN_POWER_MONITOR_HPP

#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

enum class PowerState : uint8_t {
    Unknown = 0,
    Normal,
    Low,        ///< warn the pilot
    Critical,   ///< reserve energy for SOS only
    External,   ///< USB / charger connected
};

struct PowerStatus {
    uint16_t batteryMillivolts;  ///< 0 = not measured
    int16_t batteryCurrentMilliamps;  ///< + discharging, 0 if no current sense
    uint8_t stateOfChargePercent;     ///< 255 = unknown
    PowerState state;
    bool measurementValid;
};

class IPowerMonitor {
public:
    virtual Status sample(PowerStatus& out) = 0;

protected:
    ~IPowerMonitor() = default;
};

/// Pure helper: classifies a voltage with thresholds supplied by the caller
/// (thresholds depend on the battery chosen; no chemistry is assumed here).
inline PowerState classifyBattery(uint16_t millivolts, uint16_t lowThresholdMv, uint16_t criticalThresholdMv)
{
    if (millivolts == 0u) return PowerState::Unknown;
    if (millivolts <= criticalThresholdMv) return PowerState::Critical;
    if (millivolts <= lowThresholdMv) return PowerState::Low;
    return PowerState::Normal;
}

}  // namespace pg

#endif  // PARAGUARDIAN_POWER_MONITOR_HPP
