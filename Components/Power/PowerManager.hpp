/**
 ******************************************************************************
 * @file    PowerManager.hpp
 * @brief   Portable battery / charging interpretation and peripheral power
 *          policy (5 V rail for LiDAR + audio, MAX98357A amplifier enable).
 *
 * NOTHING HERE ASSUMES HARDWARE. Battery chemistry, the voltage divider, the
 * ADC channel, the charger IC and its status-pin polarity, the regulator and
 * amplifier enable pins and their polarity are UNKNOWN until the schematic
 * exists. Therefore:
 *   - inputs arrive already converted and polarity-corrected (PowerInputs),
 *     produced by a board adapter that does not exist yet;
 *   - the battery thresholds default to 0 = "not configured": the level is
 *     then Unknown, never "Healthy"; missing data is never turned into a
 *     full battery;
 *   - the outputs are IPowerSwitch objects; the firmware default is
 *     UnconfiguredPowerSwitch (reports "not configured", does nothing).
 *
 * VOLTAGE IS NOT STATE OF CHARGE. Without a characterised discharge curve
 * (or a fuel gauge) a voltage only supports coarse Low / Critical warnings.
 * stateOfChargePercent is reported only from a real gauge input.
 *
 * POLICY (PowerPolicyConfig, conservative defaults):
 *   - LiDAR stays powered by default (it feeds flight-state detection,
 *     landing warnings, crash context and terrain proximity; it needs warm-up).
 *     It may be shed ONLY on Critical battery, on the ground, with no open
 *     emergency, and only if allowLidarShedOnGround is set.
 *   - The amplifier is enabled only while audio is requested (plus a hold
 *     time so beeps do not toggle it), and only when the 5 V rail is ready.
 *   - Nothing needed by an active emergency (amplifier for SOS audio, rail)
 *     is ever powered down by the battery policy.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_POWER_POWER_MANAGER_HPP
#define PARAGUARDIAN_POWER_POWER_MANAGER_HPP

#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

/// A digital status input after polarity correction (Unknown = not wired / not read).
enum class Signal : uint8_t { Unknown = 0, Inactive, Active };

struct PowerInputs {
    uint32_t sampleMs;
    uint16_t batteryMillivolts;   ///< after divider scaling
    bool batteryValid;            ///< ADC conversion succeeded and is in a plausible range
    uint8_t gaugePercent;         ///< only from a real fuel gauge
    bool gaugeValid;
    Signal charging;              ///< charger "charging" status output
    Signal chargeComplete;        ///< charger "done" output
    Signal supplyPresent;         ///< external supply / USB detect
};

enum class BatteryLevel : uint8_t { Unknown = 0, Healthy, Low, Critical };
enum class ChargeState : uint8_t { Unknown = 0, NotCharging, Charging, Complete, Fault };
enum class SupplyState : uint8_t { Unknown = 0, Disconnected, Present };
enum class RailState : uint8_t { Unconfigured = 0, Off, Starting, Ready, Fault };

namespace powerflag {
enum : uint8_t {
    ThresholdsUnconfigured = 1u << 0,   ///< no battery thresholds: level always Unknown
    StaleMeasurement       = 1u << 1,   ///< no valid reading for maxSampleAgeMs
    ChargerConflict        = 1u << 2,   ///< "charging" and "complete" both active
    LevelHeldWhileCharging = 1u << 3,   ///< charger voltage would look healthier than the cell
    LidarShed              = 1u << 4,
    AudioRestricted        = 1u << 5,   ///< critical battery: only high-priority audio
};
}

struct BatteryConfig {
    uint16_t lowMv = 0u;          ///< 0 = not configured (needs the battery's datasheet + characterisation)
    uint16_t criticalMv = 0u;
    uint16_t hysteresisMv = 50u;
    uint8_t confirmSamples = 3u;  ///< consecutive samples before a level change (noise / load spikes)
    uint32_t maxSampleAgeMs = 5000u;
    bool isValid() const { return (lowMv == 0u && criticalMv == 0u) || (criticalMv > 0u && lowMv > criticalMv); }
};

/// Output switch (regulator enable, amplifier SD_MODE / enable). set() must not block.
class IPowerSwitch {
public:
    virtual Status set(bool on) = 0;
    virtual bool configured() const = 0;
    /// Power-good / fault feedback if the hardware has it (Unknown otherwise).
    virtual Signal powerGood() const { return Signal::Unknown; }

protected:
    ~IPowerSwitch() = default;
};

/// Firmware default: enable pin / polarity unknown.
class UnconfiguredPowerSwitch final : public IPowerSwitch {
public:
    Status set(bool) override { return Status::NotInitialized; }
    bool configured() const override { return false; }
};

struct RailConfig {
    uint32_t settleMs = 20u;      ///< after enable, before consumers may use it (placeholder: measure on hardware)
    uint32_t offDelayMs = 2000u;  ///< no request this long before switching off (avoid toggling)
    uint32_t faultRetryMs = 5000u;
    bool defaultOn = false;       ///< safe default state at start-up before policy runs
};

/// One switched supply with request bits, settle time, hold time and fault detection.
class PowerRail {
public:
    PowerRail(IPowerSwitch* sw, const RailConfig& cfg) : sw_(sw), cfg_(cfg) {}
    void start(uint32_t nowMs);
    /// @param wanted true = at least one consumer needs it (after the off delay is applied here).
    void update(bool wanted, uint32_t nowMs);
    /// Immediately off (the supply it depends on went away), no hold time.
    void forceOff(uint32_t nowMs);
    RailState state() const { return state_; }
    bool ready() const { return state_ == RailState::Ready; }
    uint32_t switches() const { return switches_; }
    uint32_t faults() const { return faults_; }

private:
    IPowerSwitch* sw_;
    RailConfig cfg_;
    RailState state_ = RailState::Unconfigured;
    uint32_t sinceMs_ = 0u;
    uint32_t lastWantedMs_ = 0u;
    uint32_t switches_ = 0u;
    uint32_t faults_ = 0u;
};

struct PowerPolicyConfig {
    bool lidarOnByDefault = true;          ///< keep the LiDAR powered (validated policy pending)
    bool allowLidarShedOnGround = false;   ///< shed LiDAR on Critical battery on the ground only
    uint32_t ampHoldMs = 1500u;            ///< keep the amplifier on this long after the last audio request
    uint8_t criticalAudioMinPriority = 80u;///< Critical battery, no emergency: quieter alerts suppressed
};

/// What the rest of the system needs to know about power (latest value, trivially copyable).
struct PowerSnapshot {
    uint32_t sequence;
    uint32_t timestampMs;
    BatteryLevel level;
    ChargeState charge;
    SupplyState supply;
    uint16_t millivolts;          ///< 0 = unknown
    uint8_t stateOfChargePercent; ///< 255 = unknown (only from a gauge)
    uint8_t flags;                ///< powerflag::*
    RailState rail5v;
    RailState amplifier;
    bool lidarPowerRequested;
    bool audioReady;              ///< amplifier enabled AND settled (else audio cannot be heard)
    uint32_t levelChanges;
    uint32_t railFaults;
};

struct PowerContext {
    bool inFlight;
    bool emergencyActive;
    bool audioRequested;          ///< the audio scheduler wants to play
};

class PowerManager {
public:
    PowerManager(const BatteryConfig& battery, const PowerPolicyConfig& policy, IPowerSwitch* rail5v,
                 IPowerSwitch* amplifier, const RailConfig& railCfg = RailConfig{},
                 const RailConfig& ampCfg = RailConfig{});

    bool configValid() const { return battery_.isValid(); }
    void start(uint32_t nowMs);
    const PowerSnapshot& update(const PowerInputs& in, const PowerContext& ctx, uint32_t nowMs);
    const PowerSnapshot& snapshot() const { return snap_; }
    /// Whether an alert of this priority may use the speaker under the current battery policy.
    bool audioAllowed(uint8_t alertPriority, bool emergencyActive) const;

private:
    void updateLevel(const PowerInputs& in, uint32_t nowMs);
    void updateCharge(const PowerInputs& in);

    BatteryConfig battery_;
    PowerPolicyConfig policy_;
    PowerRail rail_;
    PowerRail amp_;
    PowerSnapshot snap_{};
    bool haveSample_ = false;
    uint32_t lastValidMs_ = 0u;
    BatteryLevel candidate_ = BatteryLevel::Unknown;
    uint8_t candidateCount_ = 0u;
    uint32_t lastAudioMs_ = 0u;
    bool audioSeen_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_POWER_POWER_MANAGER_HPP
