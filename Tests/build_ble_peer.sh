#!/bin/sh
# Host simulator for the Android cross-language test. No embedded build settings.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-${TMPDIR:-/tmp}/paraguardian_ble_peer}"
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wshadow -Werror \
  -I"$ROOT/Components" -I"$ROOT/Tests" \
  "$ROOT/Tests/sim/ble_peer.cpp" \
  "$ROOT/Components/Algorithms/Crash/CrashDetector.cpp" \
  "$ROOT/Components/Algorithms/Emergency/EmergencyManager.cpp" \
  "$ROOT/Components/Algorithms/Emergency/SosRequest.cpp" \
  "$ROOT/Components/Algorithms/Flight/FlightStateDetector.cpp" \
  "$ROOT/Components/Algorithms/Landing/GroundClearanceEstimator.cpp" \
  "$ROOT/Components/Algorithms/Landing/LandingSafetyAnalyzer.cpp" \
  "$ROOT/Components/Algorithms/Safety/SafetyMonitor.cpp" \
  "$ROOT/Components/Communications/Bluetooth/BleProtocol.cpp" \
  "$ROOT/Components/Communications/Bluetooth/BleEmergencyBridge.cpp" \
  -o "$OUT"
