#!/bin/sh
# -----------------------------------------------------------------------------
# ParaGuardian host unit tests.
#
# Compiles the PORTABLE driver/parser code (no HAL, no FreeRTOS) together with
# the tests and fakes using the laptop's C++ compiler, then runs the result.
# This is a test runner, not a firmware build system: the firmware is built
# only by STM32CubeIDE. Nothing here is compiled into the firmware (Tests/ is
# not a CubeIDE source folder).
#
# Usage (from anywhere):   sh Tests/run_host_tests.sh
# Optional:                CXX=g++ sh Tests/run_host_tests.sh
#                          SANITIZE=0 sh Tests/run_host_tests.sh       (no sanitizers)
#                          SANITIZE=thread sh Tests/run_host_tests.sh  (ThreadSanitizer)
# -----------------------------------------------------------------------------
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CXX="${CXX:-c++}"
OUT_DIR="${OUT_DIR:-${TMPDIR:-/tmp}/paraguardian_host_tests}"
mkdir -p "$OUT_DIR"

FLAGS="-std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Werror -pthread"
# SANITIZE=1 (default): AddressSanitizer + UndefinedBehaviorSanitizer
# SANITIZE=thread     : ThreadSanitizer (data races in the concurrency tests)
# SANITIZE=0          : no sanitizers
case "${SANITIZE:-1}" in
  1)      FLAGS="$FLAGS -fsanitize=address,undefined -fno-omit-frame-pointer" ;;
  thread) FLAGS="$FLAGS -fsanitize=thread -fno-omit-frame-pointer" ;;
  *)      ;;
esac

# Portable production sources (firmware-only folders BSP/, Tasks/, SysCore/ are excluded).
SOURCES="
  $ROOT/Components/Common/Status.cpp
  $ROOT/Components/Common/RegisterBus.cpp
  $ROOT/Components/Sensors/LSM6DSRX/Lsm6dsrx.cpp
  $ROOT/Components/Sensors/BMP585/Bmp585.cpp
  $ROOT/Components/Sensors/MMC5983MA/Mmc5983ma.cpp
  $ROOT/Components/Sensors/NEOM9N/UbxParser.cpp
  $ROOT/Components/Sensors/NEOM9N/NeoM9n.cpp
  $ROOT/Components/Sensors/TF02Pro/Tf02Pro.cpp
  $ROOT/Components/Storage/W25Q128/W25q128.cpp
  $ROOT/Components/Communications/Bluetooth/BleProtocol.cpp
  $ROOT/Components/Communications/Bluetooth/BleEmergencyBridge.cpp
  $ROOT/Components/Communications/Bluetooth/EzSerialLink.cpp
  $ROOT/Components/Communications/Bluetooth/SosMessage.cpp
  $ROOT/Components/Acquisition/SensorDataHub.cpp
  $ROOT/Components/Acquisition/AcquisitionSources.cpp
  $ROOT/Components/Acquisition/AcquisitionScheduler.cpp
  $ROOT/Components/Processing/SampleValidator.cpp
  $ROOT/Components/Processing/StationaryDetector.cpp
  $ROOT/Components/Processing/Calibration.cpp
  $ROOT/Components/Processing/CalibrationController.cpp
  $ROOT/Components/Processing/AxisAlignment.cpp
  $ROOT/Components/Processing/RedundancyVoter.cpp
  $ROOT/Components/Processing/PreprocessingPipeline.cpp
  $ROOT/Components/Algorithms/Attitude/MadgwickFilter.cpp
  $ROOT/Components/Algorithms/Attitude/GravityCompensator.cpp
  $ROOT/Components/Algorithms/Attitude/AttitudeEstimator.cpp
  $ROOT/Components/Algorithms/Navigation/ConstantAccelerationModel.cpp
  $ROOT/Components/Algorithms/Navigation/VerticalNavigationEstimator.cpp
  $ROOT/Components/Algorithms/Navigation/BaroAlphaBetaEstimator.cpp
  $ROOT/Components/Algorithms/Landing/GroundClearanceEstimator.cpp
  $ROOT/Components/Algorithms/Landing/LandingSafetyAnalyzer.cpp
  $ROOT/Components/Algorithms/Flight/FlightStateDetector.cpp
  $ROOT/Components/Algorithms/Safety/SafetyMonitor.cpp
  $ROOT/Components/Algorithms/Crash/CrashDetector.cpp
  $ROOT/Components/Algorithms/Emergency/EmergencyManager.cpp
  $ROOT/Components/Algorithms/Emergency/SosRequest.cpp
  $ROOT/Components/Algorithms/Safety/SafetyEventDispatcher.cpp
  $ROOT/Components/Output/Audio/AudioAlertScheduler.cpp
  $ROOT/Components/Output/LED/LedPatternScheduler.cpp
  $ROOT/Components/Storage/EmergencyJournal.cpp
  $ROOT/Components/Storage/LogRecords.cpp
  $ROOT/Components/Storage/FlightLog.cpp
  $ROOT/Components/Storage/StorageService.cpp
  $ROOT/Components/Storage/FlightRecorder.cpp
  $ROOT/Components/Power/PowerManager.cpp
  $ROOT/Components/Output/Audio/PcmToneOutput.cpp
  $ROOT/Components/Output/LED/GpioStatusLed.cpp
  $ROOT/Components/Sensors/LSM6DSRX/Lsm6dsrxFifo.cpp
  $ROOT/Components/Common/TaskSupervisor.cpp
"
TESTS="$(ls "$ROOT"/Tests/unit/*.cpp)"

echo "Compiler: $($CXX --version | head -n 1)"
echo "Flags:    $FLAGS"
# shellcheck disable=SC2086
$CXX $FLAGS -I"$ROOT/Components" -I"$ROOT/Tests" $SOURCES $TESTS -o "$OUT_DIR/paraguardian_tests"
(cd "$ROOT/Tests" && "$OUT_DIR/paraguardian_tests")
