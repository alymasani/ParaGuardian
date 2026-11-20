#!/bin/sh
# -----------------------------------------------------------------------------
# ParaGuardian: one entry point for every reproducible check that does not need
# hardware. Each step can be run on its own:
#
#   sh Tools/check.sh firmware      host unit tests (ASan + UBSan)
#   sh Tools/check.sh tsan          host unit tests under ThreadSanitizer
#   sh Tools/check.sh plain         host unit tests without sanitizers (fast)
#   sh Tools/check.sh backend       emergency backend tests (Python 3.9+, stdlib only)
#   sh Tools/check.sh decoder       flash-log decoder tests (golden vector shared with firmware)
#   sh Tools/check.sh android       Android JVM tests + lint + debug APK (needs JDK 17 + Android SDK)
#   sh Tools/check.sh cubeide       native STM32CubeIDE Debug + Release headless builds of a scratch copy
#   sh Tools/check.sh all           firmware, backend, decoder (+ android / cubeide when available)
#
# Nothing here sends a real message: the backend tests use the fake provider and loopback HTTP.
# STM32CubeIDE remains the firmware build system; the cubeide step only drives it headlessly.
# -----------------------------------------------------------------------------
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STEP="${1:-all}"
TMP="${TMPDIR:-/tmp}/paraguardian_check"
mkdir -p "$TMP"

firmware() { OUT_DIR="$TMP/asan" sh "$ROOT/Tests/run_host_tests.sh" | tail -n 3; }
plain()    { SANITIZE=0 OUT_DIR="$TMP/plain" sh "$ROOT/Tests/run_host_tests.sh" | tail -n 3; }
tsan()     { SANITIZE=thread OUT_DIR="$TMP/tsan" sh "$ROOT/Tests/run_host_tests.sh" | tail -n 3; }
backend()  { (cd "$ROOT/Backend" && python3 -m unittest discover -s tests -t .); }
decoder()  { (cd "$ROOT/Tools/pglog" && python3 -m unittest test_pglog_decode); }
android()  {
  if ! java -version >/dev/null 2>&1; then echo "android: SKIPPED (no Java runtime; set JAVA_HOME to a JDK 17)"; return 0; fi
  (cd "$ROOT/Android/ParaGuardian" && ./gradlew testDebugUnitTest lintDebug assembleDebug)
}
cubeide()  {
  IDE="${CUBEIDE:-/Applications/STM32CubeIDE.app/Contents/MacOS/STM32CubeIDE}"
  if [ ! -x "$IDE" ]; then echo "cubeide: SKIPPED (STM32CubeIDE not found; set CUBEIDE)"; return 0; fi
  WS="$TMP/cube"; rm -rf "$WS"; mkdir -p "$WS/.ws"
  rsync -a --exclude Android --exclude Backend --exclude Debug --exclude Release --exclude .git "$ROOT/" "$WS/ParaGuardian/"
  for cfg in Debug Release; do
    "$IDE" --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
      -data "$WS/.ws" -import "$WS/ParaGuardian" -build "ParaGuardian/$cfg" > "$WS/$cfg.log" 2>&1 || true
    grep "Build Finished" "$WS/$cfg.log" | tail -n 1
    grep -A1 "   text" "$WS/$cfg.log" | tail -n 1
  done
}

case "$STEP" in
  firmware|plain|tsan|backend|decoder|android|cubeide) "$STEP" ;;
  all) firmware; backend; decoder; android; cubeide ;;
  *) sed -n 2,20p "$0"; exit 2 ;;
esac
