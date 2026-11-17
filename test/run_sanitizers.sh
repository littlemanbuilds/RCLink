#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/test/build}"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
# LeakSanitizer is unsupported on macOS; enable it on Linux unless overridden.
if [[ "$(uname -s)" == "Darwin" ]]; then
  export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}"
else
  export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1}"
fi
"$CXX" -std=c++11 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Wpedantic -I"$ROOT/src" "$ROOT/test/native/test_rclink.cpp" -o "$BUILD/test_rclink_san"
"$BUILD/test_rclink_san"

"$CXX" -std=c++11 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Wpedantic -I"$ROOT/src" -I"$ROOT/test/native/arduino_stub" \
  "$ROOT/test/native/test_corrective.cpp" -o "$BUILD/test_corrective_san"
"$BUILD/test_corrective_san"
SANITIZE=1 BUILD="$BUILD" bash "$ROOT/test/run_helper_tests.sh"

"$CXX" -std=c++11 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Wpedantic -I"$ROOT/src" -I"$ROOT/test/native/arduino_stub" \
  "$ROOT/test/native/test_sbus.cpp" -o "$BUILD/test_sbus_san"
"$BUILD/test_sbus_san"
