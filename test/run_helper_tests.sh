#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/test/build}"
CXX="${CXX:-c++}"
JSON="${ARDUINOJSON_INCLUDE:-$ROOT/.pio/libdeps/esp32-s3-devkitc-1/ArduinoJson/src}"
[[ -f "$JSON/ArduinoJson.h" ]] || { echo "NOT_RUN: ArduinoJson 7.x headers missing: $JSON" >&2; exit 2; }
mkdir -p "$BUILD"
flags=(-std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
if [[ "${SANITIZE:-0}" == 1 ]]; then
    flags+=(-g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined)
    # LeakSanitizer is unsupported on macOS; enable it on Linux unless overridden.
    if [[ "$(uname -s)" == "Darwin" ]]; then
      export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}"
    else
      export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1}"
    fi
fi
"$CXX" "${flags[@]}" -I"$ROOT/src" -isystem "$JSON" \
    "$ROOT/test/native/test_json.cpp" -o "$BUILD/test_json"
"$BUILD/test_json"
"$CXX" "${flags[@]}" -I"$ROOT/src" -I"$ROOT/test/native/calibration_stub" \
    "$ROOT/test/native/test_calibration.cpp" -o "$BUILD/test_calibration"
"$BUILD/test_calibration"

"$CXX" "${flags[@]}" -I"$ROOT/src" -I"$ROOT/test/native/calibration_stub" -isystem "$JSON" \
    "$ROOT/test/native/test_sbus_calibration.cpp" -o "$BUILD/test_sbus_calibration"
"$BUILD/test_sbus_calibration"

# The old public Stats aggregate accepted member initialization in C++14.
"$CXX" -std=c++14 -Wall -Wextra -Wpedantic -Werror \
    -I"$ROOT/src" -I"$ROOT/test/native/calibration_stub" \
    "$ROOT/test/native/calibration_compat.cpp" -o "$BUILD/calibration_compat"
"$BUILD/calibration_compat"
