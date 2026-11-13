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
fi
"$CXX" "${flags[@]}" -I"$ROOT/src" -isystem "$JSON" \
    "$ROOT/test/native/test_json.cpp" -o "$BUILD/test_json"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$BUILD/test_json"
"$CXX" "${flags[@]}" -I"$ROOT/src" -I"$ROOT/test/native/calibration_stub" \
    "$ROOT/test/native/test_calibration.cpp" -o "$BUILD/test_calibration"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$BUILD/test_calibration"
