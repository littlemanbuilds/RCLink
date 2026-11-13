#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/test/build}"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
"$CXX" -std=c++11 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Wpedantic -I"$ROOT/src" "$ROOT/test/native/test_rclink.cpp" -o "$BUILD/test_rclink_san"
if [[ "$(uname -s)" == "Darwin" ]]; then
  ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$BUILD/test_rclink_san"
else
  ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1}" "$BUILD/test_rclink_san"
fi

"$CXX" -std=c++11 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Wpedantic -I"$ROOT/src" -I"$ROOT/test/native/arduino_stub" \
  "$ROOT/test/native/test_corrective.cpp" -o "$BUILD/test_corrective_san"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$BUILD/test_corrective_san"
SANITIZE=1 BUILD="$BUILD" bash "$ROOT/test/run_helper_tests.sh"
