#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/test/build}"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/src" "$ROOT/test/native/test_rclink.cpp" -o "$BUILD/test_rclink"
"$BUILD/test_rclink"

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/test/native/arduino_stub" -I"$ROOT/src" \
  "$ROOT/test/native/test_corrective.cpp" -o "$BUILD/test_corrective"
"$BUILD/test_corrective"
selector_status=0
"$BUILD/test_corrective" F09 || selector_status=$?
if [[ "$selector_status" -ne 2 ]]; then
  echo "corrective runner did not reject an unknown test group with exit 2" >&2
  exit 1
fi

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/test/native/arduino_stub" -I"$ROOT/src" \
  "$ROOT/test/native/v1_1_compat.cpp" -o "$BUILD/v1_1_compat"
"$BUILD/v1_1_compat"

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -DRCLINK_NO_LEGACY_GLOBAL_ALIASES -I"$ROOT/test/native/arduino_stub" -I"$ROOT/src" \
  "$ROOT/test/native/v1_1_compat.cpp" -o "$BUILD/v1_1_namespace_only"
"$BUILD/v1_1_namespace_only"
