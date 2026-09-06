#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
for CXX in g++ clang++; do command -v "$CXX" >/dev/null 2>&1 && CXX="$CXX" bash "$ROOT/test/run_native_tests.sh"; done
CXX=g++ bash "$ROOT/test/run_sanitizers.sh"
bash "$ROOT/test/check_release_contracts.sh"
bash "$ROOT/test/check_distribution.sh"
echo "RCLink host checks: PASS"
