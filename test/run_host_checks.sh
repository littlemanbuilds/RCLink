#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
for CXX in "${GCC_CXX:-g++}" clang++; do
    if command -v "$CXX" >/dev/null 2>&1; then
        CXX="$CXX" bash "$ROOT/test/run_native_tests.sh"
        CXX="$CXX" bash "$ROOT/test/run_helper_tests.sh"
    fi
done
CXX=clang++ bash "$ROOT/test/run_sanitizers.sh"
bash "$ROOT/test/check_release_contracts.sh"
bash "$ROOT/test/check_distribution.sh"
echo "RCLink host checks: PASS"
