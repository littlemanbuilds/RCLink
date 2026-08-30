#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/rclink-distribution.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT INT TERM

# Build the public candidate from an allowlist rather than trying to blacklist
# every possible private or generated working-tree file.
items=(
    .clang-format
    .github
    .gitignore
    CHANGELOG.md
    LICENSE
    README.md
    RELEASE_CHECKLIST.md
    examples
    keywords.txt
    library.json
    library.properties
    platformio.ini
    src
    test
)

tar -C "$ROOT" \
    --exclude='test/build' \
    --exclude='**/.DS_Store' \
    --exclude='**/._*' \
    --exclude='**/.native-build' \
    --exclude='**/.test-results' \
    -cf - "${items[@]}" | tar -C "$STAGE" -xf -

bash "$ROOT/test/check_release_contracts.sh" --distribution "$STAGE"
echo "distribution-contract: PASS"
