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
# Negative probes belong only to this disposable stage, never the user's tree.
for member in src/AGENTS.md src/credentials.json src/.env src/leaked.o src/RCLINK_140_FINALIZATION_REPORT.md src/RCLINK_140_CALIBRATION_FINAL_REPORT.md src/RCLINK_140_RELEASE_ASSURANCE.md; do
    touch "$STAGE/$member"
    if bash "$ROOT/test/check_release_contracts.sh" --distribution "$STAGE" >/dev/null 2>&1; then
        echo "distribution-contract: failed to reject $member" >&2
        exit 1
    fi
    rm "$STAGE/$member"
done
for member in .lmb .codex .git .aws .pio; do
    mkdir "$STAGE/src/$member"
    if bash "$ROOT/test/check_release_contracts.sh" --distribution "$STAGE" >/dev/null 2>&1; then
        echo "distribution-contract: failed to reject $member" >&2
        exit 1
    fi
    rmdir "$STAGE/src/$member"
done
# Release identity negatives operate only on disposable public files.
for member in library.properties library.json src/RCLink.h README.md CHANGELOG.md RELEASE_CHECKLIST.md; do
    cp "$STAGE/$member" "$STAGE/version-original"
    python3 -B -c 'from pathlib import Path; import sys
p=Path(sys.argv[1]); p.write_text(p.read_text().replace("1.4.0","1.4.1"))' "$STAGE/$member"
    if bash "$ROOT/test/check_release_contracts.sh" --distribution "$STAGE" >/dev/null 2>&1; then
        echo "distribution-contract: failed to reject version drift in $member" >&2
        exit 1
    fi
    mv "$STAGE/version-original" "$STAGE/$member"
done
cp "$STAGE/src/RCLink.h" "$STAGE/version-original"
python3 -B -c 'from pathlib import Path; import sys
p=Path(sys.argv[1]); p.write_text(p.read_text().replace("RCLINK_VERSION_MINOR 4","RCLINK_VERSION_MINOR 5"))' "$STAGE/src/RCLink.h"
if bash "$ROOT/test/check_release_contracts.sh" --distribution "$STAGE" >/dev/null 2>&1; then
    echo "distribution-contract: failed to reject numeric version drift" >&2
    exit 1
fi
mv "$STAGE/version-original" "$STAGE/src/RCLink.h"
echo "distribution-contract: PASS"
