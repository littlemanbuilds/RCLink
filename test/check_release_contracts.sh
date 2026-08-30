#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="$ROOT"
STRICT_DISTRIBUTION=0

fail()
{
    echo "release-contract: $*" >&2
    exit 1
}

if [[ "${1:-}" == "--distribution" ]]; then
    [[ -n "${2:-}" ]] || fail "--distribution requires a staged directory"
    TARGET="$(cd "$2" && pwd)"
    STRICT_DISTRIBUTION=1
elif [[ $# -ne 0 ]]; then
    fail "usage: $0 [--distribution DIRECTORY]"
fi

for file in \
    .clang-format \
    .gitignore \
    CHANGELOG.md \
    LICENSE \
    README.md \
    RELEASE_CHECKLIST.md \
    keywords.txt \
    library.json \
    library.properties \
    platformio.ini; do
    [[ -f "$TARGET/$file" ]] || fail "missing $file"
done

props_ver="$(awk -F= '$1=="version"{print $2}' "$TARGET/library.properties")"
json_ver="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$TARGET/library.json")"
header_ver="$(sed -n 's/^#define RCLINK_VERSION "\([^"]*\)"/\1/p' "$TARGET/src/RCLink.h")"
[[ "$props_ver" == "$json_ver" && "$props_ver" == "$header_ver" ]] || fail "version mismatch"
[[ "$props_ver" == "1.2.0" ]] || fail "unexpected release version $props_ver"
grep -Fqx 'license=MIT' "$TARGET/library.properties" || fail "library.properties must declare MIT"

python3 - "$TARGET/library.json" <<'PY'
import json
import sys

metadata = json.load(open(sys.argv[1]))
if metadata.get("license") != "MIT":
    raise SystemExit("release-contract: library.json must declare MIT")
PY

for header in \
    src/RCLink.h \
    src/Constants.hpp \
    src/Types.hpp \
    src/Config.hpp \
    src/Link.hpp \
    src/RcMacros.hpp; do
    [[ -f "$TARGET/$header" ]] || fail "missing $header"
    grep -Fq '@file' "$TARGET/$header" || fail "missing LMB @file header in $header"
done

for heading in \
    '## Why RCLink exists' \
    '## Design boundaries' \
    '## Installation' \
    '## Supported targets' \
    '## Beginner path' \
    '## API reference' \
    '## Examples' \
    '## Testing and validation' \
    '## Deliberate limitations' \
    '## Repository structure' \
    '## Version history' \
    '## License'; do
    grep -Fq "$heading" "$TARGET/README.md" || fail "README missing $heading"
done

python3 - "$TARGET/library.json" "$TARGET" <<'PY'
import json
import pathlib
import sys

metadata = json.load(open(sys.argv[1]))
root = pathlib.Path(sys.argv[2])
examples = metadata.get("examples", [])
if len(examples) != 5:
    raise SystemExit(f"release-contract: expected 5 examples, found {len(examples)}")
missing = [example for example in examples if not (root / example).is_file()]
if missing:
    raise SystemExit("release-contract: missing metadata example: " + ", ".join(missing))
PY

# Established safety/health contracts that must remain represented in the public headers.
grep -Fq 'kInvalidChannel' "$TARGET/src/Constants.hpp" || fail "invalid-channel sentinel missing"
grep -Fq 'required_roles_valid' "$TARGET/src/Types.hpp" || fail "role-validity evidence missing"
grep -Fq 'channel_valid_mask' "$TARGET/src/Types.hpp" || fail "channel-validity bitmap missing"
grep -Fq 'fsig_matching_' "$TARGET/src/Link.hpp" || fail "failsafe-signature state missing"
grep -Fq 'sw_observed_mask_' "$TARGET/src/Link.hpp" || fail "switch-observation evidence missing"
grep -Fq 'last_good_' "$TARGET/src/Link.hpp" || fail "HoldLast history missing"
grep -Fq 'set_switch_learning_enabled' "$TARGET/src/Link.hpp" || fail "switch-learning commissioning gate missing"

# RCLink deliberately supports representative non-ESP32 targets. The routine local ESP32 target is S3 only.
grep -Eq '^src_dir[[:space:]]*=[[:space:]]*test/portable_compile$' "$TARGET/platformio.ini" ||
    fail "default PlatformIO source must compile the portable smoke sketch"
grep -Eq '^lib_extra_dirs[[:space:]]*=[[:space:]]*\.$' "$TARGET/platformio.ini" ||
    fail "default PlatformIO build must register the repository library"
grep -Fq '[env:esp32-s3-devkitc-1]' "$TARGET/platformio.ini" ||
    fail "ESP32-S3 reference environment missing"
! grep -Fq '[env:esp32dev]' "$TARGET/platformio.ini" ||
    fail "redundant classic ESP32 environment remains"
grep -Fq '[env:due]' "$TARGET/platformio.ini" ||
    fail "SAM/Due representative target missing"
grep -Fq 'platform = ststm32@19.7.1' "$TARGET/platformio.ini" ||
    fail "STM32 representative target must use the validated ststm32 19.7.1 pin"

ci="$TARGET/.github/workflows/ci.yml"
grep -Fq 'platformio==6.1.19' "$ci" || fail "CI must pin PlatformIO Core 6.1.19"
grep -Fq 'actions/checkout@v5' "$ci" || fail "CI must use the current checkout action generation"
grep -Fq 'actions/setup-python@v6' "$ci" || fail "CI must use the current setup-python action generation"
grep -Fq 'pio run -e "${{ matrix.env }}"' "$ci" ||
    fail "CI portable matrix must use the repository pio run build contract"
! grep -Fq 'pio ci --project-conf' "$ci" ||
    fail "CI must not use the obsolete pio ci build contract"

if grep -Eq '^[[:space:]]*test/?[[:space:]]*$' "$TARGET/.gitignore"; then
    fail ".gitignore must not ignore committed test/"
fi

if [[ "$STRICT_DISTRIBUTION" -eq 1 ]]; then
    if find "$TARGET" -type f \( \
        -name '.DS_Store' -o -name '._*' -o -name '*.zip' -o -name '*.o' -o \
        -name '*.obj' -o -name '*.elf' -o -name '*.bin' -o -name '*.log' \
    \) | grep -q .; then
        fail "distribution contains generated, OS, archive, or build files"
    fi

    if find "$TARGET" -type d \( \
        -name '__MACOSX' -o -name '.pio' -o -name '.vscode' -o -name '.git' -o \
        -name '.lmb' -o -name '.native-build' -o -name '.test-results' -o -name build \
    \) | grep -q .; then
        fail "distribution contains private/generated directories"
    fi
fi

echo "release-contract: PASS"
