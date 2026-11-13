"""Build one consumer in protected scratch with separately provisioned prerequisites.

Arguments: source, scratch, environment, sketch, installed PlatformIO root,
and (for pico) the exact provisioned pioasm package. No installation occurs here.
"""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def digest(path):
    """Return the SHA256 of a prerequisite without modifying it."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Compile and link one selected sketch with isolated mutable build scripts."""
    source, scratch = map(Path, sys.argv[1:3])
    environment, sketch = sys.argv[3:5]
    installed = Path(sys.argv[5])
    core = Path(os.environ["PLATFORMIO_CORE_DIR"])
    assert core == scratch / "pio", "Use only the protected runner's scratch core"
    assert not Path(sketch).is_absolute() and ".." not in Path(sketch).parts
    if not (core / "penv").exists():
        (core / "penv").symlink_to(installed / "penv", target_is_directory=True)

    watched = []
    if environment == "esp32-s3-devkitc-1":
        # pioarduino 55.03.38 registers a post-size backup restore unconditionally.
        # Copy only the two mutable scripts, as in the isolated AngleSense fixture.
        # All framework archives, headers and other tools remain read-only.
        sdk = core / "packages/framework-arduinoespressif32-libs"
        original = installed / "packages/framework-arduinoespressif32-libs"
        watched = [original / "esp32s3" / name for name in
                   ("pioarduino-build.py", "pioarduino-build.py.esp32s3")]
        if sdk.is_symlink():
            assert sdk.resolve() == original.resolve()
            sdk.unlink()  # Remove only this operation-owned scratch link.
            sdk.mkdir()
            for item in original.iterdir():
                target = sdk / item.name
                if item.name != "esp32s3":
                    target.symlink_to(item, target_is_directory=item.is_dir())
                    continue
                target.mkdir()
                for child in item.iterdir():
                    if child in watched:
                        shutil.copy2(child, target / child.name)
                    else:
                        (target / child.name).symlink_to(child, target_is_directory=child.is_dir())
    elif environment == "pico":
        package = Path(sys.argv[6])
        assert (package / "package.json").is_file() and (package / ".piopm").is_file()
        target = core / "packages/tool-pioasm-rp2040-earlephilhower"
        if target.is_symlink():
            target.unlink()  # Replace only scratch discovery with the exact prerequisite.
        if not target.exists():
            target.symlink_to(package, target_is_directory=True)
        watched = [package / "package.json", package / ".piopm"]

    before = {p: digest(p) for p in watched}
    project = scratch / ("consumer-" + environment + "-" + Path(sketch).stem)
    (project / "src").mkdir(parents=True)
    library = project / "lib/RCLink"
    shutil.copytree(source / "src", library / "src")
    for name in ("library.json", "library.properties"):
        shutil.copy2(source / name, library / name)
    shutil.copy2(source / "platformio.ini", project / "platformio.ini")
    shutil.copy2(source / sketch, project / "src" / Path(sketch).name)
    dependency = Path(os.environ["PLATFORMIO_LIBDEPS_DIR"]) / environment / "ArduinoJson"
    if not dependency.exists():
        shutil.copytree(source / ".pio/libdeps/esp32-s3-devkitc-1/ArduinoJson", dependency)

    started = time.monotonic()
    command = [str(installed / "penv/bin/pio"), "run", "-d", str(project), "-e", environment]
    result = subprocess.run(command, check=False)
    assert before == {p: digest(p) for p in watched}, "Prerequisite changed"
    elf = Path(os.environ["PLATFORMIO_BUILD_DIR"]) / environment / "firmware.elf"
    if result.returncode == 0:
        assert elf.is_file() and elf.stat().st_size > 0, "Successful link must produce an ELF"
        print("LINKED_ELF", elf.stat().st_size, digest(elf), flush=True)
    print("CONSUMER", environment, sketch, "EXIT", result.returncode,
          "SECONDS", round(time.monotonic() - started, 3), flush=True)
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
