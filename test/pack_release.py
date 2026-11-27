"""Prepare RCLink release references from an exact Git commit in protected scratch."""

import argparse
import json
import re
import subprocess
import sys
import tarfile
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def run(*argv, **kwargs):
    return subprocess.run(argv, check=True, **kwargs)


def platformio_files(source, manifest):
    files = manifest["export"]["include"]
    if not files or files != sorted(set(files)) or any(
        not re.fullmatch(r"[A-Za-z0-9_./-]+", name)
        or any(part in {"", ".", ".."} for part in name.split("/"))
        or not (source / name).is_file()
        for name in files
    ):
        raise ValueError("PlatformIO export must list existing, unique literal files")
    return files


def pack_platformio(source, archive):
    from platformio import __version__
    from platformio.package.pack import PackagePacker

    if __version__ != "6.1.19":
        raise ValueError("RCLink qualification requires PlatformIO Core 6.1.19")
    manifest = json.loads((source / "library.json").read_text())
    files = platformio_files(source, manifest)
    # Use Core's actual archive and export filters without its live SPDX fetch.
    # The release contract separately checks the committed MIT metadata.
    packer = PackagePacker(str(source))
    packer.create_tarball(str(source), archive, manifest)
    with tarfile.open(archive, "r:gz") as package:
        members = package.getmembers()
        if sorted(member.name for member in members) != files:
            raise ValueError("PlatformIO package inventory differs from the export list")
        for member in members:
            path = source / member.name
            if (not member.isfile()
                or bool(member.mode & 0o111) != bool(path.stat().st_mode & 0o111)
                or package.extractfile(member).read() != path.read_bytes()):
                raise ValueError(f"PlatformIO package differs from source at {member.name}")
    return files


def archive_commit(repository, commit, output, checker):
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("Provide the full release commit SHA")
    if output.is_relative_to(repository):
        raise ValueError("Release output must be outside the source checkout")
    git = ("/usr/bin/git", "-C", repository)
    if run(*git, "rev-parse", "--verify", commit + "^{commit}",
           capture_output=True, text=True).stdout.strip() != commit:
        raise ValueError("Release SHA must identify a commit")
    run(*git, "diff-index", "--cached", "--quiet", commit, "--")

    output.mkdir(mode=0o700)
    source = output / "source"
    source.mkdir()
    raw_source = output / "source.tar"
    run(*git, "archive", "--format=tar", "--output", raw_source, commit)
    with tarfile.open(raw_source) as archive:
        members = archive.getmembers()
        if any(not (item.isfile() or item.isdir()) for item in members):
            raise ValueError("Release source contains a link or special Git member")
        archive.extractall(source, filter="data")
    raw_source.unlink()

    manifest = json.loads((source / "library.json").read_text())
    version = manifest["version"]
    prefix = f"RCLink-{version}/"
    tag = f"v{version}"
    github = sorted(member.name for member in members if member.isfile())
    platformio = platformio_files(source, manifest)
    if set(github) != set(platformio) | {".clang-format", ".gitignore"}:
        raise ValueError("The committed GitHub tree differs from RCLink's public file projection")
    arduino = [name for name in github if not name.startswith(".")]
    run("/bin/bash", source / "test/check_release_contracts.sh", "--distribution", source)

    def destination(channel, fmt, files, packager, **extra):
        return dict(channel=channel, format=fmt, files=files, packager=packager,
                    archive=str(output / f"{channel}.{fmt}"), **extra)

    policy = dict(github="littlemanbuilds/RCLink", tag=tag, version=version,
                  applicability={name: dict(status="publish", reference="RCLink README installation")
                                 for name in ("platformio", "arduino")})
    policy["destinations"] = {
        "github-zip": destination("github-zip", "zip", github, "git archive"),
        "github-tar": destination("github-tar", "tar.gz", github, "git archive"),
        "platformio": destination("platformio", "tar.gz", platformio,
                                  "PlatformIO Core 6.1.19 + LMB normalizer",
                                  registry="registry.platformio.org", type="library",
                                  owner="littlemanbuilds", name="RCLink"),
        "arduino": destination("arduino", "zip", arduino, "Python zipfile", name="RCLink"),
    }
    for channel in ("github-zip", "github-tar"):
        item = policy["destinations"][channel]
        run(*git, "archive", f"--format={item['format']}", f"--prefix={prefix}",
            "--output", item["archive"], commit)
    with ZipFile(policy["destinations"]["arduino"]["archive"], "x", ZIP_DEFLATED) as archive:
        for name in arduino:
            archive.write(source / name, prefix + name)
    raw = output / "platformio-raw.tar.gz"
    pack_platformio(source, raw)
    run(sys.executable, "-B", checker, "normalize-platformio", raw,
        policy["destinations"]["platformio"]["archive"])
    raw.unlink()
    (output / "plan.json").write_text(json.dumps(policy, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    release = commands.add_parser("commit")
    for name in ("repository", "commit", "output", "checker"):
        release.add_argument(name)
    probe = commands.add_parser("check-platformio")
    for name in ("source", "archive"):
        probe.add_argument(name, type=Path)
    args = parser.parse_args()
    if args.command == "commit":
        archive_commit(Path(args.repository).resolve(), args.commit, Path(args.output).resolve(),
                       Path(args.checker).resolve())
    else:
        pack_platformio(args.source, args.archive)


if __name__ == "__main__":
    main()
