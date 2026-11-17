"""Single-reader S.BUS capture with quiet console output by default.

Usage: python3 -B test/capture_sbus.py [--echo] VERIFIED_PORT OUTPUT.txt
Type r/s/h to send a command, other text to annotate, q (or EOF) to finish.
Producer: test/sbus_diagnostic/SbusDiagnostic.ino (not the beginner calibration helper).
Requires already installed pyserial for real capture; never builds or uploads.
"""
import argparse
import csv
import datetime
from pathlib import Path
import select
import sys
import time


def capture(device, log, command_reader, *, echo=False, console=None):
    """Record exact complete serial lines; keep partial/error evidence on exit.

    command_reader returns one line, None when idle, or an empty string for EOF.
    Binary file output preserves malformed/non-ASCII bytes as well as valid CSV.
    Only partial final records use a reversible bytes representation in a comment.
    """
    console = console if console is not None else sys.stdout
    pending = bytearray()
    notes = []
    oversized = False
    records = warnings = received = 0

    def say(message):
        print(message, file=console, flush=True)

    def warn(message):
        nonlocal warnings
        warnings += 1
        if warnings <= 5:
            say("Capture warning: " + message)
        elif warnings == 6:
            say("Capture warning: further warnings suppressed; total reported at completion")

    def annotations():
        # Defer annotations until a serial line boundary, including across chunks
        # of an oversized record. A command must never split a valid CSV record.
        for note in notes:
            log.write(note)
        notes.clear()

    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
    log.write(("# HOST_OPEN " + stamp + "\n").encode("ascii"))
    say("Capturing; r/s/h send, text annotates, q or EOF stops (echo %s)" % ("on" if echo else "off"))
    try:
        while True:
            chunk = device.read(4096)
            received += len(chunk)
            pending.extend(chunk)
            while b"\n" in pending:
                line, _, pending = pending.partition(b"\n")
                log.write(line + b"\n")
                records += 1
                if echo:
                    console.write(line.decode("ascii", errors="backslashreplace") + "\n")
                if len(line) > 4096 and not oversized:
                    warn("oversized serial record preserved")
                if not oversized:
                    try:
                        text = line.decode("ascii")
                        if text.startswith("D,") and len(next(csv.reader([text], strict=True))) != 34:
                            warn("malformed CSV record preserved")
                    except (UnicodeDecodeError, csv.Error):
                        warn("malformed/non-ASCII serial record preserved")
                oversized = False
                annotations()
            if len(pending) > 4096:
                # Bound serial buffering even if the sender never emits a newline.
                # Preserve the bytes, then append later fragments to the same line.
                log.write(pending)
                if echo:
                    console.write(pending.decode("ascii", errors="backslashreplace"))
                pending.clear()
                if not oversized:
                    warn("oversized serial record preserved")
                oversized = True
            if echo and chunk:
                console.flush()
            command = command_reader()
            if command is not None:
                if command == "" or command.strip() == "q":
                    break
                command = command.strip()
                if not command:
                    continue
                note = "# HOST %.6f %s\n" % (time.monotonic(), command)
                notes.append(note.encode("utf-8"))
                say("Command: " + command if command in ("r", "s", "h") else "Annotation: " + command)
                if command in ("r", "s", "h"):
                    if device.write(command.encode("ascii")) != 1:
                        raise OSError("serial command write incomplete")
                if not pending and not oversized:
                    annotations()
    finally:
        if pending or oversized:
            if oversized:
                # Earlier fragments were already written; terminate that incomplete
                # record explicitly before the final annotation rather than hiding it.
                log.write(pending + b"\n")
            log.write(("# HOST partial final line: " + repr(bytes(pending)) + "\n").encode("ascii"))
            warn("partial final serial record; excluded from complete-record count")
        annotations()
        log.flush()
        say("Capture complete: %d bytes, %d complete serial records, %d warnings" % (received, records, warnings))
    return received, records, warnings


def main(argv=None):
    """Open one serial reader and an exclusive output file, reporting failures."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--echo", action="store_true", help="echo complete telemetry to the terminal (may slow capture)")
    parser.add_argument("port")
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    try:
        import serial
    except ImportError as error:
        print("Capture error: pyserial is required: %s" % error, file=sys.stderr)
        return 1
    try:
        def command_reader():
            return sys.stdin.readline() if select.select([sys.stdin], [], [], 0.01)[0] else None

        with args.output.open("xb") as log:
            with serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2, exclusive=True) as device:
                capture(device, log, command_reader, echo=args.echo)
    except KeyboardInterrupt:
        print("Capture interrupted by operator", file=sys.stderr)
        return 130
    except (OSError, serial.SerialException) as error:
        print("Capture error: %s" % error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
