"""Focused synthetic serial tests; no pyserial, hardware or new dependencies."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location("capture_sbus", Path(__file__).with_name("capture_sbus.py"))
capture_sbus = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture_sbus)
CSV = b"D," + b",".join([b"1"] * 33) + b"\n"


class SerialFixture:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.commands = []
        self.closed = False

    def read(self, size):
        assert size == 4096
        value = next(self.chunks, b"")
        if isinstance(value, Exception):
            raise value
        return value

    def write(self, value):
        self.commands.append(value)
        return len(value)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True


class CaptureTests(unittest.TestCase):
    def run_capture(self, chunks, commands, **kwargs):
        device, log, console = SerialFixture(chunks), io.BytesIO(), io.StringIO()
        inputs = iter(commands)
        result = capture_sbus.capture(device, log, lambda: next(inputs, "q\n"), console=console, **kwargs)
        return device, log.getvalue(), console.getvalue(), result

    def test_quiet_preserves_every_complete_record(self):
        _, data, console, result = self.run_capture([CSV * 50, CSV[:10], CSV[10:]], [None, None, "q\n"])
        self.assertEqual([line + b"\n" for line in data.splitlines() if line.startswith(b"D,")], [CSV] * 51)
        self.assertNotIn("D,", console)
        self.assertEqual(result, (len(CSV) * 51, 51, 0))
        self.assertIn("echo off", console)

    def test_echo_and_commands_do_not_split_csv(self):
        device, data, console, _ = self.run_capture([CSV[:5], CSV[5:], CSV, CSV, CSV], ["s\n", "an endpoint\n", "r\n", "h\n", "q\n"], echo=True)
        self.assertEqual(device.commands, [b"s", b"r", b"h"])
        self.assertEqual(data.count(CSV), 4)
        self.assertEqual(console.count(CSV.decode()), 4)
        self.assertIn(b"# HOST ", data)
        self.assertIn(b"an endpoint\n", data)
        self.assertIn("Command: s", console)
        self.assertIn("Annotation: an endpoint", console)

    def test_partial_eof_and_blank_input(self):
        _, data, console, result = self.run_capture([b"D,partial", b""], ["\n", ""])
        self.assertIn(b"# HOST partial final line: b'D,partial'", data)
        self.assertEqual(result, (9, 0, 1))
        self.assertIn("partial final", console)

    def test_oversized_malformed_and_non_ascii_are_preserved(self):
        body = b"x" * 4096
        _, data, console, result = self.run_capture([body, b"xx", body, b"\nD,bad\n\xff\n"], [None, "note while oversized\n", None, "q\n"])
        self.assertIn(body + b"xx" + body + b"\n", data)
        self.assertIn(b"D,bad\n\xff\n", data)
        self.assertEqual(result[2], 3)
        self.assertIn("oversized", console)
        self.assertIn("malformed CSV", console)
        self.assertIn("non-ASCII", console)

    def test_serial_error_keeps_partial_and_summary(self):
        device, log, console = SerialFixture([b"D,partial", OSError("cable gone")]), io.BytesIO(), io.StringIO()
        with self.assertRaisesRegex(OSError, "cable gone"):
            capture_sbus.capture(device, log, lambda: None, console=console)
        self.assertIn(b"partial final", log.getvalue())
        self.assertIn("Capture complete", console.getvalue())

    def test_main_exclusive_file_cleanup_and_error_diagnostic(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "capture.txt"
            device = SerialFixture([CSV])
            module = types.SimpleNamespace(Serial=mock.Mock(return_value=device), SerialException=OSError)
            with mock.patch.dict("sys.modules", serial=module), mock.patch.object(capture_sbus.select, "select", return_value=([object()], [], [])), mock.patch.object(capture_sbus.sys, "stdin", io.StringIO("q\n")), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(capture_sbus.main(["PORT", str(output)]), 0)
            self.assertTrue(device.closed)
            module.Serial.assert_called_once_with("PORT", 115200, timeout=0, write_timeout=0.2, exclusive=True)
            saved = output.read_bytes()
            errors = io.StringIO()
            with mock.patch.dict("sys.modules", serial=module), contextlib.redirect_stderr(errors):
                self.assertEqual(capture_sbus.main(["PORT", str(output)]), 1)
            self.assertEqual(output.read_bytes(), saved)
            self.assertIn("Capture error:", errors.getvalue())

    def test_missing_dependency_has_actionable_error(self):
        errors = io.StringIO()
        with mock.patch.dict("sys.modules", serial=None), contextlib.redirect_stderr(errors):
            self.assertEqual(capture_sbus.main(["PORT", "unused.txt"]), 1)
        self.assertIn("pyserial is required", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
