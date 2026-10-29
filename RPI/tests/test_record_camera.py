"""Camera-free checks for stopping safely and preserving recorded footage."""

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "record_camera.py"
SPEC = importlib.util.spec_from_file_location("record_camera", SCRIPT)
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


class FakeCamera:
    def __init__(self, stop, fail=False):
        self.stop = stop
        self.fail = fail
        self.closed = False
        self.stopped = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def start_recording(self, stream, **kwargs):
        self.stream = stream
        stream.write(b"captured frames")

    def wait_recording(self, timeout):
        if self.fail:
            raise OSError("encoder/storage failure")
        self.stop.set()

    def stop_recording(self):
        self.stopped = True
        self.stream.write(b" / flushed")


class RecorderTests(unittest.TestCase):
    def test_stop_finishes_encoder_and_releases_camera(self):
        stop = threading.Event()
        camera = FakeCamera(stop)
        stream = io.BytesIO()
        args = recorder.parse_args([])
        with mock.patch.object(stop, "wait", return_value=False), contextlib.redirect_stdout(io.StringIO()):
            recorder.record(lambda **kwargs: camera, stream, args, stop)
        self.assertTrue(camera.stopped and camera.closed)
        self.assertEqual(stream.getvalue(), b"captured frames / flushed")

    def test_encoder_error_still_finishes_and_releases_camera(self):
        stop = threading.Event()
        camera = FakeCamera(stop, fail=True)
        stream = io.BytesIO()
        with mock.patch.object(stop, "wait", return_value=False), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(OSError, "encoder/storage"):
                recorder.record(lambda **kwargs: camera, stream, recorder.parse_args([]), stop)
        self.assertTrue(camera.stopped and camera.closed)
        self.assertIn(b"captured frames", stream.getvalue())

    def test_existing_recording_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "take.h264"
            path.write_bytes(b"valuable existing recording")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(recorder.main(["--output", str(path.with_suffix(".mp4"))]), 1)
            self.assertEqual(path.read_bytes(), b"valuable existing recording")

    def test_failed_mp4_conversion_preserves_raw_video(self):
        with tempfile.TemporaryDirectory() as folder:
            raw = Path(folder) / "take.h264"
            raw.write_bytes(b"original footage")
            failure = subprocess.CompletedProcess([], 1, stderr="remux failed")
            with mock.patch.object(recorder.shutil, "which", return_value="/fake/ffmpeg"), \
                    mock.patch.object(recorder.subprocess, "run", return_value=failure), \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertFalse(recorder.make_mp4(raw, raw.with_suffix(".mp4"), 30))
            self.assertEqual(raw.read_bytes(), b"original footage")

    @unittest.skipUnless(os.name == "posix", "POSIX stop-signal integration")
    def test_stop_signals_save_file_and_exit_cleanly(self):
        # Run the actual command, substituting only the unavailable camera hardware.
        stub = '''
class PiCamera:
    def __init__(self, **kwargs): pass
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def start_recording(self, stream, **kwargs):
        self.stream = stream
        stream.write(b"frames")
        stream.flush()
    def wait_recording(self, timeout):
        import time
        time.sleep(min(timeout, 0.02))
    def stop_recording(self):
        self.stream.write(b" / STOPPED")
        self.stream.flush()
'''
        for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=stop_signal), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                (root / "picamera.py").write_text(stub)
                output = root / "take.h264"
                env = dict(os.environ, PYTHONPATH=str(root))
                process = subprocess.Popen(
                    [sys.executable, str(SCRIPT), "--output", str(output)],
                    env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                )
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        if output.exists() and output.stat().st_size:
                            break
                        if process.poll() is not None:
                            self.fail("Recorder exited before recording started")
                        time.sleep(0.02)
                    else:
                        self.fail("Recorder did not start")
                    process.send_signal(stop_signal)
                    stdout, stderr = process.communicate(timeout=5)
                    self.assertEqual(process.returncode, 0, stderr)
                    self.assertEqual(output.read_bytes(), b"frames / STOPPED")
                    self.assertIn("H.264 saved:", stdout)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()


if __name__ == "__main__":
    unittest.main()
