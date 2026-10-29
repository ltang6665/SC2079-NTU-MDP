#!/usr/bin/env python3


import argparse
from datetime import datetime
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o", "--output", type=Path,
        help="Output .mp4 or .h264 path; default: recordings/camera_<timestamp>.mp4 beside this script",
    )
    parser.add_argument("--width", type=positive_int, default=1920)
    parser.add_argument("--height", type=positive_int, default=1080)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--rotation", type=int, choices=(0, 90, 180, 270), default=180)
    parser.add_argument("--bitrate", type=positive_int, default=17_000_000, help="H.264 bits/second (default: 17000000)")
    parser.add_argument("--preview", action="store_true", help="Show a preview on a display attached to the Pi")
    args = parser.parse_args(argv)
    if args.bitrate > 25_000_000:
        parser.error("--bitrate must be at most 25000000 for this PiCamera encoder")
    if args.output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        args.output = Path(__file__).resolve().parent / "recordings" / ("camera_" + stamp + ".mp4")
    args.output = args.output.expanduser().resolve()
    if args.output.suffix.lower() not in (".mp4", ".h264"):
        parser.error("--output must end in .mp4 or .h264")
    return args


def record(camera_factory, stream, args, stop):
    with camera_factory(resolution=(args.width, args.height), framerate=args.fps) as camera:
        camera.rotation = args.rotation
        if args.preview:
            camera.start_preview()
        print("Camera warming up...", flush=True)
        if stop.wait(2.0):
            return 0.0
        camera.start_recording(
            stream, format="h264", bitrate=args.bitrate, quality=23, inline_headers=True,
        )
        started = time.monotonic()
        next_status = started + 5.0
        print("RECORDING — press Ctrl+C to stop and save.", flush=True)
        try:
            while not stop.is_set():
                camera.wait_recording(0.25)
                now = time.monotonic()
                if now >= next_status:
                    elapsed = int(now - started)
                    print("REC {:02d}:{:02d}".format(elapsed // 60, elapsed % 60), flush=True)
                    next_status = now + 5.0
        finally:
            camera.stop_recording()
        return time.monotonic() - started


def remux_command(ffmpeg, raw_path, mp4_path, fps):
    return [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
        "-r", str(fps), "-f", "h264", "-i", str(raw_path),
        "-c:v", "copy", "-an", "-movflags", "+faststart", str(mp4_path),
    ]


def make_mp4(raw_path, mp4_path, fps):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        print("FFmpeg is not installed; your video is saved as H.264.")
        print("To create an MP4 later on a machine with FFmpeg, run:")
        print(" ".join(shlex.quote(part) for part in remux_command("ffmpeg", raw_path, mp4_path, fps)))
        return False
    print("Creating MP4 — wait for the saved message...", flush=True)
    try:
        result = subprocess.run(
            remux_command(ffmpeg, raw_path, mp4_path, fps),
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
            start_new_session=True,
        )
    except OSError as exc:
        print("Could not start FFmpeg: {}. Raw video is still saved.".format(exc), file=sys.stderr)
        return False
    if result.returncode != 0:
        print("MP4 creation failed; raw video is still saved.", file=sys.stderr)
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        return False
    print("MP4 saved: {}".format(mp4_path), flush=True)
    return True


def main(argv=None):
    args = parse_args(argv)
    raw_path = args.output.with_suffix(".h264")
    for path in {args.output, raw_path}:
        if path.exists():
            print("Refusing to overwrite an existing file: {}".format(path), file=sys.stderr)
            return 1
    try:
        from picamera import PiCamera
    except (ImportError, OSError) as exc:
        print(
            "PiCamera is unavailable: {}\nRun this on the robot Pi using the same Python "
            "environment as task1.py/task2.py. This script uses the project's legacy picamera driver."
            .format(exc), file=sys.stderr,
        )
        return 1

    stop = threading.Event()
    previous_handlers = {}

    def request_stop(signum, frame):
        stop.set()

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        signum = getattr(signal, name, None)
        if signum is not None:
            previous_handlers[signum] = signal.signal(signum, request_stop)

    raw_created = False
    error = None
    elapsed = 0.0
    print("Pi camera recorder / {}×{} @ {} fps / rotation {}°".format(
        args.width, args.height, args.fps, args.rotation,
    ), flush=True)
    print("Recording file: {}".format(raw_path), flush=True)
    try:
        try:
            raw_path.parent.mkdir(parents=True, exist_ok=True)

            with raw_path.open("xb") as stream:
                raw_created = True
                elapsed = record(PiCamera, stream, args, stop)
        except Exception as exc:
            error = exc
            print("Recording error: {}".format(exc), file=sys.stderr)
            print("Check camera access, available storage, and the requested video mode.", file=sys.stderr)

        if raw_created:
            if raw_path.stat().st_size:
                if error:
                    print("\nRecording ended after an error; preserving captured video.", flush=True)
                else:
                    print("\nRecording stopped ({:.1f} s).".format(elapsed), flush=True)
                print("H.264 saved: {}".format(raw_path), flush=True)
                if args.output.suffix.lower() == ".mp4":
                    make_mp4(raw_path, args.output, args.fps)
            else:
                raw_path.unlink()
                print("Stopped before any video was recorded.", flush=True)
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
