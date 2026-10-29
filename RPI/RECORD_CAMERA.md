# Record the robot camera

Copy `record_camera.py` to the robot Pi. It is independent of the task, Bluetooth, network, and movement code; no `.env` file is needed. It uses the same legacy `picamera` driver as the existing camera streamer.

Start it on the Pi:

```sh
python3 record_camera.py
```

It warms up for two seconds, then records continuously until **Ctrl+C**. Wait for the saved message before closing the terminal. SIGTERM and an SSH hang-up also request a clean stop. A forced kill or loss of power cannot run cleanup.

The defaults are **1920×1080, 30 fps, 180° rotation**, with hardware H.264 encoding. The rotation matches the camera mounted on this robot. Recording is silent, without audio.

Files go into `recordings/` next to the script with unique date/time names. Each recording saves an `.h264` file. If FFmpeg is installed, it also makes an `.mp4` for editing, without re-encoding. The H.264 original is kept as a backup. If FFmpeg is missing, the script prints the command for converting it later on the Pi or PC. No existing file is overwritten.

## Options

Choose a name; the parent directory is created if needed:

```sh
python3 record_camera.py --output recordings/getaway_take1.mp4
```

Use the project's normal stream resolution and frame rate:

```sh
python3 record_camera.py --width 640 --height 480 --fps 20
```

Record at 720p / 60 fps if the Pi camera supports that mode:

```sh
python3 record_camera.py --width 1280 --height 720 --fps 60
```

Save raw H.264 only, or change orientation:

```sh
python3 record_camera.py --output recordings/take1.h264
python3 record_camera.py --rotation 0
```

Add `--preview` only when you want the camera preview on a display attached to the Pi. It is off by default so recording works over SSH without a display.

## Separate movement control

Start this recorder, wait for **RECORDING**, then operate the robot using your separate movement script or controller. This script imports no motor, STM32, Android, or task modules and sends no movement commands.

The camera must be free: stop any other process using it, including an active Task 1/Task 2 camera stream. A movement-only script can run alongside the recorder. Two separate programs cannot own the same camera at once.

## References

- [PiCamera recording API](https://picamera.readthedocs.io/en/release-1.13/recipes1.html#recording-video-to-a-file)
- [FFmpeg input frame rate and stream copying](https://www.ffmpeg.org/ffmpeg.html)
