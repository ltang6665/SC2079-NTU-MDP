# Submission integration

This document describes `submission_runtime/`, the runnable companion to the source-only `submission/` folder. The original PC/RPi folders in the repository are development copies.

| Component | Reference used |
| --- | --- |
| Task 1 PC/controller/planner | Project source in the newly supplied `PC.zip` |
| Task 1 RPi/controller/support | Project source in the newly supplied `RPI.zip` |
| Image preparation and training | `IMAGE.zip` |
| Android | `android_app.zip`; all files preserved byte-for-byte |
| Task 2 | Previously aligned SmoothFlow version-50 controller and frame-stage handling |
| STM | `stm_submission/Grp28_Task1` and `stm_submission/Grp28_Task2`; firmware C/header source preserved |

## Task 1 reference

Task 1 uses the exact directed grid planner, `MazeSolver.getOptimalOrderExact()`, and the camera/edge/movement settings in the supplied ZIP. The PC generates movement commands and divides them at image checkpoints. The Pi sends one segment, waits for `OK`, requests `DETECT` for the corresponding obstacle, then sends the next segment. The final segment triggers stitching. This preserves the competition-reference segment strategy.

Internal supporting module names follow the existing renamed layout, with compatibility imports retained. Python comments/docstrings were removed and selected local names clarified; inverse syntax-tree checks verified that these transformations preserve the ZIP code. The separate interface adaptations are listed below.

## Android integration

The Android code is unchanged. The Task 1 RPi controller accepts its `CALCULATE` message as well as legacy `PATH`/`ALG|` requests. Position-only obstacle messages retain an existing facing direction or use `SKIP` until the user selects one. The Pi sends `PATHREADY` when the route arrives, matching the app's inbound parser. Malformed obstacle messages do not terminate the receiver.

These are small RPi interface additions to the Task 1 reference; the planner and segment execution strategy remain the supplied versions. Manual `ROBOT_MOVE` handling is unchanged from the supplied RPi reference.

## Detector checkpoint

Only the newly trained `bestv8n.pt` is included. `PC.zip/PC/bestv8n.pt` and `IMAGE.zip/IMAGE/bestv8n.pt` contain identical bytes. The renamed older checkpoint is excluded from both final packages. Both PC task controllers select the packaged `PC/bestv8n.pt` using an absolute path derived from their own file location.

New checkpoint SHA-256: `61f31a2b1aadc152e0f5b67e6c5738c64df43e546c5ac0ae1dc4c333ee163ba7`.

The runtime also contains the identical new checkpoint in `IMAGE/bestv8n.pt` alongside the training scripts. Model weights are resources and are not included in the source-only upload.

## Task 2 and STM

Task 2 retains the SmoothFlow `SMOOTH_REL_READY,50` handshake, fresh-arrow reverse confirmation, alignment-gated second-arrow confirmation, and frame-stage filtering. Its serial bridge retains partial UART messages and returns one complete line at a time; the streaming listener supports both Task 1's callback and Task 2's frame-context callback.

Both submitted STM projects use USART3 at 115200 baud. Flash the matching task project. The packaged CubeIDE build/debugger paths point to the actual project names, `task1_copy` and `task2_smoothflow`. Firmware application source bytes remain unchanged.

## Running and checks

From the runtime folder, start the Pi controller with `bash RPI/start_task1_bt_safe.sh` or `bash RPI/2start_task2_bt_safe.sh`, and the PC with `bash PC/run_task1.sh` or `bash PC/run_task2.sh`. Review the paired `.env` network and serial settings first. Android includes its original Gradle project and resources in the runtime companion.

Run `python3 -m unittest discover -s RPI/tests -v` for camera shutdown, actual-reference Task 1 segment progression, Android message compatibility, model path selection, and Task 2 protocol checks. Run `python3 -m pathfinding.regression_test` from the PC folder for the supplied planner regression cases. The original and transformed planners produced identical results on both regression layouts.

Physical driving, live detector inference, Bluetooth/serial operation, Android building, and camera operation were not tested on hardware. Dataset images are not part of the source upload. Experimental `pathfinding-vibe`, `Task1-old.py`, the previous hybrid planner, virtual environments, and bundled third-party library source are not included in the final upload.
