# Group 28 final runtime companion

| Folder | Contents |
| --- | --- |
| `PC/` | ZIP-reference Task 1, exact directed pathfinding, Task 2, streaming, image stitching, and the new `bestv8n.pt` |
| `RPI/` | Task 1 segment controller, Task 2 SmoothFlow controller, communication/streaming modules, and tests |
| `IMAGE/` | Image preparation/training scripts and the identical new detector checkpoint |
| `android_app/` | Complete supplied Android project, unchanged |
| `STM_code/` | Complete Task 1 and Task 2 STM32CubeIDE projects |

## Running

1. Review `PC/.env` and `RPI/.env` for the target network and serial device. The firmware baud rate is 115200.
2. Install the PC/Pi requirements in their normal environments, including the Pi's Bluetooth/camera system support.
3. Import, rebuild, and flash the matching STM project: `task1_copy` or `task2_smoothflow`.
4. Start the Pi with `bash RPI/start_task1_bt_safe.sh` or `bash RPI/2start_task2_bt_safe.sh`.
5. Start the matching PC task with `bash PC/run_task1.sh` or `bash PC/run_task2.sh`. These scripts choose the working directory containing the weights and stitching output folder.


## Model

Both PC tasks use the newly trained `PC/bestv8n.pt`. No older checkpoint or automatic fallback is included. The `IMAGE/` copy has identical bytes. Its SHA-256 is `61f31a2b1aadc152e0f5b67e6c5738c64df43e546c5ac0ae1dc4c333ee163ba7`.

## Verification

Run `python3 -m unittest discover -s RPI/tests -v` from here. Run `python3 -m pathfinding.regression_test` from `PC/`. File hashes are recorded in `MANIFEST.sha256` and can be checked with `shasum -a 256 -c MANIFEST.sha256`.

See [submission integration](SUBMISSION_INTEGRATION.md) for the source references, limited RPi-side Android adapters, and tests still requiring hardware. Upload the `.txt` files in `../submission/`; this runtime folder contains configuration, weights, resources, and vendor libraries that the source-code upload does not require.
