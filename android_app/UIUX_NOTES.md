# The Job — Android UI/UX

This branch starts from `origin/main` at `63cb647` and changes the Android presentation layer only. Task 2 was absent from that main revision; its existing launch page from `origin/task1-2button-version` is now restored with the shared theme and a separate home entry. The other branch's backend and Task 1 changes are not merged.

## Design

The supplied shooting board and script define the heist identity: vault black, paper text, stamp red, blueprint surfaces, and blue telemetry. Condensed display type, restrained racing stripes, and a compact control panel add the requested F1 influence. System fonts keep the interface available offline without new dependencies.

- Home: “The Job” briefing, immediate Bluetooth navigation, and separate Task 1 / The Job and Task 2 / The Getaway entries in portrait and landscape.
- Task 1: blueprint arena grid, readable obstacle/target IDs, compass heading, current local pose and obstacle counts, pinned Stop, manual drive, setup help, and protected reset/leave actions.
- Task 2: dedicated getaway page, static F1 route illustration, launch control, pinned Stop, Bluetooth navigation, and a local command log. Launch uses the reference page's existing `Protocol.start()` (`BEGIN`) command.
- Bluetooth: consistent connection colors and text, scan progress, paired/nearby counts and empty states, accessible device rows, and a separate received-message terminal.
- Controls use at least 48dp touch targets. Long content scrolls; landscape Bluetooth places connection controls beside device lists. The theme stays consistent in system light and dark modes.

## Scope boundaries

No changes to `BluetoothService.kt`, `Protocol.kt`, robot/obstacle models, movement commands, obstacle touch/coordinate logic, algorithms, Raspberry Pi, STM, simulator, manifest permissions, or dependency/build versions. Activity code changes only connect the new presentation, navigation, and confirmations to existing behavior. The manifest registers the restored Task 2 activity.

Pose and identification counts reflect the existing local arena model, which updates on received robot/target messages. No timer, speed, connection, or task-completion values are fabricated. The Stop button sends the existing stop command; it is not a new hardware emergency-stop mechanism.

Connect before editing. The existing app does not replay offline obstacle edits when connecting, and the existing initial-placement send is commented out. This branch preserves those behaviors; target-face/drop interactions still use the existing commands. Full functional assessment compliance therefore still needs a robot check independent of the visual redesign.

Task 2 launch and Stop report whether the existing Bluetooth service accepted the command; they do not claim the robot has started or finished. The route illustration is not live telemetry. The reference page's fixed distance, heading, and ready/live indicators are not presented as real measurements.

As in the reference implementation, both task pages use the existing `BEGIN` command. The team's corresponding Task 1 or Task 2 runner must be active on the Raspberry Pi; choosing a page does not change that runner.

## Rubric mapping

The supplied assessment PDF (pages 3–4) allocates 20% of the video grade to Android UI usability, aesthetics, ease of use, uniqueness, and creativity, and calls out small-touchscreen navigation.

| Checklist | UI treatment |
| --- | --- |
| C.1–C.2 | Bluetooth scanning/selection and send/receive terminal remain accessible. |
| C.3 | Labeled directional controls; Stop remains visible while the side panel scrolls. |
| C.4 | Selected mission updates are separate from raw received messages. |
| C.5 | Bounded arena, white obstacle numbers, visible robot pose and N/E/S/W heading. |
| C.6–C.7 | Existing touch placement, dragging/removal, and face picker retained; help explains gestures. |
| C.8 | Existing reconnection behavior retained; connection states use text as well as color. |
| C.9–C.10 | Larger white recognized target IDs, retained face stripe, and live pose rendering. |

## Device walkthrough

1. Launch in light and dark system modes; verify separate Task 1 and Task 2 buttons and access to Bluetooth.
2. Scan, select a device, connect/disconnect, and observe reconnection; check empty and populated lists.
3. Send/receive text; scroll earlier messages without being pulled to the bottom.
4. Open the arena, place and drag obstacles, hold to choose each face, and remove an obstacle by dragging outside.
5. Verify Stop stays visible while scrolling; cancel reset/leave to retain the layout.
6. Visit Bluetooth from the arena and return; landscape is retained for this route.
7. Receive target and robot messages from the real system; inspect IDs, face stripes, heading, counts, and selected event log.
8. Check the actual assessment tablet, a compact landscape display, and enlarged system font settings.
9. Open Task 2, launch and Stop with/without Bluetooth, and verify truthful send feedback. Visit Bluetooth and return; check the local command log remains. Return home and reopen Task 1 to verify the two destinations are distinct.

## Verification

- `:app:assembleDebug :app:lintDebug` passed after Task 2 restoration: 0 lint errors, 56 nonfatal warnings. Existing warnings and optional newer accessibility attributes are not suppressed.
- The verification copy used the repository's exact source, Gradle 9.5, AGP 9.3.1, dependencies, and SDK 37. Its daemon used Android Studio's JBR 25 with a 1 GB heap and two workers because the host was under memory pressure. The repository's Java 21 daemon criterion and other build settings were not changed.
- All source/resource files were compared byte for byte with the verification copy. Android resource compilation, XML references, landscape/portrait control IDs, and `git diff --check` passed.
- Contrast checks: primary white-on-red button 4.71:1; secondary text on panels 9.28:1; white obstacle IDs 6.32:1; white recognized IDs 7.32:1.
- The original UI release was based on GitHub main at `63cb647`. During Task 2 restoration, main had advanced to `2efd382` with firmware/controller changes; those unrelated changes were not merged into this UI branch.
- Android 15 emulator checks passed at 1280×800: home/arena/Bluetooth navigation, obstacle placement and dragging, face selection, real scan-progress UI, disconnected Stop feedback, and cancelling reset without losing obstacles. The arena retained both edited obstacles after a Bluetooth round trip while Android user rotation was set to portrait.
- A compact 800×480 / 1.3× font inspection confirmed the pinned Stop and scrollable controls; the larger canvas labels were rebuilt and visually rechecked. The 480×800 portrait home layout was also inspected with larger text. Screenshots are saved under `build/uiux-screenshots/`.
- Task 2 follow-up: all 68 source/resource files match the successful verification build. Both home variants have separate task buttons, and the manifest/activity/resource links pass review. The updated home rendered both entries in the Android 15 emulator, but repeated Android System UI stalls and accessibility timeouts prevented a reliable full walkthrough. Task 2 navigation, command feedback, and compact layout still need the team's physical-tablet check; the earlier emulator results above apply to the original UI release.
- Arena state remains activity-local, as in the original application; device configuration changes or process recreation are not a persistence guarantee.

Local generated artifacts (ignored by Git): `app/build/outputs/apk/debug/app-debug.apk`, `build/reports/lint-results-debug.html`, and `build/reports/uiux-build.log`.

Real robot communication, automatic reconnection, and hardware movement require the team's device walkthrough; emulator testing cannot establish those outcomes.
