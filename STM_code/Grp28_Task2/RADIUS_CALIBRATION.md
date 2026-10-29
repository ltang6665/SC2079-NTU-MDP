# Fixed steering radius calibration

The `CAL` commands in `Core/Src/main.c` run a finite, fixed-steering test using
the existing encoder and straight telemetry. They do not change the normal
`F`, `R`, `FL`, `FR`, `RL`, or `RR` command behavior.

## Commands

Send one command at a time through the usual STM command UART:

| Command | Motion |
| --- | --- |
| `CALFL30` | Forward, left steering, 30 cm encoder target |
| `CALFR30` | Forward, right steering, 30 cm encoder target |
| `CALRL30` | Reverse, left steering, 30 cm encoder target |
| `CALRR30` | Reverse, right steering, 30 cm encoder target |

The distance must be 10–200 cm. Use a clear area, start with 50 cm, and issue
only one test at a time. The servo is fixed at that direction's existing
steering endpoint, the heading PID cannot move it during the test, and the
drive ramps up over 350 ms, then holds the selected PWM until the encoder target
and brakes. At the end, the servo returns to centre.

The test speed is `RADIUS_CAL_PWM_RUN` in `Core/Src/main.c` (currently 4750;
on this board a lower compare value means more motor effort). Change only that
macro to compare another speed, rebuild, and flash. Start with the current
conservative setting; do not begin with a lower compare value in a confined
space.

## Data and calculation

Keep the existing telemetry enabled and capture the same UART log as usual.
Each test is identified by `CMD,<id>,...,CAL_FL:<cm>` (or the corresponding
direction). `ENC` records contain timestamp (field 2), command ID (field 3),
left/right encoder increments (fields 4–5), and sample interval (field 6).
`STR` records contain timestamp (field 2), command ID (field 3), yaw in
millidegrees (field 4), and actual servo CCR (field 12). Match records by
command ID.

Use `WPI` records to find the steady-speed window (`drive_percent`, field 12,
equals 100). Over that same window, sum the signed encoder increments from
`ENC` and take the change in yaw from `STR`. Convert yaw degrees to radians,
then divide the mean wheel distance by that angle:

```text
radius_cm = encoder_distance_cm / abs(yaw_change_deg * pi / 180)
```

The distance is the mean of the left/right wheel travel, using the existing
encoder scale in `main.c` (`COUNTS_PER_CM`). If there is no steady-speed window
or too little yaw change during it, increase the test distance. Average at
least three runs per direction. Reject runs where the robot slips, collides,
or is manually pushed.
This measures the radius at the current steering endpoint and speed; it does
not fit radii for intermediate steering commands.
