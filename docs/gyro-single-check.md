# Next small step: one positive X rotation

No firmware upload, motor output or calibration change. This uses the installed ACQ_MENU_V1 bench firmware, raw FIFO samples, ±500 deg/s and nominal 1 kHz. It does not apply the production calibration or gyro-to-drone matrix on the MCU.

Why: a stationary gyro can reveal bias/noise but cannot prove axis sign or scale. For a near-pure rotation, integrating corrected angular rate should approximately match the known physical angle. Here we first record a clean labelled motion; analysis follows separately. Hand positioning supports a rough sign/axis check, not precision scale acceptance.

## One recording now

Propellers removed; motors disabled. Keep sensor mounting fixed. Close Serial Monitor/other recorders.

```powershell
.\qcopter.ps1 record-xbox-menu --port COM3 --single f --note "operator_rotation; drone +X; right-hand +90 degrees; hand reference approximately 90 degrees"
```

Use the SAME drone X direction used when labelling the six-pose calibration. Do not rename axes from the sensor silkscreen. If its physical direction is unclear, stop and identify/mark it before recording.

1. Support the assembly so you can rotate about that axis without moving the sensor mount. Begin stationary.
2. Point your right thumb along drone +X. Curling fingers define positive rotation. This is an angular rotation ABOUT X, not translating along X or simply raising the +X end.
3. Press Enter when ready. Wait for `Recording test: f` (START).
4. Start from rest; gently rotate approximately +90 degrees in about 1–2 seconds, finish before the four-second capture ends, then hold still. Avoid a reverse movement during capture. Do not press Xbox A or change tests during capture.
5. Wait for `Saved test: f OK` and automatic exit. Return the assembly afterward. Share the printed folder path and describe which side physically moved upward. Record an actual reference angle/uncertainty if you have a fixed jig; do not claim 90° precision from hand motion.

This is only the first direction. Negative X and Y/Z directions follow after reviewing this one; do not collect six confusing runs at once.

## What analysis will establish

Raw sensor rates: omega_i = raw_i / 65.5 deg/s. Correct sensor bias once, then transform through an explicitly identified candidate rotation. Approximate single-axis angle is the time integral of rate. A FIFO record's read-completion time is not its measurement time: at nominal 1 kHz the sample interval is approximately 0.001 s, subject to clock uncertainty. This is not an exact timestamped motion reference. Three-axis body-rate integrals are not general Euler angles for arbitrary coupled rotations.

Direction, dominant axis, stationary portions, clipping, sequence/checksum and FIFO backlog are reviewed. No automatic calibration refit or pass based just on approximate 90°. The existing production main.cpp has a user edit (0.288364175 to 0.2808364175 in the candidate rotation); it is preserved and must be reviewed before using that matrix for flight. Current capture does not depend on it.
