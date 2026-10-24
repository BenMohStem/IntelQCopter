# Six-pose accelerometer calibration and flash storage

## What changed

The user confirmed that pose labels refer to DRONE axes, while the readings are in MPU sensor axes. The +X pose predominantly excites sensor +Y; the +Y pose predominantly excites sensor -X. Therefore three separate per-sensor-axis scale factors cannot, by themselves, produce drone-frame acceleration. This implementation applies a provisional affine correction containing bias, scale/cross-axis and mounting effects together. It is not a multivariable flight controller.

The original collection programme, all six averaged measurements, fit coefficients and residuals are preserved in [the measurement snapshot](../measurements/mpu6050/2026-10-24-six-pose/calculation.json). No source data were discarded. The old single-pose averaging at every boot was removed; it was not a six-pose calibration. Raw and corrected output are both retained.

## Derivation

Let r be the raw sensor vector in m/s², a the desired drone-frame vector, b the sensor-coordinate offset and A the combined response. At rest in a correctly aligned pose:

    r = b + A a

For each drone axis j, the two targets are +g e_j and -g e_j, using g = 9.80665 m/s². Add each pair to cancel gravity:

    c_j = (r_(+j) + r_(-j)) / 2
    b = (c_X + c_Y + c_Z) / 3

Subtract each pair to cancel the offset:

    d_j = (r_(+j) - r_(-j)) / 2 = g A e_j
    D = [d_X d_Y d_Z] = g A
    C = g inverse(D)
    a_corrected = C (r - b)

This is the unconstrained least-squares affine fit for these six symmetric target vectors when expressed as raw response versus target, then inverted. It assumes accurate stationary pose orientations. Every output row is only three products added together, as shown in correct_acceleration. It is not a Kalman filter, and it neither estimates nor subtracts gravity from the corrected acceleration.

Calculated offset b in sensor coordinates (m/s²):

```text
1.293084598, -0.079166657, -0.639507624
```

Calculated C, mapping centred sensor readings to drone axes:

```text
 0.046659658  0.992539802 -0.045743511
-0.955388252  0.053316466  0.289431328
 0.272504170  0.023832247  0.951963666
```

The three opposite-pose midpoints differ: see calculation.json. Their disagreement and the residuals can reflect alignment error, noise, temperature change or model limitations. They do not establish standard deviation, and the three +/− pairs are not independent validation data.

## Simpler code: constants in program flash

The calibration values are known before compilation, so they can be stored as constants inside main.cpp. Uploading the firmware stores the constants in ESP32 program flash along with the code. A power cycle does not erase them. Each boot uses those same constants; no separate Preferences save/load routine is needed for this fixed candidate.

This is persistence in the ESP32, not nonvolatile storage inside the MPU6050. Old Preferences records, if present from a previous firmware, are no longer read or written. No flash partition is erased by this change. Recalibration means updating the constants from new measurements and uploading again. If we later measure a new calibration on the board itself, Preferences will be appropriate for saving those runtime results without recompiling; that is deliberately deferred.

## Read main.cpp in order

1. The includes provide Arduino, Wire and the existing Adafruit MPU6050 library. No dependency changed.
2. accel_bias_x/y/z are the offsets calculated above. They describe MPU sensor axes, in m/s².
3. setup keeps motor signals inactive, starts I2C on SDA9/SCL8 at 100 kHz, and initializes the MPU at 0x68.
4. loop reads only after successful initialization.
5. sensor_x/y/z subtract the three biases from raw sensor readings.
6. drone_x/y/z are three explicit expressions. Each expression multiplies the centred readings by one row of the calculated correction matrix. This preserves the existing six-pose correction without arrays, loops, structs or flash-record code.
7. printf prints both raw sensor and corrected drone coordinates. delay keeps the bench output at approximately one update per second.

The old single-pose averaging code and obsolete offsets are absent from the active firmware. Their original source and data remain in the measurement snapshot and Git history.

## Why not the suggested setXAccelOffset example?

That API belongs to the I2Cdevlib-style MPU6050 library, rather than the current Adafruit API. The setter writes an integer bias register; it does not provide this combined scale/mounting correction or persistent storage in the MPU. Its integer argument is not an offset in m/s², and the example numbers were not measured for this sensor. No gyro bias measurements were supplied, so its example gyro offsets must not be copied either.

We keep the existing library and one software correction, avoiding unknown hardware-register changes or double subtraction. See the [actual offset setter implementation](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp).

## Your real test - not performed by the assistant

With propellers removed, upload yourself and open Monitor at 115200 baud. Press RESET/EN if necessary. The initialization should say Yes and the calibration message should say stored in firmware flash. The previous SAVED_AND_READ_BACK / LOADED_FROM_FLASH messages no longer apply to this simplified version.

Repeat all six DRONE-axis poses in a stable fixture. The corrected aligned axis should approach ±9.80665 m/s², with the other two near zero. Preserve the raw and corrected values and their variation. Repeat several NEW stationary tilted poses and check that the corrected magnitude stays near g.

Reset, then separately disconnect/reconnect power and repeat the same poses. The correction should remain consistent, within sensor variation. This checks the fixed firmware calibration persists, not a separate NVS record. Do not move the sensor relative to the frame: the fit includes the mounting orientation.

Training residual vector norms were:

- +Z: 0.107519 m/s²
- -Z: 0.107519 m/s²
- +X: 0.142461 m/s²
- -X: 0.142461 m/s²
- +Y: 0.043839 m/s²
- -Y: 0.043839 m/s²

The largest is approximately 0.1425 m/s². These are fitting residuals on the supplied means, not independent measured accuracy or standard deviations. No acceptance tolerance is inferred from them. If independent checks fail, revisit pose alignment and calibration; do not hide the error in PID gains.

## Adafruit restored: checking that correction is applied

The current dependency is again adafruit/Adafruit MPU6050 2.2.9. Hardware offset setters are absent. The library initializes/resets the MPU; no user offset register changes are requested by this firmware. Fully power-cycle the sensor before comparing with an earlier hardware-trim version, so a retained device state does not confuse the comparison.

The loop reads one event, subtracts each named bias once, and applies the original three scale/mounting expressions. RAW_SENSOR prints the library readings BEFORE our correction; DRONE prints the corrected coordinates and vector magnitude. The user's 100 ms reporting interval is preserved. A stationary corrected norm should be near 9.80665 m/s², not zero.

The project command `qcopter.ps1 check-accel` exercises the actual scalar expressions extracted from main.cpp using IEEE float32 rounding against all six recorded means. It checks both the corrected vectors against the saved fit and the error if bias were omitted or subtracted twice. This checks the source expressions with a host numerical harness, not execution on an ESP32 or a live I2C bus. The PlatformIO build separately checks embedded compilation. Original outputs are preserved under artifacts/calibration-check and artifacts/build, including failed attempts.

Neither test proves the board is running this binary or that the module's readings are accurate. After the user uploads, capture RAW_SENSOR and DRONE in all six poses and several new tilted stationary poses, then repeat after a power cycle. Preserve the actual serial output. No serial connection or upload is initiated by the assistant.

## Limits and next step

This calibrates only acceleration provisionally. Gyro bias, gyro-to-drone mounting rotation, thermal drift, sensor disconnect detection and attitude estimation are not implemented. Do not apply C directly to gyro readings: its accelerometer scale/shear terms are not a pure rigid rotation. The 100 ms report interval is for bench inspection only. Motor outputs remain disabled. Mass/inertia and gains remain deferred to standalone-FC Gazebo work.

See the numerical/build result report for offline checks actually performed. No upload, power-cycle or sensor acceptance was performed by the assistant. The numerical fit and training residuals were calculated from the user-supplied means; these are calculations, not new measurements. Calibration remains the current phase until the user verifies restoration and independent pose checks.

[Recorded results of the requested numerical check and embedded build](calibration-test-results.md). Both passed; no hardware acceptance is implied.
