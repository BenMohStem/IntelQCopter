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

## Bias in MPU6050 registers - current implementation

The user requested I2Cdevlib-style setXAccelOffset/setYAccelOffset/setZAccelOffset calls. PlatformIO now uses electroniccats/MPU6050 1.4.5 instead of Adafruit MPU6050. All runtime code remains in main.cpp with Arduino Wire and Serial.printf. PlatformIO resolves the dependency when the user builds/uploads; the assistant has not built this revision.

The three measured biases remain constants in ESP32 program flash. The MPU offset registers accept signed integer trims, not floats in m/s². At the configured ±2g range, acceleration output uses 16384 counts/g; the offset register uses 2048 counts/g. Its bit 0 must retain its original temperature-compensation setting. Therefore each change is rounded to an EVEN number:

    change = 2 * round(bias * 1024 / 9.80665)
    new_offset = baseline_offset - change

| Axis | Measured bias (m/s²) | Added to baseline register | Predicted quantization residual (m/s²) |
|---|---:|---:|---:|
| X | 1.293084598 | -270 | +0.000216 |
| Y | -0.079166657 | +16 | -0.002552 |
| Z | -0.639507624 | +134 | +0.002138 |

These are changes RELATIVE to the original register values, not absolute values to paste into the setters. The original register values were not recorded with the six poses. The implementation assumes those measurements used the same sensor's default/factory trims; independent physical verification remains essential after this library change. The predicted residuals describe register rounding only, not sensor accuracy or calibration uncertainty.

## Read setup and the helper in order

1. Start Wire on GPIO9/GPIO8 at 100 kHz and check the device responds.
2. Request a full MPU reset, wait, then initialize using the new library. This establishes a baseline before applying one correction per boot rather than intentionally adding corrections repeatedly. Confirm the printed baseline repeats on resets and power cycles; that behavior has not been measured on this module.
3. initialize selects ±2g; the code verifies that range before using its conversion factor. Its gyro default is ±250 degrees/s; gyro calibration is not added.
4. Read the three baseline accelerometer offset registers.
5. corrected_offset converts each bias to an even integer register change. Subtracting an even value preserves bit 0 for positive and negative registers alike.
6. Check all three results fit a signed 16-bit register before writing any of them.
7. Call the requested three set...AccelOffset functions, then read all three registers back and compare. Print baseline and applied values. Failed initialization/range/readback suppresses measurement output. This is a startup check, not continuous sensor-health monitoring.

## Reading acceleration afterwards

getAcceleration returns three signed raw counts. Multiply each by 9.80665/16384 to obtain sensor-coordinate m/s². Do NOT subtract accel_bias_x/y/z here: the MPU has already applied the quantized hardware bias correction. The existing three scale/mounting expressions then map those sensor readings into drone coordinates. Their coefficients were not refitted and still require independent checks.

The small hardware rounding residual is left visible rather than adding another confusing bias path. Gravity remains in the reading. No gyro offset setter is called because no measured gyro bias is available. Do not copy the example gyro numbers.

## Persistence and recalibration

The MPU's offset registers are volatile. The persistent copy is the three bias constants and scale/mounting coefficients embedded in ESP32 firmware flash. setup restores the integer registers at every boot. There is no claim that they survive by themselves inside the MPU. A future on-device calibration may use Preferences to save new runtime results; this fixed candidate needs only firmware constants.

Changing sensor, mounting, acceleration range or previous trim settings requires reviewing/repeating calibration. New measurements must be collected with a documented baseline, not after accidentally applying the same correction twice.

## Your real check - not performed by the assistant

Keep propellers removed. Upload yourself and open Monitor at 115200 baud. Record these lines:

```text
MPU_BASE_OFFSETS X=... Y=... Z=...
MPU_APPLIED_OFFSETS X=... Y=... Z=...
MPU initialized and accel offsets verified : Yes.
```

Applied minus baseline should be (-270, +16, +134). Absolute register values depend on your module. Readback establishes only that the requested integer values are present, not that acceleration is accurate.

Press RESET/EN several times, then separately power-cycle the whole sensor and ESP32. Baseline and applied register values should repeat, without accumulating correction. If they change unexpectedly, preserve the logs and stop: the reset/baseline assumption needs correction for your module.

Repeat the six stationary DRONE-axis poses and several NEW tilted poses. Check aligned ±9.80665 m/s², near-zero perpendicular components, and vector magnitude near g. Compare with the earlier software-only correction, allowing for small register quantization, noise and pose alignment. The earlier maximum training residual was 0.1425 m/s², not independent accuracy. No tolerance or standard deviation has been invented.

Sources: [Electronic Cats library and its offset setters](https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp), including PID's division by 8 and preservation of BitZero; [library metadata](https://github.com/ElectronicCats/mpu6050/blob/master/library.properties). The setters write integer words; baseline handling and readback are our explicit additions.

## Limits and next step

This calibrates only acceleration provisionally. Gyro bias, gyro-to-drone mounting rotation, thermal drift, sensor disconnect detection and attitude estimation are not implemented. Do not apply C directly to gyro readings: its accelerometer scale/shear terms are not a pure rigid rotation. The one-second report rate is for bench inspection only. Motor outputs remain disabled. Mass/inertia and gains remain deferred to standalone-FC Gazebo work.

No build, upload, power-cycle or sensor acceptance was performed by the assistant. The numerical fit and training residuals were calculated from the user-supplied means; these are calculations, not new measurements. Calibration remains the current phase until the user verifies restoration and independent pose checks.
