# Six-pose accelerometer calibration and flash storage

**Historical implementation and coefficients below.** The current secured-mount dataset has a [new provisional analysis](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md). Current firmware is the raw recorder; no candidate is accepted yet. Earlier mounting coefficients must not be reused without validation.

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

## Current implementation: Adafruit readings, MPU bias registers

The user requires bias inside the MPU while retaining Adafruit. main.cpp now uses Adafruit_MPU6050 for data/configuration and Adafruit BusIO (already a dependency) for the three accelerometer offset words. No Electronic Cats library is needed. The preceding mathematical fit remains unchanged.

write_accel_bias reads the reset-baseline 16-bit word, computes an even trim change, checks signed range, writes a big-endian word and verifies readback. Calls use X=0x06, Y=0x08, Z=0x0A. These offset words are supported by established MPU libraries but are not fully specified in the public revision-4.2 register map; do not treat their semantics as a documented promise for clones or other MPU families. Source: [I2Cdevlib offset/calibration implementation](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp), particularly its /8 conversion and preservation of BitZero.

For this MPU6050 trim convention:

$$
\Delta o_j=-2\operatorname{round}\left(\frac{1024 b_j}{g}\right),\qquad o_{new,j}=o_{baseline,j}+\Delta o_j.
$$

The changes are X=-270, Y=+16, Z=+134 counts, relative to each baseline. Even changes preserve the temperature compensation low bit. They leave predicted sensor-coordinate rounding residuals about (+0.000216, -0.002552, +0.002138) m/s². These are numerical quantization estimates, not measured accuracy.

The baseline assumption is that the original six-pose recordings used the same sensor's default trims. Adafruit begin resets the device before offsets are applied. Verify baseline values repeat across restarts; do not assume an untested module resets trims exactly as expected. If any write/configuration check fails, corrected measurement output is suppressed until restart. Earlier axes might already have been written before a later failure, so restart/power-cycle before retrying rather than applying offsets again manually.

## What is stored where?

| Location | Contents | Power-cycle behavior |
|---|---|---|
| ESP32 firmware flash | Your measured biases and correction coefficients | Retained; source of the boot-time restoration |
| MPU6050 offset registers | Integer acceleration bias trims | Volatile; reapplied by setup |
| Software expressions | Scale and sensor-to-drone frame mapping | Executed on each sample |

The complete 3-by-3 scale/mounting matrix cannot be stored as an equivalent transform in those offset words. Gyro offsets remain untouched because none were measured. No full software bias subtraction remains: applying both would double-correct the bias. Old Preferences records are not read or written.

## Your physical proof

1. Upload yourself with propellers removed. Open Monitor at 115200 baud and reset to capture startup.
2. Preserve the three MPU_OFFSET lines. Check readback=requested and requested-baseline is -270, +16 and +134 for X/Y/Z respectively.
3. Check MPU_CONFIG ends in verified=Yes. Enum codes are configuration bits, not physical range/frequency numbers; see the configuration guide.
4. Record MPU_BIAS_CORRECTED (sensor coordinates after hardware trim) and DRONE (after scale/mounting) in all six poses. Also record new tilted static poses; corrected magnitude should stay near g.
5. Reset several times, then power-cycle sensor and ESP32. Baseline and applied values should repeat, without accumulating trim. Stop and share logs if they do not.

An I2C register readback proves the requested bits were read back, not calibration accuracy. The original fit's maximum training residual was approximately 0.1425 m/s². The recorded software-only test report belongs to revision 100b879; the hardware-trim numerical model and embedded compile now pass, while actual device verification is still required. No current-run physical success is claimed.

For editing ranges, DLPF, clock and rate, use [the datasheet/configuration guide](mpu6050-configuration.md).

## Limits and next step

This calibrates only acceleration provisionally. Gyro bias, gyro-to-drone mounting rotation, thermal drift, sensor disconnect detection and attitude estimation are not implemented. Do not apply C directly to gyro readings: its accelerometer scale/shear terms are not a pure rigid rotation. The 100 ms report interval is for bench inspection only. Motor outputs remain disabled. Mass/inertia and gains remain deferred to standalone-FC Gazebo work.

See the numerical/build result report for offline checks actually performed. No upload, power-cycle or sensor acceptance was performed by the assistant. The numerical fit and training residuals were calculated from the user-supplied means; these are calculations, not new measurements. Calibration remains the current phase until the user verifies restoration and independent pose checks.

[Test history, including the current modeled trim check and successful embedded compile](calibration-test-results.md). These offline results do not validate actual hardware offset response or calibration accuracy.

## Level-reference refinement on 25 October 2026

The current matrix is now pre-multiplied by a rigid level-reference rotation fitted to the first 60% of a real stationary recording; the original matrix and measurements above remain historical. The remaining 40% was used for offline temporal holdout checks. Gyro bias is now measured in sensor axes and written through the same readback helper to 0x13/0x15/0x17 at boot. Bias is not subtracted again from the gyro in software. See [precision report](precision-2026-10-25.md) for predicted residuals, uncertainty and physical checks still required. Earlier statements that gyro bias is unmeasured apply only to the earlier revision.
