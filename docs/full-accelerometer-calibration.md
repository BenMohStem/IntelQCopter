# Full three-axis accelerometer calibration

The user requested the complete coupled correction now. The active `firmware/main.cpp` applies all three biases and the full 3×3 matrix to every coherent XYZ reading. The previous one-axis lesson was a register-verification exercise, not the complete calibration model. It remains in Git history; there is no requirement to finish that isolated test before using this prepared full implementation.

## The actual correction

Let r be the sensor vector in the same post-reset baseline convention as the raw recordings. The fitted correction is:

$$
\mathbf a_{drone}=C(\mathbf r-\mathbf b),
\qquad
\mathbf b=\begin{bmatrix}1.308683406\\-0.067669726\\-0.657188793\end{bmatrix}\ \mathrm{m/s^2}.
$$

$$
C=\begin{bmatrix}
0.953457734&-0.085144142&-0.287271311\\
0.088080052&0.990761548&-0.002594316\\
0.287429504&-0.030297733&0.947299771
\end{bmatrix}.
$$

These are the same provisional coefficients from training repetitions 1-2. Repetition 3 and the independent tilted records did not refit them. The matrix combines accelerometer scale/cross-axis and sensor-to-drone mounting effects. It is applied to the entire vector, not selected according to which axis points upward.

For instance, the first output is:

$$
a_x=C_{11}(r_x-b_x)+C_{12}(r_y-b_y)+C_{13}(r_z-b_z).
$$

The other two rows are evaluated from the same XYZ sample. Nothing is zeroed at boot according to the current pose. The output is not normalized to g. A moving drone can have real acceleration in addition to the stationary gravity/specific-force signal; this static linear sensor calibration does not remove that physical acceleration.

## Bias in MPU registers, matrix in software

Bias constants and C are retained in ESP32 program flash. At boot, Adafruit initializes/resets the MPU, the code checks each boot word against its recorded baseline or its own known retained target, writes absolute rounded XYZ bias trims and verifies readback. It validates all three boot states before writing any axis. A retained target is rewritten as the same absolute value, so it is not corrected twice; unknown third values are rejected. BOOT_OFFSET lines preserve the observed startup values for review. Unexpected baselines, configuration, I2C failures or readbacks stop normal data output. The exact recorded baselines are specific to this calibrated module.

| Sensor axis | Recorded baseline | Signed decrement | Target word | Nominal remaining bias (m/s²) |
|---|---:|---:|---:|---:|
| X | -2074 | 274 | -2348 | -0.003339103 |
| Y | 2979 | -14 | 2993 | -0.000632079 |
| Z | 1074 | -138 | 1212 | +0.003610865 |

All word changes are even, preserving compensation bit 0. The actual float32 remainders printed by the firmware can differ in the final decimal places from this double-precision table.

The small software remainder is necessary because hardware trim values are quantized. Define h as the nominal bias already removed by the hardware, and delta as what remains:

$$
h_i=d_i\frac{g}{2048},\qquad \delta_i=b_i-h_i.
$$

The register-corrected measurement is modeled as r_MPU = r - h. Therefore the implemented calculation is:

$$
C(\mathbf r_{MPU}-\boldsymbol\delta)
=C[(\mathbf r-\mathbf h)-(\mathbf b-\mathbf h)]
=\boxed{C(\mathbf r-\mathbf b)}.
$$

Only delta is subtracted in software. Subtracting the full b again would double-correct the register-applied bias. The established nominal MPU6050 trim convention is retained; its actual physical response on this module and reset behavior still need testing. Readback verifies register bits, not absolute analog accuracy. The full matrix cannot be represented by bias registers alone.

## Following main.cpp

1. `accel_bias` and `accel_matrix`: fixed fitted parameters, in sensor/body coordinates as described above.
2. `restore_accel_bias`: validate known startup states, restore all three absolute register targets, calculate rounding remainders.
3. `correct_acceleration`: subtract the three remainders and multiply by C. All nine matrix terms are used.
4. `loop`: verify offsets, read XYZ in one six-byte burst and correct that vector. Each packet contains three hardware-trimmed ADC counts and all three drone-frame outputs.

The gyro remains configured at ±500 degrees/s, but gyro bias/scale/alignment calibration is not added here. Do not apply this accelerometer matrix to gyro rates; it contains accelerometer scale/shear corrections, not just a pure mounting rotation. Motor outputs remain disabled.

## Upload and see arbitrary orientations live

Build/upload the `bench` environment yourself. Close Serial Monitor, then run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-accel --port COM3 --seconds 30
```

The PC displays corrected X/Y/Z and norm about once per second while saving every emitted sample. You can keep any stationary pose or rotate slowly through several orientations. No special pose or flipping sequence is required by the program. At rest, magnitude should be near g, subject to calibration/reference errors; during movement it need not be g. There is no position hold, gravity removal or attitude/PID loop in this bench code.

Logs save automatically under `artifacts/accel-recordings/SESSION/`: original serial.log, samples.json and summary.json. They remain private by default. The PC verifies the source fingerprint, all coefficients and trim remainders, then recomputes C for each reported ADC triplet to catch software inconsistency. That arithmetic check is not independent proof of sensor accuracy; the true physical reference is still external.

The MPU output remains nominally 1000 Hz (mode-0 filter, divider 7), but bench telemetry waits 10 ms plus processing/I2C time, so its measured report rate is lower than or near 100 Hz. It is not the original 1000 Hz FIFO recorder and is not a 500 Hz flight loop. During motion, reported sample spread includes real motion and cannot be treated automatically as sensor noise.

If a baseline mismatch occurs, stop and review logs. Power-cycle both ESP32 and MPU before retrying; do not edit baselines to hide an error. Repeat the complete stream after reset and after a full power cycle to check that offsets do not accumulate. A failed later-axis write may leave partial trims before the program stops, so start from a fresh reset/power cycle for retries.

The earlier `record-bias-x` command belongs to the previous source/protocol and is not used now. The exact original raw FIFO source remains preserved in `archive/bench-checkpoints/raw-pose1000/main.cpp.txt` for deliberately returning to raw measurements. Existing original datasets remain unchanged.

## Verification and remaining uncertainty

Source-coefficient/float32 algebra tests cover all three targets and compensation bits, 1000 arbitrary vectors, known positive/negative and tilted orientations, missing/double compensation guards and inconsistent output rejection. These are a Python mathematical model of the actual source coefficients, not execution of the compiled MCU firmware. The PlatformIO build checks compilation; physical sensor response, independent accuracy and reset/power-cycle behavior remain unvalidated until measured.

The candidate's existing independent reference was hand-held with approximately ±1 degree operator-estimated placement uncertainty. Applying the full matrix is the proper general geometry, but does not establish higher physical accuracy than the measurements support. No new Gaussian/independence assumption or Kalman/PID gain is introduced.

Sources: [the measured-data fit](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json), [tilt review](../measurements/mpu6050/2026-10-25-tilt-validation/report.md), [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp), [MPU6050 trim implementation](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp).

Final preparation: the ESP32-S3 PlatformIO build passed (19,372 bytes RAM; 307,597 bytes flash), and six algebra/protocol checks passed. [Verification record](../measurements/mpu6050/2026-10-25-full-xyz-preparation/verification.json). Original compiler output and earlier build remain local. No upload or serial-port connection was performed.
