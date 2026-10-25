# Next small lesson: observe gyro XYZ

The accelerometer calibration stays unchanged. We now observe angular velocity and temperature. We do not apply a gyro bias yet. The Arduino-style [main.cpp](../firmware/main.cpp) remains the only firmware file; motor pins remain inputs with pulldowns.

## What changed in main.cpp

1. Read 14 consecutive bytes beginning at register 0x3B instead of six bytes. Their seven signed 16-bit words are acceleration X/Y/Z, temperature, then gyro X/Y/Z. A single burst gives a coherent sensor snapshot.
2. Keep `correct_acceleration(counts, drone)` unchanged; it uses the first three words.
3. Convert the last three words to angular velocity:

\[
\omega_i\ [\mathrm{rad/s}]
=\frac{n_i}{65.5}\ [\mathrm{deg/s}]\;\frac{\pi}{180}.
\]

At the selected ±500 degrees/s range, 65.5 counts represent one degree/s. For example, 131 counts represent 2 degrees/s, or about 0.034907 rad/s. These rates are in **sensor axes**, not yet drone axes. The fitted accelerometer matrix contains accelerometer-specific scale errors: do not blindly reuse it for the gyro.

4. Convert temperature with `counts[3] / 340.0f + 36.53f` (degrees Celsius). This is the chip temperature indication, not a calibrated ambient thermometer.
5. Print one `IMU` line with all values. The `?` query reports the firmware fingerprint, accelerometer calibration and current gyro offset words. Reading gyro offsets does not change them.

The packet layout/conversions follow the [official MPU-6000/MPU-6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf). This addition is approximately 30 net firmware lines because the recording also needs provenance and units; no estimator, PID or flight code is added.

## Your next real test

Build/upload the **bench** environment yourself in PlatformIO. Close Serial Monitor before starting the recorder. Keep the drone motionless on a fixed support; any orientation is fine. Do not hold it by hand. Allow it to warm up on that support first; the recording will show whether temperature and bias are still drifting.

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-gyro --port COM3 --seconds 60
```

Use your actual COM port. Press Enter when ready. The recorder reads and preserves five seconds of settling data, then selects 60 seconds for statistics. Keep still until it prints `Saved evidence`. No six-pose sequence is needed for this stationary gyro-bias observation.

The terminal shows elapsed time, gyro XYZ and temperature about once per second. Files are saved automatically in a timestamped `artifacts/gyro-recordings/` folder:

- `serial.log`: original bytes, including settling and configuration responses.
- `samples.json`: selected measurements after settling, with raw counts and converted values.
- `summary.json`: firmware identity, current offset words, candidate bias, covariance, drift and temperature range, or the failure reason.

Original captures remain private and ignored by Git. A failed capture remains saved. The recorder sends identity queries only; it never uploads, resets deliberately, writes gyro offsets or applies calibration. Opening a USB serial connection can depend on the board's reset wiring; any resulting source/state mismatch is rejected.

## What we calculate, and what we still need to check

For a motionless sensor, model one angular-rate observation as

\[
\boldsymbol\omega_k=\mathbf b_g+\boldsymbol\epsilon_k.
\]

If the drone moves, that expression also contains real angular velocity: a sample mean then mixes movement with bias. The software cannot independently certify that your support was motionless.

Use the first two-thirds of the samples as the estimation segment:

\[
\widehat{\mathbf b}_g=\frac1N\sum_{k=1}^{N}\boldsymbol\omega_k,
\qquad
S_{ij}=\frac1{N-1}\sum_{k=1}^{N}
(\omega_{k,i}-\widehat b_{g,i})(\omega_{k,j}-\widehat b_{g,j}).
\]

Report the three sample standard deviations as \(s_i=\sqrt{S_{ii}}\). Keep all off-diagonal entries; do not assume independence or a joint Gaussian distribution. Temporal correlation means these deviations divided by \(\sqrt N\) are not automatically valid uncertainties of the bias estimate.

Reserve the final third for an offline check:

\[
\mathbf e_{\rm held}=\overline{\boldsymbol\omega}_{\rm held}-\widehat{\mathbf b}_g.
\]

That segment does not alter the candidate. A persistent residual or changing temperature suggests drift, movement or temperature sensitivity. A small residual only supports this one stationary interval; it does not validate scale, mounting, all temperatures or flight performance. The reported straight-line rate drift is a diagnostic, not a fitted compensation law.

The sensor output is configured nominally at 1 kHz, but this simple polling/printing lesson reports near or below 100 Hz. It does not record every internal sample. With the existing wide DLPF bandwidth, these subsampled statistics are **not** final noise parameters for a 500 Hz Kalman filter; aliasing and correlation require a separate correctly timed acquisition lesson. Nor is a minute sufficient for long-term Allan/bias stability conclusions.

After this capture, review the original data, time history, temperature and held-out error. Only then prepare a small gyro-bias application lesson, followed by independent rotation/frame checks. No PID or attitude integration is introduced here.

## Evidence for this preparation

The firmware compiles and the offline checks exercise packet units, invalid metadata/conversions, covariance and held-out bias estimation. They model arithmetic and protocols on the PC; they do not execute MCU firmware or prove sensor accuracy. See [preparation verification](../measurements/mpu6050/2026-10-25-gyro-observation-preparation/verification.json).

The older `record-accel` command is for `ACCEL_XYZ_V1` at its Git checkpoint. This lesson emits `IMU_XYZ_V1`; use `record-gyro`. Old raw-pose and tilt protocols still belong to their preserved raw-firmware checkpoint.

## Review: recording integrity and remaining calibration

A sufficient sample count alone does not prove a full recording. The recorder now requires device timestamps to span 90-110% of the requested duration, rejects repeated/backward timestamps and gaps above 100 ms, and records the actual duration/largest gap. The 100 ms threshold tolerates ordinary polling jitter; it is not a promise of capturing all internal 1 kHz samples. Firmware fingerprints and offset words must also agree at both ends. Failed originals remain saved. Both identity-query attempts are listed in the command audit.

**Gyro calibration is mandatory before attitude estimation.** This observation firmware deliberately leaves gyro correction unapplied. The next calibration lesson must apply an accepted measured bias exactly once, restore it each boot and verify it on fresh stationary and restart recordings. No current gyro recording was available at this review; no bias has been fabricated.
