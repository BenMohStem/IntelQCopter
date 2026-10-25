# Apply the measured gyro bias, then verify it

This lesson changes only gyro bias handling. The full accelerometer correction is unchanged; no attitude estimator, PID or motor output is added. The firmware stays in [one Arduino main.cpp](../firmware/main.cpp).

## What the code now does

The three accepted provisional constants, in sensor rad/s, are:

\[
\mathbf b_g=
\begin{bmatrix}0.003098404\\-0.004985037\\-0.014435425\end{bmatrix}.
\]

They come from the estimation segments of gyro runs 1-2; the whole third run was held out. See [the actual measurements](../measurements/mpu6050/2026-10-25-three-gyro-runs/report.md). A constant bias is an approximation: temperature and slow drift still matter. No temperature law is fitted from three runs.

1. `gyro_bias` is compiled into ESP flash, like the accelerometer constants. It survives an ESP power cycle as part of the firmware.
2. `restore_gyro_bias()` reads MPU gyro user-offset words at 0x13/0x15/0x17. Their measured calibration baseline was `[0,0,0]`. It accepts that baseline or this firmware's own target. An unknown value stops the sensor stream with an error rather than stacking a new correction.
3. Calculate integer trim from the measured bias. The nominal gyro **offset-register** sensitivity is 32.8 counts/(degree/s); the selected ±500 degree/s **measurement-output** sensitivity is 65.5 counts/(degree/s). They serve different purposes.
4. Write absolute targets `[-6,+9,+27]`, then check readback. The original factory temperature-compensation registers are not modified.
5. For every reported gyro sample, subtract only the software remainder left by rounding the offset word. Full bias is not subtracted a second time.
6. `verify_offsets()` checks accelerometer and gyro user-offset words before each burst. This is bench diagnostics; repeated register reads are not the final 500 Hz flight-loop architecture.

## Why hardware and software corrections do not add the bias twice

Let \(b_i\) be the measured bias in rad/s, and let \(q_i\) be the rounded integer decrement:

\[
q_i=\operatorname{round}\left(b_i\frac{180}{\pi}\,32.8\right),\qquad
o_{i,\mathrm{target}}=o_{i,\mathrm{baseline}}-q_i.
\]

Under the nominal register-gain model, hardware removes

\[
h_i=q_i\frac{\pi}{180\cdot32.8}.
\]

Keep only the remainder \(\delta_i=b_i-h_i\) in the software subtraction. If \(r_i\) denotes the reading at the recorded baseline, then:

\[
\underbrace{(r_i-h_i)}_{\text{reading after MPU trim}}-\underbrace{(b_i-h_i)}_{\text{software remainder}}
=r_i-b_i.
\]

`gyro[axis] = counts[axis + 4] * (PI / (180.0f * 65.5f)) - gyro_remaining_bias[axis];`

The 32.8 sensitivity is a manufacturer nominal value, not a measurement of your chip's exact trim gain. The new physical recording must check sign, magnitude and remaining mean. Arithmetic checks establish the code's nominal model, not the exact analog response or zero drift.

The offset-register scale and locations are described in the [InvenSense hardware-offset application note, section 6](https://gzuliani.github.io/arduino/files/arduino-mpu6050/invensense-hardware-offset-registers.pdf) (manufacturer-authored document hosted on a public mirror). Measurement-output scale follows the [MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).

## What you do now

Upload the current **bench** source yourself. It announces `IMU_XYZ_V2`. Close Serial Monitor before recording:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 upload --upload-port COM3
.\qcopter.ps1 record-gyro --port COM3 --seconds 60 --note "first gyro correction check"
```

Keep the drone motionless on a fixed support in any orientation until `Saved evidence` appears (five settling seconds plus 60 selected seconds). Let temperature settle first; do not hold it by hand. No accelerometer pose sequence is needed. Original logs and samples save automatically into a new private `artifacts/gyro-recordings/` folder.

The recorder verifies the uploaded source identity, current trim words, bias/remainder metadata, and every emitted corrected sample against its raw post-trim counts. It displays actual corrected rates and temperature, then reports the **residual** mean in degrees/s. It does not silently fit and apply another bias. Raw counts in the packet are already downstream of MPU hardware trim.

When this first recording is reviewed, repeat after an ESP reset and then a full power cycle, using `--note "after reset"` and `--note "after power cycle"`. Keep those physical actions separate from the serial command. MPU user offset registers are volatile; the ESP firmware restores them each boot. Do not rewrite factory OTP or assume the MPU stores this permanently.

## Acceptance remains pending

Build and offline checks pass; no upload or new physical test was made by the assistant. The expected target words are verified only by a real recording. Compare fresh stationary means to zero, temperature history and the held-out run's observed residuals (especially X about -0.016 degree/s). Do not promise a tighter accuracy than the measurements support or automatically add a residual from one run.

Rates remain in sensor axes. Known rotations, body-frame mapping, gyro scale, broader temperature behavior, correctly timed 500 Hz acquisition and final Kalman noise remain later steps. A small stationary residual alone does not validate them. Motor pins remain input pulldowns.

The former `IMU_XYZ_V1` recordings and scripts remain reproducible; the same recorder recognizes V1 observation data and V2 corrected data without interpreting them interchangeably. The historical three-run report remains offline replay evidence, not a claim that V2 ran on hardware.
