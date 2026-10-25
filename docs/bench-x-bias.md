# C5a: write and verify only accelerometer X bias

This is a small **partial calibration lesson**, prepared for your upload. The earlier fit and tilted-pose replay support a provisional candidate; the hand-held angle reference does not establish sub-degree accuracy. We now test whether the physical MPU X-offset register produces the predicted shift.

The active `firmware/main.cpp` is a shorter one-axis lesson. The exact previous raw-recorder source is preserved under `archive/bench-checkpoints/raw-pose1000/main.cpp.txt` and in Git at ad6b19b. No motor driver or output pulses are added.

## What the new code does

Only one measured constant is introduced:

```cpp
static const float accel_bias_x = 1.308683406f;
```

It is a SENSOR-X bias in m/s², not an angle and not the total gravity component on X. The constant is retained in ESP32 program flash. The MPU offset word is volatile and is restored by setup after Adafruit initialization. This lesson does not yet apply Y/Z biases, gyro correction, scale or the sensor-to-drone matrix.

The proposed hardware trim is:

$$
d=2\operatorname{round}\left(\frac{1024b_x}{9.80665}\right)=274.
$$

The completed raw recordings consistently reported a post-reset X offset word of -2074. Therefore:

$$
o_{corrected}=o_{baseline}-d=-2074-274=\boxed{-2348}.
$$

The change is even, preserving bit 0's compensation state. The code checks the recorded baseline before writing, checks the signed range, writes a full big-endian word through Adafruit BusIO, and requires exact readback. It uses an absolute target; it never repeatedly subtracts 274 from the current register.

If initialization, configuration, baseline or register checks fail, normal readings stop and an ERROR is printed. A baseline mismatch must be reviewed; do not change the recorded baseline merely to suppress the error. Power-cycle both ESP32 and MPU before retrying, and share logs if the baseline still differs. This check prevents a retained previous trim from being silently corrected a second time.

Under the established MPU6050 trim convention, the nominal X acceleration change is:

$$
\Delta a_x=\frac{274}{2048}\,9.80665
=\boxed{1.312022510\ \mathrm{m/s^2}}.
$$

At ±8 g this is a nominal decrease of 548 ADC counts. Register rounding leaves a modeled residual:

$$
b_x-\Delta a_x\approx-0.003339103\ \mathrm{m/s^2}.
$$

These are predicted values, not measured post-upload results. Trim behavior is based on the [established MPU6050 library implementation](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp); those offset semantics are not fully specified in the public register map, so actual response on this module is precisely what this bench test checks. The code retains Adafruit MPU6050 and its BusIO dependency; no second MPU library is added.

## Follow the source in this order

1. Constants: measured bias, recorded baseline and calculated target.
2. `select_x_offset`: write one absolute X-offset value and verify it.
3. `setup`: start Wire/Adafruit, select ±8 g / ±500 degrees/s / mode-0 filter / divider 7, verify configuration and baseline, restore X bias once.
4. `loop`: read sensor X, convert its ADC counts to m/s² and print. There is no second software bias subtraction.

The register helpers are from Adafruit BusIO. The live X read is checked directly through BusIO so an I²C read failure is reported. The MPU output is configured at 1000 Hz, but this readable bench lesson reports about **10 readings/second** using delay(100). It is not a 500 Hz flight loop or a new high-rate noise characterization.

X will generally **not become zero** on a stationary drone: the tilted sensor mounting gives gravity a real X component. Look for the change of approximately 1.312 m/s² between register modes, not a zero value.

## Upload yourself, then record automatically

1. Build/upload the `bench` environment yourself in PlatformIO.
2. Close Serial Monitor.
3. Keep the complete drone on a fixed support in any convenient stationary orientation. Do not reposition it between phases. Propellers remain removed.
4. Run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-bias-x --port COM3
```

The PC will run three intervals, each with two seconds of settling and twenty seconds of saved readings:

| Interval | Register word | Purpose |
|---|---:|---|
| raw_before | -2074 | Measure with the recorded baseline |
| corrected | -2348 | Measure with X correction |
| raw_after | -2074 | Check movement/drift against the first interval |

It then restores -2348 and verifies the mode acknowledgment. Total time is about 66 seconds. It sends only `?` (report), `r` (baseline) and `c` (corrected) to this known firmware; it never uploads. Its source/configuration check rejects the earlier raw recorder. On interruption it may request corrected mode, but will not claim that an unacknowledged recovery succeeded.

Files save automatically under `artifacts/bias-x-recordings/SESSION/`: original `serial.log`, exact selected samples for each interval, and `summary.json`. They are excluded from public commits. The tool reports expected and observed X decreases, plus the difference between the two raw intervals. It interpolates the raw means to the corrected interval's midpoint to reduce linear drift effects. That cannot cancel arbitrary movement or nonlinear drift; no automatic physical pass is declared.

Send the resulting folder path. We will check register acknowledgments and the actual shift before adding Y or Z. Repeat after reset and after a full power cycle to verify restoration, keeping those runs separate. A register readback alone does not prove analog calibration accuracy.

Manual Serial Monitor inspection is also possible at 921600 baud: `?` reports the current state, `r` selects baseline and `c` selects X correction. Do not open it at the same time as the PC recorder.

## Checkpoint and remaining work

This source change intentionally invalidates the original raw-recording command's source/protocol expectations. For further six-pose or tilted raw recordings, deliberately restore the preserved raw-recorder checkpoint and upload it yourself; do not mix partially calibrated data into the original raw fit.

Y/Z, gyro, matrix rows, physical reset/power-cycle verification, attitude/Kalman code, PID and motor tests remain later steps. The coefficients are provisional and the physical register effect is not yet tested. Sources for initialization/reset behavior: [Adafruit MPU6050 implementation](https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp). Preserve original captures and failures.

## Preparation checks actually completed

The final PlatformIO ESP32-S3 build passed (19,376 bytes RAM; 305,645 bytes flash). Five analytic/synthetic comparison tests passed; none executes the MPU. The binary contains the exact current source SHA, and the raw recorder checkpoint retains its original hash. [Verification record](../measurements/mpu6050/2026-10-25-bias-x-preparation/verification.json). Original compiler logs remain local. No upload, serial connection, analog-response or reset/power-cycle result is claimed.

For the full trace from your measured pose means to the rounded register correction, open [the illustrated offline report](bias-x-provenance.html) or [its LaTeX Markdown source](bias-x-provenance.md).
