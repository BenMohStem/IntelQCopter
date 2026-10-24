# Calibration check history - 24 October 2026

## Earlier software-only revision 100b879

## Scope

Engine: Python host evaluation of actual firmware scalar expressions with float32 rounding, plus a PlatformIO ESP32-S3/Arduino compile. Input poses are the six user-supplied means, not new measurements. Physical sensor/I2C behavior, mounting accuracy and residual noise are unvalidated. No Gazebo, emulator or physical drone was used. Nothing was uploaded.

Both offline checks passed. The numerical input and firmware build refer to the same main.cpp SHA-256:

```text
1c2aeb307c52eb458dbce2bbd3e2a028cde38d83578e315dd7a42c3c94ec7937
```

| Pose | Corrected drone X | Y | Z (m/s²) |
|---|---:|---:|---:|
| +Z | -0.025528 | 0.104269 | 9.800611 |
| -Z | -0.025528 | 0.104270 | -9.812689 |
| +X | 9.859738 | -0.128822 | 0.029693 |
| -X | -9.753563 | -0.128822 | 0.029693 |
| +Y | -0.027560 | 9.831203 | -0.023656 |
| -Y | -0.027560 | -9.782098 | -0.023655 |

The float32 firmware expressions match the saved double-precision fit to within 0.000001 m/s² for every component. This verifies faithful implementation of the fitted correction, not physical accuracy. The earlier maximum training vector residual remains about 0.1425 m/s².

Deliberately omitting bias subtraction or applying it twice produced a reference mismatch of about 1.424713 m/s², detected by the check. The firmware has no user hardware offset writes; bias is applied once in software.

Embedded compile: espressif32@7.1.3, esp32-s3-devkitc-1, Arduino, Adafruit MPU6050 2.2.9. PlatformIO reported SUCCESS: RAM 19300/327680 bytes; flash 304329/3342336 bytes. That memory limit is the configured application partition, not total board flash.

Compiled firmware.bin SHA-256:

```text
a14e52ab18608b0da9663605d97b11054a7086ec1b15bcd9b1cffbf5c664820a
```

Original host-check output and original compiler output are retained locally under artifacts/calibration-check and artifacts/build. Raw local logs are not included in this public documentation commit.

Run the numerical check again with `./qcopter.ps1 check-accel`; compile with `./qcopter.ps1 build`. Follow the [physical verification guide](accelerometer-calibration.md) after uploading yourself. A board pass still requires actual raw/corrected serial readings in the six poses and independent new orientations, plus a power-cycle check.


## Adafruit + BusIO hardware-trim revision 438e9d6

Adafruit remains the sensor/configuration library; BusIO writes and verifies the MPU offset words. The numerical check MODELS the trim changes (-270,+16,+134), including low-bit preservation examples and quantization. It does not execute I2C, the BusIO helper, or the device's analog offset response. The actual scalar conversion/mounting expressions were evaluated with float32 rounding on all six saved poses against the quantization-adjusted reference. That check passed; deliberately missing/doubled total-bias controls were detected. Independent physical checks remain required.

The embedded PlatformIO build also passed with Adafruit MPU6050 2.2.9 and Adafruit BusIO 1.17.4: RAM 19316 bytes; flash 305157 bytes. No firmware was uploaded. No config readback or offset readback was observed on actual hardware.

Matching main.cpp SHA-256:

```text
bb104bf32dfd235a7ee7281263b2a22045129faa4f880536117ca70983b0c704
```

Predicted corrected output for the original means (m/s²):

| Pose | X | Y | Z |
|---|---:|---:|---:|
| +Z | -0.028149 | 0.104547 | 9.802646 |
| -Z | -0.028149 | 0.104546 | -9.810655 |
| +X | 9.857118 | -0.128545 | 0.031726 |
| -X | -9.756184 | -0.128545 | 0.031726 |
| +Y | -0.030181 | 9.831478 | -0.021621 |
| -Y | -0.030181 | -9.781821 | -0.021621 |

Original local logs are retained under artifacts/calibration-check/20261024T222620874109Z and artifacts/build/20261024T222626810203Z. These raw local logs are not included in the public commit. The previous report and original logs remain preserved.

## Subsequent ±8g / DLPF 94 Hz / 500 Hz output settings

These user-requested settings and the stationary-log analyzer were prepared after the preceding tests. No new build, actual noise recording or hardware result is claimed here. See imu-noise.md.

## Filter revision on 25 October 2026

User requested higher bandwidth. The active setting is now MPU6050_BAND_260_HZ with divider 15 (8 kHz / 16 = 500 Hz), retaining ±8g and ±500 degrees/s. Earlier reports are historical; no new build, timing measurement or stationary noise result was produced for this change.

## Real stationary capture and refinement on 25 October 2026

A receive-only 120-second COM3 capture yielded 291 real samples. All raw/corrected pairs match the prior calibration transform within print rounding. A constrained level rotation and gyro bias were fitted on 174 samples and checked by offline replay on 117 held-out samples. The proposed source passed numerical expression checks and an embedded compile. No upload or post-update physical verification occurred. See [full report, original evidence locations and graphs](precision-2026-10-25.md).
