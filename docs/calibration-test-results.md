# Adafruit calibration checks - 24 October 2026

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
