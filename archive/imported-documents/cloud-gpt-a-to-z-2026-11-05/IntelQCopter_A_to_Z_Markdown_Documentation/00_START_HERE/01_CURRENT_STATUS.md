# Current project status — 5 Nov 2026

## System-by-system state

| Subsystem | Evidence | Present status |
|---|---|---|
| ESP32-S3 + CH343 USB UART | Uploaded PlatformIO logs and live CSV sessions | Working for calibration captures; occasional COM/reset issues documented |
| MPU6050 at 0x68 | 4 Nov full capture | Real 500 Hz-class data recorded; v2 FIFO path is *proposed but not hardware verified* |
| QMC5883P at 0x2C, chip ID 0x80 | v13 standalone and 4 Nov full capture | Previously intermittent; 4 Nov capture succeeded using **0x3D**, ~195 Hz fresh |
| BMP280 behind MPU XDA/XCL bypass | 4 Nov data | Working during capture; v2 direct compensation path needs on-device test |
| TOFSense F2 Mini | Described hardware, GUI tooling | No accepted 4 Nov LiDAR calibration; physical ESP32 RX pin not verified |
| Six-position accel / stationary gyro / mag rotations | 4 Nov raw archive | Numerically refitted; physical axis and dynamic-interference checks pending |
| 500 Hz FC v2 | Source archive and host tests | Motor-disabled research/bench candidate; hardware timing not yet observed |
| Xbox controller | Windows bridge in v2; GitHub esp32-xbox protocol | Virtual command path written; direct robust ESP32-S3 BLE/RC not verified |
| Motor/ESC thrust curves | Assumption-based studies | Not measured; no verified real motor-control loop |
| Actual flight | No flight logs or successful flight evidence supplied | **Not verified as of this snapshot** |

## Immediate engineering risks

1. Wrong sensor-to-body mapping can invert feedback even if all calibrated vectors look statistically excellent.
2. QMC stability was intermittent in earlier runs, and the successful sensor capture does not prove operation under motor EMI.
3. Hardware-offset-register effects need measured readback and stationary before/after tests; don't subtract a correction twice.
4. Nominal 500 Hz does not guarantee low worst-case latency when I²C transactions can time out.
5. Hover margin is unknown because motor/prop thrust, ESC lag, and battery sag were not measured.
6. Altitude precision is limited by pressure drift, propwash, tilt, vibration and absence of validated ranging.

## Latest artifact

The v2 archive is a **safe bench build** whose `IQ_MOTOR_OUTPUTS_ENABLE=0` compile-time restriction prevents physical ESC output. Do not remove that guard just because the PID arrays or Monte Carlo script return numbers.
