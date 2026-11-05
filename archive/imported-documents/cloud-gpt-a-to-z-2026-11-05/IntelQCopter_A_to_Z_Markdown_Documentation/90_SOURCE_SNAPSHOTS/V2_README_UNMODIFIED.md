# IntelQCopter Fast 500 Hz FC v2 — calibrated 7-PID + altitude Kalman (SAFE BENCH BUILD)

**Not flight-ready. Hardware ESC output is compile-time locked off and GPIOs are inputs. Remove propellers and disconnect ESC signal wires.**

This version upgrades the November 4 successful ESP32-S3 sensor driver with actual MPU6050 hardware FIFO, MPU offset registers, direct BMP280 fast integer compensation, quaternion attitude estimation, a 3-state relative-height Kalman filter, **seven complete Kp/Ki/Kd PID loops**, a 921600-baud Xbox PC bridge and statistics on assumption-based mass/inertia uncertainty.

## Start it — no root START_*.ps1

Run from `project/` in Windows PowerShell:

```powershell
py -3 -m pip install -r requirements.txt
.\tools\run_fc.ps1 -UploadFirmware  # flash + 30s logged bench run
.\tools\run_fc.ps1                  # just capture without flashing
.\tools\run_fc.ps1 -Port COM3       # specify port only if auto CH343 detection fails
py -3 tools\xbox_uart_bridge.py --inspect  # verify axis IDs
.\tools\run_fc.ps1 -Xbox            # virtual Xbox PID/mixer only
```

Reports go to `results/fc_*.log`. A good log shows `BOOT_BIAS still=1`, `BOOT_BARO zero=1`, `FC_BOOT ... fifo=1`, followed by `FC` near 500 Hz. **No results here are yet verified on the actual board.**

## Sensor profiles — important physical limits

| Sensor | Setting | Realistic output | I²C rate |
|---|---|---|---|
| MPU6050 | DLPF 188Hz gyro/184Hz accel, divider=1, 12B FIFO | 500Hz nominal | 400 kHz |
| QMC5883P | `0x3D` C1 / `0x08` C2 / `0x06` sign | 200Hz nominal (Nov 4 observed ~195Hz) | Try 400 kHz; sticky 100 kHz fallback on THIS board |
| BMP280 | normal, P×1 / T×1, IIR×4, standby 0.5ms | 166.7Hz typical, NOT 200Hz | 400 kHz |
| TOFSense F2 Mini | optional separate RX, checksum Frame0 | 50Hz maximum | UART 921600 |
| Xbox | Windows PC/gamepad -> CH343 with CRC | ~50 command frames/s | UART 921600 |

`platformio.ini`: `-DIQ_LIDAR_ENABLE=0` until the dedicated hardware RX pin is confirmed. To enable, set `IQ_LIDAR_ENABLE=1` **and** `IQ_LIDAR_RX_PIN=<verified pin>`, and check voltage/pin sharing first. Xbox is a PC bench bridge, not an independent BLE flight radio.

## Configuration and calibration

- `include/flight_core.hpp`: exact candidate gyro, acceleration and magnetometer constants from Nov 4; 7 PIDs, full 3×3 matrix corrections and quaternion estimator.
- `include/altitude_kf.hpp`: linear conditional state-space 3-state altitude Kalman, Joseph update, NIS gates, baro/optional LiDAR.
- `include/pid.hpp`: filtered derivative on measurement, integral limit and anti-windup.
- `include/bmp280_comp.hpp`: Bosch direct trim/ADC compensation with one burst, reference unit-tested.
- `include/tofsense_frame0.hpp`: LiDAR UART Frame0 checksum/data parser, reference unit-tested.
- `src/main.cpp`: I²C, hardware offsets, FIFO management, sensor scheduling, packet input, virtual FC telemetry.
- `data/model_uncertainty_PID_v2.json`: 100,000-draw assumption-prior MC, with seven complete PID candidate sets.
- `data/rate_step_synthetic_v2.json`: 300-plant-per-axis synthetic 500Hz controller step-response uncertainty.
- `docs/DEEP_ENGINEERING_ANALYSIS_v2.md`: all equations, system assumptions, thresholds, limitations, sources, uncertainties and validations.
- `tests/test_core.cpp`: host C++ unit checks.

### Critical conditions before any motor-power test

Physical IMU/MAG-to-body axes remain unverified. LiDAR wiring remains unverified. Battery/thrust/ESC lag are unmeasured. The nominal historical 45% cap is too close to the simulated hover point. No independent controller-loss or physical disarm validation exists. Therefore this version does **not** implement ESC output or arming. Inspect the engineering analysis and satisfy the bench checklist before starting any actuation software.
