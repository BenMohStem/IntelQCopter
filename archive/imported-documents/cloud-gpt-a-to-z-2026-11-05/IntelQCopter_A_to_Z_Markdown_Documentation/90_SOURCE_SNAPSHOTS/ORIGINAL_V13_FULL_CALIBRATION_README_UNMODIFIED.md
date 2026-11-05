# IntelQCopter - ORIGINAL v13 QMC5883P driver integrated in full calibration

## Why this version
Your real 2026-11-03 captures show the v13 MAG-ONLY firmware at 100 Hz / C1=0xC9 / C2=0x08 produced around 90 fresh samples/s, while v4 full calibration produced **zero** magnetic samples despite valid MPU ~490 Hz and BMP readings. Another v13 run did produce measurements for several seconds then lost I2C access. This shows the driver once worked but not that hardware or power is permanently stable.

This project **reuses the actual v13 register transaction, mode-write and polling logic** in the full 52-byte binary UART calibration application. It removes the new QMC STOP+START fallback, alternate status-throttling behavior and sentinel filters. QMC status 0x09 is read at a 2ms polling interval with no external DRDY or INT wire. QMC is configured last, *after* full MPU6050/BMP280 initialization and a v13-style MPU bypass reassertion, so subsequent initializers cannot disturb the configuration. **The effects of these changes have not been verified on your board.**

## Run (PowerShell in project root)

First flash + check:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware -Check -Seconds 20
```

If MAG freshness is nonzero and stream quality acceptable, open calibration GUI without another flash:

```powershell
.\tools\start_calibration.ps1
```

To reflash without preflight:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware
```

Default port is auto-detected (CH343), optionally use `-Port COM3`. No root START_*.ps1 files. Full firmware: 921600 baud, Wi-Fi OFF, 100 kHz I2C, QMC5883P 0x2C with CTRL1=0xC9/CTRL2=0x08/XYZ_SIGN=0x06, target MPU 500Hz, BMP ~10 Hz. All motors INPUT ONLY.

## Preserved calibration defaults

- Gyro: 30000 stationary samples.
- Accel: 5000 samples *per face* across 6 orientations.
- Mag: 2000 samples for each yaw/pitch/roll turn plus 8000 tumble samples.
- Barometer: 1000 samples.
- Settle time: 3 seconds.
- LiDAR: available in GUI, optional; use its own separate USB/UART adapter and skip from the GUI if absent.

## Important physical evidence

v13 `-MagProbe` at 100Hz (115200 baud) was **successful** on 2026-11-03: verified control register `0xC9` and around 89-91 actual fresh readings per second. A later v13 probe stopped returning I2C data several seconds into the run. A magnetometer calibration cannot succeed without real fresh magnetic samples. If it still fails in the full application, check power rails, shared I2C bus loading, wiring, MPU bypass, and signal integrity; code transplantation cannot correct intermittent electrical faults.

## Validation and safety

Changes have been checked by source tests and off-device C++ syntax checks only. An actual ESP32 PlatformIO build, UART capture and on-board sensor measurement are needed for final confirmation. Motor/ESC pins remain INPUT; this is NOT flight-control firmware. Remove props for bench work.
