# IntelQCopter v4 - complete UART calibration + QMC failure triage

This copy is integrated at `hardware/bench_calibration/esp32s3_qmc5883p_100hz`
in the QCopter repository. From this directory, use the package-local launcher:

```powershell
.\qcopter.ps1 calibration-gui
.\qcopter.ps1 calibration-gui -UploadFirmware
.\qcopter.ps1 calibration-gui -UploadFirmware -Check -Seconds 20
```

Without `-UploadFirmware`, it uses the installed firmware and starts the GUI.
With `-UploadFirmware`, it flashes this bench firmware before starting the GUI.
Add `-Check -Seconds 20` to run the UART preflight and stop without opening the
GUI. From the repository root, `.\qcopter.ps1 calibration-gui` dispatches to
the same calibration launcher.

The package launcher delegates to `tools/start_calibration.ps1`.

## Fast workflow (Windows PowerShell)

The same upload and preflight can be run directly:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware -Check -Seconds 20
```

- Resolves the CH343 serial COM port automatically (or use `-Port COM3`).
- Builds and flashes the complete sensor calibration firmware.
- Reads the CRC-protected 52-byte binary UART stream at **921600 baud**.
- Uses 500 Hz MPU target, 100 Hz QMC (`0xC9`) and BMP280 ~10 Hz, Wi-Fi off.
- Fails instead of claiming calibration when magnetometer freshness is zero.

To start the GUI **without reflash**:

```powershell
.\tools\start_calibration.ps1
```

For the GUI after reflash:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware
```

The calibration GUI contains 12 stages: stationary gyro, six accelerometer faces,
four magnetometer rotations, and barometer. LiDAR capture is removed from this
workflow. The live status line distinguishes no UART bytes from incompatible or
corrupt packets and reports valid sensor counts.

## If magnetometer is still missing: ONE direct comparison

```powershell
.\tools\start_calibration.ps1 -MagProbe -Seconds 20
```

This **temporarily reflashes the exact isolated v13 MAG-only firmware** with
100 Hz QMC, I2C at 100 kHz, Wi-Fi off and text logging at **115200**. Follow the
script's true power-cycle instruction. It saves the complete raw log in `captures/`.

A **PASS** in MAG-only probe and **FAIL** in complete firmware indicates a
system-integration issue (bus utilization, BMP/MPU interference, transport/scheduling,
sequence or power loading). A **FAIL** in both after a true power cycle means the
QMC communication/hardware/power problem remains unresolved; repeating calibration
cannot solve it. `0x80` ID or an ACK alone does not prove the sensor is converting.

**Restore full firmware after MAG-only probe**:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware -Check -Seconds 20
```

## Changes from v3

- Preserved original `tools/start_calibration.ps1` and COM auto-selection.
- **Removed** root-level `START_*` and redundant cmd wrappers.
- Added `src/mag_probe.cpp` from working v13 and isolated PlatformIO diagnostic env.
- Full firmware configures QMC immediately after MPU bypass **before BMP280**.
- QMC reads use STOP+START as the primary path after integrated logs reported
  ESP32 repeated-START failures. MPU/BMP reads retain their prior transaction.
  This targeted workaround is not hardware-confirmed until fresh XYZ appears in
  the integrated preflight.
- On eight successive QMC status failures, polling backs off to 10 Hz instead
  of flooding the bus. No auto register reconfiguration and no fake samples.
- Normal UART remains at 921600, 100 Hz QMC profile (`0xC9`, `0x08`, `0x06`),
  500 Hz MPU target and CRC 52-byte packets.
- Bench safety: ESC GPIOs INPUT ONLY. No motor actuation.

## Calibration defaults

| Stage | Default |
|---|---|
| gyro stationary | 30,000 samples |
| accelerometer six faces | 5,000 samples each |
| magnetic axis turns | 2,000 samples each |
| magnetometer 3D tumble | 8,000 samples |
| barometer | 1,000 samples |
| settle | 3 seconds |

## Safety and accuracy

No firmware is flight-validated. Never attempt flight on sensor diagnostics or
without working IMU estimation, motor mapping and independent failsafe checks.
Remove propellers for all captures; do not arm motor outputs. This is a
software-only change; the QMC defect cannot be claimed fixed until hardware logs
show valid data in the **complete** firmware. If both modes fail, inspect 3.3V
power while the QMC is started, I2C pull-ups, solder joints, MPU bypass and
truly remove all power for a cold reset.

## Serial notes

- GUI/preflight calibration firmware: **921600** CH343 UART binary.
- MAG-only v13 probe: **115200** CH343 UART text.
- PlatformIO upload speed may be **460800** or other and is independent of the firmware UART baud.
- Only one program may hold COM port at once.

QMC datasheet: https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf
