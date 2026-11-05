# Full GUI calibration firmware — original v13 driver integration

The known full calibration project is a **motor-disabled sensor recording and calibration tool**, not an ESC flight controller. It contains `src/main.cpp`, optional v13 `src/mag_probe.cpp`, GUI `tools/live_calibration_gui.py`, Python calibration scripts, UART preflight and the Windows PowerShell launcher.

## Preferred launcher

From extracted project root:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware -Check -Seconds 20
.\tools\start_calibration.ps1       # installed firmware; GUI only
.\tools\start_calibration.ps1 -Port COM3 -UploadFirmware
.\tools\start_calibration.ps1 -MagProbe -Seconds 20
```

The `-MagProbe` mode **temporarily flashes a different 115200-baud text-only firmware**. After running it, the normal 921600-baud binary calibration must be reflashed. Neither command is equivalent to a flight-controller build.

## Historical defaults

30,000 stationary gyro; 5,000 accel per each of six faces; 2,000 MAG per each of three rotations; 8,000 MAG tumble; 1,000 BMP; 3 s settle; optional four LiDAR 300-sample stages via a separate COM port or GUI skip. Auto-detect CH343 COM, GUI at 921600 baud, Wi-Fi off.

## Why preserve it

The full sensor capture generated the evidence behind the candidate calibration matrices. Even after FC v2 improvements, retain this independent calibration tool as an immutable “known-good” reference. Further changes should be tested on a copy.
