# ESP32-S3 sensor calibration bench

Updated: 2026-11-07. This folder contains the desktop calibration tool and a
separate ESP32-S3 bench firmware. It is not the Pico flight-controller runtime.
ESC/motor output and physical arming remain disabled.

## Launch

The calibration GUI, binary decoder, session writer, plots, and Windows Tcl/Tk
runtime preparation are all in one Python file:
`tools/live_calibration_gui.py`. The PowerShell launcher calls that file
directly; there is no second Python GUI bootstrap file.

From the QCopter repository root, the recommended launcher starts the installed
firmware without flashing:

```powershell
.\qcopter.ps1 calibration-gui
```

This starts the GUI without flashing. Add `-UploadFirmware` only when you
intend to build and flash the ESP32-S3 bench firmware. `-Check -Seconds 20`
runs a serial preflight and exits without opening the GUI:

```powershell
.\qcopter.ps1 calibration-gui -UploadFirmware -Check -Seconds 20
```

To start the GUI directly with Python, install its dependencies if needed and
run the same single source file:

```powershell
python -m pip install -r .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\requirements.txt
python .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\tools\live_calibration_gui.py `
  --port COM3 --gyro-samples 30000 --accel-samples 5000 `
  --mag-turn-samples 5000 --mag-tumble-samples 20000 `
  --bmp-samples 1000 --settle-seconds 3
```

The direct Python command opens the GUI only; it does not build or upload
firmware. It uses the same 12 phases and session-saving behavior as the
PowerShell launcher.

From `C:\Users\Mohamed\Desktop\Work` instead of the repository root, prefix
the path with `QCopter\`:

```powershell
python .\QCopter\hardware\bench_calibration\esp32s3_qmc5883p_100hz\tools\live_calibration_gui.py `
  --port COM3 --gyro-samples 30000 --accel-samples 5000 `
  --mag-turn-samples 5000 --mag-tumble-samples 20000 `
  --bmp-samples 1000 --settle-seconds 3
```

Live chart Y scales follow the displayed time window and use each sensor's
physical units; the latency view remains explicitly unavailable without
measured firmware timing fields.

The GUI also supports `-Wifi` for the ESP32 AP/UDP path. Cable-free capture is
not hardware-validated yet. Do not assume that selecting Wi-Fi proves the AP,
telemetry, or LiDAR forwarding works.

## Guided calibration stages

The competition run has 12 phases: 30,000 stationary gyro samples, six accel
faces, four magnetometer rotations, and 1,000 BMP samples. LiDAR setup, geometry,
and range phases are deferred until after the competition. Existing telemetry
parsing can still read v4 frames, but the GUI does not prompt for a LiDAR port or
collect LiDAR calibration samples.

## Capture storage

New sessions are written under `measurements/calibration/esp32s3/sessions/`.
Only accepted samples for the active phase are stored; stale/rejected packets
and unrelated sensor channels remain in memory for plots and health status. The
PowerShell launcher defaults to 30,000 gyro, 5,000 per accel face, 5,000 per
MAG yaw/pitch/roll turn, 20,000 MAG tumble, and 1,000 BMP samples. Raw MAG XYZ is
kept for ellipsoid fitting. A CSV is created lazily on the first accepted
sample. Completed sessions auto-save under `complete/`; a deliberately saved
partial run goes under `incomplete/` with missing phase counts in a final META
row. Starting another session offers save-incomplete, discard, or cancel when
the current session is unfinished. No empty startup CSV, skip manifest, or new
`_part02` file is created. Derived calibration and combined outputs go under
`measurements/calibration/esp32s3/analysis/`.

Live altitude supports both pressure encodings present in ESP32-S3 v3 firmware:
whole Pa and centi-Pa. It accepts only physical-range values (30–120 kPa),
establishes a median reference from the first 30 fresh valid BMP readings, and plots
relative barometric height. The plot reports invalid pressure or reference
progress instead of silently remaining blank. This display is not fused height
and does not establish terrain-relative altitude.

## Sensor profile targets (not validation claims)

| Device | Requested starting point | Evidence needed |
|---|---|---|
| MPU6050 | 500 Hz loop; FIFO; ±500 °/s; ±8 g; DLPF 188/184 Hz first, then 256/260 Hz trial, 98/94 Hz only if 188 Hz fails | Register readback, 14-byte FIFO framing, real cadence, overflow/loss, noise, filter delay and deadline data |
| QMC5883P | Shared 400 kHz I²C; normal 200 Hz low-OSR (`CTRL1=0x3D`, `CTRL2=0x08`, `XYZ_SIGN=0x06`); poll without waiting for DRDY | Fresh XYZ count/rate at 400 kHz with MPU and BMP operating concurrently; bus electrical measurements and error counters |
| BMP280 | Fastest verified useful rate, nominally about 166.7 Hz at ×1/×1 and 0.5 ms standby | Actual fresh conversion intervals, filter noise/delay and pressure-disturbance rejection |
| TOFSense-F2 Mini | Up to 50 Hz; preserve module timestamp, raw slant range and status | Confirm SKU, power/logic, connector/wiring, selected interface, frame semantics, accuracy on measured targets and installed noise |

Every 500 Hz iteration must proceed without waiting for sensor DRDY/INT,
conversion completion or LiDAR. It should use the latest coherent sample from
each device, preserve native timestamps and distinguish fresh from reused data.
The <2 ms target applies to measured per-tick sensor service time; it does not
mean BMP/LiDAR convert at 500 Hz or that end-to-end display delay is below 2 ms.

The firmware worktree is being changed by DSH. A PlatformIO build flag or source
comment is not proof that all three sensors work at 400 kHz. Do not call any
requested rate verified until the pinned build, register readbacks, sensor
freshness, transaction timing and capture evidence support it.

## Latency and telemetry

The GUI has separate Live, Temperature and Sensor Latency tabs. LiDAR geometry
and calibration controls are deferred until after the competition.
The latency tab plots MPU, QMC, BMP and LiDAR age and service time, plus rolling
P50/P95/P99/max and per-sensor error/deadline counters. Current v2/v3 packets
are 52 bytes and do not carry these per-sensor metrics; the tab correctly stays
empty and says so. Firmware v4 is specified as a 92-byte CRC-protected frame
with per-tick ages/service times, LiDAR range/status/time, freshness and valid
masks, error counters and control deadline overruns. See
[`docs/esp32-competition-readiness-plan-2026-11-06.md`](../../../docs/esp32-competition-readiness-plan-2026-11-06.md).

The GUI keeps v2/v3 compatibility. A 500 Hz packet stream or GUI redraw rate is
not a substitute for sensor-native timestamps or measured service durations.
Wi-Fi packet loss, sequence gaps, CRC errors and reconnect behavior must be
reported separately before a cable-free capture is considered complete.

## Geometry and safety

Use body axes +X forward, +Y left, +Z up; the LiDAR points along body −Z. The
current estimated F450 sensor position is `(0,0,-0.026658) m` from COM, derived
from the user-reported PCB datum. It is preliminary, editable and not measured.
The full geometry is
`H_COM = -(R r_s)_z - D (R d_s)_z`, with `d_s=(0,0,-1)` and body-to-world
`R=Rz(yaw)Ry(pitch)Rx(roll)`. The centered level-ground simplification is
`(D+0.026658) cos(roll) cos(pitch)`. The beam may hit a different terrain patch
when tilted; distinguish that clearance from navigation height and terrain
height. Never re-zero on a new range or a suspected airborne reboot.

Before wiring the F2 Mini, verify its supply requirement (manufacturer-listed
4.3–5.2 V) and 3.3 V TTL logic. Never connect 5 V logic to ESP32 RX. Remove
propellers and isolate ESC power/signals for all bench work. Do not arm motors.

## Existing evidence and limits

The November 3 integrated preflight captured MPU/BMP packets but zero MAG-valid
and zero MAG-fresh values; that result remains a real failed trial. Earlier
isolated QMC runs produced fresh readings, but do not prove that the present
shared bus/wiring works. The current DSH source edits and the requested 400 kHz,
188 Hz and BMP rates are not physical validation. No LiDAR interface or
airframe lever arm has been independently surveyed in this work session.

The large November 6 capture has no saved launch arguments, and its accepted
rows include multiple attempts/counts that differ from the GUI's default
targets. It is **not** classified as a 100% complete session and must not be
included in calibration fitting until its intended targets and provenance are
established. It is preserved in the dated legacy archive with a SHA-256 entry;
no completeness claim is inferred from its former skip manifest.
