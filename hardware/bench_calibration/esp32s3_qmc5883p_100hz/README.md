# ESP32-S3 sensor calibration bench

Updated: 2026-11-06. This folder contains the desktop calibration tool and a
separate ESP32-S3 bench firmware. It is not the Pico flight-controller runtime.
ESC/motor output and physical arming remain disabled.

## Launch

From the QCopter repository root:

```powershell
.\qcopter.ps1 calibration-gui
```

This starts the GUI without flashing. Add `-UploadFirmware` only when you
intend to build and flash the ESP32-S3 bench firmware. `-Check -Seconds 20`
runs a serial preflight and exits without opening the GUI:

```powershell
.\qcopter.ps1 calibration-gui -UploadFirmware -Check -Seconds 20
```

The GUI also supports `-Wifi` for the ESP32 AP/UDP path. Cable-free capture is
not hardware-validated yet. Do not assume that selecting Wi-Fi proves the AP,
telemetry, or LiDAR forwarding works.

## Guided calibration stages

The default session has 16 stages. The established 12 gyro, six-face accel,
magnetometer and barometer stages are followed by LiDAR range-reference stages
13–16 at 0.25, 0.50, 1.00 and 2.00 m. Each LiDAR reference requires 300 fresh,
valid readings by default. The user can skip all four when the module or its
forwarded data is unavailable; skipped stages remain marked incomplete.

Measure each target distance from the TOFSense-F2 Mini optical reference to a
flat, matte target. Keep the beam square to the target and record the measured
reference distance. A separate 3.3 V UART adapter may be selected in the GUI;
firmware v4 may instead forward range/status/timestamp over USB or Wi-Fi. Do not
connect LiDAR UART to the 400 kHz I²C bus.

LiDAR analysis requires every requested reference to meet the configured sample
minimum. Example for a complete four-range GUI CSV:

```powershell
python .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\tools\lidar_calibration.py `
  --analyze .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\captures\<capture>.csv `
  --min-samples 300 --required-distances 0.25 0.50 1.00 2.00
```

The LiDAR Geometry tab exposes the preliminary sensor-to-COM vector, attitude
and slant range. Its result is a geometry worksheet, not fused altitude. Save a
geometry configuration only after entering a physically measured lever arm.

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

The GUI has separate Live, Temperature, Sensor Latency and LiDAR Geometry tabs.
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

The large November 6 capture has an empty skip manifest but no saved launch
arguments, and its accepted rows include multiple attempts/counts that differ
from the GUI's default targets. It is **not** classified as a 100% complete
session and must not be included in calibration fitting until its intended
targets and provenance are established. Preserve it as evidence.
