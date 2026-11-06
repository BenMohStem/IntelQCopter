# ESP32-S3 guided calibration setup

The guided GUI and its matching firmware live in
[`hardware/bench_calibration/esp32s3_qmc5883p_100hz`](../hardware/bench_calibration/esp32s3_qmc5883p_100hz/README.md).
This is a separate ESP32-S3 bench application; the RP2350 Pico flight firmware
remains in its existing single C file.

## Apply the QMC read fix and verify the stream

The saved 2026-11-03 integrated preflight has 8,910 valid frames over 18.18 s,
but zero MAG-valid/fresh flags and zero XYZ values. IMU and BMP data were present.
The reported bus error points to the ESP32 repeated-START read path. The current
bench firmware now reads QMC registers with STOP+START, while MPU/BMP keep their
existing repeated-START reads. QMC `0x09` data-ready gating remains mandatory;
no cached or repeated values are invented as fresh data.

With propellers removed, ESC signals/power isolated, and only the ESP32 connected
to its CH343 serial adapter, use the explicit upload + preflight command:

```powershell
.\qcopter.ps1 calibration-gui -UploadFirmware -Check -Seconds 20
```

This flashes the standalone **calibration/bench firmware** and then checks the
921600-baud 52-byte stream. The preflight must report nonzero MAG-fresh samples
and measured rate before calibration. The target is 100 Hz; a successful
compile, upload, QMC ID or register ACK alone is not a pass. If fresh MAG data is
still zero, stop before calibration and retain the generated CSV; hardware
power, I²C wiring, MPU bypass, board revision, and the QMC's actual response
remain candidates. Do not keep changing ODR, I²C clock, bus method, and UART
settings together.

The new STOP+START build was subsequently compiled and uploaded to the detected
ESP32-S3 on COM3. Its 20-second preflight produced 9,023 valid frames at about
490.2 Hz, 161 fresh BMP readings, zero MAG-valid/fresh readings, zero CRC errors
and zero sequence loss. This confirms the transaction change compiled and
reached the board, but it did **not** fix the observed magnetometer failure.
The isolated v13 probe was then uploaded, but its 20-second log contains only
the ESP32 ROM message `DOWNLOAD(USB/UART0) waiting for download`; the probe
firmware never started, so this attempt provides no QMC result. The user then
reported testing another firmware. The board and COM3 are left untouched. Do
not treat this invalid probe as evidence of a sensor or power failure. Rerun
`-MagProbe` after the other test is finished, with the board booting the probe
normally after a confirmed all-power removal. A later November 6 capture exists,
but its completion target/attempt provenance is not sufficient to classify it
as a fully completed calibration session.

The imported v4 folder also contains an earlier isolated v13 capture,
`captures/mag_probe_v13_100hz_20261103_213504.log`: 1,683 fresh samples at
89.64 Hz, with QMC ID/control readback confirmed. It recorded one transient
status-register read failure before subsequent samples. This is evidence that
the sensor produced data in an earlier isolated run; it does not validate the
current board state or explain why the integrated firmware reports none.

## Open the GUI

To start the installed-firmware GUI without flashing, run:

```powershell
.\qcopter.ps1 calibration-gui
```

This uses the installed firmware (no upload) and starts the guided GUI. USB
serial uses 921600 baud; Wi-Fi uses the configured AP and local UDP port.
Defaults are 30,000 stationary gyro samples, 5,000 samples per accelerometer
face, 2,000 samples for each magnetic yaw/pitch/roll stage, 8,000 magnetic 3D
tumble samples, 1,000 barometer samples, then four LiDAR references at 0.25,
0.50, 1.00 and 2.00 m with 300 fresh valid samples each. Thus the default
session has 16 stages (LiDAR is steps 13–16), with an explicit option to skip
the four LiDAR ranges. Measure reference distance from the F2 optical reference
to a flat target and keep the beam normal to the target. LiDAR may arrive through
a separate 3.3 V UART adapter or firmware v4 forwarding.

The GUI includes a LiDAR Geometry worksheet with editable `r_s`, roll, pitch,
yaw and slant range, plus a separate Sensor Latency tab. The worksheet computes
full COM-to-beam-hit geometry and horizontal footprint; it is not an estimator.
Firmware v2/v3 has no per-sensor timestamps/service durations, so latency plots
remain blank with an explanation until the 92-byte v4 telemetry is implemented
by DSH. The v4 tab displays rolling P50/P95/P99/max, freshness, per-sensor error
counts and deadline misses. It distinguishes sensor service time from age,
conversion interval, filter delay, USB/Wi-Fi transport and GUI redraw time. See
[TOFSense-F2 Mini geometry and altitude fusion](lidar-geometry-and-altitude-fusion-2026-11-06.md)
for the reported mounting and estimator safety/validation requirements.

For magnetic stages, rotate the whole assembled sensor/airframe slowly around
yaw, pitch, roll and then across unrestricted 3D orientations. Keep nearby steel,
speakers, power bricks, vehicles and high-current wiring away. The GUI does not
gate the pose or verify directional coverage. After capture, analyze one or
more CSV files with:

```powershell
python .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\tools\calibrate.py .\hardware\bench_calibration\esp32s3_qmc5883p_100hz\captures\<capture>.csv
```

Review warnings and the independent validation run before using coefficients.
Current GUI calibration outputs are not connected to the Pico attitude/yaw
control path. The November 6 capture has an empty skipped-stage manifest, but
its per-attempt accepted counts differ from the default targets and there is no
recorded launch-argument manifest. Treat 100% completion as unverified and
exclude it from calibration fitting until its intended targets and attempt
provenance are established.

## Evidence and limits

The original failed preflight CSVs from the supplied v4 package are preserved in
the package `captures/` folder. The records came from the user-reported ESP32-S3
and QMC5883P; hardware was not independently checked during this documentation
update. The old failed profile used SDA GPIO9, SCL GPIO8, QMC `0x2C`, ID `0x80`,
and 100 Hz `0xC9` with a 100 kHz QMC bus. That historical profile is not the
current target. Current source still switches clocks to 100 kHz for QMC, uses
the 42/44 Hz ±250 °/s ±2 g FIFO-disabled MPU profile and emits 52-byte v3
packets. The DSH staged one-line change does not fix those behaviors. The fixed
400 kHz, 188 Hz, ±500 °/s, ±8 g, FIFO and v4 targets remain unverified. No
flight-controller firmware or motor output is part of this package.
