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
normally after a confirmed all-power removal. The full GUI has not yet captured
calibration data.

The imported v4 folder also contains an earlier isolated v13 capture,
`captures/mag_probe_v13_100hz_20261103_213504.log`: 1,683 fresh samples at
89.64 Hz, with QMC ID/control readback confirmed. It recorded one transient
status-register read failure before subsequent samples. This is evidence that
the sensor produced data in an earlier isolated run; it does not validate the
current board state or explain why the integrated firmware reports none.

## Open the GUI

After the preflight passes, run:

```powershell
.\qcopter.ps1 calibration-gui
```

This uses the installed firmware (no upload) and starts the same guided GUI as
the analyzed package. It detects the CH343 COM port, reads at 921600 baud, and
records each phase to a timestamped CSV in the package's `captures/` folder.
Defaults are 30,000 stationary gyro samples, 5,000 samples per accelerometer
face, 2,000 samples for each magnetic yaw/pitch/roll stage, 8,000 magnetic 3D
tumble samples, 1,000 barometer samples, and a 3-second settle countdown. The
calibration path has 12 stages and no LiDAR calibration stage. Keep the current
GUI focused on those stages; the separately attached F2 UART is not part of this
calibration flow. See [TOFSense-F2 Mini geometry and altitude fusion](lidar-geometry-and-altitude-fusion-2026-11-06.md)
for its reported F450 mounting and future estimator validation gates.

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
control path.

## Evidence and limits

The original failed preflight CSVs from the supplied v4 package are preserved in
the package `captures/` folder. Engine/hardware identity for those records:
user-reported ESP32-S3 + QMC5883P hardware, not newly observed in this session;
actual engine is the physical board. Assumed settings are SDA GPIO9, SCL GPIO8,
QMC address `0x2C`, ID `0x80`, 100 Hz `0xC9`, I²C 100 kHz, 500 Hz MPU target,
and 921600 baud. Sensor wiring, board revision, supply stability, and the new
read method remain unvalidated until an on-board preflight records fresh XYZ.
No flight-controller firmware or motor output is part of this package.
