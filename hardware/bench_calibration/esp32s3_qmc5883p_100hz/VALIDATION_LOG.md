# Calibration package validation log — 2026-11-03

Engine: physical ESP32-S3 board, identified by esptool on CH343 COM3. Assumed
QMC5883P `0x2C` / ID `0x80`, SDA GPIO9, SCL GPIO8, I²C 100 kHz, 100 Hz QMC,
500 Hz MPU target, and 921600-baud UART. Board revision, QMC wiring/power and
sensor operation are not independently verified. Motors remained input-only;
no motor test occurred.

## Offline checks

Command: run each `tools/test_*.py` directly with the shared Python 3.15
interpreter. Result: **10/10 PASS**.

- `test_calibration_math.py`: gyro, six-face accelerometer, 3D ellipsoid,
  BMP and synthetic calibration JSON/header output passed.
- `test_mag_3d_grid.py`: equal 3D scale, grid panes, directional triad and
  coverage metrics passed.
- `test_mag_probe_capture.py`: MAG probe parser passed.
- `test_mag_quality.py`: known-center recovery, residual/distribution holdout
  and independent field-shift warning passed.
- `test_multicsv_thermal.py`: multi-session thermal fit and live-capture
  close/save behavior passed.
- `test_pipeline.py`: binary framing, partial packets, CRC, field offsets,
  units, recovery, stage flow, fresh-only MAG/BMP and CSV passed.
- `test_timing_altitude_combine.py`: propagation, covariance, delayed/tilt
  gates, session averaging and GUI timing columns passed.
- `test_unified_calibration.py`: optional LiDAR stages, UART decoder, CSV,
  attempts, auto calibration and launcher passed.
- `test_v5_workflow.py`: temperature uncertainty, reference gates, raw CSV,
  redo provenance and optional LiDAR parser passed.
- `test_wireless_and_thermal.py`: UDP discovery/CRC and four-session thermal
  characterization passed.
- GUI `--self-test`: framing, partial packets, CRC, offsets, units and recovery
  passed.

An initial `python -m unittest discover` invocation found zero registered
unittest cases (exit 5); this package uses standalone test scripts. The scripts
above were then executed directly and passed.

## Firmware compile and hardware runs

- `platformio.exe run -e esp32-s3-calibration-500hz`: **PASS**, no upload.
  RAM 19,408 / 327,680 bytes; flash 295,993 / 3,342,336 bytes.
- Full bench firmware upload to COM3: **PASS**, ESP32-S3 identified and flash
  hash verified. This was the calibration firmware, not flight firmware.
- `qcopter.ps1 calibration-gui -Port COM3 -Check -Seconds 20`: **FAIL for MAG**.
  9,023 packets, 490.2 Hz, 161 fresh BMP samples, MAG-valid 0, MAG-fresh 0,
  CRC errors 0, sequence loss 0. Raw output is in
  `captures/usb_preflight_20261103_214043.csv`.
- `tools/start_calibration.ps1 -Port COM3 -MagProbe -Seconds 20`: diagnostic
  firmware upload succeeded, but the capture was **invalid**. The entire log is
  only `ESP-ROM ... DOWNLOAD(USB/UART0) waiting for download`; no v13 probe
  banner or measurements appeared. See
  `captures/mag_probe_v13_100hz_20261103_214259.log` and its JSON summary.
  No true all-power removal was confirmed, and the user later said they had
  flashed another firmware. The current COM3 firmware state is unknown.

The imported package also contains an earlier isolated v13 result:
`captures/mag_probe_v13_100hz_20261103_213504.log` reports 1,683 fresh samples
at 89.64 Hz with ID/control readback confirmed. It includes one initial status
read failure before subsequent valid samples. This historical capture shows
that isolated QMC output was observed previously; it does not establish the
current integrated firmware or current board state.

## Conclusion

The QMC STOP+START code compiles and uploaded. It did not restore live MAG data
in the integrated preflight. The isolated probe did not boot, so it cannot
classify the QMC, power, bus, or bypass. The calibration GUI is available but
must not be used to capture a magnetic calibration until the current firmware
is known and the integrated preflight reports fresh magnetic samples.
