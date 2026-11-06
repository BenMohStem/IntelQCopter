# Bench calibration changes and evidence

## 2026-11-07 — save only accepted calibration samples

- `tools/live_calibration_gui.py` now writes a CSV row only when the active
  phase accepts a real sensor sample. MAG/BMP phases no longer archive every
  intervening 500 Hz packet; stale and rejected samples stay visible in the
  live GUI but are not stored.
- Rows contain the active calibration sensor's channels: stationary REST keeps
  gyro, accel and MPU temperature; six-face stages keep accel and temperature;
  MAG stages keep fresh raw/corrected XYZ for ellipsoid fitting; BMP stages
  keep fresh pressure/temperature and ADC values; LiDAR stages keep valid
  ranges. The existing CSV columns remain for tool compatibility, with
  unrelated fields blank. MAG/BMP/LiDAR sample rows omit transport sequence
  IDs so normal sparse sampling is not mistaken for packet loss.
- Completed/retried session history and old captures are unchanged. No tests,
  GUI launch, sensor capture, firmware build, upload or hardware access was run
  for this source edit.

## 2026-11-03 — integrate v4 and test QMC read recovery

- Source: user-provided `IntelQCopter_100Hz_UART_Mag_Recovery_v4` from Downloads.
- `src/main.cpp`: changed only QMC register reads to STOP+START, based on the
  reported ESP32 repeated-START failure (`endTransmission(false)` /
  `i2cWriteReadNonStop`, error `-1`). MPU and BMP access retain their previous
  repeated-START helper. The internal QMC status bit still gates fresh data.
- `tools/start_calibration.ps1`: probes `py -3` before use and falls back to the
  installed shared `python.exe`; this PC has Python 3.15 but no registered
  `py -3` interpreter.
- Added the repository command `calibration-gui`; the GUI and calibration
  protocol are unchanged from the analyzed v4 package. No Wi-Fi credentials are
  stored; Wi-Fi remains disabled.

### Verification

- Direct `tools/test_*.py` scripts: PASS (all ten scripts).
- `python tools/live_calibration_gui.py --self-test`: PASS.
- `platformio.exe run -e esp32-s3-calibration-500hz`: PASS; no upload.
- Upload: PASS to ESP32-S3 detected on CH343 COM3; firmware hash verified.
- Preflight command: `qcopter.ps1 calibration-gui -Port COM3 -Check -Seconds 20`.
- Result: FAIL for MAG. 9,023 valid frames, 490.2 packet Hz, 161 fresh BMP
  samples, zero MAG-valid/fresh samples, zero CRC errors, zero sequence loss.
- Evidence: `captures/usb_preflight_20261103_214043.csv`; previous failures
  `captures/usb_preflight_20261103_213127.csv` and
  `captures/usb_preflight_20261103_213351.csv` are preserved unchanged.
- Reset state: ESP32 rebooted by esptool RTS after upload; **no full power
  removal** occurred. No motors or ESC outputs were exercised. The current
  integrated QMC behavior remains unresolved; compile/upload are not sensor
  proof.
- The imported folder's earlier v13 isolated capture recorded 1,683 fresh
  samples at 89.64 Hz with ID/control readback confirmed. It had one early
  status read failure followed by valid CSV samples. This is historical
  evidence of isolated sensor output, not proof of the board's current state.
- An isolated v13 probe was uploaded afterward, but its capture log contains
  only `ESP-ROM ... DOWNLOAD(USB/UART0) waiting for download`; probe firmware
  never ran. The user reported switching to another firmware while this check
  was pending. The probe's zero CSV samples are invalid evidence and do not show
  a QMC failure. The current COM3 firmware state is unknown; no more board I/O
  was performed.

### Next step

After the user's other firmware test is finished, rerun
`tools/start_calibration.ps1 -Port COM3 -MagProbe -Seconds 20`. It uploads the
isolated v13 MAG-only probe and pauses for a true 10-second removal of USB and
all other board power. Confirm the ESP32 boots the probe (not ROM download
mode) before interpreting the capture. If it produces fresh XYZ, reflash full
firmware and rerun the integrated preflight. If not, preserve a valid probe log
and inspect QMC power, board wiring, and MPU bypass before calibration. Do not
run a GUI calibration until the integrated preflight shows fresh MAG data.

### Engine, assumptions, and unvalidated interfaces

Engine: real ESP32-S3 board, identified by esptool on COM3. Assumed hardware:
QMC5883P at `0x2C`, ID `0x80`, SDA GPIO9, SCL GPIO8, MPU6050 `0x68`, BMP280
`0x76` via MPU bypass; assumed 100 Hz QMC (`0xC9`) and 100 kHz I²C. QMC model,
board revision, wiring, bypass, supply integrity and any external adapter
details remain user-reported or unvalidated. The UART path was measured at
921600 baud and yielded valid CRC-protected frames. Motors remained input-only.
