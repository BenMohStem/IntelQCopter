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

## 2026-11-03 follow-up hardware diagnosis

Engine: same physical ESP32-S3 DevKitC-1, silicon revision 0.2, on CH343 COM3.
The board revision and QMC supply/wiring remain assumed; observed I2C address is
0x2C and chip ID is 0x80. Motor GPIOs stayed INPUT; no motor test occurred.

- Before reflashing, COM3 emitted **zero bytes at 921600 baud**. At 115200 it
  emitted `MAG ACK=...` text from a different diagnostic image, with C1=0x00,
  C2=0x00, status 0x18 (DRDY clear), and XYZ=0,0,0. The expected v13 probe
  `BOOT_PROFILE` marker was absent. The earlier zero-sample probe capture
  `captures/mag_probe_v13_100hz_20261103_222554.log` was therefore a
  wrong-firmware capture, not evidence that v13 had failed.
- Reflashed the project calibration firmware. MPU and BMP worked immediately:
  ~490 Hz packet stream, fresh BMP near 10 Hz, zero CRC/sequence errors; MAG
  initially remained at zero.
- Reflashed the isolated v13 QMC probe. At requested ODR 100 Hz it read back
  C1=0xC9/C2=0x08 and recorded 615 fresh samples in 8 s (90.63 Hz). It had one
  transient initial status-read failure and one overflow; later samples were
  valid. Evidence: `captures/mag_probe_v13_100hz_20261103_223107.log` and JSON.
- Tried a temporary 500 Hz MAG-status polling interval in the combined image.
  That build emitted no valid packets, so the experiment was reverted. Do not
  use that interval; the source remains at 200 Hz status polling.
- Restored the normal combined image and reran preflight. **PASS:** 3,008 v3
  packets at 471.3 Hz; MPU valid on all packets; MAG fresh 96.2 Hz; BMP fresh
  near 10 Hz; CRC errors 0 and sequence loss 0. Evidence:
  `captures/usb_preflight_20261103_223853.csv`. A later 4 s confirmation yielded
  1,887 frames at 471.3 Hz, MAG fresh 96.0 Hz, BMP fresh 40, CRC errors 0,
  sequence loss 0 (`captures/usb_preflight_20261103_224928.csv`).
- The GUI had previously been launched while the incompatible text image was
  installed. The launcher now uses `launch_calibration_gui.py`, which supplies
  missing Tcl/Tk script libraries from the installed Python archives in the OS
  temp directory; Python 3.15's Tcl/Tk initialization was incomplete. The GUI
  decoder self-test and the 12-stage pipeline tests pass.
- A header-only GUI capture while idle is expected: sensor rows are written
  only after a phase starts. Use the binary preflight for hardware acceptance
  before starting any phase.

The zero-packet issue was caused by the board running a 115200-baud text
diagnostic rather than the GUI's 921600-baud v3 binary firmware. The board is
currently restored to the normal calibration image; the latest combined
preflight passes MPU, MAG and BMP. These checks demonstrate operation in this
bench setup, not flight validation or validation of unobserved board wiring.

## QMC ODR and maximum-rate follow-up — 2026-11-03

Engine: PlatformIO 7.1.3 / Arduino framework on physical ESP32-S3 DevKitC-1
revision 0.2, CH343 COM3. QMC5883P at 0x2C / ID 0x80, MPU6050 0x68 and BMP280
0x76 are the observed identities. SDA9/SCL8 and sensor rail/wiring are assumed;
pull-up values, signal rise times, breakout revision and QMC supply waveform
were not measured. The QMC data sheet Rev. E lists normal ODR through 200 Hz,
continuous mode up to 1500 Hz and I2C standard/fast mode (100/400 kHz).
Probe-only firmware uses text UART 115200, no Wi-Fi and ESC pins INPUT; the
normal calibration firmware uses 921600 binary UART and 500 Hz IMU scheduling.

- Added explicit isolated 200 Hz (`CTRL1=0xCD`), continuous (`0xC3`, 400 kHz)
  and continuous low-OSR (`0x33`, 400 kHz) profiles. Continuous polling is
  configured at 2 kHz. Added repeated-identical-vector telemetry and made
  acceptance use sensor HEALTH rate rather than decimated CSV rate.
- 200 Hz trial: boot scan saw QMC 0x2C and ID 0x80; MPU bypass and all four
  writes acknowledged. However subsequent readback and STATUS reads failed,
  with no fresh XYZ sample in 10 s. Mid-trial status sometimes read 0x18 but
  DRDY remained clear. Evidence: `captures/mag_probe_v13_200hz_20261103_230337`
  (`.log` and `.json`). **No 200 Hz rate result.**
- Datasheet continuous trial at 400 kHz: scan did not detect QMC; no QMC writes
  or samples occurred. The scan intermittently saw only MPU/BMP, then address
  0x3C, indicating unstable/changed bus visibility. Evidence:
  `captures/mag_probe_v13_continuous_20261103_230424.log` and `.json`.
  Continuous low-OSR `0x33` was not flashed because the bus was already failing.
- Restored full calibration firmware to COM3 (successful flash/hash verify).
  Subsequent two preflights received boot bytes once and then zero bytes / zero
  packets at 921600 baud. Thus this task did not restore verified GUI streaming;
  the board should be fully disconnected from all power for 10 seconds and
  restarted before further troubleshooting. Do not treat the prior 100 Hz
  historical passes as current-state verification.
- The 200 Hz experiment is inconclusive: the sensor was seen at boot but bus
  reads dropped out. The 400 kHz experiment confirms that fast-mode I2C is not
  working on the current physical wiring. Physical cause (power, bus integrity,
  pull-ups, board reset state, or electrical interference) remains unisolated.
  No motor/ESC output was enabled.
- Verification: `test_mag_probe_capture.py` passed; the 200 Hz profile compiled
  without duplicate macro warnings. Full firmware uploaded successfully, but
  UART preflight failed (0 valid packets on post-retry captures). No rate claim
  is made beyond the previously preserved 100 Hz baseline captures above.

## Follow-up: QMC read protocol retry — 2026-11-03

Same engine and hardware assumptions as the previous section: ESP32-S3 DevKitC-1
revision 0.2 / CH343 COM3; QMC ID/address, MPU/BMP identities as previously
observed; GPIO9/8 wiring and rail quality assumed; motors input-only. Inspection
found the standalone MAG probe still used repeated START for register reads,
while the integrated calibration firmware had already moved to STOP+START
because of observed ESP32 Wire read failures. Updated probe register and XYZ
reads to use the integrated firmware's STOP+START sequence.

- Parser regression passed; 200 Hz, continuous, and continuous-fast variants
  compiled with PlatformIO. Continuous modes configure 400 kHz I2C and 2 kHz
  status polling; continuous-fast uses CTRL1=0x33 (low OSR). No hardware speed
  claim follows from compilation.
- Reflashed 200 Hz probe successfully. The subsequent 12 s capture contains
  only ESP-ROM `boot: DOWNLOAD(USB/UART0)` / `waiting for download`; no probe
  banner, QMC scan, or rate data. Evidence:
  `captures/mag_probe_v13_200hz_20261103_231122.log` and `.json`. This is an
  invalid/no-boot trial, not another QMC failure. Requested a full board power
  removal so the flashed probe can boot. Continuous-mode hardware trials remain
  deferred until a valid 200 Hz baseline is obtained.

## Follow-up: intermittent 200 Hz operation — 2026-11-03

Engine/hardware assumptions unchanged: PlatformIO 7.1.3 / Arduino, ESP32-S3
DevKitC-1 revision 0.2 / CH343 COM3, QMC5883P 0x2C ID 0x80, MPU6050 0x68,
BMP280 0x76; SDA9/SCL8, sensor supply, wiring, pull-ups and board revision
assumed. MAG-only diagnostic UART 115200, I2C 100 kHz, Wi-Fi disabled, ESC
GPIOs held INPUT.

- A 100 Hz control using STOP+START register reads reached 89.74 Hz for about
  3 seconds with verified C1=0xC9/C2=0x08, then status reads began failing and
  data went stale. This demonstrates the read/power/bus issue is not unique to
  the 200 Hz ODR. Capture: `mag_probe_v13_100hz_20261103_232852.log` and `.json`.
- A 200 Hz (`CTRL1=0xCD`) STOP+START run produced a real brief burst at
  **192.56 Hz** (32 fresh readings, one overflow, no identical vectors), but
  then stopped updating; status failures accumulated and readback could not
  be maintained. `age_sec` reached >6 s by capture end. It is not a sustained
  pass. Capture: `mag_probe_v13_200hz_20261103_232926.log` and `.json`.
- A later 200 Hz repeated-START A/B run did boot after the preceding manual
  reset, briefly reached 192.06 Hz for 30 fresh readings, then remained stale
  (final age >26 s, registers not confirmed). This matches the high-OSR
  transient seen with STOP+START, so the transaction style was not the primary
  cause. Capture: `mag_probe_v13_200hz_20261103_234347.log` and `.json`.
- Reducing status polling to 250 Hz (`poll_us=4000`) yielded zero fresh samples
  and repeated STATUS NACKs at 100 kHz. Capture:
  `mag_probe_v13_200hz_20261103_233131.log` and `.json`.
- The repeated-START A/B firmware uploaded, but the 12 s serial capture again
  saw only ROM `DOWNLOAD(USB/UART0) waiting for download`; it is an invalid
  no-boot test, not a sensor-rate result. Capture:
  `mag_probe_v13_200hz_20261103_233402.log` and `.json`. User was prompted to
  power-cycle before the next capture. Do not continue COM3 reads until user
  confirms reboot and port is available.
- Capture reporting now records peak and final HEALTH separately and rejects
  a profile unless the final report shows matching profile, confirmed register
  readback, recent data (`age_sec < 1.5`), and sufficient rate. A short 192.56
  Hz burst can no longer be called a pass just because it crossed a rate limit.
- The 200 Hz register selection matches QST Rev. E example `CTRL1=0xCD`. Its
  table describes normal mode as continuous measurement with periodic output;
  the brief burst and later NACK/DRDY loss indicate unresolved implementation
  or physical bus/power behavior. No continuous-mode test has yet met the
  prerequisite of a sustained 200 Hz baseline.
- Prepared a compile-verified 200 Hz low-OSR variant `CTRL1=0x3D` (ODR 200,
  OSR1=1, OSR2=1) at 100 kHz I2C. QST's Rev. E current table gives typical
  normal-mode current 280/1180 µA at 200 Hz for OSR1=1/8 respectively. This
  variant is intended to check whether the high-OSR profile's greater sensor
  load is related to intermittent operation. No hardware trial has been run.

## Sustained 200 Hz and continuous-mode attempts — 2026-11-03

Engine/hardware assumptions: PlatformIO 7.1.3 / Arduino, physical ESP32-S3
DevKitC-1 revision 0.2 / CH343 COM3; QMC5883P 0x2C / ID 0x80 and MPU bypass
observed, SDA GPIO9/SCL GPIO8 and sensor rail/wiring/pull-ups assumed. Diagnostic
firmware UART 115200, I2C 100 kHz unless stated, Wi-Fi off, no ESC output.

- After the user power-cycled the board, the low-current 200 Hz profile
  (`CTRL1=0x3D`, ODR=200, OSR1=1, OSR2=1) ran a 30 s confirmation. HEALTH
  reported 14,730 fresh samples, 188.84 Hz, final age 0.001 s, verified
  C1=0x3D/C2=0x08 on six periodic checks, zero overflows, five status-read
  failures, and 41 repeated adjacent vectors. The result is stable and close
  to nominal 200 Hz but does not reach exactly 200. Evidence:
  `captures/mag_probe_v13_200hz_20261103_234534.log` and `.json`.
- Tried low-OSR continuous (`CTRL1=0x33`) at 100 kHz I2C and 2 kHz poll
  schedule. It produced zero fresh samples; C1 read back 0 and DRDY stayed
  clear (`0x18`). Evidence: `captures/mag_probe_v13_continuous-fast_20261103_234703.log`
  and `.json`.
- After the user rebooted the exact continuous example (`CTRL1=0xC3`) at
  100 kHz, the 20 s capture showed 53,389 not-ready polls, zero fresh samples,
  and later C1 readback 0/C2 readback 0. It is a valid probe run, but not a
  valid continuous conversion result. Evidence:
  `captures/mag_probe_v13_continuous_20261103_235625.log` and `.json`.
- Added a 20 ms settling interval after the selected CTRL1 write and before
  immediate register verification/polling, then will retry C3 once. This is a
  startup-margin experiment; it does not change the selected register value.
- Retried C3 with the 20 ms settle: after an image upload the capture hit ROM
  `waiting for download` (`mag_probe_v13_continuous_20261103_235948.log`), then
  after the user's power-cycle a 15 s run still had zero fresh data, 69,393
  not-ready polls, and periodic C1/C2 readback 0 (`mag_probe_v13_continuous_20261104_000033.log` / `.json`).
- Prepared a further `CTRL1=0x03` continuous comparison at 100 kHz. Upload
  completed, but its capture also saw ROM `waiting for download` and yielded no
  sensor result (`mag_probe_v13_continuous_20261104_000209.log` / `.json`).
  User was prompted to reboot before the next capture. No continuous profile
  has yet produced fresh magnetometer data; no ≥500 Hz result is claimed.
- Then flashed QST's exact continuous example (`CTRL1=0xC3`) at 100 kHz. The
  capture saw ROM `waiting for download`, so the image had not started and no
  QMC result can be inferred. User was prompted to power-cycle again; await
  confirmation before any further COM3 access.
- The probe's capture acceptance now uses final HEALTH (profile, confirmed
  register readback, recent sample age and rate), and treats fresh HEALTH as
  evidence even when the serial-open buffer reset discarded earlier CSV rows.
  This prevents stale bursts from passing and avoids false zero-sample reports.
- The 100 Hz `0xC9` trial with STOP+START had already confirmed intermittent
  status/bus behavior, while the low-OSR 200 Hz trial is stable. This suggests
  reduced OSR/current is beneficial, but current was not measured and causation
  is not established. Continuous mode is not yet working. No ≥500 Hz result;
  the user's target remains unverified.

## DeepSeek v18 triage and comparison — 2026-11-04

Engine/hardware assumptions: DeepSeek v18 no-purchase diagnostic firmware, built
with PlatformIO / Arduino for ESP32-S3 DevKitC-1 N8 and uploaded to CH343 COM3.
The board was identified by USB as VID 0x1A86/PID 0x55D3; serial number
5B8E062789. I2C is assumed 100 kHz, SDA GPIO9/SCL GPIO8, QMC5883P at 0x2C,
MPU6050 at 0x68, and expected BMP280 at 0x76. No oscilloscope, rail voltage,
pull-up, or wiring measurement was made. Motor pins remained INPUT; no motor
test occurred. Warm ESP resets do not remove sensor VDD.

- Copied the supplied v18 diagnostic harness into
  `hardware/bench_calibration/qmc5883p_autodiagnostics_v18`. Its dry-run
  sequence was `T0 F1 H0 A0 G5 T1 A1`; it uses warm resets and does not perform
  a physical sensor power cycle. The offline run could not start under the
  sandbox because PlatformIO's global lock is outside the workspace; the
  elevated run succeeded.
- T0 found QMC 0x2C and MPU 0x68, verified QMC ID 0x80 and parked C1=0/C2=0x08.
  The bus scan did not find 0x76, so BMP280 availability on this wiring is not
  established by this session.
- F1's 50 Hz `CTRL1=0xC5` start write ACKed. It briefly yielded 89 unverified
  CSV rows at 47.72 Hz global; the first two seconds were about 46.95 Hz. The
  stream then stalled: final age 3.086 s, 28 status failures, and periodic
  register reads showed ID 0x80 with C1=0/C2=0. The immediate post-start probe
  could read QMC ID 0x80 through both STOP+START and repeated-START, and MPU
  WHO_AM_I 0x68 through both methods, while QMC register-address ACK and BMP
  reads failed. SDA/SCL sampled high. This is evidence of a post-start sensor
  state or bus-access problem, not proof of a power brownout.
- A0 repeated-START 50 Hz, A1 repeated-START 100 Hz and T1 QST C3 continuous
  all had zero fresh samples. G5's identity-guarded QMC reset was ACKed but
  reset/default readback was not confirmed. Therefore neither read transaction
  style nor QMC soft reset fixed this setup. Full details are in the v18
  session `SESSION_20261104_000750_23688` and its original UART/upload logs.
- Flashed the previously stable project MAG-only low-OSR 200 Hz comparison
  profile (`CTRL1=0x3D`) after triage. Its 30-second capture opened COM3 while
  the ESP was in ROM `waiting for download`; it recorded no v13 marker or
  sensor samples and is not a QMC result. Stop here and request the user's
  power cycle before another serial capture. Evidence:
  `captures/mag_probe_v13_200hz_20261104_001422.log` and `.json`.

No 200 Hz retest or ≥500 Hz continuous result is established by this session.
The actual engine and hardware assumptions above remain unvalidated; do not
attribute the post-start failure to a specific electrical cause until sensor
VDD and bus signals are measured or a successful cold-power comparison is run.

## Calibration GUI setup — 2026-11-04

The user chose to calibrate with the previously stable 200 Hz normal-mode QMC
profile and asked to stop further diagnosis. PlatformIO/Arduino full calibration
firmware compiled and uploaded to ESP32-S3 CH343 COM3 with QMC `CTRL1=0x3D`
(200 Hz ODR, OSR1=1, OSR2=1), 100 kHz I2C, 921600-baud binary telemetry, and
500 Hz MPU target. Motor output remains disabled. The profile's 188.84 Hz result
was measured only in the isolated MAG-only probe; the rate and packet health
under the full sensor workload were not independently verified here.

The GUI presents 12 steps (stationary gyro, six accelerometer faces, four
magnetometer rotations, and barometer), with no LiDAR stages. Shortened prompts,
reduced header height, larger plot-row separation, and the 200 Hz QMC label are
in the saved GUI source. The GUI process was launched after upload. Windows
Computer Use window capture timed out twice, so visual layout and live packet
counts could not be confirmed from the agent view. Board/wiring and the BMP
address remain assumptions; no new sensor diagnosis was performed.

## Restore MAG data in the calibration GUI — 2026-11-04

The user reported an empty MAG plot in the full calibration GUI. The installed
profile was QMC `CTRL1=0x3D` (normal 200 Hz, low OSR) with a 100 kHz I2C bus.
The shared bus also carries 500 Hz MPU bursts and BMP reads. Raised the full
calibration bus to 400 kHz, supported by the selected sensors, leaving the QMC
profile and telemetry format unchanged. More shared-bus timing margin is a
likely reason MAG data returned; only one before/after run was made, so treat
causation as supported but not repeatedly confirmed.

PlatformIO 7.1.3 / Arduino full calibration firmware built and uploaded to the
ESP32-S3 DevKitC-1 N8 on CH343 COM3. An 8 s UART preflight at 921600 baud then
decoded 4,001 v3 packets at 500.0 packets/s, with 4,001 MAG-valid packets, 1,002
fresh MAG samples (166.8 fresh/s in the preflight's MCU-time calculation), 79
fresh BMP packets, 0 CRC errors, and 0 lost sequence numbers. Raw capture:
`captures/usb_preflight_20261104_003805.csv`. The calibration GUI was reopened
on this firmware. Motor outputs remain disabled. The register ODR is configured
for 200 Hz; the measured full-system fresh rate is lower at 166.8 Hz. I2C
pull-up values, bus rise time and actual sensor rail voltage remain unmeasured.
