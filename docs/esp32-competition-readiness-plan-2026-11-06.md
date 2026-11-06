# ESP32-S3 competition-readiness plan

Updated: 2026-11-06 (Africa/Lagos)

## Purpose and source

This plan converts the user-supplied “Senior aerospace-control…” specification
into a staged, evidence-gated engineering track. The source attachment is
`Pasted text.txt`, SHA-256
`B446DDC4278E9C41A563EE1E56410D16E769C15D1EA0E8774636CF99664B7EC9`.
It is a design request, not proof that a setting, driver, calibration or control
law works on the aircraft. The attachment estimates the competition is about two
days away; the event date has not been independently confirmed, so keep the scope
small until its schedule is known.

## Current boundaries

- Work on the isolated `feature/esp32s3-competition-fc` branch. Keep the current
  Pico/RP2350 and RP2040 architecture and its worktrees separate. Do not rewrite
  or combine other AI worktrees.
- The user reports the aircraft is connected as of 2026-11-06. Its repair state,
  fitted sensors, and wiring have not been independently inspected in this plan
  update. No COM3 capture, flash, physical test, motor-output test, or flight
  validation was performed for this update.
- Physical arming and ESC output remain disabled under `AGENTS.md`. Any candidate
  state machine or mixer is software-only; an `ARMED` message is not evidence of
  physical actuator control.
- The QMC firmware's I2C rate, MPU acquisition rate, BMP conversion rate, UART
  rate, and control-loop rate must each be measured and reported separately.
  A 500 Hz packet stream alone does not prove a 500 Hz sensor conversion or a
  500 Hz closed-loop response.
- Label each result `COMPILED ONLY`, `UNIT-TESTED`, `SIMULATED`, `BENCH-TESTED`,
  `MOTOR-TESTED`, or `FLIGHT-VALIDATED`. Keep failed captures and the previously
  usable fallback; do not silently promote a candidate.

## Evidence checked against the current project

- The QMC5883P identity was observed as address `0x2C`, chip ID `0x80`. The
  isolated low-OSR normal-mode profile `CTRL1=0x3D`, `CTRL2=0x08` produced
  188.16 fresh conversions/s over 15 seconds at 100 kHz I2C. This is an isolated
  sensor result, not proof of full-system operation.
- Full-system records conflict: one 400 kHz UART preflight reported 166.8 fresh
  MAG samples/s, while another 4,001-packet 400 kHz stream recorded zero fresh
  MAG samples. The completed calibration capture contains fresh MAG data, but
  its precise per-transaction bus clock must stay linked to its build
  configuration. The currently committed DSH `platformio.ini` selects 400 kHz;
  this source setting does not establish reliable QMC operation at that rate.
- The calibration package currently describes a 12-phase GUI without LiDAR and
  an isolated 200 Hz QMC result. The complete November 4 session is the current
  full-session calibration candidate; its fits remain provisional, and the
  QMC/HMC PCB identity and physical axes are unresolved.
- UART at 921600 baud has produced CRC/sequence-valid 500 Hz telemetry in
  preflight. That does not establish continuous sensor freshness, estimator
  validity, Xbox transport, motor response, or flight performance.
- Existing notes report different BMP rates under different configurations.
  Recompute fresh conversion intervals from `baro_new` timestamps; never count a
  reused pressure value as a new observation or claim 200 fresh samples/s without
  evidence.
- The PCB shows an HMC5883L footprint while recent captures identify a QMC5883P.
  Resolve the actual fitted sensor, SDA/SCL route, power and pull-ups after repair.

## Selected bench configuration and evidence status

Use this as the **reproducible sensor-calibration baseline** while DSH's current
flight-controller changes are being reviewed. It is not a flight release. The
existing full-session capture and isolated MAG probe are the hardware evidence;
newer source edits are not proof until tied to a build hash and capture.

| Area | Selected configuration | What the evidence establishes |
|---|---|---|
| MCU / board | ESP32-S3 DevKitC-1 N8, Arduino framework, PlatformIO Espressif32 7.1.3 | Used by the calibration project and handoff. No new DSH firmware build is recorded in this plan update. |
| Sensor bus | SDA GPIO9, SCL GPIO8. MPU6050 expected at `0x68`; QMC5883P at `0x2C`; BMP280 at `0x76` through MPU bypass. Keep Wi-Fi disabled. | IDs and sensor data were observed in the November 4 capture. The board drawing still labels an HMC5883L footprint; the physical part and repaired routing need confirmation. |
| I²C speed | Selected strategy: initialize/default at 100 kHz; use 100 kHz for all QMC reads/writes; switch to 400 kHz only around MPU/BMP transactions, then return to 100 kHz before QMC access. Do not run QMC at a global 400 kHz without a separately accepted test. | The captured calibration source uses this split-speed pattern; the isolated 100 kHz QMC probe sustained fresh data. Integrated global-400 kHz results conflict. Bus waveform and rise time are unmeasured. |
| QMC identity / range | QMC5883P, address `0x2C`, ID register `0x00 = 0x80`; `CTRL2 0x0B = 0x08` (±8 G); raw conversion `µT = counts / 37.5`. | Identity and raw conversions agree with the capture audit. Calibration values remain sensor-frame candidates; board-to-body orientation is unresolved. |
| QMC chosen ODR | Normal mode, low-OSR `CTRL1 0x0A = 0x3D` (nominal 200 Hz). Boot sequence: suspend with `CTRL1=0x00`, wait 20 ms, write `XYZ_SIGN 0x29=0x06`, write `CTRL2=0x08`, then write `CTRL1=0x3D`. | Isolated 100 kHz probe measured 188.84 fresh/s sustained (a second run measured 186.82/s). The complete November 4 session used `CTRL1=0x3D` and recorded 38,652 fresh flags at about 195.47/s in active stages; its exact bus/config lineage must be retained with that capture. |
| QMC read / freshness | Use a STOP after the register-pointer write, then a separate read START. Read status `0x09`; count fresh only when DRDY bit 0 is set and the six XYZ bytes at `0x01–0x06` are read successfully. Reject overflow bit 1. Decode XYZ little-endian. Polling may run at 500 Hz; that is not the MAG conversion rate. | The isolated 200 Hz probe used STOP+START and reported zero XYZ failures and zero overflow in its sustained run. Do not infer that this alone fixed every later integrated failure. |
| Calibration-capture MPU profile | MPU6050 ID `0x68`; `PWR_MGMT_1=0x01`, `PWR_MGMT_2=0x00`, `CONFIG=0x03` (42/44 Hz DLPF), `SMPLRT_DIV=0x01`, `GYRO_CONFIG=0x00` (±250°/s), `ACCEL_CONFIG=0x00` (±2 g), FIFO disabled, polled 14-byte output. Convert gyro with 131 LSB/(°/s), accel with 16,384 LSB/g. | This is the profile associated with the November 4 calibration audit. The saved stream has approximately 2 ms packet spacing. It does **not** validate the proposed high-range FIFO flight profile. |
| Future FC MPU candidate | Target 500 Hz with `CONFIG=0x01` (188/184 Hz filters), `SMPLRT_DIV=0x01`, `GYRO_CONFIG=0x08` (±500°/s; 65.5 LSB/(°/s)), `ACCEL_CONFIG=0x10` (±8 g; 4,096 LSB/g), and experimental 14-byte `FIFO_EN=0xF8`. Keep temperature enabled if included in FIFO; do not set `TEMP_DIS`. If using a bounded 56-byte batch (four frames), log backlog/age and state whether older frames are integrated or discarded. Read back every register. | **Not proven on hardware.** Existing calibration biases/matrices were generated under the older profile and must be checked for transfer before use. The FIFO path still needs alignment, timing, age, overflow, and physical gravity/zero-rate verification. |
| BMP280 calibration capture | BMP280 ID `0x58`, expected address `0x76`; old captured source requests normal mode, temperature ×2, pressure ×1, IIR ×4, and 63 ms standby. It polls/labels pressure on a 5 ms schedule and logs raw ADC plus compensated SI values. | The complete session has 32,956 `bmp_new` flags (~166.67 flags/s), but this host cadence conflicts with the 63 ms configured standby and does not prove 166.67 independent conversions. Confirm actual register readback, data-ready state, and conversion timestamps. |
| Future BMP candidate | Normal mode, temperature ×1, pressure ×1, 0.5 ms standby; `CTRL_MEAS=0x27`. Start with IIR ×2 (`CONFIG=0x04`); compare ×4 (`CONFIG=0x08`) using timestamped data and step/noise measurements. Read status before marking a conversion fresh. | Datasheet-derived target only. Do not label cached reads fresh or claim a proven 200 Hz pressure conversion rate. The expected nominal ceiling for the stated profile is about 166.7 Hz. |
| UART / GUI | 921600 baud, 8N1, Wi-Fi off; binary v3 packet is 52 bytes with `A5 5A` sync, 500 Hz scheduled packet cadence, sequence/timestamp, validity/freshness flags, and CRC-16/CCITT-FALSE over bytes 0–49 in bytes 50–51. | CRC-valid, sequential 500 packet/s streams were observed in preflight and in capture segments. This proves the telemetry stream, not a 500 Hz control loop or fresh conversion from every sensor. |
| Calibration flow | Keep the 12-phase GUI: 30,000 stationary MPU rows; six accelerometer faces ×5,000; yaw/pitch/roll MAG turns ×2,000 each; MAG tumble ×8,000; BMP ×1,000. No LiDAR stage. | The November 4 10:09 session reached every target, skipped no stages, and has 98,868 rows plus a matching result JSON. This is the only identified complete session. |
| Actuators / Xbox | Motor pins remain inactive; physical arming disabled. Windows USB/serial Xbox bridge is bench-only until a real flight receiver and adapter are verified. | No actuator, thrust, motor-order, receiver-loss, or flight behavior has been validated. |

### Proven results and calibration limits

- **HARDWARE OBSERVED — isolated QMC:** with `CTRL1=0x3D`, `CTRL2=0x08`,
  `XYZ_SIGN=0x06`, 100 kHz I²C and STOP+START reads, one 15-second run measured
  188.16 fresh conversions/s; other recorded runs measured 188.84 and 186.82/s.
  These results validate the isolated sensor profile on the tested setup, not
  the current DSH flight-controller rewrite.
- **HARDWARE OBSERVED — completed sensor calibration:** the 2026-11-04 10:09
  capture has 98,868 packet rows, all 12 phases complete, no skipped stages,
  38,652 MAG-fresh flags, 32,956 BMP-fresh flags, and no sequence gaps within
  contiguous recorded phases. MAG-fresh rate is about 195.47/s in those phases.
  The gaps between phases are operator pauses, not packet loss.
- **HARDWARE OBSERVED — separate integrated 400 kHz preflight:** one 8-second
  run produced 4,001 valid 500 Hz packets and 1,002 fresh MAG flags (166.8/s),
  with zero CRC errors and sequence loss. Another 4,001-packet 400 kHz stream
  recorded zero fresh MAG. Keep both results; 400 kHz is not yet a reliable
  selected MAG bus rate.
- **OFFLINE RECOMPUTED — MAG candidate:** on the 14,000 accepted fresh MAG
  calibration rows, the independent SPD fit reports hard-iron offset
  `[-29.4823, -18.3980, -35.4743] µT` and symmetric correction matrix
  `[[0.973367, 0.005037, 0.038278], [0.005037, 0.964535, -0.017912],
  [0.038278, -0.017912, 1.067011]]`. Correct as
  `Cmag * (raw_sensor_uT - hard_iron_uT)`; the fit gives 1.2504 µT corrected
  magnitude SD (3.069% CV) in-session. This is **provisional**, not a flight
  calibration: the field magnitude scale is inferred, sensor-to-body rotation
  and heading sign are unresolved, and powered-motor interference was not tested.
- **OFFLINE RECOMPUTED — accelerometer / gyro:** the audit's accelerometer bias
  is `[1.26466, -0.05910, -0.67874] m/s²`; six-face fitted RMS is about
  `0.0533 m/s²`, with correction matrix
  `[[0.999284, -0.038751, -0.032968], [0.041351, 0.992794, 0.040449],
  [0.042481, -0.040867, 0.987205]]`; apply `Cacc * (a_measured - bias)` once.
  Same-face holdout is repeatable, but leave-one-face-out residual reaches
  `0.355 m/s²`, so absolute pose/mounting error remains. The 60-second REST gyro bias at
  about 28 °C is `[+0.16197, -0.30495, -0.87743] °/s`, with single-sample SD
  `[0.04773, 0.04486, 0.04320] °/s`. Bias changed across other sessions and
  temperatures; these are session-specific candidates, not immutable flight
  constants.
- **NOT PROVEN:** QMC 400 kHz reliability, the new ±500°/s/±8 g FIFO profile,
  BMP independent conversion cadence at the proposed settings, LiDAR output or
  terrain fusion, body-axis/heading correctness, Xbox receiver behavior, a
  500 Hz closed-loop controller, motor response, or flight.
- **Not selected:** QMC continuous mode or `CTRL1=0xC3`. Earlier attempts did
  not produce a stable sensor stream. Do not continue to higher-rate MAG modes
  until the 200 Hz profile and full-system bus behavior are repeatable.
- **Fallback and failed alternatives:** the historical v13 100 Hz `CTRL1=0xC9`
  profile at 100 kHz produced about 89.7–90.6 fresh/s. It belongs to that
  register-map/profile variant; do not treat `0xC9` as a simple ODR edit to the
  current QST Rev. A `0x3D` profile. Restore that archived driver as a whole if
  it is needed as a fallback. The 200 Hz
  high-OSR `CTRL1=0xCD` trial repeatedly lost status/readback or held DRDY low;
  it is not the fallback. Continuous `0xC3` attempts did not establish a stable
  stream. These are separate probe trials, not evidence for the current DSH code.

The user reported the drone connected on 2026-11-06. This update did not access
the port or interact with the hardware. At the time of writing, DSH had an
uncommitted edit in the active ESP32 calibration `src/main.cpp`; keep that
source out of this plan-only change and re-audit it after DSH pins a revision.

## Decisions applied from the new specification

| Topic | Plan decision |
|---|---|
| I2C 400 kHz | Keep as a comparison target, not a production requirement yet. Compare against the 100 kHz isolated QMC baseline under identical full-sensor load, with register readback, fresh-data counts and repeated captures. Preserve the known fallback. |
| QMC 200 Hz | Start with normal mode `CTRL1=0x3D`, `CTRL2=0x08`; verify the actual write/readback and independent fresh conversions. Defer continuous mode and higher-rate experiments until after the near-term release window. |
| MPU profiles | Audit current register writes and sample source first. Proposed FAST188 is the initial candidate; BALANCED98 is a fallback; ULTRA256 is experimental. No automatic profile switching. Require FIFO overflow, frame alignment, age and timing evidence. |
| BMP280 | Measure the actual configured conversion rate and age. The spec's approximately 166.7 Hz example is a calculation for its stated oversampling/standby settings, not a measured rate for this board. Use timestamped reuse at the control rate. |
| LiDAR geometry | The user reports a central underside TOFSense-F2 Mini on the F450, beam along body −Z, with preliminary sensor-to-COM offset `(0,0,-0.026658) m`. Follow the separate geometry/altitude-fusion note; measure the assembled offset and verify interface/timestamps before use. Do not restore a LiDAR stage to the existing 12-phase GUI. |
| Xbox mapping | Use the new proposal as the planned mapping: right-stick deflection slews bounded roll/pitch angle targets; left-stick X changes yaw heading when reliable; left-stick Y commands vertical rate in altitude mode; RT/LT have mode-specific bounded trim or manual-throttle behavior. Verify transport and document the mapping before implementation. This supersedes the earlier draft for planning, not as a claim about a working receiver. |
| Arming | Retain the repository lock. The spec's long-press/combination idea may be analyzed in software, but it does not authorize physical arming, motor output or a flight test. |
| RP2350 | Keep as a later migration after the ESP32 reference source and its validation evidence are preserved. Do not migrate the competition candidate now. |

## Staged work and acceptance gates

### 0. Protect and audit the current candidate

- Record branch, `HEAD`, staged files, worktree edits and hashes before each
  review. Preserve all concurrent edits and do not include them accidentally in
  documentation commits.
- Complete a source audit of the ESP32 candidate: existing feature, exact source
  file, compile configuration, sensor register writes/readback, calibration
  loading, GUI and protocol, Xbox transport, motor-pin mode, error paths, known
  bugs, and evidence for every claim.
- During the initial 2026-11-06 audit, `platformio.ini` and `src/main.cpp` had
  uncommitted changes. The `main.cpp` SHA-256 changed during read-only inspection
  (`3AAFFB577532B79676F6C29302E0D3C7FCC94BF36A75528533DB06F7FD07B6DA` to
  `A5A01360326C2FF10B2DBC9EF4F62253920E2AF3BF7EFBEE0D63EF021DEE3670`). The
  then-observed `platformio.ini` SHA-256 was
  `EABB92D2F405C620166C024E992FB140C69BD3A1FEA496A50E56F484C474D1D8`. These
  are historical candidate hashes, not validation of the current DSH source.
- Initial static-review flags from the changing candidate include the 100 kHz
  to 400 kHz bus edit, user-reported LiDAR presence with unverified electrical
  interface/routing/configuration, a serial pilot-command parser
  whose controller/bridge endpoint still needs verification, and a 10 ms I2C
  transaction timeout against a 2 ms loop target. Recheck against a pinned
  source revision; this is an issue list, not a complete code review.
- Do not alter or commit those firmware edits as part of this plan update. Do not
  claim a build result until a safe, reproducible build is run on a pinned source
  snapshot and recorded.
- At this plan-only revision, `HEAD` is `b0697a6` and DSH has an additional
  uncommitted change to `hardware/bench_calibration/esp32s3_qmc5883p_100hz/src/main.cpp`.
  Preserve it; do not include it in the plan commit. Re-pin and review after DSH
  says the source is ready.

### 1. Establish a stable sensor and telemetry baseline

- Pin the last full-session calibration firmware, configuration, calibration
  JSON, raw CSV, and source hashes. Preserve the 200 Hz isolated QMC profile as
  a candidate fallback, while distinguishing isolated from integrated results.
- On repaired, identified hardware, measure MPU fresh sample rate/FIFO behavior,
  QMC fresh rate and age, BMP conversion cadence, shared-bus occupancy, UART
  throughput, errors and missed deadlines. Compare 100 kHz and 400 kHz without
  changing other variables. Keep motors locked out.
- Verify MPU range/DLPF/FIFO by register readback. Avoid calling an acquisition
  profile validated from compile-time constants alone.
- Use a binary telemetry frame with sync, sequence, timestamp, firmware/config
  ID, CRC, validity/freshness flags, sample age, timing and per-sensor error
  counters. Keep serial writes nonblocking and retain raw PC-side logs.

### 2. Preserve and extend calibration

- Keep the existing GUI and its 12-step gyro/accelerometer/MAG/barometer path.
  Keep automatic diagnostics usable without the GUI; do not add a LiDAR stage.
- Keep standalone LiDAR distance/offset checks outside that GUI. Plan them only
  after the reported F2 Mini identity, electrical interface, body alignment, and
  independently measured target distances are verified. See
  `lidar-geometry-and-altitude-fusion-2026-11-06.md`.
- Load versioned/checksummed calibration at boot. Validate sensor identity,
  register configuration and CRC. Keep raw measurements; apply hard/soft-iron
  and accelerometer matrices exactly once. Never replace saved calibration with
  weak boot samples or automatically re-zero during an ambiguous restart.
- Keep the November 4 fit provisional until independent computation reproduces
  it from verified fresh raw rows, axes/units are reconciled, and the sensor on
  the repaired PCB is identified.

### 3. Build the smallest safe control candidate

- Start with a lightweight quaternion estimator using real timestamps and
  validity/age checks. Reject stale or repeated data as fresh independent
  observations. Degrade yaw behavior explicitly when magnetic heading is not
  reliable; study error-state Kalman fusion later.
- Implement the proposed cascaded roll/pitch angle-to-rate, inner rate, yaw-rate,
  altitude and vertical-rate loops only after their command and sensor contracts
  are verified. Use anti-windup, saturation and bounded command slew without
  limiting disturbance response.
- Derive gains from measured or clearly labeled estimated mass, inertia, motor
  authority, delay, noise and filter latency. Keep simulation-only three-term
  candidates separate from any hardware candidate; do not invent validated PID
  coefficients.
- Identify the real Xbox transport (BLE, supported USB host or PC bridge) and
  test end-to-end timestamps, dead zones, loss detection and the planned mapping.
  A Windows serial bridge is not a flight receiver.
- Keep barometric altitude, LiDAR beam-hit terrain height, local vertical
  position, velocity and observable terrain estimate distinct. Apply the full
  configurable lever-arm equation and gate by range/status, attitude geometry,
  sample age and innovation. The offline helper is not a flight estimator; no
  reset-on-range behavior or ground truth is assumed.

### 4. Validate software, then stop at the authorized boundary

- Safe software checks: parser/CRC and sequence handling, quaternion math, PID
  saturation/anti-windup, stale sensor rejection, sensor dropout, timing
  statistics, boot/reset states, motor-mix bounds and simulation under inertia,
  battery-sag and command-loss uncertainty.
- Report mean/P95/P99/worst-case loop time, deadline misses, FIFO backlog, sample
  age, I2C occupancy, UART drops and task utilization. A nominal 2 ms schedule
  alone is not proof of deterministic 500 Hz operation.
- Freeze a candidate only with exact source/config/calibration hashes, measured
  registers, test evidence, fallback, unresolved assumptions and a rollback path.
- Hardware, motor and flight gates remain blocked until PCB repair, actual sensor
  and actuator mapping verification, a concrete authorized adapter, a separate
  guarded procedure, and the user's explicit change to the current physical
  arming restriction.

## After the near-term release window

Keep experiments in a separate branch: QMC continuous mode and OSR comparisons,
MPU ULTRA256 and anti-alias/notch studies, thermal calibration, Allan deviation,
motor/thrust/ESC characterization, inertia measurement, pressure disturbance,
advanced Kalman fusion, battery sensing, and eventual RP2350 migration.
Autonomous flight remains deferred. No higher-rate or higher-order design is
accepted merely because it is theoretically faster or more complex.
