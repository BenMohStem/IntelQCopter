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
- The aircraft is disconnected for PCB repair. No COM3 session, flash, physical
  test, motor-output test, or flight validation is part of the current pass.
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
  MAG samples/s, while a later 400 kHz full-stream capture had 4,001 valid
  packets and zero fresh MAG samples. A completed calibration capture contains
  fresh MAG data, but the exact firmware/configuration lineage must be pinned
  from its raw records before selecting a release setting. The current local
  `platformio.ini` edit selects 400 kHz; it is uncommitted and unvalidated.
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
- At the start of this plan update, `platformio.ini` and `src/main.cpp` had
  uncommitted changes. The `main.cpp` SHA-256 changed during read-only inspection
  (`3AAFFB577532B79676F6C29302E0D3C7FCC94BF36A75528533DB06F7FD07B6DA` to
  `A5A01360326C2FF10B2DBC9EF4F62253920E2AF3BF7EFBEE0D63EF021DEE3670`). Treat
  that candidate as actively changing, not audited or validated. The current
  observed `platformio.ini` SHA-256 is
  `EABB92D2F405C620166C024E992FB140C69BD3A1FEA496A50E56F484C474D1D8`.
- Initial static-review flags from the changing candidate include the 100 kHz
  to 400 kHz bus edit, user-reported LiDAR presence with unverified electrical
  interface/routing/configuration, a serial pilot-command parser
  whose controller/bridge endpoint still needs verification, and a 10 ms I2C
  transaction timeout against a 2 ms loop target. Recheck against a pinned
  source revision; this is an issue list, not a complete code review.
- Do not alter or commit those firmware edits as part of this plan update. Do not
  claim a build result until a safe, reproducible build is run on a pinned source
  snapshot and recorded.

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
