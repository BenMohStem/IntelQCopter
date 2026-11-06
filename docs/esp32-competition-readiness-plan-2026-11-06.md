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

## User-directed sensor and calibration revision — 2026-11-06

This revision supersedes earlier forward-looking decisions in this document
that kept the QMC on 100 kHz, kept the calibration GUI LiDAR-free, or selected
the 42/44 Hz MPU profile as the next capture profile. The targets below still
require implementation and measurement on a pinned source build. The separate
desktop GUI edit adds a completion chime; it does not validate or alter DSH
firmware.

### Shared I²C: fixed 400 kHz target

- Configure one shared bus at **400 kHz** for the MPU6050 (`0x68`), QMC5883P
  (`0x2C`), and BMP280 (`0x76`/`0x77`, confirm the fitted address). Do not switch
  the bus to 100 kHz around QMC transactions and do not silently fall back to a
  slower clock while reporting the run as 400 kHz.
- The QMC5883P datasheet supports I²C Fast-mode up to 400 kHz. That makes this
  a valid target, not a guarantee about the repaired PCB, wiring, pull-ups,
  capacitance, controller driver, or this sensor population. Check bus rise
  time, pull-up values, total capacitance, grounding, transaction framing,
  timeout recovery, NACKs, short reads, and register readback. QST's Fast-mode
  limits include 300 ns maximum rise time and 200 pF bus capacitance.
- Run the MPU, QMC, and BMP together at 400 kHz and measure each sensor's error
  counters and transfer time. Keep the old 100 kHz QMC capture and isolated
  profile as preserved comparison evidence, not as the new operating target.
  If 400 kHz fails, diagnose and correct the cause before proposing a fallback;
  document any fallback as a separate, explicit profile.

### 500 Hz loop: use the latest data without waiting

- The control schedule remains 500 Hz (2 ms period). **No control iteration
  waits for QMC DRDY, a data-ready interrupt, BMP conversion, or a LiDAR frame.**
  Every iteration obtains a coherent snapshot of the latest available
  measurement from every sensor and continues on schedule.
- Keep acquisition nonblocking and track for every sensor: sample sequence or
  conversion counter where available, sample timestamp, last-success time,
  `new_sample`/`reused_sample`, age at the control iteration, and errors. Reusing
  a held sample is allowed; counting it as a newly converted observation is
  not. Mark stale data explicitly and let the estimator apply its documented
  age/degradation policy.
- QMC target remains normal low-OSR 200 Hz (`CTRL1=0x3D`, `CTRL2=0x08`,
  `XYZ_SIGN=0x06`) pending measured results. Poll/read the latest valid XYZ
  without blocking on DRDY. Use DRDY to identify/log conversion freshness and
  overflow, not as a gate that skips or stalls a 500 Hz PID iteration. A 200 Hz
  conversion naturally supplies new data on only about two of every five
  500 Hz iterations; the intervening iterations use the last valid sample with
  its unchanged timestamp and increasing age.
- MPU acquisition should use the FIFO at the 500 Hz sample rate. Poll FIFO
  count without waiting for an interrupt; consume complete, frame-aligned
  records, maintain sample-time reconstruction and bounded backlog, and report
  overflow, partial-frame, stale-frame, and reset counts. Never present a
  repeated register value as a fresh sample merely to fill the loop rate.
- BMP and LiDAR are asynchronous sources. Continue the 500 Hz loop using their
  latest values and age; do not fabricate 500 Hz conversion rates. Sensor
  callbacks/tasks must not hold the PID task in blocking I²C/UART calls.

### MPU6050: bandwidth sequence, range and FIFO

Keep the historical calibration-capture profile and its data provenance
unchanged. For the next bench/control candidate use:

| Profile | `CONFIG.DLPF_CFG` | `SMPLRT_DIV` | Nominal output | Use |
|---|---:|---:|---|---|
| FAST188 first | `1` (`CONFIG=0x01`) | `1` | 500 Hz from the 1 kHz gyro output; gyro BW 188 Hz, accel BW 184 Hz | First candidate, with measured causal software filtering |
| ULTRA256 trial | `0` (`CONFIG=0x00`) | `15` | 500 Hz from the 8 kHz gyro output; gyro BW 256 Hz, accel BW 260 Hz | Try only after FAST188 works; compare vibration/noise, aliasing, latency and control timing |
| BALANCED98 fallback | `2` (`CONFIG=0x02`) | `1` | 500 Hz from the 1 kHz gyro output; gyro BW 98 Hz, accel BW 94 Hz | Use if FAST188 cannot meet measured quality/timing requirements |

Do not use the previous 42/44 Hz `DLPF_CFG=3` capture setting as the routine
fallback. The divider differs at `DLPF_CFG=0`: InvenSense specifies an 8 kHz
gyro output with DLPF disabled, so `SMPLRT_DIV=15` is required for a 500 Hz
sample rate. Using divider `1` there would request 4 kHz and is not the 500 Hz
candidate.

For all three profiles, use gyro ±500 °/s (`GYRO_CONFIG=0x08`, 65.5 LSB/(°/s)),
accelerometer ±8 g (`ACCEL_CONFIG=0x10`, 4096 LSB/g), and enable FIFO for
temperature, three gyro axes, and accelerometer (`FIFO_EN=0xF8`, nominally
14 bytes per complete record). Verify every written register by readback and
confirm actual sample cadence, FIFO ordering/frame length, scale factors,
overflow handling, and timestamp alignment. Existing calibration constants
must be rechecked against the new range and filter profile.

Apply causal software low-pass filtering to gyro/accel only after measuring
bench noise and vibration. Compare FAST188 first, then ULTRA256; return to
FAST188 if 256 Hz increases noise, aliasing, missed deadlines, or estimator
instability. Use BALANCED98 only if 188 Hz also fails. Measure filter attenuation
and phase/group delay; do not hide extra software-filter delay behind the MPU's
internal DLPF. Add a motor-frequency notch only if a measured spectrum supports
it. No filter coefficients or control gains are considered proven before
logged, repeatable bench evidence.

### BMP280 at the fastest useful rate with noise control

- Start from verified BMP280 settings for the highest useful fresh conversion
  rate: normal mode, pressure ×1 and temperature ×1, `tstandby=0.5 ms` (the
  Bosch data sheet's nominal normal-mode ODR is about 166.7 Hz for this
  oversampling/standby combination). Confirm actual registers and observe
  measuring/data-ready behavior; do not count each 500 Hz host poll as a new
  conversion. Use a coherent burst read of pressure and temperature.
- At 500 Hz, reuse each new BMP observation for the intervening loop iterations
  while its age grows. Tune the BMP internal IIR setting and, if useful, a
  causal software pressure filter using stationary noise and controlled
  pressure/airflow disturbances. Report variance reduction and response delay;
  reject stale, inconsistent, or pressure-disturbed observations instead of
  smoothing them into plausible altitude.
- If the requested conversion rate cannot be reached with valid data, keep the
  highest verified rate, report its actual interval, and optimize filtering
  separately. Do not trade data integrity or block the 500 Hz loop to force a
  nominal rate.

### Add LiDAR to calibration and add sensor-latency views

- The desktop GUI now schedules four LiDAR range-reference stages (steps 13–16)
  after the existing 12 gyro/accelerometer/MAG/BMP stages: 0.25, 0.50, 1.00 and
  2.00 m. Each requires 300 fresh valid readings by default. The target distance
  is measured from the optical reference to a flat target; keep the beam square
  to the target. Invalid returns are logged but do not count. The operator can
  connect a separate 3.3 V UART adapter, or use firmware-forwarded LiDAR data in
  the v4 telemetry; missing LiDAR can be skipped and is recorded as incomplete.
  Do not assume it shares the I²C bus.
- The LiDAR analysis action fits measured range against reference range only
  when all four required distances meet the configured sample minimum. It keeps
  the newest redo attempt per distance per capture, reports invalid counts,
  bias, standard deviation, robust deviation and fit residuals, and preserves
  raw CSV values. This is bench range calibration, not validation of the
  aircraft terrain estimator.
- The GUI's LiDAR Geometry tab exposes the configured sensor-to-COM vector,
  roll/pitch/yaw and slant range. It evaluates the full vector geometry and
  beam-hit footprint. The default `(0,0,-0.026658) m` lever arm is preliminary;
  the configuration remains editable and physically unverified until measured.
  Geometry results are an explanatory worksheet only, not an estimator.
- A dedicated Sensor Latency tab plots separate MPU, QMC, BMP280 and LiDAR
  sample-age and per-tick service-time traces. Rolling P50/P95/P99/max, the
  `<2 ms` service target, freshness mask, per-sensor cumulative error counts and
  control-deadline misses are shown. The GUI records v4 metrics in calibration
  CSV rows. Legacy v2/v3 firmware has no per-sensor timing fields, so the tab
  explicitly reports telemetry unavailable and plots no invented values.
- To supply the GUI with physical timestamps and per-sensor metrics, firmware
  must implement the versioned 92-byte v4 packet below on both USB and Wi-Fi.
  Until then, the GUI retains v2/v3 decoding and external LiDAR UART support.
- Keep all descriptions short and plot labels separated. The GUI completion
  chime sounds at countdown end and only after a phase reaches its full valid
  sample target; skips/incomplete stages do not sound as successes.
- Distinguish measured sensor service time from conversion interval, packet-
  arrival interval and GUI redraw time; none of these may be substituted for
  another.
- Target under 2 ms for each sensor's nonblocking acquisition/service time and
  for sensor I/O contribution to the 2 ms control-loop budget wherever the
  hardware permits. Distinguish that from conversion period, sensor-internal
  filter delay, source-to-host age, and GUI display latency. BMP and LiDAR
  conversions are inherently slower than 2 ms; do not label their natural
  sample interval as transaction latency, and do not claim end-to-end latency
  below 2 ms without source/host timing evidence. Report all measured
  distributions, including results that miss the target.

#### v4 timing and LiDAR wire format for DSH

Keep the existing four-byte sync/version/flags prefix and 46-byte v2/v3 base
payload unchanged. Set version byte 2 to `4`, extend the packet to 92 bytes, and
append these 40 little-endian bytes at offsets 50–89; bytes 90–91 hold the
existing CRC-16/CCITT (`poly=0x1021`, initial value `0xFFFF`) over bytes 0–89:

| Extension offset | Type | Meaning |
|---:|---|---|
| 50–57 | 4 × `uint16` | sample age in µs at this 500 Hz tick: MPU, QMC, BMP, LiDAR |
| 58–65 | 4 × `uint16` | sensor service time in µs charged to this tick, same order |
| 66–69 | `int32` | latest F2 slant range in mm |
| 70–73 | `uint32` | F2 module sample time in ms |
| 74–75 | `uint16` | F2 signal strength |
| 76 | `uint8` | F2 status |
| 77 | `uint8` | valid-sample mask, bits 0–3 = MPU/QMC/BMP/LiDAR |
| 78 | `uint8` | fresh-this-tick mask in the same bit order |
| 79 | `uint8` | F2 module ID |
| 80–87 | 4 × `uint16` | cumulative acquisition/error counters: MPU/QMC/BMP/LiDAR |
| 88–89 | `uint16` | cumulative 500 Hz control deadline overruns |

Use `0xFFFF` for unavailable age/service, and saturate cumulative counters at
`0xFFFE`. Age is `control_tick_time - native_sample_time`; it increases while a
held value is reused. The valid mask means the latest held sample is usable;
fresh means a new conversion/sample arrived during this tick. Service time is
the measured nonblocking sensor I/O/parse work charged to this 2 ms tick, not
the sensor conversion period or GUI/network delay. Preserve the actual F2
timestamp, status and raw slant range. Use identical framing and CRC over USB
and UDP; the desktop must report Wi-Fi packet loss/reordering independently.

### Cable-free calibration over Wi-Fi

- Provide a calibration-only ESP32-S3 Wi-Fi AP/UDP transport so the operator can
  tumble the aircraft without a USB cable attached. Keep USB serial as a
  fallback and, when flashing is required, use USB only for that operation; the
  calibration capture itself should not require a cable.
- Audit the existing Wi-Fi attempt before replacing it. Confirm AP startup,
  PC association steps, local UDP bind/handshake, receiver discovery, packet
  cadence, CRC, sequence/timestamp preservation, bounded queue behavior, and
  reconnect/timeout messages. Do not log Wi-Fi passwords or commit new secrets.
- Keep Wi-Fi transmission off the 500 Hz acquisition/control path. Expose queue
  drops, send errors, sequence gaps, CRC errors and receive jitter to the GUI;
  never silently treat UDP loss/reordering as a complete dataset. Count only
  valid received sensor samples toward each calibration stage.
- Compare QMC raw field magnitude/noise and MAG-fresh rate with Wi-Fi disabled
  and enabled while stationary and during equivalent rotation. Wi-Fi telemetry
  is accepted for calibration only if it does not create unacceptable magnetic
  bias/noise or deadline loss; preserve both captures and state the measured
  difference.
- The desktop GUI/launcher already contains a Wi-Fi UDP path (`--wifi`, AP
  address and local receive port) and PowerShell `-Wifi` option. This is
  implementation evidence only; do not call the end-to-end path working until
  the pinned firmware and GUI complete a cable-free capture. **Codex owns the
  desktop GUI/launcher; DSH owns ESP32 firmware and sensor telemetry.** DSH must
  not edit `tools/live_calibration_gui.py` or the PowerShell launcher while
  Codex is updating them.

### Acceptance gates for DSH implementation

1. Pin and report the exact branch/commit/source hash and preserve the currently
   staged and unstaged DSH work. Do not reset, restore, or overwrite concurrent
   edits.
2. Implement the fixed 400 kHz shared I²C setting and demonstrate all three
   sensors working together without clock switching. Record register readbacks,
   bus errors, read durations, QMC fresh conversion rate, and bus electrical
   measurements. Keep all previous failures and comparison captures.
3. Demonstrate a nonblocking 500 Hz loop that advances every 2 ms independent
   of DRDY/interrupts/conversion completions; include per-sensor timestamps,
   age, freshness/reuse, and bounded stale-data behavior.
4. Measure FAST188 with software filtering before trying ULTRA256; use
   BALANCED98 only if FAST188 fails the documented quality/timing gates. Verify
   ±500 °/s, ±8 g, FIFO framing/overflow and actual 500 Hz cadence by readback
   and capture.
5. Measure the BMP280's independent conversion interval and filtering noise vs
   delay. Verify the F2 interface, target-range stream, native timestamp/status,
   and implement the v4 fields above so steps 13–16 and the latency tab receive
   real LiDAR data over both USB and Wi-Fi.
6. Demonstrate a cable-free Wi-Fi calibration capture with valid CRC/sequence,
   bounded loss reporting and a QMC Wi-Fi-on/off interference comparison.
7. Report compile/host-test/bench-capture evidence separately. A compile is not
   a hardware pass. Do not enable motor outputs or physical arming; do not call
   this flight validated. If a bench flash/power cycle enters ROM download mode,
   stop and prompt the user for the required reboot.
8. Document exact test conditions, failed trials, limits and rollback. Commit
   and push the completed milestone under the repository working agreement;
   never claim publication unless the push succeeds.

### Datasheet references used for this revision

- [QST QMC5883P product page and current datasheet](https://www.qstcorp.com/comp_prod/QMC5883P): QMC Fast-mode target and device capabilities.
- [TDK InvenSense MPU-6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf): DLPF bandwidth/delay, sample-rate divider, ranges and FIFO fields.
- [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf): normal-mode ODR, conversion timing, filtering and burst readout.
- [Nooploop TOFSense downloads](https://support.nooploop.com/cn/tofsense/download/): F2 Mini datasheet listing; verify the exact hardware revision and interface before implementation.

### Ready-to-send DSH prompt

> Continue from the current DSH ESP32-S3 source without discarding any staged
> or unstaged work. First report branch, HEAD, source hash, `git status`, and
> summarize the current edits; preserve them exactly. Current desktop changes
> are owned by Codex—do not edit `tools/live_calibration_gui.py`, its LiDAR
> analyzer, or the PowerShell launcher. Use this plan as the firmware work
> order. Keep one shared I²C bus at 400 kHz for MPU6050, QMC5883P and BMP280,
> including all QMC reads. If QMC fails at 400 kHz, diagnose pull-ups, rise
> time/capacitance, voltage, grounding, address, register sequence/readback,
> transfer framing, bus ownership and timeout recovery; do not clock-switch or
> silently fall back to 100 kHz. Measure the three devices together and retain
> error/short-read/NACK and transaction-duration evidence.
>
> The PID schedule is 500 Hz (2 ms). Never wait on DRDY, INT, FIFO-full, BMP
> conversion, or LiDAR reception. At every iteration use a coherent snapshot of
> the latest available data, preserve each native timestamp, increment age for
> reused samples, and set fresh/reused/valid bits correctly. Do not relabel held
> data as a new conversion. Keep bounded stale-data behavior and count missed
> deadlines, I²C errors, FIFO overflow/partial frames and per-sensor acquisition
> failures.
>
> For MPU6050 use ±500 °/s, ±8 g and FIFO (`FIFO_EN=0xF8`, 14-byte frame).
> Start with DLPF_CFG=1 and SMPLRT_DIV=1 (188/184 Hz) plus measured causal
> software filtering. Only after that profile is measured, try DLPF_CFG=0 with
> SMPLRT_DIV=15 (256/260 Hz). Return to 188 Hz if noise, aliasing, phase delay,
> vibration or deadline performance regresses. Use DLPF_CFG=2 (98/94 Hz) only
> if 188 Hz cannot meet the documented gates. Do not use the 42/44 Hz mode as a
> routine fallback. Verify register readbacks, FIFO chronology, loss/overflow,
> reconstructed sample time, units/scales, and actual cadence.
>
> Configure BMP280 for the highest useful verified conversion rate (nominal
> approximately 166.7 Hz at pressure ×1, temperature ×1 and 0.5 ms standby).
> Measure actual fresh conversions; do not call each 500 Hz poll a new sample.
> Keep the PID nonblocking and filter pressure/altitude only after quantifying
> variance reduction, disturbance rejection and delay. Preserve a slower rate
> only if the faster configuration is genuinely invalid or unstable, and state
> the measured reason.
>
> LiDAR is the user-reported TOFSense-F2 Mini at the centered underside of the
> F450, +X forward/+Y left/+Z up, optical axis −Z. Verify its module, electrical
> route, UART/I²C mode, frame/status semantics and native timestamp. The
> preliminary optical-reference point is (0,0,−0.008) m from the lower PCB;
> COM is (0,0,+0.018658) m from that datum, giving provisional
> `r_s=(0,0,−0.026658) m` from COM. Keep this vector configurable and measure it
> physically. For body-to-world `R=Rz(yaw)Ry(pitch)Rx(roll)`, beam `d_s=(0,0,−1)`
> and slant range `D`, use
> `H_COM=−(R r_s)_z−D(R d_s)_z`; the centered level-ground simplification is
> `(D+0.026658)cos(roll)cos(pitch)`. Track the beam footprint, slope/obstacle
> limitations, invalid range/status, tilt, timestamps, covariance and
> innovation rejection. Keep barometric datum, local navigation height,
> vertical velocity, terrain height and beam-hit clearance distinct. Only
> confirmed stationary ground startup may initialize the datum; never re-zero
> on a new range or suspected airborne reboot. Do not enable arming or motors.
>
> Implement the desktop's version-4 92-byte telemetry in both USB and Wi-Fi,
> preserving the 52-byte v2/v3 decoder compatibility. After the 4-byte header
> and existing 46-byte base payload, append 40 bytes at offsets 50–89:
> `uint16 age_us[MPU,QMC,BMP,LIDAR]`, `uint16 service_us[same order]`,
> `int32 lidar_range_mm`, `uint32 lidar_sensor_time_ms`, `uint16 lidar_signal`,
> `uint8 lidar_status`, `uint8 valid_mask`, `uint8 fresh_mask`, `uint8 module_id`,
> `uint16 cumulative_sensor_errors[MPU,QMC,BMP,LIDAR]`, and `uint16 cumulative
> deadline_overruns`. CRC-16/CCITT remains at bytes 90–91 over bytes 0–89.
> `0xFFFF` means an unavailable age/service; saturate cumulative counters at
> `0xFFFE`. Define age from control-tick time to the native conversion time;
> service is measured nonblocking I/O/parse cost charged to that 2 ms tick.
> Fresh bits mean new native sample this tick; valid bits mean latest held sample
> remains usable. Preserve LiDAR status, raw slant range and module timestamp.
> Do not report packet inter-arrival or GUI redraw as sensor latency.
>
> The updated GUI schedules LiDAR references 0.25/0.50/1.00/2.00 m as steps
> 13–16, each requiring 300 fresh valid samples by default, and has a separate
> latency tab. Complete the firmware fields; do not edit the desktop code. Keep
> USB as fallback, but finish AP/UDP Wi-Fi for cable-free tumbling. Keep all UDP
> sends off the 500 Hz path; report queue drops, sequence gaps, CRC errors,
> reconnects and receive jitter. Compare QMC field noise/fresh rate with Wi-Fi
> off/on. No secrets in source or logs. Document current failed trials and
> publish only your own intended firmware milestone after measured evidence.
> Separate compile, host-level, bench and flight evidence; a compile is not a
> hardware pass. If a power cycle or reboot is needed, ask me before proceeding.

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
| I²C speed | Historical November 4 capture profile: initialize/default at 100 kHz; use 100 kHz for QMC reads/writes; switch to 400 kHz around MPU/BMP transactions. This is evidence lineage, not the next target. | The isolated 100 kHz QMC probe sustained fresh data. Integrated global-400 kHz results conflict. Bus waveform and rise time are unmeasured. The user-directed next target is fixed 400 kHz for all three sensors. |
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

### DSH source snapshot and next hardware step — 2026-11-06

The current repository snapshot is branch `feature/esp32s3-competition-fc`,
`HEAD=ac47d4c` (`Implement fixed 400 kHz shared I2C bus for all sensors`). The
worktree is clean and the branch matches its configured upstream. This latest
commit changes only
`hardware/bench_calibration/esp32s3_qmc5883p_100hz/src/main.cpp` (SHA-256
`05B44BA8EF69E63933B56CFA12D1197E34942D664AA53F6A5C11621D0E3B358F`); it is
not a change to the separate flight-controller candidate.

The committed bench-calibration source now configures one 400 kHz I2C bus,
`Wire.setTimeOut(30)`, QMC address `0x2C` with normal-mode `CTRL1=0x3D`,
`CTRL2=0x08`, `XYZ_SIGN=0x06`, and a 5 ms QMC poll schedule. It configures the
MPU6050 for DLPF 188/184 Hz (`CONFIG=0x01`), divider 1, ±500 °/s, ±8 g and
`FIFO_EN=0xF8`. BMP280 is configured with `CTRL_MEAS=0x27` (normal mode,
pressure ×1, temperature ×1), `CONFIG=0x04` (IIR ×2, 0.5 ms standby), and a
6 ms host read schedule. These are implemented register writes, not all
verified register readbacks. The commit description reports 500 Hz packets,
100% sensor-valid packets, zero CRC errors, MAG fresh at 166.7 Hz and BMP fresh
at 166.2 Hz. No new capture/log in the commit ties those figures to the source
hash; retain the earlier integrated 400 kHz success and zero-MAG runs as
conflicting hardware evidence until a pinned, repeatable capture is saved.

Static audit flags in that bench source: its QMC conversion to microtesla uses
`raw * 0.29`, while the 8 G QMC5883P sensitivity is 3750 LSB/G, equivalent to
`raw / 37.5` µT/count. Resolve this scale discrepancy before using the firmware
calibrated vector. The source also marks a successful six-byte register read as
fresh without checking the BMP data-ready status; a 6 ms read schedule alone
does not prove a new conversion on every read. The defaults in the source are
zero hard-iron/gyro/accelerometer offsets and identity scale matrices; the
independent Nov 4 fitted candidates are not loaded automatically.

The separate FC candidate remains
`hardware/experimental_esp32s3_fc_dsh_candidate/src/main.cpp` (SHA-256
`59D2A54BA9ACBBEBBF55060790BB15CB935BAA5A4630C22C07327641061AB815`, last
changed in `ea10ac4`). Its PlatformIO build flag requests 400 kHz, but this
source defines `QMC_I2C_HZ=100000` and switches to 100 kHz for QMC transactions;
its I2C timeout is 10 ms. It therefore does not yet implement the selected
fixed shared 400 kHz requirement. It has the 188 Hz / ±500 °/s / ±8 g MPU
settings and FIFO enabled, but disables MPU temperature while enabling the
temperature FIFO slot. It reads at most one FIFO frame and timestamps it at
host-read time. The 52-byte v3 packet has no LiDAR range or per-sensor age and
service-time fields. Its calibration defaults are zero gyro/accelerometer
bias, identity accelerometer matrix, and invalid magnetometer calibration; it
does not load the independent calibration fit. Its vertical estimator currently
uses raw LiDAR range without tilt or sensor-to-COM correction.

**LiDAR remains an explicit next hardware step, not removed from the plan.** The
user will solder the remaining connection and plug in the centrally mounted
TOFSense-F2 Mini. After that connection, verify its actual UART pins, logic
levels, power, baud rate, packet format, status/quality and timestamps before
accepting data. The current FC candidate uses UART1 RX GPIO1/TX GPIO2 at
115200 baud and parses a TF-style 9-byte `0x59 0x59` frame; compatibility with
the fitted F2 Mini is not yet established. Keep GUI calibration steps 13–16
(0.25, 0.50, 1.00 and 2.00 m) and the LiDAR latency plot in scope. Use a
separate 3.3 V UART adapter for calibration or add the planned v4 forwarded
range/status/timestamp fields. Validate the configurable sensor-to-COM vector
physically and apply the user's tilt/lever-arm equation before fusing altitude.

This status was prepared from repository files only. No build, flash, serial
capture, or physical LiDAR check was performed for this update.

## Decisions applied from the new specification

| Topic | Plan decision |
|---|---|
| I2C 400 kHz | **Superseded by the user-directed revision above:** fixed shared 400 kHz is now the next target for QMC, MPU and BMP. Keep the verified 100 kHz capture as historical comparison/fallback evidence, and diagnose any integrated 400 kHz failure. |
| QMC 200 Hz | Start normal mode `CTRL1=0x3D`, `CTRL2=0x08` on the fixed 400 kHz bus. Never wait for DRDY/INT in the 500 Hz loop; use the latest valid sample every iteration with original timestamp and explicit reuse/age. |
| MPU profiles | Start FAST188 (`DLPF_CFG=1`, divider 1) with ±500 °/s, ±8 g, FIFO and measured software filtering; then try ULTRA256 (`DLPF_CFG=0`, divider 15), reverting to FAST188 on regression. BALANCED98 (`DLPF_CFG=2`) is the fallback only if 188 fails. |
| BMP280 | Measure the actual configured conversion rate and age. The spec's approximately 166.7 Hz example is a calculation for its stated oversampling/standby settings, not a measured rate for this board. Use timestamped reuse at the control rate. |
| LiDAR geometry | The user reports a central underside TOFSense-F2 Mini on the F450, beam along body −Z, with preliminary sensor-to-COM offset `(0,0,-0.026658) m`. Add a GUI range/geometry stage after verifying the fitted module, interface, status, rate and timestamp capability. Measure the assembled offset and keep it configurable. |
| GUI ownership and sound | Codex owns the desktop GUI/launcher: add a completion cue at the end of the countdown and another on every fully completed phase; add the LiDAR calibration and sensor-latency views. DSH owns ESP32 firmware and the data needed by those views. |
| Cable-free calibration | Complete and validate the existing AP/UDP firmware path. Preserve packet CRC, sequence/timestamps and visible loss/error counts. Compare QMC noise with Wi-Fi enabled and disabled before accepting Wi-Fi capture as calibration evidence. |
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
- At the start of this plan/GUI work, the base `HEAD` was `638d876`; the plan
  and GUI change was published in `24d44e1`. DSH has staged and unstaged changes
  to `hardware/bench_calibration/esp32s3_qmc5883p_100hz/src/main.cpp`. The
  working file hash observed during this update is recorded above. Preserve it;
  do not include it in the plan/GUI commit. Re-pin and review after DSH says the
  source is ready.

### 1. Establish a stable sensor and telemetry baseline

- Pin the last full-session calibration firmware, configuration, calibration
  JSON, raw CSV, and source hashes. Preserve the 200 Hz isolated QMC profile as
  a candidate fallback, while distinguishing isolated from integrated results.
- On repaired, identified hardware, measure MPU fresh sample rate/FIFO behavior,
  QMC fresh rate and age, BMP conversion cadence, shared-bus occupancy, UART or
  Wi-Fi throughput, errors and missed deadlines. Target a fixed shared 400 kHz
  bus; use 100 kHz only as a controlled diagnostic comparison. Keep motors
  locked out.
- Verify MPU range/DLPF/FIFO by register readback. Avoid calling an acquisition
  profile validated from compile-time constants alone.
- Use a binary telemetry frame with sync, sequence, timestamp, firmware/config
  ID, CRC, validity/freshness flags, sample age, timing and per-sensor error
  counters. Keep serial writes nonblocking and retain raw PC-side logs.

### 2. Preserve and extend calibration

- Preserve the existing gyro/accelerometer/MAG/barometer phases and add an
  explicit TOFSense-F2 Mini range/geometry calibration stage to the GUI. Keep
  automatic diagnostics usable without the GUI. A missing LiDAR must be shown
  as unavailable/incomplete without preventing unrelated sensor stages from
  running.
- Verify the F2 Mini identity, electrical interface, frame/status fields, range,
  sample timing, body alignment, and independently measured target distances
  before accepting its calibration. Record sensor-to-COM offset as a measured,
  configurable parameter. See
  `lidar-geometry-and-altitude-fusion-2026-11-06.md`.
- Keep the dedicated Sensor Latency GUI tab separate from the temperature tab.
- The calibration capture must support cable-free Wi-Fi operation over the
  ESP32 AP/UDP path as well as USB serial. The transport must retain packet
  sequence, timestamp, CRC validation, and explicit loss accounting.
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
