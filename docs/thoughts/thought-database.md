# IntelQCopter — thought database for concurrent models

**Revision 2 — 2026-11-03 23:55 (Africa/Lagos).** Supersedes revision 1 (23:08).
Read this file FIRST before acting on COM3, the ESP32-S3 bench firmware, or hardware.

**Collaborative agent: "GPT 6 Luna".** The user has stated that this project is being
worked jointly by this session and an agent the user calls **gpt 6 luna**. This file is
the shared working memory between us. If you are GPT 6 Luna (or a later version), treat
sections 2, 7 and 11 as binding, append your own findings under section 12 rather than
overwriting history, and keep every capture/log preserved.

---

## 1. Project identity
- Repo: `QCopter/` at workspace root `C:\Users\Mohamed\Desktop\Work`.
- IntelQCopter: F450/Q450 quadcopter. Main architecture (`project.json`): **RP2350
  aircraft + RP2040 handheld** (Pico). The **ESP32-S3** workstream is a separate
  bench-calibration / sensor-diagnostic target, **not flight firmware**.
- Safety agreement `QCopter/AGENTS.md` (rev 2026-10-28): physical motor arming
  DISABLED; no auto-arm; preserve original evidence; record unknown hardware in the
  plan instead of inventing it; local commit + push after every prompt that changes
  the project (never empty commits, never rewrite history, never force-push).
- Motor policy `docs/motor-limits-2026-10-28.md`: **10% armed idle / 45% normal
  per-motor cap / 65% boost cap**, applied to the final coupled allocation.
  Percentages are ESC-command range, **not** measured RPM/thrust.
- Command catalog: `project.json` + `qcopter.ps1`. Pico runtimes stay single-file:
  `firmware/flight_controller/flight_controller.c`,
  `firmware/handler_controller/handler_controller.c`.

---

## 2. Safety / operating rules (binding for every model)
- **COM3** = WCH CH343 USB-UART (`VID_1A86 PID_55D3`) → the **ESP32-S3 bench board**.
  **COM4** (`VID_2E8A PID_000A`, disconnected) → RP2350/Pico native USB.
- **Only one program may own the port.** Before touching COM3, confirm no other
  serial monitor / PlatformIO monitor / GUI holds it.
- The user's instruction of 2026-11-03 ~22:44: *"there is a model working on the
  drone now / just continue and when the drone is available i will tell you"*.
  At 23:31 the user said **"continue analysing and setting up"** and confirmed the
  joint work with **gpt 6 luna**, but has **NOT** yet declared the board free.
  → Do not start a flash/capture that would collide with a concurrent run.
- **Never** enable motor/ESC output, never arm, never spin motors, never remove the
  INPUT-only configuration of motor GPIOs `12, 11, 14, 13`. Propellers off.
- Preserve **all** evidence: never delete or overwrite anything in `captures/`,
  `artifacts/`, `archive/`, or the `GPT_TMP` zips.
- After a real full-power removal, only a **true 10-second all-power removal**
  (USB *and* board power) counts as a power cycle; a USB replug or esptool RTS
  reset does not (documented repeatedly in `VALIDATION_LOG.md`).
- Record the engine/assumptions in every report: which board, which firmware, which
  baud, which I2C clock, warm reset vs cold power cycle.

---

## 3. Hardware identity
| Item | Value | Confidence |
|---|---|---|
| COM3 bridge | WCH CH343 (`oem37.inf`, Started) | verified |
| MCU | ESP32-S3 DevKitC-1, silicon rev 0.2 | reported by esptool |
| SDA / SCL | GPIO9 / GPIO8 | assumed, user-reported |
| MPU6050 | `0x68`, bypass via `0x37` bit1 | observed |
| BMP280 | `0x76` behind MPU XDA/XCL bypass | observed |
| QMC5883P | `0x2C`, chip ID `0x80` | observed |
| I2C clock | 100 kHz (400 kHz **fails** — chip becomes invisible) | observed |
| UART (integrated) | 921600, 52-byte v3 CRC-16/CCITT-FALSE | observed |
| UART (MAG probe) | 115200 text | observed |
| Motor GPIOs | 12/11/14/13, **INPUT ONLY** | by design |

Unverified: board revision, QMC solder joints, pull-up values, 3.3 V rail waveform
under simultaneous MPU+BMP+QMC+UART load, breakout revision. See
`docs/missing-information-and-problems.md`.

---

## 4. STATUS CHANGE — the integrated magnetometer now works
Revision 1 of this database recorded "MAG integrated = 0 fresh". **That is now
outdated.** The integrated calibration firmware was subsequently measured as passing.

### 4.1 Passing integrated preflights (v3 binary, 921600 baud)
| Capture | Frames | Packet Hz | MAG fresh | BMP fresh | CRC / seq errors |
|---|---|---|---|---|---|
| `captures/usb_preflight_20261103_223853.csv` | 3,008 | 471.3 | **96.2 Hz** | ~10 Hz | 0 / 0 |
| `captures/usb_preflight_20261103_224928.csv` | 1,887 | 471.3 | **96.0 Hz** | 40 samples | 0 / 0 |

Verified directly from the CSVs by this session: in both files
`mag_nonzero_xyz` equals the row count, the `0x04` (MAG valid) and `0x10` (MAG fresh)
flag bits appear, and decoded XYZ is plausible (e.g. `-1223, -1715, -2444`).
Latest flags histogram for the 4 s run: `71,79,87,95,103,111,119,127` — i.e. MPU
valid + BMP valid + MAG valid/fresh + BMP fresh + late bits all present.

### 4.2 Earlier failing preflight (kept for contrast)
`captures/usb_preflight_20261103_223347.csv` — 5,004 frames, 490.2 Hz, flags only
`65/67/75/99`, `mx=my=mz=0` for every row, MAG-valid bit never set, BMP fine.

### 4.3 What actually changed between fail and pass (partially uncertain)
`VALIDATION_LOG.md` (§ "2026-11-03 follow-up hardware diagnosis") records:
1. A temporary 500 Hz MAG-status polling build emitted no valid packets and was **reverted**.
2. The normal combined image was restored and the preflight then passed (96.2 Hz).
3. The launcher was switched to `launch_calibration_gui.py` because Python 3.15's
   Tcl/Tk init was incomplete (Tcl/Tk script libraries are now supplied from the
   installed Python archives in the OS temp directory).
The log does **not** isolate whether the pass came from the revert, from the power
cycle/full reset, or from the restored image. **Treat the exact cause as unresolved**
until a controlled A/B is recorded. Do not claim "STOP+START fixed it".

---

## 5. ODR / maximum-rate findings (the current frontier)
QMC5883P datasheet Rev. E: normal ODR up to 200 Hz, continuous mode up to 1500 Hz,
I2C standard/fast (100/400 kHz).

| Profile | CTRL1 | I2C | Result | Evidence |
|---|---|---|---|---|
| 100 Hz normal | `0xC9` | 100 kHz | **works** — 89.7–90.6 Hz sustained | `..._100hz_20261103_223107.log/.json` (615 samples, 90.63 Hz), `..._100hz_20261103_232852` (266 samples, 89.74 Hz, then degraded) |
| 200 Hz high-OSR | `0xCD` | 100 kHz | **fails** — boot sees ID `0x80`, then status/readback NACKs, DRDY stays clear; one brief 192.56 Hz burst then stale | `..._200hz_20261103_232717`, `233131`, `232926`, `234347` |
| **200 Hz low-OSR** | **`0x3D`** | 100 kHz | **WORKS — 188.84 Hz sustained** | `..._200hz_20261103_234534.json/.log` |
| Continuous | `0xC3` | 400 kHz | **fails** — QMC not detected on scan; bus visibility changed (saw `0x3C`) | `..._continuous_20261103_234811` (ROM/waiting), `..._continuous-fast_..._234703` |
| Continuous low-OSR | `0x33` | 400 kHz | not flashed (bus already failing) | — |

**The headline result** — `captures/mag_probe_v13_200hz_20261103_234534.json`:
```
profile 200, CTRL1=0x3D, CTRL2=0x08, I2C=100000, poll_us=2000, stop_start=1
measured_fresh_hz = 188.84   fresh = 14730   not_ready = 21888
overflow = 0   status_fail = 5   xyz_fail = 0   verify_fail = 0
identical = 41   regs_ok = 1   writes_ack = 1
last_status = 0x18   xyz = -1346,-1653,-2497   age_sec = 0.001
csv_samples = 1424   (csv_every=4 → decimated; HEALTH rate is the true rate)
verify: id=0x80 C1=0x3D C2=0x08 this_read=1 confirmed_ever_and_not_contradicted=1  (x6)
errors: none
```
Backed by `..._234444.json`: 186.82 Hz, 1,864 fresh, 0 overflow, `age_sec=0.000`.

**Interpretation:** the high-OSR (`0xCD`) profile's greater internal sensor load
appears to be what breaks the 100 kHz bus; reducing OSR (`0x3D`) restores a
sustained ~189 Hz. This is a *consistent* explanation with the datasheet current
table (280 µA vs 1180 µA typical at 200 Hz for OSR1=1/8), but it has **not** been
electrically verified (no supply/rise-time measurement).

**Reporting rule introduced by this work:** a rate is only accepted when the *final*
HEALTH report shows matching profile, confirmed register readback, `age_sec < 1.5`,
and sufficient rate — a short burst can no longer be called a pass.

---

## 6. Root-cause insight worth keeping
Two earlier "failures" were **wrong-firmware artifacts**, not sensor faults:
- Before reflashing, COM3 emitted **zero bytes at 921600** but `MAG ACK=...` text at
  **115200** with `C1=0x00, C2=0x00, status 0x18`, XYZ `0,0,0`. The board was running
  a different (text diagnostic) image. Therefore
  `captures/mag_probe_v13_100hz_20261103_222554.log` is a **wrong-firmware capture**,
  not proof that v13 failed.
- Several probe attempts (e.g. `..._231122`, `..._233402`, `..._214259`) captured only
  `ESP-ROM ... DOWNLOAD(USB/UART0) waiting for download` — the flashed image **never
  booted**. Those are invalid/no-boot trials; they must never be read as sensor data.
**Rule:** always confirm the expected banner / `BOOT_PROFILE` / `CSV,` lines before
interpreting any capture.

---

## 7. Instructions for a model joining this work
1. Read this file, then `QCopter/AGENTS.md`, then
   `hardware/bench_calibration/esp32s3_qmc5883p_100hz/VALIDATION_LOG.md` (it is the
   live lab notebook — read the **last** section for current state).
2. `git log --oneline -10` and `git status` first; another model may have just committed.
   Do not revert, rebase or force-push another model's work.
3. Check timestamps of `src/main.cpp`, `src/mag_probe.cpp`, `platformio.ini`,
   `tools/*.py` and the newest `captures/` entry before assuming state.
4. Do not overwrite or delete captures. New evidence gets a new timestamped file.
5. One variable at a time (per `docs/magnetometer-triage-2026-11-03.md`): do not
   change ODR, I2C clock, transaction style and UART settings together.
6. Cold power-cycle (true 10 s) before blaming firmware; a warm/ROM-reset boot is
   not evidence.
7. Never claim a physical/electrical cause without a measurement.
8. Append findings to section 12 (or `VALIDATION_LOG.md`) — do not rewrite history.

---

## 8. Open issues
1. **Why 400 kHz fails** — QMC disappears from the bus entirely. Suspect pull-ups,
   wiring capacitance or breakout revision. No rise-time/scope measurement exists.
2. **Why high-OSR 200 Hz (`0xCD`) fails while low-OSR (`0x3D`) works** — consistent
   with sensor current draw, unverified electrically.
3. **Intermittent degradation even at 100 Hz** — `..._232852` ran 89.74 Hz for ~3 s
   then status reads began failing. Cause unisolated (power/bus/thermal/state).
4. **Integrated vs probe discrepancy is now closed for 100 Hz, open for 200 Hz** —
   the integrated firmware still runs the 100 Hz profile; adopting `0x3D` there is
   an open task.
5. **Exact cause of the 22:33→22:39 fail→pass transition** — see §4.3.
6. **`main.cpp` edited 22:47:42 by a concurrent process** — later commits exist
   (`0a86a84` … `e90a609`); re-read before relying on any earlier line description.
7. **Unvalidated hardware** — board revision, solder, pull-ups, rail quality.
8. **Calibration has still never been run** — `calibration-gui` was deliberately not
   used until the integrated preflight passed, which it now has. A guided 12-stage
   calibration run is the next meaningful milestone.
9. **Flight-side gaps** (unchanged, from `docs/missing-information-and-problems.md`):
   no physical sensor/radio/ESC pipeline on RP2350, PCB draft unfinished, gains are
   simulation candidates on an assumed 1.2 kg plant, no VIO/metric localization.

---

## 9. Concrete numbers (quick reference)
- Integrated pass: **471.3 Hz** packets, **MAG fresh 96.2 / 96.0 Hz**, BMP ~10 Hz, 0 CRC, 0 seq loss.
- Integrated fail (older): 490.2 Hz, 5,004 frames, `mx=my=mz=0`.
- 100 Hz probe: 615 samples @ **90.63 Hz**; 266 samples @ **89.74 Hz**.
- 200 Hz low-OSR probe: 14,730 fresh @ **188.84 Hz** (peak = final = 188.84), 0 overflow.
- 200 Hz high-OSR: brief **192.56 Hz** burst then stale (`age_sec` 26.8 s).
- MAG probe CSV is decimated (`csv_every=4`); use HEALTH rate for rate claims.
- Historical standalone: 1,683 fresh @ 89.64 Hz; historical full capture: 11,000
  accepted samples from 112,377 rows, fitted centre `[-36.30,-22.98,-27.83]` µT,
  radius 45.46 µT, in-sample residual 1.15% — **in-sample only**.

---

## 10. Repository state and sibling material
- Concurrent model's commits (2026-11-03 evening): `0a86a84` (23:08) high-rate probe
  profiles, `3c756e6` (23:14) align probe reads with stable I2C protocol, `639dce6`
  (23:36) diagnose intermittent 200 Hz, `94ae98e` (23:38) low-current 200 Hz profile,
  `e543f11` (23:50) confirm low-OSR 200 Hz + continuous test, `e90a609` (23:51)
  record repeated-start 200 Hz trial.
- Untracked at rev-2 time: `docs/thought-database-2026-11-03.md`, `tmp/`, several
  `artifacts/simulation/2026-10-20-engines/my-first-test-gazebo-*` outputs, and two
  `captures/iq_500hz_*.skipped.json`. One modified file:
  `artifacts/simulation/2026-10-20-engines/gazebo-ros2-transport.json`.
- Sibling workspaces present: `IntelQCopter-USTHB`, `QCopter-ESP32-Xbox`,
  `QCopter-Stepwise`, `Friend-ESP32-FC`, `Drone` (+`Drone.7z`), `ClaudCode`,
  `PICO_SCICALC`, `PyTorch-YOLO`, `TensorFlow`, `TMP`.
- Downloads material: `GPT_TMP/` (~39 dirs, 46 zips, ~1.86 GB of calibration
  handoffs), `MAGNETOMETER_CAPTURE_REVIEW.md`, `IntelQCopter_Calibration_Review.m`,
  `PCB_PCB_drone_2026-10-28(1).json`, `message.txt`, `message(1).txt`, handbooks
  and `REFERENCE_OLD_MAG_QUALITY.png`. Unrelated: FPGA coursework, invoices,
  unrelated videos.

---

## 11. Collaboration protocol with GPT 6 Luna
- **Shared memory = this file + `VALIDATION_LOG.md`.** Long-lived conclusions go
  here; raw run-by-run evidence goes in `VALIDATION_LOG.md` and `captures/`.
- **Do not both drive COM3.** Claim the board explicitly in a message before a run,
  and end the claim when done. Only one owner at a time.
- **Append, don't overwrite.** Add a dated rev at the bottom (section 12) instead of
  editing another model's records.
- **Attribute work.** If you change firmware/tools, commit with a descriptive message
  so the other model can see what moved.
- **Escalate to the user** for: motor/ESC output, arming, anything that could spin a
  propeller, or a decision that changes the safety policy.
- **Report honestly.** Distinguish measured, inferred and untested. Never present a
  compile or an upload as a sensor result.

---

## 12. Revision log
- **rev 1 — 2026-11-03 23:08.** Initial database: project identity, safety rules,
  hardware identity, evidence index, open issues, model-change log. Recorded
  "integrated MAG = 0 fresh" and "MAG-only probe works at ~91 Hz".
- **rev 2 — 2026-11-03 23:55.** Corrected §4: the **integrated firmware now passes**
  (MAG fresh 96.2 / 96.0 Hz, 0 CRC/seq loss). Added §5 ODR table incl. the
  **200 Hz low-OSR (`0x3D`) sustained 188.84 Hz** result and the 400 kHz failure.
  Added §6 wrong-firmware/no-boot artifact rule. Added §11 collaboration protocol
  with GPT 6 Luna. Rebased open issues on the current state.
- **rev 3 — 2026-11-04 01:10.** Added §13 Downloads evidence, §14 sibling/repo
  structure, §15 current integrated firmware state, updated §8 open issues with
  new findings (PCB = HMC5883L design not QMC5883P; SDA/SCL conflict unresolved;
  coordination sqlite3 registered). Registered thought database in
  `QCopter/.git/intelqcopter-coordination.sqlite3` (scope `docs/thought-database-2026-11-03.md`,
  owner deepseek-v4.1-flash). *Next model: append rev 4 below; do not edit rev 1-3 in place.*

---

## 13. Downloads evidence (received after rev 2, subagent aceb3867)
Read-only pass on `C:\Users\Mohamed\Downloads` — nothing created/modified, no COM port opened.

### 13.1 message.txt / message(1).txt are firmware, NOT user narrative
The task premise that these are prose was wrong. Both are ESP32-S3 Arduino firmware source:
- `message.txt` (13,849 B, 10-28): MPU6050 + ESP32Servo + Preferences + Xbox controller; **SDA=9/SCL=8**; motors GPIO 1/12/14/2; 2000 µs loop (500 Hz); armed idle 1120 µs; 1000/2000 µs ESC range; NVS namespace `"fc"`; A=arm, X=disarm, Y=kill-latch. **No magnetometer code.**
- `GPT_TMP\message(1).txt` (11,640 B, 11-01): Bluepad32 + Adafruit MPU6050 + BMP280; **SDA=8/SCL=9**; motors {14,12,2,4}; ESC 1060–1860 µs; 400 Hz; complementary filter; `kAngleKp 268`, `kRateKp 192`, `kYawKp 150`. **No magnetometer.**

### 13.2 params.txt — F450 X-config parameters (10-24 15:14)
- **Total mass:** mean 1038.0000 g, std 16.0312 g, variance 257.0 g²; scalar **1.038 kg** — matches USTHB report exactly
- **Inertia:** I_XX = I_YY = **0.006421**, I_ZZ = **0.011853** kg·m² — matches USTHB report exactly
- **Components:** frame center 272 g; battery 3S 4200 mAh 320 g; avionics 33 g ("ESP + MPU6050 + Protoboard"); wiring 45 g; per-leg ESC 40 g; per-leg motor **A2212** 52 g
- **Geometry:** ARM_COUNT 4; ARM_ANGLE_RAD 0.785398; R_arm 0.225 m
- **Kalman:** 12-state; no magnetometer / yaw-sensor term
- **No hard-coded calibration centres/radii/residuals** — all computed from input CSVs

### 13.3 PCB_PCB_drone_2026-10-28(1).json — EasyEDA board (10-28 21:48)
- **2-layer copper board:** all 57 routing tracks on BottomLayer (layer 2); zero on TopLayer
- **Board size:** outline tracks form 279.5 × 379.1 units; unit scale 10 units = 2.54 mm → **≈71.0 mm × 96.3 mm** (BBox claims 102.9 × 121.9 mm but is inconsistent — treat outline as authoritative)
- **Components (12 placed):** ESP32S3DevKit1 (44 pads), MPU6050 (8 pads), BMP280 (7 pads), **COMPASS-HMC5883L (5 pads)**, 4× 3PIN headers, CN_4PIN_HEADERS ×2, 2× 2-pin headers (11.1V/GND battery inputs)
- **Netlist:** one shared I2C bus (SDA/SCL/VCC/GND); MPU auxiliary bus XDA/XCL (+AD0, INT, DRDY); 5 motor channels M1–M5; UART0 TX/RX; battery 11.1V/GND
- **⚠ Critical discrepancy:** **zero occurrences of "QMC"**; the compass footprint is **HMC5883L**, not QMC5883P. The whole QMC5883P campaign therefore concerns a device *not placed on this board revision* — likely an external breakout module on the same bus, but **not proven by the JSON alone.** No RP2040/RP2350 anywhere; `IntelQCopter` does not appear in the JSON.
- **BBox layer / copper-area layer:** unverified (minor)

### 13.4 MATLAB calibration lab — `IntelQCopter_Calibration_Review.m` (11-02 22:13)
- R2021b+, **no toolboxes** (hand-written `localQuantile`)
- Six-face accelerometer fit, barometer, per-file timing, independent ellipsoid fit vs Python SciPy
- Guard: *"From the raw magnetic vectors alone no accurate absolute yaw can be certified"*
- Actual measured values from `REFERENCE_OLD_MAG_QUALITY.json` (input `iq_500hz_20261102_154514.csv`, QMC5883P):
  - 11,000 fresh rows; YAW/PITCH/ROLL 2000 each; TUMBLE 5000
  - **Hard-iron offset (µT): [-36.316287, -22.987787, -27.821833]**
  - **Soft-iron matrix:** condition number **1.864826**
  - Held-out radial RMS **1.3061%**; training 1.1311%
  - Coverage 87.5% (128 directions, 18°); bootstrap 12/12

### 13.5 Images
- `REFERENCE_OLD_MAG_QUALITY.png` — confirmed magnetometer-calibration figure; off-origin shell corrected to centred sphere near ±40 µT
- `Screenshot (19)/(21).png` — **NOT drone-related** (Xilinx ISE 14.7 ISim 4-bit adder testbenches, French console)
- `Screenshot (22)/(23).png` — not read; same batch, inferred FPGA lab (uncertain)

### 13.6 Tooling
- **PlatformIO Core is NOT in Downloads** but `.pio/build` object files exist under `Tmp\drone_code` and `GPT_TMP` → PlatformIO + ESP32-S3 toolchain exists elsewhere on the machine (likely `%USERPROFILE%\.platformio`), unverified
- MATLAB R2024a installer (.7z, 12.66 GB) present; whether MATLAB is actually installed: unverified
- `analysis\README.md` (194,809 B, inside `IntelQCopter_USB_100Hz_Complete_Calibration`) — **richest unread artifact**, likely consolidated results record

### 13.7 Open items from Downloads
1. message.txt/ message(1).txt are firmware — premise corrected
2. **PCB is HMC5883L design, not QMC5883P** — reconcile: is QMC an external module, on which bus/pins?
3. **SDA/SCL conflict unresolved:** WIRING_DIAGRAM.md says SDA=8/SCL=9; message.txt says SDA=9/SCL=8; message(1).txt says SDA=8/SCL=9. Treated as primary hypothesis by troubleshoot doc.
4. Screenshot (22)/(23) not read; page counts not extracted; Copper_Area layer not verified

---

## 14. Sibling folder / repo structure (received after rev 2, subagent 89fb8cff)
Read-only pass; `7z l` used only for archive listing; nothing created/modified.

### 14.1 Critical structural finding
`IntelQCopter-USTHB`, `QCopter-ESP32-Xbox`, `QCopter-Stepwise` are **NOT separate projects**. Each `.git` is a pointer file:
```
IntelQCopter-USTHB\.git      → gitdir: C:/Users/Mohamed/Desktop/Work/QCopter/.git/worktrees/IntelQCopter-USTHB
QCopter-ESP32-Xbox\.git      → gitdir: .../QCopter/.git/worktrees/esp32-xbox
QCopter-Stepwise\.git        → gitdir: .../QCopter/.git/worktrees/QCopter-Stepwise
```
They are **linked git worktrees of ONE repository**: one object DB, one remote `https://github.com/BenMohStem/IntelQCopter.git`, **209 commits total**, branches `main`, `esp32-xbox`, `codex/stepwise-esp32`, `codex/usthb-university-report`, `codex/motor-limits-system-tools`, `codex/qmc-200hz-continuous-diagnosis`.
- `QCopter/.git` also holds `intelqcopter-coordination.sqlite3` (multi-agent coordination table `claims` with scope/task/owner/token/created/review_after)
- **Deleting a worktree folder loses that branch's uncommitted work** (branch ref survives). History rewrite/gc in `QCopter/.git` affects all three.

### 14.2 Per-folder summary
| Folder | Branch | HEAD | Commits | Status | Purpose |
|---|---|---|---|---|---|
| QCopter (main) | main | various | 209 total | — | RP2350 aircraft + RP2040 handheld + LoRa |
| IntelQCopter-USTHB | codex/usthb-university-report | 7e77c87 (10-31) | 159 | clean | Publishable university report, no firmware |
| QCopter-ESP32-Xbox | esp32-xbox | af29870 (10-28) | 57 | dirty (1) | ESP32-S3 + Xbox FC, ROS2 companion, PCB |
| QCopter-Stepwise | codex/stepwise-esp32 | fb6103f (10-31) | 177 | dirty (35) | Sensor-first lessons, frozen calibration |
| Friend-ESP32-FC | none | — | — | — | Friend rewrite, publishing-excluded, isolated |
| Drone | master | 7bd54c1 (10-17) | 1 | dirty (8) | Original ESP32-S3 FC prototype (ancestor) |
| TMP/Drone | main | e2c9ea2 (10-28) | — | dirty (5) | MPU-only motor-policy experiments |

### 14.3 Safety-relevant finding
`TMP/Drone/main.cpp` is the **only sibling containing physical ESC/PWM output code** (10/45/65% caps, deliberate A-press arming, disarm on stale input >250 ms). **Nothing was flashed**; motor order/direction/ESC endpoints unvalidated. All other FC trees keep outputs disabled.

### 14.4 USTHB report numbers (labelled "Supplied simulation inputs", NOT aircraft measurements)
- Mass 1.038 kg (σ 0.0160312 kg); Jxx=Jyy=0.006421; Jzz=0.011853 kg·m² — **identical to params.txt**
- Hover thrust 10.179 N total, 2.545 N per rotor; arm length 225 mm
- Rate-loop gains Kp/Ki/Kd = 0.12/0.2/0.0008 (illustrative, not flight tuning)
- Accelerometer calibration: bias [1.30868341, −0.06766973, −0.65718879] m/s²; reserved-pose errors direction 0.0668°–1.1093°
- Attitude estimator: filter output std 0.007000°/0.005692° vs accel-angle std 0.177569°/0.213074°; 54 offline checks passed
- Real 1 kHz timing: trials 4–7 passed (max 812–822 µs, 0 overruns); trials 1 and 3 failed
- **Do not quote these as aircraft measurements or flight gains.**

### 14.5 Relationship map
- QCopter = single repository; USTHB/Xbox/Stepwise = worktrees on branches of it
- Drone = standalone original ESP32-S3 FC prototype (17 Oct), explicitly frozen
- Drone.7z = full backup of Drone (96.8 MB uncompressed, includes .pio and .git), newest entry 23 Oct
- TMP/Drone = separate later continuation of motor-policy experiments; different repo/history/source from top-level Drone
- Friend-ESP32-FC = external rewrite, publishing-excluded, never imported
- PyTorch-YOLO / TensorFlow = ML environments supporting perception; TensorFlow is legacy CUDA isolation
- ClaudCode / PICO_SCICALC = not drone work

---

## 15. Current integrated firmware state (as of 00:53, 2026-11-04)
The other model (GPT 6 Luna / deepseek-v4.1-flash lineage) is actively working COM3.

### 15.1 Git state
- On branch `codex/qmc-200hz-continuous-diagnosis`
- Latest commit `06369bc` (00:53:03): "Route fresh MAG samples through GUI plots"
- Previous commits: `e90a609` record repeated-start 200 Hz trial; `e543f11` confirm low-OSR 200 Hz + continuous test; `94ae98e` add low-current 200 Hz profile; `639dce6` diagnose intermittent QMC 200 Hz; `3c756e6` align QMC probe reads with stable I2C protocol; `0a86a84` add QMC high-rate probe profiles and record bench trials; `c2c825f` diagnose calibration UART and sensor stream failures
- Modified: `artifacts/simulation/2026-10-20-engines/gazebo-ros2-transport.json`
- Untracked: `artifacts/simulation/2026-10-20-engines/my-first-test-gazebo-*` outputs

### 15.2 Integrated firmware (passing)
- Profile: QMC **CTRL1=0x3D** (normal 200 Hz, OSR1=1, OSR2=1), **I2C raised to 400 kHz**, 921600 baud v3 binary, Wi-Fi off
- PlatformIO 7.1.3 / Arduino, ESP32-S3 DevKitC-1 N8 on CH343 COM3
- **8 s UART preflight:** 4,001 v3 packets at 500.0 packets/s; **4,001 MAG-valid packets; 1,002 fresh MAG samples (166.8 fresh/s)**; 79 fresh BMP packets; **0 CRC errors, 0 lost sequence numbers**
- Capture: `captures/usb_preflight_20261104_003805.csv`
- Motor outputs remain disabled; ESC GPIOs INPUT only
- Measured full-system fresh rate (166.8 Hz) is lower than register ODR (200 Hz); causation of the margin not established
- I2C pull-up values, bus rise time, actual sensor rail voltage: **unmeasured**

### 15.3 GUI MAG plot path (just committed)
- GUI now sends latest fresh magnetic vector with regular bounded plot snapshot (not one event per sample)
- Health line reports MAG valid / fresh / plotted counts
- Python syntax/self-check passed; in-memory v3 QMC packet check passed through decoder → Tk plot snapshot → 3D cloud append
- **Actual live window NOT visually confirmed** — do not claim rendered live MAG until plotted counter rises and cloud is visible

### 15.4 Outstanding after this work
- Continuous mode (CTRL1=0xC3) never produced fresh data; 400 kHz works for integrated but probe-only 400 kHz attempts saw QMC disappear from bus
- Post-start sensor state / bus-access problem not attributed to a specific electrical cause
- Board revision, QMC solder, pull-ups, rail quality: unvalidated
- Calibration GUI not yet run through the 12 stages with live MAG

---

## 16. Open issues — consolidated (updates to §8)
Critical:
1. **MAG now works in integrated firmware** at 166.8 fresh/s (200 Hz low-OSR, 400 kHz I2C). Next meaningful milestone: run calibration-gui through 12 stages with live MAG; propellers off, ESC isolated.
2. **PCB is HMC5883L design, not QMC5883P.** Reconcile before further calibration: is QMC5883P an external breakout module, on which bus/pins? PCB JSON alone cannot prove bus connection.
3. **SDA/SCL conflict unresolved:** WIRING_DIAGRAM.md says SDA=8/SCL=9; message.txt says SDA=9/SCL=8; message(1).txt says SDA=8/SCL=9; troubleshoot doc treats reversal as primary hypothesis. Directly relevant to QMC/MAG reliability.
4. **400 kHz works in integrated image but probe-only 400 kHz attempts made QMC invisible.** Causation unisolated (pull-ups, wiring capacitance, breakout revision, supply waveform).
5. **COM3 is held by the concurrent model.** Do not flash/capture until user confirms board free and true 10-s all-power removal.
6. **No physical motor test or flash has occurred** in this session; all other FC trees keep outputs disabled; TMP/Drone has the only physical PWM code and nothing was flashed there either.

High:
7. **Exact cause of fail→pass transition (22:33 → 22:39, then 400 kHz raise)** unresolved — treat as supported-but-not-repeatedly-confirmed.
8. **Continuous mode (0xC3) never sustained fresh data.** No ≥500 Hz result claimed; user's target remains unverified.
9. **Coordination database** `QCopter/.git/intelqcopter-coordination.sqlite3` has table `claims`; thought database registered as scope `docs/thought-database-2026-11-03.md`.

---

## 17. Collaboration protocol with GPT 6 Luna (reiterated)
- **Shared memory = this file + `VALIDATION_LOG.md`.** Long-lived conclusions here; raw run-by-run evidence in `VALIDATION_LOG.md` and `captures/`.
- **Do not both drive COM3.** Claim the board explicitly in a message before a run; end claim when done. Only one owner at a time.
- **Append, don't overwrite.** Add a dated rev at the bottom instead of editing another model's records.
- **Attribute work.** Commit with descriptive messages so the other model can see what moved.
- **Escalate to the user** for motor/ESC output, arming, anything that could spin a propeller, or any safety-policy decision.
- **Report honestly.** Distinguish measured / inferred / untested. Never present a compile or upload as a sensor result.

---

## 18. Revision log (final)
- **rev 1 — 2026-11-03 23:08.** Initial database.
- **rev 2 — 2026-11-03 23:55.** Integrated firmware passes; ODR table; wrong-firmware rule; collaboration protocol.
- **rev 3 — 2026-11-04 01:10.** Downloads evidence (§13), sibling/repo structure (§14), current integrated firmware state (§15), consolidated open issues (§16). Registered in coordination sqlite3. *End of rev 3 — next model appends rev 4.*

## 19. Revision 4 — 2026-11-05: DSH coordination, cloud GPT v2, and cleanup plan

### Current coordination state
- The user says DSH is actively working on project cleanup/source changes. At
  last read-only inspection on this branch, modified files were
  `firmware/flight_controller/flight_controller.c` and
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/src/main.cpp`; untracked
  files were `firmware/flight_controller/flight_controller.c.tmp` and
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz.zip` (59,451,892 bytes).
- Latest observed HEAD and origin were both `7833cf8` (“Add QMC5883P/MPU6050
  sensor calibration + PID flight control config”). Do not overwrite or
  reformat the two modified sources while DSH is active. Inspect its finished
  diff and status before taking over. Do not delete the `.tmp` or ZIP without
  determining whether either is DSH's source/evidence.
- A top-level plan was created at `C:\Users\Mohamed\Desktop\Work\PLAN.md`.
  It defines inventory → preserve/archive → formatting-only readability pass →
  calibration analysis → Xbox/FC review → scoped commits/push. Share this plan
  with DSH before broad renames, deletes, or moves.
- Drone is disconnected for PCB repair. No COM3 sessions, flashing, or physical
  validation should be attempted until the user confirms repair and connection.

### Review of cloud GPT artifact (read-only)
- User supplied `Downloads/IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip`
  (1,958,455 bytes; 44 files). Its manifest says source + host tests only; no
  actual ESP32 PlatformIO build or physical validation. It is a separate bench
  ESP32-S3 candidate, not the RP2350 flight runtime.
- Safety lock is explicit: `IQ_MOTOR_OUTPUTS_ENABLE=0`, GPIOs are inputs, and
  there is no physical arming path. Xbox is a Windows PC pygame → CH343 UART
  bridge (~50 Hz), not direct onboard pairing or a flight receiver. Its D-pad
  chord is barometer re-zero, not arm/disarm. Keep these distinctions.
- `data/20261104_raw_calibration_candidate_UNVERIFIED.json` is marked
  provisional and automatic flight application is false. It reports gyro bias
  `[0.161969,-0.304950,-0.877433] dps`; accel bias `[1.264656,-0.059103,-0.678745]
  m/s²` and a full 3x3 matrix; MAG hard-iron `[-29.482301,-18.397984,-35.474273]
  µT` plus det-1 relative soft-iron matrix. It derives from a single November 4
  capture, with MAG holdout magnitude RMS 3.464%, and explicitly requires axis
  and powered-motor checks. Do not use as final FC calibration.
- ZIP's seven PID values and Monte Carlo standard deviations are candidate
  simulation values based on an assumption-based F450 inertia/thrust model, not
  measured flight gains. No thrust curve, motor response, axis convention, or
  physical failsafe is established. Do not convert these into actuator commands.
- The ZIP's own notes say actual firmware build and hardware remain unverified.
  Its copied sensor code records 400 kHz with 100 kHz fallback; this conflicts
  with the latest integrated 400 kHz MAG-zero capture and 100 kHz isolated probe
  success. Resolve only with a repaired-board combined capture.

### DSH diff review guidance (not a verdict on its uncommitted work)
- Last seen DSH diff introduces MPU ±500 dps / ±8 g ranges, FIFO configuration,
  and gyro offset constants while editing the bench calibration `main.cpp`.
- Before accepting it, check that host engineering-unit conversion and all
  calibration calculations use the matching sensitivities (65.5 LSB/dps and
  4096 LSB/g), verify FIFO frame byte layout and reset/overflow behavior, and
  ensure the 52-byte protocol remains compatible with the GUI.
- Keep hardware offset-register writes disabled until the factor is confirmed;
  DSH's new macro defaults off, which is the safe choice. Confirm comments do not
  claim unmeasured performance.
- Current working diff visibly contains UTF-8 mojibake (`â€”`, BOM) and comments
  duplicating old and new register settings. Clean encoding/comment duplication
  only after DSH says the edit is ready; preserve behavior and isolate formatting
  changes from register/algorithm changes.

### Calibration dataset admission rule
- Analyze only a session with explicit completion evidence showing 100% for all
  required phases. A large CSV, a passing UART preflight, a 100% sensor coverage
  score, or an output JSON alone does not prove the guided calibration session
  completed. Preserve partial and skipped sessions, but exclude them from fits.
- `captures/calibration_package_20261104/iq_500hz_20261104_100907.csv` exists;
  its completion metadata still needs verification before it is admitted.
- After admission: calculate and plot gyro bias/noise, six-face accel offset
  and full 3x3 correction, MAG hard/soft-iron ellipsoid fit with held-out error,
  coverage, covariance/std deviations, and barometer reference. State units,
  frames, fit limitations, and keep all results provisional until board/axis
  applicability is verified.

### Project cleanup execution order
- Use `C:\Users\Mohamed\Desktop\Work\PLAN.md` as the canonical project plan.
- First read-only inventory and classify files; then record keep/archive/delete
  manifest. Do not delete merely “old” evidence or source snapshots. Archives
  retain provenance and original bytes. Keep generated SDK/build/cache and
  personal files out of version control.
- Consolidate multiple thought bases into one indexed `docs/thoughts/` source
  only after DSH has stopped editing them; preserve dated revisions rather than
  overwriting conclusions. The current file is still the active handoff.
- User requests readable code without behavior changes. Perform formatting-only
  edits after DSH's source work is complete, with a diff check proving only
  whitespace/comments changed.

## 20. Revision 5 — 2026-11-05: cleanup handoff and complete calibration evidence

### Canonical knowledge base and project plan
- `docs/thoughts/thought-database.md` is now the single canonical thought base.
  `docs/thoughts/coordination.md` is its short index/entry point.
- The former root database is retained unchanged at
  `archive/thought-bases/2026-11-05/thought-database-2026-11-03-original.md`;
  its old path is a compatibility pointer. Append new revisions only to the
  canonical file. `docs/thoughts/README.md` explains this layout.
- Canonical plan is `docs/project-plan.md`; a convenience copy is also at
  `C:\Users\Mohamed\Desktop\Work\PLAN.md` per user request. Update both at
  milestone boundaries.
- Cleanup manifest: `archive/project-cleanup-manifest.md`. Preserve every
  decision and rationale there; do not remove evidence, custom ROS/Gazebo
  sources, RViz/URDF, linked worktrees, or calibration captures.

### Latest source and coordination state
- Latest observed commit before cleanup: `bb843d7`, DSH Xbox/barometer work.
  At cleanup start, only calibration `src/main.cpp` was modified; `docs/thoughts/`
  was untracked. `flight_controller.c.tmp` and the 59 MB calibration ZIP were
  untracked source/package artifacts. Their classification is in the cleanup
  manifest; preserve originals until separately reviewed.
- User says DSH has stopped and grants this session cleanup freedom. Source
  behavior edits are still out of scope for cleanup; a readability-only pass
  must be whitespace/comment-only and must not overlap unreviewed firmware diffs.
- Drone is disconnected for PCB repair. No COM3 access, flash, or physical tests.

### Calibration completion check (offline data only)
- Candidate complete run:
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/calibration_package_20261104/iq_500hz_20261104_100907.csv`.
- Streaming CSV inspection found 98,868 data rows and 12 phase attempts. Accepted
  sample counts met configured targets: REST 30,000; each of six accel faces
  5,000; three axis MAG turns 2,000 each; MAG tumble 8,000; BMP 1,000. The
  `.skipped.json` contains an empty `skipped_stages` list, and matching
  `calibration_20261104_100907.json` exists. Treat this as the only currently
  evidenced full guided session; keep this determination in analysis provenance.
- This completion finding does not imply the parameter fit is flight-ready. The
  generated MAG fit is diagonal (the full ellipsoid was degenerate); axis
  mapping/PCB identity and powered-motor effects remain unvalidated. Use the
  raw CSV to independently recompute statistics and plots, preserve existing
  JSON/PNG as prior outputs, and label results provisional.
- Session's own README claims 400 kHz integrated MAG ~196 Hz, while later
  preflights show zero MAG at 400 kHz and a clean isolated 100 kHz probe passed.
  Treat the session README as authored summary, not raw proof of this session's
  end-to-end magnetic validity; inspect raw timestamps and flags before accepting
  sensor-specific rows in any new calibration calculation.

### Cleanup classification summary
- Keep tracked `tmp/` files: although named “tmp”, they are versioned project
  lessons, utilities, and evidence; not disposable scratch.
- Keep `output/pdf/` deliverables, `build/` scripts and source outputs pending
  dependency review, `.local/`, `build/ros2`, all `artifacts/`, `captures/`,
  `measurements/`, the ESP32 `.zip`, and original source snapshots. Do not bulk
  delete `build/` based on its name; it includes source/evidence alongside
  rebuildable caches.
- Only explicit tool caches such as `build/zig-cache` and
  `build/tectonic-cache` are candidates for removal after confirming no active
  process uses them; record exact removed paths in the manifest. Do not remove
  ROS/Gazebo installed workspaces or SDKs in this pass.
- The `.tmp` controller file is an older, distinct source snapshot (not byte
  identical to current firmware); archive it with its timestamp/hash rather than
  deleting or silently promoting it.

## 21. Revision 6 — 2026-11-05: cleanup execution and source preservation

- Renamed the canonical shared database, coordination handoff, project plan,
  and cleanup manifest to lowercase kebab-case paths. Updated the historical
  thought-base pointer and indexes; the historical root database remains
  preserved byte-for-byte in `archive/thought-bases/`.
- Archived the distinct controller `.tmp` snapshot and the local calibration
  ZIP with hashes recorded in `archive/project-cleanup-manifest.md`. The ZIP
  contains generated PlatformIO outputs and stays excluded from Git.
- Preserved the stopped DSH's uncommitted calibration `src/main.cpp` candidate
  separately, with SHA-256 recorded in the manifest. The candidate includes
  unreviewed sensor configuration changes; the repository working copy was
  restored to the current committed version so this cleanup does not adopt
  firmware behavior changes.
- No caches, source, captures, simulations, or other evidence were deleted.
  `tmp/` and mixed-content `build/` remain classified for a later path-by-path
  review; no physical or device validation was performed.

## 22. Revision 7 — 2026-11-05: Cloud GPT A-to-Z review and raw cross-check

- Preserved the owner-supplied A-to-Z Markdown snapshot under
  `archive/imported-documents/cloud-gpt-a-to-z-2026-11-05/`. Its internal
  SHA-256 manifest verified 90 of 90 listed source files; 91 Markdown files
  totaling 418,911 uncompressed bytes were extracted without content edits.
  The embedded AI handoff text is archived reference material, not active
  instructions. The review is `docs/thoughts/cloud-gpt-a-to-z-review.md`.
- Independently parsed the complete November 4 10:09 capture. It has 98,868
  rows; all twelve phase attempts reached their accepted sample targets and the
  skipped-stage list is empty. MAG `mag_new=1` count is 38,652. Each active phase
  has ~500 Hz packet rows and ~195 Hz fresh MAG; the full-file duration includes
  operator gaps. `mag_ok=1` alone is not a freshness count.
- Recomputed REST gyro means/SDs match the A-to-Z snapshot. Applying its MAG
  candidate correction to the 14,000 fresh MAG calibration rows reproduces
  corrected norm mean 40.7434 µT and SD 1.2504 µT. This checks in-session
  arithmetic, not independent heading truth or cross-session generalization.
- Confirmed the A-to-Z accel correction matrix is the inverse of the saved
  forward scale matrix (product differs from identity by at most about
  `5e-10`), consistent with the original fitter's `measured = S * true + b`
  model. Firmware must use the direct-correction convention once, not apply both.
- The A-to-Z claim of 3.46% error on separate prior sessions is not matched by a
  corresponding multi-session result/input in this checkout. The local MAG
  quality JSON is for a different capture and explicitly says there was only
  one independent session. Keep external-session performance unverified.
- The software capture identifies QMC5883P, but PCB design files indicate an
  HMC5883L footprint. Physical part/routing remains unresolved until PCB repair.
  The A-to-Z Xbox mapping differs from the user's requested layout, and its
  future LiDAR material must not restore the removed calibration GUI stage.
- Static review of the separate Downloads FC v2 archive confirms compile-time
  motor lock and a QMC 400 kHz attempt with 100 kHz fallback in source. No build,
  upload, COM3, or physical test was run; these source settings are not hardware
  validation. The v2 archive remains separate from the active Git source.

## 23. Revision 8 — 2026-11-05: independent audit source found and archived

- Follow-up located `Downloads/GPT_TMP/IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip`.
  Its SHA-256 is `7C55BA598286AA67B2BFA5BDB8E70560E952B2FC521470F40E59D2749BF6C4BC`;
  all 40 entries in its internal SHA manifest verify. The 10:09, 02:57, 03:17,
  and 04:08 raw CSVs inside match the repository files byte-for-byte.
- Imported the audit report, machine-readable results, unverified calibration
  candidate, corrected CSV derivative, model comparison, and seven plots under
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`.
  The 31 MB ZIP, duplicated raw inputs and full scripts remain in
  `Downloads/GPT_TMP`; the scripts were not rerun in this cleanup pass.
- The report provides cross-session MAG magnitude checks (3.794% on the 02:57
  session and 3.456% on 03:17) with matching input identities. This is sourced
  from the audit outputs, not a new execution; all candidate parameters remain
  provisional.
- Clarified the audit's gyro saturation rates: 13.26% across all MAG stage rows,
  versus 21.99% in TUMBLE and 4.83% in ROLL. The latter values are stage-specific
  and do not conflict with the combined rate.

## 24. Revision 9 — 2026-11-05: local build tree classified further

- Inventory of ignored `build/` counted 24,580 files (~1,311.9 MiB). It is not a
  disposable build-output folder: it mixes dependency environments, local ROS
  workspaces, scripts, reports, datasets, CAD trials, and source/evidence snapshots.
  Detailed path classes and largest areas are recorded in
  `docs/build-directory-inventory-2026-11-05.md`.
- Found `build/stepwise-seed/`, a 19-file ESP32-S3 board-diagnostics project
  snapshot with a plan, hardware assumptions, source and validation tools. It has
  no `.git` directory and is not equivalent to a regenerable cache. The separate
  `QCopter-Stepwise` worktree remains intact; do not overwrite its live changes.
- A copy-and-verify relocation of that snapshot to `archive/source-snapshots/`
  was rejected by the command safety policy before execution. No files were
  copied or removed. Leave the snapshot untouched; do not retry by another command.
- Earlier removal attempts for `build/zig-cache/` and `build/tectonic-cache/` were
  also rejected. Do not bypass or retry those removals. The inventory and failed
  operation are recorded in the project plan and cleanup manifest.
- Corrected the README and architecture map so `build/` is described as ignored
  but mixed-use. No build contents, source behavior, or calibration evidence were
  modified. `docs/` had no exact duplicate Markdown, text or JSON files in the
  scoped duplicate scan.

## 25. Revision 10 — 2026-11-05: correct the active lab-log handoff

- The thought-base index pointed at nonexistent `docs/VALIDATION_LOG.md`. The
  actual live ESP32 bench chronology is
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/VALIDATION_LOG.md`.
- Fixed the active AI reading order to link the repository `AGENTS.md` and that
  real validation log. The coordination page now points to the same record before
  calibration or bench changes.
- Checked local links in the root README, docs index, project plan, architecture,
  thought-base index/current handoff and archive index/manifest. All local links
  in that active-navigation scope resolve. No historical thought-base revisions
  or preserved source documents were rewritten.
