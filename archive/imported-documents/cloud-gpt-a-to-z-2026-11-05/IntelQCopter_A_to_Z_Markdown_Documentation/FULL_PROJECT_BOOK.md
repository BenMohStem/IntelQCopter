# IntelQCopter — combined project reference book

**Companion to the separate Markdown chapters in this ZIP.** The chapters take precedence over any conflicting historical snapshot.


---

<!-- BEGIN: 00_START_HERE/00_README_FIRST.md -->

# IntelQCopter — complete project knowledge base

**Snapshot:** 5 November 2026. **Scope:** the entire documented engineering effort through the **ESP32-S3 500 Hz fast calibrated FC v2** source and the successful 4 November sensor-calibration capture. This collection is intended for the project owner, local coding agents, reviewers and future contributors. It is not a flight clearance document.

## Where to begin

1. Read `01_CURRENT_STATUS.md`, `02_EVIDENCE_AND_CONFIDENCE.md` and `03_TIMELINE.md` in this folder.
2. For actual current firmware, start at `../04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md` and `../04_FIRMWARE/05_BUILD_CONFIGURATION.md`.
3. For physical measurements, read `../03_CALIBRATION/01_NOV4_CAPTURE_OVERVIEW.md` and the individual sensor audit documents.
4. For mathematics and the seven-loop controller, read `../05_ESTIMATION/` and `../06_CONTROL/`.
5. For the next experiments, use `../07_TESTING/01_BENCH_TEST_ORDER.md` and `../09_ROADMAP/01_IMMEDIATE_PRIORITIES.md`.

## Nonnegotiable distinctions

- **Observed:** The 4 Nov full calibration capture contains 98,868 records with live MPU6050, QMC5883P and BMP280 data. Its actual QMC control value was **0x3D**, nominal 200 Hz, despite an earlier package being named *100hz*.
- **Implemented in source:** FC v2 contains a 500 Hz nominal scheduler, 12-byte MPU FIFO acquisition, calibrated matrices, boot offset handling, quaternion estimator, 3-state altitude Kalman, seven virtual PID loops, Xbox-to-UART bench interface and safety checks.
- **Not demonstrated on physical hardware:** FC v2 executing its complete fast loop, gains controlling real motors, thrust mapping, ESC arming/failsafe, verified body-frame signs, integrated LiDAR and any successful free flight.
- **Estimated:** Vehicle mass, inertia, motor lag, thrust capacity and Monte Carlo uncertainty depend on assumptions rather than hardware weighing or a thrust stand.

## Quick source map

- `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip` — most recent v2 bench-control source.
- `IntelQCopter_Calibration_20261104.zip` — successful sensor capture, original fitting program and saved outputs.
- `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip` — independently refitted candidate constants and validation.
- `IntelQCopter_100Hz_Original_v13_Driver_Full_Calibration.zip` — transition from isolated working magnetometer to full GUI calibration.
- `IntelQCopter_Volume_I_Study_Edition2.pdf` — user-supplied theoretical study; **some sensor names and assumed plant constants do not describe the measured aircraft**.

## What this knowledge base is not

It does not silently modify firmware or GitHub. It is a readable, searchable set of `.md` documents. Statements about builds, simulation and motor-off tests are not evidence of flight suitability. Preserve your original raw files and commits; make changes on branches and retain logs.


---

<!-- BEGIN: 00_START_HERE/01_CURRENT_STATUS.md -->

# Current project status — 5 Nov 2026

## System-by-system state

| Subsystem | Evidence | Present status |
|---|---|---|
| ESP32-S3 + CH343 USB UART | Uploaded PlatformIO logs and live CSV sessions | Working for calibration captures; occasional COM/reset issues documented |
| MPU6050 at 0x68 | 4 Nov full capture | Real 500 Hz-class data recorded; v2 FIFO path is *proposed but not hardware verified* |
| QMC5883P at 0x2C, chip ID 0x80 | v13 standalone and 4 Nov full capture | Previously intermittent; 4 Nov capture succeeded using **0x3D**, ~195 Hz fresh |
| BMP280 behind MPU XDA/XCL bypass | 4 Nov data | Working during capture; v2 direct compensation path needs on-device test |
| TOFSense F2 Mini | Described hardware, GUI tooling | No accepted 4 Nov LiDAR calibration; physical ESP32 RX pin not verified |
| Six-position accel / stationary gyro / mag rotations | 4 Nov raw archive | Numerically refitted; physical axis and dynamic-interference checks pending |
| 500 Hz FC v2 | Source archive and host tests | Motor-disabled research/bench candidate; hardware timing not yet observed |
| Xbox controller | Windows bridge in v2; GitHub esp32-xbox protocol | Virtual command path written; direct robust ESP32-S3 BLE/RC not verified |
| Motor/ESC thrust curves | Assumption-based studies | Not measured; no verified real motor-control loop |
| Actual flight | No flight logs or successful flight evidence supplied | **Not verified as of this snapshot** |

## Immediate engineering risks

1. Wrong sensor-to-body mapping can invert feedback even if all calibrated vectors look statistically excellent.
2. QMC stability was intermittent in earlier runs, and the successful sensor capture does not prove operation under motor EMI.
3. Hardware-offset-register effects need measured readback and stationary before/after tests; don't subtract a correction twice.
4. Nominal 500 Hz does not guarantee low worst-case latency when I²C transactions can time out.
5. Hover margin is unknown because motor/prop thrust, ESC lag, and battery sag were not measured.
6. Altitude precision is limited by pressure drift, propwash, tilt, vibration and absence of validated ranging.

## Latest artifact

The v2 archive is a **safe bench build** whose `IQ_MOTOR_OUTPUTS_ENABLE=0` compile-time restriction prevents physical ESC output. Do not remove that guard just because the PID arrays or Monte Carlo script return numbers.


---

<!-- BEGIN: 00_START_HERE/02_EVIDENCE_AND_CONFIDENCE.md -->

# Evidence rules, verification levels and claim ledger

Use these terms consistently in issues, PRs, reports and AI prompts.

| Label | Meaning | Example |
|---|---|---|
| **MEASURED** | Observed on the real board with a saved instrument trace | 4 Nov calibration CSV has 98,868 records |
| **REPRODUCED** | Independently recomputed using the raw measurement and another method | 3×3 QMC calibration reproduced by nonlinear and algebraic fits |
| **CONFIGURED** | Value appears in actual program settings | `IQ_MAG_CONTROL1=0x3D` in successful snapshot |
| **IMPLEMENTED** | Algorithm present in source, host tests may pass | FIFO and altitude KF in FC v2 |
| **HOST TESTED** | Unit test or mock compilation without ESP32 hardware | Bosch integer compensation tests |
| **SIMULATED** | Output of a declared idealized plant | Controller overshoot, hover-command statistics |
| **ASSUMED** | Unverified numeric parameter | 6 N/motor max thrust, 40 ms lag, vehicle mass prior |
| **OPEN** | Unverified or contradictory | Motor order, actual axis map, LiDAR UART RX pin |

## A test's chain of evidence

`source commit -> build flags -> toolchain -> board and wiring -> instrument/log -> parser -> metrics -> acceptance criterion`.

Missing steps limit conclusions. An I²C ACK is not a valid sample. A register readback is not a sustained sampling test. An IMU count of 500 Hz is not evidence that all FC computations completed before 2 ms. A valid Monte Carlo output is conditional on its input distributions.

## Warning examples

- *Incorrect:* “QMC hardware is certified at 200 Hz.”
- *Defensible:* “The QMC provided ~195 fresh samples/s during the Nov 4 calibration capture with `0x3D`; earlier startup failures were documented.”
- *Incorrect:* “The drone weighs 1.104 kg.”
- *Defensible:* “The geometry model estimates 1.104 ± 0.051 kg before real weighing.”
- *Incorrect:* “PID v2 is tuned.”
- *Defensible:* “All 21 gains exist in source; candidate rates are derived from assumed inertia and validated in a synthetic plant only.”

Never silently carry an assumption from a study PDF into a flight-control constant without retaining its provenance.


---

<!-- BEGIN: 00_START_HERE/03_TIMELINE.md -->

# Chronological engineering timeline

This is a version map of *parallel subprojects*, not a single monotonically increasing release line.

| When | Development/event | Result and important qualification |
|---|---|---|
| Earlier | RP2350/LoRa and ESP32-S3/Xbox/ROS2/Gazebo proposals | Distinct branches and prototypes; avoid conflating pinouts and runtimes |
| Oct 2026 | PCB work, modeling, preliminary motor limits and control scaffolding | Research/simulation; no flight demonstrated |
| Nov 1 | Engineering study of dynamics, math, filters and control | Several numbers marked synthetic or assumed |
| Nov 2 | 500 Hz full GUI sensor-calibration work | Binary stream, calibration sessions, PC analysis tooling |
| Nov 3 | QMC v12/v13 magnetometer isolation | **v13 100 Hz `0xC9` worked** at ~90 Hz in some logs, 200 Hz failed then |
| Nov 3 | QMC v14-v18 experiments | Slow/fast I²C, Wi-Fi interference, power cycles, COM and transaction failures; no stable universal repair |
| Nov 3 evening | Original v13 driver transplanted to full calibration | Restored a real path to successful integrated captures |
| Nov 4 morning | Complete calibration `100907` recording | 98,868 full data frames; live QMC, IMU, BMP; no LiDAR measurements |
| Nov 4 | Independent numeric calibration audit | Updated gyro/accel/mag constants, holdout tests, warnings about cross-axis confidence |
| Nov 4 | First 500 Hz FC integration v1 | Motor-disabled estimation/control model, initial mathematical tests |
| Nov 4/5 | Fast FC v2 | 500 Hz FIFO, BMP fast path, 400 kHz primary bus, 7 PIDs, UART Xbox bench bridge; no ESC |
| Nov 5 | This Markdown documentation snapshot | Documentation only; no firmware changes or flight test |

## Magnetometer turning points

**v13 100 Hz** and **Nov 4 `0x3D` 200 Hz** are different confirmed successful *sessions*, not a contradiction to erase: silicon configuration, bus mode, firmware scheduling and power state differed. Multiple intermediate combinations failed. Read `../02_SENSORS/02_QMC5883P.md` and `../07_TESTING/04_HISTORIC_FAILURES.md` before rewriting the driver.

## Project deadline context

The project owner previously reported a competition on 5 November 2026 and that the quadcopter had not yet flown. Do not use this deadline to lower arming, receiver-loss, motor-direction or thrust-margin safety requirements.


---

<!-- BEGIN: 00_START_HERE/04_GLOSSARY.md -->

# Glossary — hardware, estimation and controls

- **Accelerometer specific force:** proper acceleration excluding gravity in the inertial frame; at rest an upright calibrated sensor often measures about +1 g along its body-up axis.
- **ACK:** I²C address acknowledged; not proof that correct sensor data can be read.
- **Body FLU:** X forward, Y left, Z up; right-handed coordinate frame used in v2 mathematical assumptions.
- **CH343:** USB-to-serial converter visible as a Windows COM port; physical UART baud is distinct from I²C clock.
- **CoM / CG:** center of mass / gravity, important for mixer geometry and inertia.
- **DRDY:** data-ready bit/status or physical pin. QMC register `0x09` is read over I²C; a separate DRDY wire is not required in this project.
- **DLPF:** low-pass filter inside IMU; bandwidth affects both noise and phase delay.
- **ESC:** electronic speed controller for brushless motor; no physical outputs in safe v2 firmware.
- **FIFO:** hardware buffer of timestamp-adjacent IMU measurements; decouples sampling from MCU servicing but can overflow.
- **FLU / NED / ENU:** coordinate conventions; conversions must use proper det+1 rotations, not arbitrary reflected axis swaps.
- **Fresh:** one *new physical sensor conversion*; not merely a repeated frame or same cached register bytes.
- **Hard iron:** additive magnetic offset, typically due to remanent magnetic material.
- **Soft iron:** linear transformation of magnetic field due to material or sensor anisotropy; corrected using full 3×3 matrix.
- **I²C bypass:** MPU6050 register setting giving host access to sensors wired to XDA/XCL, not an external INT signal.
- **Innovation/NIS:** KF measurement minus prediction / normalized squared surprise used for outlier gating.
- **N150:** optional stripped Samsung companion computer in an assumed second aircraft mass profile; hardware integration uncertain.
- **PI/PID:** proportional-integral (plus derivative) control; a gain value is meaningless without units, sampling and actuator mapping.
- **QMC5883P:** actual magnetometer, at `0x2C` and ID `0x80`, distinct from HMC5883L or QMC5883L.
- **Quaternion:** unit 4-vector used for nonsingular 3D orientation propagation.
- **R² / RMS / SD:** explained variance / root mean squared error / standard deviation; fit residuals are not equivalent to absolute physical accuracy.
- **X-mixer:** translates total thrust and roll/pitch/yaw torques into four actuator requests; motor order and yaw polarity must be verified.


---

<!-- BEGIN: 00_START_HERE/05_REPRODUCIBILITY_AND_PROVENANCE.md -->

# Sources, artifact provenance and reproducibility

## Primary bundles used to author this knowledge base

1. `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip` — current source/readme/config, PID model, estimator, tests, CLI scripts.
2. `IntelQCopter_Calibration_20261104.zip` — user-supplied 4 Nov raw CSV, calibration JSON, plot, script and skipped-stage record.
3. `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip` — independent numerical re-analysis (including prior captures, candidate JSON and generated reports).
4. `IntelQCopter_100Hz_Original_v13_Driver_Full_Calibration.zip` — working MAG driver and complete calibration tooling.
5. `IntelQCopter_Complete_AI_Handoff_2026-11-03.zip` — early history, v13-v18 evidence, mass model and reference sources.
6. `IntelQCopter_Volume_I_Study_Edition2.pdf` — user-supplied mathematical study; theoretical source only for a number of physical assumptions.

## Remote version control

Public reference: <https://github.com/BenMohStem/IntelQCopter>. As checked on 5 Nov, the remote `main` head was `e2fa10862b0c`; `esp32-xbox` head `af29870011ac`; `codex/qmc-200hz-continuous-diagnosis` head `7833cf87adca`. The supplied v2 ZIP is a **separate local artifact** and should not be assumed present in `main` or deployed to the board. Requery these SHAs before relying on them in the future.

## Reproduce data analysis

Keep raw inputs immutable and compute derivatives in a new directory. The independent audit bundle documents `run_independent_audit.py`, `create_outputs.py`, and `validate_outputs.py`. Compare engineering-unit CSV fields against raw integer axes and firmware sensitivities; use held-out rotation sessions to detect overfitting.

## Reproduce source tests

From the actual v2 folder:

```powershell
py -3 tests/test_project.py
py -3 tools/uncertainty_pid_v2.py
py -3 tools/simulate_rate_v2.py
```

Tests for a simulated/mocked C++ environment cannot establish ESP32 electrical timing or safe motor actuation. Do not modify the calibration JSON in place without recording its source and re-running diagnostics.


---

<!-- BEGIN: 01_HARDWARE/01_AIRFRAME_GEOMETRY.md -->

# Airframe geometry and coordinate conventions

The flight project is described as an F450-class X quadcopter with two red and two white arms. “450” usually designates nominal motor-to-motor diagonal dimension; the actual span, motor positions, arm height and board mounts have not been measured for these files. Some older descriptions called the vehicle “5-inch”; do not mix those dimensions with the F450 mass model.

## Working frame convention

- Body X forward, Y left, Z up (FLU), right-handed.
- World Z up in v2 estimator/control model.
- Sensor-frame raw XYZ must be rotated into body coordinates using a **determinant +1** orthogonal rotation matrix.
- Motor positions used by the geometric model are at approximate X/Y components ±0.1591 m for 0.450 m diagonal span; positions should be remeasured on the actual assembly.
- The mass model reports COM relative to its declared origin near the lower PCB; this is *not automatically* the IMU origin or a ROS base_link origin.

## Physical placement known from project description

3S 4200 mAh pack mounted on/above lower center PCB; ESP32-S3 custom sensor PCB approximately 75 × 100 mm; ESCs about midway along each arm, positioned beneath; four orange A2212/10T-class motors at arm ends. Downward TOFSense F2 Mini around center of lower board. Optional stripped Samsung N150 board intended near center, with horizontal COM aligned to frame center.

## What to measure

Measure motor center coordinates, diagonal, horizontal COM, vertical distances of battery/sensors/ESCs, propeller clearance, and the exact LiDAR optical-center-to-reference offset. Photograph and label nose direction, motor 1–4 order and sensor XYZ arrows. Once the real dimensions exist, update the physics model and URDF; do not carry zero off-diagonal inertia as a fact if battery or wiring is asymmetric.


---

<!-- BEGIN: 01_HARDWARE/02_PCB_PINOUT_AND_BUSES.md -->

# ESP32-S3 PCB and present pinout knowledge

| Signal | Source-level configuration or observation | Confidence |
|---|---|---|
| Main I²C SDA | ESP32 GPIO9 | In calibration/FC source; confirm schematic |
| Main I²C SCL | ESP32 GPIO8 | In calibration/FC source; confirm schematic |
| MPU6050 | I²C `0x68` (alternate `0x69` can be supported) | Observed at 0x68 |
| QMC5883P | I²C `0x2C`, ID `0x80` | Observed |
| BMP280 | `0x76`, optional `0x77`; attached on MPU XDA/XCL aux pins | Observed/implemented; depends on bypass |
| CH343 | Windows USB serial COM3 or COM4 | Observed; number not guaranteed |
| Historical motor GPIOs | FL, FR, RR, RL = GPIO12,11,14,13 | **Code annotation only**, output disabled |
| MPU INT / QMC DRDY | Not required for current software-polling path | No dedicated line necessary |
| LiDAR RX on ESP32 | Undefined in safe v2: `IQ_LIDAR_RX_PIN=-1` | OPEN |

## MPU bypass requirements

MPU6050 `USER_CTRL` register `0x6A=0x00` disables its internal I²C master. `INT_PIN_CFG` **register** `0x37=0x02` enables access to XDA/XCL through bypass. This is an I²C register named “INT_PIN_CFG”; it does not imply a physical interrupt wire.

## Design checks

Use a continuity tester, not only an IDE pin macro, to determine actual on-PCB routes. Confirm I²C pull-ups terminate at a legal logic voltage, not battery. Check BMP280 breakout CSB/SDO strapping if not detected. Verify all ground references including UART adapter ground. No motor pin should be connected to an unknown driver until safe-start behavior is established.


---

<!-- BEGIN: 01_HARDWARE/03_POWER_AND_SIGNAL_INTEGRITY.md -->

# Power, PCB integrity and electrical risks

## Known power system

Battery: 3-cell 4200 mAh lithium-polymer pack. Four nominal 40 A ESCs distribute high transient motor currents. The ESP32-S3, IMU, MAG, and BMP run from low-voltage regulated rails whose detailed schematic, brownout thresholds, and rail decoupling have not been independently examined.

## Historic QMC problem

Older tests detected `0x2C` with chip ID `0x80` and wrote `CTRL1`, then lost I²C reads after entering measurement mode. Possible root causes included sensor-specific repeated-START compatibility, supply integrity, reset timing, MPU bypass interference or bus loading. Wi-Fi transmission introduced interference at some configurations, but failure also appeared with Wi-Fi off. Do not assert “Wi-Fi was the cause.”

## What to inspect with instrument

1. Verify regulated 3.3 V at the sensor *pins* at boot, radio-disabled operation, and configuration changes.
2. Observe 3.3 V droop and ripple on an oscilloscope during motor/ESC switching (later controlled lab test).
3. Check SDA/SCL rise times, stuck-low events and pull-ups with a logic analyzer or scope; judge whether 400 kHz is physically supported.
4. Shorten noisy lines, maintain grounds, route magnetometer away from power conductors and motors where practical.
5. Record cold power cycle versus warm reset; a USB-only disconnect may not depower the sensor if another supply remains connected.
6. Keep LiPo connections isolated from any direct ESP32 GPIO or 3.3 V pin.

## Acceptance

No unexplained resetting, voltage collapse, I²C bus lockup, serial loss, or sensor invalidity in a sustained bench run. Strong magnetic fields and current-generated offsets must be checked again with controlled motor-power tests; a static compass calibration is not the full story.


---

<!-- BEGIN: 01_HARDWARE/04_MOTORS_ESCS_PROPELLERS.md -->

# Motors, ESCs, propellers and propulsion limitations

Hardware described: four orange A2212/10T-class motors, generally called 1400 KV by the project owner, four 40 A ESCs, 3S LiPo, F450 frame. Some older documents mention 1000 KV alternatives and 8045 propellers; **currently installed prop geometry and actual motor winding type remain unverified**.

## What the present numerical model assumes

Idealized maximum thrust **6 N per motor** with 1σ prior 1.5 N, first-order thrust lag 40 ms with 1σ prior 15 ms, yaw moment-per-thrust-arm ratio 0.015 m with uncertainty, and thrust linearly proportional to normalized motor command. These are assumptions from simulation, not real ESC characteristics. A 40 A ESC rating does not supply thrust data.

## Identification sequence

With a suitable guarded, restrained thrust stand, a calibrated force gauge and propeller safety, measure each motor separately at low and medium commands. Record supply voltage under load, command method, stable thrust, current, response lag and motor heating. Verify idle, spin direction, propeller handedness and ESC startup only after an independent hardware cut-off exists. Never infer free-flight thrust margin from an unmeasured “45% throttle” value.

## Critical limit from model

For hypothetical 1.104 kg mass and 4×6 N thrust, a **linear thrust surrogate** predicts hover around 45%. The historical 45% command cap may prevent hover or leave no stabilization headroom. ESC PWM values and actual thrust fractions are not necessarily the same. Calculate thrust margin using measured thrust curves and actual total mass before changing output limits.

## Still open

Motor numbering/order, spin direction, PWM or DShot capabilities, physical ESC signal GPIOs, signal ground continuity, battery voltage/current measurement, guarded operation, arming and independent kill path.


---

<!-- BEGIN: 01_HARDWARE/05_MASS_CENTER_OF_MASS_BOM.md -->

# Mass budget, BOM, COM and uncertainty

## Described physical inventory

| Item | Quantity | Notes |
|---|---:|---|
| F450-like frame, hardware and spacers | 1 | Two red/two white arms |
| 3S 4200 mAh LiPo | 1 | Lower-center area |
| A2212/10T motors | 4 | 1400 KV described; model not independently confirmed |
| 40 A ESCs | 4 | Mid-arm/below arm |
| 75×100 mm electronics PCB | 1 | ESP32-S3, MPU6050, QMC5883P, BMP280, regulators, connectors |
| TOFSense-F2 Mini | 1 | Downward near center |
| Samsung N150 companion motherboard | optional | Screen/battery removed in proposed variant |

## Current *assumption-based* profile estimates

| Property | Basic | N150 variant |
|---|---:|---:|
| Total mass | 1.104 ± 0.051 kg | 1.319 ± 0.083 kg |
| COM z relative to source origin | +0.01866 m | +0.01185 m |
| Jxx | 0.00978 kg·m² | 0.01030 kg·m² |
| Jyy | 0.01039 kg·m² | 0.01091 kg·m² |
| Jzz | 0.01917 kg·m² | 0.01956 kg·m² |

## Error propagation

The standard deviations came from component and geometry priors, *not* repeated independent aircraft measurements. Independence between masses and position errors may be unjustified if hardware is mechanically linked. Mass uncertainty matters to collective feedforward; inertia uncertainty propagates into rate-loop torque gains; COM position affects arm leverage and static balance. Off-diagonal inertia terms should be estimated or bounded if asymmetric components are installed.

## Measurements needed

Weigh the final frame *including cable ties, bolts, wiring, battery and actual props*. Weigh or estimate individual subassemblies; photograph their measured body coordinates and vertical heights; repeat the weighing. Do the same after adding the N150 motherboard. Update the model and only then claim physical posterior uncertainty.


---

<!-- BEGIN: 01_HARDWARE/06_COMPANION_COMPUTER_AND_ROS2.md -->

# Optional companion computer, ROS2, vision and autonomy

The project contemplates removing the screen and battery from a Samsung N150-class portable computer and mounting its remaining electronics near the airframe center for ROS2, a camera and YOLO processing. This is a **second proposed configuration**; the successful 4 Nov sensor calibration is not evidence of integrated ROS2 or YOLO flight.

## Resource and safety constraints

An older single/dual-core Atom-class computer with approximately 1 GB RAM (as discussed in prior planning) cannot be assumed to run contemporary full ROS2 + PyTorch inference with low, deterministic latency. A minimal headless Linux configuration, lightweight inference, measured memory use and resource-limited processes may help, but the companion PC must never own the primary 500 Hz stabilization timing. A loss of ROS2, USB or camera must not keep a motor armed.

## Intended separation

- ESP32-S3: IMU acquisition, calibration, attitude/altitude estimation, validated motor control, failsafes.
- Companion computer: high-latency mission planning, object detection, logging, mapping and optional setpoint suggestions.
- Interface: versioned CRC-protected UART or another measured link, with timestamps and command authority rules.

## Specific study-document trap

The Claude volume mentions an HMC5883L in parts of its hardware overview; the measured board has QMC5883P. Similarly, avoid generic RPLidar `LaserScan` drivers for the TOFSense F2 Mini: this is a downward time-of-flight ranger using its own message format, not necessarily a 360° scanning LiDAR. Camera/YOLO tasks must be evaluated against actual CPU/RAM capabilities rather than a generic tutorial.


---

<!-- BEGIN: 01_HARDWARE/07_XBOX_INPUT_HARDWARE.md -->

# Xbox input hardware and communications reality

The pilot interface target is an Xbox gamepad. The **ESP32-S3 supports BLE, not Bluetooth Classic**, and actual Xbox firmware/software pairing compatibility must be verified. The current FC v2 avoids assuming a reliable direct wireless connection by using a Windows PC gamepad -> CH343 UART bridge for bench tests only.

## v2 virtual commands

- Left stick X: yaw-rate request.
- Left stick Y: vertical-speed request.
- Right stick X/Y: roll/pitch angle requests.
- D-pad Up + LB + RB: barometer height re-zero request **only disarmed**.
- PC sends CRC-protected serial command frames; a 200 ms freshness watchdog invalidates stale input.

## Other repository branch

`esp32-xbox` documented D-pad Down + A/B hold gestures for stable/navigation mode selection, and D-pad Down + LT for latched disarm. Those branch conventions must not be silently spliced into the newer v2 command parser without a coherent authority and safety review.

## For real aircraft use

Requires tested controller power-up and disconnect behavior, independent cut-off/arming state machine, radio quality assessment, stick center calibration, dead zones, bound setpoints, command precedence, lost-link policy and watchdog that has actually been tested while the propulsion system is restrained. The current USB bench bridge is **not a flight radio**.


---

<!-- BEGIN: 02_SENSORS/01_SENSOR_SYSTEM_OVERVIEW.md -->

# Sensor suite, buses and multirate schedule

The complete current FC is a four-sensor design: MPU6050 gyro/accelerometer, QMC5883P three-axis magnetic field, BMP280 pressure/temperature, and optional Nooploop TOFSense F2 Mini downward range. These measurements cannot all be fresh at 500 Hz.

| Sensor | Nominal physical setting in FC v2 | Host scheduling | Purpose |
|---|---|---|---|
| MPU6050 | 500 Hz FIFO at 188/184 Hz DLPF | 2 ms loop, drain bounded backlog | Angular rate and 3D specific force |
| QMC5883P | `0x3D`, ~200 Hz ODR | Poll status every 4 ms | Heading observation, anomaly-aware |
| BMP280 | 166.7 Hz typical ×1/×1 normal mode | Poll ~6 ms | Relative pressure-derived altitude |
| TOFSense F2 Mini | Up to 50 Hz | Optional UART parser | Ground range with quality and tilt gating |

## Time should be carried with data

Each measurement needs an acquisition timestamp, validity flag, freshness/age, configuration ID and optional covariance or quality statistic. A 500 Hz main loop can use the **most recent valid** magnetometer/pressure sample and apply an estimator correction only on fresh arrivals. It must not count 500 repeated barometer values as 500 independent observations.

## Bus topology

ESP32-S3 main I²C GPIO9 SDA / GPIO8 SCL; the BMP resides behind the MPU auxiliary bypass on XDA/XCL. The QMC shares the primary electrical bus. FC v2 is configured for 400 kHz for MPU/BMP and falls back to 100 kHz for QMC read transactions after problems. This is a source-level feature, **not a measured optimal bus profile**.

## Startup order and lock

No ESC output until all prerequisite sensors are recognized, calibrated and assigned physically verified body axes, while input and loop timing meet defined limits. Current v2 enforces a permanent motor-off compile-time guard.


---

<!-- BEGIN: 02_SENSORS/02_MPU6050_REGISTER_CONFIG.md -->

# MPU6050 — register-level acquisition and filtering

## Known silicon and bus

MPU6050 at I²C `0x68`, with `0x69` possible when AD0 is strapped high. Direct six-axis accelerometer/gyro. The FC v2 uses primary ESP32 Wire bus and an aux bypass for BMP280 access; there is no requirement for a physical MPU INT wire.

## FC v2 normal configuration

| Property | Configured | Meaning |
|---|---|---|
| DLPF_CFG | `1` | Gyro nominal 188 Hz bandwidth/~1.9 ms delay; accelerometer nominal 184 Hz/~2.0 ms |
| Sample rate divider | `SMPLRT_DIV=1` | 1 kHz base / (1+1) = 500 Hz |
| Gyro full-scale | ±250°/s | November calibration range; saturation possible during fast tumbles |
| Accel full-scale | ±2 g | November calibration range |
| FIFO | 12-byte accel XYZ + gyro XYZ | Temperature not in each FIFO frame |
| AUX master | disabled | Allows BMP XDA/XCL bypass |
| `INT_PIN_CFG=0x02` | bypass bit | Not an external interrupt pin requirement |

## Scaling and consistency

At ±250°/s the nominal gyro conversion is 131 LSB/(°/s); at ±2g accelerometer nominal sensitivity is 16384 LSB/g. Verify actual register readback before applying conversions. If ranges are changed, the raw-to-SI conversion changes, and calibration biases must be re-expressed consistently; no silently reused counts-to-degrees factors.

## FIFO and latency

FIFO buffering protects against short host delays but introduces measurement age when backlog accumulates. The v2 source limits burst drains, uses newest complete frame, counts discarded frames, detects misalignment and overflow, and resets FIFO when needed. A FIFO backlog is not free precision: repeatedly integrating a stale frame as though it came from the current clock instant produces timing errors. Report FIFO count, `fifo_lost`, `fifo_over`, read duration and effective sample age.

## Boot hardware offsets

Gyro offset register addresses 0x13–0x18 and accelerometer offset area 0x06–0x0B are used by v2. The code must read original values, preserve reserved/factory temperature-compensation bits, write bounded corrections, read them back, then measure **residual bias**; theoretical offset LSB scaling should not substitute for before/after board measurements.

## Research reference

TDK/InvenSense MPU-6000/6050 Register Map Rev 4.2: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf>.


---

<!-- BEGIN: 02_SENSORS/03_QMC5883P_REGISTER_CONFIG.md -->

# QMC5883P — actual compass identity, register map and working profiles

## Chip identity

This is **QMC5883P** at I²C **0x2C**, identification byte **0x80** at register `0x00`. Earlier diagrams or books mentioning HMC5883L/QMC5883L are not the hardware identification. Never use the HMC5883L register order, address or gain constants for these captures.

## Registers referenced in experiments

| Register | Meaning | Known values |
|---|---|---|
| `0x00` | ID | 0x80 |
| `0x01..0x06` | XYZ little-endian signed counts | Three int16s |
| `0x09` | Internal data status (DRDY/overflow) | Polled via I²C, no external DRDY wire |
| `0x0A` | Control 1 | Profiles below |
| `0x0B` | Control 2 | 0x08 (±8 G setting) |
| `0x29` | XYZ sign/control | 0x06 in working programs |

## The two genuinely successful histories

- **v13 standalone:** `CTRL1=0xC9`, 100 Hz nominal, often **~90 fresh Hz**, with some sporadic I²C read failures. It once failed after several seconds; it was not perfectly reliable.
- **4 Nov full calibration:** `CTRL1=0x3D`, nominal 200 Hz low oversampling and **~195 Hz fresh** in the successful capture. The ZIP name containing “100Hz” is not evidence of actual register settings. The later v2 source follows `0x3D`.

200 Hz profile `0xCD` failed in earlier v13 tests. Frequency, oversampling bits, initialization ordering, I²C electrical rate, power state and other bus activity all changed across versions. The claim is “this particular combination worked in this capture,” **not** “every 200 Hz mode works.”

## Data conversion and calibration

In the working ±8 G setting, the observed configuration uses nominal 3750 counts/G (check QST revision). Convert raw counts to µT at that configured sensitivity before applying full 3×3 correction. Matrix and hard-iron offset are sensor-frame quantities; they do not establish physical heading until the magnetometer orientation relative to aircraft is measured.

## Guardrails

Check status freshness; discard invalid or overflowed XYZ; sanity-check vector norm and age; don't trust large abrupt magnetic-field disturbances. Do not automatically reinitialize on every missed reading: repeated writes can perturb a previously running sensor. Time-limit failed I²C transactions and preserve independent IMU acquisition.

Primary reference: QST QMC5883P Rev E datasheet, <https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf>.


---

<!-- BEGIN: 02_SENSORS/04_BMP280_FILTERS_AND_COMPENSATION.md -->

# BMP280 — pressure altitude, filter and real output rate

BMP280 is a pressure/temperature sensor (typically `0x76`) behind MPU6050 XDA/XCL bypass on this PCB. Raw ADC pressure and temperature require device-specific factory compensation coefficients. FC v2 caches those trims and reads a single burst of raw data, then uses the Bosch integer compensation routines.

## Actual v2 profile

- Normal measurement mode.
- Temperature oversampling ×1; pressure oversampling ×1.
- IIR filter ×4.
- Standby 0.5 ms.
- Typical **166.7 Hz** effective pressure output according to Bosch's mode table, not 200 independent new conversions per second.
- Firmware polls approximately once every 6 ms, ignores duplicated raw conversion values and records freshness separately.

## Why not simply maximize smoothing?

Pressure oversampling and IIR reduce high-frequency noise but increase conversion time, group delay and transient response time. The vertical estimator uses inertial acceleration for prediction and slow pressure for drift correction; lag cannot be wished away by “more Kalman” or higher polling frequency. Test actual ascent/step response before choosing higher IIR.

## Relative height

Pressure altitude relative to a boot baseline P0 is estimated by:

\[
h = 44330\left[1-\left(\frac{P}{P_0}\right)^{0.190294958}\right].
\]

This is an atmospheric approximation and assumes the baseline reference remains meaningful. It does not correct propwash, local airflow, thermal drift, weather variations or aircraft-specific pressure-port placement.

## Calibration and re-zero

At boot, collect stable pressure measurements, compute P0 and its statistical spread, then establish z=0. v2 also supports D-pad Up+LB+RB disarmed-only re-zero; never allow an in-flight uncommanded baseline jump. Air pressure variation with temperature is not direct evidence of true vertical motion.

Reference: Bosch BMP280 datasheet, <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf>.


---

<!-- BEGIN: 02_SENSORS/05_TOFSENSE_F2_MINI.md -->

# TOFSense F2 Mini — range sensor integration and calibration

Device: Nooploop TOFSense F2 Mini mounted approximately under the lower center PCB and pointed toward the ground. The November 4 successful calibration session contains **no validated LiDAR samples**; the earlier GUI allows LiDAR stages to be skipped without losing IMU/MAG/BMP data.

## Known software interfaces

- Older calibration GUI expects LiDAR on **a second USB-to-UART COM port**, not embedded in the ESP32 52-byte calibration packet.
- FC v2 includes an optional 16-byte Nooploop Frame0 parser, checksum/status checks and tilt-compensated relative height input.
- `IQ_LIDAR_ENABLE=0` and `IQ_LIDAR_RX_PIN=-1` by default. No actual ESP32 GPIO routing has been verified.
- Typical maximum fresh range output is around 50 Hz for the referenced F2 Mini mode; verify exact model firmware and mode.

## Geometry

If the downward sensor measures slant distance r to a locally flat ground plane and body tilt angle is α, the vertical component is approximately `r cos(α)`, after accounting for the sensor's body-axis orientation and optical-center offset. Incorrect sign of +Z or incorrect offset can shift all altitude readings.

## Calibration plan

Calibrate at actual known distances 0.25, 0.50, 1.00 and 2.00 m, with 300 samples per distance in the historical GUI defaults. Measure uncertainty, dropouts, confidence/signal-strength flags and tilt behavior. Validate the ranging surface and minimum distance. Store measured offset/scale/variance per distance, and use innovations to reject bad measurement updates rather than forcing the Kalman filter to follow them.

## Flight readiness

A LiDAR can improve near-ground height estimation but cannot be assumed to work over every surface, in bright light, at large tilt or through propwash/dust. Do not arm on an unverified UART RX pin or sensor status.

Reference: Nooploop TOFSense F2 Mini datasheet and `nlink_tofsense_frame0` parser cited in FC v2 sources.


---

<!-- BEGIN: 02_SENSORS/06_I2C_BUDGET_100K_VS_400K.md -->

# I²C bandwidth, scheduling and transaction hazards

## Separate clocks

- **I²C clock**: electrical bus clock in Hz (100 kHz and 400 kHz tested/planned).
- **UART baud**: physical CH343 serial bit rate (921600 baud for binary capture and FC bench; 115200 for text-only v13 MAG probe).
- **Sensor ODR**: internal conversion frequency (500 Hz MPU, ~200 Hz QMC, ~167 Hz BMP in v2).
- **Control frequency**: how often the controller runs its calculations (target 500 Hz).

These are not interchangeable. A 921600-baud UART does not repair I²C register access.

## First-order bus budget

At 400 kHz, a 12-byte I²C FIFO read consumes at least about 108 wire-clock bits for the bytes alone (~270 µs), plus address phases, register pointer, repeated starts, transaction overhead and MCU latency. At 100 kHz the corresponding byte payload requires at least ~1.08 ms, already more than half of a 2 ms loop. MAG, pressure and setup/health reads further compete for bus time.

The above is a *lower bound*, not measured completion time; different repeated-START and STOP behaviors, errors, timeouts and FIFO backlog can dominate actual latency.

## Implemented approach

- MPU/BMP use 400 kHz primary bus in FC v2.
- QMC may fall back to 100 kHz transaction rate after confirmed problems.
- FIFO burst management is bounded rather than endlessly catching up.
- BMP factory trims cached and raw ADC acquired in one burst.
- Text reporting at about 1 Hz; do not dump multiple verbose CSV lines inside each 2 ms deadline.

## Required bench data

At least several minutes of `FC` and `FC_TIMING`: actual loop rate, minimum/mean/worst duration, FIFO drops, QMC fallback state, MAG and BMP fresh rates, bus error counts and P99 latency. Measure under motion and with representative UART traffic. A 5 ms timeout violates a 2 ms control deadline: fault handling must be bounded and take priority over convenience.


---

<!-- BEGIN: 02_SENSORS/07_CLOCKS_INTERRUPTS_AND_FIFO.md -->

# Timing design — sample clock, FIFO, callbacks and deadlines

An MCU scheduler should distinguish **sensor acquisition**, **estimator update**, **controller update** and **telemetry**. FC v2 uses a nominal absolute 2 ms scheduling loop. This avoids accumulated `delay(2)` drift but does not make all I²C calls bounded below 2 ms.

## Sources of latency

- MPU internal DLPF ~2 ms group delay, even before host I²C transaction and FIFO age.
- Host services FIFO late: newest sample age depends on backlog length and burst handling.
- QMC status polled every 4 ms may report a conversion that finished just after previous poll.
- BMP normal-mode conversion cycle and IIR introduce their own delay.
- UART console/USB buffering and Windows COM activity can burst unrelated to the main loop.

## Why no external INT/DRDY was requested

The actual board had unreliable or unavailable physical interrupt lines. The proven calibration path uses software polling and internal status bits. A future hardware revision can use MPU INT for deterministic FIFO wakeup, but do not claim it is connected now. `INT_PIN_CFG` in MPU config is a bypass-control register name, not physical interrupt wiring.

## Recommended log record

`{acq_timestamp_us, sensor_id, seq, age_us, flags, raw_values, calibrated_values, read_duration_us, fifo_level, missed_deadline}`.

Control should always consume a coherent snapshot and reject invalid/aged signals; it should never block while waiting for a missing MAG or BMP conversion. Keep a histogram of deadline overruns, not only average Hz.


---

<!-- BEGIN: 02_SENSORS/08_SENSOR_FRAMES_SIGN_AND_ALIGNMENT.md -->

# Sensor axes, mounting transforms and sign validation

The independent calibration matrices operate in **each sensor's native XYZ axes**. They are not a statement that sensor X is aircraft roll X. A valid physical mounting matrix `R_BS` transforms sensor-frame vectors into the body frame and must satisfy:

\[
R_{BS}^T R_{BS}=I,\qquad \det R_{BS}=+1.
\]

## Recommended order

1. Decode raw sensor XYZ in chip register order.
2. Convert to engineering units (rad/s, m/s², µT).
3. Apply offsets and full 3×3 calibration *in the sensor frame*.
4. Rotate with the individually verified `R_BS` for gyro/accel/MAG.
5. Feed coherent body-frame vectors into quaternion propagation, accelerometer gravity correction and yaw innovation logic.

## Key historical issue

A simple GUI display X/Y exchange was found to have determinant −1 if treated as a 3D physical rotation. A reflection is not a legal rigid-body orientation. A display convenience must never enter the estimator or mixer.

## Propeller-free verification

- Place aircraft level in the declared FLU reference; static specific force should point roughly +body Z.
- Pitch the nose down and check the **sign of attitude and gyro rate**, not just magnitude.
- Roll to the right, yaw manually, and compare magnetic yaw trend with gyro yaw.
- Rotate MAG in a level circle and verify heading angle continuity and magnetic magnitude before any motor test.

The v2 source currently uses identity sensor-to-body placeholders. **Unverified identity mappings block physical flight**.


---

<!-- BEGIN: 03_CALIBRATION/01_NOV4_CAPTURE_OVERVIEW.md -->

# 4 November 2026 — successful integrated raw calibration session

## What was actually delivered

User archive: `IntelQCopter_Calibration_20261104.zip`, containing `iq_500hz_20261104_100907.csv`, `calibration_20261104_100907.json`, a MAG image, original `calibrate_sensors.py`, skipped-stages metadata and README. The independent audit reprocessed the raw CSV without copying its calibration coefficients.

## Sample inventory

| Stream/stage | Measured count or reported property | Interpretation |
|---|---|---|
| Total received rows | **98,868** | Combined session, not all static gyro or full-mag training samples |
| UART sampling | ~500 Hz class, 2 ms timestamp targets | Must distinguish timestamp target from timing jitter |
| Gyro rest | 30,000 | ~60 s stationary bias estimation |
| Six-face accelerometer | 30,000 | 5,000 each face; assumes nominal orthogonal poses |
| MAG rotation training | 14,000 | 2,000 each yaw/pitch/roll plus 8,000 3D tumble |
| Fresh MAG flags over entire raw session | 38,652 (prior review) | ~196 Hz over recorded windows; not 500 Hz independent observations |
| BMP rest | 1,000 accepted | Pressure SD 0.640 Pa, altitude SD 0.0548 m |
| LiDAR | 0 accepted | Not measured, postponed |

## Important actual settings

Despite some ZIP folder names saying “100 Hz”, the successful firmware snapshot configured **QMC Control1 `0x3D`** (nominal 200 Hz) and observed about **195 Hz fresh**. The source used the QMC5883P chip and 921600-baud CH343 binary UART. A separate v13 firmware had earlier worked with `0xC9` (nominal 100 Hz). These are distinct experiments.

## Main conclusions

- Real integrated MPU/MAG/BMP acquisition was demonstrated during the session.
- Independent raw-register conversions agreed with reported engineering units within serialization/rounding precision.
- The original saved MAG ellipsoid fitter had a defective/trivial solution construction; independent refit gave candidate 3×3 parameters.
- Sensor alignment to aircraft axes, thermal drift and operating with motors running were not validated.
- No claim of flight-controller or ESC functionality follows from this calibration capture.


---

<!-- BEGIN: 03_CALIBRATION/02_GYRO_BIAS_AND_NOISE.md -->

# MPU6050 gyroscope — stationary calibration and precision

## Independent 4 November estimates

30,000 stationary samples, using original calibrated engineering-unit stream:

| Axis | Mean zero-rate bias, deg/s | Within-run SD, deg/s |
|---|---:|---:|
| X | +0.161968974 | 0.047727251 |
| Y | −0.304950151 | 0.044855293 |
| Z | −0.877433074 | 0.043199902 |

In software, with matching units and full-scale configuration, `omega_cal = omega_raw - b_g`. For the attitude estimator, convert degrees/s to radians/s **after or consistently with bias correction**. These numbers are sensor-frame values; apply proper `R_BS` before interpreting as body p/q/r.

## Hardware offset version

FC v2 writes coarse gyro compensation to MPU6050 offset registers, reads back, and samples ~600 stationary readings after that write. The correct residual subtraction is **post-write measured residual**, not original raw bias again. Log original register words, written words, post-write mean and SD to detect wrong LSB scaling or temperature-compensation-bit corruption.

## Precision is not accuracy

Stationary white-noise SD is small, but it is not a complete Allan deviation or temperature bias model. Other 4 November sessions showed between-session biases larger than within-run uncertainty. A large 30k sample count does not cancel temperature drift or systematic offset. A brief stationary check at the actual board temperature remains justified.

## Saturation warning

Fast MAG tumble recorded some gyro samples near ±250°/s limit (~21.7% of samples with at least one axis close to that limit in previous analysis). The owner plans to revisit the full-scale range. Changing the range changes sensitivity and potentially noise; log the new range and redo any required scale verification rather than reusing ±250°/s constants blindly.


---

<!-- BEGIN: 03_CALIBRATION/03_ACCEL_SIX_FACE_FIT.md -->

# MPU6050 accelerometer — full six-face matrix calibration

## Independent candidate

Bias vector, in m/s²:

```text
b_a = [ 1.264655572, -0.059103429, -0.678744839 ]
```

Direct correction matrix `C_a`, dimensionless:

```text
[ 0.999284313  -0.038750762  -0.032967877 ]
[ 0.041351387   0.992793975   0.040449121 ]
[ 0.042481183  -0.040866969   0.987204883 ]
```

Use `a_cal = C_a · (a_SI - b_a)` in sensor coordinates, with the same original ±2 g ADC scaling. The cross-axis entries are not optional if reproducing the fitted solution; a single scale factor cannot implement this 3×3 map.

## Fit statistics and limitation

30,000 samples across six nominal face orientations (5,000 per face). Reanalysis obtained **0.05331 m/s²** component RMS fit residual and **0.05334 m/s²** chronological within-pose holdout. Corrected gravity-magnitude RMS was approximately **0.0240 m/s²**. Leave-one-face-out errors in an earlier audit reached ~0.356 m/s², demonstrating sensitivity to face placement and possible axis misalignment.

## Important interpretation

Small within-pose residuals prove repeatability and the numerical fit, not an exact physical mounting rotation. Six hand-held or manually placed faces may not be perfectly orthogonal; those positioning errors are conflated with accelerometer cross-axis coefficients. For high precision, repeat with an alignment jig or surveyed orientations if available.

## Hardware accel offsets

MPU6050 hardware offset registers can remove a coarse shift but **cannot implement cross-axis calibration**. If hardware offsets are written at startup, adjust the remaining bias based on empirically observed register effect so there is no double correction. Preserve factory temperature compensation bits. Validate corrected norm and direction after all transforms before enabling attitude feedback.


---

<!-- BEGIN: 03_CALIBRATION/04_MAG_HARD_SOFT_IRON_FIT.md -->

# QMC5883P magnetometer — independently fitted 3×3 calibration

## Independent candidate from raw 4 Nov rotations

Hard-iron bias in microtesla:

```text
b_m = [ -29.482301006, -18.397983738, -35.474272846 ]
```

Relative full soft-iron correction, symmetric, determinant normalized to 1:

```text
C_m =
[ 0.973366985   0.005036971   0.038277934 ]
[ 0.005036971   0.964534610  -0.017911741 ]
[ 0.038277934  -0.017911741   1.067011358 ]
```

Apply **in the sensor frame**:

\[
m_{cal}= C_m(m_{raw\,µT}-b_m).
\]

The fitted norm is **~40.743 µT** in the relative scaling, with SD **1.2504 µT** (coefficient of variation **3.069%**). Full-matrix held-out magnitude RMS error was ~3.46%, versus ~5.06% for a nonlinear diagonal-only model.

## Independent cross-checks

Two mathematical fits (robust nonlinear and algebraic ellipsoid) agreed on offset to better than ~0.1 µT per axis in the earlier audit. Validation on independent pre-10:09 sessions produced ~3.79% and ~3.46% relative RMS magnitude error. That is evidence the correction generalizes *for those recording conditions*, not proof of accurate heading in a motor-powered environment.

## Original fitting defect

The prior saved `calibrate_sensors.py` attempted an ellipsoid regression admitting a trivial constant solution. It also labeled the inverse-squared ellipsoid equation matrix as though it were the direct correction. These errors produced degenerate coefficients and a nearly collapsed plot. The independent fit did **not** discard 970 legitimate low uncorrected-norm samples simply for being below 20 µT.

## Physical limits

Ellipsoid fitting from rotation gives offset and relative shape but not absolute Earth-field scale, body-frame orientation, magnetic declination, or current-induced disturbances. Use a physically proper `R_BS` with det+1 and test the vector with real motor currents in a controlled environment. Disable yaw magnetic correction if field norm, coherence, freshness or inclination gates fail.


---

<!-- BEGIN: 03_CALIBRATION/05_MAG_WORKING_CONFIG_HISTORY.md -->

# Why the magnetometer finally worked — the history without oversimplification

## What failed

Multiple isolated and full GUI branches previously detected QMC5883P ID=0x80 and ACKed writes, then failed reads on status `0x09` after starting measurement mode. The failing v13 200 Hz `0xCD` attempt recorded zero fresh samples. v17 unattended tests had seven completed variants with no fresh MAG data and additional experiments blocked by COM busy. Wi-Fi made low-rate behavior worse in some observations, but problems persisted with Wi-Fi off.

## What worked

- Original isolated v13 source: **`0xC9` 100 Hz**, readback `0xC9`/`0x08`, measured ~90 fresh Hz during the Nov 3 probe. A later probe was not indefinitely stable.
- Original v13 driver transplanted into full firmware: recovered a path to valid integrated MAG data.
- Successful 4 Nov capture: **`0x3D`** low-oversampling nominal 200 Hz, ~195 new readings/s, together with MPU/BMP. The successful capture's firmware config is more authoritative than its `100hz` archive name.

## Lessons

1. Do not replace a working transaction pattern with an elaborate fallback just because a datasheet theoretically permits it.
2. Chip ID/readback alone does not prove valid measurements; capture actual XYZ sample count and timestamps.
3. One working session does not establish reliability under Wi-Fi, hot boot, motor EMI or bus saturation.
4. Cold power removal and firmware-only reset are different experiments.
5. Do not re-label the physical `0x3D` success as 100 Hz to fit a previous assumption.

## Future change discipline

Preserve the Nov 4 successful source as a golden baseline. Put any new 400 kHz FIFO/FC migration in a separate branch. Use a regression trace with MAG rate, I²C errors, control readbacks, actual power state and MPU/BMP load before merging changes.


---

<!-- BEGIN: 03_CALIBRATION/06_BAROMETER_BASELINE_AND_VARIANCE.md -->

# BMP280 baseline, noise and zero altitude

## November 4 stationary capture

1,000 accepted barometric samples. Mean pressure approximately **98,382.027 Pa**, stationary pressure SD **0.63965 Pa**, and observed relative-altitude SD **0.05484 m**. These describe a particular stationary test and sensor/filter setting, not a universal pressure sensor absolute-height accuracy specification.

## Startup sequence

1. Read and validate BMP identity and factory trim.
2. Configure normal mode and a bounded wait for stable readings.
3. Collect stationary, fresh pressure observations and compute mean P0, SD and sample count.
4. Set relative altitude zero to P0; initialize altitude-estimator height state and covariance consistently.
5. Reject resets while commanded/estimated motion is significant, and do not accept disconnected sensor zeroing.

## D-pad altitude re-zero

`Up + LB + RB` on the Xbox bridge requests disarmed-only re-zero. Only a fresh, validated pressure window should update P0. Never let a button-press while airborne instantly shift the reference; it would create an uncontrolled height error and PID transient.

## Kalman model tuning

FC v2 uses **0.22 m** provisional barometric measurement SD, deliberately larger than 0.055 m stationary, to reflect unmeasured flow/temperature/propwash effects. This is not an experimentally fitted dynamic variance. Check innovation residuals, temperature and pressure port placement before decreasing the assumed R. LiDAR, when validated, can independently stabilize near-ground relative height.


---

<!-- BEGIN: 03_CALIBRATION/07_LIDAR_CALIBRATION_PLAN.md -->

# Optional LiDAR calibration — complete plan for when available

The GUI intentionally supports skipping LiDAR while still collecting gyro, accel, MAG and barometer data. The 4 Nov session did not have any accepted LiDAR samples, and this should remain an explicit **pending** status.

## Planned stage targets

| Reference distance | Accepted readings |
|---|---:|
| 0.25 m | 300 |
| 0.50 m | 300 |
| 1.00 m | 300 |
| 2.00 m | 300 |

Use a rigid reference surface and an independent ruler/tape. Record sensor lens-to-target distance, mounting Z offset and known orientation. Use the original Nooploop UART frame protocol and compare status bits/precision fields, not raw distance alone.

## Fit and validation

Compute per-range mean, median, SD, dropouts and bias against the ruler. Test different target reflectances, angles and lighting as appropriate for the model. Fit a simple offset/scale only if the residuals support it. Do not force a single linear correction if distance errors change discontinuously or low-confidence readings dominate.

## ESP32 integration

Before using range in FC v2, physically identify RX GPIO, voltage levels and UART ownership. Do not confuse the separate GUI USB-UART port with the FC's disabled optional UART parser. Match timestamp and coordinate frame; use tilt compensation only when ground geometry is sensible and attitude estimate reliable. Use innovation gates in altitude Kalman and retain barometric fallback for missing range.


---

<!-- BEGIN: 03_CALIBRATION/08_TEMPERATURE_NOISE_ALLAN_AND_VALIDATION.md -->

# Calibration error budget, temperature effects and better validation

## Error hierarchy

1. Random sensor white noise (measurable from stationary data).
2. Temporally correlated noise, vibration and aliasing (not reduced as 1/sqrt(N) when samples are correlated).
3. Bias shift with sensor temperature, regulator heat, startup time and PCB strain.
4. Accelerometer face/alignment error, magnetometer orientation and hard/soft iron drift.
5. Electrical interference, motor current, pressure flow and time-alignment errors.
6. Quantization, ADC full-scale saturation and uncompensated manufacturer trim.

## Data-collection improvements

Record sensor temperature and elapsed time alongside bias. Repeat stationary gyro sessions after warm-up and after power cycles. Use Allan deviation or bias stability curves to separate white noise, bias instability and random walk, with clear observation-window assumptions. Collect held-out six-face and independent MAG rotation sessions; do not train and validate on the exact same samples.

## How to report uncertainty

State whether intervals represent: within-session SD, bootstrap interval over blocks, between-session variation, setup error estimate or assumed Monte Carlo prior. The independent gyro 30k bias is estimated with high numerical precision, but that precision should not be reported as temperature-invariant physical accuracy.

## Trigger for recalibration

Change of sensor mounting, full-scale ranges, magnetometer distance to power conductors, firmware filter settings, large persistent innovation residuals, changed ground-frame orientation or shock/damage. Make versioned calibration files with date, sensors, config registers, firmware hash and raw-capture checksum.


---

<!-- BEGIN: 03_CALIBRATION/09_APPLYING_MATRICES_IN_REAL_TIME.md -->

# Applying hardware offsets and 3×3 matrices in the FC loop

**Correct order for a software-only reference:**

```cpp
// Conceptual pseudocode. Values and units must match actual ADC config.
Vec3 gyro_dps  = raw_gyro_to_dps(raw_gyro);
Vec3 accel_si  = raw_accel_to_mps2(raw_accel);
Vec3 mag_uT    = raw_mag_to_uT(raw_mag);
Vec3 gyro_cal  = gyro_dps - gyro_bias_dps;
Vec3 accel_cal = C_accel * (accel_si - accel_bias_si);
Vec3 mag_cal   = C_mag * (mag_uT - mag_bias_uT);
// Then apply *individually verified* sensor-to-body rotations.
```

## With gyro/accel hardware offset registers

The hardware outputs change before converting counts to SI. If a known bias is compensated by hardware, measure the effective delta; only the **remaining residual** belongs in software. A full cross-axis matrix still must run every new sensor sample. Preserve temperature-compensation bits and don't write magnetometer hard/soft iron into unsupported registers: QMC correction stays in software.

## Floating-point workload

A 3×3 matrix-vector product is nine multiplies and six adds, negligible compared with I²C or UART waits at 240 MHz. Every 2 ms loop may apply *latest calibrated* MAG, but it must only execute magnetic Kalman/heading corrections when a **fresh** MAG conversion appears. No benefit to re-correcting identical cached MAG bytes at 500 Hz as independent data.

## Full-scale caveat

All stored calibration quantities use sensor-frame engineering units. If MPU full-scale is raised later, nominal ADC sensitivities change; the same SI correction may remain a first candidate, but raw conversion, noise and hardware-offset scaling need revalidation. Cross-axes and biases can shift with mounting and temperature.


---

<!-- BEGIN: 03_CALIBRATION/10_CALIBRATION_DATA_FORMATS.md -->

# Calibration file formats and provenance

## Full session files

- `iq_500hz_20261104_100907.csv`: authoritative raw/engineering-unit time series from the successful capture.
- `calibration_20261104_100907.json`: output of original calibration program; **not automatically authoritative for MAG 3×3** because of its fitting error.
- `iq_500hz_20261104_100907.skipped.json`: skipped-stage metadata; stage status should not be inferred merely from missing CSV columns.
- `calibration_20261104_100907.mag.png`: original plot (MAG correction method later found to be wrong).
- `calibrate_sensors.py`: original algorithm, useful for history and regression, not a validation oracle.

## Independently computed files

- `calibration_candidate_UNVERIFIED.json`: carefully unit-labeled gyro, accel and MAG candidate constants.
- `independent_calibration_results.json`: detailed fit statistics and validation.
- `calibration_candidate_UNVERIFIED.hpp`: C++ constants for integration after axis validation.
- `corrected_capture_CANDIDATE.csv.gz`: derived outputs for all 98,868 rows.
- `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.pdf`: methods and warnings.

## Storage rules

Keep originals immutable; preserve timestamps, sample flags, raw int16 registers and units. Name transformed outputs as **derived** and pin the source commit/config in metadata. Do not mix `C_accel` correction matrix with the inverse of a least-squares normal matrix; do not mix normalized MAG shape calibration with a truly calibrated absolute field strength.


---

<!-- BEGIN: 04_FIRMWARE/01_SOFTWARE_BRANCHES_AND_RELEASES.md -->

# Software tracks, Git branches and artifact families

The IntelQCopter repository history includes multiple independent tracks. A new version number in one track should not silently supersede another.

## Branch families

- `main`: older RP2350 flight control/modeling/PCB work; its contents are not the November ESP32 sensor calibration source by default.
- `esp32-xbox`: ESP32-S3 Xbox and ROS2/Gazebo architecture, host/simulated flight-control logic; physical motor arming disabled in reviewed source.
- `codex/stepwise-esp32`: incremental hardware bench and verified sensor acquisition.
- `codex/qmc-200hz-continuous-diagnosis`: magnetometer problem investigation; check actual branch head before cherry-picks.

## Local artifact families

1. Calibration GUI and sensor source v5-v11; v13 transplant; `100Hz_Original_v13_Driver_Full_Calibration.zip`.
2. Magnetometer-only forensic v12-v18 (standalone nonbinary experiments).
3. Raw calibration and independent audit from 4 Nov.
4. FC bench v1, integrated FC core v1 and Fast 500 Hz FC v2.
5. Academic study (`Volume I Edition 2`) and uncertainty-model packages.

## Version-control discipline

Before committing, compare `platformio.ini`, `src/main.cpp` and `include/` against the target branch. Do not merge the v13 MAG-only `main.cpp` as the FC `main.cpp`: it would remove the other sensors and/or binary transport. Keep a golden calibration capture/config tag. Use a separate `docs/` branch or project `docs` folder for this knowledge base if desired; no GitHub changes are implied by this archive.


---

<!-- BEGIN: 04_FIRMWARE/02_FULL_CALIBRATION_GUI.md -->

# Full GUI calibration firmware — original v13 driver integration

The known full calibration project is a **motor-disabled sensor recording and calibration tool**, not an ESC flight controller. It contains `src/main.cpp`, optional v13 `src/mag_probe.cpp`, GUI `tools/live_calibration_gui.py`, Python calibration scripts, UART preflight and the Windows PowerShell launcher.

## Preferred launcher

From extracted project root:

```powershell
.\tools\start_calibration.ps1 -UploadFirmware -Check -Seconds 20
.\tools\start_calibration.ps1       # installed firmware; GUI only
.\tools\start_calibration.ps1 -Port COM3 -UploadFirmware
.\tools\start_calibration.ps1 -MagProbe -Seconds 20
```

The `-MagProbe` mode **temporarily flashes a different 115200-baud text-only firmware**. After running it, the normal 921600-baud binary calibration must be reflashed. Neither command is equivalent to a flight-controller build.

## Historical defaults

30,000 stationary gyro; 5,000 accel per each of six faces; 2,000 MAG per each of three rotations; 8,000 MAG tumble; 1,000 BMP; 3 s settle; optional four LiDAR 300-sample stages via a separate COM port or GUI skip. Auto-detect CH343 COM, GUI at 921600 baud, Wi-Fi off.

## Why preserve it

The full sensor capture generated the evidence behind the candidate calibration matrices. Even after FC v2 improvements, retain this independent calibration tool as an immutable “known-good” reference. Further changes should be tested on a copy.


---

<!-- BEGIN: 04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md -->

# Fast FC v2 — actual source architecture

**Archive:** `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip`. This is the most recent supplied source for a 500 Hz mathematical FC, configured **motor-off at compile time**.

## Key files

| Path inside archive | Responsibility |
|---|---|
| `src/main.cpp` | GPIO/input safety, I²C sensors, boot, FIFO, BMP/MAG updates, scheduler, UART and diagnostics |
| `include/flight_core.hpp` | Vector/matrix/quaternion math, calibration constants, airframe model, 7 PIDs, virtual motor mixer, health bits |
| `include/pid.hpp` | Filtered derivative, clamping, anti-windup |
| `include/altitude_kf.hpp` | 3-state [height, velocity, accel bias] Kalman with Joseph update |
| `include/bmp280_comp.hpp` | Bosch raw pressure/temperature compensation |
| `include/control_link.hpp` | $IQ2 CRC serial command decoder |
| `include/tofsense_frame0.hpp` | Optional LiDAR frame parser |
| `platformio.ini` | Build environment and safety flags |
| `tools/run_fc.ps1` | Upload vs capture, CH343 detection |
| `tools/xbox_uart_bridge.py` | Windows Xbox -> UART commands for virtual bench only |
| `tests/test_core.cpp`, `tests/test_project.py` | Host numerical and code-path tests |

## Control data path

`MPU FIFO + QMC + BMP (+optional LiDAR) -> SI conversion -> hardware residual / 3×3 matrices -> body alignment -> quaternion + height KF -> seven cascaded PIDs -> virtual X mixer -> FC log`.

It is not a real four-motor closed loop. **IQ_MOTOR_OUTPUTS_ENABLE=0** is checked and the controller explicitly refuses to clear itself for flight. Compile-time motor lock remains an essential safety constraint.

## Observability

`FC_BOOT`, `BOOT_BIAS`, `BOOT_BARO`, `FC`, `FC_TIMING`, and `FC_MODEL` contain health and timing fields. These are diagnostic telemetry, not an approval signal. Onboard timing and sensor success of v2 have not been supplied in the same way that the 4 Nov calibration raw capture was.


---

<!-- BEGIN: 04_FIRMWARE/04_SCHEDULER_FIFO_AND_LATENCY.md -->

# Scheduler, FIFO buffering and low-latency design

## Core target

500 Hz controller means one 2 ms period. FC v2 uses absolute microsecond deadlines and a bounded FIFO read path. `SMPLRT_DIV=1` with DLPF configured gives nominal 500 Hz internal IMU updates.

## FIFO contract

Each selected MPU FIFO frame contains **12 bytes**: accelerometer XYZ (6 bytes) and gyroscope XYZ (6 bytes). The v2 driver measures `FIFO_COUNT`, reads a limited burst, chooses the newest complete frame and records discarded/backlogged samples; resets when near overflow or misaligned. The temperature sample is cached, not included in that selected 12-byte frame.

## Why a FIFO isn't a speed multiplier

A sensor sample can be produced on time but processed late if the host is blocked by a QMC retry or BMP transaction. The latest frame might have less age than the oldest buffered frame, but dropping frames changes the true integration interval. Use actual sample timing/freshness and count missed deadlines. Host capture frame rate is not a substitute for controller timing measurement.

## Monitoring

Read the v2 logs: `hz`, `imu`, `fifo`, `fifo_empty`, `fifo_lost`, `fifo_over`, `late`, `miss`; `FC_TIMING` records sensor read maxima and loop max, QMC fallback, BMP duplicate count. A high average rate with a long tail of 5–10 ms stalls is not safe deterministic 500 Hz operation.

## Next optimization after measurement

If I²C is the bottleneck, reduce optional traffic, split bus domains where physically possible, decimate status reporting, use short bounded timeouts, and consider a SPI-capable future IMU revision rather than silently overrunning deadlines. Do not change a working shared bus without an A/B capture.


---

<!-- BEGIN: 04_FIRMWARE/05_BUILD_CONFIGURATION.md -->

# PlatformIO v2 build flags and Windows workflow

## PlatformIO source settings

```ini
[platformio]
default_envs = iq-fc-safe
[env:iq-fc-safe]
platform = espressif32@7.1.3
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
build_unflags = -std=gnu++11
build_flags =
  -std=gnu++17
  -DCORE_DEBUG_LEVEL=0
  -DIQ_I2C_HZ=400000
  -DIQ_WIFI_ENABLED=0
  -DIQ_MAG_CONTROL1=0x3D
  -DIQ_MOTOR_OUTPUTS_ENABLE=0
  -DIQ_AIRFRAME_PROFILE=0
  -DIQ_LIDAR_ENABLE=0
  -DIQ_LIDAR_RX_PIN=-1
```

## Usage — safe bench

```powershell
py -3 -m pip install -r requirements.txt
.\tools\run_fc.ps1 -UploadFirmware  # build, upload, log
.\tools\run_fc.ps1                  # log only
.\tools\run_fc.ps1 -Port COM3       # override auto-detection
py -3 tools\xbox_uart_bridge.py --inspect
.\tools\run_fc.ps1 -Xbox           # gamepad virtual-input mode
```

No root `START_*.ps1` is required. Close every other COM program before flashing or running the bench bridge; only one process can own the CH343 at a time. The USB upload baud of 460800 and UART running baud of 921600 are separate channels/settings. Windows PowerShell 5.1 encoding (UTF-8 BOM vs em dash) caused older script parsing failures; v2 retains an ASCII-friendly launcher.

## Do not silently change

Disable `IQ_MOTOR_OUTPUTS_ENABLE` guard only after independent actuation design/review; the project is expressly not ready for that. Do not set LiDAR enable with unknown RX pin. Do not force 200 Hz BMP fresh status by just polling faster. Preserve the proven QMC profile while benchmarking bus speed.


---

<!-- BEGIN: 04_FIRMWARE/06_BOOT_AND_REGISTER_OFFSET_SEQUENCE.md -->

# Boot sequence, register offsets and zero references

## FC v2 boot intent

1. Keep motor GPIOs in INPUT state, disable Wi-Fi, start UART diagnostics.
2. Start I²C on verified GPIOs; detect MPU and establish AUX bypass (`0x6A/0x37`).
3. Initialize QMC5883P (preserved successful profile); initialize BMP280 with factory trims and compensation.
4. Apply coarse MPU gyro/accelerometer **register** offsets; read back to verify writes.
5. Acquire ~600 stationary observations *after* writing offsets. Estimate residual gyro bias and assess stability; reject motion.
6. Initialize quaternion roll/pitch from calibrated gravity (yaw initially arbitrary); check up-axis plausibility.
7. Average fresh stable BMP pressure and set P0, relative zero altitude, Kalman initial state.
8. Enable/reset FIFO only **after** stationary calibration; then start nominal 500 Hz loop.
9. Print boot health/model and preserve motor disabled state.

## Double-correction hazard

If hardware offset removes part of original bias, software must subtract only the remaining offset. `flight_core.hpp` provides `setResidualBias()` and `setHardwareAccelDelta()` to adjust the remaining software state. Because accelerometer cross-axis matrix cannot be written into MPU registers, it remains a per-sample software transform.

## Validation priority

Offset register scaling is partly a source-level *theoretical conversion* in v2 and is not verified on this actual board. Record before/after raw stationary counts, temperature, register readbacks, transformed gravity, gyro SD and any clipping. Wrong register signs or factory compensation bits can make an apparently precise calibration worse.

## During a flight-worthy future boot

A stationary gyro condition must be enforced before arming and stale P0 must be rejected. Current v2 never arms. The controller is a bench core and should not be represented as a completed arming state machine.


---

<!-- BEGIN: 04_FIRMWARE/07_UART_BINARY_AND_XBOX_PROTOCOL.md -->

# UART communications and command framing

## Historical calibration stream

The full calibration recorder used **52-byte binary frames** near 500 Hz with raw MPU, QMC and BMP values, flags, timestamps/sequence information, and CRC validation. At 52×500 = 26,000 payload bytes/s, 8N1 line use is roughly 260 kbit/s, below a 921600-baud CH343 link's nominal bit rate. This does not prove Windows capture lacks buffering or resets; verify packet loss and CRC.

## v2 FC telemetry

The FC bench project emits predominantly **ASCII diagnostics at low rate** (`FC`, `FC_TIMING`, etc.), while the serial RX path parses Xbox messages. It is **not a drop-in replacement** for the full calibration GUI binary feed. Use the correct tool for whichever firmware is on the board.

## Xbox `$IQ2` command

`$IQ2,<seq>,<throttle>,<roll>,<pitch>,<yaw>,<vertical>,<buttons>*<4hexCRC16>` plus newline. CRC-16/CCITT computed on the payload after `$` up to `*`. The parser checks count, bounds, finite numbers, monotonic sequence and age. Historical `$IQ` accepted with vertical=0 compatibility. Command setpoint fields are normalized and need documented physical scaling in the FC.

## Serial sharing hazard

The v2 Xbox bridge and v2 terminal capture cannot both open the same COM port at once. The real Wi-Fi stack remains disabled, so there is no secondary network link for simultaneous operation. A serious next iteration may require separate control UART and diagnostics UART or an explicitly multiplexed protocol with priority and rate budget.

## Reset hazard

DTR/RTS behavior on CH343 can reset ESP32; a programming baud speed change is not a field control setting. Cold-power cycles should be logged and distinguished from reset-only events.


---

<!-- BEGIN: 04_FIRMWARE/08_POWERSHELL_GUI_AND_TOOLS.md -->

# Windows tooling, launchers, COM detection and known regressions

## Preferred user experience

Only use the scripts inside `tools/`, not a collection of root `START_*.ps1` files. For full sensor calibration: `tools/start_calibration.ps1`. For FC v2 bench: `tools/run_fc.ps1`. Both support auto-selecting the CH343 when visible, with optional `-Port COM3` override.

## Historical bugs worth remembering

| Problem | Cause/correct response |
|---|---|
| PowerShell “string is missing terminator” | Windows PowerShell 5.1 misinterpreted non-ASCII UTF-8 without BOM; use ASCII/BOM scripts |
| `-SkipLidar` interpreted as `GyroSamples` int | Bad positional forwarding; use named options or named splatting |
| `--upload-port requires an argument` | Launcher passed blank variable; resolve port *before* building command |
| GUI didn't offer LiDAR | Launcher forced skip when second COM unspecified; GUI now has skip control |
| `pio device monitor` refused redirection | It expects an interactive terminal; use pyserial logger when saving to file |
| COM3 vanished after cold boot | Windows USB re-enumeration/reset; detect again and don't pretend fixed COM number |
| USB serial busy/permission denied | Another process has exclusive COM handle; close it and use `finally` cleanup |

## How to debug

Confirm driver in Device Manager, enumerate `pyserial.tools.list_ports`, close monitors, use a sane reset/power-cycle procedure and log exact errors. Distinguish failure to open a serial port from a firmware sensor-read failure. A program that prints “upload success” but then zero frames is not equivalent to a broken magnetometer.


---

<!-- BEGIN: 04_FIRMWARE/09_DIAGNOSTIC_LOG_FIELD_REFERENCE.md -->

# FC v2 log fields — how to interpret them

Look at actual firmware `src/main.cpp` for canonical formatting. The following are the important fields, not a guarantee that your exact run has produced them.

## Startup

- `BOOT_BIAS`: stationary residual check, count, register status, means and SDs; must not blindly read `still=1` without checking motion.
- `BOOT_ATTITUDE`: roll/pitch initialized from measured gravity, yaw arbitrary.
- `BOOT_BARO`: pressure baseline and accepted count; `zero=1` indicates initial reference computed.
- `FC_BOOT`: sensor path/FIFO/I²C configuration.
- `FC_MODEL`: assumed mass and inertia, hover surrogate, PID parameters, `ESC=OFF`.

## Running `FC` line

- `hz`: observed loop rate over reporting window (not worst-case period).
- `imu`, `mag`, `bmp`: counts of measurement paths; compare with timestamps/fresh flags.
- `fifo`, `fifo_empty`, `fifo_lost`, `fifo_over`: health of 12-byte MPU buffer pipeline.
- `late`, `miss`: scheduler overruns and skipped deadlines.
- `parser_ok`, `parser_bad`, `link`: serial command parser and pilot freshness.
- `baro0`, `alt`, `vz`, `hsigma`: vertical reference, estimate and uncertainty.
- `roll`, `pitch`, `yaw`, `p`, `q`, `r`: estimated attitude/body rates; verify physical axes.
- `health`: bit mask of fault reasons; see `include/flight_core.hpp` enum.
- `virt`: four normalized virtual motor demands; **not GPIO output**.

## `FC_TIMING`

`imu_max_us`, `mag_max_us`, `bmp_max_us`, `loop_max_us`, `qmc_slow`, `bmp_identical`, `fifo_fail`, `baro_reject`, `lidar_ok`.

## Acceptance philosophy

An achieved average `hz=500` with a 5 ms `loop_max_us` is a failure of the strict 2 ms deadline objective. Health should be evaluated over long motion/cold-start traces, not a single first-second screen capture. Save complete logs under `results/` and record the exact build flags.


---

<!-- BEGIN: 04_FIRMWARE/10_REPOSITORY_AND_COLLABORATION_GUIDE.md -->

# How two AIs and the project owner should collaborate on firmware

## Branch and hardware ownership

- Only one process can own the ESP32's COM port or flash at once. Close GUI, monitor, pyserial recorder, Xbox bridge, and other IDE monitors before switching.
- Each agent works on a separate branch/worktree; avoid editing the same `src/main.cpp` without shared diffs.
- Preserve a tag/ZIP of successful 4 Nov sensor source and raw captures before any FIFO/filter refactor.

## Every change should include

`Commit/patch ID; source base; changed files; changed register values; build flags; local/sim tests; board power state; UART port/baud; hardware test; before/after metrics; remaining unknowns`.

## Review requirements

- Model changes: verify units, sign conventions, solver assumptions and physics dimensions.
- Calibration changes: verify source raw sessions, fit and holdout, hardware offset/matrix application order and actual register readbacks.
- Timing changes: show actual worst-case loop and I²C latency, not just a CPU benchmark.
- Controller changes: compare 7 gains, anti-windup, allocation saturation, step response and uncertainty; never assert stability from a single synthetic trajectory.
- Motor output changes: independent safety, power, failsafe, direction and thrust evidence. The present FC build must remain motor-disabled.

## Good working pattern

One AI owns a small, precise source patch; the other reviews tests and raw logs. Users choose which verified branch to flash. Documentation should remain updated whenever source changes, and the final build should print a version identifier/hash at boot.


---

<!-- BEGIN: 04_FIRMWARE/11_COMPILING_AND_TESTING_FC_V2.md -->

# Build and test the current FC v2 safely

## System requirements

Windows PowerShell 5.1+, Python 3, pyserial, optional pygame for Xbox, VS Code/PlatformIO (`espressif32@7.1.3`, `esp32-s3-devkitc-1`), USB CH343 driver and reliable USB power. The original project source is C++17 with modular headers.

## Safe recipe

1. Extract **v2** source to a new folder, no duplicate nested manual edits.
2. Confirm `IQ_MOTOR_OUTPUTS_ENABLE=0` and `IQ_WIFI_ENABLED=0` in `platformio.ini`.
3. Run `py -3 tests/test_project.py`, plus source/host tests documented in README.
4. Verify every sensor/ESC signal pin and remove propellers; disconnect ESC signal leads.
5. Run `.\tools\run_fc.ps1 -UploadFirmware` once; capture boot diagnostic and 30 seconds running.
6. Run `.\tools\run_fc.ps1` for repeats, and compare long-run timing.
7. Use `py -3 tools\xbox_uart_bridge.py --inspect` for input mapping before `-Xbox` virtual mode.

## What passing tests mean

Pure C++ numerical tests passed in prior authoring environment, including quaternion, Kalman, PID, CRC and compensation routines. Source syntax checks were performed off-device, not native physical ESP32 timing. PlatformIO compilation and board operation are still required. A successful capture is valuable data, not authorization to arm motors.

## If sensor regression appears

Reflash the preserved full calibration golden source and compare results on the same power state and wiring. If it works and FC v2 fails, the problem is in the new FIFO/clock/scheduling integration or its electrical interaction; don't immediately declare the QMC dead.


---

<!-- BEGIN: 05_ESTIMATION/01_COORDINATE_FRAMES.md -->

# Coordinate frames and physical interpretation

## v2 convention

Mathematics assumes right-handed **FLU** body axes: +X forward, +Y left, +Z up. World also uses +Z up. The unit quaternion `q` maps vectors from body to world. Euler angles are for display and outer small-angle loops, not the core integration state.

## Transform sequence

- `gyro_sensor` -> gyro hardware offset / measured residual -> deg/s -> rad/s -> MPU sensor-to-body rotation -> body p,q,r.
- `accel_sensor` -> m/s² -> software bias and 3×3 matrix -> MPU sensor-to-body rotation -> proper specific force.
- `mag_sensor` -> µT -> hard-iron offset and 3×3 soft-iron matrix -> MAG sensor-to-body rotation -> heading observation.
- LiDAR range frame -> aligned downward ray -> height correction after tilt compensation and reference offset.

## Source-level hazard

`Cal::RbodyImu` and `Cal::RbodyMag` are identity placeholders in `include/flight_core.hpp`. This is **not an asserted physical measurement**. A single swapped sign can turn a nominally stable PID loop into positive feedback.

## Coordinate checks

At rest level with +Z up, specific force should be approximately +g along body Z; manually roll/pitch/yaw and check the derivative sign of each reported Euler axis and body rate. Confirm `R_BS` orthonormality and determinant +1; display-only mirrored axes must never enter the control orientation matrix.


---

<!-- BEGIN: 05_ESTIMATION/02_QUATERNION_ATTITUDE_ESTIMATOR.md -->

# Quaternion attitude estimation

v2 stores a body-to-world unit quaternion `q=[w,x,y,z]`, advances it by corrected gyroscope angular velocity, and normalizes each integration. At the start of a stationary session, roll/pitch are initialized from the calibrated gravity vector; yaw remains arbitrary absent a validated magnetic reference.

## Propagation

\[
\dot q=\tfrac12 q\otimes [0,\omega_B],\quad
q_{k+1}=\operatorname{normalize}(q_k+\Delta t\dot q_k).
\]

For the 2 ms loop, rates must be in rad/s and the multiplication order must match body-to-world convention. The current first-order numerical integration is a pragmatic approximation; its error rises with rotation speed and irregular dt. Use recorded timestamps and physical axis tests.

## Correction

The source computes a small proportional correction from the cross product between normalized measured specific force and gravity predicted in the body frame. Acceleration correction is gated by near-1g norm (0.85–1.15g in source), to avoid incorrectly interpreting strong vehicle maneuvers as tilt. A slower magnetic yaw correction is applied only when fresh, plausible MAG data exists, and reference direction is initialized from an initial level-consistent field.

## Not an EKF

This is an integrated quaternion + geometric/complementary correction, **not** a quaternion extended Kalman filter. The altitude subsystem is a separate linear 3-state KF. If maintaining strictly linear control, quaternion geometry remains the kinematics/estimator layer; the control laws can still be cascaded linear PID near hover.

## Failure modes

Non-unit quaternion, clipped gyro, invalid accel orientation, sustained acceleration, magnetometer interference, timestamp gaps and unverified body axes. All require health gating; magnetometer should not be forced into the estimate when corrupted.


---

<!-- BEGIN: 05_ESTIMATION/03_MAG_YAW_OBSERVER.md -->

# Magnetometer heading observer and disturbances

v2 calibrates MAG in sensor coordinates and applies `RbodyMag`. MAG acceptance checks include fresh data age (<120 ms), finite vector and norm plausibility (roughly 15–90 µT in source). New measurements can correct yaw drift through a slow body-up-axis innovation.

## Relative yaw, not certified compass heading

The first accepted horizontal magnetic vector establishes a local world horizontal reference. This yields a relative yaw constraint; it does not automatically supply geographic north, declination or externally measured body heading. Correcting a reflected axis would create false yaw.

## Quality gates

- Check valid/overflow flags and internal QMC status `0x09`.
- Reject stale or implausible field magnitudes; include a coherent vector-direction innovation check in future revisions.
- Do not treat every 500 Hz loop as a new MAG observation when physical ODR is ~200 Hz.
- Compare yaw with gyro-only drift and manual orientation rotations in propeller-free tests.

## Motor magnetic interference

ESC/motor/battery currents induce fields that may swamp compass offset calibration. An ellipsoid fit with motors off does not predict powered heading quality. Future controlled tests should vary motor current under restraints and inspect MAG norm and direction; if untrustworthy, reduce yaw magnetic authority and use gyro yaw-rate control without pretending absolute compass hold.


---

<!-- BEGIN: 05_ESTIMATION/04_THREE_STATE_ALTITUDE_KALMAN.md -->

# Height, vertical speed and acceleration-bias Kalman filter

## State definition

\[
x=[z, v_z, b_a]^T
\]

where z is relative world-up height in meters, v_z vertical velocity m/s and b_a vertical accelerometer bias m/s².

Quaternion-rotated, gravity-compensated specific force is the input `u = (R_WB(q) a_body)_z - g`. For sample interval dt,

\[
F=\begin{bmatrix}1&dt&-dt^2/2\\0&1&-dt\\0&0&1\end{bmatrix},\quad
B=\begin{bmatrix}dt^2/2\\dt\\0\end{bmatrix},\quad
x_{k+1}=Fx_k+Bu.
\]

Covariance predicts via `P'=F P Fᵀ+B σ_a² Bᵀ + Q_bias`. The current code uses `sigmaAcceleration=0.6 m/s²` and `sigmaBiasRandomWalk=0.035` in an assumed process-noise representation; tune via residual statistics rather than laboratory fantasies.

## Measurement updates

- BMP: `z_baro = pressureAltitude(P;P0)`, measurement row H=[1,0,0], provisional operating SD 0.22 m.
- Optional LiDAR: relative ground range corrected by tilt and baseline, higher trust only with valid range/quality.
- Sequential updates on fresh arrivals; innovation gating approximately 5σ BMP and 4σ LiDAR.
- Joseph covariance update helps preserve positive numerical covariance and symmetry after correction.

## Observability limits

A single barometer gives height only relative to drifting pressure reference; dynamic pressure and temperature changes can mimic height. A bias-augmented state can reduce drift but cannot manufacture an absolute vertical reference. Near-ground LiDAR, once calibrated, is a valuable independent height observation. Without horizontal velocities/position sensors there is no XY hold.


---

<!-- BEGIN: 05_ESTIMATION/05_FUSION_R_Q_AND_CONFIDENCE.md -->

# Uncertainty-driven fusion: choosing Q, R and confidence gates

A Kalman filter needs physically meaningful process/measurement variance, not arbitrary extremely small R values that make the estimate look smooth. The 4 Nov BMP stationary relative-altitude SD was 0.0548 m, yet FC v2 uses **0.22 m** nominal dynamic measurement SD to allow for airflow and environment. Both numbers must be labeled with their test conditions.

## Recommended validation logic

1. Estimate sensor noise from *independent* stationary and dynamic captures, with timestamps and a documented preprocessing filter.
2. Track innovation `ν=y-Hx_pred` and `S=H P_pred Hᵀ+R` for each sensor.
3. Count NIS gate acceptances and rejections; investigate persistent one-sided residuals as bias/misalignment rather than increasing process noise forever.
4. Repeated cached BMP/QMC values cannot reduce covariance every 2 ms; update only for fresh conversions.
5. For LiDAR, adapt R using validity/signal or documented uncertainty only after confirming physical frame and range.

## Correlated errors

Vertical acceleration and quaternion tilt share the same IMU data; their errors are coupled. Treating them as fully independent overestimates certainty. Pressure drift is temporally correlated. The simple conditional linear KF in v2 is valuable but not a full vehicle navigation covariance system.

## Robustness

On MAG loss, preserve gyro-rate roll/pitch control and mark yaw heading untrusted; on BMP invalidity, prevent altitude-hold virtual demand from pretending a verified height; on stale pilot UART, invalidate commands. In the current bench build all motor demands remain virtual anyway.


---

<!-- BEGIN: 05_ESTIMATION/06_ESTIMATOR_FAULTS_AND_RESET.md -->

# Estimator faults, loss of reference and reset policy

The FC `Health` bitmask includes IMU missing, time invalid, gyro residual not accepted, accel bad, barometer not zeroed, pilot stale, attitude bad, model assumed, MAG unverified, clipped gyro/accel, stale pressure, model invalid, rejected Kalman measurements and LiDAR unverified.

## Propeller-free fault injections

- Disconnect MAG or force no fresh status: heading should go untrusted, while gyro/accel logging continues.
- Disconnect barometer: altitude reference becomes unhealthy; no false hold claim.
- Inject a large baro step or LiDAR outlier in host tests: NIS gate increments rejection counter.
- Skip IMU FIFO frames and simulate slow I²C: deadline/freshness counters must expose the problem.
- Feed invalid quaternion/NaN/pilot packets: control core should reset integrators and output zero virtual motors.

## Re-zero policy

Barometer zero is a reference change, not physical motion. It should be allowed only when disarmed, with fresh stable pressure, and must reset altitude KF state and height target consistently. The present source sets `setBaroZero()` and resets PID integrals; controller physical arming is permanently unavailable.

## Future failsafe criteria

Actual motor system needs dedicated state transitions and independent recovery/kill logic; resetting a software health flag is not sufficient. Never permit a transient “all sensors okay for one tick” to auto-arm or resume thrust.


---

<!-- BEGIN: 06_CONTROL/01_CASCADE_ARCHITECTURE.md -->

# Controller architecture — all seven PID loops

The v2 controller uses **cascaded linear PID near hover**, not LQR in parallel or an untested nonlinear motor actuation law.

## Inner and outer loops

- Roll angle PID -> requested body roll rate p* -> roll-rate PID -> roll torque τx.
- Pitch angle PID -> requested body pitch rate q* -> pitch-rate PID -> pitch torque τy.
- Xbox yaw stick -> yaw-rate request r* -> yaw-rate PID -> yaw torque τz. **No yaw angle hold PID.**
- Height PID -> vertical-speed correction v* -> vertical-speed PID -> commanded world-up acceleration a*.
- Collective feedforward: `T ≈ m(g+a*)/cos(tilt)`, then a virtual four-motor X mixer.

## Frequency

Each PID is calculated within the 500 Hz nominal main source loop using the actual dt, but its input may be derived from slower pressure measurements via prediction and fusion. The source does not claim every barometer conversion is 500 Hz.

## Safety and readiness

Virtual outputs only. `Controller::Model::clearedForFlight()` is hardcoded false. No physical ESC pulses/arming/validated mixer are present. The entire graph is useful for mathematical reasoning, bench logging and simulator tests, not free flight.


---

<!-- BEGIN: 06_CONTROL/02_PID_GAIN_TABLE_WITH_UNITS.md -->

# All 21 PID parameters — numerical table and units

**Source:** v2 `data/PID_TABLE_v2.md`; the rate gains are inertia-dependent candidates, the other gains are chosen design candidates. They are not flight-identified.

| Loop | Kp | Ki | Kd | Source of ±SD | Output quantity |
|---|---:|---:|---:|---|---|
| Roll angle | 0.650000 | 0.020000 | 0.030000 | design intervals: 0.15, 0.008, 0.010 | angular rate rad/s |
| Pitch angle | 0.650000 | 0.020000 | 0.030000 | same as roll | angular rate rad/s |
| Roll rate | 0.078253646 | 0.039126823 | 0.000058690 | 0.00356 / 0.00178 / 2.67e-6 | torque N·m |
| Pitch rate | 0.083149939 | 0.041574970 | 0.000062362 | 0.00362 / 0.00181 / 2.72e-6 | torque N·m |
| Yaw rate | 0.087423709 | 0.027607487 | 0.000115031 | 0.00378 / 0.00119 / 4.97e-6 | torque N·m |
| Relative height | 0.400000 | 0.005000 | 0.015000 | design intervals: 0.15 / 0.003 / 0.007 | vertical speed m/s |
| Vertical speed | 1.000000 | 0.060000 | 0.025000 | design intervals: 0.30 / 0.020 / 0.010 | acceleration m/s² |

## Read the units first

The PID implementation integrates `error*dt` in SI time and filters derivative of measurement. Thus gain dimensions differ between angle, rate and vertical loops. An arbitrary number copied into an ESC PWM library would be wrong by dimensional analysis. The modeled torque requests need a calibrated thrust/moment allocator before any physical actuation.

## Statistical limits

Rate gain 1σ values are propagated from **assumed** J uncertainty (nominal J scales gains linearly). Other ±SD ranges are design ranges and do not represent frequentist confidence intervals. Torque authority, motor lag and saturation alter the physically optimal gains; their true distributions have not been measured.


---

<!-- BEGIN: 06_CONTROL/03_RATE_PID_POLE_PLACEMENT.md -->

# Rate PID gains from inertia and pole-placement intuition

Near hover and for one axis with negligible cross coupling, the rotational plant is `J_i dω_i/dt = τ_i`. The preliminary PI rule chooses a second-order closed-loop shape:

\[
K_{p,i}=2\zeta\omega_n J_{ii},\qquad
K_{i,i}=\omega_n^2 J_{ii},\qquad
K_{d,i}=0.006\,J_{ii}.
\]

Default roll/pitch `(ωn,ζ)=(2 rad/s,2)`, yaw `(1.2 rad/s,1.9)`. The Kd term is intentionally small, with derivative on measured rate, and is not derived from this pure PI pole placement. The code filters derivative before using it.

## Worked roll example

With `Jxx≈0.009781 kg·m²`: Kp=2×2×2×0.009781≈0.07825; Ki=2²×0.009781≈0.03912; Kd=0.006×0.009781≈5.87×10⁻⁵. These output modeled torque (N·m) for errors in rad/s, not normalized motor signal.

## Model uncertainty

If J is uncertain by σ_J and the gain formula is linear in J, first-order gain SD is `σ_K≈K(σ_J/J)`. Correlated J entries, rotor lag, unmeasured thrust curves and actuator limits add systematic uncertainty *outside* this simple propagation.

## Actuator lag and stability

With a first-order motor lag τm, an ideal continuous unfiltered PID characteristic can be written `(τmJ)s³+(J+Kd)s²+Kp s+Ki=0`, with Routh-Hurwitz condition `(J+Kd)Kp > τmJ Ki` for positive coefficients. That check does not certify the real discrete filtered PID, delay, quantized ESC command or saturated mixer. An experimental step response is required.


---

<!-- BEGIN: 06_CONTROL/04_ANGLE_AND_YAW_LOOPS.md -->

# Roll/pitch angle and yaw-rate control

## Angle loops

For roll and pitch, the outer PID compares an angle request with quaternion-derived Euler roll/pitch, then requests a body rate. These gains are substantially lower-bandwidth than the inner rate loops. FC v2 clamps commanded roll/pitch to approximately **±15°** during virtual bench operation and outer rate request to configured bounds.

The `PID.step(..., derivativeInput=gyro_axis)` pattern uses measured angular rate as the derivative channel for angle controllers, avoiding derivative kick when setpoint jumps. Confirm gyro and Euler signs match FLU conventions before interpreting PID feedback as negative feedback.

## Yaw

The Xbox left-X input maps to yaw-rate request, clamped in v2 near **±1.4 rad/s**. There is no yaw-angle PID, which avoids imposing an absolute heading when MAG mounting/ref is unverified. Slow magnetic yaw correction is in the estimator only; if MAG trust fails, yaw angle drifts but rate control can still be evaluated in a safe simulator.

## Required experiments

With motors disabled, hand-rotate aircraft while holding roll/pitch setpoints at zero; verify virtual torque tends to oppose measured motion in all axes. Cross-axis responses must not change sign. Only after physical axis mapping and actuator measurement can gains be related to actual rotor thrust.


---

<!-- BEGIN: 06_CONTROL/05_ALTITUDE_AND_VERTICAL_SPEED_PID.md -->

# Height hold and vertical-speed control — control strategy and limitations

## Reference generation in v2

The controller integrates commanded vertical speed (Xbox left Y) to update relative `heightTarget`; a centered stick requests zero v_z with height correction. The source uses a dead zone (~0.07), normal max speed request about ±0.6 m/s, and height-target clamp about −2 to +8 m in the **virtual** test model.

## Two PID stages

- Height error `z*−z_hat` goes to **height PID** `(0.4,0.005,0.015)` and produces a vertical-speed correction.
- Vertical-speed error `v*−v_hat` goes to **vertical PID** `(1.0,0.06,0.025)` and produces vertical acceleration in m/s².
- Thrust feedforward is `m(g+a*) / cos(tilt)` with lower cos clamp; converted through assumed T_max to virtual motor fractions.

The inner altitude controller is only as reliable as its vertical-speed estimate; differentiating barometer pressure naïvely amplifies noise, so v2 uses the Kalman state `v_z` predicted by accelerometer and corrected by fresh pressure/range observations.

## Why true hover remains difficult

BMP280 pressure responds to atmospheric changes, dynamic pressure from motion and propwash; attitude errors leak horizontal acceleration into vertical dynamics. LiDAR can help near ground but was not in the successful calibration and the current ESP32 RX pin is unverified. No absolute height/position guarantee is possible from BMP alone. Hover with actual motors is not demonstrated.


---

<!-- BEGIN: 06_CONTROL/06_PID_ANTI_WINDUP_AND_D_FILTER.md -->

# PID implementation details — discrete integrator, derivative and saturation

The v2 `include/pid.hpp` implements:

\[
u=K_p e+K_i\int e\,dt-K_d D_{filtered}(y).
\]

The derivative is **on measurement** rather than error; this avoids setpoint derivative spikes. The derivative state uses an exponential first-order low-pass with `alpha=exp(-2π f_D dt)`. Rates use 32 Hz derivative filter, angles 15 Hz, height 6 Hz, vertical 8 Hz.

## Integral handling

The controller clamps integral state to an `iCap`, clamps output to `outputLimit`, and conditionally cancels the most recent integration step if it would drive an already saturated output further into saturation. The controller resets integrators when the model/state/pilot is not ready, and on virtual motor mixer saturation.

## Why nonzero Kd is not automatically useful

A small Kd satisfies the user's request to track all three gains, but a noisy, delayed derivative can destabilize real motors. MPU DLPF already introduces delay; cascading further derivative filters changes phase margin. The real optimum depends on measured actuator lag and vibration spectrum. Source gains are initial **simulation-only** candidates.

## Good logging

Record each loop's setpoint, measurement, proportional term, integral state, derivative state, clamp flags, dt and output. Current v2 telemetry prints only condensed virtual mixer/health values, so deeper PID traces require a controlled logging extension that does not block the 2 ms loop.


---

<!-- BEGIN: 06_CONTROL/07_VIRTUAL_X_MIXER.md -->

# X mixer: torque and thrust to four **virtual** motor fractions

The v2 mixer treats four motor thrusts as an idealized linear actuator. With mass m, gravity g, desired vertical acceleration a_z, nominal diagonal span D and tilt θ, it requests collective:

\[
T_{sum}=\frac{m(g+a_z)}{\cos(tilt)}.
\]

Define `arm=D/(2sqrt(2))`, `base=Tsum/(4 Tmax)`, `roll=τx/(4 arm Tmax)`, `pitch=τy/(4 arm Tmax)`, `yaw=τz/(4 κ Tmax)`. For source-annotated motor order FL,FR,RR,RL:

```text
FL = base + roll - pitch + yaw
FR = base - roll - pitch - yaw
RR = base - roll + pitch + yaw
RL = base + roll + pitch - yaw
```

Fractions are clamped to 0..1. Source flags saturation and resets controller integrals. **These are unverified signs and motor order**, not calibrated ESC outputs; they are intentionally never written to GPIO.

## Why simple clamping is not full allocation

Independent clipping changes requested torques and may create large yaw/roll errors. Proper flight-grade allocation considers available thrust reserve, total thrust priority, constraints, actuator asymmetry, and saturation-aware integrator behavior. It must be validated with motor directions, real commanded thrust curves and an independent kill path.


---

<!-- BEGIN: 06_CONTROL/08_MASS_INERTIA_AND_DYNAMICS.md -->

# Rigid-body dynamics from Lagrangian model to a PID plant

For body angular velocity ω and center-of-mass inertia matrix J, rigid-body dynamics are:

\[
J\dot\omega =\tau - \omega\times(J\omega).
\]

For a small-angle, low-rate hover approximation with J diagonal and little gyroscopic coupling, each rate channel behaves approximately `Jii ωdot_i = τ_i`, motivating a simpler per-axis rate PID. The full Euler/Lagrange formulation in the Claude study is useful for understanding what was neglected and when the approximation fails.

## Assumption-based profile A

`m≈1.104 kg`, `J≈diag(0.009781,0.010390,0.019170) kg·m²`, with nonzero uncertainty. Jx and Jy differ because the geometrical model is not exactly symmetric. Actual off-diagonal components depend on battery, ESCs, mounting and wires, and should not be silently forced zero in final physical calculations.

## Unmodeled effects

Motor reaction torque, rotor gyroscopic moments, thrust dynamics, propeller aerodynamics, arm flexibility, vibration, battery sag, actuator dead zone and airflow interaction. They are not captured by the single double-integrator approximation. The mass model and PID derivation establish a *starting hypothesis* for bench identification, not a proof of hover.


---

<!-- BEGIN: 06_CONTROL/09_MONTE_CARLO_AND_ERROR_PROPAGATION.md -->

# Monte Carlo mass/inertia/PID uncertainty — correct interpretation

FC v2 provides a 100,000-draw study using assumed distributions for mass, inertia, motor diagonal, motor maximum thrust, yaw leverage and thrust lag. Outputs include means, SDs, percentiles for gains and surrogate hover fractions, and idealized stability checks.

## Main modeled inputs

| Parameter | Basic prior center | Prior 1σ |
|---|---:|---:|
| Mass m | 1.104 kg | 0.0506 kg |
| Jxx | 0.009781 kg·m² | 0.000446 |
| Jyy | 0.010390 kg·m² | 0.000454 |
| Jzz | 0.019170 kg·m² | 0.000831 |
| Tmax per motor | 6 N | 1.5 N |
| Motor lag | 0.040 s | 0.015 s |
| Motor diagonal | 0.450 m | 0.010 m |

## Results to interpret carefully

The modeled median hover normalized fraction is around **45.2%** with the assumed 6 N thrust; a significant fraction of cases exceed a historical 45% cap. This is evidence of an **uncertain or possibly infeasible assumed operating point**; not evidence of real throttle position.

## Improvements needed for scientific credibility

Obtain measured component mass distributions, measured motor force curves and delay, estimate covariance between inertia components, and use physically consistent correlated variations in Monte Carlo. Report conditional distributions, not “95% aircraft confidence” when the input priors are guesses. Propagate calibration parameter covariance into attitude/altitude filter performance as well as torque gains.


---

<!-- BEGIN: 06_CONTROL/10_RATE_RESPONSE_SIMULATIONS.md -->

# Synthetic rate-step response and PID tuning workflow

Earlier idealized control simulations found a higher-bandwidth PI setting generated roughly 35% overshoot under an **assumed 40 ms motor lag**; a slower conservative setting generated around 10% in that model. FC v2's conservative candidate was evaluated across additional synthetic 500 Hz plants and reported ~6% median overshoot in one sampling of assumed plants.

These results cannot be compared with actual step data until rotor/ESC response and thrust conversion are measured. Overshoot percentage alone is insufficient: rise time, settling time, steady-state error, saturation, peak torque, filtering delay, stability margins, noise sensitivity and battery variation all matter.

## Proposed rate-tuning sequence

1. Keep physical ESC disconnected and test discrete PID math with a known synthetic linear plant.
2. Measure actual rotor lag, thrust/torque leverage and max thrust independently.
3. Update the simulation with motor saturation, dead zone, sample jitter and measured noise.
4. Analyze robust margins over uncertainty intervals; verify each axis sign.
5. Only with independent safety implementation and restrained checks, move to hardware actuation testing.

The user requested a learning-oriented PID design: retain clear units and every Kp/Ki/Kd term; avoid silently replacing PID with LQR before the controller architecture is understood and verified.


---

<!-- BEGIN: 06_CONTROL/11_XBOX_COMMANDS_AND_ALTITUDE_CHORD.md -->

# Xbox commands, authority and altitude re-zero gesture

## Virtual bench command mapping

Left X = yaw angular rate; left Y = requested vertical speed; right X/Y = roll and pitch angles. The v2 source clamps roll/pitch to ±15° and yaw rate to about ±1.4 rad/s. Height target is integrated from normalized vertical stick with an approximately ±0.6 m/s setpoint.

The bridge uses serial `$IQ2` CRC frames, validates sequence and normalized ranges, and a 200 ms timeout rejects stale commands. Direct BLE pairing and a verified real receiver are not part of this v2 evidence.

## Requested button combination

D-pad **Up + LB + RB** requests BMP280 relative-altitude re-zero **only while disarmed**. The source accepts the chord once per press and only when recent pressure data is available; it resets altitude Kalman, target and PID integrators. This is a reference/estimation operation, not physical battery/ESC arming.

## Important competing branch conventions

`esp32-xbox` previously assigned D-pad Down + A/B to flight/navigation mode selection and D-pad Down + LT for a held disarm. The current UART bench parser is independent. A future merged controller needs one authoritative input map, hard gesture precedence and a lost-link policy that has been tested with the real receiver and motors restrained.


---

<!-- BEGIN: 07_TESTING/01_BENCH_TEST_ORDER.md -->

# Hardware test sequence — safe and reproducible

This plan establishes evidence stepwise. **It does not authorize installation of propellers or a free-flight attempt.**

## Phase 0: physical safety

Disconnect ESC signal wires, remove propellers, verify power rails, I²C pull-ups, UART ground and sensors' orientation. Retain motor-off compile-time guard. Only one serial reader/flash process at a time.

## Phase 1: golden sensor regression

Run preserved 4 Nov calibration source as a baseline: MPU valid, QMC fresh at profile `0x3D`, BMP valid, UART packet losses/CRC minimal. This gives a reference when new FIFO/400k versions fail. Record actual power state.

## Phase 2: FC v2 boot

Build `iq-fc-safe` with UART 921600 and Wi-Fi 0. Confirm MPU offset register readbacks, gyro post-write residual, accelerometer norm, MAG ID/status, pressure boot P0 and FIFO enabled. Confirm source boot prints `ESC=OFF`.

## Phase 3: 500 Hz timing

Collect 10 minutes of `FC` and `FC_TIMING` without blocking stdout. Record real `loop_max_us`, missed periods, FIFO losses/overflow, QMC fallback and fresh rates under board motion. Compare 100k/400k only under controlled changes.

## Phase 4: estimator hand tests

Hold level; pitch, roll, yaw in known directions; test quaternion signs and MAG behavior. Raise/lower board against ruler; evaluate baro altitude drift and KF innovations. Do not trust output signs from the GUI alone.

## Phase 5: virtual control and Xbox

Test gamepad axis dead zones/scale, D-pad Up+LB+RB re-zero disarmed, 200 ms link-loss, valid CRC and sequence rejection, virtual torque response and anti-windup. ESC GPIO remains inactive.

## Phase 6: physical propulsion feasibility (future, separate)

Weigh final vehicle, identify motors/props/ESC mapping, measure thrust with guarded independent test fixture, implement/test arming and kill path before contemplating controlled propulsion. Recompute gains based on measured actuator model.


---

<!-- BEGIN: 07_TESTING/02_ACCEPTANCE_METRICS_AND_LOGS.md -->

# What counts as success — measurable acceptance criteria

| Aspect | Example metric | Acceptance status now |
|---|---|---|
| UART binary calibration | CRC errors, sequence gaps, packet rate | 4 Nov reported successful capture; 500 Hz target observed |
| QMC real output | Fresh XYZ/sec, ID, status, range, repeatability | ~195 Hz in successful Nov 4 trace; earlier intermittent failures |
| MPU rate | FIFO actual frames/sec, overflow and backlog | FC v2 source implemented; on-device FIFO trace still missing |
| 500 Hz scheduler | Max/P99 loop time, missed deadline count | No real FC v2 trace yet; acceptance must be set before actuation |
| Gyro calibration | Stationary mean and SD after hardware offset | Raw 30k calibration done; new register path not yet tested |
| Attitude | Physically measured sign, roll/pitch transient, drift | Not independently signed-off |
| Barometric height | Drift, step delay, ruler comparison, NIS | Stationary SD measured; dynamic uncertainty unverified |
| Xbox | Command CRC, disarm/loss policy, failsafe latency | Bridge host tests only, no flight receiver validation |
| Motors | Thrust map, lag, current, order, direction, stop | Not measured; physical outputs disabled |

## How to compare two firmware revisions

Hold sensor placement/power/temperature and logging method constant. Change one configuration at a time (e.g. I²C clock). Log full distributions, not a single average. Note that a short test with zero failures is a weak claim about rare tail faults.

## Recommended report summary

`commit, firmware SHA, board, timestamp, power state, I2C Hz, UART baud, MAG C1, MPU DLPF/FIFO, BMP profile, boot offsets, average/p99/max loop us, missed/overflows, sensor fresh Hz, UART loss, anomalies, verdict, next action`.


---

<!-- BEGIN: 07_TESTING/03_FAULT_INJECTION_TEST_MATRIX.md -->

# Fault injection matrix (safe, motor-off)

| Injected condition | Expected response | What to record |
|---|---|---|
| No magnetometer ACK / frozen data | MAG invalid; no forced yaw correction | Age, error count, gyro-only yaw trend |
| QMC read error after config | Bounded retry/fallback, IMU path maintained | Register readback, Wire error, loop duration |
| BMP disconnected or invalid | Height untrusted, altitude PID virtual demands not accepted | `baro0`, `H_BARO_STALE`, KF state |
| Accel saturated | Flag H_ACCEL_CLIPPED, suspend affected estimate/control | Range, raw count, attitude and health |
| Gyro clipped | Flag H_GYRO_CLIPPED; no false precision | Raw p/q/r, integrator reset |
| MPU FIFO overflow | Record FIFO loss and reset boundedly | Frames dropped, loop stalls, sensor age |
| 5–10 ms synthetic task stall | Report missed deadline; no silent time-series compression | FC_TIMING, late/miss |
| Xbox unplugged / serial stopped | Pilot invalid after timeout, virtual outputs zero | link age, parser counters |
| Corrupted/duplicate `$IQ2` packet | CRC/sequence reject | accepted/rejected counters |
| Press altitude-zero chord while stale pressure | Do not accept zero | resetAccepted/Deferred and pressure age |
| Bad mass/inertia matrix | Reject invalid model | `H_MODEL_INVALID`, zero virtual motor |

The present software fault injection is not a substitute for a verified independent physical motor-cutoff architecture. No motor actuation should be enabled to “try” the failure tests.


---

<!-- BEGIN: 07_TESTING/04_HISTORIC_FAILURES_AND_FIXES.md -->

# Failure history and recovered engineering lessons

## Magnetometer

v13 100 Hz `0xC9` occasionally worked ~90 Hz, other settings failed after configuration. v14–v18 attempted read modes, power-cycle control, Wi-Fi stress, bus loading and diagnostics; all tested combinations were not uniformly healthy. A later v13 test proved the chip could still output at ~90 Hz. Transplanting the original driver into full code was followed by the successful 4 Nov capture using **`0x3D` ~195 Hz**. This supports a configuration/interference sensitivity and successful integrated operation, not a completely determined root cause.

## Serial toolchain

PowerShell UTF-8 em dash parser error; malformed argument forwarding `-SkipLidar` into integer; empty `--upload-port`; COM port changing after USB reset; “port unavailable after 60 s”; COM busy, pyserial exclusive handle and Windows CH343 driver; `pio device monitor` incompatible with redirected stdin/stdout. Every error requires a targeted workflow fix, not a sensor reflash.

## Calibration math

Original full-ellipsoid MAG fit permitted a trivial solution and saved the wrong matrix interpretation. Original corrected plot collapsed due to multiplying by an ellipsoid equation matrix. Independent fit used all legitimate low-norm samples and validated on held-out rotations. A GUI axis reflection was caught and must not be copied into physical orientation correction.

## Modeling

Initial Claude book used HMC5883L as a described sensor and an illustrative 1.2 kg/6 N motor plant; real measured MAG is QMC5883P and actual aircraft mass/propulsion remain unknown. Earlier synthetic PI design produced excessive overshoot under assumed motor lag. These are model-review issues, not evidence of a flying system.


---

<!-- BEGIN: 07_TESTING/05_CALIBRATION_VALIDATION_EXPERIMENTS.md -->

# Independent sensor-calibration validation experiments

## Gyro

Repeat bias at cold/warm conditions, measure time-dependent bias, and estimate Allan variance when enough stationary records exist. Read hardware offset registers and verify residual means. Exclude obvious movement using accelerometer norm and gyro variance. Do not assume first few boot samples represent fully warmed sensor.

## Accelerometer

Use precisely oriented six faces if possible and independent known tilts. Compare raw 3D values and corrected magnitude `|a|≈g` for all six poses. Cross-axis matrix must be tested against orientations not used to fit; otherwise fit may absorb placement error. Validate after any ±2g range change.

## Magnetometer

Collect separate full 3D rotations after remounting, compare corrected norm distribution and held-out angle/heading, and verify direction mapping. Compare motor power off versus controlled powered conditions once safety architecture supports guarded motor tests. Check field magnitude trend for EMI.

## Barometer

Compare pressure-derived relative height at actual ruler-controlled positions, in still air, then near gentle flow/temperature variation without propulsion. Log sample age, IIR response and zero-reference drift. Confirm the disarmed D-pad reset works only once per chord.

## LiDAR

When connected, known 25/50/100/200 cm distances, varying reflectivity and tilt; measure status/variance and fusion innovation. No accepted 4 Nov LiDAR samples means no claim of successful calibration today.


---

<!-- BEGIN: 07_TESTING/06_MOTOR_THRUST_IDENTIFICATION_PLAN.md -->

# Propulsion system identification — prerequisite for real flight gains

Current rate gains are torque-domain and depend on **unmeasured** maximum rotor thrust, thrust-command curve, time constant, yaw reaction torque, mass and inertia. Real hardware identification must precede any claim that PIDs or hover commands are “best.”

## Controlled lab setup

Use a secure thrust fixture, current/voltage measurement, guarded propeller enclosure, calibrated scale, fire-safe LiPo practices, a physical disconnect/kill and a competent operator. Document true propeller dimension/pitch and spin direction. This is a separate, supervised actuation test—not a suggested unrestrained bench spin.

## Measurements

- `command` vs stable thrust `T_i`, supply voltage and current per motor.
- Dead zone, startup threshold, hysteresis, command cap and response dynamics to small safe command steps.
- Differences between the four motors/ESCs and effect of pack voltage sag.
- Real reaction yaw torque coefficient or conservative identifiable bounds.
- Heating and limits before continuous operation.

## Model update

Fit an appropriate monotone thrust surrogate and motor response model with residual/uncertainty estimates; do not assume `T=Tmax*u`. Update mass from scale, arm coordinates from ruler, motor map/rotation from physical inspection. Recompute closed-loop stability margins and feasible hover setpoint with correction reserve.

## Safety rule

This plan does not override the v2 motor-off guard. Independent arming, receiver-loss stop and physical cut-off remain required before a vehicle-wide motor test.


---

<!-- BEGIN: 07_TESTING/07_SAFETY_AND_FLIGHT_READINESS.md -->

# Flight readiness gate — what is currently missing

**As of this documentation snapshot, no validated free-flight capable controller has been shown.** A source with PID, a good calibration and a simulated thrust mixer is not a flight-ready firmware.

## Required evidence before motor-output firmware is even considered

- Verified physical ESP32 pins, ground, power sequencing and supply robustness under electrical load.
- Verified body/IMU/MAG axes and gravity orientation (det+1 transforms), gyro signs and mixer signs.
- Actual motor order, rotation direction, propeller pairing and ESC control interface.
- Mass, COM and feasible hover thrust with meaningful motor correction margin at actual pack voltage.
- Validated arming state machine, independent physical kill/cut-off, tested RC loss/stale command behavior, safe startup and bounded throttle.
- 500 Hz worst-case timing under representative load, sensor freshness, watchdog, integrator anti-windup and fault response.
- Altitude estimator dynamic validation; no claims of position hold without XY sensing.
- Controlled, restrained and propeller-free testing protocol; actual propulsion tests only in appropriate protective equipment/test facilities.

## Explicitly blocked now

`IQ_MOTOR_OUTPUTS_ENABLE=0`; `clearedForFlight()` always false. Do not remove these gates in the name of speed, competition timing or apparently stable simulated step responses. A future motor-capable project should be separately reviewed and tested.


---

<!-- BEGIN: 08_RESEARCH/01_DYNAMICS_AND_STATE_SPACE.md -->

# Mathematical foundations — vehicle equations and linearization

## Full rigid-body idea

Position, velocity, attitude and angular rate are the vehicle states. In inertial coordinates the force sum governs translation, gravity subtracts from total thrust vector, and `J omega_dot = tau - omega×Jomega` governs rotation. Lagrangian mechanics reproduces these physical laws and helps track energy, mass properties and generalized forces.

## Hover-linear models

For small angles, yaw reference fixed, and ideal rotor dynamics, angular rate dynamics are roughly a torque-controlled integrator. The angle dynamics add another integrator; hence cascade angle->rate PID. Vertical motion around hover can be linearized as `zdot=vz`, `vzdot ≈ T/m -g`; thrust control must compensate mass and tilt. This is valid locally, not for arbitrary acrobatics or saturated motors.

## Where state space helps

Even with PID as the controller, state-space models are useful for the altitude KF, simulated plant `x_{k+1}=Ax_k+Bu_k`, observability and control authority analysis, and MIMO coupling audit. Full LQR should not silently replace a comprehensible PID that the owner is learning; it can be an offline comparator after real plant identification.

## Source

User-supplied **IntelQCopter Volume I Edition 2** studies Lagrangian equations, linearization, Laplace, transfer functions, state-space, PID, LQR and Kalman. Its HMC sensor label and illustrative mass/motor assumptions should be corrected in future revisions **only with measurement evidence**.


---

<!-- BEGIN: 08_RESEARCH/02_PRIMARY_DATASHEETS_AND_PAPERS.md -->

# Primary references, not unverified blog summaries

## Sensor hardware

1. MPU-6000/6050 Register Map Rev 4.2, timing, DLPF, FIFO, offsets and aux bypass: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf>.
2. BMP280 Datasheet Rev 1.26, timing, normal mode and factory compensation: <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf>.
3. QST QMC5883P Rev E, chip ID and control/status registers: <https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf>.
4. Betaflight `compass_qmc5883` driver for independent register comparison: <https://github.com/betaflight/betaflight/blob/master/src/main/drivers/compass/compass_qmc5883.c>.
5. Nooploop TOFSense F2 Mini Datasheet and Frame0 parser: <https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf>, <https://github.com/nooploop-dev/nlink_unpack/blob/master/nlink_tofsense_frame0.c>.

## State estimation and flight control

6. Mahony, Hamel & Pflimlin, *Non-linear Complementary Filters on SO(3)* (2008): <https://openresearch-repository.anu.edu.au/items/db91f02d-9a17-4ead-bc64-86ed5bc9c6f4>.
7. Sabatini et al. (2014), inertial/barometric altitude/vertical-velocity fusion: <https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/>.
8. MIT 2.003SC dynamics, MIT 2.14/16.30 feedback, MIT 6.041SC probability, MIT 16.333 aircraft control. Useful educational references, *not* specific board validation.
9. Bluepad32 FAQ on ESP32-S3 BLE-only host and Xbox firmware compatibility: <https://bluepad32.readthedocs.io/en/latest/FAQ/>.

## Reading guideline

Prefer silicon manufacturer documents for register timing/sensitivity, raw captures for what the actual PCB did, peer-reviewed filtering/control papers for methods, and host scripts for reproducibility. Distinguish empirical board behavior from a theoretically permitted sensor mode.


---

<!-- BEGIN: 08_RESEARCH/03_ROBUSTNESS_AND_UNCERTAINTY_METHOD.md -->

# Robustness, statistics and what an error bar means

A physical drone model is not accurate merely because it has many decimal places. This project has excellent repeated sensor data but unmeasured propulsion and some uncertain geometry. Use transparent uncertainties rather than masking gaps.

## Suggested hierarchy

- **Type A:** estimate noise/variance and covariance from repeated measurements, accounting for temporal correlation, Allan slope and between-session changes.
- **Type B:** manufacturer tolerances, ruler/scale resolution, fixture orientation and sensor mounting error.
- **Structural model error:** missing motor lag, nonideal mixer, time delay, power droop, thrust curve and propwash, which cannot be encoded by just smaller Gaussian σ.
- **Empirical verification:** new independent measurements at relevant operating conditions.

## Model propagation

For function y=f(x), local first-order covariance is `Σ_y≈J_f Σ_x J_fᵀ`. For strong nonlinearities (hover ∝m/Tmax), Monte Carlo over bounded physically plausible priors is more useful; retain correlations and report rejected infeasible models. A 100,000-draw result remains a *conditional* distribution if input variances are assumed.

## Model validation

Use calibration holdout sessions; do not train on the same identical samples used to judge performance. Evaluate estimator NIS/NEES if ground truth exists, rate-step transient vs measured motor plant, and worst-case loop latency under stress. State uncertainty in units and never claim sub-centimeter hover solely from stationary barometer SD.


---

<!-- BEGIN: 08_RESEARCH/04_DESIGN_TRADEOFFS_AND_FUTURE_HARDWARE.md -->

# Performance tradeoffs and future design options

## MPU and bus

An MPU6050 is usable for this bench but is limited by shared I²C bandwidth and register-level FIFO behavior. A future SPI IMU with data-ready and hardware timestamps could reduce bus contention and measurement age, if the board is redesigned and tests prove the improvement. Do not redesign hardware before measuring the present `FC_TIMING` bottlenecks.

## Magnetometer

The QMC5883P can be genuinely fresh near 200 Hz in one demonstrated capture, but its board-specific I²C mode failures make it a higher risk than raw datasheet ODR suggests. A magnetometer on a short, remote, shielded mounting location might reduce motor field interference; actual location and current response need measurement.

## Altitude

Fast BMP normal-mode ×1/×1 with IIR×4 can give roughly 166 Hz, but true vertical accuracy depends much more on flow shielding, gyro attitude, acceleration-bias tracking and ground range reference. Optional F2 LiDAR is valuable near ground when verified. Better barometric processing is not equivalent to 200 independent pressure samples/s.

## Communication

921600 CH343 binary capture is adequate for historical 52-byte/500 Hz telemetry under good host conditions, but Windows control/diagnostic tools contend for one COM port. Future flight firmware needs separate reliable RC/radio authority, failsafe and telemetry priorities; Xbox USB bridge is lab tooling.

## Controller architecture

Keep cascade PID as the first learning/validation target; use quaternion geometry and conditional linear altitude KF. Consider model feedforward, LQR, system identification and extended estimators only after physical plant and noise measurements justify the complexity.


---

<!-- BEGIN: 09_ROADMAP/01_IMMEDIATE_PRIORITIES.md -->

# Priority roadmap — immediate engineering work after Nov 5 snapshot

## P0: preserve the golden baseline

Make immutable backups of the successful 4 Nov full sensor source/archive, raw CSV, calibration candidate JSON, and the v2 FC source. Record checksums, actual `0x3D` MAG profile and 921600 UART. Do not overwrite the baseline with a failed FIFO experiment.

## P1: verify FC v2 on real ESP32 with motors OFF

Run PlatformIO real build, upload, log 30 seconds and then a longer 10-minute run. Check boot offset write/readback and residual bias, true 500 Hz loop timing, FIFO lost/overflow, MAG rate, BMP rate, I²C fallback and UART. This is the highest-yield next task.

## P2: verify geometry and estimator

Physically survey sensor orientations, body axes and static acceleration sign. Confirm calibrated gravity at multiple known poses, MAG yaw trend, pressure height at known ruler heights and re-zero chord. Keep motor outputs disabled.

## P3: identify propulsion and safety

Weigh actual assembled drone; physically map motors/ESC; establish independently safe guarded thrust/lag measurement and voltage/current monitoring. Implement reviewed, verified arming, independent cut-off and lost-link behavior in a **separate** hardware-test phase.

## P4: add LiDAR after physical pin validation

Calibrate with independent ruler 25/50/100/200 cm, verify 16-byte frames/checksum and update rate, fit uncertainty, and re-evaluate altitude Kalman under static and dynamic tests. No claiming near-ground hover improvement until observed.

## P5: autonomy and companion PC

Only after primary stabilization is flight-verified; ROS2/YOLO loads must never preempt time-critical FC control. Preserve minimal hardware and clear failure authority.


---

<!-- BEGIN: 09_ROADMAP/02_OPEN_QUESTIONS.md -->

# Outstanding unknowns — answer with measurements, not guesses

1. What is the actual weighed ready-to-fly mass, battery model/mass and complete propulsion inventory?
2. What propeller diameter, pitch and handedness are physically installed?
3. What are the exact motor GPIO -> ESC -> arm positions and rotation directions?
4. What are the measured per-motor thrust curves, yaw torque leverage and lag with the 3S pack?
5. Which physical MPU and QMC XYZ directions correspond to body FLU, verified via hand rotations?
6. Does MPU hardware-offset readback match its predicted scaling and preserve factory compensation bits on this exact chip?
7. Can FC v2 sustain a real 500 Hz loop at 400 kHz I²C with near-zero overruns over extended operation?
8. How often does QMC fall back to 100 kHz and is it stable after warm/cold starts and representative electrical load?
9. How does BMP static/dynamic pressure noise change under enclosed mounting, airflow, temperature and actual propwash?
10. Which ESP32 RX pin, logic voltage and UART port are connected to the TOFSense F2 Mini?
11. Is the Xbox controller's BLE firmware compatible with direct ESP32-S3, and what independent kill/failsafe exists?
12. Can the proposed N150 hardware actually run the planned ROS2/vision software within its RAM/power budget?
13. Are the mass/inertia model coordinate origin, LiDAR offset and URDF base_link definitions consistent?
14. What flight-test acceptance criteria and safety review sign-off process will be required?

Document dates and evidence when answering. A guess from a product listing should remain marked as an assumption.


---

<!-- BEGIN: 09_ROADMAP/03_TECHNICAL_DEBT_AND_KNOWN_GAPS.md -->

# Technical debt and integration gaps

## Highest-risk

- Physical safety: ESC output driver, arming, independent kill, lost-link policy and motor order not validated.
- Estimator frames: sensor-to-body matrices still identity placeholders in v2.
- Vertical control: no validated LiDAR integration and unmeasured dynamic BMP pressure disturbance.
- Dynamics: motor thrust curve, lag, yaw torque and actual COM/inertia missing.
- FIFO and I²C: 400 kHz integration off-device tested but not benchmarked on board.

## Source-level cleanup candidates

- Separate sensor HAL from controller core, keep a stable binary sensor capture/diagnostic mode.
- Add version/hash to boot output and structured logs with timestamps and units.
- Introduce single authoritative mode/command schema across historical ESP32/Xbox branches.
- Make fault state machine explicit with no automatic recovery to armed after any critical fault.
- Add deterministic unit tests for motor allocator saturation, dt outliers, power-fault response, sensor frame transforms and magnetometer norm gates.
- Validate pressure compensator numerics on several operating points and prove read-scheduling freshness.

## Calibration pipeline

Replace the old degenerate MAG ellipsoid fitter, remove misleading plotted “corrected” shape, and store candidate parameter confidence/provenance in one signed/versioned artifact. Revalidate full accel matrix with better face fixture and gyro drift with temperature logs.

## Avoid false optimization

Do not chase the fastest apparent average sample rate if the real worst-case delay, sensor covariance, actuation latency or reliability becomes worse.


---

<!-- BEGIN: 09_ROADMAP/04_DECISION_LOG.md -->

# Design decisions, rationale and reversibility

| Decision | Rationale | Revisit trigger |
|---|---|---|
| ESP32-S3 current sensor-FC platform | Physical board and CH343 captures already exist | New board measured/validated |
| QMC5883P not HMC5883L | Chip ID/address and original captures | Hardware replacement/clear ID evidence |
| Keep Nov 4 MAG `0x3D` in v2 | Actual integrated high-rate capture worked | Controlled new A/B evidence |
| Wi-Fi disabled for FC bench | Previous interference and deterministic timing priority | Validated power, EMI and separate telemetry task |
| 921600 baud for calibration/bench | Successful binary acquisition, known CH343 | Measured CRC/overrun problems |
| Use full accel/MAG 3×3 matrices | Independent raw fitting and holdout | Independent physical mapping/temperature test |
| Use MPU offset registers with residual verification | Reduce coarse bias, retain software cross-axis | Register effect mismatch on hardware |
| MPU FIFO at 500 Hz | Buffer sensor timing and reduce host jitter | On-board overflow/latency regression |
| BMP ×1/×1 IIR×4 ~167 Hz | Freshness/latency compromise | Dynamic step/noise measurements |
| Seven cascaded PIDs | User's learning and inspectability objective | Measured plant shows strong MIMO coupling |
| Quaternion + 3-state vertical KF | Robust orientation, relative height/velocity tracking | New independent navigation sensors |
| Motor actuation off in v2 | No flight control or failsafe validation | Full independent hardware-readiness review |

Every changed decision should link to a source commit and measured evidence. Do not turn the “revisit trigger” into permission to skip safety.


---

<!-- BEGIN: 09_ROADMAP/05_AI_HANDOFF_PROMPT.md -->

# Instructions for another engineering AI

You are joining **IntelQCopter**, a custom ESP32-S3 F450-class quadcopter project. Read `00_START_HERE/00_README_FIRST.md`, `01_CURRENT_STATUS.md`, `02_EVIDENCE_AND_CONFIDENCE.md`, all of `03_CALIBRATION/`, `04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md`, `05_ESTIMATION/`, `06_CONTROL/`, and `07_TESTING/` before editing firmware.

## Current ground truths

- Actual MAG is QMC5883P `0x2C`, ID `0x80`.
- Successful November 4 full capture had MAG `0x3D`, ~195 Hz; isolated v13 `0xC9` 100 Hz also worked on another run.
- Use November 4 independent numerical candidate constants, not the original degenerate MAG ellipsoid output.
- Current FC v2 is **motor-disabled**; host tests do not establish actual board timing.
- Mass, inertia and motor thrust/lag uncertainty are assumption-based.
- Direct Xbox BLE and ESP32 LiDAR RX are unverified; virtual Xbox/CH343 bridge only.

## Your work contract

1. Work on a new Git branch/versioned project folder, preserving source artifacts and immutable raw data.
2. State exactly what you changed, expected impact and why measured evidence supports it.
3. Maintain `tools/` PowerShell entry points, CH343 autodetection and Windows-friendly workflow.
4. Keep `IQ_MOTOR_OUTPUTS_ENABLE=0`; do not enable ESC outputs or remove failsafe barriers.
5. Run host tests plus actual PlatformIO build where available, then ask the human for a motor-off hardware log; label what you did not run.
6. Evaluate numerical residuals, 500 Hz worst-case timing, FIFO health, magnetometer freshness and barometer KF NIS rather than average-only metrics.
7. Report formulas, dimensions, uncertainties and physical limitations; do not claim flight readiness.

You can develop a fault-injection suite or better model identification analysis now, without inventing a motor thrust curve or conflicting with the other AI’s COM port session.


---

<!-- BEGIN: 09_ROADMAP/06_SESSION_AND_CHANGELOG_TEMPLATE.md -->

# Reusable experiment template

Copy the following for each firmware change or board test.

```markdown
# IntelQCopter test: <short name>
Date/time and timezone:
Operator:
Git branch + commit / ZIP SHA256:
Power state: cold sensor power cut / ESP reset / USB-only reset
Hardware: board, wiring, LiPo/USB, props removed, ESC signals disconnected
Firmware: environment, flags, QMC C1/C2, MPU DLPF/range/FIFO, BMP mode, I2C Hz, UART baud
Change from baseline:
Hypothesis:
Procedure:
Expected observable success/failure:
Log paths + sizes + checksums:
Metrics: IMU/MAG/BMP fresh rates, loop avg/P99/max us, late/miss,
  fifo loss/overflow, I2C errors, CRC/sequence loss, temperature,
  baro NIS, raw gyro residual and accel g norm.
Safety: physical motor output disabled? verified?
Result: PASS / FAIL / INCONCLUSIVE
Evidence and caveats:
Next experiment:
```

## One-change-at-a-time examples

- `0x3D QMC profile, 400 kHz -> 100 kHz QMC reads` while holding all other settings fixed.
- Boot gyro offset feature off -> on with same sensor temperature and stationary orientation.
- BMP IIR×4 -> ×2 with ruler-controlled step, measuring lag and pressure SD.

For repeated experiments, maintain a `runs.csv` alongside raw logs rather than overwriting `last_run.log`.

