# IntelQCopter — A-to-Z Markdown documentation index

> Snapshot: **5 November 2026** · Generated project reference documentation. **Motor-disabled FC only: not flight-ready.**

## Navigation

All chapters are separate and searchable `.md` files. Start with `00_START_HERE/00_README_FIRST.md`. Source snapshots in `90_SOURCE_SNAPSHOTS/` are preserved verbatim for historical comparison, and may contain **outdated assumptions or superseded sensor labels**.

### 00_START_HERE — Project overview, chronology, evidence and references

- [IntelQCopter — complete project knowledge base](00_START_HERE/00_README_FIRST.md)
- [Current project status — 5 Nov 2026](00_START_HERE/01_CURRENT_STATUS.md)
- [Evidence rules, verification levels and claim ledger](00_START_HERE/02_EVIDENCE_AND_CONFIDENCE.md)
- [Chronological engineering timeline](00_START_HERE/03_TIMELINE.md)
- [Glossary — hardware, estimation and controls](00_START_HERE/04_GLOSSARY.md)
- [Sources, artifact provenance and reproducibility](00_START_HERE/05_REPRODUCIBILITY_AND_PROVENANCE.md)

### 01_HARDWARE — Airframe, wiring, electronics and propulsion

- [Airframe geometry and coordinate conventions](01_HARDWARE/01_AIRFRAME_GEOMETRY.md)
- [ESP32-S3 PCB and present pinout knowledge](01_HARDWARE/02_PCB_PINOUT_AND_BUSES.md)
- [Power, PCB integrity and electrical risks](01_HARDWARE/03_POWER_AND_SIGNAL_INTEGRITY.md)
- [Motors, ESCs, propellers and propulsion limitations](01_HARDWARE/04_MOTORS_ESCS_PROPELLERS.md)
- [Mass budget, BOM, COM and uncertainty](01_HARDWARE/05_MASS_CENTER_OF_MASS_BOM.md)
- [Optional companion computer, ROS2, vision and autonomy](01_HARDWARE/06_COMPANION_COMPUTER_AND_ROS2.md)
- [Xbox input hardware and communications reality](01_HARDWARE/07_XBOX_INPUT_HARDWARE.md)

### 02_SENSORS — Sensors, registers, clocks, frames and signal integrity

- [Sensor suite, buses and multirate schedule](02_SENSORS/01_SENSOR_SYSTEM_OVERVIEW.md)
- [MPU6050 — register-level acquisition and filtering](02_SENSORS/02_MPU6050_REGISTER_CONFIG.md)
- [QMC5883P — actual compass identity, register map and working profiles](02_SENSORS/03_QMC5883P_REGISTER_CONFIG.md)
- [BMP280 — pressure altitude, filter and real output rate](02_SENSORS/04_BMP280_FILTERS_AND_COMPENSATION.md)
- [TOFSense F2 Mini — range sensor integration and calibration](02_SENSORS/05_TOFSENSE_F2_MINI.md)
- [I²C bandwidth, scheduling and transaction hazards](02_SENSORS/06_I2C_BUDGET_100K_VS_400K.md)
- [Timing design — sample clock, FIFO, callbacks and deadlines](02_SENSORS/07_CLOCKS_INTERRUPTS_AND_FIFO.md)
- [Sensor axes, mounting transforms and sign validation](02_SENSORS/08_SENSOR_FRAMES_SIGN_AND_ALIGNMENT.md)

### 03_CALIBRATION — Raw data, fitted constants, uncertainties and application

- [4 November 2026 — successful integrated raw calibration session](03_CALIBRATION/01_NOV4_CAPTURE_OVERVIEW.md)
- [MPU6050 gyroscope — stationary calibration and precision](03_CALIBRATION/02_GYRO_BIAS_AND_NOISE.md)
- [MPU6050 accelerometer — full six-face matrix calibration](03_CALIBRATION/03_ACCEL_SIX_FACE_FIT.md)
- [QMC5883P magnetometer — independently fitted 3×3 calibration](03_CALIBRATION/04_MAG_HARD_SOFT_IRON_FIT.md)
- [Why the magnetometer finally worked — the history without oversimplification](03_CALIBRATION/05_MAG_WORKING_CONFIG_HISTORY.md)
- [BMP280 baseline, noise and zero altitude](03_CALIBRATION/06_BAROMETER_BASELINE_AND_VARIANCE.md)
- [Optional LiDAR calibration — complete plan for when available](03_CALIBRATION/07_LIDAR_CALIBRATION_PLAN.md)
- [Calibration error budget, temperature effects and better validation](03_CALIBRATION/08_TEMPERATURE_NOISE_ALLAN_AND_VALIDATION.md)
- [Applying hardware offsets and 3×3 matrices in the FC loop](03_CALIBRATION/09_APPLYING_MATRICES_IN_REAL_TIME.md)
- [Calibration file formats and provenance](03_CALIBRATION/10_CALIBRATION_DATA_FORMATS.md)

### 04_FIRMWARE — History, firmware architecture, build and diagnostics

- [Software tracks, Git branches and artifact families](04_FIRMWARE/01_SOFTWARE_BRANCHES_AND_RELEASES.md)
- [Full GUI calibration firmware — original v13 driver integration](04_FIRMWARE/02_FULL_CALIBRATION_GUI.md)
- [Fast FC v2 — actual source architecture](04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md)
- [Scheduler, FIFO buffering and low-latency design](04_FIRMWARE/04_SCHEDULER_FIFO_AND_LATENCY.md)
- [PlatformIO v2 build flags and Windows workflow](04_FIRMWARE/05_BUILD_CONFIGURATION.md)
- [Boot sequence, register offsets and zero references](04_FIRMWARE/06_BOOT_AND_REGISTER_OFFSET_SEQUENCE.md)
- [UART communications and command framing](04_FIRMWARE/07_UART_BINARY_AND_XBOX_PROTOCOL.md)
- [Windows tooling, launchers, COM detection and known regressions](04_FIRMWARE/08_POWERSHELL_GUI_AND_TOOLS.md)
- [FC v2 log fields — how to interpret them](04_FIRMWARE/09_DIAGNOSTIC_LOG_FIELD_REFERENCE.md)
- [How two AIs and the project owner should collaborate on firmware](04_FIRMWARE/10_REPOSITORY_AND_COLLABORATION_GUIDE.md)
- [Build and test the current FC v2 safely](04_FIRMWARE/11_COMPILING_AND_TESTING_FC_V2.md)

### 05_ESTIMATION — Quaternion, compass, height Kalman and covariance

- [Coordinate frames and physical interpretation](05_ESTIMATION/01_COORDINATE_FRAMES.md)
- [Quaternion attitude estimation](05_ESTIMATION/02_QUATERNION_ATTITUDE_ESTIMATOR.md)
- [Magnetometer heading observer and disturbances](05_ESTIMATION/03_MAG_YAW_OBSERVER.md)
- [Height, vertical speed and acceleration-bias Kalman filter](05_ESTIMATION/04_THREE_STATE_ALTITUDE_KALMAN.md)
- [Uncertainty-driven fusion: choosing Q, R and confidence gates](05_ESTIMATION/05_FUSION_R_Q_AND_CONFIDENCE.md)
- [Estimator faults, loss of reference and reset policy](05_ESTIMATION/06_ESTIMATOR_FAULTS_AND_RESET.md)

### 06_CONTROL — Seven PID loops, dynamics, mixer, Xbox and uncertainty

- [Controller architecture — all seven PID loops](06_CONTROL/01_CASCADE_ARCHITECTURE.md)
- [All 21 PID parameters — numerical table and units](06_CONTROL/02_PID_GAIN_TABLE_WITH_UNITS.md)
- [Rate PID gains from inertia and pole-placement intuition](06_CONTROL/03_RATE_PID_POLE_PLACEMENT.md)
- [Roll/pitch angle and yaw-rate control](06_CONTROL/04_ANGLE_AND_YAW_LOOPS.md)
- [Height hold and vertical-speed control — control strategy and limitations](06_CONTROL/05_ALTITUDE_AND_VERTICAL_SPEED_PID.md)
- [PID implementation details — discrete integrator, derivative and saturation](06_CONTROL/06_PID_ANTI_WINDUP_AND_D_FILTER.md)
- [X mixer: torque and thrust to four **virtual** motor fractions](06_CONTROL/07_VIRTUAL_X_MIXER.md)
- [Rigid-body dynamics from Lagrangian model to a PID plant](06_CONTROL/08_MASS_INERTIA_AND_DYNAMICS.md)
- [Monte Carlo mass/inertia/PID uncertainty — correct interpretation](06_CONTROL/09_MONTE_CARLO_AND_ERROR_PROPAGATION.md)
- [Synthetic rate-step response and PID tuning workflow](06_CONTROL/10_RATE_RESPONSE_SIMULATIONS.md)
- [Xbox commands, authority and altitude re-zero gesture](06_CONTROL/11_XBOX_COMMANDS_AND_ALTITUDE_CHORD.md)

### 07_TESTING — Bench plans, defects, safety and acceptance criteria

- [Hardware test sequence — safe and reproducible](07_TESTING/01_BENCH_TEST_ORDER.md)
- [What counts as success — measurable acceptance criteria](07_TESTING/02_ACCEPTANCE_METRICS_AND_LOGS.md)
- [Fault injection matrix (safe, motor-off)](07_TESTING/03_FAULT_INJECTION_TEST_MATRIX.md)
- [Failure history and recovered engineering lessons](07_TESTING/04_HISTORIC_FAILURES_AND_FIXES.md)
- [Independent sensor-calibration validation experiments](07_TESTING/05_CALIBRATION_VALIDATION_EXPERIMENTS.md)
- [Propulsion system identification — prerequisite for real flight gains](07_TESTING/06_MOTOR_THRUST_IDENTIFICATION_PLAN.md)
- [Flight readiness gate — what is currently missing](07_TESTING/07_SAFETY_AND_FLIGHT_READINESS.md)

### 08_RESEARCH — Engineering math, primary documents and research directions

- [Mathematical foundations — vehicle equations and linearization](08_RESEARCH/01_DYNAMICS_AND_STATE_SPACE.md)
- [Primary references, not unverified blog summaries](08_RESEARCH/02_PRIMARY_DATASHEETS_AND_PAPERS.md)
- [Robustness, statistics and what an error bar means](08_RESEARCH/03_ROBUSTNESS_AND_UNCERTAINTY_METHOD.md)
- [Performance tradeoffs and future design options](08_RESEARCH/04_DESIGN_TRADEOFFS_AND_FUTURE_HARDWARE.md)

### 09_ROADMAP — Unresolved questions, priority work and handoff

- [Priority roadmap — immediate engineering work after Nov 5 snapshot](09_ROADMAP/01_IMMEDIATE_PRIORITIES.md)
- [Outstanding unknowns — answer with measurements, not guesses](09_ROADMAP/02_OPEN_QUESTIONS.md)
- [Technical debt and integration gaps](09_ROADMAP/03_TECHNICAL_DEBT_AND_KNOWN_GAPS.md)
- [Design decisions, rationale and reversibility](09_ROADMAP/04_DECISION_LOG.md)
- [Instructions for another engineering AI](09_ROADMAP/05_AI_HANDOFF_PROMPT.md)
- [Reusable experiment template](09_ROADMAP/06_SESSION_AND_CHANGELOG_TEMPLATE.md)

### 90_SOURCE_SNAPSHOTS — Verbatim historical Markdown references (not current truth)

- [Scope, provenance, and engineering discipline](90_SOURCE_SNAPSHOTS/NOV1_ENGINEERING_STUDY_UNMODIFIED.md)
- [AGENTS.md — IntelQCopter local AI coding instructions](90_SOURCE_SNAPSHOTS/NOV3_AGENT_GUIDELINES_UNMODIFIED.md)
- [IntelQCopter — comprehensive engineering handoff for a second AI](90_SOURCE_SNAPSHOTS/NOV3_PROJECT_HANDOFF_UNMODIFIED.md)
- [IntelQCopter Calibration Package - 2026-11-04](90_SOURCE_SNAPSHOTS/NOV4_CALIBRATION_SESSION_README_UNMODIFIED.md)
- [IntelQCopter - Independent Calibration Audit (2026-11-04)](90_SOURCE_SNAPSHOTS/NOV4_INDEPENDENT_AUDIT_README_UNMODIFIED.md)
- [IntelQCopter - ORIGINAL v13 QMC5883P driver integrated in full calibration](90_SOURCE_SNAPSHOTS/ORIGINAL_V13_FULL_CALIBRATION_README_UNMODIFIED.md)
- [IntelQCopter v2 — deep engineering analysis and reproducible build contract](90_SOURCE_SNAPSHOTS/V2_DEEP_ENGINEERING_ANALYSIS_UNMODIFIED.md)
- [Dynamics, calibration and uncertainty — IntelQCopter](90_SOURCE_SNAPSHOTS/V2_MODEL_AND_UNCERTAINTY_UNMODIFIED.md)
- [All seven PID loop parameters, units and uncertainty — v2](90_SOURCE_SNAPSHOTS/V2_PID_TABLE_UNMODIFIED.md)
- [IntelQCopter Fast 500 Hz FC v2 — calibrated 7-PID + altitude Kalman (SAFE BENCH BUILD)](90_SOURCE_SNAPSHOTS/V2_README_UNMODIFIED.md)
- [Hardware test plan — do not attach propellers](90_SOURCE_SNAPSHOTS/V2_TEST_PLAN_UNMODIFIED.md)
- [Pilot interface (bench only)](90_SOURCE_SNAPSHOTS/V2_XBOX_AND_UART_UNMODIFIED.md)

## Source confidence

- The Nov 4 MAG success at **0x3D (~195 Hz)** supersedes interpreting a file name `100hz` as the actual sensor profile.
- Calibration constants are independent *candidates* with known orientation/temperature limits.
- Mass, inertia, thrust and PID gain SD values are assumption-prior calculations; physical masses/motors were not measured.
- The most recent v2 C++ FC source remains hardware-untested and ESCs disabled.
