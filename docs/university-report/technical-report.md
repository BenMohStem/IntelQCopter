# IntelQCopter
## Design and validation of a modular quadcopter flight control system

**Professional and Personal Project**

Technical report and system requirements specification

**Report reference IQC-PPP-002 | Revision 2.0 | 28 October 2026**

Student name, student number, institution, supervisor and academic year: **to be completed by the student before submission**.

The project investigates how a low-cost F450 quadcopter can acquire trustworthy sensor measurements, estimate its state and execute stable flight commands through an understandable embedded controller. An optional companion computer extends the platform towards local waypoint navigation and obstacle avoidance.

The main conclusion at this stage is that a reproducible inertial measurement and software-validation foundation exists, while complete closed-loop flight, the revised sensor wiring and autonomous navigation still require verification. The report therefore presents both the engineering design and the evidence needed to advance it responsibly.

This submission combines requirements, a bill of materials, physical modelling, control and estimation methods, software architecture, verification, project planning and professional development. It is written for an engineering assessor who needs to distinguish intended performance from demonstrated results.

**Submission status:** technically reviewed working draft. University marking criteria and cover details have not been supplied. Quantitative targets labelled Proposed require review. Personal reflection and contribution statements require student confirmation. No MIT affiliation, endorsement or certification is claimed.

---
## Abstract and report guide

IntelQCopter is a student engineering project based on an F450-style quadcopter with four nominal 1400 KV motors, 9047 propellers, 40 A electronic speed controllers and a 3S 4200 mAh lithium-polymer battery. The immediate implementation uses an ESP32-S3 flight controller and an intended Xbox Series X/S input interface. The longer-term architecture retains separate RP2350 aircraft and RP2040 handheld controllers with LoRa and CC1101 radio links.

The principal technical challenge is to build reliable feedback control from imperfect measurements under a 1 ms nominal computation period. The proposed manual mode regulates yaw rate, roll and pitch angles, and vertical motion. It intentionally does not hold horizontal position. The optional ROS 2 companion sends bounded motion requests to the same flight controller rather than directly commanding motors.

Existing project evidence includes multi-pose accelerometer calibration, gyroscope restart checks, finite-duration inertial acquisition traces, source-based host tests and explicitly labelled offline estimator experiments. A held-out stationary replay demonstrates reduced output variation, but provides no independent attitude accuracy reference. A disturbance experiment produces approximately 5.2 to 5.35 degrees of peak angle error under lateral acceleration, identifying a limitation that must be resolved before flight acceptance [P1-P4].

The report proposes measurable requirements, interface contracts and staged acceptance gates. It includes a procurement-oriented materials list without invented supplier prices, a mathematical model with stated assumptions, and a verification plan that preserves failed trials. No new hardware experiment, flight or autonomous mission was performed to create this document.

### Navigation

[[CONTENTS]]

---
## 1 Project purpose and professional context

### 1.1 Problem statement

A quadcopter is unstable without feedback: small attitude deviations redirect thrust and generate unintended motion. A working demonstration requires more than a smooth sensor graph or a compiling program. The designer must establish reference frames, identify actuator behaviour, bound computation delays and verify that actual inputs pass through the intended control algorithm.

The project addresses this problem by constructing a transparent platform whose source code, measurements and acceptance decisions can be examined. Its educational value lies in the complete reasoning chain from requirements to evidence. This reflects the verification and validation distinction in systems engineering: verification asks whether the specified implementation is correct, whereas validation asks whether it meets the intended operational need [R1].

### 1.2 Objectives and deliverables

O1: obtain reproducible calibrated inertial measurements and explicit uncertainty estimates. O2: implement a readable standalone embedded controller with a nominal 1 kHz update target. O3: validate angle and vertical control through an identified model and software-in-the-loop tests before actuator activation. O4: establish an optional, bounded FC-companion communication contract. O5: prepare waypoint navigation and obstacle avoidance only when localization and sensing are sufficient. O6: demonstrate professional engineering through traceable decisions, cost awareness, risk management and critical reflection.

The deliverables are source-controlled firmware, calibration procedures, a hardware interface definition, simulation integration, original logs, derived plots, a materials list and a technical report. Each item must state its version and validation status. A PCB design is not delivered by this report; the student currently undertakes its construction.

### 1.3 Scope and stakeholders

The student is the system owner, developer and physical-test operator. The academic supervisor reviews the learning objectives and evidence. Potential team members use shared requirements and recorded interfaces to avoid inconsistent changes. The intended user is initially the student pilot operating a restricted prototype, not a commercial customer.

The initial scope excludes GPS navigation, guaranteed outdoor autonomy, flight certification, long-range radio operation and individual-cell battery protection. A garden-like scene inspired by USTHB may support simulation, but is not an accurate site reconstruction or permission for real operations.

---
## 2 Concept of operations and system boundaries

### 2.1 Standalone operation

The FC starts, initializes sensors, restores calibration, checks identities and enters a non-actuating bench-ready state. In the eventual stable-flight mode, the left stick commands yaw rate and vertical speed; the right stick commands roll and pitch angles. Centering roll and pitch requests a level attitude. It does not request zero horizontal speed: inertia and wind may continue moving the aircraft. Centering the vertical input requests altitude retention through the vertical control law.

RT increases the bounded collective or vertical demand; LT reduces it. A gentle stick response and explicit dead zones are preferred. The exact boost law and its interaction with vertical-speed control remain to be finalized; a collective override must not silently defeat altitude control. The historical proposed boost ceiling is 1700-1800 microseconds, leaving correction authority, rather than driving all motors directly to the upper endpoint.

### 2.2 Assisted operation

The ground station accepts n ordered mission points, with n at least two. After approval, the first is the start and the last is the destination. The companion computer validates coordinates, plans motion and reacts to observed obstacles. It sends limited setpoints to the FC and receives health/state feedback. The ground station may monitor or abort, but must not secretly execute the onboard planner during a claimed onboard-autonomy test.

### 2.3 Mode and gesture contract

| Input | Intended effect | Status |
|---|---|---|
| D-pad down plus A held 5 s | Request stable mode | Decided; implement and verify |
| D-pad down plus B held 5 s | Request navigation mode | Decided; valid companion and mission required |
| D-pad down plus LT held 2 s | Request disarm | Decided; trigger threshold to define |
| RT and LT | Increase and reduce demand | Decided; exact bounded law pending |

Gesture timers require continuous holding, reset on release and produce one event per activation. An accepted navigation request must be distinguishable from a rejected request. Connecting a companion must not change mode automatically.

**Present boundary:** physical arming and motor outputs remain disabled. The report specifies future operation without authorizing it. Loss-of-controller and loss-of-companion responses require an explicit state-dependent policy before flight.

---
## 3 Flight control requirements

Requirement status is **D** for a user decision, **P** for a proposed engineering target and **TBD** for an unresolved value. D describes intended behaviour, not demonstrated completion. Tests are identified in Chapter 16; detailed parameter assumptions appear in the modelling chapters.

| ID | Requirement and status | Verification |
|---|---|---|
| FC-01 | D: stable mode shall run without a companion computer | V01, V07 |
| FC-02 | D: yaw input shall command angular rate | V06 |
| FC-03 | D: roll and pitch shall use angle feedback with inner rate loops | V06 |
| FC-04 | D: neutral vertical input shall hold altitude; displacement shall command vertical speed | V09 |
| FC-05 | D: initial stable mode shall not add horizontal braking or position hold | V06 |
| FC-06 | D: input shaping shall be gentle, bounded and documented | V07 |
| FC-07 | P: boost shall preserve attitude correction headroom; collective ceiling 1700-1800 microseconds pending selection | V06 |
| FC-08 | D: controller saturation and integral windup shall be handled explicitly | V06 |
| FC-09 | D: present firmware shall not produce actuator pulses or permit physical arming | V01 |
| HMI-01 | D: left stick yaw/vertical; right stick roll/pitch; RT increase and LT decrease | V07 |
| HMI-02 | D: down plus A held 5 s shall request stable mode | V07 |
| HMI-03 | D: down plus B held 5 s shall request navigation only when ready | V07, V10 |
| HMI-04 | D: down plus LT held 2 s shall request disarm | V07 |

### Six controllers and one unresolved architecture detail

The requested six PID controllers comprise two for roll, two for pitch, one for yaw rate and one for the vertical channel. Altitude hold and vertical-speed control can be arranged using an outer proportional height-to-speed law followed by an inner vertical-speed PID. This retains six full PID blocks, but the outer law, limits and transitions still require approval. Counting PID blocks does not determine a correct control architecture.

The intent of FC-05 is especially important for assessment. It prevents a simulator-only position controller from concealing poor attitude control. If position braking is later added, it becomes a separate requirement with an observable velocity estimate and additional verification.

---
## 4 Sensing real time and interface requirements

| ID | Requirement and status | Verification |
|---|---|---|
| EST-01 | D: calibration shall preserve documented mounting, frames and independent validation data | V03 |
| EST-02 | D: volatile MPU trims shall be restored at boot and checked without double bias removal | V02 |
| EST-03 | D: gyro bias, scale and sensor-to-body mapping shall be verified | V03 |
| EST-04 | D: attitude estimates shall report acceleration sensitivity and noise assumptions | V03, V06 |
| EST-05 | D: vertical estimation shall use height and vertical velocity states | V09 |
| EST-06 | D: stale, repeated, saturated and invalid data shall be identifiable | V04, V08 |
| EST-07 | P: slow measurements shall retain their true timestamps and age | V04, V05 |
| RT-01 | D: complete FC update target shall be 1000 Hz | V05 |
| RT-02 | P: initial critical-path budget shall be at most 800 microseconds | V05 |
| RT-03 | P: no observed deadline miss during a defined 10 min loaded test | V05 |
| RT-04 | D: communications and logging shall have bounded waits and buffers | V05, V08 |
| RT-05 | D: ESP runtime shall remain one readable Arduino-style main.cpp | Inspection, build |
| RT-06 | D: sampling, estimation, control and ESC update rates shall be distinguished | V05 |
| IF-01 | D: I2C logic and pull-ups shall use 3.3 V with verified supply routing | V01 |
| IF-02 | D: MPU bypass configuration shall be restored and read back after reset | V02 |
| IF-03 | D: lidar I2C mode and address shall be verified after configuration | V04 |
| IF-04 | P: slow bus shall start at 100 kHz; changes require electrical and timing checks | V04, V05 |

The nominal 1 kHz sampling stream does not prove a complete 1 kHz FC. The deadline includes acquisition handling, estimation, control, mixing and publication. Slow-sensor accesses must not introduce an unbounded delay. A 40 A ESC rating does not specify its accepted pulse frequency or actuator bandwidth.

---
## 5 Architecture and electrical interface definition

[[ARCHITECTURE]]

Figure 1. Proposed signal architecture. Blocks show responsibilities and interfaces, not a completed or flight-validated system. The FC remains the sole owner of motor commands.

| Connection | Planned assignment |
|---|---|
| MPU6050 | SDA GPIO9; SCL GPIO8; 3.3 V; AD0 grounded; INT unused |
| Magnetometer | SDA to MPU XDA; SCL to MPU XCL; 3.3 V; DRDY unused |
| BMP280 | SDA GPIO41; SCL GPIO40; 3.3 V; CSB to 3.3 V; SDO grounded |
| TOFSense F2 Mini | Regulated 5 V; RX/SDA to GPIO41; TX/SCL to GPIO40 |
| ESC signal connectors 1 to 4 | GPIO4, GPIO1, GPIO14, GPIO21; physical corner mapping pending |
| FC to companion | Proposed framed 3.3 V UART; pins, rate and connector TBD |

The magnetometer assignment consolidates the recent XDA/XCL routing request. The MPU must have its internal auxiliary master disabled and bypass enabled after initialization; read-modify-write operations must preserve FIFO bits. Normal sampling does not require rewriting these bits, but a full MPU reset requires restoration [R3].

Only one verified regulated 5 V BEC positive feeds the FC supply. All ESC signal grounds, sensor grounds and FC grounds form one common reference. Other BEC positives remain isolated. The propulsion current path stays outside thin FC tracks; USB and external supply arrangements must prevent backfeeding.

The lidar UART RX/TX pins become SDA/SCL in I2C mode [R5]. During UART configuration on GPIO40/41, isolate the BMP280 and other bus slaves. Stop UART before enabling I2C. The normal I2C address is 0x08 for ID zero; Arduino Wire expects that seven-bit address, not a shifted read/write byte. Confirm the actual clone pin labels before manufacturing.

---
## 6 Bill of materials for the flight platform

This list specifies the intended assembly rather than claiming that every item is purchased or available. Exact manufacturer identifiers, ownership and measured masses require an inventory. No supplier price or total cost is invented. Quantities are for one aircraft; propeller spares are recommended but excluded from the installed count.

| ID and quantity | Material or component | Purpose and acceptance check |
|---|---|---|
| M01 x1 | F450 or Q450 X frame with plates and arms | Structural assembly; verify motor diagonal, stiffness and mounting holes |
| M02 x4 | Nominal 1400 KV brushless motors | Exact model TBD; inspect shaft, mounts and propeller compatibility |
| M03 x4 | Nominal 40 A ESCs | Verify voltage range, input protocol and which unit has a suitable BEC |
| M04 x4 | 9047 propellers, matched CW and CCW pairs | Verify handedness, hub fit, balance and motor recommendation |
| M05 x1 | 3S 4200 mAh LiPo, marked 120C | Inspect condition; weigh; characterize useful capacity and current demand |
| M06 x1 | ESP32-S3 DevKitC-1 | Confirm board variant, exposed pins, flash/PSRAM and USB arrangement |
| M07 x1 | GY-521 MPU6050 eight-pin module | Check device identity, pin order, offsets and mounting orientation |
| M08 x1 | BMP280 six-pin module | Confirm BMP rather than BME identity; verify I2C at 0x76 |
| M09 x1 | GY-271 style magnetometer module | Identify actual HMC/QMC silicon and matching driver; inspect 3.3 V pull-ups |
| M10 x1 | TOFSense F2 Mini and cable | Verify exact model, saved I2C mode, connector orientation and range status |
| M11 x1 | Xbox Series X/S controller | Confirm firmware, BLE transport, reconnect and input mapping |
| M12 x1 set | Power distribution assembly | Rated wiring/connectors; common reference; propulsion current outside FC |
| M13 x1 | Suitable regulated avionics supply | Selected verified ESC BEC or separate regulator, not parallel BEC outputs |
| M14 x1 set | Carrier PCB and module sockets | User-made; fit, continuity, insulation and antenna clearance to verify |
| M15 x1 set | Fasteners, spacers, battery straps and insulation | Secure battery and rigid sensor reference; prevent conductive contact |

Battery capacity and C marking do not establish a measured 504 A usable discharge capability. Similarly, four 40 A ESC labels do not prove that the battery, connectors or wiring can safely deliver 160 A. Selection is based on measured propulsion demand and component ratings, with thermal and transient margin.

---
## 7 Measurement optional hardware and cost planning

| ID and quantity | Material or equipment | Procurement or implementation note |
|---|---|---|
| M16 x1 set | Battery divider and ADC RC filter | Candidate 47 kOhm / 10 kOhm, 1 percent resistors and 100 nF; verify ADC range |
| M17 x1 | Current measurement assembly | Sensor/range TBD; compatible conditioned output and external high-current path |
| M18 x4 | Three-pin ESC signal headers | Label GND, isolated BEC positive and signal; cable order must be checked |
| M19 x1 set | Lidar solder pads or connector | Four conductors; strain relief for remote sensor |
| T01 x1 | USB-TTL adapter | 3.3 V logic, 921600 baud capability and suitable 5 V lidar power |
| T02 x1 | Digital multimeter | Voltage, resistance and continuity; known uncertainty |
| T03 x1 | Current instrument | Suitable DC clamp or rated inline sensor; never place ammeter across battery |
| T04 x1 set | Scale, level, angle fixture and ruler | Record resolution and repeatability; rigid fixtures preferred to hand-held poses |
| T05 x1 | 3S balance charger and suitable charging setup | Use battery-compatible settings and inspect pack condition |
| T06 x1 set | Soldering, insulation and inspection tools | Assembly quality, magnification and continuity before power |
| O01 x1 | Raspberry Pi 4/5 or Jetson plus storage | Optional companion; exact model, OS, power and cooling TBD |
| O02 x1 | Compatible RGB camera and mount | Optional perception; calibrate intrinsics, extrinsics and timestamps |
| O03 x1 set | Companion power and serial interface | Optional; size separately and verify logic level and shared reference |
| F01 x1 each | RP2350 FC and RP2040 handheld boards | Separate future architecture, not mandatory ESP32 hardware |
| F02 x2 | Compatible LoRa modules | Future main air/handheld link; frequency and local rules to review |
| F03 x4 | CC1101 modules | Future two secondary links per side; not populated for current ESP version |

### Cost method

Use the supplied CSV to enter verified supplier, currency, quotation date, unit cost, shipping and acquisition status. Installed hardware, shared test tools and optional expansion must be subtotalled separately to avoid inflating the aircraft price with equipment already owned.

[[EQ:C_{base}=\sum_i n_i c_i+C_{shipping}+C_{fabrication}]]

[[EQ:C_{planned}=C_{base}+C_{contingency}]]

Contingency is a separately justified allowance, not an invented market price. A preliminary 15 percent allowance may be used as a planning scenario only, with sensitivity at 10 and 25 percent. The current total remains TBD until quotations are entered. A free software licence does not remove integration, debugging or maintenance effort.

---
## 8 Component selection and design trade studies

### 8.1 Controller and radio strategy

ESP32-S3 is the immediate platform because the student has chosen it for the near-term prototype and wants Arduino-style library support. The Pico architecture remains a separate long-term objective rather than a simultaneous dependency. Maintaining two architectures at once would increase verification and interface effort without resolving the current sensor and frame gates.

ESP32-S3 supports Bluetooth LE rather than Bluetooth Classic [R2]. A BLE Xbox library is a relevant candidate: the asukiaaa project documents Series X controller firmware and NimBLE version considerations [R6]. This supports investigating direct BLE operation, but does not establish compatibility for the exact board, controller firmware and dependency versions in this project. The decision gate is an actual connect/reconnect and input-latency test, not a generic library claim.

### 8.2 Sensor buses

I2C is retained because the selected modules support it and the immediate design benefits from reduced routing complexity. The new XDA/XCL arrangement can reduce physical jumpers but introduces a boot-time bypass dependency and additional bus traffic. SPI could offer a different bandwidth and failure model, but would not automatically improve the current module-level integration. Bus choice is accepted through timing, electrical and recovery tests.

### 8.3 Lidar placement

| Option | Benefit | Limitation and decision criterion |
|---|---|---|
| Downward | Local height observation close to the surface | Terrain and tilt projection matter; no forward obstacle range |
| Forward | Range along a narrow collision direction | No complete depth image; side/overhead obstacles may be missed |
| Additional depth sensor later | Wider geometric observability | Cost, mass, power and integration increase; outside initial BOM |

One RGB camera plus one fixed ray cannot guarantee complete free-space detection. The initial recommendation is to leave the mounting decision open until the priority between reliable altitude and constrained forward navigation is decided. A depth camera in a diagnostic simulation must be labelled as additional sensing, not silently attributed to the real hardware.

### 8.4 Decision discipline

Each trade record should state alternatives, constraints, chosen rationale and the test that could overturn the decision. Weighted scoring is useful only when weights and ratings are justified. No numerical trade-study scores are fabricated here.

---
## 9 Geometry mass and rigid body dynamics

Use an explicitly documented world frame, preferably ENU, and a body frame such as FLU. All signs below assume body x forward, y left and z upward. If the actual calibration uses another frame, transform it rather than changing signs informally. A rotation matrix R maps body coordinates to world coordinates.

For a nominal 450 mm motor diagonal, the arm radius is L = 0.225 m and the in-plane component is a = L divided by square root of two, approximately 0.1591 m. Motor positions are the four sign combinations of a in the body plane; motor numbering must be linked to physical corners before mixing is accepted.

[[GEOMETRY]]

Figure 2. Nominal F450 X geometry, not a manufactured dimensional drawing. Each rotor position has a signed x/y lever arm. CW/CCW signs remain to be assigned and verified.

### Force balance from Newton second law

Each rotor produces thrust Ti along body positive z. Their sum T is in newtons. Transform that thrust into world coordinates, add downward gravity and any drag/disturbance, then divide the net force by mass to obtain acceleration.

[[EQ:T=\sum_{i=1}^{4}T_i,\qquad m\dot{\mathbf{v}}=R\mathbf{e}_3T-mg\mathbf{e}_3-D\mathbf{v}+\mathbf{f}_{ext}]]

Here e3 is the unit vertical vector and D is a drag model, not a measured constant yet. At level hover with negligible vertical drag, acceleration is zero, so T0 = mg. With the supplied simulation mass 1.038 kg and g = 9.80665 m/s squared, T0 is about 10.179 N, or 2.545 N per rotor under symmetric loading. This is a model calculation, not measured thrust capability.

[[EQ:J\dot{\boldsymbol{\omega}}=\boldsymbol{\tau}-\boldsymbol{\omega}\times(J\boldsymbol{\omega})]]

Angular momentum balance gives the rotational equation. J is the inertia tensor about the centre of mass; the cross term arises because body axes rotate. At small rates near hover the term is second order, motivating a local linear model while retaining the nonlinear dynamics for simulation comparison.

---
## 10 Model parameters identification and uncertainty

| Parameter | Supplied simulation value | Qualification |
|---|---|---|
| Total mass m | 1.038 kg | Model input, not a certified weighing |
| Mass standard deviation | 0.0160312 kg | Based on supplied variance assumptions |
| Jxx and Jyy | 0.006421 kg m squared | Approximate geometry and distribution |
| Jzz | 0.011853 kg m squared | Approximate geometry and distribution |
| Battery mass | 320 g in parameter file | Conflicts with earlier rough 150 g estimate; weigh pack |
| Motor radius | 0.225 m | Measure actual motor centres |

The parameter file is preserved conceptually as an input source, not an authority for all statistical assumptions. In particular, combining four independent mass uncertainties differs from treating four identical errors as fully correlated. The inertia products and offset centre of mass are also not established by a diagonal approximation.

For a uniform thin arm, inertia depends on the integral of squared distance along the arm, not simply all its mass at the motor. For general components, use the parallel-axis theorem to move each component inertia to the aircraft centre of mass. Unknown geometry should be expressed as a parameter range and tested for sensitivity.

[[EQ:J_{xx}=\sum_i\left(J_{xx,i}^{COM}+m_i(y_i^2+z_i^2)\right)]]

### Measurement programme

Weigh the complete flight-ready assembly and each removable major component. Determine centre of mass using repeatable support or balance measurements in more than one direction. Record reference accuracy, repeated readings and mounting configuration. A static scale measures force divided by the assumed local gravitational acceleration; a repeated stable reading alone does not prove calibration accuracy.

Propulsion identification requires a guarded thrust fixture with appropriate load sensing, voltage/current logging and a defined command sequence. Fit the steady-state command-to-thrust curve and estimate the step response time constant. Do not infer thrust from KV, ESC current rating or battery C marking. Any motor experiment requires separate authorization and a physical safety procedure; it is not enabled by this report.

For a differentiable output y = f(p), a first-order parameter covariance approximation is Sigma_y = G Sigma_p G transpose, where G contains sensitivity derivatives. If nonlinearities or broad uncertainty make this approximation poor, use justified bounded sweeps or Monte Carlo sampling. Do not assign a Gaussian distribution merely because a quantity is uncertain.

---
## 11 Linear models discrete implementation and PID design

Near level hover, neglecting second-order coupling yields local roll and pitch double integrators and a yaw-rate single integrator. The local vertical perturbation is driven by thrust change about mg. These are explanatory approximations whose validity must be checked against the nonlinear simulator [R8].

[[EQ:G_{\phi}(s)=\frac{1}{J_{xx}s^2},\quad G_{\theta}(s)=\frac{1}{J_{yy}s^2},\quad G_r(s)=\frac{1}{J_{zz}s},\quad G_z(s)=\frac{1}{ms^2}]]

An identified actuator lag multiplies these plants by 1/(tau_m s + 1). Sensor filtering, sample/hold, computation delay, output saturation and voltage dependence must also be included. Ideal double-integrator gains alone would overlook the limiting dynamics of the real aircraft.

For x dot = A x + B u with a held input and sampling period Ts, the exact discretization is:

[[EQ:A_d=e^{AT_s},\qquad B_d=\int_0^{T_s}e^{A\tau}B\,d\tau]]

For an angle/rate pair driven by torque, the discrete updates are angle_next = angle + Ts rate + Ts squared torque/(2J) and rate_next = rate + Ts torque/J. This identifies the units and the dependence on the actual interval. At Ts = 0.001 s, small numerical coefficients do not justify ignoring delays or accumulated timing error.

### Design and acceptance strategy

Use the inner rate loops first, then close the slower angle loops. Define bandwidth separation, command limits and filtered derivative action. Integral gain removes persistent error but can accumulate during saturation; derivative action amplifies measurement noise without filtering. Ki and Kd may therefore be zero during an incremental lesson, provided the design reason and later acceptance are explicit.

MOD-01 D requires declared frames; MOD-02 D requires physical and discrete derivations; MOD-03 D requires identified gains and stability analysis; MOD-04 D requires uncertainty analysis. Initial Proposed targets are phase margin at least 45 degrees, gain margin at least 6 dB, small-step overshoot at most 10 percent and settling within 5 percent in at most 1 s. These are design proposals, not measured achievements.

Final numerical PID gains remain TBD. Their absence is an identification dependency, not an invitation to fabricate precise values from incomplete propulsion data.

---
## 12 Calibration statistics and state estimation

For raw acceleration r, apply calibrated bias b and transformation C. If hardware trims already remove most of b, subtract only the documented software remainder. C may contain scale, cross-axis and mounting corrections; it is not restricted to a table-level adjustment.

[[EQ:\mathbf{a}_{cor}=C(\mathbf{r}-\mathbf{b})]]

MPU offset registers are volatile user configuration. Store constants in firmware or an explicitly designed persistent FC record, restore at boot and verify readback. Do not repeatedly add fitted offsets to an already corrected baseline. A new mounting invalidates mounting-dependent calibration evidence.

[[EQ:\bar{\mathbf{x}}=\frac{1}{N}\sum_i\mathbf{x}_i,\qquad S=\frac{1}{N-1}\sum_i(\mathbf{x}_i-\bar{\mathbf{x}})(\mathbf{x}_i-\bar{\mathbf{x}})^T]]

The diagonal of S contains variances; off-diagonal entries describe joint fluctuations. Zero covariance is not a general independence proof. Likewise, covariance of the mean equals S/N only under suitable independent-sample assumptions. At high sampling rates filtering and physical dynamics correlate adjacent measurements, so 20000 samples are not necessarily 20000 independent observations.

### Attitude and vertical filters

A scalar angle filter predicts from angular rate and corrects using an acceleration-derived angle. The correction gain depends on predicted uncertainty P and observation variance R. Q must represent the specified process model, including its sampling interval and bias assumptions.

[[EQ:P_k^-=P_{k-1}+Q_k,\qquad K_k=\frac{P_k^-}{P_k^-+R_k},\qquad P_k=(1-K_k)P_k^-]]

For a vertical state [z,vz], use the constant-acceleration discrete prediction: z_next = z + dt vz + dt squared az/2 and vz_next = vz + dt az. Barometer or correctly projected downward lidar supplies observations with explicit delays and validity. The two-state design does not automatically estimate acceleration bias; that limitation must be analysed or the model extended.

The supplied mass variance is not a valid direct entry in Q. First propagate how uncertain mass, thrust and disturbances affect the state. Monitor innovations, covariance symmetry and numerical consistency. A small estimated covariance is not evidence of real accuracy without a suitable reference and model checks.

---
## 13 Embedded software and timing design

The runtime remains a single Arduino-style main.cpp, using C-like functions and structs where practical, Allman braces, one statement per line and Serial.printf output. Existing libraries are used for device support rather than recreating low-level drivers without need. Dependency versions and the PlatformIO configuration are recorded.

### Execution model

The acquisition path timestamps new data, checks sequence/integrity and applies the frozen calibration. Estimation consumes a coherent sample. Control calculates bounded setpoints and commands; the mixer reserves attitude authority before saturation. Logging and slow operations receive snapshots through bounded exchanges. One owner manages each I2C bus. Dual-core execution is introduced only when its synchronization and timing are measured.

The nominal 1 ms interval is a system deadline, not merely a timer setting. A proposed allocation of 800 microseconds critical work and 200 microseconds reserve must be measured under the full expected load. Historical handler maxima around 822 microseconds already exceed that proposed work budget and leave at most 178 microseconds before 1 ms if directly comparable; the handler is not the whole future controller.

### Failure and diagnostic contract

Every measurement carries a source identity, time, validity and sequence. A timeout must terminate predictably. A full queue produces a counted loss or explicit degraded state. Stale data must not be re-labelled as a new observation. Invalid measurements do not silently become zero angles, zero altitude or zero obstacle distance.

Debug/profiling levels separate compact counters from high-volume raw recording. Logging must never change the nominal control behaviour silently. Capture task timing, maximum and quantiles, queue occupancy, sensor age and reset reasons. Record finite test duration and load: zero observed deadline misses is not a mathematical worst-case execution-time proof.

### Release rules

Each meaningful iteration is committed and pushed on the dedicated branch. Source hashes bind results to code; a changed source or configuration requires impact analysis. Credentials, environments and unrelated files are excluded. A host test with synthetic clocks and fake peripherals is labelled a host test, not an ESP32-S3 hardware emulation or a real-time measurement.

---
## 14 Companion communication and autonomous navigation

COM-01 D requires independent stable operation without ROS 2. COM-02 P proposes framed 3.3 V UART with protocol version, message length, sequence, mission identifier, units, command validity and CRC. COM-03 P proposes a 10 Hz heartbeat and 300 ms expiry for initial tests. These values and the pin assignment remain reviewable.

The FC acknowledges the accepted mode and reports health. Invalid frames do not renew a command lease. A mode transition initializes affected references/integrators to avoid discontinuity. Loss of heartbeat is detected independently of whether the last setpoint was numerically reasonable. The approved failure response is still TBD and must not be silently replaced by an automatic landing or fall.

### Navigation and perception requirements

| ID | Requirement | Verification |
|---|---|---|
| NAV-01 D | Visit every approved mission node in order | V10 |
| NAV-02 P | Initial node acceptance radius 0.5 m with 1 s dwell and a speed condition | V10 |
| NAV-03 D | Replan from observed obstacles without hidden obstacle truth | V11 |
| NAV-04 D | Distinguish image detections from metric range and free space | V11 |
| NAV-05 TBD | Select and validate an observable localization source | V10, V11 |

YOLO detects image content; it does not by itself establish metric position or the complete collision-free volume [R7]. The companion also needs camera calibration, body-camera extrinsics, timestamps, a localization method and a planning representation. An object class not detected by a network must not be treated automatically as empty space.

ROS frames map, odom and base_link must have defined origins and transformations. A local waypoint mission cannot be validated without a metric localization reference. Ground-truth pose may be used for diagnostic experiments only if the oracle interface is disclosed and excluded from realistic acceptance.

Choose speed from sensing and braking ability. For speed v, total detection-to-action latency tau and available deceleration a, a first-order stopping distance is v tau + v squared/(2a), plus uncertainty margin. A narrow ray and uncertain braking may require a very restricted navigation envelope. Exact companion hardware and ROS distribution are selected after the interface and workload are defined.

---
## 15 Battery protection state management and risk

ENE-01 D requires calibrated total-pack voltage and current measurements. ENE-02 D requires an energy estimate that distinguishes voltage sag, capacity and current integration. ENE-03 records the historical request to warn at 35 percent and disarm at 20 percent without automatic landing. It is a policy requiring explicit pre-flight review: a motor cut in flight causes a fall. It does not override the present prohibition on physical arming.

For the candidate 47 kOhm upper and 10 kOhm lower divider, the ideal ADC voltage is battery voltage divided by 5.7; 12.6 V gives about 2.21 V. Resistor tolerance, ADC calibration, input range, filtering and transients remain to be checked. GPIO5 was a candidate ADC input, not a final approved assignment. A suitable current sensor must preserve the external high-current path.

[[EQ:SOC(t)=SOC(0)-\frac{1}{C_{usable}}\int_0^t I(\tau)\,d\tau]]

Use consistent charge units and an independently characterized usable capacity. A complete-pack estimate cannot protect an individual weak cell. Balance charging is a separate function. Hysteresis, persistence time, uncertainty and invalid-sensor handling must be defined before thresholds can drive a physical action.

| Hazard | Consequence | Control and remaining gate |
|---|---|---|
| Wrong axis or motor sign | Positive feedback and loss of control | Signed rotations and mixer checks before actuator tests |
| Multiple BEC positives | Backfeed, heating or damage | One verified source; continuity and voltage inspection |
| I2C hang or stale sample | Uncontrolled feedback input | Bounded timeouts and invalid-state logic |
| Saturation and windup | Delayed recovery | Anti-windup, headroom and saturation logs |
| Lost pilot/companion | Uncontrolled command persistence | Expiring messages; flight-phase policy TBD |
| Overclaimed obstacle coverage | Collision | Restricted envelope, missed-detection and off-ray tests |

The initial state sequence is BOOT, SELF_TEST and BENCH_READY. Armed stable and navigation states remain inaccessible. A reset never implies arming. Failure severity and phase determine the future response; this document does not claim certification or quantitative hazard probabilities.

---
## 16 Verification and simulation campaign

| Test | Requirement coverage | Evidence and method |
|---|---|---|
| V01 | IF-01, FC-09, COM-01 | Inspect power, common ground and non-actuating outputs |
| V02 | EST-02, IF-02 | Cold boot, MCU reset and MPU reset; readback and identities |
| V03 | EST-01/03/04, MOD-01 | Independent poses, signed rotations and reference uncertainty |
| V04 | IF-03/04, EST-06/07 | Known ranges, invalid returns, bus removal and restart |
| V05 | RT-01/02/03/04/06 | Full-load hardware timing, queues and deadlines |
| V06 | FC-02/03/05/07/08, MOD-02/03/04 | Steps, model sensitivity, saturation and genuine FC path |
| V07 | HMI-01/02/03/04, FC-01/06 | Input mapping, gestures, reconnect and companion absence |
| V08 | COM-02/03, EST-06, RT-04 | Corruption, replay, delayed and expired messages |
| V09 | FC-04, EST-05 | Vertical response and sensor loss with disclosed model |
| V10 | NAV-01/02/05 | Two, three and n nodes; frame/localization and progression |
| V11 | NAV-03/04 | Unknown, unclassified and off-ray obstacles; no oracle |
| V12 | ENE-01/02/03 | Calibrated reference, charge integration and threshold faults |

### Planned Gazebo experiments

Use ten distinct garden-like environments with stored obstacle layouts and seeds. Propose five autonomous noise-seed trials per environment, giving 50 trials, and five manual command scenarios covering takeoff, motion and landing. These are future tests, not completed flights. Keep failed runs and compare revisions under the same conditions.

The manual path is simulated or real gamepad input through the input adapter, actual FC algorithm, motor commands and physics engine. The autonomous path adds ground-station mission upload and the onboard companion planner before that same FC. Test scripts may stimulate inputs and record outputs; they must not apply hidden pose/velocity/force corrections to finish a task.

Record attitude, rates, height, speed, setpoints, motor requests, saturation, sensor age, innovations, computation times, mission index and collision events. Save video and timestamped logs automatically, including abnormal termination. Review video at 10-15 frames/s for trajectory interpretation; use full-rate logs for 1 kHz timing.

Proposed demonstration targets are stationary attitude RMS at most 2 degrees, altitude RMS at most 0.20 m and terminal position error at most 0.5 m within a declared envelope. No collision across the campaign is necessary but not sufficient to claim safety. With zero failures in 50 independent comparable trials, the one-sided 95 percent upper failure bound is about 5.8 percent; scene dependence weakens that interpretation.

---
## 17 Existing results and limits of interpretation

The following findings are reviewed project records, not new tests performed for this report. The source links in the evidence manifest locate the original reports. A result is not transferred automatically to a later firmware revision.

| Evidence | Recorded result | What it does not demonstrate |
|---|---|---|
| P1 calibration records | Repeated six-pose fitting and separate tilted-pose review | Arbitrary-motion accuracy or certified angle reference |
| P2 gyro/restart records | Measured trim restoration and residual assessment | Zero drift in flight or verified physical gyro axes |
| P3 FIFO timing | About 1000.69 samples/s; observed handler maximum 822 microseconds | Complete PID loop, 1 ms worst-case guarantee or revised I2C timing |
| P4 offline estimator | 80000 reused samples; reserved replay and four model cases | New hardware test, MCU estimator timing or Gazebo flight |
| P5 source/host/build | 54 Python checks, 19 host cases and ESP32 build pass | Latest upload, real peripheral operation or flight readiness |

The 27 October input-guard build used 19504 bytes RAM and 312901 bytes flash according to its saved summary. Its source SHA begins f26d314e; it was not uploaded in that evidence record. The latest historical hardware timing SHA begins 91947b5c. Mixing those two revisions would invalidate the claim that the current code achieved the historical timing [P3,P5].

### Experimental reasoning

The estimator study froze noise parameters from three recordings and reserved another recording for replay. This limits fitting on the evaluation data. A two-factor experiment crossed imposed gyro bias with imposed translational acceleration, retaining their interaction. It therefore tests a specific failure mechanism more efficiently than many repetitions of the same stationary condition.

However, the real recordings are autocorrelated, and the simulation deliberately uses its own white-noise assumptions. The evidence cannot support a general Gaussian model or an absolute confidence bound on physical attitude. These limitations are scientific findings, not cosmetic caveats to remove from a high-grade report.

---
## Reserved recording evidence

[[IMAGE:reserved]]

Figure 3. Existing reserved-log replay from the Python float64 study [P4]. All samples enter the analysis; displayed points are reduced for readability. This is an offline analysis of real logs, not a new live test or a flight.

| Quantity | Roll | Pitch |
|---|---|---|
| Accelerometer angle standard deviation | 0.177569 degrees | 0.213074 degrees |
| Filter output standard deviation | 0.007000 degrees | 0.005692 degrees |
| Filter mean minus acceleration-angle mean | -0.033769 degrees | -0.039908 degrees |

The fixed evaluation interval excludes the first two seconds. The smaller fluctuations are repeatability results only. The nonzero mean differences were retained rather than fitted away, and no independent level/angle reference exists for this recording. The lag correlations shown in the figure also contradict an exact independent white-noise interpretation. A polished graph does not remove those limitations.

---
## 18 Estimator experiment and critical evaluation

The model study used a pure-roll 10 degree amplitude, 0.5 Hz trajectory over 20 s. It applied an extra lateral acceleration from 8 to 12 s and optionally a 0.1 degree/s gyro bias. The same random draws and motion were reused across cases, isolating the selected factors [P4].

| Imposed gyro bias | Lateral acceleration | Angle RMSE | Peak error |
|---|---|---|---|
| 0 degrees/s | 0 g | 0.004312 degrees | 0.012507 degrees |
| 0 degrees/s | 0.1 g | 2.114109 degrees | 5.191244 degrees |
| 0.1 degrees/s | 0 g | 0.157612 degrees | 0.172872 degrees |
| 0.1 degrees/s | 0.1 g | 2.214463 degrees | 5.354520 degrees |

[[IMAGE:factorial]]

Figure 4. Existing four-case model experiment [P4], including adverse outcomes. The engine is Python float64 with explicit synthetic motion/noise. It is neither Gazebo nor an ESP32 execution-time test.

### Interpretation and next decision

An accelerometer responds to specific force, not gravity alone. A lateral acceleration can therefore resemble tilt. For a level body, 0.1 g creates an apparent angle of arctan(0.1), approximately 5.71 degrees, while the acceleration magnitude rises only to about 1.005 g. A magnitude-only gate may miss the disturbance.

The appropriate next work is to evaluate innovation consistency, bias modelling and acceleration-sensitive correction on independent dynamic references. Simply lowering the gain until a stationary graph looks smoother would not address observability. Increasing sample count also does not remove a systematic model error. No estimator is accepted for flight by this result.

---
## 19 Project organization schedule and resources

The work breakdown follows acceptance dependencies rather than adding many unverified features at once. Later software may be prepared offline, but cannot inherit acceptance from a failed earlier stage. If stage k fails, its dependent stages are marked blocked or invalidated, repaired on top of the corrected stage and tested again. Old evidence and commits remain available.

| Work package | Entry condition | Deliverable and exit gate |
|---|---|---|
| W0 hardware baseline | Component inventory | Verified power, pin labels, mounting and no actuation |
| W1 sensor integrity | W0 | Boot trims, identities and repeatable measurements |
| W2 physical frames | W1 | Signed known rotations; independent references |
| W3 revised buses | W1 | Bypass, BMP/lidar I2C and reset recovery |
| W4 estimator/timing | W2 and W3 | Dynamic evaluation and full-load 1 kHz evidence |
| W5 control/mixer | Identified propulsion model and W4 | Host/Gazebo response, saturation and margins |
| W6 standalone pilot | Verified BLE transport and W5 | End-to-end stable mode without companion |
| W7 navigation | W6 plus localization/perception | Missions, obstacle tests and message expiry |
| W8 motor-test review | Relevant prior gates and explicit authorization | Separate controlled procedure; no authorization implied |

### Schedule and critical path

No submission deadline or weekly availability is known. The schedule is therefore an event-driven plan, not an invented calendar commitment. The critical dependencies are physical frame confirmation, sensor availability, propulsion identification and a compatible input transport. The companion/perception work can be developed independently at its interface, but autonomous acceptance waits for a stable lower-level controller and observable localization.

Track planned versus actual effort for each work package, recording causes of rework. Prepare a reviewable minimum submission around verified sensor work and control design even if flight hardware is delayed. This protects academic deliverability without falsely claiming the final aircraft is complete.

Roles may be combined in a student project, but development and review remain separate activities. Record who produced code, who operated hardware, who checked calculations and which external libraries or AI tools assisted. The repository is the change history, not a substitute for a concise explanation of individual contribution.

---
## 20 Professional development ethics and sustainability

### Learning outcomes supported by this project

The project develops requirements writing, dimensional reasoning, embedded programming, experimental design, uncertainty analysis, control theory and technical communication. It also demonstrates professional judgement when an attractive graph is rejected as insufficient evidence, or a deadline encourages a smaller verified scope rather than an unverified flight claim.

| Competency | Portfolio evidence | Reflection to complete in the student own words |
|---|---|---|
| Experimental method | Multi-pose fit and held-out evaluation | Explain why one pose could not identify all errors |
| Embedded engineering | Source hashes, FIFO traces and bounded interfaces | Describe a timing/debugging decision personally understood |
| Mathematical modelling | Frame definition, force balance and discrete model | Derive one update and identify its assumptions |
| Project management | Gate plan, change history and inventory | Explain a scope reduction and its effect on delivery |
| Communication | Wiring tables, plots and evidence limitations | Identify feedback that changed the explanation |

### Personal reflection boundary

The engineering choices and evidence are documented, but personal experience cannot be inferred from repository activity. Before submission, the student should add a short authentic account of the initial objective, a difficulty encountered, the action taken, what changed and the next learning objective. Do not claim independent authorship of assisted work or experience that did not occur.

A useful reflection links a concrete observation to a change in understanding: for example, how distinguishing repeatability from accuracy changed the acceptance criterion. The assessor should be able to question that reasoning orally and receive an explanation in the student's own words.

### Responsible practice

Document library licences, source reuse, tools and assistance according to institutional policy. Record AI assistance accurately: this report was prepared with AI drafting/structuring support; the student must verify technical content, complete personal contributions and follow the institution's disclosure requirements. Do not sign an untrue sole-authorship statement.

Modular headers, replaceable sensors, preserved code history and reusable tools reduce avoidable replacement and duplicated effort. Battery handling, damaged-pack disposal, electronic waste and any copper-etching chemicals require appropriate local procedures. No environmental life-cycle score or legal compliance claim is fabricated.

---
## 21 Conclusions readiness and submission checklist

IntelQCopter has a useful evidence-led foundation in inertial measurement, calibration and incremental software development. The strongest current contribution is the explicit link between implementation, original logs and the limits of interpretation. The estimator disturbance study demonstrates why smooth stationary behaviour is insufficient for aircraft control.

The next engineering priorities are physical axis validation, completion of the revised sensor-bus integration, measured execution under load and identification of propulsion dynamics. PID gains and autonomous performance should follow those gates. A camera detector, a 1 kHz timer and a complete-looking PCB do not individually establish flight readiness.

### Assessment preparation checklist

| Area | Before submission |
|---|---|
| Institutional fit | Add student/supervisor/institution details, module template and actual deadline |
| Marking criteria | Map the university rubric to chapters and evidence; no assumed mark weights |
| Technical ownership | Be able to derive force balance, explain frames and demonstrate calibration application |
| Materials and cost | Complete owned/to-buy inventory, exact models and dated supplier quotations |
| Results | Keep measured, replayed, simulated and planned results visibly separate |
| Personal contribution | Add authentic reflection, responsibilities and assistance disclosure |
| Reproducibility | Confirm commit/source identifiers and evidence links before final export |
| Presentation | Prepare concise demonstration and answer limitations without overstating success |

### Suggested oral defence

Present the need and scope, then show the system architecture, one mathematical derivation and one original evidence example. Explain the adverse acceleration result and the decision it causes. Finish with the gated next steps and cost uncertainties. A proposed 8-10 minute talk is a preparation aid, not a known university requirement.

Likely questions include: Why does a level aircraft still drift? Why does an MPU reset affect the magnetometer link? What is the difference between S/N and the uncertainty of correlated samples? Why are PID gains not fixed yet? How do we prove the FC rather than a hidden simulator controller moves the drone? What would falsify the current estimator model?

**Readiness statement:** this is a university technical-report draft with a verified document build and reviewed historical evidence. It is not a completed flight demonstration, a signed final submission or a guarantee of a grade.

---
## Appendix Software resources and reproducibility

| Resource | Role and configuration record |
|---|---|
| PlatformIO and Arduino ESP32 | Embedded build; preserve platformio.ini, board variant and dependency versions |
| Adafruit MPU6050 and supporting libraries | Current sensor access; document any register-level trim and bypass configuration |
| Xbox Series X BLE library candidate | Input integration; pin compatible library/NimBLE versions after physical verification |
| ROS 2 C++ packages | Optional companion and ground tools; record distribution, middleware, OS and build instructions |
| Gazebo | User-built physics simulation; record exact engine/version, world, plugins, seeds and model parameters |
| RViz | State and model visualization; not the physics engine or proof of dynamics |
| YOLO inference runtime | Optional RGB detections; record model/weights hash, licence, resolution and measured latency |
| NAssistant | Lidar UART-to-I2C parameter configuration and readback |
| Python analysis scripts | Calibration, logging, replay and numerical experiments; record dependencies and input hashes |
| KiCad | User-led carrier design and interface inspection; exact physical module fit still requires checking |
| Git and GitHub | Version history, evidence links and release tags; no secrets or local environments |

The intended simulation engine is Gazebo. Isaac Sim is excluded from the current workflow following the student's instruction. Historical videos or tests from another engine must retain that identity rather than being relabelled. The local ROS 2 Jazzy/CycloneDDS setup is a development configuration; it does not establish compatibility with an unselected companion computer.

### Reproducing a reported result

Check out the exact evidence commit, verify the configuration and input hashes, run the stated command, retain stdout/stderr and compare against predeclared metrics. For figures reproduced in this report, the accompanying evidence-manifest.json identifies the source reports and their hashes. The copied figures retain the original bytes. No new sensor recording was generated for the submission.

Some original hardware recordings remain in private local evidence folders. Public summaries and hashes do not replace access to those records. Before academic assessment, prepare an examiner-accessible evidence package with the required originals, remove unrelated personal information and state any access restrictions. A failed original must not be removed merely because its result is inconvenient.

The report, standalone HTML, editable Markdown and procurement CSV are generated from reviewed source. Their document checks establish layout and consistency only; they do not establish aircraft performance. A student should be able to explain or reproduce the analysis presented as their assessed work.

---
## References and project evidence

References are primary manufacturer, project-maintainer or academic sources. Access/review date: 28 October 2026. Requirements and numerical targets introduced by this report are explicitly marked Proposed rather than attributed to these sources.

[R1] NASA. Systems Engineering Handbook, fundamentals and verification/validation. https://www.nasa.gov/reference/2-0-fundamentals-of-systems-engineering/

[R2] Espressif. ESP32-S3 datasheet and Bluetooth LE overview. https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf and https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/ble/overview.html

[R3] TDK InvenSense. MPU-6000 and MPU-6050 Register Map and Descriptions, revision 4.2. https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

[R4] Bosch Sensortec. BMP280 datasheet. https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

[R5] Nooploop. TOFSense-F Datasheet V2.0, connector reference and protocol. https://ftp.nooploop.com/downloads/tofsense/TOFSense-F_Datasheet_V2.0_en.pdf ; https://support.nooploop.com/tofsense/hardware/ ; https://support.nooploop.com/tofsense/protocol/

[R6] Asuki Kono. XboxSeriesXControllerESP32 library and documented firmware/dependency conditions. https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32

[R7] Ultralytics. Object detection task documentation. https://docs.ultralytics.com/tasks/detect/ . Exact model, weights, version, licence and training data for deployment remain TBD.

[R8] MIT OpenCourseWare. 16.30 Feedback Control Systems, Fall 2010. https://ocw.mit.edu/courses/16-30-feedback-control-systems-fall-2010/ . Academic background for state-space, stability and implementation analysis; no affiliation or evaluation of this project is implied.

[P1] Project accelerometer report: measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md and associated tilted-pose report. [P2] Gyro report: measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md. [P3] FIFO/frame report: measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md.

[P4] Offline estimator report and original derived figures: measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md. [P5] Input-guard build/host summary: measurements/software/2026-10-27-input-guard/summary.json. [P6] Current plan, knowledge/evidence.json, supplied params.txt and user decisions. These paths are relative to the Stepwise repository. The accompanying evidence manifest records exact hashes and repository links.
