# IntelQCopter
## Autonomous Quadcopter

**Professional and Personal Project**

**IQC-PPP-002 | Pico system edition 4.0 | 28 October 2026**

Student, institution, supervisor and academic year: complete before submission.

Target system: RP2350 flight controller, RP2040 handheld, LoRa main command link and two reserved CC1101 links per side. An onboard Raspberry Pi or NVIDIA Jetson running ROS 2 is being sought; neither has been selected. The red/white F450 cover is AI concept artwork, not a test photograph. Flight acceptance remains pending; motor outputs remain disabled.

---
## Contents and evidence legend

[[CONTENTS]]

| Figure label | Interpretation |
|---|---|
| Real log / historical fit | Derived from recorded project evidence; reference limits remain |
| Existing synthetic model | A prior numerical experiment, not flight or Gazebo |
| Analytic example / schematic | Explains a law or geometry under stated assumptions |
| Proposed architecture / campaign | Intended design or test, not a completed capability |

---
## Project in one page

**Objective:** a standalone stable-flight controller, extended progressively to autonomous waypoint navigation and obstacle avoidance. The aircraft must move through the actual FC command path.

[[VIS:system-paths]]

[[VIS:readiness]]

**D** = user decision; **P** = proposed target; **TBD** = unresolved. Historical hardware evidence, offline replay, synthetic models and planned architectures are labelled separately. No new hardware or flight experiment was run for this edition.

**Target versus prototype:** RP2350/RP2040 is the intended project architecture. The ESP32-S3/Xbox work is an experimental branch that supplied preliminary sensor and software evidence; those measurements are not RP2350 timing or flight acceptance. The companion is optional for stable manual flight and required for the intended onboard navigation mode.

**Reading map:** physics -> calibration -> estimation -> control -> perception/navigation -> verification. Complete requirements and materials follow the visual chapters. Pin wiring and low-level syntax are outside the main academic narrative; the preserved engineering baseline remains available as a reference.

---
## Repository versions and engineering history

[[REPO]]

[[VIS:version-map]]

| Branch or workstream | Purpose | Interpretation |
|---|---|---|
| main | RP2350 flight_controller.c and RP2040 handler_controller.c; Pico SDK; ROS 2 packages | Intended architecture; prototype source is not hardware/flight acceptance |
| esp32-xbox | ESP32-S3 / Xbox experimental implementation | Alternative prototype; do not transfer its platform-specific assumptions silently |
| codex/stepwise-esp32 | Incremental sensor/calibration experiments and current report | ESP32 evidence retains board, source and configuration identity |
| ROS 2 / Gazebo | Companion, perception, mission and simulation packages | Shared research workstream; each test must name its actual engine and command path |

**Regular version control:** each meaningful completed change receives a descriptive local commit and a GitHub push after relevant checks. Preserve failed logs and earlier versions. If a step fails, repair it in a new commit and revalidate its dependants; do not rewrite history or silently reuse old acceptance. Conversations alone do not need empty commits.

**Report lineage:** v2.1 detailed engineering baseline -> v3 visual mathematical edition -> v4 Pico-target system edition. Earlier documents remain in reference files and Git history. The repository is public; credentials, environments and unrelated personal material are excluded. Source/code baselines and hashes are listed separately from measured results.

---
## RP2350 flight controller and RP2040 handheld

[[VIS:pico-responsibilities]]

[[VIS:handheld-chain]]

**Aircraft:** RP2350 Cortex-M33 target, Pico SDK, one flight_controller.c. Core 0 is proposed for the bounded 1 kHz estimation/control path; core 1 handles radio and slower work through timestamped queues. Each bus has one owner; slow requests sharing the IMU bus must be scheduled without blocking the critical path. RP2350 provides hardware single-precision floating point [R10].

**Handheld:** RP2040, Pico SDK, one handler_controller.c; two centred joysticks, LB/RB, LT/RT, additional direction/mode buttons and a display. Neutral calibration, dead zone, filtering, debounce, sequence and link-health handling precede transmission. Six analogue inputs require an acquisition solution: the existing prototype uses a 74HC4051 multiplexer; settling and channel calibration need physical checks. Display, power source and final gesture mapping remain to select.

**Actual source status [P6]:** the Pico FC scaffold still declares a 500 Hz loop and stub radio/sensor/output interfaces; the report's 1 kHz target therefore requires implementation and fresh RP2350 profiling. Neither the existing scaffold nor ESP32 measurements prove this target. Physical outputs remain disabled.

Preliminary calibration informs the design but is not transferred blindly: changing mounting, range, scale or acquisition settings requires affected checks to be repeated.

---
## LoRa command link and reserved CC1101 channels

[[VIS:radio-budget]]

[[EQ:T_{sym}=\frac{2^{SF}}{BW},\qquad T_{link}=T_{air}+T_{queue}+T_{decode},\qquad T_{age}<T_{valid}]]

One LoRa module on each side carries the principal pilot-command link. Two CC1101 modules on each side are reserved for secondary data links; their roles are not implemented or assumed safety redundancy. SPI/UART details depend on the chosen LoRa module; CC1101 is an SPI-controlled sub-GHz transceiver [R11].

**Interface contract:** version, sequence, normalized sticks/triggers, buttons, CRC and validity age. The legacy Pico command frame is 22 bytes; future mode/mission changes require an explicit versioned extension. The LoRa cadence is selected from measured packet airtime, interference and required command age, not from the 1 kHz inner-loop rate. The FC holds bounded references between fresh packets.

**Acceptance:** exact modules, antennas, band, power supply, command rate, loss bursts, delay, reconnect and radio coexistence. Multiple radios sharing power, spectrum or antennas are not automatically independent. Stale commands must be rejected; the flight-phase loss response remains to approve. Local RF operating constraints must be checked before transmission.

---
## Flight behaviour and control architecture

[[VIS:control-cascade]]

[[EQ:e_k=r_k-y_k,\qquad u_k=K_p e_k+K_i\sum_{j=0}^{k}e_j\Delta t_j+K_d\dot e_{f,k}]]

| Pilot input | Controlled variable | Neutral behaviour |
|---|---|---|
| Right stick | Roll and pitch angles | Level attitude; no horizontal brake or position hold |
| Left horizontal | Yaw rate | Zero commanded yaw rate; no absolute-heading hold |
| Left vertical | Vertical speed | Height retention through the vertical law |
| RT / LT | Bounded increase / decrease | Gentle profile; reserve correction authority |

**Six PID blocks:** two angle, two rate, one yaw rate and one vertical-speed PID. An outer proportional height-to-speed law is an option, not a frozen implementation. Mode transitions, boost interaction and vertical saturation still require design review.

**Personal design rationale:** I chose PID control to apply the feedback-control concepts I learned: error feedback, proportional response, integral rejection of persistent error, derivative damping, stability and sampling. Its interpretable gains let me connect mathematics to measurements. This is an educational and engineering choice, not a claim that PID is optimal for every flight condition. Cascades, filtering and saturation must still be justified experimentally.

---
## F450 geometry and force balance

[[VIS:geometry-forces]]

[[EQ:L=0.225\ \mathrm{m},\qquad a=\frac{L}{\sqrt{2}}\approx0.1591\ \mathrm{m}]]

[[EQ:T=\sum_{i=1}^{4}T_i,\quad m\dot{\mathbf v}=R\mathbf e_3T-mg\mathbf e_3-D\mathbf v+\mathbf f_{ext}]]

[[EQ:J\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times(J\boldsymbol\omega),\qquad\boldsymbol\tau_i=\mathbf r_i\times\mathbf F_i]]

**Meaning:** sum the rotor forces in one reference frame; subtract gravity; the remaining force accelerates the mass. Lever arms convert each rotor force into roll/pitch torque. Rotor reaction torque contributes yaw. The cross term is angular-momentum coupling in rotating body axes.

**Frame proposal:** ENU world, FLU body. Confirm calibration conventions and physical motor corners before mixing. Geometry is nominal; CW/CCW signs are not assigned by this illustration.

---
## Hover operating point and uncertain parameters

[[VIS:hover-sensitivity]]

[[EQ:T_0=mg\approx10.179\ \mathrm{N},\qquad T_{i,0}\approx2.545\ \mathrm{N}\quad(m=1.038\ \mathrm{kg})]]

| Parameter | Supplied model value | Qualification |
|---|---|---|
| Mass | 1.038 kg; stated sigma 0.0160312 kg | Simulation input; verify mass and covariance assumptions |
| Jxx, Jyy | 0.006421 kg m squared | Approximate inertia about centre of mass |
| Jzz | 0.011853 kg m squared | Products of inertia not established |
| Battery | 3S 4200 mAh; marked 120C | 320 g in parameter file versus old rough 150 g estimate; weigh |
| Propulsion | 1400 KV; 9047; 40 A ESCs | Exact motor and thrust/command response unknown |

[[EQ:J_{xx}=\sum_i[J_{xx,i}^{COM}+m_i(y_i^2+z_i^2)],\qquad\Sigma_y\approx G\Sigma_pG^T]]

**Measurement gate:** complete mass, centre of mass, motor positions, thrust curve, actuator lag, torque coefficient and power demand. KV, ESC amperage and C markings cannot supply those measurements. Propulsion tests require separate authorization and a suitable fixture.

---
## Linear model and intuitive stability

[[VIS:damping-poles]]

[[EQ:G_\phi(s)=\frac{1}{J_{xx}s^2},\quad G_\theta(s)=\frac{1}{J_{yy}s^2},\quad G_r(s)=\frac{1}{J_{zz}s},\quad G_z(s)=\frac{1}{ms^2}]]

[[EQ:G_{act}(s)=\frac{1}{\tau_m s+1},\qquad s^2+2\zeta\omega_n s+\omega_n^2=0]]

[[EQ:K_p=J\omega_n^2,\qquad K_d=2\zeta\omega_nJ\quad\mathrm{(ideal\ torque\ PD\ example)}]]

These transfer functions are local hover approximations, not global flight models. The step-response plot illustrates damping; it does not identify drone gains. Add measured actuator lag, filtering, delays, voltage effects and saturation before design acceptance.

**Proposed small-signal targets:** phase margin >=45 degrees; gain margin >=6 dB; overshoot <=10 percent; settling within 5 percent <=1 s. Final six-loop numerical gains remain TBD pending identification. Compare the local model with nonlinear physics rather than assuming the two are identical.

---
## Discrete controller and sample timing

[[VIS:sample-update]]

[[EQ:\dot{\mathbf x}=A\mathbf x+B\mathbf u\ \Longrightarrow\ \mathbf x_{k+1}=A_d\mathbf x_k+B_d\mathbf u_k]]

[[EQ:A_d=e^{A\Delta t},\qquad B_d=\int_0^{\Delta t}e^{A\tau}B\,d\tau]]

[[EQ:\phi_{k+1}=\phi_k+\Delta t\,p_k+\frac{\Delta t^2}{2J_{xx}}\tau_{x,k},\qquad p_{k+1}=p_k+\frac{\Delta t}{J_{xx}}\tau_{x,k}]]

[[VIS:discrete-poles]]

**Target period:** 1 ms. Timestamp acquisition, estimation, PID and output separately. Slow sensors update at their own cadence; an old sample is not new because the controller reads it again. ESC command rate depends on the actual ESC protocol, not its 40 A rating.

**Implementation rule:** coherent state snapshots, bounded queues/timeouts and one owner per I2C bus. A full MPU reset requires bypass restoration before accessing downstream sensors.

---
## Calibration experimental design

[[VIS:calibration-protocol]]

[[EQ:\mathbf r_j=\mathbf b+A\mathbf a_j+\boldsymbol\epsilon_j,\qquad\mathbf a_j\in\{\pm g\mathbf e_x,\pm g\mathbf e_y,\pm g\mathbf e_z\}]]

[[EQ:H=[\,\mathbf h_1\ \cdots\ \mathbf h_{12}\,],\quad\mathbf h_j=[1,a_{x,j},a_{y,j},a_{z,j}]^T,\quad Y=[\,\mathbf r_1\ \cdots\ \mathbf r_{12}\,]]]

[[EQ:\widehat M=YH^T(HH^T)^{-1}=[\,\widehat{\mathbf b}\ \widehat A\,],\qquad C=\widehat A^{-1}]]

**Identifiability:** H must have full row rank. Six independent gravity directions reveal offset and directional response; one level pose cannot identify the full matrix. Equal fit weights do not imply exact pose references.

**Evidence [P1]:** 18 recordings, 20 seconds each; repetitions 1-2 fit the candidate and repetition 3 is reserved. A detached mounting invalidates its associated dataset. The secured-mount data and later tilted-pose review remain distinct. Standard gravity 9.80665 m/s squared is an assumption, not a local gravimeter reading.

---
## Full matrix correction and held out residuals

[[VIS:calibration-matrix]]

[[EQ:\mathbf a_{cor}=C(\mathbf r-\mathbf b)]]

[[VIS:calibration-residuals]]

**Read the heatmap by row:** each corrected body axis may depend on all three raw axes. C includes scale/cross-axis/mounting effects in this historical fit. The bias and matrix are not interchangeable with three MPU offset registers.

**Do not recalibrate from this page.** These are historical offline-fit coefficients [P1], not a newly installed calibration. Residuals contain fixture/placement uncertainty; they are not certified sensor accuracy. Frozen firmware constants require their own recorded provenance and reset/power-cycle verification.

---
## Covariance and useful information

[[VIS:covariance-geometry]]

[[EQ:\bar{\mathbf x}=\frac1N\sum_i\mathbf x_i,\qquad S=\frac1{N-1}\sum_i(\mathbf x_i-\bar{\mathbf x})(\mathbf x_i-\bar{\mathbf x})^T]]

[[EQ:\operatorname{Cov}(\bar{\mathbf x})=\frac1{N^2}\sum_i\sum_j\operatorname{Cov}(\mathbf x_i,\mathbf x_j)]]

[[VIS:uncertainty-sampling]]

For independent samples, the last expression reduces to Sigma/N. With correlation, cross-time terms remain. More samples do not automatically remove drift, reference error or modelling bias. The density curves show widths only; neither joint Gaussianity nor independence is asserted. Report repeatability and accuracy separately.

---
## Gyro calibration and frame validation

[[VIS:boot-gyro]]

[[EQ:\boldsymbol\omega_b=R_{bs}(\boldsymbol\omega_s-\mathbf b_g),\qquad\Delta\theta_{bias}(t)=b_g t]]

[[VIS:gyro-bias-noise]]

**Bias is not scale or frame mapping.** Motionless data estimate bias/repeatability. Known positive and negative rotations about physical body X, Y and Z test sign and mapping; a measured rotation reference is additionally needed for scale.

MPU user offsets are volatile. Restore against the documented reset baseline, read back, then apply only the remaining software bias. Do not subtract the full bias twice or use the accelerometer scale matrix as a gyro rotation. Physical gyro direction/scale checks remain pending.

---
## Multisensor fusion versus a simple average

[[VIS:fusion-versus-average]]

For two unbiased independent measurements of the same scalar x, let y1 = x + e1 and y2 = x + e2, with variances R1 and R2. Choose an unbiased linear estimate xhat = (1-w)y1 + w y2; its variance is minimized by inverse-variance weighting.

[[EQ:P(w)=(1-w)^2R_1+w^2R_2,\quad\frac{dP}{dw}=0\ \Longrightarrow\ w^*=\frac{R_1}{R_1+R_2}]]

[[EQ:P^*=\frac{R_1R_2}{R_1+R_2}\le\min(R_1,R_2),\qquad P_{average}=\frac{R_1+R_2}{4}]]

[[EQ:\mathbf w^*=\frac{R^{-1}\mathbf{1}}{\mathbf{1}^TR^{-1}\mathbf{1}},\qquad P^*=(\mathbf{1}^TR^{-1}\mathbf{1})^{-1}\quad\mathrm{(full\ covariance)}]]

**Example:** sigma1 = 1 and sigma2 = 3 in the same units. A 50/50 average gives sigma = 1.581, worse than sensor 1 alone. Optimal weighting gives sigma = 0.949. The noisier sensor contributes only 10 percent. No Gaussian assumption is required for this minimum-variance linear weighting under the stated covariance model.

**Kalman connection:** a prediction is fused with an observation using uncertainty and dynamics. Gyro rate and acceleration cannot be averaged directly: their units and models differ. Integrate the gyro and derive a valid gravity-angle observation first. With equal independent static-sensor variance, the ordinary mean is already optimal among unbiased linear combinations. Known correlation requires the full positive-definite covariance R; bias or model mismatch can invalidate the predicted physical improvement.

---
## Scalar Kalman filter and trust

[[VIS:kalman-loop]]

[[EQ:\theta_k^-=\theta_{k-1}+\Delta t\,\omega_k,\quad P_k^-=P_{k-1}+Q_k,\quad\nu_k=z_k-\theta_k^-]]

[[EQ:K_k=\frac{P_k^-}{P_k^-+R_k},\qquad\theta_k=\theta_k^-+K_k\nu_k,\qquad P_k=(1-K_k)P_k^-]]

[[VIS:kalman-trust]]

| Historical scalar model [P4] | Q (rad squared / step) | R (rad squared) | Conditional limiting K |
|---|---|---|---|
| Roll | 3.5948594e-12 | 9.6143606e-6 | 0.00061129 |
| Pitch | 3.2286779e-12 | 1.3818086e-5 | 0.00048326 |

For independent discrete gyro-rate noise, angle-process variance is sigma_rate squared times dt squared. Continuous white-rate spectral density instead integrates proportionally to dt. Keep those models and units distinct. Existing candidate gains are model-dependent; stationarity and autocorrelation limit their interpretation [P4].

---
## Original recording replay

[[IMAGE:reserved]]

Original derived figure [P4], reproduced without alteration. Engine: Python float64 replay of a reserved real 1 kHz recording. This is not live sensor output, an MCU filter execution or a flight. All samples enter the analysis; displayed points are reduced for readability.

| Reserved result | Roll (degrees) | Pitch (degrees) |
|---|---|---|
| Acceleration-angle standard deviation | 0.177569 | 0.213074 |
| Filter-output standard deviation | 0.007000 | 0.005692 |
| Filter mean minus observation mean | -0.033769 | -0.039908 |

The evaluation excludes a fixed two-second initialization interval. No independently measured attitude reference exists for this recording; lower fluctuation does not demonstrate absolute accuracy. Mean differences were retained, not fitted away. The lower panel shows why independent-white-noise assumptions need checking.

---
## Why dynamic acceleration defeats a quiet filter

[[VIS:accel-failure]]

[[EQ:\theta_{apparent}=\arctan(a_{lat}/g),\qquad\|\mathbf a\|/g=\sqrt{1+(a_{lat}/g)^2}]]

At a_lat = 0.1g, apparent tilt is 5.71 degrees while the norm is only about 1.005g. A magnitude-only acceptance gate can miss the disturbance.

| Existing model case [P4] | RMSE (degrees) | Peak error (degrees) |
|---|---|---|
| No added disturbance | 0.004312 | 0.012507 |
| 0.1g acceleration | 2.114109 | 5.191244 |
| 0.1 degree/s gyro bias | 0.157612 | 0.172872 |
| Both | 2.214463 | 5.354520 |

**Decision:** evaluate innovations, bias dynamics and acceleration-sensitive correction on independent dynamic references before acceptance. These are existing Python synthetic-model results; no Gazebo or physical flight is implied. Smoothing or collecting more stationary samples cannot remove an unobservable force component.

---
## Vertical estimation and sensor geometry

[[VIS:vertical-filter]]

[[EQ:z_{k+1}=z_k+\Delta t\,v_{z,k}+\tfrac12\Delta t^2 a_{z,k},\qquad v_{z,k+1}=v_{z,k}+\Delta t\,a_{z,k}]]

[[EQ:P_k^-=A_dP_{k-1}A_d^T+Q_k,\quad K_k=P_k^-H^T(HP_k^-H^T+R_k)^{-1}]]

[[VIS:vertical-observation]]

The two-state model estimates z and vz; it does not automatically estimate accelerometer bias. Q must follow the selected disturbance model and sample interval. A mass variance cannot be copied directly into Q. Barometer drift, lidar status, tilt projection, terrain, measurement age and dropout all require tests. Bottom-versus-front lidar placement remains a decision, not an interchangeable software setting.

---
## Gentle control authority and anti windup

[[VIS:input-headroom]]

[[EQ:u_{raw}=u_P+u_I+u_D,\qquad u=\operatorname{clip}(u_{raw},u_{min},u_{max})]]

[[EQ:\dot u_I=K_i e+K_{aw}(u-u_{raw})\quad\mathrm{(one\ antiwindup\ option)}]]

**Design priorities:** bounded angle/rate requests, gentle centre response, filtered derivative, explicit saturation, attitude correction headroom and bumpless mode changes. The plotted command values and shaping curve are examples, not firmware settings or accepted ESC endpoints.

**Boost is not maximum output.** A proposed 1700-1800 microsecond collective ceiling preserves some room for mixing, but the exact relationship between RT/LT, vertical speed and collective remains unresolved. Verify signs, mixer geometry and spin directions before physical actuator work. All motor outputs remain disabled.

---
## Real time evidence including failures

[[VIS:timing-budget]]

[[EQ:T_s=1000\ \mu\mathrm s,\qquad T_{critical}+T_{reserve}\le T_s]]

**Measured history [P3]:** trial 2 was malformed and rejected; earlier trials exceeded the handler budget. Repair separated acquisition/calculation from synchronous serial formatting using a bounded queue. The later 822 microsecond maximum leaves only 178 microseconds to 1 ms if compared directly; a complete future FC includes additional work.

All measured timing on this page belongs to the **ESP32-S3 preliminary platform**, not RP2350. The main Pico scaffold's 500 Hz setting and unconnected drivers remain a separate implementation baseline [P6].

**Proposed target:** <=800 microseconds critical work, >=200 microseconds reserve, and no observed deadline miss in a defined 10-minute loaded test. Historical samples near 1000.69 Hz do not prove a periodic hard deadline. Inter-service gaps, clocks and source changes remain relevant.

**P5 build only:** 54 Python checks, 19 actual-source host cases, ESP32 compile; 19504 bytes RAM and 312901 bytes flash. Source f26d314e was not uploaded in that record. Historical timing source 91947b5c is different. A host fake peripheral is not an ESP32 timing measurement.

**Bench source configuration:** accelerometer +/-8g, gyro +/-500 degrees/s, nominal 260 Hz low-pass setting with readback checks. Sensor bandwidth is not the PID bandwidth. These settings do not validate the revised sensor wiring or a full control loop.

---
## ROS 2 software implementation and interfaces

[[VIS:ros2-implementation]]

| Boundary | Implementation contract |
|---|---|
| Ground station to companion | IP network, mission upload/approval, status and cancel; separate from LoRa pilot link |
| Camera and perception | Stamped RGB plus intrinsics; detections become metric obstacles only with validated geometry |
| Localization | Pose/velocity/covariance with coherent map, odom and body frames; freshness checked |
| Mission and navigator | Ordered-node progression, local-map updates, replanning and bounded references |
| Serial bridge to RP2350 | Versioned setpoints and health; CRC, timestamps, expiry and authority arbitration |
| Evidence recording | rosbag2/topics, video and FC logs aligned to recorded clocks and source versions |

**Existing code [P6]:** qcopter_autonomy contains C++ mission, navigator, YOLO and panel-pose nodes plus custom messages; qcopter_description contains the F450 URDF/RViz model. The existing mission service specifies start/finish only. A cancellable n-node ROS action with progress feedback is a proposed extension, not an already verified feature [R13].

**Communication design:** use small bounded histories; sensor streams prioritize freshness, mission requests need acknowledged handling, and the FC independently rejects stale commands. ROS QoS compatibility must be checked; reliable delivery alone does not guarantee a deadline [R13]. ROS 2 runs on the companion, while the RP2350 runs Pico SDK C firmware rather than a full ROS installation.

---
## YOLO to obstacle avoidance architecture

[[VIS:yolo-architecture]]

[[EQ:\mathbf z_{det}=(u_1,v_1,u_2,v_2,c,s),\qquad\mathbf d_c\propto K_c^{-1}[u,v,1]^T]]

[[EQ:U^*=\arg\min_U\sum_k\left(w_p\|\mathbf p_k-\mathbf p_{goal}\|^2+w_u\|\mathbf u_k\|^2+w_c\psi(d_k-d_{safe})\right)]]

**YOLO supplies:** bounding boxes, class and score [R7]. It does not supply complete metric free space. Camera intrinsics Kc, camera-to-body extrinsics, timestamps, tracking and uncertainty are additional contracts.

**Fusion gate:** associate a lidar ray with a visual hypothesis only when geometry, timing and field of view justify it. Unassociated or unobserved space stays unknown. No detection does not imply no obstacle. Localization must make waypoint coordinates observable before a metric mission can execute.

Panel obstacles may be absent from pretrained classes. Training data, panel recognition and a class-agnostic fallback remain to be selected and tested.

**Candidate planning objective:** penalize goal error, control effort and inadequate clearance, subject to the motion model, input bounds and observed-space constraints. U is a candidate control sequence; psi is a clearance penalty. This does not select or validate a planner. The FC remains responsible for bounded commands. Model weights, runtime, licence, speed and companion hardware remain TBD.

---
## Perception limits and stopping clearance

[[VIS:perception-limits]]

[[VIS:stopping-frames]]

[[EQ:\mathbf p_w=R_{wb}\left(R_{b\ell}[d,0,0]^T+\mathbf t_{b\ell}\right)+\mathbf t_{wb}]]

[[EQ:d_{clear}>v\tau+\frac{v^2}{2a_{brake}}+d_{margin}]]

**One RGB camera plus one fixed ray cannot observe the entire collision corridor.** Forward lidar helps one direction; downward lidar helps local height under terrain/tilt assumptions. Metric monocular reconstruction needs additional information and validation; a pixel bearing alone has ambiguous depth.

The point transform assumes the lidar ray follows its local positive x axis: rotate/translate lidar to body, then body to world. Extrinsics and pose timestamps must be known; otherwise range cannot be placed reliably in the obstacle map.

The clearance graph assumes 0.25 s total latency and 1 m/s squared braking only to explain scaling. These are not measured aircraft parameters. Reduce the operational envelope until latency, braking, localization and missed-detection behaviour are verified. Any simulator depth camera or perfect pose must be declared as extra sensing or an oracle.

---
## Missions with two three or many nodes

[[VIS:mission-state]]

[[VIS:mission-nodes]]

[[EQ:\|\widehat{\mathbf p}-\mathbf p_i\|\le r_{accept},\quad\|\widehat{\mathbf v}\|\le v_{accept}\quad\mathrm{for}\quad t\ge t_{dwell}]]

**Proposed arrival:** radius 0.5 m, dwell 1 s, speed bound TBD; adapt to localization uncertainty. Increment every intermediate node, not only the first. Final arrival and landing/termination are separate policies. The node sketch is an input mission, not a computed safe trajectory.

**Communication:** versioned framed 3.3 V UART is proposed, with length, CRC, sequence, timestamp, units, mission identity and command expiry. Proposed heartbeat 10 Hz and expiry 300 ms require review. Invalid frames do not renew validity. Companions may be absent; plugging one in never changes mode automatically. Loss reactions remain to approve.

---
## Navigation implementation and remaining gaps

[[VIS:navigation-implementation]]

[[EQ:f(n)=g(n)+h(n),\qquad h(n)=\|\mathbf p_n-\mathbf p_{goal}\|]]

[[EQ:r_{inflate}\ge r_{vehicle}+d_{stop}+k_\sigma\sigma_{position}+d_{margin}]]

**Existing algorithm [P6]:** an A* grid planner and waypoint follower exist in the portable C++ core. The inspected prototype uses 0.2 m cells, fixed 0.65 m obstacle inflation and a bounded local XY map. Those constants are prototype settings, not validated aircraft clearance or general 3D navigation. The target must account for swept propeller volume, stopping distance and localization uncertainty.

**Existing range demonstration:** known-size blue panels can supply monocular PnP range when their 1.09 x 1.69 m inner dimensions and trained detector are explicitly confirmed. This does not solve arbitrary obstacle depth. Simulated odometry/contact and any optional depth-image branch must be declared; a real localization source and unknown-space policy remain unresolved.

**Implementation sequence:** synchronize sensors and transforms -> validate metric obstacle observations -> update the local map -> replan -> track the route with bounded angle/yaw-rate/vertical references -> send through the RP2350 bridge. Iterate intermediate nodes; do not teleport or inject forces. Loss of valid pose/perception must invalidate autonomous commands under an approved response policy.

---
## Onboard computer selection remains open

[[VIS:companion-selection]]

**We are looking for either a Raspberry Pi 4/5 or an NVIDIA Jetson for the onboard ROS 2 computer. Neither is selected or procurement-confirmed.** The FC remains independently usable without it; autonomous mission execution needs a suitable companion and valid localization.

| Decision factor | Raspberry Pi candidate | Jetson candidate |
|---|---|---|
| Common software | Same portable ROS 2 C++ mission/navigation core | Same portable ROS 2 C++ mission/navigation core |
| Inference | Benchmark CPU or selected accelerator/runtime | Benchmark supported GPU/runtime for the exact Jetson |
| Integration | Camera driver, supported OS/ROS and serial bridge | Camera driver, supported OS/ROS and serial bridge |
| Flight cost | Board, storage, camera, regulator, cooling and mounts | Board, storage, camera, regulator, cooling and mounts |
| Selection evidence | Measured latency, power, mass, temperature and cost | Measured latency, power, mass, temperature and cost |

[[EQ:m_{new}=m_{baseline}+\Delta m_{FC/RF}+m_{computer}+m_{camera}+m_{power/mounts}]]

The 1.038 kg parameter set predates this target assembly. Delta m_FC/RF includes replacing the prototype controller and adding radios/antennas. Recompute mass, centre of mass and inertia. Shared C++ source does not imply identical binaries, inference backends or OS images. Select from measured end-to-end latency, localization quality, power, mass and cost, not nominal AI throughput alone.

---
## Proving that the FC really flies the simulation

[[VIS:simulation-proof]]

[[VIS:campaign-confidence]]

**Test contract:** stimulate normal handheld/radio or companion interfaces; log source hashes, messages, estimates and motor requests. Do not apply hidden pose, speed or force corrections to complete a mission. Host-compiled RP2350 C control code requires disclosed shims; it is not RP2350 instruction emulation or hardware timing evidence.

**Planned Gazebo campaign:** ten environments, five noise seeds each; five additional manual command scenarios. Preserve all failures. Video reviewed at 10-15 fps supports trajectory analysis; full-rate logs establish controller timing. No new campaign has been run for this report.

**P targets:** attitude RMS <=2 degrees, altitude RMS <=0.20 m, endpoint error <=0.5 m, zero collision in the declared campaign. They are reviewable objectives, not achievements. Zero failures in 50 independent comparable trials gives a 95 percent upper failure probability of about 5.8 percent, not zero risk.

---
## Battery estimation and state policy

[[VIS:battery-model]]

[[EQ:SOC(t)=SOC(0)-\frac1{C_{usable}}\int_0^t I(\tau)\,d\tau,\qquad V_{ADC}=V_{pack}\frac{R_{bottom}}{R_{top}+R_{bottom}}]]

**Measurements:** calibrated whole-pack voltage and current, useful capacity, voltage sag and sensor validity. The plots show ideal laws, not measured endurance or an SOC calibration. Marked 120C and four 40 A ESCs do not establish a usable total discharge current.

**Historical requested thresholds:** warning at 35 percent and motor disarm at 20 percent, without automatic safety landing. A motor cut in flight causes a fall; reconfirm this policy explicitly before any flight phase. Hysteresis, persistence, uncertainty and fault responses remain TBD. Whole-pack software monitoring does not protect individual cells.

**Current states:** BOOT -> SELF_TEST -> BENCH_READY only. Reset does not arm. Physical motor outputs remain disabled. One verified regulated avionics source; common grounds; no parallel BEC positives; propulsion current stays outside FC copper. Separate authorization is required for motor tests.

---
## Project gates risks and personal development

[[VIS:gates]]

| Risk or dependency | Engineering response |
|---|---|
| Wrong axes / signs / motor mapping | Known signed rotations and mixer checks before actuators |
| Lost data / I2C hang / stale commands | Bounded waits, validity/age checks and explicit degraded state |
| Saturation / boost / mode transition | Headroom, anti-windup and bumpless references |
| Missing hardware or deadline | Deliver the verified subset; prepare later stages offline without acceptance |
| Camera range / localization gaps | Restrict envelope; do not infer unobserved free space |

**Professional outcomes:** requirements -> model -> experiment -> interpretation -> decision. The personal reflection should explain one real difficulty, the student's action, the evidence and the change in understanding. Do not invent experience or sole authorship. Disclose library reuse and AI assistance under institutional policy.

**Management:** commit each meaningful change; preserve failed evidence; invalidate affected downstream acceptance after an earlier-stage failure. No deadline or rubric was supplied, so the schedule is gate-driven rather than an invented calendar. Add authentic contributions, cover fields, quotations and the actual rubric before submission. Reuse modular hardware and tools; follow appropriate battery, electronic-waste and etching-chemical procedures.

---
## Materials and resources

**Installed aircraft BOM:** quantities refer to one intended aircraft, not confirmed stock. Verify exact models, physical dimensions, ratings, ownership and masses. Prices are blank until dated quotations are obtained. Optional/future hardware is listed separately on the next page.

[[BOM:aircraft]]

**Selection logic:** RP2350 is the aircraft FC and RP2040 the custom handheld. One LoRa and two reserved CC1101 modules are allocated to each side. The exact radio modules, display, antennas and handheld power remain to choose. The magnetometer's actual silicon and driver must be identified rather than trusting the GY-271 board name. The ESP32/Xbox branch is historical experimental work, not the target bill of materials.

---
## Measurement equipment optional hardware and cost

[[BOM:other]]

[[EQ:C_{base}=\sum_i n_i c_i+C_{shipping}+C_{fabrication},\qquad C_{planned}=C_{base}+C_{contingency}]]

Use the CSV inventory for owned/to-buy status, vendor, currency, quote date, unit cost and measured mass. Separate shared tools from aircraft cost. A 15 percent contingency with 10/25 percent sensitivity is only a planning scenario. Total cost remains TBD; no supplier price is fabricated.

---
## Verification matrix and proposed acceptance

| Test | Coverage | Essential evidence |
|---|---|---|
| V01 | IF-01, FC-09, COM-01 | Power, ground, non-actuating outputs, companion absence |
| V02 | EST-02, IF-02 | Cold boot, reset, MPU recovery and readback |
| V03 | EST-01/03/04, MOD-01 | Independent poses, signed rotations, uncertainty |
| V04 | IF-03/04, EST-06/07 | Known range, invalid returns, removal/restart |
| V05 | RT-01/02/03/04/06 | Full-load timing, queue/loss counters and deadlines |
| V06 | FC-02/03/05/07/08, MOD-02/03/04 | Model, gains, steps, saturation, source path |
| V07 | HMI-01/02/03/04, FC-01/06, SYS-02 | RP2040 inputs, mapping, gestures and command generation |
| V08 | COM-02/03, EST-06, RT-04 | CRC, replay, stale/delayed messages |
| V09 | FC-04, EST-05 | Vertical response and lost sensor |
| V10 | NAV-01/02/05 | All mission nodes, frames and localization |
| V11 | NAV-03/04 | Off-ray/unclassified obstacles; no oracle |
| V12 | ENE-01/02/03 | Reference power measurement and threshold faults |
| V13 | RF-01/02 | LoRa airtime/age/loss/reconnect and CC1101 coexistence; reserved links remain inactive |

**Every result:** commit and source/configuration hashes, engine or hardware identity, original logs, assumptions, fixed criteria, verdict and limits. New gains or configuration invalidate incompatible acceptance. An attractive video is insufficient without the command path and data. No failure is deleted to create a pass.

**Assessment evidence:** mathematical reasoning, sourced design choices, reproducible original measurements, adverse-result interpretation, resource/risk management and authentic reflection. No guaranteed mark or MIT endorsement is claimed.

---
## Requirements register one

The historical baseline is preserved separately. This register retains the original identifiers, updates platform-specific handheld/software requirements and adds target-system/radio requirements. RP2040 gesture mapping remains TBD rather than silently inheriting Xbox A/B buttons. Detailed pinout/API syntax stays outside the main submission.

[[REQ:one]]

---
## Requirements register two

[[REQ:two]]

**Software inventory:** Pico SDK and C toolchain; one flight_controller.c and one handler_controller.c; selected radio/sensor libraries; ROS 2 C++; Gazebo physics; RViz; YOLO inference; NAssistant configuration; Python analysis; KiCad; Git/GitHub. ESP32/Arduino/Adafruit/Xbox dependencies belong to preliminary evidence only. Pin versions, licences, model hashes and commands. Isaac Sim is excluded.

**Evidence access:** original private recordings must be made available to the examiner through an appropriate sanitized evidence package. Public summaries/hashes are not substitutes for the originals. The supplied model parameters are not independently measured aircraft properties.

---
## References and evidence provenance

**Primary technical sources:** [R1] NASA systems-engineering verification/validation: https://www.nasa.gov/reference/2-0-fundamentals-of-systems-engineering/ . [R2] Espressif ESP32-S3 datasheet and BLE support: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf . [R3] TDK MPU6050 register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf . [R4] Bosch BMP280 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf .

[R5] Nooploop TOFSense-F V2.0, hardware and protocol: https://support.nooploop.com/tofsense/ . [R6] Historical Xbox BLE prototype library: https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 . [R7] Ultralytics detector outputs: https://docs.ultralytics.com/tasks/detect/ . [R8] MIT OCW 16.30 feedback control: https://ocw.mit.edu/courses/16-30-feedback-control-systems-fall-2010/ . [R9] ROS frame conventions: https://www.ros.org/reps/rep-0105.html . Reviewed 28 October 2026; sources support methods, not acceptance of this aircraft.

[R10] Raspberry Pi RP2350 and RP2040 specifications: https://www.raspberrypi.com/products/rp2350/ and https://www.raspberrypi.com/documentation/microcontrollers/microcontroller-chips.html . [R11] TI CC1101 transceiver: https://www.ti.com/product/CC1101 . [R12] Semtech LoRa airtime trade-off: https://www.semtech.com/design-support/faq/faq-lora . [R13] ROS 2 action and QoS contracts: https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Actions.rst and https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst .

---
## Implementation evidence and report provenance

**Project evidence:** [P1] measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md and analysis.json; [P2] measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md; [P3] measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md; [P4] measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md and analysis.json; [P5] measurements/software/2026-10-27-input-guard/summary.json. Paths are relative to the Stepwise checkout. Evidence-manifest.json records source hashes and repository links.

**[P6] Inspected prototype code:** main commit bce665ad97adbfcb282f02970a1db599e67e6479, firmware/flight_controller/flight_controller.c, firmware/handler_controller/handler_controller.c and companion/ros2_ws/src/qcopter_autonomy. Read-only source inspection supports the implementation inventory; no new Pico build, upload, radio test or ROS flight is claimed. source-inventory.json records exact file hashes and repository links.

**Figure provenance:** assets/visual/figure-manifest.json identifies every chart as a historical measured summary, replay/model result, analytic example, schematic or proposal. Model-only plots use explicit assumptions and do not represent Gazebo or hardware tests. The F450 artwork is AI-generated using the built-in image tool; image-provenance.json retains its prompt and origin. It depicts a plausible red/white F450, not the student's assembled hardware.

**Preserved detail:** reference/engineering-baseline-v2.1.md and reference/visual-edition-v3.md retain the earlier ESP32-oriented documents. Their pin maps and Xbox-specific gestures are historical, not the Pico target interface. This revision updates the intended architecture and explanatory design; it neither enables motors nor accepts RP2350 timing, radio delivery or autonomous flight. No firmware or calibration constant is changed.
