# IntelQCopter
## Autonomous Quadcopter

**USTHB — Faculty of Electrical Engineering (FGE)**
**Department of Automatic Control — Robotics and Intelligent Systems (RSI)**
**Fourth-year engineering | Personal and Professional Project (RSI7.8)**
**27 October 2026 | Academic year 2026-2027**


---
## IntelQCopter in one page

[[VIS:project-brief]]

**Purpose:** build a quadcopter that flies stably under pilot commands, then navigates between approved waypoints while avoiding observed obstacles. **Learning:** apply feedback control, identification, estimation and experimental validation to one integrated engineering project.

| Reading route | Essential question | Pages |
|---|---|---|
| 1. Flight controller | How does the aircraft sense, estimate and stabilize? | 3-10 |
| 2. ROS 2 and YOLO | How does the onboard computer perceive and navigate? | 11-13 |
| 3. Gazebo and RViz | How do the model and actual software get tested? | 14-15 |
| 4. Resources | What hardware, software and development method are needed? | 16-17 |
| 5. Handheld and communication | How do pilot and companion commands reach the FC? | 18-19 |

**Target:** RP2350 aircraft; RP2040 handheld. Stable manual flight works without the companion. Raspberry Pi 4/5 or NVIDIA Jetson is being sought for autonomous operation.

**Evidence:** secured sensor recordings and offline analyses exist; Pico integration and autonomous flight remain unvalidated. Figures distinguish recorded evidence, analytic examples and proposed designs. Physical motor outputs remain disabled.

**Notation:** bold = vector/matrix; dot = time derivative; hat = fitted estimate; bar = sample mean; T superscript = transpose; -1 superscript = inverse; || || = vector length. Symbols are defined locally; the same letter can have a different role on another page.

---
## 1. Flight controller | F450 physics and geometry

[[VIS:frames-geometry]]

[[EQ:a=L\cos45^\circ=\frac{L}{\sqrt{2}}=0.1591\ \mathrm{m},\quad L=0.225\ \mathrm{m}]]

[[EQ:m\dot{\mathbf v}=R\mathbf e_3T-mg\mathbf e_3-D\mathbf v+\mathbf f_{ext}]]

**R rotates directions:** R is a dimensionless 3 x 3 rotation matrix; thrust points along the drone's upward axis; R expresses that axis in the world frame. At level hover, R e3 = e3 and T = mg. **D damps motion:** D = diag(dx,dy,dz) is a local linear drag approximation, not a measured constant. Drag is -D v; each d has units N s/m.

**L is an arm length, not the frame diagonal:** nominal 450 mm opposite-motor spacing gives L = 225 mm from centre to motor. On an X frame, the two horizontal projections are equal: L² = a² + a², hence a = L/sqrt(2). Coordinates (±a,±a) determine torque lever arms.

**Symbols:** m: mass (kg); v: world velocity (m/s); T: total thrust (N); e3 = [0,0,1] transpose; g: gravity (9.80665 m/s² assumed); f_ext: disturbance force (N). World: ENU; body: FLU; mounting and signs require physical checks. Here R is rotation, not sensor covariance.

---
## 1.1 Physics | torque and PID design model

[[VIS:damping-poles]]

[[EQ:T=\sum_{i=1}^4T_i,\quad\boldsymbol\tau_i=\mathbf r_i\times\mathbf F_i,\quad J\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times(J\boldsymbol\omega)]]

[[EQ:J_{xx}\ddot\phi\simeq\tau_x,\quad J_{yy}\ddot\theta\simeq\tau_y,\quad J_{zz}\dot r\simeq\tau_z,\quad m\ddot z\simeq\Delta T]]

[[EQ:G_\phi(s)=\frac{1}{J_{xx}s^2},\quad G_\theta(s)=\frac{1}{J_{yy}s^2},\quad G_r(s)=\frac{1}{J_{zz}s},\quad G_z(s)=\frac{1}{ms^2}]]

| Supplied simulation inputs | Value | Interpretation |
|---|---|---|
| m; stated mass standard deviation | 1.038 kg; 0.0160312 kg | Not independently measured |
| Jxx = Jyy; Jzz | 0.006421; 0.011853 kg m² | Approximate COM inertias |
| Hover total / per rotor | 10.179 / 2.545 N | mg, assuming equal loading |

**Symbols:** i: rotor; r_i: lever-arm vector (m); F_i: rotor force (N); τ: torque (N m); J: inertia matrix (kg m²); ω: angular velocity (rad/s); φ/θ: roll/pitch (rad); r: yaw rate (rad/s); z: height (m); ΔT = T - mg; s: Laplace variable (1/s); G: input-to-output transfer function. Rotor reaction torque supplies yaw.

**Gain example:** for one ideal torque-controlled axis with PD feedback, J here is the selected axis inertia; matching the closed-loop denominator gives Kp = J ωn² and Kd = 2 ζ ωn J. Here Kp is in N m/rad and Kd in N m s/rad. These are design relations, not identified flight gains.

**Design scope:** zero-initial-condition Laplace transform gives Jxx s² Φ = Τx, hence Gφ = Φ/Τx. Φ and Τx are the Laplace transforms of φ and τx. Hover linearization assumes small angles/rates and principal axes; it is not a global model. Identify actuator lag Gact = 1/(τm s + 1), where τm is motor/ESC time constant (s), plus delays, thrust curve and saturation. Final gains remain pending those measurements. Plot: educational damping example; ζ is damping ratio, ωn natural frequency.

---
## 1.2 Control | cascaded PID and digital execution

[[VIS:pid-feedback]]

[[EQ:e_k=r_k-y_k,\quad u_k=K_p e_k+I_k+K_d\frac{e_k-e_{k-1}}{\Delta t},\quad I_{k+1}=I_k+K_i e_k\Delta t]]

[[EQ:u=\operatorname{clip}(u_{raw},u_{min},u_{max}),\quad I_{k+1}\leftarrow I_{k+1}+K_{aw}(u-u_{raw})\Delta t]]

| Command | Feedback law | Centred-stick behaviour |
|---|---|---|
| Roll / pitch | Angle PID -> angular-rate PID | Level attitude; no horizontal brake/position hold |
| Yaw | Angular-rate PID | Zero commanded yaw rate |
| Vertical | Height reference -> vertical-speed PID | Maintain height through vertical loop |
| RT / LT | Bounded increase / decrease | Gentle profile; retain correction authority |

**Why PID:** I chose it to apply learned error feedback, stability, integral error rejection and derivative damping. Six PID blocks: 2 angle + 2 rate + 1 yaw-rate + 1 vertical-speed; the outer height-to-speed law remains to finalize.

**Symbols:** k: sample index; Δt: actual interval (s); r/y/e: reference/output/error; u: requested actuation; I: integral contribution; Kp/Ki/Kd: loop gains; Kaw: anti-windup gain (1/s); clip: enforce bounds. Filter the derivative in implementation; the difference above explains its unfiltered principle.

**1 kHz target:** timestamp acquisition -> estimate -> PID -> mixer. Proposed collective ceiling 1700-1800 µs reserves PID headroom; it is not measured thrust. Proposed margins: phase >=45°, gain >=6 dB; overshoot <=10%; settling within 5% <=1 s. Numerical gains require identification and closed-loop tests.

---
## 1.3 Sensors | six-pose calibration explained

[[VIS:calibration-fit]]

[[EQ:\mathbf r_j=\mathbf b+A_s\mathbf a_j+\boldsymbol\epsilon_j,\quad\widehat M=[\widehat{\mathbf b}\ \widehat A_s],\quad\mathbf a_{cor}=\widehat A_s^{-1}(\mathbf r-\widehat{\mathbf b})]]

[[EQ:\mathbf h_j=[1,a_{x,j},a_{y,j},a_{z,j}]^T,\quad H=[\mathbf h_1\ \cdots\ \mathbf h_N],\quad Y=[\mathbf r_1\ \cdots\ \mathbf r_N]]]

[[EQ:Y\simeq MH,\quad\widehat M=YH^T(HH^T)^{-1}]]

**What are H, Y and M-hat?** H stores the known reference poses, one column per pose; Y stores the corresponding measured three-axis means. M-hat is the fitted 3 x 4 coefficient table: its first column is bias b; its last three columns are sensor response As. The hat means estimated, not exact. For N fit poses: H is 4 x N; Y is 3 x N.

**Why this fit?** Choose M to minimize the sum of squared residuals ||Y - MH||². Differentiating and setting the gradient to zero gives M H H-transpose = Y H-transpose. Full row rank of H allows inversion, producing the formula. Then invert As to correct all three axes together.

**Symbols:** j: pose; r_j: raw acceleration; a_j: known gravity reference (m/s²); ε_j: residual; b: bias (m/s²); As: dimensionless response; a_cor: corrected acceleration. Six orientations ±g along X/Y/Z reveal offsets, scales and cross-axis effects. One pose cannot.

**Sensor roles:** MPU6050 measures acceleration/rate; BMP280 supplies pressure-derived altitude; GY-271 supplies a magnetic heading observation after chip identification/calibration; lidar supplies one range ray.

**Evidence [P1]:** 18 x 20 s recordings; repetitions 1-2 fit, third held out. Fixture uncertainty remains. MPU registers are volatile: restore/read back at boot; do not subtract bias twice. Full matrix correction stays in software. Gyro needs separate motionless bias plus known signed-rotation checks; those rotation checks remain pending.

---
## 1.4 Estimation | why weighted fusion beats averaging

[[VIS:fusion-versus-average]]

For two unbiased independent measurements of the same scalar x, let y1 = x + e1 and y2 = x + e2, with variances R1 and R2. Choose an unbiased linear estimate xhat = (1-w)y1 + w y2; its variance is minimized by inverse-variance weighting.

[[EQ:P(w)=(1-w)^2R_1+w^2R_2,\quad\frac{dP}{dw}=0\ \Longrightarrow\ w^*=\frac{R_1}{R_1+R_2}]]

[[EQ:P^*=\frac{R_1R_2}{R_1+R_2}\le\min(R_1,R_2),\qquad P_{average}=\frac{R_1+R_2}{4}]]

[[EQ:\mathbf w^*=\frac{R^{-1}\mathbf{1}}{\mathbf{1}^TR^{-1}\mathbf{1}},\qquad P^*=(\mathbf{1}^TR^{-1}\mathbf{1})^{-1}\quad\mathrm{(full\ covariance)}]]

**Symbols:** x: true scalar; y1/y2: observations; e1/e2: errors; R1/R2: error variances (measurement units squared); w: weight of sensor 2; P: estimate-error variance; σ: standard deviation; R in the matrix formula: full observation-error covariance; bold 1: vector of ones; bold w: weight vector; *: minimum-variance choice; P_average: equal-weight variance.

**Example:** sigma1 = 1 and sigma2 = 3 in the same units. A 50/50 average gives sigma = 1.581, worse than sensor 1 alone. Optimal weighting gives sigma = 0.949. The noisier sensor contributes only 10 percent. No Gaussian assumption is required for this minimum-variance linear weighting under the stated covariance model.

**Kalman connection:** a prediction is fused with an observation using uncertainty and dynamics. Gyro rate and acceleration cannot be averaged directly: their units and models differ. Integrate the gyro and derive a valid gravity-angle observation first. With equal independent static-sensor variance, the ordinary mean is already optimal among unbiased linear combinations. Known correlation requires the full positive-definite covariance R; bias or model mismatch can invalidate the predicted physical improvement.

---
## 1.5 Estimation | attitude and height Kalman filters

[[VIS:estimator-control]]

[[EQ:\theta_k^-=\theta_{k-1}+\omega_k\Delta t,\quad K_k=\frac{P_k^-}{P_k^-+R_k},\quad\theta_k=\theta_k^-+K_k(z_k-\theta_k^-)]]

**Attitude:** integrate gyro rate to predict angle; correct using an accelerometer-derived gravity angle. P-minus = prior angle-error variance; R = observation variance; K = dimensionless trust. θ/z are estimated/observed angles (rad); ω is gyro rate (rad/s); k is sample; Δt is interval (s). Superscript minus means prediction. Prediction variance adds process uncertainty Q each step.

[[EQ:h_k^-=h_{k-1}+v_{k-1}\Delta t+\tfrac12 a_k\Delta t^2,\quad v_k^-=v_{k-1}+a_k\Delta t]]

[[EQ:K_h=\frac{P_{hh}^-}{P_{hh}^-+R_h},\quad K_v=\frac{P_{vh}^-}{P_{hh}^-+R_h},\quad h_k=h_k^-+K_h\nu_k,\quad v_k=v_k^-+K_v\nu_k]]

**Height:** ν = measured height - predicted height. h/v/a = world height (m), vertical speed (m/s), gravity-compensated acceleration (m/s²). Phh = height-error variance (m²); Pvh = speed/height error covariance (m²/s); Rh = height-observation variance (m²); Kh dimensionless; Kv in 1/s. Propagate both uncertainties and their covariance; this joint height-speed estimate feeds the vertical PID.

**Sensor limits:** reject invalid/stale observations; barometer drift, lidar tilt/terrain and dropouts need tests. Bottom-versus-front lidar placement remains open. Historical scalar K values [P4], roll 0.00061129/pitch 0.00048326, are conditional stationary-model values, not accepted flight tuning.

---
## 1.6 Evidence | real sensor replay and its limits

[[IMAGE:reserved]]

| Reserved replay [P4] | Roll | Pitch |
|---|---|---|
| Raw angle standard deviation | 0.177569 degrees | 0.213074 degrees |
| Filter output standard deviation | 0.007000 degrees | 0.005692 degrees |
| Filter minus observation mean | -0.033769 degrees | -0.039908 degrees |

**Engine:** original Python float64 replay of reserved real 1 kHz data; unchanged plot. Not MCU execution or flight. Fixed 2 s initialization excluded. No independent attitude reference: quieter output does not prove accuracy.

**Adverse test [P4]:** synthetic 0.1g lateral acceleration produced 5.191 degrees peak error; acceleration plus gyro bias produced 5.355 degrees. Apparent gravity tilt is arctan(0.1) = 5.71 degrees while magnitude is only 1.005g. Norm gating alone is insufficient. Dynamic acceptance remains blocked.

---
## 1.7 Energy | battery estimation and avionics safety

[[VIS:battery-model]]

[[EQ:SOC(t)=SOC(0)-\frac1{C_{usable}}\int_0^t I(\tau)\,d\tau,\qquad V_{ADC}=V_{pack}\frac{R_{bottom}}{R_{top}+R_{bottom}}]]

**Symbols:** SOC: remaining usable-charge fraction; SOC(0): initial fraction; t/τ: elapsed/dummy time (s); I: discharge current (A, positive out); C_usable: usable charge (A s; multiply Ah by 3600); V_ADC/V_pack: divider output/pack voltage (V); R_top/R_bottom: upper/lower divider resistances (ohms).

**Measurements:** calibrated whole-pack voltage and current, useful capacity, voltage sag and sensor validity. The plots show ideal laws, not measured endurance or an SOC calibration. Marked 120C and four 40 A ESCs do not establish a usable total discharge current.

**Historical requested thresholds:** warning at 35 percent and motor disarm at 20 percent, without automatic safety landing. A motor cut in flight causes a fall; reconfirm this policy explicitly before any flight phase. Hysteresis, persistence, uncertainty and fault responses remain TBD. Whole-pack software monitoring does not protect individual cells.

**Current states:** BOOT -> SELF_TEST -> BENCH_READY only. Reset does not arm. Physical motor outputs remain disabled. One verified regulated avionics source; common grounds; no parallel BEC positives; propulsion current stays outside FC copper. Separate authorization is required for motor tests.

---
## 2. ROS 2 and YOLO | onboard computer architecture

[[VIS:ros-pipeline]]

**Computer sought:** Raspberry Pi 4/5 or NVIDIA Jetson, with RGB camera. Neither is selected. The aircraft FC remains autonomous as a stabilizer without this computer; the computer adds mission planning and obstacle avoidance, not motor-level control.

| Layer | Role | Remaining decision |
|---|---|---|
| Camera and YOLO | Timestamped images; object hypotheses | Camera calibration, trained panel data, model/runtime |
| Localization and mapping | Pose; metric obstacle observations; unknown space | Real localization source, scale and uncertainty |
| Planner and route follower | Replan around obstacles; track approved nodes | Dynamic clearance, expiry and loss policies |
| FC bridge | Bounded angle/yaw-rate/vertical references | Physical serial acceptance and timing |

**Portable implementation [P6]:** C++ mission, navigator, YOLO and panel-pose nodes, custom messages and F450 description exist. Shared source does not imply identical Pi/Jetson binaries, camera drivers or acceleration backends. Benchmark end-to-end latency, power, mass, cooling and cost before selection.

**Mass consequence:** adding camera, computer, regulator, cooling and mounts changes mass, centre of mass and inertia. Recompute the supplied 1.038 kg model before gain validation. ROS 2 runs on the companion; RP2350 uses Pico SDK C.

---
## 2.1 YOLO | detection, depth and obstacle clearance

[[VIS:yolo-architecture]]



[[EQ:\mathbf d_c\propto K_c^{-1}[u,v,1]^T,\quad d_{clear}>v\tau+\frac{v^2}{2a_{brake}}+d_{margin}]]

**Symbols:** d_c: camera-frame ray direction; K_c: intrinsic camera matrix; u/v inside [u,v,1]: pixel coordinates; d_clear: available clearance (m); v in the stopping law: speed (m/s); τ: total latency (s); a_brake: available braking deceleration (m/s²); d_margin: clearance allowance (m). These local symbols differ from control input u and world velocity vector v.

**YOLO [R7]:** class, box and confidence; not metric free space. Camera calibration, timestamps, tracking and camera/body transforms are required. Associate the single lidar ray only when geometry/time justify it. Unobserved space stays unknown; no detection is not proof of clearance.

**Hardware limit:** one RGB camera + one fixed ray cannot observe the full collision corridor. Downward lidar aids altitude; forward lidar aids one direction. Placement remains open. Existing known-panel PnP assumes a trained detector and confirmed 1.09 x 1.69 m panel dimensions; it does not recover arbitrary obstacle depth.

---
## 2.2 Navigation | approved nodes and online replanning

[[VIS:mission-nodes]]

[[VIS:navigation-implementation]]

[[EQ:f(n)=g(n)+h(n),\quad r_{inflate}\ge r_{vehicle}+d_{stop}+k_\sigma\sigma_{position}+d_{margin}]]

[[EQ:\|\widehat{\mathbf p}-\mathbf p_i\|\le r_{accept},\quad\|\widehat{\mathbf v}\|\le v_{accept}\quad\mathrm{for}\quad t\ge t_{dwell}]]

**Symbols:** n: search node; f: A* priority; g(n): accumulated path cost, not gravity; h(n): remaining-cost estimate; r_inflate/r_vehicle: map inflation/vehicle radius (m); d_stop/d_margin: stopping distance/margin; kσ: uncertainty multiplier; σ_position: position standard deviation (m); p-hat: estimated position; p_i: node i target; v-hat: estimated velocity; r_accept/v_accept: arrival limits; t/t_dwell: time within limits/required duration.

**Target:** PC chooses n nodes -> approves mission -> companion localizes, maps, replans and tracks -> FC receives bounded references. Intermediate-node advancement must be tested; final arrival and landing are separate policies.

**Existing [P6]:** portable A* and waypoint follower; 0.2 m grid, fixed 0.65 m inflation, bounded XY map. These are prototype assumptions, not validated 3D clearance. Current ROS service accepts start/finish only; a cancellable n-node action remains proposed. Proposed arrival: 0.5 m radius and 1 s dwell, speed bound TBD.

---
## 3. Gazebo and RViz | model and test environment

[[VIS:garden-model]]

| Asset or tool | What it represents | What it cannot prove alone |
|---|---|---|
| F450 URDF / RViz | Links, visual geometry, sensor frames, transforms | RViz is a viewer, not a flight physics engine |
| Gazebo physics model | Mass, inertia, collisions, rotor forces and sensor models | Assumed parameters are not measured hardware |
| Garden scenario | Panels at varied locations; mission nodes | Illustration is not an accurate USTHB reconstruction |
| Camera / lidar simulation | Observable images and one lidar ray | Perfect depth/pose must not be granted silently |

**Model input:** m = 1.038 kg; Jxx = Jyy = 0.006421, Jzz = 0.011853 kg m²; nominal F450 diagonal 450 mm; 1400 KV motors/9047 props/40 A ESCs. Thrust curve, motor lag, exact battery/companion mass and drag remain unmeasured.

**Construction:** visual/collision geometry -> measured mass/inertia -> motor dynamics -> sensor noise/delay -> actual FC algorithm -> test command sources. URDF supports visualization; the Gazebo model/plugins must independently implement forces and sensors.

**Figure:** illustrative 3D garden and F450 geometry generated for this report; not a Gazebo screenshot, rendered mission or reconstruction. Panel locations are test parameters, hidden from the navigation input.

---
## 3.1 Validation | prove the actual FC drives the plant

[[VIS:simulation-proof]]

| Campaign | Stimulus | Required evidence |
|---|---|---|
| Manual: 5 scenarios | Simulated pilot messages only | Actual FC estimates/PID/motor requests; takeoff, motion, landing |
| Autonomous: 10 environments x 5 seeds | Approved mission nodes only | Companion perception/planning -> FC setpoints -> plant |
| Fault injection | Noise, stale data, link loss, invalid ranges | Bounded response; explicit verdict; retain failures |

**No hidden help:** no pose teleportation, injected route-following forces or script that bypasses the FC. Record source/configuration hashes, command timestamps, sensor inputs, controller states, motor requests and plant truth. Truth is available for scoring, not secretly fed to autonomy. Host C execution is not MCU instruction emulation or timing proof.

**Proposed targets:** attitude RMS <=2°, height RMS <=0.20 m, endpoint <=0.5 m; zero collisions in the declared campaign. RMS means root mean square of tracking error. These are targets, not achieved results. No new Gazebo campaign was run for this edition. Video at 10-15 fps aids review; full-rate logs establish timing.

**Timing evidence [P3]:** historical ESP32 acquisition handler maxima 814-822 µs; earlier failed trials 1381/1249 µs retained. This is neither full FC timing nor RP2350 profiling. The inspected Pico scaffold remains 500 Hz with hardware stubs; 1 kHz requires fresh integration/load tests.

**Progression rule:** power -> calibration/frame -> estimation/timing -> PID/mixer -> manual -> navigation. Failure invalidates affected downstream acceptance. No physical motor authorization is granted by this report.

---
## 4. Resources | hardware and components

| Subsystem | Intended materials | Selection status |
|---|---|---|
| Airframe and propulsion | F450 red/white; 4 x 1400 KV motors; 4 x 9047 props; 4 x 40 A ESC | Exact motor/ESC models and dynamics to verify |
| Power | 3S 4200 mAh LiPo, marked 120C; verified avionics regulator; voltage/current sensing | Actual capacity, sag, mass and current limits to measure |
| Aircraft control | RP2350 board, MPU6050, BMP280, GY-271 magnetometer, TOFSense F2 mini | Exact boards/compass silicon and drivers to confirm |
| Handheld | RP2040 board; 2 sticks; 2 triggers; buttons; display; power; 74HC4051 | Display/power/mux validation open |
| Radio totals | 2 LoRa; 4 CC1101; 6 matched antennas | Exact modules/band/antennas open; CC1101 reserved |
| Autonomy option | Pi 4/5 OR Jetson, RGB camera, storage, regulator, cooling, mounts | Being sought; not purchased/selected |
| Integration and measurement | PCB/headers/wire; multimeter; scale; level/fixtures; reference instruments | Full 37-entry inventory in CSV |

[[EQ:C_{base}=\sum_i n_i c_i+C_{shipping}+C_{fabrication}]]

**Symbols:** C_base: base project cost; i: inventory item; n_i: quantity; c_i: unit price; C_shipping/C_fabrication: shipping/fabrication costs. All cost terms use one stated currency; no numeric total exists until quotations are entered.

**Budget:** no fabricated prices. Dated quotes, owned/to-buy status, shipping and measured mass belong in bill-of-materials.csv. Shared tools and optional companion are costed separately.

**Mass update:** replace prototype FC mass; add radios, antennas, camera, computer, regulator and mounts. Recompute centre of mass and inertia before gain validation.

---
## 4.1 Resources | software and professional method

| Software | Project use | Engineering output |
|---|---|---|
| Pico SDK / C toolchain / CMake | RP2350 FC; RP2040 handheld | Reproducible firmware build and pinned versions |
| ROS 2 / C++ | Companion mission, navigation, bridge | Packages, interface contracts, bounded message age |
| YOLO inference runtime | Camera detections | Model hash, training scope, measured latency |
| Gazebo | Dynamic plant, motor and sensor simulation | Repeatable scenarios, physics configuration, logs |
| RViz | F450 URDF, frames, markers and sensor views | Visual checks; no implied physics simulation |
| Python / NumPy / Matplotlib | Calibration, uncertainty and plots | Repeatable numerical analyses and retained adverse results |
| Git / GitHub | Version control and review | Descriptive commits, source hashes, no force-push |
| KiCad | User-designed electronics | Module dimensions, clearances and connectivity checks |

**Methods reused:** sensor/radio libraries, ROS middleware and inference frameworks avoid unnecessary reinvention. Record exact versions, licences and configuration. Software availability does not prove its hardware interface or flight behaviour.

**Professional contribution:** formulate requirements; derive dynamics; design calibration experiments; interpret uncertainty and failures; integrate perception, control and testing. Complete the personal reflection with actual actions/results, not invented experience. Attribute libraries and AI assistance under university policy.

**Acceptance dossier:** 50 requirements, 13 verification cases, 37-entry materials inventory, original evidence provenance and source fingerprints. Costs remain TBD until dated quotations are recorded. An attractive video complements, but does not replace, reproducible data.

---
## 5. Handheld | RP2040 pilot interface

[[VIS:handheld-interface]]

| Input | Intended behaviour | Validation |
|---|---|---|
| Right stick | Pitch / roll angle reference | Neutral/dead zone; axis sign; gentle limits |
| Left horizontal | Yaw-rate reference | Correct direction and rate bound |
| Left vertical | Vertical-speed reference | Centre height retention through FC vertical law |
| RT / LT | Bounded increase / decrease | Preserve PID headroom; no direct motor bypass |
| LB / RB and extra buttons | Mode/request functions | Final RP2040 gestures remain to define |
| Display | Mode, battery and link status | Must not block input or command handling |

**Acquisition:** six analogue channels (four stick axes + two triggers) require an acquisition arrangement; the existing 74HC4051 mux needs channel settling and calibration checks. Power and display models remain to select.

**Command path:** centre/scale -> dead zone -> gentle response -> limits -> packet. FC validates freshness and mode before using references. Sticks request motion; the FC retains its own stabilization and motor authority. Mode changes require deliberate validated gestures, not connection alone.

**Independence:** no Pi/Jetson is needed for stable pilot mode. The companion may be absent at boot. Physical motor arming remains disabled during the current bench stage.

---
## 5.1 Links | LoRa, CC1101 and companion contract

[[VIS:communication-contract]]

[[EQ:T_{sym}=\frac{2^{SF}}{BW},\quad T_{age}=t_{now}-t_{sample}<T_{valid}]]

**Symbols:** Tsym: LoRa symbol duration (s); SF: spreading factor; BW: bandwidth (Hz); t_now/t_sample: receiver time/sample time on a consistent clock; T_age: command age; T_valid: allowed age. Symbol duration is not complete packet latency.

**Pilot link:** 1 LoRa module per side. Versioned fields: sequence, normalized controls, buttons, CRC and validity. Measure full airtime, queueing, loss bursts, interference, reconnect and supply integrity. The legacy 22-byte frame needs an explicit versioned extension for new fields. Radio cadence is slower than the local 1 kHz target loop.

**Secondary links:** 2 CC1101 modules per side, reserved for later data. Their exact modules/roles and coexistence remain unvalidated; they are not automatic safety redundancy. Select matched band/antennas and verify applicable RF constraints.

**Companion link:** proposed framed 3.3 V UART with length, CRC, sequence, units, mission identity, timestamp and expiry. Proposed heartbeat 10 Hz/expiry 300 ms require validation. Invalid frames do not refresh validity. ROS QoS uses bounded histories and freshness; reliable delivery does not guarantee a deadline.

**Authority contract:** PC uploads approved nodes; companion navigates; FC accepts bounded references only in the selected mode and owns motor commands. Plugging in the companion does not select autonomous mode. Loss policies and final-arrival actions remain to approve; manual FC must remain usable without it.

---
## References and reproducible evidence

[[REPO]]

**Institution:** USTHB; Faculté de Génie Électrique; Département Automatique; Robotique et Systèmes Intelligents (RSI), fourth-year engineering. Official programme listing: https://fge.usthb.dz/pages/Orientation-2026-2027 . Published RSI curriculum (2025 update), PDF p. 11: Projet Personnel Professionnel, RSI7.8, semester 7, 2 credits, coefficient 1. Source: https://fge.usthb.dz/storage/disk/Robotique-et-systemes-intelligents-MAJ-2025.pdf . The teacher's assessment rubric remains to obtain.

**Primary technical sources:** [R3] TDK MPU6050 register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf . [R4] Bosch BMP280: https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/ . [R5] Nooploop TOFSense: https://support.nooploop.com/tofsense/ . [R7] YOLO detection: https://docs.ultralytics.com/tasks/detect/ .

[R8] MIT OCW feedback control: https://ocw.mit.edu/courses/16-30-feedback-control-systems-fall-2010/ . [R9] ROS frames: https://www.ros.org/reps/rep-0105.html . [R10] RP2350: https://www.raspberrypi.com/products/rp2350/ . [R11] CC1101: https://www.ti.com/product/CC1101 . [R12] Semtech LoRa: https://www.semtech.com/design-support/faq/faq-lora . [R13] ROS 2 concepts: https://docs.ros.org/en/jazzy/Concepts.html . These support methods, not acceptance of this aircraft.

**Project evidence:** [P1] secured-mount six-pose calibration; [P2] gyro restart validation; [P3] FIFO/frame/timing; [P4] reserved recording replay and adverse synthetic models; [P5] input guard checks. Raw evidence is distinguished from derived summaries. [P6] inspected Pico/ROS source commit bce665ad97adbfcb282f02970a1db599e67e6479; source-inventory.json records exact files. No fresh RP2350 flight, hardware-radio acceptance or complete autonomous mission is claimed.

**Reproduce:** report/report.md is editable; tools/build_report.py rebuilds the PDF/HTML; report/ data files and manifests retain requirements, inventory, assumptions and hashes. Original private recordings require a sanitized examiner evidence package. Cover = AI F450 concept, not a hardware photograph. Mathematical schematics and proposed test environments are labelled. Report edition: 27 October 2026; institutional/source checks: 28 October 2026.
