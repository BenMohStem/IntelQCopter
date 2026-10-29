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
| 1. Flight controller | How does the aircraft sense, estimate and stabilize? | 3-13 |
| 2. ROS 2 and YOLO | How does the onboard computer perceive and navigate? | 14-16 |
| 3. Gazebo and RViz | How do the model and actual software get tested? | 17-18 |
| 4. Resources | What hardware, software and development method are needed? | 19-20 |
| 5. Handheld and communication | How do pilot and companion commands reach the FC? | 21-22 |

**Target:** RP2350 aircraft; RP2040 handheld. Stable manual flight works without the companion. Raspberry Pi 4/5 or NVIDIA Jetson is being sought for autonomous operation.

**Evidence:** secured sensor recordings and offline analyses exist; Pico integration and autonomous flight remain unvalidated. Figures distinguish recorded evidence, analytic examples and proposed designs. Physical motor outputs remain disabled.

**Notation:** bold = vector/matrix; dot = time derivative; hat = fitted estimate; bar = sample mean; T superscript = transpose; -1 superscript = inverse; || || = vector length. Symbols are defined locally; the same letter can have a different role on another page.

**Full mathematical edition:** applied appendices A1-A30 develop the proofs, frequency analysis and complete registers. Use the PDF outline to navigate directly.


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


[[EQ:K_p=J\omega_n^2,\quad K_d=2\zeta\omega_nJ\quad\mathrm{(ideal\ angle\ PD\ example)}]]

**Gain symbols:** ζ: damping ratio; ωn: natural frequency (rad/s); J: selected-axis inertia. Kp is N m/rad; Kd is N m s/rad. This ideal angle-to-torque relation differs from the rate-loop example used for Nyquist.

**Scope:** zero initial conditions give Jxx s² Φ = Τx, hence Gφ = Φ/Τx; Φ/Τx are transforms of angle/torque. Identify actuator lag, delays, thrust response and saturation before real gain selection. Plot = educational damping example; supplied mass/inertia remain unmeasured inputs.

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
## 1.3 PID diagrams | separate terms and flight axes

[[VIS:pid-parallel]]

[[VIS:yaw-height-pid]]

**Read left to right:** references define the desired motion; estimated motion returns through negative feedback; PID requests are limited before mixing. Saturation feedback prevents the integral term from accumulating an impossible request.

**Six blocks:** roll/pitch each have angle + rate PID; yaw has rate PID; the vertical path has speed PID. The outer height/climb reference law is still a design choice. All diagrams describe intended software paths; physical output remains disabled.

---
## 1.4 PID stability | Nyquist in one picture

[[VIS:nyquist-example]]

[[EQ:L(s)=C(s)G(s),\quad C(s)=K_p+\frac{K_i}{s}+K_d\frac{\omega_f s}{s+\omega_f}]]

[[EQ:G(s)=\frac{1-T_d s/2}{(Js+d)(\tau_m s+1)(1+T_d s/2)}]]

**Symbols:** L: dimensionless loop gain, distinct from arm length; C: rate PID; G: torque-to-rate plant; s: Laplace variable; J: axis inertia; d: rotational damping; τm: motor lag; Td: delay represented by first-order Padé; ωf: derivative-filter corner. Feedback is negative and unity-scaled.

**Assumed example:** J = 0.006421 kg m²; d = 0.01 N m s/rad; τm = 35 ms; Td = 15 ms; ωf = 2π x 80 rad/s. Base Kp/Ki/Kd = 0.12/0.2/0.0008 in rate-loop torque units. Only J comes from the supplied model; other numbers are educational assumptions. Gain units: Kp in N m s/rad, Ki in N m/rad, Kd in N m s²/rad.

| Gain multiplier μ | Phase margin | Unstable poles Z | Full clockwise winding N |
|---|---|---|---|
| 1 | 52.31° | 0 | 0 |
| 8 | -7.41° | 2 | 2 |

The critical point is -1 + j0. All three gains scale by μ. The winding calculation includes the origin-pole indentation and outer closure; panel A retains every sampled frequency branch and panel B zooms around -1. The origin pole sends the low-frequency branches to infinity; a finite picture cannot contain that limit at linear scale. **Not flight gain selection:** full derivation, units and checks are in the mathematical edition.

---
## 1.5 PID stability | read poles in the complex plane

[[VIS:poles-plane]]

[[EQ:s_p=\sigma+j\omega,\quad e^{s_p t}=e^{\sigma t}[\cos(\omega t)+j\sin(\omega t)]]

| Pole property | Physical interpretation |
|---|---|
| σ < 0 | This mode decays |
| σ > 0 | This mode grows: model instability |
| Larger absolute ω | Faster oscillation |
| μ increases | Scales all three illustrative PID gains |

**Symbols:** sp: closed-loop pole (1/s); σ/ω: real/imaginary parts; j: square root of -1; t: time (s); μ: gain multiplier. Roots solve 1 + L(s) = 0 after clearing denominators. An unstable mode is enough to invalidate the model's closed-loop stability.

**Read the plane:** horizontal position gives growth/decay; vertical position gives oscillation. The main view follows the complex pair as gains change; the second panel shows all five poles for both cases. Two dimensions show these quantities directly. This is an analytic model sweep, not a drone trajectory or sampled-MCU test. Delays, saturation, coupling and uncertain hardware still need validation.

---
## 1.6 Sensors | six-pose calibration explained

[[VIS:calibration-fit]]

[[EQ:b_i=\frac{\bar r_{i,+}+\bar r_{i,-}}{2},\quad c_i=\frac{2g}{\bar r_{i,+}-\bar r_{i,-}}]]

**One-axis intuition:** opposite-pose means r+ / r- give bias bi and dimensionless correction scale ci; corrected reading = ci (raw - bi). This assumes exact aligned ±g poses; the full fit below also models cross-axis/mounting effects.

[[EQ:\mathbf r_j=\mathbf b+A_s\mathbf a_j+\boldsymbol\epsilon_j,\quad\widehat M=[\widehat{\mathbf b}\ \widehat A_s],\quad\mathbf a_{cor}=\widehat A_s^{-1}(\mathbf r-\widehat{\mathbf b})]]

[[EQ:\mathbf h_j=[1,a_{x,j},a_{y,j},a_{z,j}]^T,\quad H=[\mathbf h_1\ \cdots\ \mathbf h_N],\quad Y=[\mathbf r_1\ \cdots\ \mathbf r_N]]]

[[EQ:Y\simeq MH,\quad\widehat M=YH^T(HH^T)^{-1}]]

**Read the fit:** H is the 4 x N table of known gravity poses; Y is the 3 x N table of measured XYZ means. M-hat is fitted: first column bias b, last three columns response As. Minimize squared residuals; the normal equation is M HH-transpose = YH-transpose. Full row rank permits the inverse. Detailed proof is in the full edition.

**Symbols:** j: pose; r_j: raw acceleration; a_j: known gravity reference (m/s²); ε_j: residual; b: bias (m/s²); As: dimensionless response; a_cor: corrected acceleration. Six orientations ±g along X/Y/Z reveal offsets, scales and cross-axis effects. One pose cannot.

**Sensor roles:** MPU6050 measures acceleration/rate; BMP280 supplies pressure-derived altitude; GY-271 supplies a magnetic heading observation after chip identification/calibration; lidar supplies one range ray.

**Evidence [P1]:** 18 x 20 s recordings; repetitions 1-2 fit, third held out. Fixture uncertainty remains. MPU registers are volatile: restore/read back at boot; do not subtract bias twice. Full matrix correction stays in software. Gyro needs separate motionless bias plus known signed-rotation checks; those rotation checks remain pending.

---
## 1.7 Uncertainty | sensor spread and fusion weights

[[VIS:stats-pipeline]]

[[EQ:\bar x=\frac{1}{N}\sum_i x_i,\quad s^2=\frac{1}{N-1}\sum_i(x_i-\bar x)^2,\quad s=\sqrt{s^2}]]

[[VIS:fusion-versus-average]]

[[EQ:\widehat x=(1-w)y_1+wy_2,\quad w^*=\frac{R_1}{R_1+R_2},\quad P^*=\frac{R_1R_2}{R_1+R_2}]]

**Symbols:** xi: samples; N: count; x-bar: mean; s²/s: sample variance/standard deviation. For unbiased independent observations y1/y2 of the same quantity, R1/R2 are error variances, w sensor2weight, P fused variance. Variance carries squared units.

**Example:** σ1 = 1, σ2 = 3. A 50/50 mean gives σ = 1.581; optimal weighting gives0.949. Equal independent noise makes ordinary averaging optimal among unbiased linear combinations; unequal/correlated noise needs its proper covariance model.

**Kalman input:** estimate observation-error variance R after calibration under a justified stationary reference. Motion, bias, fixture error and temporal correlation are separate. Only independent samples give Var(mean) = σ²/N. Gyro rate and acceleration require a physical model before fusion; they cannot be directly averaged.

---
## 1.8 Estimation | attitude and height Kalman filters

[[VIS:estimator-control]]

[[EQ:\theta_k^-=\theta_{k-1}+\omega_k\Delta t,\quad K_k=\frac{P_k^-}{P_k^-+R_k},\quad\theta_k=\theta_k^-+K_k(z_k-\theta_k^-)]]

[[EQ:P_k^-=P_{k-1}+Q_k,\quad P_k=(1-K_k)P_k^-]]

**Attitude:** integrate gyro rate to predict angle; correct using an accelerometer-derived gravity angle. P-minus = prior angle-error variance; R = observation variance; K = dimensionless trust. θ/z are estimated/observed angles (rad); ω is gyro rate (rad/s); k is sample; Δt is interval (s). Superscript minus means prediction. Q is angle-process uncertainty per step. Posterior P is variance; its square root is standard deviation.

[[EQ:h_k^-=h_{k-1}+v_{k-1}\Delta t+\tfrac12 a_k\Delta t^2,\quad v_k^-=v_{k-1}+a_k\Delta t]]

[[EQ:K_h=\frac{P_{hh}^-}{P_{hh}^-+R_h},\quad K_v=\frac{P_{vh}^-}{P_{hh}^-+R_h},\quad h_k=h_k^-+K_h\nu_k,\quad v_k=v_k^-+K_v\nu_k]]

**Height:** ν = measured height - predicted height. h/v/a = world height (m), vertical speed (m/s), gravity-compensated acceleration (m/s²). Phh = height-error variance (m²); Pvh = speed/height error covariance (m²/s); Rh = height-observation variance (m²); Kh dimensionless; Kv in 1/s. Propagate both uncertainties and their covariance; this joint height-speed estimate feeds the vertical PID.

**Sensor limits:** reject invalid/stale observations; barometer drift, lidar tilt/terrain and dropouts need tests. Bottom-versus-front lidar placement remains open. Historical scalar K values [P4], roll 0.00061129/pitch 0.00048326, are conditional stationary-model values, not accepted flight tuning.

---
## 1.9 Evidence | real sensor replay and its limits

[[IMAGE:reserved]]

| Reserved replay [P4] | Roll | Pitch |
|---|---|---|
| Raw angle standard deviation | 0.177569 degrees | 0.213074 degrees |
| Filter output standard deviation | 0.007000 degrees | 0.005692 degrees |
| Filter minus observation mean | -0.033769 degrees | -0.039908 degrees |

**Engine:** original Python float64 replay of reserved real 1 kHz data; unchanged plot. Not MCU execution or flight. Fixed 2 s initialization excluded. No independent attitude reference: quieter output does not prove accuracy.

**Adverse test [P4]:** synthetic 0.1g lateral acceleration produced 5.191 degrees peak error; acceleration plus gyro bias produced 5.355 degrees. Apparent gravity tilt is arctan(0.1) = 5.71 degrees while magnitude is only 1.005g. Norm gating alone is insufficient. Dynamic acceptance remains blocked.

---
## 1.10 Energy | battery estimation and avionics safety

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


**Two editions:** this visual report gives the essentials; IntelQCopter-Full.pdf develops the derivations, Bode and Black–Nichols analysis, detailed verification and complete registers. The previous full edition is preserved separately under reference/.

---
## A1. Physics | rotating thrust and projecting lift

[[VIS:frames-geometry]]

[[EQ:\mathbf f_w=R\mathbf f_b,\quad R=R_z(\psi)R_y(\theta)R_x(\phi),\quad R^TR=I,\quad\det R=1]]

R changes coordinate basis, not the physical force. Its columns are the body basis vectors expressed in world coordinates. Orthogonality preserves length; determinant +1 excludes reflection. I is identity; ψ/θ/φ are yaw/pitch/roll parameters (rad). Body thrust is fb = T e3; world thrust is therefore R e3 T.

At level hover, the upward axes coincide and T = mg. Tilting by α reduces vertical support to T cos α. If acceleration and drag are zero, T = mg/cos α; horizontal thrust T sin α produces horizontal acceleration. Returning angle to zero does not itself remove horizontal velocity.

[[EQ:a=L\cos45^\circ=L/\sqrt{2},\quad L^2=a^2+a^2]]

Nominal 450 mm opposite-motor spacing gives L = 225 mm centre-to-motor distance and a = 159.1 mm projection. These are geometric assumptions, not manufacturing measurements. Each rotor position (±a,±a) determines its roll/pitch moment arm. D in the force law is a separate linear drag approximation; for wind, air-relative velocity or an appropriate disturbance model is required.

---
## A2. Identification | mass inertia and parameter uncertainty

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

**Symbols:** i: component; mi: mass; yi/zi: position relative to assembly COM; Jxx,i at COM: component inertia about its own centre; Σp/Σy: parameter/output covariance; G: local sensitivity/Jacobian of outputs with respect to parameters. The uncertainty approximation is first order.

---
## A3. PID gains | derive PD and a cubic stability condition

[[EQ:J\ddot\phi=\tau,\quad\tau=K_p(\phi_r-\phi)-K_d\dot\phi]]

For constant reference and an ideal torque actuator, the homogeneous equation is J φ-double-dot + Kd φ-dot + Kp φ = 0. Divide by J and match a desired second-order denominator:

[[EQ:s^2+\frac{K_d}{J}s+\frac{K_p}{J}=s^2+2\zeta\omega_n s+\omega_n^2]]

[[EQ:K_p=J\omega_n^2,\quad K_d=2\zeta\omega_nJ]]

J is selected-axis inertia (kg m²), ωn natural frequency (rad/s), ζ damping ratio. For this angle-to-torque example, Kp is N m/rad and Kd is N m s/rad. These identities explain gain scaling; they do not identify the real actuator or account for inner-loop bandwidth.

Add positive Ki. Differentiating the homogeneous integral equation gives characteristic polynomial J s³ + Kd s² + Kp s + Ki. For Ki > 0, its Routh first column is J, Kd, (Kd Kp - J Ki)/Kd, Ki. Strict left-half-plane stability requires:

[[EQ:J>0,\quad K_d>0,\quad K_p>0,\quad K_i>0,\quad K_dK_p>JK_i]]

At Ki = 0 reduce to the second-order PD polynomial, rather than retaining an artificial integrator root. Motor lag, delay, filtering and saturation change this analysis. Final six-loop gains require identification and closed-loop validation; the inequality is not a complete aircraft stability proof.

---
## A4. Digital PID | filter derivative and limit integration

[[VIS:pid-parallel]]

[[EQ:D_k=\alpha_kD_{k-1}+(1-\alpha_k)K_d\frac{e_k-e_{k-1}}{\Delta t_k},\quad\alpha_k=\frac{\tau_D}{\tau_D+\Delta t_k}]]

Backward-Euler discretization of τD D-dot + D = Kd e-dot gives this recurrence. D is derivative contribution; τD is filter time constant (s); α is dimensionless; Δtk is measured interval. Positive τD and Δtk give 0 < α < 1. At constant error, D decays.

[[EQ:I_{k+1}=I_k+\Delta t_k[K_i e_k+K_{aw}(u_k-u_{raw,k})]]]

I is integral contribution, u_raw unsaturated request, u applied bounded request, and Kaw anti-windup rate (1/s). Applied minus requested output opposes accumulation when clipping occurs. Derivative on measurement can avoid setpoint kick; document that reference-path difference. The frequency example uses its explicitly stated parallel feedback form.

Use coherent timestamps; bound waits/output/integration; reject invalid intervals; handle stale observations and mode changes. Target 1 kHz means nominal 1 ms, not automatic complete-FC deadline acceptance. Slow sensors update asynchronously and physical outputs remain disabled.

---
## A4.1 Anti-windup | why return the saturation error?

[[VIS:anti-windup-example]]

[[EQ:u_{raw}=v+I,\quad v=K_pe+D,\quad u=\operatorname{clip}(u_{raw},u_{min},u_{max})]]

[[EQ:\dot I=K_ie+K_{aw}(u-u_{raw})]]

**Derive the sign:** upper saturation means u < u_raw. Their difference is negative, so it opposes further integral accumulation. Lower saturation reverses this sign. Without saturation u = u_raw, so ordinary integration is recovered exactly.

Hold e and v constant during upper saturation. Substitute u = u_max and collect I terms:

[[EQ:\dot I+K_{aw}I=K_ie+K_{aw}(u_{max}-v),\quad I_* =u_{max}-v+\frac{K_ie}{K_{aw}}]]

[[EQ:I(t)=I_*+[I(0)-I_*]e^{-K_{aw}t},\quad T_{aw}=1/K_{aw}]]

This solution holds only while that saturation regime persists. Kaw has units 1/s; T_aw is the tracking time constant. The integrator approaches a finite value rather than growing indefinitely. It need not become zero when the error becomes zero: a nonzero integral can supply steady load compensation.

For explicit Euler, the tracking error multiplier is 1 - Δt Kaw. Its isolated constant-regime recurrence decays if 0 < Δt Kaw < 2; monotonic decay needs at most 1. This condition alone does not prove closed-loop or switching stability.

**Quadcopter integration gate:** actual motor clipping happens after mixing. Recover achievable collective/torques through a validated allocator before feeding an equivalent output error back to each integrator; subtracting raw motor PWM directly from axis torque is dimensionally wrong. This experiment changes no firmware or motor policy.

---
## A5. Frequency model | parameters and rate-loop units

[[EQ:C(s)=K_p+\frac{K_i}{s}+K_d\frac{\omega_f s}{s+\omega_f},\quad L(s)=C(s)G(s)]]

[[EQ:G(s)=\frac{1-T_d s/2}{(Js+d)(\tau_m s+1)(1+T_d s/2)}]]

| Parameter | Assumption | Unit / provenance |
|---|---|---|
| J | 0.006421 | kg m²; supplied model |
| d | 0.01 | N m s/rad; assumed rotational damping |
| τm; Td | 0.035; 0.015 | seconds; assumed lag/delay |
| ωf | 2π x 80 | rad/s; assumed derivative corner |
| Kp; Ki; Kd | 0.12; 0.2; 0.0008 | Assumed rate-loop gains |

The damped rate equation is J rate-dot + d rate = torque. Kp has units N m s/rad, Ki N m/rad and Kd N m s²/rad. Multiply all gains by μ to form Cμ. L is dimensionless; it is not the geometric arm length on this page.

The delay fraction is first-order Padé, an approximation of exp(-Td s), not an exact delay. Its right-half-plane zero is an approximation feature, not a measured aircraft zero. For L = NL/DL, unity negative feedback gives poles from DL + NL = 0, a fifth-order polynomial here.

One shared NumPy calculation supplies coefficients, roots, margins and graphics in analysis/control-design-example.json. It does not simulate a flight or certify the sampled MCU implementation. Damping, lag, delay and gains are educational assumptions.

---
## A6. Nyquist | prove the winding convention

[[VIS:nyquist-contour]]

[[EQ:F(s)=1+L(s)=\frac{D_L(s)+N_L(s)}{D_L(s)},\quad N_{cw}=-\frac{\Delta\arg F}{2\pi}=Z-P]]

P counts open-loop poles in the enclosed right half-plane; Z counts closed-loop poles there; Ncw counts clockwise encirclements of -1 by L. Applying the argument principle clockwise supplies the minus sign. The same contour must be used for every count.

The PID integrator gives a pole at s = 0. Indent the contour to its right, excluding that boundary pole, then close the outer right-half-plane arc. A finite positive-frequency trace alone does not include these pieces. Here P = 0; μ = 1 gives Ncw = Z = 0, while μ = 8 gives Ncw = Z = 2.

**Checks:** compute at indentation radius 10^-4 and outer radius 10^6, repeat at 10^-3/10^5, and compare the integer winding with all polynomial roots. Refine crossover brackets and check normalized characteristic-polynomial residuals. This is a finite numerical consistency check for the specified rational model, not a physical stability proof.

If a pole or zero lies on the contour, a new limit/contour analysis is required. Reference [R14] defines the frequency-domain margins; a plot does not replace assumptions and root counts.

---
## A6.1 Nyquist | where do the large turns come from?

The loop contains Ki/s. Near s = 0 the derivative term vanishes, the actuator and Padé factors tend to one, and the damped rate plant tends to 1/d. Therefore:

[[EQ:C(s)=K_i/s+O(1),\quad G(s)=1/d+O(s),\quad L_\mu(s)=\frac{\mu K_i}{d s}+O(1)]]

[[EQ:L_\mu(j\omega)\sim-j\frac{\mu K_i}{d\omega},\qquad |L_\mu(j\omega)|\longrightarrow\infty\quad(\omega\to0^+)]]

**Meaning:** the apparent missing ends in a linear close-up are not missing data or a finite oval. They extend without bound. The conjugate branch goes upward. Both are included in the symmetric-log overview; the linear close-up retains undistorted distance to -1.

On the right indentation s = ε exp(jθ), θ increases from -π/2 to +π/2. Substitution gives:

[[EQ:L_\mu(\epsilon e^{j\theta})\sim\frac{\mu K_i}{d\epsilon}e^{-j\theta},\quad |L|\approx\frac{\mu K_i}{d\epsilon}]]

For ε = 10^-4, Ki = 0.2 and d = 0.01, the large mapped arc has approximate radius 200000 μ: 200000 for baseline and 1600000 for μ = 8. This joins the two low-frequency ends through the right side in the leading-order approximation. The exact rational values, not this asymptote, are plotted in A6.

| Contour piece, in traversal order | Image in the loop plane |
|---|---|
| Negative branch: -jR to -jε | Conjugate frequency branch |
| Right origin indentation | Large connecting arc; do not omit |
| Positive branch: +jε to +jR | Positive-frequency branch |
| Clockwise outer RHP arc | Tiny closure near L = 0 as R grows |

The symmetric-log transform is continuous, one-to-one and increasing on each axis. It preserves whether a closed curve encloses the transformed critical point, but changes lengths, angles and curvature. Compute Δarg(1+L) from the original complex values, never from displayed distances. The actual characteristic-root count independently checks the winding result.

---
## A7. Bode | gain phase and crossover margins

[[VIS:bode-example]]

[[EQ:M(\omega)=20\log_{10}|L(j\omega)|,\quad\varphi(\omega)=\arg L(j\omega)]]

[[EQ:PM=180^\circ+\varphi(\omega_{gc}),\quad GM_{dB}=-20\log_{10}|L(j\omega_{pc})|]]

ωgc is unity-gain crossover; ωpc is negative-real-axis phase crossover (-180 degrees modulo 360). PM is additional phase lag to the critical point at unity gain. GM is gain increase to unity at phase crossover. Report all relevant crossings rather than hiding a limiting one [R14].

**Model results:** μ = 1: PM = 52.3065 degrees, GM = 15.9195 dB. μ = 8: PM = -7.4061 degrees, GM = -2.1423 dB, consistent with two unstable roots. Frequency is rad/s, not Hz. Positive μ shifts magnitude by 20 log10 μ but leaves phase unchanged; the crossover moves. Magnitude is dimensionless before conversion to dB.

Padé models rationalize delay; their high-frequency phase is not the exact delay's unbounded phase. These margins belong to one assumed continuous rate loop. Sampling, uncertainty, estimator dynamics, saturation and interaction among six loops still require analysis. None is a measured aircraft robustness margin.

---
## A8. Black–Nichols | one gain-phase chart

[[VIS:nichols-example]]

[[EQ:(x,y)=(\arg L(j\omega),20\log_{10}|L(j\omega)|),\quad T_{cl}(s)=\frac{L(s)}{1+L(s)}]]

Frequency parameterizes the curve; axes are phase (degrees) and magnitude (dB). The critical point is (-180 degrees, 0 dB), repeated every 360 degrees. This is another representation of the same loop, not another controller.

Light grid contours show constant closed-loop magnitude |Tcl| in dB under unity negative feedback. Their shape follows |L/(1+L)|, which grows large near L = -1. Crossing an arbitrary contour is not itself a stability criterion [R15].

At 0 dB, phase distance from -180 degrees gives phase margin for that crossing; at -180 degrees, magnitude distance from 0 dB gives gain margin. Use the continuous phase branch and all relevant crossings, retaining the independent root/winding check. Black–Nichols and Nichols gain-phase chart name this same representation.

---
## A9. 3D sensitivity | gains cannot ignore actuator lag

[[VIS:stability-surface]]

[[EQ:\alpha(\mu,\tau_m)=\max_i\Re\{p_i(\mu,\tau_m)\}]]

pi are all five closed-loop poles; α is the largest real part (1/s). Strict α < 0 means every mode decays in this rational model; α > 0 means at least one grows. A real slow pole can dominate even when a selected oscillatory pair looks well damped.

Sweep μ = 0.5 to 8 and assumed motor lag τm = 15 to 80 ms, holding other assumptions fixed. Each grid point recomputes every root. The zero contour is this model's boundary, not an experimentally verified flight envelope or confidence surface.

Identify propulsion dynamics across representative voltage/loading before choosing real gains. Then assess sampling, saturation, estimator dynamics, all loops and parameter uncertainty. A dense coloured surface cannot replace those measurements.

---
## A10. Calibration | experiment and identifiability

[[VIS:calibration-protocol]]

[[EQ:\mathbf r_j=\mathbf b+A\mathbf a_j+\boldsymbol\epsilon_j,\qquad\mathbf a_j\in\{\pm g\mathbf e_x,\pm g\mathbf e_y,\pm g\mathbf e_z\}]]

[[EQ:H=[\,\mathbf h_1\ \cdots\ \mathbf h_{12}\,],\quad\mathbf h_j=[1,a_{x,j},a_{y,j},a_{z,j}]^T,\quad Y=[\,\mathbf r_1\ \cdots\ \mathbf r_{12}\,]]]

[[EQ:\widehat M=YH^T(HH^T)^{-1}=[\,\widehat{\mathbf b}\ \widehat A\,],\qquad C=\widehat A^{-1}]]

**Identifiability:** H must have full row rank. Six independent gravity directions reveal offset and directional response; one level pose cannot identify the full matrix. Equal fit weights do not imply exact pose references.

**Evidence [P1]:** 18 recordings, 20 seconds each; repetitions 1-2 fit the candidate and repetition 3 is reserved. A detached mounting invalidates its associated dataset. The secured-mount data and later tilted-pose review remain distinct. Standard gravity 9.80665 m/s squared is an assumption, not a local gravimeter reading.

---
## A11. Calibration | complete fit and reserved residuals

[[VIS:calibration-matrix]]

[[EQ:\mathbf a_{cor}=C(\mathbf r-\mathbf b)]]

[[VIS:calibration-residuals]]

**Read the heatmap by row:** each corrected body axis may depend on all three raw axes. C includes scale/cross-axis/mounting effects in this historical fit. The bias and matrix are not interchangeable with three MPU offset registers.

**Do not recalibrate from this page.** These are historical offline-fit coefficients [P1], not a newly installed calibration. Residuals contain fixture/placement uncertainty; they are not certified sensor accuracy. Frozen firmware constants require their own recorded provenance and reset/power-cycle verification.

---
## A12. Calibration | derive the normal equations

[[EQ:Y=MH+E,\quad M=[\mathbf b\ A_s],\quad H\in\mathbb R^{4\times N},\quad Y\in\mathbb R^{3\times N}]]

Each H column is [1, ax, ay, az] transpose: the leading 1 models constant bias. Each Y column is the corresponding XYZ measurement mean. E is residual error; M is an unknown 3 x 4 table. Its first column is bias b, the next three are sensor-response As. N is fit-pose count.

[[EQ:J(M)=\operatorname{tr}[(Y-MH)(Y-MH)^T],\quad\frac{\partial J}{\partial M}=-2(Y-MH)H^T]]

J sums every squared residual; tr is the sum of diagonal entries. Perturbing M gives dE = -dM H. The two equal scalar trace terms in dJ combine into the stated gradient. Set it to zero:

[[EQ:MHH^T=YH^T\quad\Longrightarrow\quad\widehat M=YH^T(HH^T)^{-1}]]

Full row rank of H makes HH-transpose positive definite: for any nonzero u, u-transpose HH-transpose u = ||H-transpose u||² > 0. The quadratic is strictly convex row by row, so the solution is unique. Poor pose geometry can still be ill-conditioned. Gaussianity is not required for the minimizer.

**Reference uncertainty:** this fit treats H as fixed. Fixture/hand-held angle errors affect H; more samples at the same uncertain pose do not remove them. Weighted or error-in-variables fitting needs justified reference covariance, not guessed precision. A hat means fitted estimate, not exact truth.

---
## A13. Statistics | prove the N - 1 variance correction

[[EQ:\bar X=\frac{1}{N}\sum_iX_i,\quad s^2=\frac{1}{N-1}\sum_i(X_i-\bar X)^2]]

Assume independent, identically distributed readings Xi with mean μ and finite variance σ². Expand Xi - Xbar = (Xi - μ) - (Xbar - μ), sum, and use the zero sum of centred sample deviations:

[[EQ:\sum_i(X_i-\bar X)^2=\sum_i(X_i-\mu)^2-N(\bar X-\mu)^2]]

[[EQ:E\!\left[\sum_i(X_i-\bar X)^2\right]=N\sigma^2-N\frac{\sigma^2}{N}=(N-1)\sigma^2]]

The first expectation has N equal variance terms; the second uses Var(Xbar) = σ²/N. Therefore E[s²] = σ². N is sample count and E is expectation over repeated experiments. Gaussianity is not needed; finite variance and the stated sample assumptions are.

The square root s is useful but not exactly unbiased for σ. Concavity gives E[s] <= sqrt(E[s²]) = σ. More samples reduce this finite-sample effect, not drift or fixture error. Correlated samples change Var(Xbar) and require a different uncertainty calculation.

A sensor's repeatability standard deviation is not its absolute accuracy. A reference, thermal model, calibration uncertainty and dynamic test are needed before interpreting a quiet signal as correct attitude.

---
## A14. Statistics | covariance and information geometry

[[VIS:covariance-geometry]]

[[EQ:\bar{\mathbf x}=\frac1N\sum_i\mathbf x_i,\qquad S=\frac1{N-1}\sum_i(\mathbf x_i-\bar{\mathbf x})(\mathbf x_i-\bar{\mathbf x})^T]]

[[EQ:\operatorname{Cov}(\bar{\mathbf x})=\frac1{N^2}\sum_i\sum_j\operatorname{Cov}(\mathbf x_i,\mathbf x_j)]]

[[VIS:uncertainty-sampling]]

For independent samples, the last expression reduces to Sigma/N. With correlation, cross-time terms remain. More samples do not automatically remove drift, reference error or modelling bias. The density curves show widths only; neither joint Gaussianity nor independence is asserted. Report repeatability and accuracy separately.

**Symbols:** x_i: sample vector; x-bar: sample mean; N: sample count; S: sample covariance; Σ: population covariance; Cov: covariance operator. The double sum retains all cross-time covariance terms; independence is the extra assumption needed for Σ/N.

---
## A15. Statistics | zero covariance does not prove independence

Let X be standard normal and Y = X² - 1. Symmetry gives E[X] = 0 and E[X³] = 0; E[Y] = E[X²] - 1 = 0. Thus:

[[EQ:\operatorname{Cov}(X,Y)=E[XY]-E[X]E[Y]=E[X^3-X]=0]]

Yet X determines Y exactly, so they are dependent. Y >= -1 also excludes a non-degenerate Gaussian distribution for Y. The pair is not jointly Gaussian even though X is Gaussian.

[[EQ:\varphi_{\mathbf X}(\mathbf t)=\exp(j\mathbf t^T\boldsymbol\mu-\tfrac12\mathbf t^T\Sigma\mathbf t)]]

For a jointly Gaussian vector, block-diagonal covariance makes this characteristic function factor into a product for the blocks. That factorization gives independence. The implication relies on joint Gaussianity; Gaussian-looking marginal histograms do not prove it.

Here φ is the characteristic function, t its real argument, μ the mean vector, Σ population covariance and j the imaginary unit. A vector is jointly Gaussian when every real linear projection is Gaussian. Calibration is affine and preserves this property if it already holds; it does not make arbitrary sensor noise jointly Gaussian.

For fixed C, every projection satisfies u-transpose C ε = (C-transpose u)-transpose ε; if the original vector is jointly Gaussian, the right side is Gaussian by definition. This proves preservation under the linear calibration. E[C ε] = C E[ε] = 0 only when the original error is unbiased; a fitted correction with uncertain reference does not guarantee exact zero error.

Shared vibration, filtering, timing and temperature can create dependence. Finite covariance estimates and stationary histograms cannot prove an exact distributional model. Kalman filtering may still be used as a model-based linear estimator while these limitations are stated and tested.

---
## A16. Gyro | bias frame and scale are separate

[[VIS:boot-gyro]]

[[EQ:\boldsymbol\omega_b=R_{bs}(\boldsymbol\omega_s-\mathbf b_g),\qquad\Delta\theta_{bias}(t)=b_g t]]

[[VIS:gyro-bias-noise]]

**Bias is not scale or frame mapping.** Motionless data estimate bias/repeatability. Known positive and negative rotations about physical body X, Y and Z test sign and mapping; a measured rotation reference is additionally needed for scale.

MPU user offsets are volatile. Restore against the documented reset baseline, read back, then apply only the remaining software bias. Do not subtract the full bias twice or use the accelerometer scale matrix as a gyro rotation. Physical gyro direction/scale checks remain pending.

**Symbols:** ωs/ωb: sensor/body angular-rate vectors (rad/s); bg: sensor-rate bias; Rbs: orthogonal sensor-to-body rotation; Δθbias: accumulated angle error; t: duration (s). A full accelerometer scale/cross-axis fit is not automatically a valid gyro rotation.

---
## A17. Kalman | prediction correction and model units

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
## A18. Kalman | prove the scalar gain and correlation correction

[[EQ:\widehat x=(1-K)\widehat x^-+Kz,\quad P(K)=(1-K)^2P^-+K^2R]]

Assume unbiased, uncorrelated prediction and observation errors with variances P-minus and R. Both refer to the same variable. The weights sum to one; K is dimensionless. Minimize the resulting error variance:

[[EQ:\frac{dP}{dK}=-2(1-K)P^-+2KR=0\quad\Longrightarrow\quad K=\frac{P^-}{P^-+R}]]

The second derivative is 2(P-minus + R) > 0, proving a unique minimum when total variance is positive. Substitution gives posterior variance P = P-minus R/(P-minus + R). This minimum-variance linear result does not require Gaussianity.

[[EQ:K=\frac{P^- - C_e}{P^-+R-2C_e},\quad C_e=\operatorname{Cov}(e_p,e_m)]]

For correlated prediction/measurement errors ep/em, add 2K(1-K)Ce to P(K) before differentiating. The formula follows, with the ordinary gain recovered at Ce = 0. Ignoring correlation can make covariance overconfident. If P-minus + R - 2Ce is zero, the error difference has zero variance and this formula cannot divide by it.

With linear models and jointly Gaussian errors, the conditional mean is linear and this is also the exact model-based mean-square estimator. Dynamic acceleration, bias and nonlinear angle conversion restrict that interpretation. Historical scalar gains are conditional stationary-model values, not flight tuning.

---
## A19. Vertical Kalman | coupled height and speed uncertainty

[[EQ:P_{hh}^-=P_{hh}+2\Delta tP_{hv}+\Delta t^2P_{vv}+q_{hh}]]

[[EQ:P_{hv}^-=P_{hv}+\Delta tP_{vv}+q_{hv},\quad P_{vv}^-=P_{vv}+q_{vv}]]

h is height (m), v vertical speed (m/s); Phh/Pvv are their variances and Phv = Pvh is cross-covariance. Take covariance of h-next = h + dt v and v-next = v, then add process uncertainty q. Uncertain velocity becomes uncertain height; the cross term cannot be discarded arbitrarily.

For independent acceleration error held constant within one interval, with variance σa²:

[[EQ:q_{hh}=\tfrac14\Delta t^4\sigma_a^2,\quad q_{hv}=\tfrac12\Delta t^3\sigma_a^2,\quad q_{vv}=\Delta t^2\sigma_a^2]]

Continuous white acceleration spectral density qc instead gives qhh = qc dt³/3, qhv = qc dt²/2, qvv = qc dt. These are different noise models and units: σa² is in m²/s⁴; qc in m²/s³. Do not interchange them just because both use the letter Q.

For height observation variance Rh, Kh = Phh-minus/(Phh-minus + Rh), Kv = Pvh-minus/(Phh-minus + Rh). The innovation corrects both estimates. Covariance units are m², m²/s and m²/s² respectively. A two-variable estimator does not automatically estimate bias, drift, terrain or attitude error.

This uncertainty propagation supplies PID feedback. Invalid ranges, data age, sensor loss and covariance consistency remain experimental gates; no state-feedback control law is introduced.

---
## A20. Adverse test | acceleration is not gravity

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

**Symbols:** a_lat: lateral acceleration; g: gravitational acceleration; θ_apparent: false tilt inferred as gravity; RMSE: square root of mean squared attitude error relative to the synthetic reference. Peak error is the largest absolute error over that case.

---
## A21. Timing | retain failed and successful observations

[[VIS:timing-budget]]

[[EQ:T_s=1000\ \mu\mathrm s,\qquad T_{critical}+T_{reserve}\le T_s]]

**Measured history [P3]:** trial 2 was malformed and rejected; earlier trials exceeded the handler budget. Repair separated acquisition/calculation from synchronous serial formatting using a bounded queue. The later 822 microsecond maximum leaves only 178 microseconds to 1 ms if compared directly; a complete future FC includes additional work.

All measured timing on this page belongs to the **ESP32-S3 preliminary platform**, not RP2350. The main Pico scaffold's 500 Hz setting and unconnected drivers remain a separate implementation baseline [P6].

**Proposed target:** <=800 microseconds critical work, >=200 microseconds reserve, and no observed deadline miss in a defined 10-minute loaded test. Historical samples near 1000.69 Hz do not prove a periodic hard deadline. Inter-service gaps, clocks and source changes remain relevant.

**P5 build only:** 54 Python checks, 19 actual-source host cases, ESP32 compile; 19504 bytes RAM and 312901 bytes flash. Source f26d314e was not uploaded in that record. Historical timing source 91947b5c is different. A host fake peripheral is not an ESP32 timing measurement.

**Bench source configuration:** accelerometer +/-8g, gyro +/-500 degrees/s, nominal 260 Hz low-pass setting with readback checks. Sensor bandwidth is not the PID bandwidth. These settings do not validate the revised sensor wiring or a full control loop.

**Symbols:** T: target period; Tcritical: full critical-path workload; Treserve: remaining timing margin. The proposed allocation is a budget, not a measured complete-controller bound. A handler duration and an inter-service gap are distinct observations.

---
## A22. Perception | distance observability and stopping

[[VIS:perception-limits]]

[[VIS:stopping-frames]]

[[EQ:\mathbf p_w=R_{wb}\left(R_{b\ell}[d,0,0]^T+\mathbf t_{b\ell}\right)+\mathbf t_{wb}]]

[[EQ:d_{clear}>v\tau+\frac{v^2}{2a_{brake}}+d_{margin}]]

**One RGB camera plus one fixed ray cannot observe the entire collision corridor.** Forward lidar helps one direction; downward lidar helps local height under terrain/tilt assumptions. Metric monocular reconstruction needs additional information and validation; a pixel bearing alone has ambiguous depth.

**Frames:** rotate/translate lidar -> body -> world with calibrated extrinsics and synchronized pose. The beam follows local positive x.

**Example only:** 0.25 s latency and 1 m/s² braking are illustrative, not measured. Validate missed detections, localization and stopping; declare any simulator perfect pose or depth camera.

**Symbols:** p_w: world point (m); R_ab maps frame b directions into frame a; t_ab locates frame b origin in frame a (m); d: lidar range; w/b/ℓ: world/body/lidar frames; v: speed; τ: latency; a_brake: deceleration; d_clear/d_margin: clearance and added margin.

---
## A23. Navigation | actual algorithm and remaining gaps

[[VIS:navigation-implementation]]

[[EQ:f(n)=g(n)+h(n),\qquad h(n)=\|\mathbf p_n-\mathbf p_{goal}\|]]

[[EQ:r_{inflate}\ge r_{vehicle}+d_{stop}+k_\sigma\sigma_{position}+d_{margin}]]

**Existing algorithm [P6]:** an A* grid planner and waypoint follower exist in the portable C++ core. The inspected prototype uses 0.2 m cells, fixed 0.65 m obstacle inflation and a bounded local XY map. Those constants are prototype settings, not validated aircraft clearance or general 3D navigation. The target must account for swept propeller volume, stopping distance and localization uncertainty.

**Existing range demonstration:** known-size blue panels can supply monocular PnP range when their 1.09 x 1.69 m inner dimensions and trained detector are explicitly confirmed. This does not solve arbitrary obstacle depth. Simulated odometry/contact and any optional depth-image branch must be declared; a real localization source and unknown-space policy remain unresolved.

**Implementation sequence:** synchronize sensors and transforms -> validate metric obstacle observations -> update the local map -> replan -> track the route with bounded angle/yaw-rate/vertical references -> send through the RP2350 bridge. Iterate intermediate nodes; do not teleport or inject forces. Loss of valid pose/perception must invalidate autonomous commands under an approved response policy.

---
## A24. Verification | complete acceptance matrix

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
## A25. Requirements | register one

[[REQ:one]]

D = user decision; P = proposed target; TBD = unresolved. A written requirement is not a passed test.

---
## A26. Requirements | register two

[[REQ:two]]

Changed hardware/configuration can invalidate affected acceptance. Preserve original evidence, including failures.

---
## A27. Materials | complete aircraft inventory

[[BOM:aircraft]]

Quantities describe the intended assembly, not confirmed stock. Exact models, interfaces, masses and ratings require verification.

---
## A28. Materials | handheld optional equipment and costs

[[BOM:other]]

Use the shared CSV for dated quotations, ownership, currency and measured mass. Do not invent prices or transfer old prototype assumptions onto different target hardware.

---
## A29. Evidence | reproduce the analysis and preserve the full edition

A source hash identifies bytes, not a pass. Each result needs engine/hardware, configuration, duration, original data, analysis method, fixed criteria, verdict and limits. The unchanged original replay remains distinct from analytic figures.

**Preserved full reference:** reference/full-edition-4.1.pdf and its source are byte-identical to commit600a20a. Its historical metadata and assumptions remain historical; this updated full edition and the short report provide the current academic structure. The archive is available on the same report branch rather than discarded.

**New analysis:** tools/control_design_example.py writes the assumed rational PID model, coefficients, all roots, margins and contour checks to analysis/control-design-example.json. The root/winding counts agree; alternative contour radii/grid refinement and normalized polynomial residuals are checked. All frequency diagrams use this common calculation.

**Rebuild:** python tools/build_report.py . for the short report; add --full for this edition. Shared evidence and material records avoid independent conflicting copies. Domain knowledge records distinguish facts, assumptions and open gates; they are not a chat transcript or hidden reasoning store.

[R14] Frequency margins: https://www.mathworks.com/help/control/ref/dynamicsystem.margin.html . [R15] Nichols design geometry: https://www.mathworks.com/help/control/ug/design-requirements.html . These primary references support the method; figures here are computed with NumPy/Matplotlib, not claimed MATLAB validation.

No real flight gains, new Gazebo mission or hardware acceptance are claimed. A marking rubric and named supervisor remain to obtain. Complete authentic personal contributions and attribute libraries/AI under university policy. Rigour means linking requirements, model, experiment, evidence and decision—not promising a mark.

---
## A30. Fusion | inverse covariance weights and their proof

[[VIS:fusion-versus-average]]

For two unbiased independent measurements of the same scalar x, let y1 = x + e1 and y2 = x + e2, with variances R1 and R2. Choose an unbiased linear estimate xhat = (1-w)y1 + w y2; its variance is minimized by inverse-variance weighting.

[[EQ:P(w)=(1-w)^2R_1+w^2R_2,\quad\frac{dP}{dw}=0\ \Longrightarrow\ w^*=\frac{R_1}{R_1+R_2}]]

[[EQ:P^*=\frac{R_1R_2}{R_1+R_2}\le\min(R_1,R_2),\qquad P_{average}=\frac{R_1+R_2}{4}]]

[[EQ:\mathbf w^*=\frac{R^{-1}\mathbf{1}}{\mathbf{1}^TR^{-1}\mathbf{1}},\qquad P^*=(\mathbf{1}^TR^{-1}\mathbf{1})^{-1}\quad\mathrm{(full\ covariance)}]]

**Example:** sigma1 = 1 and sigma2 = 3 in the same units. A 50/50 average gives sigma = 1.581, worse than sensor 1 alone. Optimal weighting gives sigma = 0.949. The noisier sensor contributes only 10 percent. No Gaussian assumption is required for this minimum-variance linear weighting under the stated covariance model.

**Kalman connection:** a prediction is fused with an observation using uncertainty and dynamics. Gyro rate and acceleration cannot be averaged directly: their units and models differ. Integrate the gyro and derive a valid gravity-angle observation first. With equal independent static-sensor variance, the ordinary mean is already optimal among unbiased linear combinations. Known correlation requires the full positive-definite covariance R; bias or model mismatch can invalidate the predicted physical improvement.

**Proof:** w are weights, 1 is the all-ones vector, and λ is a Lagrange multiplier. Minimize w-transpose R w subject to 1-transpose w = 1. Positive-definite covariance R gives the condition 2 R w - λ 1 = 0 gives w = (λ/2) R-inverse 1. Enforcing the constraint supplies the denominator in the displayed formula. The objective is strictly convex, so this is the unique minimum. The scalar result follows for diagonal R. Known bias/reference error is not removed by this covariance weighting.
