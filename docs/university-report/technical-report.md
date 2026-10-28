# IntelQCopter
## Autonomous Quadcopter

**Professional and Personal Project**

**IQC-PPP-002 | Pico system edition 5.0 | 27 October 2026**

Student, institution, supervisor and academic year: complete before submission.

Target system: RP2350 flight controller, RP2040 handheld, LoRa main command link and two reserved CC1101 links per side. An onboard Raspberry Pi or NVIDIA Jetson running ROS 2 is being sought; neither has been selected. The red/white F450 cover is AI concept artwork, not a test photograph. Flight acceptance remains pending; motor outputs remain disabled.

---
## Project brief and reading map

[[CONTENTS]]

**Aim:** standalone stable flight, then onboard waypoint navigation and obstacle avoidance. Target: RP2350 aircraft + RP2040 handheld. ESP32-S3/Xbox remains a separate experimental version.

**Evidence legend:** historical log/replay = recorded source; analytic = explanatory model; proposed = unverified target. No new flight or Gazebo campaign was run for this report. Motor outputs remain disabled.


**Math notation:** bold = vector/matrix; dot = time derivative; hat = estimate; bar = sample mean; superscript T = transpose; -1 = inverse; || || = Euclidean norm; Σ = summation when used with indices; exp = exponential. Subscripts name an axis, sample or quantity. Symbols are local to each page.

---
## System architecture and radio links

[[VIS:system-paths]]

[[VIS:pico-responsibilities]]

**Aircraft:** RP2350/Pico SDK; one flight_controller.c. Proposed core 0: bounded 1 kHz acquisition, estimation and PID; core 1: radio and slow work. Timestamped queues, one bus owner, bounded waits.

**Handheld:** RP2040/Pico SDK; one handler_controller.c; two centred joysticks, LB/RB, LT/RT, buttons and screen. Existing 74HC4051 acquisition needs settling/calibration tests. Display, power and gestures remain TBD.

**Radio:** 1 LoRa per side for pilot commands; 2 CC1101 per side reserved, not validated redundancy. Version, sequence, CRC and command age are mandatory. Airtime determines radio cadence; it is independent of the 1 kHz control loop.

**Implementation gap [P6]:** inspected Pico scaffold is 500 Hz with sensor/radio/output stubs. ESP32 timing cannot validate the RP2350 target. Companion: seeking Raspberry Pi 4/5 or NVIDIA Jetson; neither selected.

---
## Flight behaviour and control architecture

[[VIS:control-cascade]]

[[EQ:e_k=r_k-y_k,\qquad u_k=K_p e_k+K_i\sum_{j=0}^{k}e_j\Delta t_j+K_d\dot e_{f,k}]]

**Symbols:** k, j: sample indices; Δt: sample interval (s); r: reference; y: measured output; e: error; u: actuator request; Kp, Ki, Kd: proportional, integral and derivative gains (units depend on the loop); dotted e_f: filtered error derivative.

| Pilot input | Controlled variable | Neutral behaviour |
|---|---|---|
| Right stick | Roll and pitch angles | Level attitude; no horizontal brake or position hold |
| Left horizontal | Yaw rate | Zero commanded yaw rate; no absolute-heading hold |
| Left vertical | Vertical speed | Height retention through the vertical law |
| RT / LT | Bounded increase / decrease | Gentle profile; reserve correction authority |

**Six PID blocks:** 2 angle + 2 rate + 1 yaw-rate + 1 vertical-speed. An outer height-to-speed P law remains an option. Validate transitions, boost and saturation.

**Why PID?** I chose PID to apply what I learned: proportional response, integral error rejection, derivative damping, stability and sampling. Measurable gains connect theory to experiments. Cascades, filters and saturation still require validation.
---
## F450 dynamics and operating point

[[VIS:geometry-forces]]

[[EQ:L=0.225\ \mathrm{m},\quad a=L/\sqrt{2}=0.1591\ \mathrm{m},\quad T=\sum_{i=1}^4T_i]]

[[EQ:m\dot{\mathbf v}=R\mathbf e_3T-mg\mathbf e_3-D\mathbf v+\mathbf f_{ext}]]

[[EQ:J\dot{\boldsymbol\omega}=\boldsymbol\tau-\boldsymbol\omega\times(J\boldsymbol\omega),\quad\boldsymbol\tau_i=\mathbf r_i\times\mathbf F_i]]

**Symbols:** L: centre-to-motor distance; a: X-frame arm projection (m); i: rotor index; T_i, T: individual/total thrust (N); m: mass (kg); v: world velocity (m/s); R: body-to-world rotation; e3: upward unit vector; g: gravity (m/s²); D: drag matrix; f_ext: external force (N); J: inertia matrix (kg m²); ω: body angular velocity (rad/s); τ: torque (N m); r_i: rotor position (m); F_i: rotor force (N). Jxx/Jyy/Jzz are principal-axis inertia entries.

| Supplied simulation parameter | Value | Status |
|---|---|---|
| Mass; stated standard deviation | 1.038 kg; 0.0160312 kg | Not independently measured |
| Jxx = Jyy; Jzz | 0.006421; 0.011853 kg m squared | Approximate COM inertias |
| Hover thrust, total / rotor | 10.179 / 2.545 N | From mg, equal rotor loading |
| Motors / props / ESCs | 1400 KV / 9047 / 40 A | Exact motor response unknown |

**Interpretation:** net force accelerates the mass; lever arms create torque. Nominal ENU world/FLU body frames require physical sign checks. Reweigh the complete target assembly and identify thrust, lag and torque before numerical gain acceptance. Battery mass estimates conflict: 320 g parameter file versus old rough 150 g.
---
## From hover model to digital control

[[VIS:damping-poles]]

[[EQ:G_\phi=\frac1{J_{xx}s^2},\quad G_\theta=\frac1{J_{yy}s^2},\quad G_r=\frac1{J_{zz}s},\quad G_z=\frac1{ms^2},\quad G_{act}=\frac1{\tau_m s+1}]]

[[EQ:K_p=J\omega_n^2,\quad K_d=2\zeta\omega_nJ\quad\mathrm{(ideal\ torque\ PD\ example)}]]

[[EQ:\mathbf x_{k+1}=A_d\mathbf x_k+B_d\mathbf u_k,\quad A_d=e^{A\Delta t},\quad B_d=\int_0^{\Delta t}e^{A\tau}B\,d\tau]]

[[EQ:\phi_{k+1}=\phi_k+\Delta t\,p_k+\frac{\Delta t^2\tau_{x,k}}{2J_{xx}},\quad p_{k+1}=p_k+\frac{\Delta t\tau_{x,k}}{J_{xx}}]]

**Symbols:** G: transfer function; φ/θ: roll/pitch (rad); r: yaw rate (rad/s); z: height (m); s: Laplace variable (1/s); τ_m: actuator time constant (s); ζ: damping ratio; ω_n: natural frequency (rad/s); J: selected axis inertia. x/u: state/input vectors; A/B: continuous state/input matrices; Ad/Bd: sampled equivalents; p: roll rate (rad/s); τ_x: roll torque; τ in the integral: dummy time. Kp/Kd use torque-domain units here.

**Scope:** local hover LTI model; add measured actuator lag, filtering, latency, voltage and saturation. Plot = analytic damping example, not identified gains. Six final PID gains remain TBD.

**Proposed targets:** phase margin >=45 degrees; gain margin >=6 dB; overshoot <=10%; settling within 5% <=1 s. At 1 kHz, use measured sample intervals; sensor and ESC update rates need not equal controller rate.
---
## Calibration by experimental design

[[VIS:calibration-protocol]]

[[EQ:\mathbf r_j=\mathbf b+A\mathbf a_j+\boldsymbol\epsilon_j,\quad\mathbf a_j\in\{\pm g\mathbf e_x,\pm g\mathbf e_y,\pm g\mathbf e_z\}]]

[[EQ:\widehat M=YH^T(HH^T)^{-1}=[\widehat{\mathbf b}\ \widehat A],\quad C=\widehat A^{-1},\quad\mathbf a_{cor}=C(\mathbf r-\mathbf b)]]

**Symbols:** j: pose index; r: raw acceleration vector (m/s²); b: bias; A: dimensionless sensor response matrix; a_j: reference acceleration; ε: measurement error; ex/ey/ez: unit axes; Y: measured pose columns; H: reference/design columns; M: fitted bias-and-response block; C: correction matrix; a_cor: corrected acceleration. H is defined in the text below; g = 9.80665 m/s².

[[VIS:calibration-residuals]]

**Design [P1]:** 18 x 20 s recordings; repetitions 1-2 fit, repetition 3 held out. H contains [1, ax, ay, az] columns and must have full row rank. Six directions identify bias/scale/cross-axis response; one pose cannot. Historical held-out angle errors span about 0.067-1.109 degrees, including fixture uncertainty.

**Gyro:** motionless data estimate bias; known signed rotations test body mapping and scale. MPU offset registers are volatile: restore/read back at boot, then apply only residual software correction. Never subtract bias twice. Full accelerometer matrix correction remains software; it is not three hardware offsets. Signed gyro rotation tests remain pending.
---
## Multisensor fusion versus a simple average

[[VIS:fusion-versus-average]]

For two unbiased independent measurements of the same scalar x, let y1 = x + e1 and y2 = x + e2, with variances R1 and R2. Choose an unbiased linear estimate xhat = (1-w)y1 + w y2; its variance is minimized by inverse-variance weighting.

[[EQ:P(w)=(1-w)^2R_1+w^2R_2,\quad\frac{dP}{dw}=0\ \Longrightarrow\ w^*=\frac{R_1}{R_1+R_2}]]

[[EQ:P^*=\frac{R_1R_2}{R_1+R_2}\le\min(R_1,R_2),\qquad P_{average}=\frac{R_1+R_2}{4}]]

[[EQ:\mathbf w^*=\frac{R^{-1}\mathbf{1}}{\mathbf{1}^TR^{-1}\mathbf{1}},\qquad P^*=(\mathbf{1}^TR^{-1}\mathbf{1})^{-1}\quad\mathrm{(full\ covariance)}]]

**Symbols:** x: true scalar; y1/y2: observations; e1/e2: errors; R1/R2: error variances (measurement units squared); w: weight of sensor 2; P: estimate-error variance; σ: standard deviation; R in the matrix formula: full observation-error covariance; bold 1: vector of ones; bold w: weight vector; *: minimum-variance choice; P_average: equal-weight variance.

**Example:** sigma1 = 1 and sigma2 = 3 in the same units. A 50/50 average gives sigma = 1.581, worse than sensor 1 alone. Optimal weighting gives sigma = 0.949. The noisier sensor contributes only 10 percent. No Gaussian assumption is required for this minimum-variance linear weighting under the stated covariance model.

**Kalman connection:** a prediction is fused with an observation using uncertainty and dynamics. Gyro rate and acceleration cannot be averaged directly: their units and models differ. Integrate the gyro and derive a valid gravity-angle observation first. With equal independent static-sensor variance, the ordinary mean is already optimal among unbiased linear combinations. Known correlation requires the full positive-definite covariance R; bias or model mismatch can invalidate the predicted physical improvement.
---
## Kalman estimation and uncertainty

[[VIS:kalman-loop]]

[[EQ:\theta_k^-=\theta_{k-1}+\Delta t\omega_k,\quad P_k^-=P_{k-1}+Q_k,\quad K_k=\frac{P_k^-}{P_k^-+R_k}]]

[[EQ:\theta_k=\theta_k^-+K_k(z_k-\theta_k^-),\quad P_k=(1-K_k)P_k^-]]

[[VIS:covariance-geometry]]

[[EQ:S=\frac1{N-1}\sum_i(\mathbf x_i-\bar{\mathbf x})(\mathbf x_i-\bar{\mathbf x})^T,\quad\operatorname{Cov}(\bar{\mathbf x})=\frac1{N^2}\sum_{i,j}\operatorname{Cov}(\mathbf x_i,\mathbf x_j)]]

**Symbols:** θ: estimated angle (rad); ω: gyro rate (rad/s); z_k: observed angle, not altitude on this page; P: angle-error variance (rad²); Q: process variance per step; R: observation variance; K: dimensionless Kalman gain; superscript minus: prediction before correction. S: sample covariance; x_i: sample vector; x-bar: mean; N: sample count; Cov: covariance operator; Σ: population covariance.

**Read S:** diagonal = axis variance; off-diagonal = co-variation. Only independent samples give Sigma/N. More samples do not remove bias, drift or fixture error; Gaussianity is not established.

**Historical model [P4]:** limiting K roll 0.00061129, pitch 0.00048326; conditional on its Q/R and timing, not flight tuning. Discrete independent rate noise gives Q = sigma_rate squared x dt squared; a continuous spectral-density model scales with dt instead. Check residuals, autocorrelation and dynamics before selecting gains.
---
## Measured benefit and demonstrated limitation

[[IMAGE:reserved]]

| Reserved replay [P4] | Roll | Pitch |
|---|---|---|
| Raw angle standard deviation | 0.177569 degrees | 0.213074 degrees |
| Filter output standard deviation | 0.007000 degrees | 0.005692 degrees |
| Filter minus observation mean | -0.033769 degrees | -0.039908 degrees |

**Engine:** original Python float64 replay of reserved real 1 kHz data; unchanged plot. Not MCU execution or flight. Fixed 2 s initialization excluded. No independent attitude reference: quieter output does not prove accuracy.

**Adverse test [P4]:** synthetic 0.1g lateral acceleration produced 5.191 degrees peak error; acceleration plus gyro bias produced 5.355 degrees. Apparent gravity tilt is arctan(0.1) = 5.71 degrees while magnitude is only 1.005g. Norm gating alone is insufficient. Dynamic acceptance remains blocked.

---
## Vertical estimation and sensor geometry

[[VIS:vertical-filter]]

[[EQ:z_{k+1}=z_k+\Delta t\,v_{z,k}+\tfrac12\Delta t^2 a_{z,k},\qquad v_{z,k+1}=v_{z,k}+\Delta t\,a_{z,k}]]

[[EQ:P_k^-=A_dP_{k-1}A_d^T+Q_k,\quad K_k=P_k^-H^T(HP_k^-H^T+R_k)^{-1}]]

**Symbols:** z: height (m); v_z: vertical velocity (m/s); a_z: world-frame vertical acceleration after gravity removal (m/s²); k: sample index; Δt: interval (s); state x = [z, v_z] transpose. Ad: state transition; P: state-error covariance; Q: process covariance; H: measurement mapping; R: measurement covariance; K: Kalman gain matrix. Matrix entries carry the corresponding state/measurement units.

[[VIS:vertical-observation]]

The two-state model estimates z and vz; it does not automatically estimate accelerometer bias. Q must follow the selected disturbance model and sample interval. A mass variance cannot be copied directly into Q. Barometer drift, lidar status, tilt projection, terrain, measurement age and dropout all require tests. Bottom-versus-front lidar placement remains a decision, not an interchangeable software setting.
---
## Control authority and real-time evidence

[[VIS:input-headroom]]

[[VIS:timing-budget]]

[[EQ:u=\operatorname{clip}(u_P+u_I+u_D,u_{min},u_{max}),\quad I_{k+1}=I_k+K_i e_k\Delta t+K_{aw}(u-u_{raw})\Delta t]]

**Symbols:** u_raw: unsaturated request; u: limited request; uP/uI/uD: PID contributions; u_min/u_max: bounds; clip: bound to interval; I: integrator contribution (not battery current); Ki: integral gain; Kaw: anti-windup feedback gain (1/s); e: reference minus measurement; k: sample index; Δt: interval (s). Pulse widths are in microseconds.

**Manual target:** gentle sticks, RT increase/LT decrease; proposed collective ceiling 1700-1800 microseconds leaves PID correction headroom. This is a design target, not measured thrust. Anti-windup and transition tests precede acceptance.

**Historical ESP32 timing [P3]:** observed handler maxima 814, 819, 812 and 822 microseconds in successful trials; earlier failures reached 1381 and 1249 microseconds. One malformed trial was rejected. Preserve every trial. Finite acquisition-handler timing is neither complete FC timing nor a hard deadline/WCET proof, and is not RP2350 evidence.

**Next gate:** profile acquisition -> estimation -> PID -> output under radio/sensor load; log missed deadlines, data age and queue loss. Physical motor outputs remain disabled.
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
## Perception for obstacle avoidance

[[VIS:yolo-architecture]]



[[EQ:\mathbf d_c\propto K_c^{-1}[u,v,1]^T,\quad d_{clear}>v\tau+\frac{v^2}{2a_{brake}}+d_{margin}]]

**Symbols:** d_c: camera-frame ray direction; K_c: intrinsic camera matrix; u/v inside [u,v,1]: pixel coordinates; d_clear: available clearance (m); v in the stopping law: speed (m/s); τ: total latency (s); a_brake: available braking deceleration (m/s²); d_margin: clearance allowance (m). These local symbols differ from control input u and world velocity vector v.

**YOLO [R7]:** class, box and confidence; not metric free space. Camera calibration, timestamps, tracking and camera/body transforms are required. Associate the single lidar ray only when geometry/time justify it. Unobserved space stays unknown; no detection is not proof of clearance.

**Hardware limit:** one RGB camera + one fixed ray cannot observe the full collision corridor. Downward lidar aids altitude; forward lidar aids one direction. Placement remains open. Existing known-panel PnP assumes a trained detector and confirmed 1.09 x 1.69 m panel dimensions; it does not recover arbitrary obstacle depth.

**Companion sought:** Pi 4/5 or Jetson. Select by measured end-to-end latency, power, mass, cooling, camera/OS support and cost. Same C++ core does not imply the same binary or inference backend. Metric localization remains unresolved.
---
## Mission nodes and online navigation

[[VIS:mission-nodes]]

[[VIS:navigation-implementation]]

[[EQ:f(n)=g(n)+h(n),\quad r_{inflate}\ge r_{vehicle}+d_{stop}+k_\sigma\sigma_{position}+d_{margin}]]

[[EQ:\|\widehat{\mathbf p}-\mathbf p_i\|\le r_{accept},\quad\|\widehat{\mathbf v}\|\le v_{accept}\quad\mathrm{for}\quad t\ge t_{dwell}]]

**Symbols:** n: search node; f: A* priority; g(n): accumulated path cost, not gravity; h(n): remaining-cost estimate; r_inflate/r_vehicle: map inflation/vehicle radius (m); d_stop/d_margin: stopping distance/margin; kσ: uncertainty multiplier; σ_position: position standard deviation (m); p-hat: estimated position; p_i: node i target; v-hat: estimated velocity; r_accept/v_accept: arrival limits; t/t_dwell: time within limits/required duration.

**Target:** PC chooses n nodes -> approves mission -> companion localizes, maps, replans and tracks -> FC receives bounded references. Intermediate-node advancement must be tested; final arrival and landing are separate policies.

**Existing [P6]:** portable A* and waypoint follower; 0.2 m grid, fixed 0.65 m inflation, bounded XY map. These are prototype assumptions, not validated 3D clearance. Current ROS service accepts start/finish only; a cancellable n-node action remains proposed. Proposed arrival: 0.5 m radius and 1 s dwell, speed bound TBD.

**Transport:** framed 3.3 V UART with CRC, sequence, units, timestamps and expiry. Proposed 10 Hz heartbeat/300 ms expiry need validation. Plugging in a companion never changes mode automatically; approved loss responses remain open.
---
## Proving that the FC really flies the simulation

[[VIS:simulation-proof]]

[[VIS:campaign-confidence]]

**Test contract:** stimulate normal handheld/radio or companion interfaces; log source hashes, messages, estimates and motor requests. Do not apply hidden pose, speed or force corrections to complete a mission. Host-compiled RP2350 C control code requires disclosed shims; it is not RP2350 instruction emulation or hardware timing evidence.

**Planned Gazebo campaign:** ten environments, five noise seeds each; five additional manual command scenarios. Preserve all failures. Video reviewed at 10-15 fps supports trajectory analysis; full-rate logs establish controller timing. No new campaign has been run for this report.

**P targets:** attitude RMS <=2 degrees, altitude RMS <=0.20 m, endpoint error <=0.5 m, zero collision in the declared campaign. They are reviewable objectives, not achievements. Zero failures in 50 independent comparable trials gives a 95 percent upper failure probability of about 5.8 percent, not zero risk.

---
## Materials and resource budget

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

**Software:** Pico SDK C; ROS 2 C++; Gazebo; RViz; YOLO; Python analysis; Git/GitHub. ESP32/Arduino/Xbox belongs to the alternative prototype. Isaac Sim excluded.
---
## Battery estimation and state policy

[[VIS:battery-model]]

[[EQ:SOC(t)=SOC(0)-\frac1{C_{usable}}\int_0^t I(\tau)\,d\tau,\qquad V_{ADC}=V_{pack}\frac{R_{bottom}}{R_{top}+R_{bottom}}]]

**Symbols:** SOC: remaining usable-charge fraction; SOC(0): initial fraction; t/τ: elapsed/dummy time (s); I: discharge current (A, positive out); C_usable: usable charge (A s; multiply Ah by 3600); V_ADC/V_pack: divider output/pack voltage (V); R_top/R_bottom: upper/lower divider resistances (ohms).

**Measurements:** calibrated whole-pack voltage and current, useful capacity, voltage sag and sensor validity. The plots show ideal laws, not measured endurance or an SOC calibration. Marked 120C and four 40 A ESCs do not establish a usable total discharge current.

**Historical requested thresholds:** warning at 35 percent and motor disarm at 20 percent, without automatic safety landing. A motor cut in flight causes a fall; reconfirm this policy explicitly before any flight phase. Hysteresis, persistence, uncertainty and fault responses remain TBD. Whole-pack software monitoring does not protect individual cells.

**Current states:** BOOT -> SELF_TEST -> BENCH_READY only. Reset does not arm. Physical motor outputs remain disabled. One verified regulated avionics source; common grounds; no parallel BEC positives; propulsion current stays outside FC copper. Separate authorization is required for motor tests.
---
## Verification gates and professional method

[[VIS:gates]]

| Gate | Required evidence before progression |
|---|---|
| Bench integrity | Common ground, regulated supplies, non-actuating outputs, reset/readback |
| Sensors and calibration | Held-out poses, signed gyro rotations, reference uncertainty, invalid/dropout recovery |
| Real-time control | Full-load deadlines, stale-command rejection, PID saturation, headroom, source-path trace |
| Pilot and radio | RP2040 inputs/gestures; LoRa airtime, loss, reconnect; coexistence |
| Autonomous mission | Localization, every intermediate node, unknown obstacles, sensor loss; no oracle |
| Power and flight release | Reference voltage/current/SOC tests; approved loss/battery policies; separate motor authorization |

**Acceptance targets, not achievements:** attitude RMS <=2 degrees; height RMS <=0.20 m; endpoint <=0.5 m; zero collisions in the declared campaign. All 50 requirements and 13 verification cases remain in the repository.

**Professional method:** requirement -> model -> experiment -> evidence -> decision. Each checked change gets a descriptive commit and GitHub push. Preserve failed trials; repair a failed step and invalidate downstream acceptance. No history rewriting or invented pass.

**Submission completion:** add student/institution/supervisor, authentic personal reflection, actual contributions, library/AI attribution under university policy, dated quotations and rubric mapping. No full-mark or institutional endorsement is claimed.

---
## Repository sources and evidence access

[[REPO]]

| Version | Role |
|---|---|
| main | Intended RP2350 FC + RP2040 handheld; normal ROS 2 packages |
| esp32-xbox | Alternative ESP32-S3/Xbox prototype |
| codex/stepwise-esp32 | Incremental sensor evidence and this report |

**Technical sources:** [R3] TDK MPU6050 register map; [R4] Bosch BMP280 datasheet; [R5] Nooploop TOFSense-F hardware/protocol; [R7] Ultralytics detection; [R8] MIT OCW 16.30 feedback control; [R9] ROS REP-105 frames; [R10] Raspberry Pi RP2350/RP2040 specifications; [R11] TI CC1101; [R12] Semtech LoRa; [R13] ROS 2 actions/QoS. Full primary-source URLs are preserved in reference/pico-system-edition-v4.1.md.

**Project evidence:** [P1] secured-mount calibration; [P2] gyro restart; [P3] FIFO/gyro frame and timing; [P4] offline attitude replay and adverse models; [P5] software input guards. evidence-manifest.json preserves paths/hashes. [P6] inspected Pico/ROS main commit bce665ad97adbfcb282f02970a1db599e67e6479; exact files in source-inventory.json. Source inspection is not hardware acceptance.

**Supporting dossier:** 38-page prior edition; full materials CSV; requirements register; source/evidence manifests; figure assumptions; preserved failures. Private originals require a sanitized examiner evidence package; public hashes alone do not replace logs.

**Provenance:** green diagrams and analytic plots are labelled; historical replay remains unchanged. Cover = AI F450 concept, not a hardware photograph. Edition dated 27 October 2026; later review dates stay in provenance. No firmware, calibration, motor authorization or flight acceptance is changed by this editorial edition.


