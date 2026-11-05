---
title: "IntelQCopter: A Rigorous Engineering Study"
subtitle: "Dynamics, calibration, estimation, linear and discrete control, fault handling, validation"
author: "Technical study of BenMohStem/IntelQCopter"
date: "1 November 2026"
lang: en-US
abstract: |
  Source-grounded engineering study for IntelQCopter. It derives the equations behind the control stack, identifies branch-specific maturity and code-level gaps, and defines a reproducible plan for measurement, simulation, and safety-critical validation. Nothing in this report certifies airworthiness.
---

# Scope, provenance, and engineering discipline

**Repository:** <https://github.com/BenMohStem/IntelQCopter>. Inspected `main` at commit `e2fa10862b0ca38263dcbf99a8423a6fecfc3223` (repository tree, production source, tests, calibration, control reports); `esp32-xbox` at `af29870011ac36752ed39fd8b8697379e3909f4a`; and `codex/stepwise-esp32` at `fb6103f666083b2debf8a5e72f1f11ff43d91092`. The inspection is targeted rather than a line-by-line review of all 879 tree entries.

**Evidence taxonomy.** In this report, **M** means measured on actual hardware, **I** means implemented in source, **T** means exercised in host/emulator tests, **S** means demonstrated in a simulation with declared assumptions, and **P** means planned or unvalidated. A simulation with a 1.2 kg assumed plant must not be silently promoted to a hardware finding.

**Distinct tracks.** `main` uses an RP2350 flight controller and RP2040 handheld, a 500 Hz target flight-control loop, LoRa packet scaffolding, two angle-plus-bias Kalman filters, a height/vertical-speed Kalman filter, and six PID objects. No real flight sensor, radio or ESC output pipeline is connected to the main runtime. `esp32-xbox` models an ESP32-S3/Xbox/ROS2/Gazebo architecture with broader software integration, but physical flight remains unvalidated. The **stepwise ESP32** development line has genuine IMU and Xbox observations, frozen calibration evidence and an explicit incremental milestone plan; the current installed firmware is a motor-disabled measurement harness, not a validated PID flight controller. The tracks' hardware and evidence must not be mixed.

The mathematical workflow is: define units and reference frames; measure/calibrate; derive a nonlinear model; linearize about a stated operating point; design an estimator; synthesize discrete controllers; verify actuator feasibility; introduce fault transitions; run repeatable tests; and only then consider hardware commissioning. At every step preserve parameter provenance and the original failed as well as successful tests.

# System and model inventory

| Element | Current branch evidence | Limits and open questions |
|:--|:--|:--|
| Frame and propulsion | F450/Q450, 1400 KV motors, 9047 props, four 40 A ESCs, 3S 4200 mAh | Actual motor/prop curves, ESC protocol, frame mass and inertia not identified |
| Flight processor | RP2350 (`main`); ESP32-S3 (other lines) | Target loop frequency is not a demonstrated end-to-end hardware deadline |
| Sensors | MPU6050, BMP280, HMC5883L planned on `main` | Main physical drivers absent; stepwise ESP32 has actual IMU data; magnetometer heading not fused |
| Estimation | Two [angle, bias] linear filters, one [height, vertical speed] filter on `main` | Independent attitude axes; vertical dynamics require correctly rotated specific force |
| Control | Roll and pitch outer P plus inner PI, yaw-rate PI, one altitude PID | Six controller objects; assumed gains; motor allocation coupling |
| Commands | RP2040 22-byte command frame, sequence, CRC, 150 ms timeout | No real LoRa receive in main; CRC is an integrity check, not authentication |
| Safety | Invalid data gates, disabled physical ESC output, battery software flags | Fault recovery and airborne policies not validated; real sensor timestamps absent |
| Companion | ROS2/C++ and perception, Gazebo and other historical simulators | Real metric localization and blind obstacle detection unresolved |

For a first-principles design, the principal unknown parameter vector is

$$
\boldsymbol\theta=(m,J_{xx},J_{yy},J_{zz},\mathbf r_{\rm CG},T_i(u,V),Q_i(u,V),\tau_i,\mathbf b_g,\Sigma_g,\Sigma_a,\Sigma_p,\ldots).
$$

Unknown parameters should be entered as ranges or distributions, not fictitious point measurements.

# Coordinate systems and six-degree-of-freedom dynamics

## Frames and state

The `main` derivations choose local world frame $W$ with $z$ up and aircraft body frame $B$ with $x$ forward, $y$ left, $z$ up (**FLU**). Let $R=R_z(\psi)R_y(\theta)R_x(\phi)$ transform body vectors into world coordinates. Define position $\mathbf r_W$, velocity $\mathbf v_W$, body angular rate $\boldsymbol\omega_B=(p,q,r)^T$, mass $m$, and body inertia matrix $J$.

For a diagonal inertia matrix and a locally rigid body, Newton--Euler gives

$$
\dot{\mathbf r}_W=\mathbf v_W,
\quad m\dot{\mathbf v}_W=R\begin{bmatrix}0\\0\\T_\Sigma\end{bmatrix} - mg\mathbf e_3+\mathbf F_{\rm drag},
$$

$$
J\dot{\boldsymbol\omega}_B+\boldsymbol\omega_B\times (J\boldsymbol\omega_B)=\boldsymbol\tau_B.
$$

The torque cross product matters during simultaneous roll, pitch and yaw. For example,

$$
\dot p=\frac{\tau_x+(J_y-J_z)qr}{J_x}.
$$

A simple small-angle roll design drops $qr$ **only near hover**; a full nonlinear simulator should retain it.

A standard ZYX Euler representation relates the angle derivatives to body rates:

$$
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}
=\begin{bmatrix}
1&\sin\phi\tan\theta&\cos\phi\tan\theta\\
0&\cos\phi&-\sin\phi\\
0&\sin\phi/\cos\theta&\cos\phi/\cos\theta
\end{bmatrix}\begin{bmatrix}p\\q\\r\end{bmatrix}.
$$

Therefore **$p=\dot\phi$ and $q=\dot\theta$ are only small-attitude approximations**. The independent angle Kalman filters in `main` effectively use that approximation. Near $\cos\theta=0$ Euler-rate conversion becomes singular. For larger maneuvers, use a normalized quaternion $q_{WB}$ with $\dot q=\tfrac12 q\otimes[0,\boldsymbol\omega_B]$ (under that quaternion multiplication convention), then derive Euler angles only for display or pilot targets.

## Actuator geometry and the X mixer

Let each motor be $a=D/(2\sqrt2)$ from the body axes, with logical order front-left (FL), front-right (FR), rear-right (RR), rear-left (RL). For $D=0.45$ m,

$$a=\frac{0.45}{2\sqrt2}=0.15910\ {\rm m}.$$

Under a provisional alternating spin convention, the thrust-to-generalized-force map is

$$
\begin{bmatrix}T_\Sigma\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
=\begin{bmatrix}
1&1&1&1\\a&-a&-a&a\\-a&-a&a&a\\-\kappa&\kappa&-\kappa&\kappa
\end{bmatrix}
\begin{bmatrix}T_{FL}\\T_{FR}\\T_{RR}\\T_{RL}\end{bmatrix}.
$$

The signs assume the documented FLU axes and one candidate motor-spin order; **verify the actual vehicle's motor number, propeller handedness, and sensor axes before making physical commands**.

The current normalized mixer forms correction vector

$$
\mathbf d=(r_c-p_c-y_c,\,-r_c-p_c+y_c,\,-r_c+p_c-y_c,\,r_c+p_c+y_c)^T
$$

and adds a common collective. The code calculates the range of corrections, scales all corrections together if that range exceeds available motor span, and shifts collective into a feasible interval. This preserves relative correction proportions more faithfully than independently clipping all four motors, but saturation still changes available control authority.

## Motor physics and uncertainty

The ideal aerodynamic approximation is $T_i=k_T\Omega_i^2$, $Q_i=k_Q\Omega_i^2$, so $\kappa=k_Q/k_T$ has units of length. In practice, $T_i=f(u_i,V_{\rm pack},\mathrm{prop},\mathrm{airflow})$ is nonlinear and may depend on battery sag, motor speed and ESC dynamics. A simple identified lag model is

$$\tau_m\dot T_i+T_i=T_{i,\rm cmd}.$$

This is an approximation to be measured, not a universal physical law. KV indicates approximate unloaded motor speed per applied volt; it cannot give $k_T$, current, propeller efficiency, or thrust-to-weight ratio without tests. An ESC's 40 A rating is not a motor current measurement.

# Hover equilibrium, linearization, and plant transfer functions

## Required operating point

At level stationary hover, $\sum_i T_i=mg$, $\boldsymbol\tau=0$, and for a symmetric airframe $T_{i,0}=mg/4$. The `main` assumed plant uses

$$m=1.2\ {\rm kg},\quad J=(0.012,0.012,0.022)\ {\rm kg\,m^2},\quad T_{\max}=6\ {\rm N/motor},\quad\tau_m=0.04\ {\rm s}.$$

Then $mg=11.76798$ N, $T_{i,0}=2.941995$ N. With the **simulation-only linear thrust surrogate** $T_i=T_{\max}u_i$, the equilibrium command is $u_h=0.4903325$. Normal command maximum $u=0.45$ gives $10.8$ N total, less than weight. The theoretical normal-maximum hover mass under this surrogate is

$$m_{\rm max,normal}=\frac{4\cdot6\cdot0.45}{9.80665}=1.1013\ {\rm kg}.$$

At a 65% boost ceiling, the analogous mass is approximately $1.5908$ kg. Neither is a measured real-world flight limit. The discrepancy means the assumed plant, cap, hover feed-forward, and requested maneuver must be reconciled **without secretly increasing the user's configured limit**.

## First-order Taylor linearization

For dynamics $\dot x=f(x,u;\theta)$, select a real equilibrium $(x_0,u_0)$ and write $x=x_0+\delta x$, $u=u_0+\delta u$. Then

$$\delta\dot x=A\delta x+B\delta u+O(\|\delta\|^2),\quad A=\left.\frac{\partial f}{\partial x}\right|_0,\ B=\left.\frac{\partial f}{\partial u}\right|_0.$$

For quadratic thrust, $T(\Omega_0+\delta\Omega)=k_T\Omega_0^2+2k_T\Omega_0\delta\Omega+k_T(\delta\Omega)^2$. Thus the local actuator slope is $2k_T\Omega_0$, **not a universal constant**. For a measured ESC curve $T=f_T(u)$ the correct slope is $f'_T(u_0)$, evaluated at the operating point.

## Roll rate plant with motor lag

Take the small-angle, decoupled roll equation $J_x\dot p=4aT_{\max}u_\phi^{a}$ and lag $\tau_m\dot u_\phi^{a}+u_\phi^{a}=u_\phi$. Under zero perturbation initial conditions,

$$\frac{P(s)}{U_\phi(s)}=\frac{b_x}{s(\tau_ms+1)},\qquad b_x=\frac{4aT_{\max}}{J_x}.$$

Since the embedded source uses deg/s for rate measurements, $b_{x,\deg}=b_x(180/\pi)\approx18231.4\ \mathrm{deg\,s^{-2}}$ per normalized torque proxy. The angle plant has one additional integrator:

$$G_\phi(s)=\frac{b_x}{s^2(\tau_ms+1)}.$$

Analogous yaw-rate effectiveness under the assumed reaction-torque coefficient is $b_{\psi,\deg}=4\kappa T_{\max}(180/\pi)/J_z\approx937.57$ deg/s$^2$ per normalized proxy. These are **model effectiveness numbers**, not measured motor capability.

## Vertical plant

Set $\lambda=d_v/m=0.12/1.2=0.1$ s$^{-1}$ and $b_h=4T_{\max}/m=20$ m/s$^2$ per normalized collective. The small-signal height plant is

$$G_h(s)=\frac{b_h}{s(s+\lambda)(\tau_ms+1)}=\frac{20}{s(s+0.1)(0.04s+1)}.$$

These transfer functions are **local and unsaturated**. They omit yaw/roll coupling, wind, voltage sag, aerodynamic prop wash, contact, estimator error, network delay and varying load.

# Six linear controllers: derivation and actual gains

The six objects on `main` are (1) roll-angle P, (2) roll-rate PI, (3) pitch-angle P, (4) pitch-rate PI, (5) yaw-rate PI, (6) altitude PID. A controller object with zero I or D gain remains a valid PID structure. This is **not** six full independent three-term controllers.

## Rate PI synthesis

For initial design, ignore actuator lag so $G(s)=b/s$. With $C(s)=K_p+K_i/s$ the characteristic equation is

$$1+C(s)G(s)=0\ \Longrightarrow\ s^2+bK_ps+bK_i=0.$$

Match it to $s^2+2\zeta\omega_ns+\omega_n^2$:

$$\boxed{K_p=\frac{2\zeta\omega_n}{b},\qquad K_i=\frac{\omega_n^2}{b}.}$$

With roll/pitch $\zeta=0.9$, $\omega_n=8.4$ rad/s, and the degree-based model $b=18231.4054$, the gains are $K_p=0.000829338$ and $K_i=0.003870245$. With yaw $\omega_n=2.8$ rad/s, the yaw gains are $K_p=0.005375614$ and $K_i=0.008362066$. This reproduces the gain constants in `flight_controller.c` and `docs/model-guide/control.tex` **under the declared plant assumptions**.

Restore motor lag. The rate-loop characteristic becomes

$$\tau_ms^3+s^2+bK_ps+bK_i=0.$$

A necessary and sufficient Routh--Hurwitz condition for strictly positive $\tau_m,b,K_p,K_i$ is

$$\boxed{K_p>\tau_mK_i.}$$

For roll, $0.000829338>0.04(0.003870245)=0.000154810$, so the assumed local rate loop passes. But this result is not a proof for sampled, clipped, delayed or physically unidentified actuators.

## Angle cascades

The outer roll and pitch controller uses $p_{\rm ref}=K_a(\phi_{\rm ref}-\hat\phi)$ with $K_a=3$ s$^{-1}$; at $3^\circ$ angle error it requests $9^\circ$/s body-rate correction under the small-angle assumption. The inner controller tracks that rate. Including the rate PI and motor lag, the cascade characteristic is

$$\tau_ms^4+s^3+bK_ps^2+b(K_i+K_aK_p)s+K_abK_i=0.$$

For the baseline roll and pitch model, the repository's design document reports positive quartic Routh determinants $\Delta_2=10.4832$, $\Delta_3\approx1003.5325$. The corresponding continuous **local linear** model is Hurwitz. The result does not certify robust stability against wrong axis signs, changing motor effectiveness or constrained allocation.

## Altitude PID synthesis and the fourth-order test

The controller uses $e_h=h_{\rm ref}-\hat h$ and estimates derivative error from commanded and measured vertical velocity rather than numerically differentiating a barometer:

$$u_h=K_pe_h+K_i\int e_hdt+K_d(v_{\rm cmd}-\hat v).$$

An ideal double-integrator approximation $G=b_h/s^2$ and a target polynomial $(s+p_h)(s^2+2\zeta_h\omega_hs+\omega_h^2)$ yield

$$K_d=\frac{2\zeta_h\omega_h+p_h}{b_h},\quad K_p=\frac{\omega_h^2+2\zeta_h\omega_hp_h}{b_h},\quad K_i=\frac{p_h\omega_h^2}{b_h}.$$

With $\omega_h=1.05$, $p_h=0.56$, $\zeta_h=0.9$, $b_h=20$, the source uses $(K_p,K_i,K_d)=(0.108045,0.03087,0.1225)$.

Restore drag and lag:

$$0.04s^4+1.004s^3+2.55s^2+2.1609s+0.6174=0.$$

For a quartic $a_4s^4+a_3s^3+a_2s^2+a_1s+a_0$, positive coefficients are insufficient: also require

$$\Delta_2=a_3a_2-a_4a_1>0,\qquad \Delta_3=a_3a_2a_1-a_4a_1^2-a_3^2a_0>0.$$

The model gives $\Delta_2=2.473764$ and $\Delta_3\approx4.72321$, so the **unsaturated continuous local** altitude model is stable. The repo also derives an uninstalled alternate candidate $(0.09912357,0.02796822,0.1103068)$ which matches the known assumed lag and drag; it must not be mistaken for the installed tune.

## PID in discrete time and saturation

For samples at actual intervals $\Delta t_k$, a practical PI/PID update is

$$e_k=r_k-y_k,\quad I_k=\operatorname{clip}(I_{k-1}+e_k\Delta t_k,I_{\min},I_{\max}),$$

$$D_k=-\frac{y_k-y_{k-1}}{\Delta t_k},\quad u_k=\operatorname{clip}(K_pe_k+K_iI_k+K_dD_k,u_{\min},u_{\max}).$$

The source's `pid_update` performs derivative-on-measurement, per-controller output clamping, and conditional integration. **However, the downstream four-motor allocator can still saturate even when each upstream PID is individually unsaturated.** A more complete anti-windup study must track achieved versus requested collective and torque after allocation, then freeze/back-calculate relevant integrators and log saturation by axis.

For discrete linear stability, form the augmented plant **including estimators, integral states, computational/communication delays and actuators** and verify all closed-loop eigenvalues lie inside the unit circle $|z_i|<1$. A 500 Hz target alone proves nothing about rate of missed deadlines, filtering latency or actual sensor age.

# Sensor physics, calibration, and measurement uncertainty

## The critical distinction: accelerometer specific force

An ideal accelerometer reads specific force, not world acceleration:

$$\mathbf f_B=R^T(\ddot{\mathbf r}_W-\mathbf g_W),\quad \mathbf g_W=(0,0,-g)^T.$$

At level rest it reads approximately $[0,0,g]^T$; in ideal free fall it reads approximately zero. Correct vertical acceleration requires

$$a_{W,z}=[R\mathbf f_B]_z-g$$

with consistent m/s$^2$ units, calibrated axes and correct orientation. A static accelerometer tilt estimate may be corrupted by lateral acceleration or propeller vibration even if it is statistically precise at rest.

## Gyroscope bias and precision

Use a stationary model $\omega_{m,i}=b_{g,i}+n_i$ and estimate $\hat b_{g,i}=\frac1N\sum_{k=1}^N\omega_{m,i,k}$. Correct each axis with $\omega_{c,i}=\omega_{m,i}-\hat b_{g,i}$. If samples are independent with variance $\sigma_i^2$, then $\operatorname{Var}(\hat b_{g,i})=\sigma_i^2/N$; for correlated samples replace $N$ by an effective sample count (the naive expression can underestimate uncertainty).

`main` demands at least 500 stationary samples and initial per-axis gyro sigma $\le0.1$ deg/s; it rejects implausibly large biases or accelerometer-motion evidence. These are provisional software acceptance gates. Bias is temperature-dependent, and **bias estimation alone cannot calibrate scale or axis alignment**. The stepwise branch has real IMU data and frozen coefficients; do not repeat the full campaign without a named failing reference check.

## Six-face accelerometer fit

For axis $i$, assume raw readings $r_i=b_i+s_i f_i$ in g. In poses with expected specific force $f_i=+1$ and $-1$, let the means be $\mu_i^+$, $\mu_i^-$. Then

$$b_i=\frac{\mu_i^++\mu_i^-}{2},\quad s_i=\frac{\mu_i^+-\mu_i^-}{2},\quad g_i=\frac1{s_i}=\frac{2}{\mu_i^+-\mu_i^-}.$$

The corrected reading is $\hat f_i=g_i(r_i-b_i)$. As an example, if positive and negative face means are $1.035$ g and $-0.965$ g, then $b_i=0.035$ g, $s_i=1$, $g_i=1$. A raw level reading $1.035$ g corrects to $1.000$ g. Use independent withheld orientations for validation: fitting the calibration poses is not proof of general accuracy. A full $3\times3$ calibration matrix may be necessary if cross-axis misalignment is significant.

## Magnetic calibration and heading

A useful general model is $\mathbf m_{raw}=S\mathbf m_B+\mathbf b_m+\mathbf n_m$ with hard-iron offset $\mathbf b_m$ and soft-iron distortion $S$. The corrected field is $\hat{\mathbf m}_B=S^{-1}(\mathbf m_{raw}-\mathbf b_m)$; a full ellipsoid fit uses all 3D orientations to estimate cross-axis terms. The source presently implements a preliminary min/max plus diagonal scale fit (minimum 300 data points with coverage checks), not a full soft-iron matrix. Heading must also be tilt-compensated using a consistent body/world frame and local magnetic declination if geographic north is required. Motor-current-dependent magnetic interference cannot be removed by one static zero-current fit. The magnetometer is not currently fused into the `main` yaw estimate.

## Barometric height

The BMP280 produces raw temperature and pressure counts plus factory trim coefficients; compensation is not simply a scale multiplication. The `main` code decodes the coefficients and implements pressure/temperature compensation. Physical register acquisition is not connected on `main`. Relative height from a ground pressure reference $p_0$ may be approximated by

$$h\approx44330\left[1-\left(\frac{p}{p_0}\right)^{0.1903}\right]\ {\rm m}.$$

Near sea level, $dh/dp\approx-0.083\ \mathrm{m/Pa}$ (approximate). Changes in weather, rotor wash, port shielding and thermal conditions can create biases; a stable barometer is not automatically an accurate absolute altimeter. Source `docs/calibration-flash.md` correctly stores calibration coefficients but recomputes volatile ground-height zero each session.

## Ranging, battery sensors, and calibration persistence

Distance, voltage and current can use reference-pair regression $y=ax+b$ when the physical relationship is sufficiently linear. Validate against withheld points across operating range. For a downward rangefinder over a flat ground plane, body-axis slant range requires tilt correction and a known mounting orientation; invalid returns and surface-dependent errors need explicit checks. Coulomb counting requires a real current sensor; a voltage divider alone cannot measure discharged charge.

`main` calibration flash reserves two 4096-byte sectors, checks schema, CRC and profile/board identity, commits the inactive slot last, verifies saved gyro bias, and writes only while disarmed. These are useful robustness mechanisms but do not establish brownout endurance or physical persistence without a real power-interruption test. CRC32 detects accidental corruption but does not authenticate calibration data.

# Kalman filtering and data fusion derived from the source

## General linear filter

A linear sampled estimator uses

$$x_{k+1}=Fx_k+Bu_k+w_k,\qquad z_k=Hx_k+v_k,$$

where $w_k$ and $v_k$ are zero-mean noises of covariances $Q$ and $R$ under the model's assumptions. Prediction and innovation are

$$\hat x^-_k=F\hat x^+_{k-1}+Bu_{k-1},\quad P^-_k=FP^+_{k-1}F^T+Q,$$

$$\nu_k=z_k-H\hat x^-_k,\quad S_k=HP^-_kH^T+R,\quad K_k=P^-_kH^TS_k^{-1}.$$

Correction is $\hat x^+_k=\hat x^-_k+K_k\nu_k$ with numerically robust Joseph covariance update

$$P^+_k=(I-K_kH)P^-_k(I-K_kH)^T+K_kRK_k^T.$$

Every covariance entry has physical units. A positive covariance does not make an incorrect physical measurement model correct.

## Main roll/pitch filter

For one axis, $x=(\alpha,b_g)^T$ with input $\omega_m$, the source uses

$$F=\begin{bmatrix}1&-\Delta t\\0&1\end{bmatrix},\quad B=\begin{bmatrix}\Delta t\\0\end{bmatrix},\quad H=\begin{bmatrix}1&0\end{bmatrix}.$$

The state prediction is $\alpha^- = \alpha^+ +(\omega_m-b_g^+)\Delta t$ and $b_g^-=b_g^+$. The accelerometer supplies a tilt-angle pseudo-measurement $z=\alpha_{acc}$. The modeled process covariance is

$$Q=\begin{bmatrix}\sigma_g^2\Delta t^2+q_b\Delta t^3/3&-q_b\Delta t^2/2\\-q_b\Delta t^2/2&q_b\Delta t\end{bmatrix}.$$

`main` initializes $\sigma_g^2=0.04$ (deg/s)$^2$ per sample, $q_b=0.0001$ (deg/s)$^2$/s and $R=4$ deg$^2$. With scalar angle measurement, $S=P_{00}^-+R$, $K=(P^-_{00},P^-_{10})^T/S$. The *off-diagonal* covariance allows the accelerometer innovation to correct the gyro-bias estimate even though the accelerometer does not directly measure bias.

**Critical limitations:** gyro body rates are approximately treated as Euler rates, horizontal accelerations are treated as gravity-induced tilt, and the core has no innovation gate or measurement-age field. If a lateral acceleration generates an apparent 15-degree tilt, a small configured $R$ can make the filter overtrust the wrong attitude observation. Candidate remedies are a 3D quaternion attitude estimator, acceleration-norm/dynamics consistency tests, adaptive covariance inflation, and innovation-based fault gates. These must be evaluated on saved physical logs and simulated maneuvers, not merely added as heuristics.

## Altitude filter and process-noise conventions

Take $x=(h,v_z)^T$ and known world-up acceleration $a_k$ assumed constant per sample:

$$x_{k+1}=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}x_k+\begin{bmatrix}\Delta t^2/2\\\Delta t\end{bmatrix}a_k+w_k.$$

If a random acceleration error $\eta_k$ with variance $\sigma_a^2$ is held constant during each sample,

$$Q_{held}=\sigma_a^2\begin{bmatrix}\Delta t^4/4&\Delta t^3/2\\\Delta t^3/2&\Delta t^2\end{bmatrix}.$$

If acceleration error is instead **continuous white noise** of intensity $q_c$, then

$$Q_{continuous}=q_c\begin{bmatrix}\Delta t^3/3&\Delta t^2/2\\\Delta t^2/2&\Delta t\end{bmatrix}.$$

These are not interchangeable. Source `altitude_kf_predict` uses the held-sample model with $\sigma_a=1.5$ m/s$^2$, not a continuous white-noise spectral density. Measurements update via $H=[1,0]$. The default embedded `altitude_estimator` initializes `baro_noise=2.0`, whereas the host SITL reset changes it to `0.0025`, **an 800-fold variance difference**. A bench-to-simulation comparison must explicitly explain or reconcile those configurations. The discrepancy alone does not prove which value is right.

## Consistency tests and gating

For a scalar observation, normalized innovation squared (NIS) is

$$\mathrm{NIS}_k=\frac{\nu_k^2}{S_k}.$$

If the noise model is calibrated, a scalar measurement yields approximately a $\chi^2$ distribution with one degree of freedom. For example, a 99% upper quantile is about 6.63; repeated unusually large innovations may indicate a wrong motion model, abnormal acceleration, magnetic disturbance, or sensor fault. Rejecting every large sample blindly can itself cause divergence. Log innovation, predicted covariance, actual reference residual and confidence. Check NEES or Monte Carlo coverage when independently known truth exists; apply robust estimation when outliers are expected.

For nonlinear fusion, the extended Kalman filter (EKF) predicts through $f(x,u)$ and linearizes with the Jacobians $F_k=\partial f/\partial x$ and $H_k=\partial h/\partial x$ around the current estimate; a quaternion error-state formulation avoids large-angle Euler singularities. It is not automatically superior when unmodeled acceleration, time delay, mounting angles or incorrect reference frames dominate the error.

# Timing, numerical correctness, and fault-handling architecture

## Deterministic scheduling

The RP2350 target schedules 500 Hz (2 ms) and rejects control steps with $\Delta t>10$ ms. A target period is not an observed response-time distribution. Record for every sample: acquisition timestamp, sensor sequence, filter execution timestamp, command age, control-computation duration, motor-output transmission duration, and missed-deadline count. Estimate not only average latency but worst-case and upper quantiles; distinguish sensor data-ready time from the time the host reads a FIFO burst. The stepwise ESP32 observed about 1000.57 FIFO samples/s over a preferred 20-second gyro recording with a maximum retrieval gap of 2.586 ms; this is **not** a full PID-loop timing guarantee.

The RP2040 command frame is 22 bytes at nominal 100 Hz and contains CRC-16/CCITT-FALSE, 16-bit sequence, bounded axes/triggers and six buttons. The RP2350 command watchdog expires a frame after 150 ms. This detects many accidental bit errors and stale packets but does not authenticate their origin, prove radio reliability, or handle reboot/rejoin semantics without a protocol design.

## A rigorous fault taxonomy

A practical FC should distinguish **invalid numerical value**, **valid but stale sample**, **out-of-range physical value**, **innovation inconsistent with model**, **sensor bus malfunction**, **control deadline miss**, **lost command authority**, **motor/actuator saturation**, **brownout/power fault**, and **mode-transition fault**. Each fault requires (i) detection and timestamp, (ii) severity, (iii) state transition, (iv) actuator demand rule, (v) event logging, (vi) explicit recovery permission.

Example evidence matrix:

| Fault injection | Expected mathematical behavior | Acceptance evidence |
|:--|:--|:--|
| IMU sample marked invalid | No new control demand; stop/inhibit state is explicit | Host unit test plus MCU boundary test |
| Valid flag but sample 200 ms old | Should be rejected based on acquisition time, not reprocessed | Timestamp fault-injection test; missing today on `main` |
| Inject NaN into rate or target | No propagation to motor commands | Finite-input/output property tests |
| 150 ms expired pilot command | Autonomous/manual authority transition must be defined | Parser/watchdog test and physical link fault plan |
| One motor ceiling reached | Achieved generalized forces and integrator recovery logged | Allocation stress tests under current/voltage limits |
| Accelerometer lateral acceleration | Tilt observation should not silently dominate the gyro | Sensor replay and innovation statistics |
| Incorrect gyro-axis sign | Negative feedback must be rejected before physical output | With-propellers-removed sign fixture and motor-disabled plot |
| Barometer unavailable | Height-hold unavailable; state transition documented | Replay + scenario test |
| Flash interrupted during save | Last committed calibration remains available if compatible | Hardware interruption/endurance test, pending |
| Battery 20% policy reached | Current source latches mathematical disarm regardless of altitude | Explicit airborne risk assessment; hardware path not connected |

**Code-specific observations (not claims of demonstrated failures):** `sample_is_healthy` checks `imu_valid`, `imu_calibrated`, and finite six IMU measurements, but it has no acquisition timestamp or plausibility bounds for a moving sample. `flight_control_step` requires barometer validity for its only flight mode and does not fuse magnetometer heading. It resets controller states on several invalid-input conditions. These are intentional software scaffold choices and future integration obligations, not evidence of physical safety.

The battery policy in current `main` source is warning at estimated SOC $\le35\%$ and *latched immediate disarm at* $\le20\%$, with no automatic landing. Executing that rule while airborne could cause uncontrolled descent; it must undergo explicit hazard analysis and cannot be marketed as a flight-safe battery failsafe. Some older documentation describes a landing recommendation; prefer the actual current source and date-stamped policy. Pack SOC accuracy requires current measurement, capacity characterization and safe pack-voltage behavior. Never substitute a fixed aggregate voltage threshold for cell-level protection.

## A verifiable state machine

A proposed state decomposition for future hardware—not claimed to exist in `main`—is `BOOT -> SENSOR_CHECK -> CALIBRATION_REQUIRED/READY -> ARMED_IDLE -> TAKEOFF -> HOLD/MANUAL/AUTONOMOUS -> LANDING -> DISARMED`, plus well-defined `FAULT_INHIBIT` transitions. Every arrow must have guard conditions, persistence semantics, controlled authority ownership and tests. On the ground, faults usually inhibit actuation; in flight, action depends on which control channels are still trustworthy. Sensor loss, command loss, battery emergency and complete flight-controller failure cannot all be solved by a single generic "set motor output to zero" rule. Test emergency handling with propellers removed and in simulation before any controlled real-aircraft commissioning.

# Autonomous navigation and observability

An inertial sensor does not give drift-free horizontal position by double integration: even a tiny acceleration bias creates position error growing as $\frac12 b_at^2$. The barometer observes approximate height, and a downward single-ray range sensor can aid ground distance, but neither determines horizontal $x,y$ position. A monocular camera can provide image bearings and visual features; a bounding box alone does not produce unique metric distance without known object size, geometry, stereo/depth, motion parallax or another metric prior. Detecting known colored panels in simulated scenes does not generalize automatically to unknown obstacles, occlusion and changing light.

Coordinate frame consistency is part of perception correctness: camera optical frame, body FLU frame, odometry/world frame, magnetometer frame, and ROS conventions must be connected by calibrated transforms with timestamps and covariance. System-level mission states must respect authority arbitration between pilot, autopilot and companion; the companion should emit bounded position/velocity/attitude targets rather than bypassing the low-level safety boundary.

# Identification and validation program

## Recommended order of measurement

1. **Identity and reference frames.** Confirm actual boards/sensors, register configuration, axis arrows and rotation-matrix determinant $+1$. Preserve frozen calibrations; use existing IMU logs first.
2. **Mass and center of gravity.** Weigh complete assembled drone, not just reference frame and battery. Record each component's mass and position.
3. **Inertia.** Use a bifilar or torsional suspension method; estimate $J_x,J_y,J_z$ with uncertainty and repeatability, and record the setup formula/geometric assumptions.
4. **Sensor metrology.** Verify gyro angle against independent known physical rotation; run six-face accelerometer validation; pressure zero against known elevation changes; inspect timestamps, FIFO gaps, saturation and temperature trends.
5. **Propulsion map on a secured test bench.** With appropriate professional precautions, capture static thrust, power, command response, voltage and temperature. Verify motor order and spin. Never attempt these with an unsecured aircraft or nearby people.
6. **Model fitting.** Estimate $T(u,V)$, local hover slope $dT/du$, thrust lag $\tau_m$, drag and frame inertia; report confidence intervals and a held-out dataset.
7. **Estimator replay.** Run exact production filter functions on recorded sensor streams with saved reference motion and simulated faults; report innovations, RMSE, lag and covariance consistency.
8. **Discrete controller test.** Test tracking, disturbances, yaw-pitch-roll coupling, injection of biased/stale sensors, allocation saturation and missed deadlines across the parameter uncertainty box.
9. **Physical integration boundary.** Require independently verified pins, ESC protocol and stop signal, real calibrated data age, disarmed-only flash writes, current/voltage telemetry and an explicit safety review before physical motor arming.

## Performance metrics

For a command step of amplitude $\Delta r$, report rise time $t_r$, 2% settling time $t_s$, overshoot $M_p=(y_{peak}-r)/\Delta r$ for an upward step, steady-state error, RMS tracking error $\sqrt{\frac1N\sum e_k^2}$, peak angular rate, and integrated absolute error $\sum|e_k|\Delta t$. Also report motor-command headroom, fraction of time saturated, minimum pack voltage, estimator innovations and sample freshness. Compare identical experiments on baseline/alternate plants; do not tune until the measured physical actuator slope is known.

For uncertainty analysis, perturb mass, inertia, motor lag, slope and sensor variances across bounded credible ranges. A positive Routh criterion at one nominal point proves only nominal local stability. An apparent 673/675 grid success is a result for the **chosen grid and model**, not a probability of flight safety. Use counterexamples and boundary cases to refine the model, not to discard failures.

# Source-grounded review: priorities

| Priority | Review issue | Evidence and proposed verification |
|:--|:--|:--|
| Critical, before any flight | Under 1.2 kg/linear 6 N surrogate, 45% normal cap is below ideal hover | Check `sim/model/simulation_parameters.json`, `docs/motor-limits-2026-10-28.md`; identify real measured thrust curve |
| Critical, before any flight | Real sensor/radio/ESC adapter absent on RP2350 `main` | `flight_controller.c` entry point intentionally creates empty samples and never arms |
| Critical, before any flight | Physical sign/frame mapping not accepted | Stepwise plan and October 30 gyro review require known physical reference |
| High | Source battery policy disarms at 20% even airborne | Study consequences and safe architecture; do not change user policy silently |
| High | Embedded barometer variance 2.0 vs SITL 0.0025 | Align units and configuration provenance with actual sensor characterization |
| High | No fresh timestamp/innovation gate in `sample_is_healthy` | Define stale and inconsistent-sample tests and recovery strategy |
| High | Upstream PI anti-windup does not account for downstream mixer saturation | Log requested and achieved generalized inputs; test integrator recovery |
| High | Independent roll/pitch angle filters do not represent full 3D kinematics | Use retained attitude data to decide quaternion/observer requirements |
| High | Unknown full mass/inertia and motor thrust/current/lag curves | Prioritize measurements; report uncertainty and withheld-fit validation |
| Medium | No real horizontal localization/metric unknown-obstacle validation | Establish camera calibration, odometry, metric geometry and failure gates |

# Guided problem set with solutions

**Problem 1 (hover feasibility).** For $m=1.2$ kg and four 6 N maximum-thrust surrogate motors, calculate ideal hover command and normal thrust margin with $u_{max}=0.45$.

*Solution.* $u_h=mg/(4T_{max})=0.49033$. At 45%, maximum thrust $10.8$ N while weight is $11.76798$ N. Margin $10.8-11.76798=-0.96798$ N. The surrogate cannot hover with the normal cap; this conclusion is conditional on linear thrust and declared mass.

**Problem 2 (rate PI gains).** Derive $K_p,K_i$ for $G(s)=b/s$ and desired roots with $\zeta=0.9$, $\omega_n=8.4$ rad/s, $b=18231.4$. Then assess lag $\tau_m=0.04$ s.

*Solution.* $K_p=2\zeta\omega_n/b=0.00082934$, $K_i=\omega_n^2/b=0.00387025$; $K_p-\tau_mK_i\approx0.00067453>0$, so the specific continuous third-order rate model passes Routh stability.

**Problem 3 (accelerometer calibration).** A sensor-axis face pair gives means $+1.06$ and $-0.94$ g. Compute offset and corrective gain.

*Solution.* $b=(1.06-0.94)/2=0.06$ g; $g=2/(1.06-(-0.94))=1$. A raw $0.56$ g becomes $0.50$ g. An independent tilted reference orientation is still required for verification.

**Problem 4 (gyro drift).** A residual constant bias of $0.1$ deg/s remains after calibration. How far can a pure gyro-integrated angle drift in 30 seconds?

*Solution.* $\Delta\alpha=b_g t=3^\circ$ (neglecting other errors). This motivates bias estimation/verification and trustworthy external corrections.

**Problem 5 (Kalman noise).** For the angle filter with $R=4$ deg$^2$ and prior $P^-_{00}=1$ deg$^2$, compute the angle Kalman gain if $P^-_{10}=0$.

*Solution.* $K_\alpha=1/(1+4)=0.2$. The correction moves 20% of the innovation into the angle state at that step; the gain changes as covariance evolves.

**Problem 6 (sample noise).** At 500 Hz, calculate $Q_{vv}$ for a held-sample vertical acceleration standard deviation 1.5 m/s$^2$.

*Solution.* $Q_{vv}=\sigma_a^2\Delta t^2=2.25(0.002)^2=9\times10^{-6}$ m$^2$/s$^2$ per prediction. Do not use the same numeric 1.5 as a continuous spectral-density parameter.

**Problem 7 (saturation).** Suppose a roll PI output itself is under its `output_limit`, but the mixer must uniformly reduce correction amplitude by 30%. Does the existing `pid_update` conditional anti-windup necessarily stop the rate integrator?

*Solution.* No. The PI sees its own internal saturation test; it does not see the final achieved torque after allocator scaling. An allocation-aware tracking or back-calculation scheme is a design investigation.

**Problem 8 (data-age fault).** A driver returns a finite, previously calibrated gyro sample repeatedly for 200 ms while leaving `imu_valid=true`. Can `sample_is_healthy` alone detect the fault?

*Solution.* No; there is no acquisition timestamp or repeated-sequence check in that helper. The driver or controller boundary must impose independently tested freshness semantics.

# A finite curriculum for this project

| Module | Theoretical learning target | Practical artifact and exit criterion |
|:--|:--|:--|
| 1. Coordinates | SO(3), Euler vs body rates, rotations, units | Frame/sign diagram and a 3-axis transform replay |
| 2. Dynamics | Newton--Euler, rotor torque, 6-DOF model | Verify nonlinear equations against current simulator |
| 3. Parameters | Least squares, uncertainty, mass/inertia/thrust identification | Source-labelled parameter table with error bars |
| 4. Sensors | Specific force, gyro drift, calibration matrices | Saved real reference comparisons; do not refit without evidence |
| 5. Data fusion | Covariance, KF/EKF, observability, NIS/NEES | Logged innovation and attitude/height truth study |
| 6. Linear control | Equilibria, Jacobians, Laplace, PID pole placement, Routh | Reproduce exact `main` gains and locate stability limits |
| 7. Discrete control | Sampling, zero-order hold, delay and integrators | End-to-end timing profile and sampled pole analysis |
| 8. Actuators | Mixing, force allocation, saturation, anti-windup | Command-to-thrust model and saturation stress tests |
| 9. Fault handling | Safety invariants, state machines, watchdogs, recovery | Fault-injection matrix and explicit state-transition tests |
| 10. Autonomy | Localization, visual geometry, navigation authority | Mission replay using measurable metric state and known limitations |

**Recommended starting lesson:** coordinate frames and recorded IMU axis signs, followed immediately by rotational dynamics and the current rate PI derivation. For the stepwise ESP32 track, the next practical milestone is a **live, motor-disabled attitude preview using existing preserved calibration**, not another unbounded calibration campaign.

# Primary repository references

1. [README, main architecture and simulation caveats](https://github.com/BenMohStem/IntelQCopter/blob/main/README.md).
2. [Production RP2350 flight-control C source](https://github.com/BenMohStem/IntelQCopter/blob/main/firmware/flight_controller/flight_controller.c) and [production-source host tests](https://github.com/BenMohStem/IntelQCopter/blob/main/firmware/flight_controller/tests/test_flight_controller.c).
3. [Mathematical derivations and current controller design](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/model-guide/control.tex) and [complete model report](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/modelisation/main.pdf).
4. [Simulation parameter assumptions](https://github.com/BenMohStem/IntelQCopter/blob/main/sim/model/simulation_parameters.json) and [calculated design script](https://github.com/BenMohStem/IntelQCopter/blob/main/tools/model/calculate_current_control_design.py).
5. [Sensor calibration](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/sensor-calibration.md), [persistent storage](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/calibration-flash.md), and [motor policy](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/motor-limits-2026-10-28.md).
6. [Unresolved engineering information](https://github.com/BenMohStem/IntelQCopter/blob/main/docs/missing-information-and-problems.md) and [measurement worksheets](https://github.com/BenMohStem/IntelQCopter/tree/main/measurements).
7. [Stepwise ESP32 development milestones](https://github.com/BenMohStem/IntelQCopter/blob/codex/stepwise-esp32/docs/PLAN.md) and [preferred real gyro recording review](https://github.com/BenMohStem/IntelQCopter/blob/codex/stepwise-esp32/measurements/mpu6050/2026-10-30-rotation-review/README.md).
8. [BMP280 manufacturer datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf), [MPU6050 manufacturer product specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf), and [PX4 multicopter tuning and mixer explanation](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter).

**Final safety boundary:** This document is an analysis and curriculum. It does not authorize flashing, motor arming, running propellers, physical flight, or changes to a battery policy. Real acceptance requires independently reviewed physical hardware, calibrated axes, test environments, measurable controller outputs and staged operator decisions.
