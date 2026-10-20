# Worked drone model, calibration and flight-controller integration

This note answers the four connected questions: what gyro calibration actually
does, how our current model leads to control/estimator gains, how ROS communicates
with the C core, and how that same core participates in native Gazebo simulation.
Numerical calculations are reproducible with
`tools/calculate_current_control_design.py`; the full results are in
`artifacts/model-design/current/calculations.json`. These are mathematical model
calculations, not new implementation tests or a new flight recording. No firmware,
gain initialization or running simulation parameter was changed.

![Calculated local continuous poles, not a new flight test](../artifacts/model-design/current/local-model-poles.png)

## 1. Gyro calibration: yes, it averages stationary measurements

For each axis, use the measurement equation

$$g_k=\omega_k+b+n_k,$$

where $g_k$ is measured rate, $\omega_k$ is true rate, $b$ is zero-rate bias and
$n_k$ is zero-mean noise. A stationary aircraft has approximately $\omega_k=0$.
For $N$ accepted fresh measurements,

$$\hat b=\frac1N\sum_{k=1}^N g_k,\qquad
g_k^{\rm corrected}=g_k-\hat b.$$

Thus 0.8 degrees/s is an example estimated bias, not a universal correction or a
number hard-coded into the firmware. Negative biases are subtracted with their
sign: a bias of -0.3 means adding 0.3 to a later reading.

### How the existing C calculates this

`imu_stationary_cal_add` accepts one sample pair; `cal_stats3_add` implements
Welford's online mean/variance, independently for x,y,z:

1. Increment $N$.
2. Compute $\delta=g_k-\bar g_{N-1}$.
3. Update $\bar g_N=\bar g_{N-1}+\delta/N$.
4. Update $M_{2,N}=M_{2,N-1}+\delta(g_k-\bar g_N)$.
5. At finalization, $s^2=M_2/(N-1)$ estimates sample variance.

This produces the ordinary arithmetic mean without storing every sample and
with better numerical behavior than subtracting two large raw squared sums.
`imu_gyro_cal_finish` stores $\hat b_i=\bar g_i$ and
$\operatorname{Var}(\hat b_i)\approx s_i^2/N$. `imu_cal_apply` subsequently
subtracts the stored mean from each gyro axis. `sensor_sample_apply_imu_calibration`
is an optional wrapper that writes corrected values back into a sensor sample.

The current acceptance rules include:

- Disarmed capture; fresh samples are a caller responsibility.
- At least 500 samples; equal accelerometer/gyro sample counts.
- Initial per-sample acceleration magnitude 0.8..1.2 g and each rate <=10 degrees/s.
- Final mean acceleration magnitude 0.9..1.1 g.
- Each mean gyro rate <=5 degrees/s in magnitude.
- Per-axis accelerometer variance <=0.0004 g² and gyro variance <=0.01 (degrees/s)².

The last gyro bound means a capture sigma no greater than 0.1 degrees/s per axis.
At $N=500$, that corresponds to bias-mean sigma no greater than
$0.1/\sqrt{500}\approx0.00447$ degrees/s **under an independent-sample assumption**.
Filtered adjacent samples can be correlated, so $s^2/N$ can be optimistic. The
number of iterations does not by itself establish elapsed time or independence.

### What is already implemented versus still missing

Mean removal, variance checking and fit uncertainty already exist; replacing them
with a simpler average would not improve them. Physical startup acquisition,
warm-up/temperature qualification, ensuring unique samples, storage in flash and
actually applying the fit in the Pico's real driver path remain unfinished. The
Pico main currently supplies invalid zero sensor data rather than a calibrated IMU.

A future physical calibration should record sensor temperature and repeat after
warm-up at several stable temperatures. Fit $b_i(T)$ only after measuring it; do
not invent a compensation slope or continuously relearn zero rate while rotating.
For correlated samples, estimate effective sample count or uncertainty from
independent batches. Long stationary records/Allan analysis help distinguish white
noise, drift and random-walk behavior. Current thresholds are provisional: for
example, a sensor/capture with sigma 0.2 degrees/s will be rejected by the 0.1
degrees/s capture bound, even though simulation deliberately injects 0.2 degrees/s.
That mismatch must be resolved using measured sensor settings/noise, not by
silently accepting motion or pretending synthetic noise is measured hardware.

Temperature dependence and a digital temperature channel are documented in the
[MPU6050 specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf).

## 2. Separate supplied information from assumptions

| Parameter | Current knowledge |
|---|---|
| Frame | F450/Q450 family; 0.45 m motor diagonal is the working geometry, not a measured clone dimension |
| Motor | Orange hobby motor, 1400 KV; exact model/thrust curve unknown |
| Propeller | 9047: nominal 9 inch diameter and 4.7 inch pitch; mass/aerodynamic coefficients unknown |
| ESC | 40 A rating; this is not measured current or thrust response |
| Battery | 3S 4200 mAh 120C; user-estimated mass 150 g, unverified |
| Controller period | Intended 0.002 s; host simulation uses this virtual interval |
| Real vehicle mass/inertia | Not yet measured |
| Thrust gain/lag/yaw drag torque | Not yet identified |

Two different assumed models exist and must not be mixed:

| Quantity | Recorded flight baseline | Separate component URDF estimate |
|---|---:|---:|
| Mass, kg | 1.2 | 0.740 |
| $J_x$, kg m² | .012 | .00885852 |
| $J_y$, kg m² | .012 | .00906322 |
| $J_z$, kg m² | .022 | .01718682 |
| Component measurements verified? | No | No |

The component estimate uses guessed 330 g frame, 55 g each motor, 10 g each prop
and the user's approximate 150 g battery. Other electronics/ESC masses were
excluded as requested. The actual completed airframe may differ substantially.
The native Gazebo recording has not been switched to this 740 g model.

For the worked baseline only, assume maximum thrust $T_{max}=6$ N per motor,
reaction torque/thrust ratio $\kappa=.015$ m, thrust lag $\tau=.04$ s and linear
drag $d=.12$ N s/m. They cannot be deduced uniquely from 1400 KV, 9047, 40 A and
3S. KV describes a speed/voltage relationship, not a thrust/command curve.

Numerical precision of calculations must not be confused with accuracy of these
assumptions. Performance objectives such as settling time, overshoot, noise
amplification and allowable motor saturation must also be selected: plant
parameters alone do not specify one unique PID design.

## 3. Construct the nonlinear differential equations

Use a right-handed body frame x forward, y left, z up. ROS angles normally use
radians; the current inner C API uses degrees. The bridge must explicitly convert
them. See [ROS REP 103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst).

Let $r,v$ be world position/velocity, $R$ rotate body vectors into world axes,
$\omega=[p,q,r_b]^T$ be body angular velocity, and $J$ inertia about the centre
of mass. We use $r_b$ for yaw rate here to avoid confusing it with position $r$.

### Translation

Newton's law gives mass times acceleration equal to the sum of external forces:

$$\dot r=v,$$
$$m\dot v=R\begin{bmatrix}0\\0\\T\end{bmatrix}
-\begin{bmatrix}0\\0\\mg\end{bmatrix}-dv+f_{dist},\qquad
T=\sum_{i=1}^{4}T_i.$$

The thrust vector points along body z. Multiplying by $R$ gives its world
direction. Gravity points down in world z. Linear drag opposes velocity.

### Rotation

Angular momentum balance in rotating body coordinates gives

$$J\dot\omega=\tau_B-\omega\times(J\omega),\qquad
\dot R=R[\omega]_\times.$$

If $J$ is diagonal, the x equation is
$J_x\dot p=\tau_x+(J_y-J_z)qr_b$, with cyclic counterparts for y,z.
These coupling products vanish to first order around a stationary level hover.
General coupled flight still needs them; scalar transfer functions are local models.

### Motors, thrust and mixer

A more physical rotor model would use $T_i=k_T\Omega_i^2$ and
$Q_i=k_Q\Omega_i^2$, with measured speed/current/voltage dynamics. Those
coefficients are unknown. The current simulator instead assumes

$$\tau\dot u_i^a+u_i^a=u_i,\qquad T_i=T_{max}u_i^a.$$

Here normalized demand $u_i$ is **linear in the simulator's thrust model**;
it is not established that real ESC PWM or DShot command is linear in thrust.
For a better real model linearized about hover, replace $T_{max}$ in local
control effectiveness by the measured derivative $dT/du$ at the operating point.

For motor diagonal $D$, the x/y motor coordinate magnitude is

$$a=\frac{D}{2\sqrt2}=0.1590990258\ {\rm m}.$$

With motor order FL,FR,RR,RL, thrusts produce

$$\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}=
\begin{bmatrix}
1&1&1&1\\
a&-a&-a&a\\
-a&-a&a&a\\
-\kappa&\kappa&-\kappa&\kappa
\end{bmatrix}
\begin{bmatrix}T_{FL}\\T_{FR}\\T_{RR}\\T_{RL}\end{bmatrix}.$$

The corresponding unsaturated mixer is

$$\begin{aligned}
u_{FL}&=c+u_\phi-u_\theta-u_\psi,\\
u_{FR}&=c-u_\phi-u_\theta+u_\psi,\\
u_{RR}&=c-u_\phi+u_\theta-u_\psi,\\
u_{RL}&=c+u_\phi+u_\theta+u_\psi.
\end{aligned}$$

Substitution shows $T=4T_{max}c$, $\tau_x=4aT_{max}u_\phi$,
$\tau_y=4aT_{max}u_\theta$ and $\tau_z=4\kappa T_{max}u_\psi$.
These signs/spin assumptions describe the simulator; physical motors/axes still
need verification. The C allocator changes this simple linear relationship when
torque demands saturate or collective is shifted.

### Hover equilibrium

At level hover, zero acceleration requires $T=mg$. Therefore

$$T_{hover,each}=\frac{mg}{4}=2.941995\ {\rm N},\qquad
c_0=\frac{mg}{4T_{max}}=0.4903325$$

for the 1.2 kg/6 N baseline. This explains the nominal .5 hover value. It is not
a real motor setting derived from the KV label. The 740 g estimate would require
1.81423025 N each and a different feed-forward under the same assumed thrust model.

## 4. Obtain local transfer functions

Linearize at $v=0$, $\omega=0$, level attitude, and $T=mg$. For small angles,
$\dot\phi\approx p$, $\dot\theta\approx q$, $\dot\psi\approx r_b$;
$\ddot x\approx g\theta$ and $\ddot y\approx-g\phi$ if yaw is zero.

Define control effectiveness in the C controller's degree-based units:

$$b_\phi=\frac{4aT_{max}}{J_x}\frac{180}{\pi},\quad
b_\theta=\frac{4aT_{max}}{J_y}\frac{180}{\pi},\quad
b_\psi=\frac{4\kappa T_{max}}{J_z}\frac{180}{\pi},\quad
b_h=\frac{4T_{max}}m,\quad\lambda=\frac dm.$$

With zero perturbation initial conditions, Laplace transforming
$\dot p=b_\phi u_\phi^a$ and $\tau\dot u_\phi^a+u_\phi^a=u_\phi$
gives $sP=b_\phi U_\phi^a$ and $(\tau s+1)U_\phi^a=U_\phi$. Divide to obtain

$$G_p(s)=\frac{P(s)}{U_\phi(s)}=\frac{b_\phi}{s(\tau s+1)},\qquad
G_\phi(s)=\frac{\Phi(s)}{U_\phi(s)}=\frac{b_\phi}{s^2(\tau s+1)}.$$

Pitch has the same forms with $b_\theta$; yaw rate uses $b_\psi/[s(\tau s+1)]$.
Rate is already second order with actuator lag; angle is third order.

For vertical perturbations $\delta c=c-c_0$, gravity cancels at equilibrium:

$$\ddot h+\lambda\dot h=b_h\delta c^a.$$

Laplace transformation plus the same thrust lag gives

$$G_h(s)=\frac{H(s)}{\Delta C(s)}=
\frac{b_h}{s(s+\lambda)(\tau s+1)}.$$

Ignoring drag and lag reduces this to $b_h/s^2$, but this simpler model does not
retain all dynamics used in the current baseline. Numerically:

$$G_p(s)=\frac{18231.4054}{s(.04s+1)},\quad
G_{r_b}(s)=\frac{937.5673}{s(.04s+1)},\quad
G_h(s)=\frac{20}{s(s+.1)(.04s+1)}.$$

Angular outputs above are degrees/s or degrees; substituting radian-based gains
without the conversion would change loop gain by about 57.3.

## 5. Derive the current six gains

### Inner rate PI

First neglect motor lag to choose an initial bandwidth. With $G=b/s$ and
$C=K_p+K_i/s$, unity feedback gives

$$1+CG=0\quad\Longrightarrow\quad s^2+bK_ps+bK_i=0.$$

Choose target polynomial $s^2+2\zeta\omega_ns+\omega_n^2$ and match coefficients:

$$K_p=\frac{2\zeta\omega_n}{b},\qquad K_i=\frac{\omega_n^2}{b}.$$

The selected simulation design uses $\zeta=.9$, roll/pitch
$\omega_n=12(.7)=8.4$ rad/s and yaw $\omega_n=4(.7)=2.8$ rad/s. These bandwidths
are engineering choices, not measurements inferable from motor KV.

The outer roll/pitch angle controller is P-only with $K_a=3$; its output is a
rate target. The six `pid_t` objects therefore include two P controllers, three
PI controllers and one altitude PID. Rate-loop gains use degree-based inputs.

### Single altitude PID

For the simplified double-integrator plant $G=b_h/s^2$, the PID characteristic is

$$s^3+b_hK_ds^2+b_hK_ps+b_hK_i.$$

Choose $(s+p_h)(s^2+2\zeta_h\omega_hs+\omega_h^2)$, expand, and equate coefficients:

$$K_d=\frac{2\zeta_h\omega_h+p_h}{b_h},\quad
K_p=\frac{\omega_h^2+2\zeta_h\omega_hp_h}{b_h},\quad
K_i=\frac{p_h\omega_h^2}{b_h}.$$

Use $\omega_h=1.5(.7)=1.05$ rad/s, $p_h=.8(.7)=.56$/s and $\zeta_h=.9$.
The C code implements the height PID's derivative contribution as
$K_d(v_{cmd}-\hat v)$, avoiding a numerical difference of noisy height.
The moving reference satisfies approximately $\dot h_{ref}=v_{cmd}$ unless its
distance limiter activates. There is no additional seventh vertical-speed PID.

### Result for the existing 1.2 kg baseline

| Loop | Kp | Ki | Kd |
|---|---:|---:|---:|
| Roll angle | 3 | 0 | 0 |
| Roll rate | .0008293381 | .0038702447 | 0 |
| Pitch angle | 3 | 0 | 0 |
| Pitch rate | .0008293381 | .0038702447 | 0 |
| Yaw rate | .0053756141 | .0083620664 | 0 |
| Altitude | .108045 | .03087 | .1225 |

These reproduce the current scale-.7 design and C initializers; they are not new
hardware tuning recommendations. Changing mass/inertia changes the calculated
gains even if desired bandwidth is unchanged. The separate 740 g estimate gives
different gains, saved alongside the baseline in the JSON, without installing them.

## 6. Check the dynamics we neglected during initial gain matching

### Rate PI with motor lag

Using $G=b/[s(\tau s+1)]$ instead of $b/s$ changes the characteristic to

$$\tau s^3+s^2+bK_ps+bK_i.$$

For $b,\tau,K_p,K_i>0$, the remaining cubic Routh condition is
$bK_p>\tau bK_i$, equivalently $K_p>\tau K_i$.
For roll/pitch it becomes

$$.04s^3+s^2+15.12s+70.56,$$

whose poles are $-7$ and $-9\pm13.0767j$. They are not the poles of the
lag-free target second-order polynomial. This is why a second-order initial
design is not the end of the calculation.

### Complete roll/pitch cascade

The closed inner rate transfer is

$$T_p(s)=\frac{b(K_ps+K_i)}{\tau s^3+s^2+bK_ps+bK_i}.$$

The outer loop sees $T_p/s$ and gain $K_a$. Multiplying $1+K_aT_p/s=0$ by its
denominator gives

$$\tau s^4+s^3+bK_ps^2+b(K_i+K_aK_p)s+K_abK_i.$$

For the baseline this is

$$.04s^4+s^3+15.12s^2+115.92s+211.68.$$

For a quartic $a_4s^4+a_3s^3+a_2s^2+a_1s+a_0$ with positive coefficients,
the additional Hurwitz conditions are

$$\Delta_2=a_3a_2-a_4a_1>0,$$
$$\Delta_3=a_3a_2a_1-a_4a_1^2-a_3^2a_0>0.$$

Here $\Delta_2=10.4832$ and $\Delta_3=1003.532544$, both positive. The complete
continuous local cascade poles are approximately
$-2.54124$, $-10.67161$, $-5.89357\pm12.66510j$.

### Altitude with lag and vertical drag

Using the fuller vertical plant gives

$$\tau s^4+(1+\tau\lambda)s^3+(\lambda+b_hK_d)s^2+b_hK_ps+b_hK_i.$$

With the existing gains:

$$.04s^4+1.004s^3+2.55s^2+2.1609s+.6174.$$

Its Hurwitz determinants are 2.473764 and 4.7232075492, both positive. Its poles
are approximately $-22.3550$, $-1.37553$, $-.684731\pm.181921j$.

These results establish local continuous unsaturated model stability with ideal
state/rate feedback. They do **not** include Kalman-estimator dynamics, radio/ROS
delay, sampled implementation, battery sag, thrust uncertainty, contact, saturation
or the full nonlinear model. A sampled analysis must discretize the motor/vehicle
state with the actual hold/update order, augment the controller/filter states and
check all eigenvalues lie inside the unit circle. It is not enough to display the
continuous poles or assume that every 500 Hz implementation has zero delay.

### An analytically matched fourth-order altitude candidate

One can include the assumed lag/drag in coefficient matching instead of ignoring
them. Choose

$$\tau(s+\rho)(s+p_h)(s^2+2\zeta_h\omega_hs+\omega_h^2).$$

The fixed cubic coefficient requires
$\rho=1/\tau+\lambda-p_h-2\zeta_h\omega_h=22.65$/s. Expanding the other terms gives

$$K_d=\frac{\tau[\omega_h^2+2\zeta_h\omega_hp_h+\rho(p_h+2\zeta_h\omega_h)]-\lambda}{b_h},$$
$$K_p=\frac{\tau[p_h\omega_h^2+\rho(\omega_h^2+2\zeta_h\omega_hp_h)]}{b_h},\qquad
K_i=\frac{\tau\rho p_h\omega_h^2}{b_h}.$$

For the assumed baseline this yields Kp=.09912357, Ki=.02796822, Kd=.1103068.
It exactly matches the selected continuous local target poles
$-22.65$, $-.56$, $-.945\pm.457684j$. This is an **uninstalled analytic candidate**,
not a claim of improved full-flight results. Current code/gains are unchanged.

## 7. Calculate Kalman gains from noise and sampling, not motor KV

PID design uses plant dynamics and performance objectives. Kalman design uses
state dynamics, observation models, covariance assumptions and sampling times.
Sensor calibration estimates bias/noise, but cannot independently identify every
in-flight process-noise term or make these assumptions automatically correct.

### Roll/pitch angle and bias

For each angular axis,

$$x=\begin{bmatrix}\alpha\\b_g\end{bmatrix},\quad
F=\begin{bmatrix}1&-\Delta t\\0&1\end{bmatrix},\quad H=[1,0],$$
$$x^-_{k}=Fx^+_{k-1}+\begin{bmatrix}\Delta t\\0\end{bmatrix}g_k.$$

The current code's noise convention is

$$Q=\begin{bmatrix}
\sigma_g^2\Delta t^2+q_b\Delta t^3/3&-q_b\Delta t^2/2\\
-q_b\Delta t^2/2&q_b\Delta t
\end{bmatrix},\qquad R=\sigma_{\alpha,acc}^2.$$

Prediction and correction:

$$P^-=FP^+F^T+Q,\qquad S=HP^-H^T+R,$$
$$K=P^-H^TS^{-1},\quad\nu=z-Hx^-,\quad x^+=x^-+K\nu,$$
$$P^+=(I-KH)P^-(I-KH)^T+KRK^T.$$

Using the existing assumptions $\Delta t=.002$, $\sigma_g^2=.04$,
$q_b=.0001$ and $R=4$ degree², the conditional limiting gain is approximately

$$K_\alpha=.000966191,\qquad K_b=-.000223499.$$

The negative bias gain is expected from the angle model's negative bias
coupling; it is not an invalid probability. Conditional posterior sigmas approach
.06217 degrees for angle and .02079 degrees/s for residual bias. These are model
covariance predictions, not measured flight-error bounds. Startup gains differ.

Accelerometer angle noise is not directly `accel_noise_g²`: observation geometry
must be propagated. At level rest, $\alpha\approx a_y$ in radians when acceleration
is expressed in g, so $\sigma_\alpha^2\approx(180/\pi)^2\sigma_{a_y,g}^2$.
During translational acceleration, that gravity-based observation is biased;
stationary white-noise variance alone does not describe it.

### Altitude: 500 Hz prediction, 50 Hz height observations

$$x=\begin{bmatrix}h\\v\end{bmatrix},\quad
F=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix},\quad
G=\begin{bmatrix}\Delta t^2/2\\\Delta t\end{bmatrix},\quad H=[1,0],$$
$$x^-=Fx^++Ga_{up},\qquad Q=G\sigma_a^2G^T.$$

With $\sigma_a=1.5$ m/s² and $\Delta t=.002$, each prediction adds

$$Q=\begin{bmatrix}9\times10^{-12}&9\times10^{-9}\\9\times10^{-9}&9\times10^{-6}\end{bmatrix}.$$

The host reset uses $R_{baro}=.0025$ m². Applying ten predictions then one
height correction yields periodic limiting gains at correction time:

$$K_h\approx.0834306,\qquad K_v\approx.181649.$$

Do not apply those as fixed gains every 2 ms: the height observation interval is
20 ms. The actual C calculates its current gain from covariance whenever a fresh
measurement is flagged.

If a valid independent lidar-derived height with $R_{lidar}=.0004$ m² is fused
after each barometer correction, the conditional periodic gains become:

| Correction | Height gain | Velocity gain |
|---|---:|---:|
| Barometer first | .0207565 | .0741164 |
| Lidar second | .1148311 | .4100348 |

Startup, missing floor returns, tilt, terrain, dropouts and other observation
ordering change these values. Sharing attitude error between projected lidar
height and inertial acceleration can introduce correlations not represented by
this simple independent-noise calculation.

The garden injects barometer sigma .03 m (variance .0009), while the host filter
uses .0025; lidar noise sigma .01 m is injected, while the update uses .0004.
Those conservative tuning variances are not measured sensor calibrations. Keep
injected noise, assumed filter noise and actual observed residual/error statistics
separately labelled rather than claiming they are identical.

## 8. Trace parameter uncertainty without inventing distributions

For small uncertainty, propagate a parameter covariance by a Jacobian:

$$\Sigma_y\approx J_f\Sigma_\theta J_f^T,\qquad
J_f=\frac{\partial f}{\partial\theta}.$$

For example $b_\phi\propto aT_{max}/J_x$, so
$\delta b_\phi/b_\phi\approx\delta a/a+\delta T_{max}/T_{max}-\delta J_x/J_x$.
With fixed target bandwidth, $K_p,K_i\propto1/b_\phi$, reversing those signs.
For a fixed simplified altitude target, gains scale with $m/T_{max}$.

The same component measurements influence both mass and inertia; their errors
can be correlated. Do not automatically add independent percentages if the same
weighing/position error contributes to several terms. Uniform/Gaussian parameter
distributions and standard deviations need measured support or an explicit
scenario assumption. Current JSON numbers are point assumptions, not confidence
intervals. Local Hurwitz determinants can be recomputed across declared parameter
ranges, but a finite grid alone does not prove every continuous combination stable.

Measurements that matter most for real gain accuracy are full takeoff mass/COM,
component positions/inertia, 3S thrust-versus-command near hover, motor step
response/lag, yaw torque effectiveness, and timestamped sensor noise/drift at the
actual digital filtering/sample settings. Simulate these as uncertain parameters
until hardware arrives; no board selection is required for the derivation.

## 9. How ROS 2 communicates with the flight controller

### What exists today: workstation simulation

```text
Gazebo state + synthetic sensor generation / shared C++ mission core
    -> local TCP IPC (port 9241)
    -> ROS adapter publishes garden/sensors_commands
    -> DDS delivers to garden_c_flight_core subscriber
    -> subscriber calls the compiled flight_controller.c DLL
    -> garden/control_reply returns four motor demands + diagnostics
    -> local IPC reply -> motor dynamics -> native Gazebo force/torque
```

`sim/garden/ros_transport.py` implements the simulator-side `RosFlight` calls.
`sim/ros2_garden_bridge.py` runs the actual DDS publishers/subscribers and calls
`qc_sitl_step` through ctypes. `sim/engines/ros_bridge.py` adapts evidence paths for
the native engine. Both DDS adapter nodes currently run in one process. The C core
is host-compiled, not an RP2350 firmware image/emulator running inside Gazebo.

Separately, the compiled C++ ROS mission/perception nodes publish an atomic
`autonomy/setpoint` message containing roll/pitch angles in radians, yaw rate in
rad/s, centered throttle and validity. They have passed separate DDS integration
checks; that entire ROS C++ topic graph is not yet the recorded flight path.

### Future physical connection: proposed, not implemented

```text
ROS C++ mission -> autonomy/setpoint -> wired bridge on companion
       USB CDC or UART frames -> RP2350 core-1 parser -> bounded setpoint mailbox
       RP2350 core 0 runs estimators/PIDs locally at 500 Hz -> eventual ESC drivers
```

The microcontroller does not currently run ROS 2 or micro-ROS. A wired protocol
still needs implementation: framing/version, message type, sequence plus session,
CRC, bounded units, heartbeat/freshness, acknowledgements/status and explicit
pilot/companion authority. The existing 22-byte LoRa pilot frame is not already a
ROS serial protocol. Pins/baud/USB identity remain deployment choices, not things
already configured on an absent board.

Convert ROS radian setpoints to C degrees at one documented boundary. Use local
reception time plus negotiated sequence/session for freshness rather than trusting
that remote ROS and Pico clocks share an epoch. A slower companion setpoint stream
(for example 20–50 Hz) must not clock the inner 500 Hz PID loop. Loss of companion
commands needs an explicit valid fallback/landing supervisor; do not copy the
current scaffold's zero-motor early returns into an armed airborne design.

Telemetry should return state estimates, covariances/innovations, battery status,
sensor/link health, selected command authority and timing/drop counts. Physical
arming remains disabled. LoRa stays the planned pilot channel; unused CC1101
channels are separate from the wired companion interface.

## 10. How the existing C core is integrated into Gazebo

### Build the same C source for the host

`tools/build_sitl.ps1` compiles `flight_controller/flight_controller.c` with
`QCOPTER_HOST_TEST` and `QCOPTER_SITL_DLL` into `build/sitl/flight_core.dll`.
These flags expose the host ABI and exclude Pico-specific main/peripheral calls;
the PID/filter/mixer equations remain the same. The C++ mission/navigation core
is compiled into a separate host library.

### Supply measurements, not perfect controller states

The simulator generates body specific force and angular rate from native rigid
body state, adds declared noise, and supplies barometer/lidar models. The C
controller estimates roll/pitch/height/vertical speed. Horizontal localization for
navigation currently comes from simulated odometry. Camera pixels come from native
Gazebo rendering. The current downward lidar is an analytical floor ray, not a
native range-sensor or physical TOFSense driver.

`qc_sitl_step` consumes nine sensor floats (acceleration xyz, gyro xyz, upward
acceleration, height, new-height-sample flag), five command floats (roll, pitch,
yaw rate, throttle, boost), dt and validity flags. It returns four normalized
motor demands and sixteen diagnostics. The external simulation mission supervisor
provides virtual spool/landing gating; this is not physical arming permission.

### Close the physical feedback loop

For each .002 s simulation interval:

1. Read current native Gazebo state/camera buffers.
2. Generate the declared sensor observations and timestamp/update flags.
3. Obtain bounded mission/pilot-mode commands.
4. Execute the C control step, directly or through the documented DDS adapter.
5. Evolve each motor's assumed first-order thrust lag.
6. Calculate total thrust and the three differential torques.
7. Rotate body force/torque into world coordinates and apply native link wrench.
8. Advance native Gazebo physics by one step and repeat.

`sim/gazebo/engine_server.py` uses `Link.add_world_wrench` in the simulation
pre-update callback, then obtains the resulting native state after the physics
step. It does not move the drone by setting pose to follow a prerecorded path.
Gazebo's [Link API](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1Link.html)
documents the world-frame wrench and its one-step lifetime.

Launch the existing integrated recorded flight from the project root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/run_native_garden.ps1 -RunPrefix model-study-01
```

Use a new run name each time. Videos, CSV, gains/uncertainty diagnostics and
validation go under `artifacts/simulation/2026-10-20-engines`; original stdout and
stderr are under `artifacts/test-runs/native-local`. The simulator's .002 s virtual
interval is not proof of a 2 ms wall-clock DDS round trip or physical MCU deadline.

The separate component URDF/RViz model is not automatically the physics body used
by this flight adapter. Importing an URDF by itself neither runs a C controller nor
adds aerodynamic thrust. When replacing the body, update mass, COM, inertia,
rotor positions and force application consistently, then recalculate/assess gains.
The next architectural integration remains the complete standard ROS C++ camera,
pose, perception, mission and command graph, followed later by a physical wired
bridge; neither has been silently claimed complete by these model calculations.
