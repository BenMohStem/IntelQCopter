# Control workshop: from settings to the moving drone

> Scope: native Gazebo 10.5 physics and OGRE2 cameras; the actual ESP32 flight controller C source compiled into a Windows DLL; actual ROS2 C++ mission, planner, YOLO and known-panel PnP processes. This is software integration evidence. The flight uses synthetic sensors, synthetic calibration and perfect Gazebo localization. It does not validate real ESP32 timing, BLE, UART wiring, motors or flight safety. Physical ESC outputs remain disabled.

## 1. Start, save, stop and restart

From the **QCopter-ESP32-Xbox** checkout, run ` .\qcopter.ps1 station ` and open **http://127.0.0.1:8088**.

1. Choose FC-only manual or FC + onboard ROS2, and one of ten garden layouts.
2. Expand a settings group, or leave all defaults as they are.
3. Start. A new tab opens immediately and automatically loads the live station when Gazebo is ready.
4. Use the real Xbox. For navigation, review and approve all route nodes once.
5. End session. The session stops, its automatic recording is finalized, and the flight tab closes. The home workshop stays open.

Every launch saves `settings.json` inside its unique `artifacts/gazebo-suite/<run>` directory. A reset preserves these settings and creates another original trial. The saved profile only changes future launches. It cannot change a flight already running.

The video records at **15 frames per simulation second**. `flight.csv` records physics/FC values at **500 samples per simulation second**. `telemetry.jsonl` records live C estimator diagnostics at the camera cadence. `wire-inputs.jsonl` or `mapped-xbox.jsonl` records what actually entered the FC. `real-xbox.csv` preserves the original XInput reports.

Video is written automatically, using fragmented MP4. Completed fragments are more recoverable after interruption than a single unfinished MP4 index, but abruptly killing a process can lose the final fragment. Use End session. Closing a browser tab leaves the flight running. The workshop remains available after the flight ends; `.\qcopter.ps1 station-stop` also requests a graceful save.

## 2. Which process has authority?

### Manual

**Real Xbox → Windows XInput → explicitly recorded Gentle stick shaping → raw 16-byte HID → C FC → four motor fractions → motor lag/thrust → Gazebo physics.**

Gentle maps the largest right-stick request to 5° and softens small inputs. The FC still uses angle control for roll/pitch and rate control for yaw. With fresh horizontal localization, the C FC now brakes on right-stick release, then holds the captured release position. A held stick still commands an angle. Braking requires a temporary counter-tilt; zero angle alone cannot stop existing horizontal velocity. Without a valid localization feed, the FC falls back to angle leveling and cannot promise horizontal hold. The simulation explicitly supplies perfect Gazebo x/y, world velocity and yaw as a sensor observation; real hardware has no such driver yet.

### Autonomous

**PC route approval once → drone ROS2 mission → onboard observed obstacle map and A* planner → roll/pitch angle, yaw rate, vertical speed → CRC-protected QN UART frame → C FC → motor fractions → physics.**

The PC does not stream coordinates for the drone to follow. The drone computer visits the intermediate nodes itself. The simulator supplies odometry, RGB and ground contact. Supplying perfect pose is a substantial simplification: real single-RGB-camera navigation still needs a tested localization method. Known-panel PnP estimates only the deliberately recognizable panels; it is not general depth sensing.

The isolated authority proof sends a mode-selection gesture and then only identical neutral pilot packets. Its PC node does not subscribe to odometry or images. It sends one route request. The wire audit checks neutral HID, CRCs, monotonically increasing companion sequence numbers, publisher identity, intermediate-node acknowledgements and the final landing/disarm frame. A synthetic feedback-based “virtual human” from older tests is not evidence of real human control.

Gazebo integrates forces. The only model wrench in `engine.py` is computed from the C core's motor demands, motor lag and drag. There is no mission-dependent force, teleport, scripted velocity or position-setting operation after the initial spawn. True obstacle geometry is used for collision/clearance measurement, not passed into the onboard planner.

## 3. Six control loops and the exact discrete PID

For a setpoint $r_k$, measurement $y_k$ and sample period $h=0.002\,\mathrm{s}$:

$$
e_k=r_k-y_k,
\qquad I_k=\operatorname{clip}(I_{k-1}+h e_k,-I_{\max},I_{\max}).
$$

This equation gives the candidate integral. The implementation only accepts that candidate when the output is unsaturated, or when the error drives an already saturated output back toward its allowed range. The output itself is also clamped to the loop limit. On the first sample its derivative is zero.

The reusable PID differentiates the **measurement**, avoiding a derivative kick when the command changes:

$$
D_k=-\frac{y_k-y_{k-1}}{h},
\qquad u_k=K_p e_k+K_i I_k+K_d D_k.
$$

For an angle loop, degrees in and degrees/second out imply $K_p$ in $\mathrm{s}^{-1}$, $K_i$ in $\mathrm{s}^{-2}$ and dimensionless $K_d$. For a rate loop, degrees/second in and normalized motor correction out imply $K_p$ in $\mathrm{s/degree}$, $K_i$ in $1/\mathrm{degree}$ and $K_d$ in $\mathrm{s^2/degree}$.

The roll cascade is:

$$
\phi_{\mathrm{request}}
\xrightarrow{\text{angle PID against }\hat\phi}
p_{\mathrm{request}}
\xrightarrow{\text{rate PID against }p_{\mathrm{gyro}}}
u_\phi.
$$

Pitch has the equivalent two loops. Yaw has one rate loop. The sixth PID is altitude. These provisional gains are the unchanged defaults:

| Loop | Kp | Ki | Kd | Integral state bound |
|---|---:|---:|---:|---:|
| Roll angle | 3 | 0 | 0 | 0 |
| Pitch angle | 3 | 0 | 0 | 0 |
| Roll rate | 0.000829338 | 0.003870245 | 0 | 15 |
| Pitch rate | 0.000829338 | 0.003870245 | 0 | 15 |
| Yaw rate | 0.005375614 | 0.008362066 | 0 | 8 |
| Altitude | 0.108045 | 0.03087 | 0.1225 | 2 |

An integral bound of zero prevents accumulation even if Ki is edited. Integrator limits bound the integral state, not the whole PID output. Increasing Kp generally makes correction stronger; increasing Ki removes persistent error but can increase overshoot; Kd reacts to motion/noise. These trends are not a proof of stability for arbitrary tuning.

Altitude uses a vertical speed request to advance a height reference:

$$
z^{\mathrm{ref}}_k=z^{\mathrm{ref}}_{k-1}+h v^{\mathrm{request}}_{z,k},
\qquad e_{z,k}=z^{\mathrm{ref}}_k-\hat z_k.
$$

Its derivative term uses desired minus estimated vertical speed:

$$
u_{z,k}=K_p e_{z,k}+K_i I_{z,k}
+K_d\left(v^{\mathrm{request}}_{z,k}-\hat v_{z,k}\right).
$$

Thus centring the stick requests zero vertical speed and holds the current height reference. The C core also applies reference limits, tilt compensation, boost slew and mixer headroom. Editing the panel does not remove those limits.

## 4. Kalman settings: variances, gains and uncertainty

The angle estimator has one angle per axis plus its estimated gyro bias. Let $x=[\theta,b]^T$. Its prediction is:

$$
x_k^-=\begin{bmatrix}1&-h\\0&1\end{bmatrix}x_{k-1}
+\begin{bmatrix}h\\0\end{bmatrix}\omega_k.
$$

With gyro sample variance $q_g$ and bias random-walk density $q_b$:

$$
Q=\begin{bmatrix}q_g h^2+q_b h^3/3&-q_b h^2/2\\-q_b h^2/2&q_b h\end{bmatrix},
\qquad P_k^-=F P_{k-1}F^T+Q.
$$

The accelerometer produces an angle measurement. With $H=[1\;0]$ and angle measurement variance $R$:

$$
\nu_k=y_k-Hx_k^-,\quad S_k=H P_k^-H^T+R,\quad
K_k=P_k^-H^T/S_k,\quad x_k=x_k^-+K_k\nu_k.
$$

The panel defaults are $q_g=0.04\,\mathrm{(degree/s)^2}$, $q_b=0.0001\,\mathrm{(degree/s)^2/s}$, $R=4\,\mathrm{degree^2}$. These are **variances**, not standard deviations. The live graph reads the actual C Kalman gains and covariance, rather than recomputing a decorative curve.

The altitude estimator uses $x=[z,v_z]^T$:

$$
F=\begin{bmatrix}1&h\\0&1\end{bmatrix},\quad
B=\begin{bmatrix}h^2/2\\h\end{bmatrix},\quad
x_k^-=F x_{k-1}+B a_{z,k},\quad Q=\sigma_a^2 B B^T.
$$

Its settings are acceleration **standard deviation** $\sigma_a=1.5\,\mathrm{m/s^2}$ and barometer height **variance** $R_z=2\,\mathrm{m^2}$. Squaring a variance again is wrong; forgetting to square a standard deviation is also wrong. The barometer only corrects when a new sample arrives, approximately every 20 ms, whereas prediction runs every 2 ms.

For any plotted state component:

$$
\sigma_i=\sqrt{P_{ii}},\qquad
\text{predicted 1-sigma interval}=\hat x_i\pm\sigma_i.
$$

This interval depends on the assumed noise model. It is not an experimental confidence interval for an unidentified real drone. Compare truth-minus-estimate against this band in simulation, then repeat with independently measured reference data on hardware.

## 5. F450 geometry and editable plant parameters

With diagonal $d=0.45\,\mathrm m$, each motor has arm coordinates $\pm a$, where:

$$
a=\frac{d}{2\sqrt2}=0.15910\,\mathrm m.
$$

The applied fraction $f_i$ follows the motor demand $u_i$ with first-order lag:

$$
f_{i,k+1}=f_{i,k}+(u_{i,k}-f_{i,k})(1-e^{-h/\tau_m}),
\qquad T_i=T_{\max}f_i.
$$

For the FL, FR, RR, RL ordering used by this plant:

$$
T=T_1+T_2+T_3+T_4,\quad
\tau_x=a(T_1-T_2-T_3+T_4),\quad
\tau_y=a(-T_1-T_2+T_3+T_4),\quad
\tau_z=c_q(-T_1+T_2-T_3+T_4).
$$

Mass, inertia, thrust, drag and motor lag are assumptions until measured. At the default $m=1.2\,\mathrm{kg}$ and $T_{\max}=6\,\mathrm N$:

$$
f_{\mathrm{hover}}\approx\frac{mg}{4T_{\max}}
=\frac{1.2\times9.80665}{24}\approx0.4903.
$$

The FC nominal collective is 0.5. Editing mass changes the plant but does not silently retune the controller or its hover assumption. That mismatch is useful for robustness experiments. A large mismatch can prevent flight.

## 6. ROS2 navigation settings and the multi-node fix

The onboard navigator forms a target horizontal velocity from position error and then acceleration from velocity error:

$$
v^{\mathrm{request}}_{xy}=\operatorname{limit}(k_x(p_{\mathrm{path}}-p),v_{\max}),\quad
a^{\mathrm{request}}_{xy}=\operatorname{clip}(k_v(v^{\mathrm{request}}_{xy}-v),-a_{\max},a_{\max}).
$$

It rotates that acceleration into body coordinates and requests bounded roll/pitch angles. Defaults: position gain 1/s, velocity gain 1.8/s, maximum speed 0.8 m/s, acceleration limit 1.4 m/s², angle limit 15°, heading gain 1.2/s and yaw rate limit 45°/s.

The seven-node stall occurred when a later visual observation enlarged an obstacle clearance region around the current position. A* rejected its own start and the mission did not retry. The correction permits an outward-only path through the additional 0.65 m planning margin, provided every sampled edge stays outside the observed obstacle plus 0.34 m aircraft radius and does not reduce clearance. Escape speed is capped at 0.25 m/s. It never deletes an obstacle or changes the drone pose. Failed plans retry each simulation second and expose a reason such as `GOAL_INSIDE_CLEARANCE`. An unsafe requested goal should remain blocked; successful completion must not be forced.

## 7. Change one thing and keep the evidence

1. Keep a baseline run and its `settings.json`.
2. Change one gain/noise/model value. Start a fresh run.
3. Compare command vs response, overshoot, settling, motor saturation, uncertainty and obstacle clearance.
4. Inspect the original wire packets and mission log when authority matters.
5. Restore defaults if a change is worse. Preserve failed trials.

| Question | Source to inspect |
|---|---|
| Angle/rate/height control, estimators, calibration | `firmware/src/flight_controller.cpp` |
| Public FC ABI and UART protocol | `firmware/include/fc_api.h` |
| Route acceptance and intermediate-node transitions | `companion/ros2_ws/src/qcopter_autonomy/src/pilot_mission_node.cpp` |
| A*, observed obstacle clearance, velocity feedback | `companion/ros2_ws/src/qcopter_autonomy/src/navigation.cpp` |
| Takeoff, navigation, hover, descent and landed phases | `companion/ros2_ws/src/qcopter_autonomy/src/mission.cpp` |
| Dynamics, actuator force and truth telemetry | `simulation/gazebo/engine.py` |
| Synthetic sensors calling actual C control step | `simulation/gazebo/fc_binding.py` |
| Settings ranges, defaults and immutable snapshot inputs | `simulation/gazebo/experiment_settings.py` |

The more complete model derivation remains in [the model and integration guide](current-model-and-fc-integration.html). This workshop is an experimental interface, not an automatic gain calculator or proof that arbitrary values are stable.


## 8. Current braking and trigger mathematics

The canonical [illustrated F450 guide](current-model-and-fc-integration.html#16-why-centring-the-stick-requires-a-braking-loop) now derives release-position braking, its ideal poles and unavoidable stopping distance step by step. Chapters 17–18 cover Gentle shaping, RT/LT collective limits, zero-sum motor allocation, anti-windup and accelerometer correction gating.

Keep this page for editable settings, process authority and evidence capture. Use the math guide for derivations and the [launch guide](run-simulation.md) for startup and troubleshooting. Earlier versions are recorded in the [archive index](../archive/documentation/README.md).
