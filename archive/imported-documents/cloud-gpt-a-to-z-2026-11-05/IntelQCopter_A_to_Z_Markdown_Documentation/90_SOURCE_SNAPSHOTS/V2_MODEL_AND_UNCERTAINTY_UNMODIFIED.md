# Dynamics, calibration and uncertainty — IntelQCopter

## 1. Reference frames and physical calibration

Choose a consistent **right-handed** body frame with +X forward, +Y left, +Z up (FLU) and world frame z up. Let sensor measurements be in sensor frame S. The mapped, calibrated MPU and QMC vectors are

```
a_B = R_BS_imu  C_a (a_S_counts*g/16384 - b_a)
w_B = R_BS_imu (w_S_counts/131 - gyro_post_hardware_residual_dps) * pi/180
m_B = R_BS_mag C_m (m_S_counts/37.5 - b_m)
```

All three matrices operate on raw **chip-axis** data; `R_BS_imu` and `R_BS_mag` must be valid rotations (R.T R=I, det R=+1). 3x3 calibration `C_a` includes cross-axis correction, but does NOT determine the mounting orientation `R_BS`.

- `b_a=(+1.264655572, -0.059103429, -0.678744839)` m/s², fitted across six manually positioned faces.
- `b_m=(-29.482301006, -18.397983738, -35.474272846)` µT, fitted on 14,000 accepted magnetometer samples.
- `gyro_original_bias=(+0.161968974, -0.304950151, -0.877433074)` °/s, measured on 30,000 stationary readings.
- Gyro stationary within-run standard deviations `(0.047727,0.044855,0.043200)` °/s; not the same thing as temperature/run-to-run bias uncertainty.
- BMP pressure stationary SD `0.640 Pa`, corresponding to roughly `0.0548 m` relative altitude SD, one short session.
- Correction matrices and exact constants are in `include/flight_core.hpp` and `data/20261104_raw_calibration_candidate_UNVERIFIED.json`. Accel face placement and magnetic heading alignment remain unverified.

The gyroscope has **hardware offset registers**, 0x13..0x18, whose ±1000-dps calibration scale is 32.8 LSB/dps. The project adds `-round(b_dps*32.8)` to the existing register offsets at boot, verifies readbacks, then estimates fresh post-register residual bias from stationary data. This is not the same as writing an arbitrary 3x3 calibration matrix into a sensor register.

## 2. Rigid-body plant

Let `R_WB` map body to world, body angular velocity `omega=(p,q,r)` rad/s, `r` center-of-mass location. In an ideal rigid-body model:

```
m * r_ddot = R_WB * [0,0,sum_i T_i] - [0,0,m*g] + F_disturbance
J * omega_dot = tau - omega x (J*omega)
R_dot = R * skew(omega)
```

For an X configuration, rotor centers are approximately `(±a,±a,0)`, `a=D/(2sqrt(2))`, rotor thrust `T_i=k_T*Omega_i²`, reaction moment magnitude `Q_i=k_Q*Omega_i²`. True thrust and drag coefficients, motor spin directions, and motor time constants have **not been measured**. The code's mixer is therefore virtual normalized thrust fractions, not ESC PWM commands. `J` includes diagonals and products of inertia; nominal off-diagonals are near zero by the symmetric **assumed geometry**, NOT because they were experimentally established zero.

The motor plant approximation `tau_m*dT_i/dt + T_i = T_requested_i` assumes tau_m=0.040s; this is a reference-study guess. Real ESC throttle→thrust has nonlinearities, battery-voltage dependence and saturation.

## 3. Two alternative, assumption-based mass models

| Parameter | A: basic F450 | B: assumed stripped N150 |
|---|---:|---:|
| Mass m | 1.104 kg | 1.319 kg |
| Mass 1-sigma input | 0.05055 kg | 0.08284 kg |
| Jxx | 0.0097810 kg m² | 0.0103001 kg m² |
| Jyy | 0.0103899 kg m² | 0.0109127 kg m² |
| Jzz | 0.0191702 kg m² | 0.0195616 kg m² |
| SD(Jxx) | 0.00044574 kg m² | 0.00048956 kg m² |
| SD(Jyy) | 0.00045353 kg m² | 0.00049798 kg m² |
| SD(Jzz) | 0.00083141 kg m² | 0.00083780 kg m² |

Both models are conditional on uncertain masses and geometry. They are NOT measured CAD inertias. The Claude Volume I 1.2 kg mass and the older 1.902 kg N150 figure are **different assumptions**; verify the stripped computer's physical weight and orientation before choosing a profile. `platformio.ini` uses A by default; set `-DIQ_AIRFRAME_PROFILE=1` for provisional B only.

## 4. Controller design and uncertainty

With linearized hover thrust `T_i=Tmax*u_i`, max thrust **assumed** 6.0 N/motor with arbitrarily chosen ±1.5 N design uncertainty, 0.45 m **assumed** motor diagonal ±0.01 m, reaction-torque ratio `kappa=0.015 m` ±25%:

```
b_roll  = 4*a*Tmax/Jxx         b_pitch = 4*a*Tmax/Jyy
b_yaw   = 4*kappa*Tmax/Jzz
Kp_rate = (2*zeta*omega_n) / b
Ki_rate = omega_n^2 / b
```

Using `zeta=.9`, `omega_n_roll/pitch=8.4 rad/s`, `omega_n_yaw=2.8 rad/s`, the **book reference** basic-frame normalized-thrust-fraction PI gains, for rate errors in rad/s, are:

```
                 roll          pitch          yaw
Kp           0.038731       0.041142       0.268383
Ki           0.180745       0.191996       0.417485
```

The FC uses a **slower virtual preview** design with `zeta=1.7`, `omega_n_roll/pitch=3.0 rad/s`, `omega_n_yaw=1.4 rad/s`. For basic-frame A this changes the virtual rate gains to approximately:

```
             roll       pitch       yaw
Kp           0.026128   0.027755    0.253473
Ki           0.023054   0.024489    0.104370
```

The original book-inspired gains produce **~35.1%** overshoot in the illustrative 40ms motor-lag 20deg/s rate step. The slower roll-gain example produces **~9.77%** overshoot under the identical simplified plant, but is much slower and not experimentally justified. See `tools/simulate_rate_step.py` and `data/simulated_rate_PI_summary.json`. These results are synthetic and do not constitute real PID tuning.

The dynamics include motor-lag factor `(1+tau_m*s)` and **only a preliminary continuous-time** stability check `Kp > tau_m*Ki`; it does not establish phase margins with actual MCU timing, sensors and ESCs. The current 500 Hz loop performs a quaternion gyro/accel complementary estimate, angle P=3 s^-1, then rate PI with integrator limiting and reset on virtual saturation. `omega x Jomega` is included, but a real motor dynamic inverse has NOT been identified. Magnetic yaw fusion and altitude feedback are deliberately disabled until body-axis and measurement validations.

First-order uncertainty propagation under **independence**:

```
(sigma_b/b)^2 ~= (sigma_a/a)^2 + (sigma_T/T)^2 + (sigma_J/J)^2
sigma_Kp/Kp ~= sigma_b/b ; sigma_Ki/Ki ~= sigma_b/b
```

A **separate** 100,000-draw Monte Carlo, conditional on independent Gaussian priors and motor thrust truncated to 2..10 N (not measured confidence intervals), is in `data/model_uncertainty_results.json`. It shows broad plausible variation in gains and hover thrust. The uncertainties do **not** validate the assumed models. Thrust is especially poorly known.

## 5. Hover, battery and saturation checks

The 45%-normal/65%-boost limits came from an older repository controller; they are **not a calibrated ESC thrust curve**. If fraction directly represented thrust, `u_hover=m*g/(4*Tmax)`:

```
A basic: 0.451106
B N150:  0.53996 (approx.)
```

Thus A already slightly exceeds the 45% historical cap and B exceeds it substantially under this highly simplified model. If the cap applies to PWM duty or pulse width rather than **thrust fraction**, the comparison becomes even less straightforward. Neither model demonstrates usable hover authority with reserve for roll and pitch corrections. Actual thrust curves, battery voltage sag, frame weight, propellers and ESC protocol must be measured before any flight-tuning gains are meaningful.

## 6. Error model and handling

- **Measurement faults:** I2C request failure, invalid 6-axis packet, stale QMC sample, BMP pressure outside 10..120 kPa, out-of-range accelerometer norm, quaternion NaN. Faults set bit flags; IMU loss stops integration and virtual motor commands.
- **Timing faults:** 2ms tick deadline misses and observed dt outside 0.5..6ms are logged. On time fault, gyro estimation step and rate integral are not advanced.
- **Pilot-link faults:** CRC16-invalid packets are ignored. Last valid input expires at 200ms, causing virtual motor outputs zero and integrator reset. A checked USB serial frame is **not authenticated or hardened radio control**.
- **Calibration faults:** gyro boot check rejects moving/unsteady startup; invalid BMP zeroing leaves altitude undefined. No false success values should be substituted.
- **Model faults:** mass, thrust and J assumptions carry explicit uncertainty; `Model::clearedForFlight()` is hard-false until the real powertrain has been identified.
- **Missing protection:** genuine ESC output watchdog, receiver BLE link health, independent kill switch, low-voltage cutoff, LiDAR driver/calibration, true heading, and flight-tested estimator are NOT implemented in this release.

## 7. Evidence provenance

**Direct measurement:** November 4 capture with 98,868 recorded packets, 30k gyro, 30k accel, 14k accepted mag and 1k BMP calibration samples; same uploaded firmware configured `IQ_MAG_CONTROL1=0x3D`. This supports **sensor acquisition and numeric correction**, not flight.

**Reference math:** User-supplied Claude *IntelQCopter Volume I, Edition 2*, equations for rigid-body dynamics, mass uncertainty, rate PI pole placement and fault states. The PDF's HMC5883L model is **not the actual physical magnetometer** (QMC5883P). Chapter 18 PID uses 1.2kg/Jx=.012/Tmax=6N all **ASSUMED**; this project computes candidate gains from the separate F450 assumed model instead.

**Engineering inference:** chosen estimator gain, sample-age thresholds, controller latency assumptions, bandwidth and pilot-link protocol. All must be tested, tuned and documented with physical hardware.
