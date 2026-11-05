# All seven PID loop parameters, units and uncertainty — v2

**Do not transfer these directly to ESC PWM.** Only the rate gains are propagated from the *assumed* inertia distribution. Other uncertainties are design ranges, not confidence intervals from flight testing. All outputs are for virtual-motor bench simulation.

## Default basic-F450 profile A

| Loop | Kp ± prior SD | Ki ± prior SD | Kd ± prior SD | Units / interpretation |
|---|---:|---:|---:|---|
| Roll angular rate | 0.078253646 ± 0.00356 | 0.039126823 ± 0.00178 | 5.8690234e-05 ± 2.67e-06 | Nm vs rad/s error |
| Pitch angular rate | 0.083149939 ± 0.00362 | 0.04157497 ± 0.00181 | 6.2362454e-05 ± 2.72e-06 | Nm vs rad/s error |
| Yaw angular rate | 0.087423709 ± 0.00378 | 0.027607487 ± 0.00119 | 0.0001150312 ± 4.97e-06 | Nm vs rad/s error |
| Roll angle | 0.65 ± 0.15 | 0.02 ± 0.008 | 0.03 ± 0.01 | rad/s setpoint |
| Pitch angle | 0.65 ± 0.15 | 0.02 ± 0.008 | 0.03 ± 0.01 | rad/s setpoint |
| Relative altitude | 0.4 ± 0.15 | 0.005 ± 0.003 | 0.015 ± 0.007 | m/s setpoint |
| Vertical speed | 1 ± 0.3 | 0.06 ± 0.02 | 0.025 ± 0.01 | m/s² setpoint |

## Assumption-based physical model

| Property | A basic F450 | B hypothetical N150 |
|---|---:|---:|
| Mass kg | 1.10385 ± 0.0505 | 1.31924 ± 0.0828 |
| Jxx kg m² | 0.00978171 ± 0.000445 | 0.0103009 ± 0.00049 |
| Jyy kg m² | 0.0103937 ± 0.000453 | 0.0109127 ± 0.0005 |
| Jzz kg m² | 0.0191719 ± 0.000829 | 0.0195622 ± 0.000836 |
| Idealized hover normalized motor fraction | 0.486248 ± 0.155 | 0.580362 ± 0.186 |

## Interpretation

- Model priors are synthetic from assumed part masses/locations and unmeasured motor response, **not physical confidence intervals**.
- High-precision gain decimals arise from arithmetic, not real motor identification. Do not treat them as flight-tuned.
- PID rate gains command SI torque (Nm), angle controller outputs rad/s setpoints, vertical speed PID commands m/s², altitude PID commands m/s.
- The 45% historical normalized motor cap may not leave enough hover margin. Actual ESC command need not be proportional to thrust.
- The synthetic step-response results are in `data/rate_step_synthetic_v2.json` and the complete model and design is `docs/DEEP_ENGINEERING_ANALYSIS_v2.md`.
