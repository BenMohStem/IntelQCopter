# All 21 PID parameters — numerical table and units

**Source:** v2 `data/PID_TABLE_v2.md`; the rate gains are inertia-dependent candidates, the other gains are chosen design candidates. They are not flight-identified.

| Loop | Kp | Ki | Kd | Source of ±SD | Output quantity |
|---|---:|---:|---:|---|---|
| Roll angle | 0.650000 | 0.020000 | 0.030000 | design intervals: 0.15, 0.008, 0.010 | angular rate rad/s |
| Pitch angle | 0.650000 | 0.020000 | 0.030000 | same as roll | angular rate rad/s |
| Roll rate | 0.078253646 | 0.039126823 | 0.000058690 | 0.00356 / 0.00178 / 2.67e-6 | torque N·m |
| Pitch rate | 0.083149939 | 0.041574970 | 0.000062362 | 0.00362 / 0.00181 / 2.72e-6 | torque N·m |
| Yaw rate | 0.087423709 | 0.027607487 | 0.000115031 | 0.00378 / 0.00119 / 4.97e-6 | torque N·m |
| Relative height | 0.400000 | 0.005000 | 0.015000 | design intervals: 0.15 / 0.003 / 0.007 | vertical speed m/s |
| Vertical speed | 1.000000 | 0.060000 | 0.025000 | design intervals: 0.30 / 0.020 / 0.010 | acceleration m/s² |

## Read the units first

The PID implementation integrates `error*dt` in SI time and filters derivative of measurement. Thus gain dimensions differ between angle, rate and vertical loops. An arbitrary number copied into an ESC PWM library would be wrong by dimensional analysis. The modeled torque requests need a calibrated thrust/moment allocator before any physical actuation.

## Statistical limits

Rate gain 1σ values are propagated from **assumed** J uncertainty (nominal J scales gains linearly). Other ±SD ranges are design ranges and do not represent frequentist confidence intervals. Torque authority, motor lag and saturation alter the physically optimal gains; their true distributions have not been measured.
