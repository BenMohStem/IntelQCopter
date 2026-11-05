# IntelQCopter v2 — deep engineering analysis and reproducible build contract

**Status: motor-disabled, source-complete, hardware-unverified. Do not install propellers or fly this build.**

Source of truth: the November 4 successful capture; F450 assumption-based mass/inertia source; Claude Volume I, Edition 2 (a mathematical study, not a verified aircraft description). Measured and assumed values below must never be conflated. The attached book calls the MAG an HMC5883L in some chapters; **the actual successful PCB device is QMC5883P, address 0x2C, ID 0x80**. Its source project called `100hz` actually runs QMC control register `0x3D` corresponding to nominal 200 Hz, approximately 195 Hz measured. Preserve that exact successful configuration until hardware changes are validated.

## What v2 implements and what it does not

- **500 Hz software clock:** 2,000 µs absolute deadlines; measured source-loop time, IMU/MAG/BMP I²C costs, misses and freshness counters. Running the code fast does not imply the ESC control path is safe or tuned.
- **MPU6050:** 0x68 (or 0x69), GPIO9 SDA / GPIO8 SCL, MPU AUX bypass for BMP on XDA/XCL, DLPF_CFG=1 (gyro 188 Hz bandwidth/delay 1.9 ms; accelerometer 184 Hz/delay 2.0 ms), SMPLRT_DIV=1 (500 Hz), gyro ±250 dps / accel ±2 g *as recorded*; accelerometer + three gyro axes in 12-byte hardware FIFO frames; overflow detection/reset, backlog counters, 400 kHz primary bus. No external INT required. DO NOT silently raise ±250 dps when the saved calibration constants were derived at that range; change and verify all scaling together.
- **At boot:** write factory-relative gyro correction to MPU6050 offset registers 0x13–0x18, read back, then measure **post-write residual gyro bias** over 600 stationary measurements, rejecting motion. Write coarse accelerometer correction to 0x06–0x0B, preserve each temperature-compensation LSB and check readback. The software matrix bias is adjusted by the *theoretical register-induced delta* so bias is **not counted twice**. This is an unverified first-pass assumption about the offset scaling on the mounted chip: validate using before/after stationary observations and factory trim.
- **QMC5883P:** register 0x0A=0x3D, control2=0x08, sign=0x06, internal DRDY status register 0x09 (no DRDY pin). Nominal 200 Hz; poll 250 Hz for fresh data. The fast-mode (400 kHz) attempt downgrades **only QMC transactions** to 100 kHz after one confirmed fault; MPU and BMP still use 400 kHz. This is an explicit exception to the desired all-400-kHz configuration because your prior tests found 400 kHz QMC failure. The working driver otherwise remains intact.
- **BMP280:** cached factory trim, one burst of raw temperature/pressure registers and Bosch integer compensation, ×1 pressure, ×1 temperature, IIR4, 0.5 ms standby, NORMAL mode. Bosch's own table gives **166.7 Hz typical**, not 200 Hz. The main loop checks every 6 ms; identical raw registers are not called a new conversion. A larger IIR factor reduces noise at the cost of dynamic lag and is not automatically better for altitude control.
- **TOFSense F2 Mini:** optional 16-byte Nooploop Frame0 parser with checksum, range status, precision and signal strength. The LiDAR can reach **50 Hz**, not 200 Hz. UART RX pin remains intentionally unassigned and `IQ_LIDAR_ENABLE=0` until verified against physical wiring. ESP32 LiDAR UART, if enabled, is distinct from the CH343 UART used for the Xbox bench bridge. LiDAR readings are tilt-compensated and fused only if valid.
- **Quaternions:** propagate the gyro each 2 ms using body-to-world unit quaternions and gravity correction; slow magnetic yaw correction only when the 3×3-corrected magnetometer passes magnitude/age gates. Sensor-to-body rotations are **identity placeholders**. An incorrect axis mapping is enough to cause runaway motor correction: verify it on the real PCB. Quaternion propagation is nonlinear kinematics; the controller is still cascaded linear PID, not EKF or LQR.
- **Vertical linear Kalman:** state `x=[height, vertical_velocity, vertical_acceleration_bias]`; conditional linear prediction from quaternion-rotated accelerometer; sequential barometer and optional LiDAR measurement updates with Joseph covariance update, NIS innovation gating, process uncertainty and stale-data rejection. Without LiDAR/GNSS, slow absolute barometric drift cannot be eliminated, and vertical acceleration bias is difficult to distinguish from height drift. This is **not** an absolute altitude/position estimator.
- **Seven cascaded PIDs:** roll and pitch angle; roll, pitch and yaw rate; relative height and vertical speed. All PID sets contain nonzero **Kp, Ki, Kd**; derivative-on-measurement low-pass filtering; integral limits and conditional anti-windup. The rate gains output torque (N m), not PWM/ESC counts. Mapped to an idealized X virtual mixer only, with model-aware hover feedforward and a cos(tilt) correction. **No physical motor outputs**.
- **Xbox:** Windows/gamepad->CH343 921600 baud `$IQ2` CRC-framed bench bridge; left X yaw rate, left Y vertical speed, right X/Y roll/pitch angles; D-pad Up + LB + RB re-zeroes BMP altitude only in the disarmed build. Watchdog 200 ms. No tested direct BLE link to the ESP32-S3, no RF redundancy, no independent kill switch and no motor arming state machine; it is not a flight receiver.

## Calibration actually applied

Gyro stationary bias (deg/s): `(+0.161968974, -0.304950151, -0.877433074)`. Stationary SD (deg/s): `(+0.047727251, +0.044855293, +0.043199902)`. With hardware offsets, subtract **only the new post-write residual**. A later temperature shift requires a fresh stationary check or a measured temperature-bias model; no invented drift coefficient is installed.

Accelerometer bias in m/s²: `(1.264655572, -0.059103429, -0.678744839)`.

`C_a` (true sensor-frame calibrated reading `a_cal = C_a * (a_after_hw - b_after_hw)`):

```
[ +0.999284313  -0.038750762  -0.032967877 ]
[ +0.041351387  +0.992793975  +0.040449121 ]
[ +0.042481183  -0.040866969  +0.987204883 ]
```

Magnetometer sensor-frame hard-iron bias in µT `(-29.482301006, -18.397983738, -35.474272846)`.

`C_m` (relative soft iron corrected `m_cal = C_m * (m_uT - b_m)`):

```
[ +0.973366985  +0.005036971  +0.038277934 ]
[ +0.005036971  +0.964534610  -0.017911741 ]
[ +0.038277934  -0.017911741  +1.067011358 ]
```

The matrix is a **candidate**: magnetometer norms fit the calibration rotations, but the board-to-body rotation and hard/soft iron disturbance when ESCs run still need checks. Do not import the old GUI's XY reflection with det=-1 as a sensor-to-body *rotation*.

BMP280 prior stationary pressure SD `0.640 Pa`; relative-altitude SD `0.0548 m` at rest. In the flight Kalman we deliberately use **0.22 m** barometer observation SD as a provisional more conservative noise model (unmeasured propwash, dynamic pressure, thermal drift). The filter innovation NIS and rejection counters MUST be inspected during prop-off and prop-on tests before reducing this value.

## Core models, units and full uncertainty

FLU body frame `+X forward, +Y left, +Z up`. Motor geometry and actual motor rotation sequence must be physically validated; virtual mixer only.

`J * omega_dot = torque - omega × (J*omega)`. We use **linear PID** for commanded torque; this v2 build does NOT enable the nonlinear gyroscopic feedforward term (the rigid-body coupling is part of the plant). The inertia tensor is nominally diagonal but the uncertainty on off-diagonal elements is included in the JSON (component-location uncertainty). All inertia entries are kg·m².

F450 **basic** assumption prior `m_A = 1.104 ± 0.05055 kg`, `J_A ≈ diag(0.0097810,0.0103899,0.0191702) kg m²`, `COM ≈ [0,0,+0.018658] m` relative to the lower PCB origin (FLU positive-Z is UP; confirm geometric convention before copying transforms). Diagonal 1σ `(0.00044574,0.00045353,0.00083141)`.

F450 **with stripped N150** assumption prior `m_B = 1.319 ± 0.08284 kg`, `J_B ≈ diag(0.0103001,0.0109127,0.0195616)`; `COM≈[0,0,+0.011850] m` (under source assumptions). Diagonal 1σ `(0.00048956,0.00049798,0.00083780)`.

These do NOT come from weighing the actual craft. The Monte Carlo samples independent diagonal inertias and mass using the prior SD, plus assumed `max_thrust_per_motor=6 ±1.5 N`, `lag=0.040 ±0.015 s`, `motor_diagonal=0.450±0.010 m`, `yaw_torque_per_thrust=0.015±0.00375 m`. These rotor parameters are unmeasured; the 95% conditional intervals in `data/model_uncertainty_PID_v2.json` are **not measured physical confidence intervals**. The source geometry model is in `data/F450_assumption_based_inertia.json`.

The X mixer under idealized normalized thrust `T_i = u_i*T_max` has roll/pitch lever `a = diagonal/(2*sqrt(2))`; yaw gain `κ=TQ/T` is entirely assumed. Hover fraction `u_h = m*g/(4*T_max)`. Normal historical command cap 45% is near or below the basic aircraft's assumed hover command; it is unsafe to infer sufficient thrust margin from controller gains. Actual T_max, hover curve and ESC time constant need bench identification.

## Model-derived rate PID, including Kd

Near hover, `J_i * d(omega_i)/dt = tau_i`. Torque output `tau_i` in **N·m**. A PI design for a double-integrator-in-loop rate dynamic with target second-order polynomial yields:

```
Kp_rate_i = 2*zeta*omega_n*J_ii
Ki_rate_i = omega_n^2*J_ii
Kd_rate_i = (0.006 seconds)*J_ii
```

The last coefficient is intentionally small and is used **only** with a filtered derivative on the measured angular rate. The exact derivative's filter changes pole locations, so no analytical stability result alone certifies this controller. The default low-bandwidth design uses `(omega_n,zeta)=(2 rad/s,2)` for roll/pitch and `(1.2 rad/s,1.9)` for yaw. This design is conservative because the motor lag is assumed 40 ms, the thrust mapping is not measured and the sample stream is unverified under motor EMI.

The ideal 3rd-order characteristic with first-order motor lag (for *unfiltered* PID) is:

`(tau_m J) s^3 + (J+Kd) s^2 + Kp s + Ki = 0`.

A necessary sufficient Routh-Hurwitz condition for positive coefficients is `(J+Kd)*Kp > tau_m*J*Ki`. The Monte Carlo computes the fraction of samples meeting this **idealized** condition; real discrete derivative filters, saturation, transport delays and nonlinear motor response require real tests.

Outer loops are intentionally slower. Roll/pitch angle `Kp=.65, Ki=.020, Kd=.030` (error in rad, output rate in rad/s). Yaw is a **rate** request only; no yaw-angle PID. Relative height `Kp=.40,Ki=.005,Kd=.015` (error in m, output m/s), and vertical speed `Kp=1.00, Ki=.060, Kd=.025` (error in m/s, output m/s²). These are provisional **design selections**, not fitted from mass/inertia: rate gains depend on J, while altitude-to-acceleration gains are normalized by mass at the final thrust feedforward `T = m(g+a*)/cos(tilt)`.

The dynamic derivative coefficient is filtered at 32 Hz for rate, 15 Hz for angle, 6/8 Hz for height/velocity. Repeated measurements must NOT be described as increasing estimator precision without correcting for temporal correlation.

## Altitude linear state-space Kalman filter

State `x=[z,v_z,b_a]^T`, acceleration input `u = (R_WB(q)*f_B)_z - g` in m/s². Within one time step `dt`:

```
F = [1 dt -0.5*dt^2]
    [0  1       -dt  ]
    [0  0        1   ]
B = [0.5*dt^2, dt, 0]^T
x[k+1] = F*x[k] + B*u
P[k+1] = F*P[k]*F^T + B*sigma_a^2*B^T + Q_bias
```

Barometer measurement `y_b = z + noise`, `H_b=[1 0 0]`, with P0 boot-averaged: `z_b = 44330*(1-(P/P0)^0.190294958)`. LiDAR relative height (if valid, only close to level and ground-referenced) `z_L = range*cos(tilt) - range_ground_reference`, with the same H and a lower measurement SD only if reported range quality is valid. NIS gates reject >5σ baro and >4σ LiDAR innovations. Joseph covariance update prevents numerical loss of symmetry.

**Unobservable quantities / constraints:** A single barometer cannot estimate absolute altitude or its slow offset relative to world; an accelerometer's gravity-compensated vertical acceleration is highly sensitive to pitch/roll misalignment and vibration; integrating it twice drifts. Without actual LiDAR calibration, a barometer-only hover is not guaranteed centimeter-accurate. No XY position/velocity source is fitted, so no horizontal position hold is claimed. With no GPS, heading reference is arbitrary and magnetometer can only limit relative drift, subject to alignment and magnetic disturbance.

## Speed budget and bus arbitration

The IMU FIFO samples 12 bytes at nominal 500 Hz. Reading FIFO count and a 12-byte sample at 400 kHz is nominally a few hundred microseconds on the physical I²C wires, **not** a guaranteed end-to-end latency: actual time includes repeated-start behavior, clock stretching, mutex/driver overhead and missed conversions. The code prints **measured maxima** of each sensor operation and 1-second loop periods in `FC_TIMING` lines. The FIFO returns the newest buffered 12-byte packet, counts discarded frames, and resets if misaligned or nearly full instead of executing an arbitrarily long catch-up loop.

The QMC remains on the user's historically validated 0x3D 200 Hz ODR with internal status gating. It tries 400 kHz but downgrades permanently to 100 kHz after a fast-mode fault. `Wire.setTimeOut(5)` bounds a fault but one timeout can still miss a 2 ms control deadline; a single long blocked transaction is NEVER acceptable as proof of 500 Hz real-time scheduling. The reported `FC_TIMING`/FIFO counts determine whether bus separation or a faster MPU SPI-capable IMU is warranted in the next hardware revision.

For BMP, Bosch documents ×1/×1 **166.7 Hz typical** with 0.5 ms standby (maximum conversion timing may lower this). IIR x4 is a compromise: x16 would reduce noise but introduce substantial vertical-state measurement delay. The flight controller does **not** pretend a 200 Hz host poll produces 200 independent pressure samples.

## Flight safety and validation requirements (NO MOTOR OUTPUTS)

1. **Static orientation:** Hold the assembled drone level, nose-down and right-down; verify IMU X/Y/Z and the sign of world-up gravity. Confirm each sensor-to-body matrix is a real 3D rotation with det +1. Confirm QMC gyro yaw direction and heading signs while disarmed.
2. **Boot offsets:** Compare pre/post gyro and accel hardware offset registers, stationary means, temperature, matrix-corrected gravity and range; if the hardware register effect differs from the theoretical factor, disable hardware accel offsets and retain software 3×3 until empirically corrected.
3. **Timing:** Record at least 10 minutes of 500 Hz while deliberately moving sensors; require predictable FIFO frame count, near-zero drops, bounded I²C errors, and no large control-deadline misses. Investigate frequency spikes during 1 Hz diagnostics.
4. **Altitude:** Verify zero at ground, known 0.5 m / 1 m / 2 m elevations (independent ruler). Evaluate drift over 10 minutes, near draft/ventilation, warm-up, and later propwash. Do not reduce R until dynamic residual statistics justify it. LiDAR wiring must be measured before setting the RX pin.
5. **Controller step tests:** Apply small virtual-angle/yaw/vertical input commands with Xbox. Verify proper yaw, pitch and roll signs and no integral windup, with no PWM connected to motors.
6. **Propulsion system identification:** Measure each motor thrust-vs-command curve with the actual propeller and LiPo under restraints, ESC responsiveness, idle/max/hover thrust, battery sag and motor RPM. Weigh the complete actual airframe and measure main mass placements; update J and CoM.
7. **Safety architecture:** Redundant/independent motor disarm, link-loss timeout with tested behavior, brownout detection, arming gates, motor ordering/direction, ESC safe startup, independent battery monitoring, physical prop-off bench validation. None of these exist as a verified actuating path here.

## Research and primary sources used

1. [InvenSense MPU-6000/MPU-6050 Register Map Rev 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf): sample-rate divisor, DLPF frequency/delay, FIFO registers and I2C bypass.
2. [Bosch BMP280 Datasheet Rev 1.26](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf): Table 13 conversion timings, Table 14 normal ODR, IIR response, pressure integer compensation.
3. [QST QMC5883P Rev E](https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf), with [Betaflight QMC5883P driver](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/compass/compass_qmc5883.c) for independent register comparison.
4. [Nooploop TOFSense-F2 Mini Datasheet](https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf) and [Nooploop Frame0 parser](https://github.com/nooploop-dev/nlink_unpack/blob/master/nlink_tofsense_frame0.c): 50Hz LiDAR, 921600 UART default, 16-byte protocol.
5. Mahony, Hamel, Pflimlin (2008), [Non-linear complementary filters on SO(3)](https://openresearch-repository.anu.edu.au/items/db91f02d-9a17-4ead-bc64-86ed5bc9c6f4): geometric attitude observers with bias handling.
6. Sabatini et al. (2014), [A Sensor Fusion Method for Tracking Vertical Velocity and Height Based on Inertial and Barometric Altimeter Measurements](https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/): barometer/IMU fusion, vertical velocity sensitivity and drift.
7. Wu et al. (2025), [Indoor multi-sensor UAV altitude fusion and fault-tolerant adaptive Kalman filtering](https://doi.org/10.1088/1742-6596/3133/1/012015): barometer bias and LiDAR innovation handling.
8. [Cascade altitude and attitude PID control research (2017 ICUAS)](https://ieeexplore.ieee.org/abstract/document/7991400): cascade PID with linear Kalman relative-height estimation.
9. [Bluepad32 ESP32-S3 hardware/firmware FAQ](https://bluepad32.readthedocs.io/en/latest/FAQ/): ESP32-S3 BLE only; Xbox Wireless Controller firmware 5.x BLE support, firmware 3.x/4.x BR/EDR incompatible with S3.
10. **User-supplied Claude Volume I Edition 2** in `docs/references/`: insightful mechanics/probability/control derivation, but HMC magnetometer and assumed plant values are **not the measured QMC5883P board**.

## Reproduce tests

```powershell
py -3 -m pip install numpy pyserial pygame
py -3 tests/test_project.py
py -3 tools/uncertainty_pid_v2.py
py -3 tools/simulate_rate_v2.py
.\tools\run_fc.ps1 -UploadFirmware
.\tools\run_fc.ps1
py -3 tools\xbox_uart_bridge.py --inspect
.\tools\run_fc.ps1 -Xbox
```

The ESP32-S3 hardware must be powered properly and tested with all propellers removed and ESC signal leads disconnected. The bundled GCC mock test is a syntax check and cannot verify timing/electrical behavior. `tests/test_core.cpp` checks pure numerical routines. The complete **ESP32 PlatformIO build still must be run on the Windows PC**; it is not claimed to have been completed here.
