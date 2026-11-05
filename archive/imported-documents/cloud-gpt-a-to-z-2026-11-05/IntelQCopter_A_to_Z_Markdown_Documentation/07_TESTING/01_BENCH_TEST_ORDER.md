# Hardware test sequence — safe and reproducible

This plan establishes evidence stepwise. **It does not authorize installation of propellers or a free-flight attempt.**

## Phase 0: physical safety

Disconnect ESC signal wires, remove propellers, verify power rails, I²C pull-ups, UART ground and sensors' orientation. Retain motor-off compile-time guard. Only one serial reader/flash process at a time.

## Phase 1: golden sensor regression

Run preserved 4 Nov calibration source as a baseline: MPU valid, QMC fresh at profile `0x3D`, BMP valid, UART packet losses/CRC minimal. This gives a reference when new FIFO/400k versions fail. Record actual power state.

## Phase 2: FC v2 boot

Build `iq-fc-safe` with UART 921600 and Wi-Fi 0. Confirm MPU offset register readbacks, gyro post-write residual, accelerometer norm, MAG ID/status, pressure boot P0 and FIFO enabled. Confirm source boot prints `ESC=OFF`.

## Phase 3: 500 Hz timing

Collect 10 minutes of `FC` and `FC_TIMING` without blocking stdout. Record real `loop_max_us`, missed periods, FIFO losses/overflow, QMC fallback and fresh rates under board motion. Compare 100k/400k only under controlled changes.

## Phase 4: estimator hand tests

Hold level; pitch, roll, yaw in known directions; test quaternion signs and MAG behavior. Raise/lower board against ruler; evaluate baro altitude drift and KF innovations. Do not trust output signs from the GUI alone.

## Phase 5: virtual control and Xbox

Test gamepad axis dead zones/scale, D-pad Up+LB+RB re-zero disarmed, 200 ms link-loss, valid CRC and sequence rejection, virtual torque response and anti-windup. ESC GPIO remains inactive.

## Phase 6: physical propulsion feasibility (future, separate)

Weigh final vehicle, identify motors/props/ESC mapping, measure thrust with guarded independent test fixture, implement/test arming and kill path before contemplating controlled propulsion. Recompute gains based on measured actuator model.
