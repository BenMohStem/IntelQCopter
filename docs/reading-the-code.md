# Reading the controller code

This guide follows the code in execution order. The flight controller is a **non-flying scaffold**: it computes estimator/PID/mixer math in a host test, but the RP2350 program has no sensor readers and never sends values to ESCs. The RP2040 hand controller is only a boot-message placeholder.

## Flight controller ([`flight_controller.c`](../flight_controller/flight_controller.c))

Open the linked source and follow these sections from top to bottom.

1. **Headers and loop constants (lines 1–24).** `math.h` supplies angle functions and `isfinite`; Pico headers are omitted for the host test. `FC_LOOP_HZ` sets the *intended* rate to 500 Hz (2 ms per step); it does not schedule a real-time loop yet. `FC_MAX_CONTROL_DT` rejects time gaps over 10 ms.
2. **State and input records (lines 26–86).** `pid_t` stores PID tuning and history. `angle_kf_t` holds roll/pitch angle and gyro-bias estimates plus a 2×2 covariance. `altitude_kf_t` holds height and vertical speed plus covariance. `pilot_command_t` and `sensor_sample_t` define the data that future radio and sensor drivers must supply. `control_output_t` contains four normalized *numbers*, not electrical ESC signals.
3. **Estimator and PID starting values (lines 89–128).** These are provisional constants for software exercising. PID gains are not flight-tuned; filter noise values are assumptions. The two angle filters are roll and pitch; yaw is intentionally rate-only.
4. **Small PID helpers (lines 130–165).** `clampf` bounds a value. `pid_reset` clears controller history. `pid_update` computes setpoint error, derives on the measurement to avoid a setpoint kick, limits the integrator/output, and conditionally prevents integrator windup. Invalid or excessive `dt` resets the PID and returns zero.
5. **Roll/pitch Kalman update (lines 167–194).** First it predicts angle from gyro rate after subtracting estimated gyro bias. It predicts covariance, then compares the accelerometer angle with the prediction (`innovation`). Kalman gains blend the two estimates and update their covariance. `p00` is angle variance, so `sqrt(p00)` is the predicted angle standard deviation.
6. **Altitude Kalman prediction/update (lines 197–227).** The state is height and vertical speed. `altitude_kf_predict` advances both using gravity-compensated vertical acceleration and grows covariance. `altitude_kf_baro_update` corrects height and speed using a barometer height sample. These functions are not yet validated against a vehicle/sensor simulation.
7. **Input sanity check (lines 230–238).** `sample_is_healthy` requires valid IMU status and finite accelerometer/gyro values. It does not perform sensor communication or calibration.
8. **One control calculation (lines 241–317).** `flight_control_step` begins with four zero outputs and disarmed status. It rejects invalid sensor data or time steps; converts acceleration into approximate roll/pitch; updates the two angle filters and, when flagged valid, altitude filter; checks pilot commands; clamps roll/pitch to ±35°, yaw rate to ±180°/s, and throttle to 0–1. Outer roll/pitch angle PIDs create desired angular rates. Inner roll/pitch rate PIDs and the yaw-rate PID compute corrections. If altitude hold is requested with a valid barometer, a height PID adds a bounded throttle correction. Finally, a provisional X-frame mixer calculates front-left/front-right/rear-right/rear-left normalized values.
9. **Safety boundary (lines 309–317).** Motor order and signs are expressly provisional. Every call leaves `armed=false`; there is no PWM, DShot, arming state machine, or ESC driver. Treat these outputs as test math only.
10. **RP2350 entry point (lines 319–343).** Core 1 is reserved for slow work, but currently only sleeps. Core 0 prints that outputs are disabled and repeatedly calls the math function with empty sensor/command records. That deliberate empty input fails the IMU check and cannot arm or fly.

## Battery monitoring ([`battery_monitor.c`](../flight_controller/battery_monitor.c))

The battery module is hardware-independent and is linked into the RP2350 image, but sensor drivers do not feed it yet. Its scalar Kalman filter smooths total pack voltage and keeps a voltage standard deviation. Its SOC Kalman filter predicts charge using measured current (coulomb counting), then optionally corrects using a pack-specific *rested* open-circuit-voltage-to-SOC observation. That lookup and resting detector are not implemented because the exact pack has not been identified. It reports SOC standard deviation to show uncertainty instead of presenting a falsely precise percentage. Under load, total pack voltage is not used as an SOC observation.

The module can request a controlled landing on a configured low-pack/low-cell/current warning or external BMS alert; it recommends disarming only when the caller says the aircraft is landed. Invalid or stale samples return a sensor-fault action. It cannot disconnect the battery or replace hardware cell protection. Configuration thresholds are deliberately caller-provided and have no flight-ready defaults.

## Hand controller ([`controller.c`](../controller/controller.c))

There are five executable lines of behavior: include standard output and Pico SDK headers; initialize USB/UART stdio; wait 1.5 seconds for a terminal; print that inputs and radio are unconfigured; then spin doing nothing. **No joystick, screen, button, LoRa, or CC1101 code exists yet.** This file is a buildable RP2040 placeholder, not an implemented transmitter.

## What current test results mean

`flight_controller/tests/test_flight_controller.c` compiles the same C implementation with `QCOPTER_HOST_TEST`; it feeds scripted values to the math and filter functions. The Kalman Monte Carlo gives the filter noisy synthetic sensor measurements and compares its estimate with a known synthetic roll trajectory over 500 trials. The separate mass Monte Carlo varies explicitly assumed frame/battery priors. These checks verify software arithmetic under those inputs; they do not simulate motors, airframe response, or flight stability.

For source links, test commands, measured results, and the project status, see the [top-level README](../README.md) and the [timestamped evidence index](../artifacts/test-runs/README.md).
