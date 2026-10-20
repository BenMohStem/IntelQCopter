# Reading the controller code

This guide follows the code in execution order. The RP2350 program now schedules a 500 Hz control loop and contains a radio-frame parser, but sensor acquisition and the RF driver are stubs and ESC outputs remain disabled. The RP2040 program scans a proposed hand-controller input map and emits debug frames over USB; LoRa transmission is not connected.

## Flight controller ([`flight_controller.c`](../flight_controller/flight_controller.c))

Open the linked source and follow these sections from top to bottom.

1. **Headers and loop constants.** `math.h` supplies angle functions and `isfinite`; Pico headers are omitted for the host test. `FC_LOOP_HZ` is the 500 Hz target (2 ms). The embedded entry point schedules that period and counts missed deadlines, but no physical RP2350 timing has been measured. `FC_MAX_CONTROL_DT` rejects time gaps over 10 ms.
2. **State and input records.** `pid_t` stores PID tuning and history. `angle_kf_t` holds roll/pitch angle and gyro-bias estimates plus a 2×2 covariance. `altitude_kf_t` holds height and vertical speed plus covariance. `pilot_command_t` and `sensor_sample_t` define the data that future radio and sensor drivers must supply. `control_output_t` contains four normalized *numbers*, not electrical ESC signals.
3. **Estimator and PID starting values.** These are provisional constants for software exercising. PID gains are not flight-tuned; filter noise values are assumptions. The two angle filters are roll and pitch; yaw is intentionally rate-only.
4. **Small PID helpers.** `clampf` bounds a value. `pid_reset` clears controller history. `pid_update` computes setpoint error, derives on the measurement to avoid a setpoint kick, limits the integrator/output, and conditionally prevents integrator windup. Invalid or excessive `dt` resets the PID and returns zero.
5. **Roll/pitch Kalman update.** First it predicts angle from gyro rate after subtracting estimated gyro bias. It predicts covariance, then compares the accelerometer angle with the prediction (`innovation`). Kalman gains blend the two estimates and update their covariance. `p00` is angle variance, so `sqrt(p00)` is the predicted angle standard deviation.
6. **Altitude Kalman prediction/update.** The state is height and vertical speed. `altitude_kf_predict` advances both using gravity-compensated vertical acceleration and grows covariance. `altitude_kf_baro_update` corrects height and speed using a barometer height sample. These functions are not yet validated against a vehicle/sensor simulation.
7. **Input sanity check.** `sample_is_healthy` requires valid and calibrated IMU status and finite accelerometer/gyro values. It does not perform sensor communication or calibration.
8. **One control calculation.** `flight_control_step` starts disarmed with zero outputs. It rejects invalid sensor data/time steps, derives approximate gravity roll/pitch, updates independent angle-plus-bias Kalman filters and the [height,speed] Kalman filter, and checks pilot commands. Roll/pitch angle P controllers request angular rates; two inner rate PI controllers and one yaw-rate PI controller request torque proxies. Centered throttle (0.5) requests zero vertical speed; moving the stick integrates a height target. Exactly one altitude PID uses height error, its integral, and commanded-minus-estimated vertical speed for its derivative term. Tilt compensation adjusts collective. Together these are exactly six PID objects (some I/D gains are zero). Barometer corrections occur only on fresh samples. The FLU X mixer computes front-left/front-right/rear-right/rear-left thrust proxies, with positive pitch increasing rear thrust. Hardware outputs remain disabled. See the detailed [mathematical document](modelisation/main.tex) for units, gain derivations, simulation overrides and uncertainty assumptions.
9. **Safety boundary.** Motor order and signs are expressly provisional. Every call leaves `armed=false`; there is no PWM, DShot, arming state machine, or ESC driver. Treat these outputs as test math only.
10. **RP2350 entry point.** Core 1 polls the nonblocking radio stub and can queue fixed command frames. Core 0 drains the queue and executes the timed loop. Sensor records are deliberately empty, the radio stub returns no frames, and all motor outputs remain disabled.

## Battery monitoring (inside `flight_controller.c`)

The battery module is hardware-independent and is contained in the RP2350 application source, but sensor drivers do not feed it yet. Its scalar Kalman filter smooths total pack voltage and keeps a voltage standard deviation. Its SOC Kalman filter predicts charge using measured current (coulomb counting), then optionally corrects using a pack-specific *rested* open-circuit-voltage-to-SOC observation. That lookup and resting detector are not implemented because the exact pack has not been identified. It reports SOC standard deviation to show uncertainty instead of presenting a falsely precise percentage. Under load, total pack voltage is not used as an SOC observation.

The module can request a controlled landing at/below the configured SOC limit (set to 20% by the current test configuration), on a configured low-pack/low-cell/current warning, or on an external BMS alert; it recommends disarming only when the caller says the aircraft is landed. Invalid or stale samples return a sensor-fault action. It cannot disconnect the battery or replace hardware cell protection. Configuration thresholds are deliberately caller-provided and have no flight-ready defaults. No battery sensors or landing action are wired into the flight loop yet.

## Hand controller ([`handler_controller.c`](../handler_controller/handler_controller.c))

The RP2040 code configures the ADC mux and six active-low buttons, calibrates joystick centers at boot, normalizes the Mode 2 axes, applies a 4% dead band, debounces buttons, then generates a sequence-numbered CRC-protected command frame at 100 Hz. It prints every tenth frame over USB for inspection. The frame is not sent by RF yet: the exact LoRa module and band are unconfirmed. LT/RT are reported as analog values; buttons are data only, with no arm/disarm behavior. See the [input map and packet format](controller-inputs.md).

## Radio command path

The wire-protocol definitions are embedded in both firmware C files. The host-test script verifies that these sections remain identical. The RP2350 core-1 loop currently contains an explicit no-data radio stub. If a driver supplies frames, core 1 queues them for core 0; core 0 validates the packet, rejects non-new sequence numbers, maps normalized input to roll/pitch angle and yaw-rate setpoints, and expires commands after 150 ms. This path is software scaffolding, not an operating radio link.

## Sensor calibration

All calibration collectors and correction functions live in `flight_controller.c`. The control core rejects an IMU sample unless it has been marked calibrated; `sensor_sample_apply_imu_calibration` applies accepted accelerometer and gyro results and sets that flag. No calibration values are captured from physical hardware yet. See the [calibration procedure and limits](sensor-calibration.md).

## What current test results mean

`flight_controller/tests/test_flight_controller.c` compiles the same C implementation with `QCOPTER_HOST_TEST`; it feeds scripted values to the math and filter functions. The Kalman Monte Carlo gives the filter noisy synthetic sensor measurements and compares its estimate with a known synthetic roll trajectory over 500 trials. The separate mass Monte Carlo varies explicitly assumed frame/battery priors. These checks verify software arithmetic under those inputs; they do not simulate motors, airframe response, or flight stability.

For source links, test commands, measured results, and the project status, see the [top-level README](../README.md) and the [timestamped evidence index](../artifacts/test-runs/README.md).
