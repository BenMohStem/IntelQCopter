# Flight readiness gate — what is currently missing

**As of this documentation snapshot, no validated free-flight capable controller has been shown.** A source with PID, a good calibration and a simulated thrust mixer is not a flight-ready firmware.

## Required evidence before motor-output firmware is even considered

- Verified physical ESP32 pins, ground, power sequencing and supply robustness under electrical load.
- Verified body/IMU/MAG axes and gravity orientation (det+1 transforms), gyro signs and mixer signs.
- Actual motor order, rotation direction, propeller pairing and ESC control interface.
- Mass, COM and feasible hover thrust with meaningful motor correction margin at actual pack voltage.
- Validated arming state machine, independent physical kill/cut-off, tested RC loss/stale command behavior, safe startup and bounded throttle.
- 500 Hz worst-case timing under representative load, sensor freshness, watchdog, integrator anti-windup and fault response.
- Altitude estimator dynamic validation; no claims of position hold without XY sensing.
- Controlled, restrained and propeller-free testing protocol; actual propulsion tests only in appropriate protective equipment/test facilities.

## Explicitly blocked now

`IQ_MOTOR_OUTPUTS_ENABLE=0`; `clearedForFlight()` always false. Do not remove these gates in the name of speed, competition timing or apparently stable simulated step responses. A future motor-capable project should be separately reviewed and tested.
