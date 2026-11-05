# Controller architecture — all seven PID loops

The v2 controller uses **cascaded linear PID near hover**, not LQR in parallel or an untested nonlinear motor actuation law.

## Inner and outer loops

- Roll angle PID -> requested body roll rate p* -> roll-rate PID -> roll torque τx.
- Pitch angle PID -> requested body pitch rate q* -> pitch-rate PID -> pitch torque τy.
- Xbox yaw stick -> yaw-rate request r* -> yaw-rate PID -> yaw torque τz. **No yaw angle hold PID.**
- Height PID -> vertical-speed correction v* -> vertical-speed PID -> commanded world-up acceleration a*.
- Collective feedforward: `T ≈ m(g+a*)/cos(tilt)`, then a virtual four-motor X mixer.

## Frequency

Each PID is calculated within the 500 Hz nominal main source loop using the actual dt, but its input may be derived from slower pressure measurements via prediction and fusion. The source does not claim every barometer conversion is 500 Hz.

## Safety and readiness

Virtual outputs only. `Controller::Model::clearedForFlight()` is hardcoded false. No physical ESC pulses/arming/validated mixer are present. The entire graph is useful for mathematical reasoning, bench logging and simulator tests, not free flight.
