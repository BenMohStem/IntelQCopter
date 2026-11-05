# PID implementation details — discrete integrator, derivative and saturation

The v2 `include/pid.hpp` implements:

\[
u=K_p e+K_i\int e\,dt-K_d D_{filtered}(y).
\]

The derivative is **on measurement** rather than error; this avoids setpoint derivative spikes. The derivative state uses an exponential first-order low-pass with `alpha=exp(-2π f_D dt)`. Rates use 32 Hz derivative filter, angles 15 Hz, height 6 Hz, vertical 8 Hz.

## Integral handling

The controller clamps integral state to an `iCap`, clamps output to `outputLimit`, and conditionally cancels the most recent integration step if it would drive an already saturated output further into saturation. The controller resets integrators when the model/state/pilot is not ready, and on virtual motor mixer saturation.

## Why nonzero Kd is not automatically useful

A small Kd satisfies the user's request to track all three gains, but a noisy, delayed derivative can destabilize real motors. MPU DLPF already introduces delay; cascading further derivative filters changes phase margin. The real optimum depends on measured actuator lag and vibration spectrum. Source gains are initial **simulation-only** candidates.

## Good logging

Record each loop's setpoint, measurement, proportional term, integral state, derivative state, clamp flags, dt and output. Current v2 telemetry prints only condensed virtual mixer/health values, so deeper PID traces require a controlled logging extension that does not block the 2 ms loop.
