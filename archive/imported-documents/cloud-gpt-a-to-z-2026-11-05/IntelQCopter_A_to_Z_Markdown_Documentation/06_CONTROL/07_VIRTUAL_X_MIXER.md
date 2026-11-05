# X mixer: torque and thrust to four **virtual** motor fractions

The v2 mixer treats four motor thrusts as an idealized linear actuator. With mass m, gravity g, desired vertical acceleration a_z, nominal diagonal span D and tilt θ, it requests collective:

\[
T_{sum}=\frac{m(g+a_z)}{\cos(tilt)}.
\]

Define `arm=D/(2sqrt(2))`, `base=Tsum/(4 Tmax)`, `roll=τx/(4 arm Tmax)`, `pitch=τy/(4 arm Tmax)`, `yaw=τz/(4 κ Tmax)`. For source-annotated motor order FL,FR,RR,RL:

```text
FL = base + roll - pitch + yaw
FR = base - roll - pitch - yaw
RR = base - roll + pitch + yaw
RL = base + roll + pitch - yaw
```

Fractions are clamped to 0..1. Source flags saturation and resets controller integrals. **These are unverified signs and motor order**, not calibrated ESC outputs; they are intentionally never written to GPIO.

## Why simple clamping is not full allocation

Independent clipping changes requested torques and may create large yaw/roll errors. Proper flight-grade allocation considers available thrust reserve, total thrust priority, constraints, actuator asymmetry, and saturation-aware integrator behavior. It must be validated with motor directions, real commanded thrust curves and an independent kill path.
