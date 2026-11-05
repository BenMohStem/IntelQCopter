# Synthetic rate-step response and PID tuning workflow

Earlier idealized control simulations found a higher-bandwidth PI setting generated roughly 35% overshoot under an **assumed 40 ms motor lag**; a slower conservative setting generated around 10% in that model. FC v2's conservative candidate was evaluated across additional synthetic 500 Hz plants and reported ~6% median overshoot in one sampling of assumed plants.

These results cannot be compared with actual step data until rotor/ESC response and thrust conversion are measured. Overshoot percentage alone is insufficient: rise time, settling time, steady-state error, saturation, peak torque, filtering delay, stability margins, noise sensitivity and battery variation all matter.

## Proposed rate-tuning sequence

1. Keep physical ESC disconnected and test discrete PID math with a known synthetic linear plant.
2. Measure actual rotor lag, thrust/torque leverage and max thrust independently.
3. Update the simulation with motor saturation, dead zone, sample jitter and measured noise.
4. Analyze robust margins over uncertainty intervals; verify each axis sign.
5. Only with independent safety implementation and restrained checks, move to hardware actuation testing.

The user requested a learning-oriented PID design: retain clear units and every Kp/Ki/Kd term; avoid silently replacing PID with LQR before the controller architecture is understood and verified.
