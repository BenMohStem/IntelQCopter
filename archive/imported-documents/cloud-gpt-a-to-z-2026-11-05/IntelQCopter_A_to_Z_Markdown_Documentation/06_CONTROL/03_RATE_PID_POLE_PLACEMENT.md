# Rate PID gains from inertia and pole-placement intuition

Near hover and for one axis with negligible cross coupling, the rotational plant is `J_i dω_i/dt = τ_i`. The preliminary PI rule chooses a second-order closed-loop shape:

\[
K_{p,i}=2\zeta\omega_n J_{ii},\qquad
K_{i,i}=\omega_n^2 J_{ii},\qquad
K_{d,i}=0.006\,J_{ii}.
\]

Default roll/pitch `(ωn,ζ)=(2 rad/s,2)`, yaw `(1.2 rad/s,1.9)`. The Kd term is intentionally small, with derivative on measured rate, and is not derived from this pure PI pole placement. The code filters derivative before using it.

## Worked roll example

With `Jxx≈0.009781 kg·m²`: Kp=2×2×2×0.009781≈0.07825; Ki=2²×0.009781≈0.03912; Kd=0.006×0.009781≈5.87×10⁻⁵. These output modeled torque (N·m) for errors in rad/s, not normalized motor signal.

## Model uncertainty

If J is uncertain by σ_J and the gain formula is linear in J, first-order gain SD is `σ_K≈K(σ_J/J)`. Correlated J entries, rotor lag, unmeasured thrust curves and actuator limits add systematic uncertainty *outside* this simple propagation.

## Actuator lag and stability

With a first-order motor lag τm, an ideal continuous unfiltered PID characteristic can be written `(τmJ)s³+(J+Kd)s²+Kp s+Ki=0`, with Routh-Hurwitz condition `(J+Kd)Kp > τmJ Ki` for positive coefficients. That check does not certify the real discrete filtered PID, delay, quantized ESC command or saturated mixer. An experimental step response is required.
