# Rigid-body dynamics from Lagrangian model to a PID plant

For body angular velocity ω and center-of-mass inertia matrix J, rigid-body dynamics are:

\[
J\dot\omega =\tau - \omega\times(J\omega).
\]

For a small-angle, low-rate hover approximation with J diagonal and little gyroscopic coupling, each rate channel behaves approximately `Jii ωdot_i = τ_i`, motivating a simpler per-axis rate PID. The full Euler/Lagrange formulation in the Claude study is useful for understanding what was neglected and when the approximation fails.

## Assumption-based profile A

`m≈1.104 kg`, `J≈diag(0.009781,0.010390,0.019170) kg·m²`, with nonzero uncertainty. Jx and Jy differ because the geometrical model is not exactly symmetric. Actual off-diagonal components depend on battery, ESCs, mounting and wires, and should not be silently forced zero in final physical calculations.

## Unmodeled effects

Motor reaction torque, rotor gyroscopic moments, thrust dynamics, propeller aerodynamics, arm flexibility, vibration, battery sag, actuator dead zone and airflow interaction. They are not captured by the single double-integrator approximation. The mass model and PID derivation establish a *starting hypothesis* for bench identification, not a proof of hover.
