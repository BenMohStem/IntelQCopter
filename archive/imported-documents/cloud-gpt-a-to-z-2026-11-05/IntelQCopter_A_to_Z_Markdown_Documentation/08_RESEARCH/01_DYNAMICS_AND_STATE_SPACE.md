# Mathematical foundations — vehicle equations and linearization

## Full rigid-body idea

Position, velocity, attitude and angular rate are the vehicle states. In inertial coordinates the force sum governs translation, gravity subtracts from total thrust vector, and `J omega_dot = tau - omega×Jomega` governs rotation. Lagrangian mechanics reproduces these physical laws and helps track energy, mass properties and generalized forces.

## Hover-linear models

For small angles, yaw reference fixed, and ideal rotor dynamics, angular rate dynamics are roughly a torque-controlled integrator. The angle dynamics add another integrator; hence cascade angle->rate PID. Vertical motion around hover can be linearized as `zdot=vz`, `vzdot ≈ T/m -g`; thrust control must compensate mass and tilt. This is valid locally, not for arbitrary acrobatics or saturated motors.

## Where state space helps

Even with PID as the controller, state-space models are useful for the altitude KF, simulated plant `x_{k+1}=Ax_k+Bu_k`, observability and control authority analysis, and MIMO coupling audit. Full LQR should not silently replace a comprehensible PID that the owner is learning; it can be an offline comparator after real plant identification.

## Source

User-supplied **IntelQCopter Volume I Edition 2** studies Lagrangian equations, linearization, Laplace, transfer functions, state-space, PID, LQR and Kalman. Its HMC sensor label and illustrative mass/motor assumptions should be corrected in future revisions **only with measurement evidence**.
