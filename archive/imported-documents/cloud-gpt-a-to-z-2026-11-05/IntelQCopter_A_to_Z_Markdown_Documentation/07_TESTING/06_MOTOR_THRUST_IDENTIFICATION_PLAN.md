# Propulsion system identification — prerequisite for real flight gains

Current rate gains are torque-domain and depend on **unmeasured** maximum rotor thrust, thrust-command curve, time constant, yaw reaction torque, mass and inertia. Real hardware identification must precede any claim that PIDs or hover commands are “best.”

## Controlled lab setup

Use a secure thrust fixture, current/voltage measurement, guarded propeller enclosure, calibrated scale, fire-safe LiPo practices, a physical disconnect/kill and a competent operator. Document true propeller dimension/pitch and spin direction. This is a separate, supervised actuation test—not a suggested unrestrained bench spin.

## Measurements

- `command` vs stable thrust `T_i`, supply voltage and current per motor.
- Dead zone, startup threshold, hysteresis, command cap and response dynamics to small safe command steps.
- Differences between the four motors/ESCs and effect of pack voltage sag.
- Real reaction yaw torque coefficient or conservative identifiable bounds.
- Heating and limits before continuous operation.

## Model update

Fit an appropriate monotone thrust surrogate and motor response model with residual/uncertainty estimates; do not assume `T=Tmax*u`. Update mass from scale, arm coordinates from ruler, motor map/rotation from physical inspection. Recompute closed-loop stability margins and feasible hover setpoint with correction reserve.

## Safety rule

This plan does not override the v2 motor-off guard. Independent arming, receiver-loss stop and physical cut-off remain required before a vehicle-wide motor test.
