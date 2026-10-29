# offline-estimator / codex-stepwise / 2026-10-29

- Branch/base: codex/stepwise-esp32, 430fab5.
- Scope: tools/estimation, docs/attitude-preparation.md; additive event/handoff.
- Added acceleration_ambiguity.py: standard-library analytic calculation, no simulation engine or physical device. Ideal noiseless constant-orientation cases, unknown initial attitude, no transition history.
- Result: stationary 10-degree tilt and level constant acceleration yield identical gyro/accelerometer inputs, including exactly 1 g magnitude. Demonstrates why magnitude-only gating cannot guarantee valid tilt. No filter improvement/flight acceptance claimed.
- Checks: model equality/angle/norm assertions pass; domain catalog check passes. Existing gyro-frame test failed to import serial under default Python; no dependency installation attempted.
- Existing firmware edit preserved and excluded: gyro matrix element 0.288364175f changed to 0.2808364175f; inconsistent with frozen configuration. No source fingerprint refreshed.
- Physical gates: gyro sign/scale, estimator dynamics, actual timing and motor interfaces remain unresolved. No board available, upload, port access or actuator output.
- Next smallest action: model bounded acceleration/recovery with frozen candidate gains; resolve existing coefficient edit before firmware acceptance.
- Integrator: same agent, explicit-file review; reservation released after publication.
