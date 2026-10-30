# Pitch direction and held-stick fix

Two functional changes only: negate the right vertical stick mapping; stop clearing display readiness merely because an unchanged stick has not produced a report for300ms. Before: that timeout set neutral_seen=false, so the command went to0 and required recentering. After: retain the last decoded display target while connected and initial input received. Disconnect still zeros targets and requires neutral after reconnect. The independent300ms Xbox-worker-to-telemetry watchdog remains, so a stopped worker cannot leave its packet valid indefinitely.

This preview has no actuator path. Holding the last BLE report while connected is not an accepted flight-link failsafe; a measured liveness policy must be resolved before connecting targets to motors. No new task/queue/optimization or calibration change. FIFO nominal1kHz configuration retained.

Operator requests correctness/readability first, optimization later. Two small code corrections, no extra hardware test campaign. Check pitch forward/back and hold one stick still for2seconds, then release. Host regression covers both pitch endpoints, held values without reports, disconnect/reconnect and neutral recovery, plus bounded monotonic mapping.

ESP32 build and focused native actual-function tests passed. Hardware upload/check pending; no physical correction success claimed before operator observation.

