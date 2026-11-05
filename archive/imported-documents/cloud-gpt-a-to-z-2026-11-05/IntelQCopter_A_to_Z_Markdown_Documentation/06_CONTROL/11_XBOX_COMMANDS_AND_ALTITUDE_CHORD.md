# Xbox commands, authority and altitude re-zero gesture

## Virtual bench command mapping

Left X = yaw angular rate; left Y = requested vertical speed; right X/Y = roll and pitch angles. The v2 source clamps roll/pitch to ±15° and yaw rate to about ±1.4 rad/s. Height target is integrated from normalized vertical stick with an approximately ±0.6 m/s setpoint.

The bridge uses serial `$IQ2` CRC frames, validates sequence and normalized ranges, and a 200 ms timeout rejects stale commands. Direct BLE pairing and a verified real receiver are not part of this v2 evidence.

## Requested button combination

D-pad **Up + LB + RB** requests BMP280 relative-altitude re-zero **only while disarmed**. The source accepts the chord once per press and only when recent pressure data is available; it resets altitude Kalman, target and PID integrators. This is a reference/estimation operation, not physical battery/ESC arming.

## Important competing branch conventions

`esp32-xbox` previously assigned D-pad Down + A/B to flight/navigation mode selection and D-pad Down + LT for a held disarm. The current UART bench parser is independent. A future merged controller needs one authoritative input map, hard gesture precedence and a lost-link policy that has been tested with the real receiver and motors restrained.
