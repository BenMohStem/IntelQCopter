/*
 * IntelQCopter Xbox Controller Interface — documentation / planning addition.
 *
 * This describes the intended Xbox controller input mapping for the drone
 * flight controller, enforcing the motor cap policy (10% idle / 45% normal /
 * 65% boost) on the mathematical mixer output. No ESC driver consumes these
 * values; no physical motor output path exists yet. This is a design/planning
 * document recorded in the firmware source for future adapter work.
 */

/* The intended Xbox Series X/S controller mapping (user-requested design):
 *
 *   Right joystick (X: roll correction, Y: pitch correction):
 *     - Right/left stick deflection -> roll (positive right = right roll)
 *     - Up/down stick deflection  -> pitch (positive up = forward pitch)
 *
 *   Left joystick (X: collective / throttle vertical speed, Y: yaw rate):
 *     - Up/down deflection      -> vertical speed command (positive up = climb)
 *     - Right/left deflection    -> yaw rate (positive right = clockwise yaw)
 *
 *   RT (right trigger):
 *     - Progressive climb / collective boost. Only active when armed and
 *       all inputs valid (calibrated sensors, neutral position, fresh IMU).
 *     - Maximum per-motor cap: 65% (boost cap from motor policy).
 *
 *   LT (left trigger):
 *     - Progressive descent / collective reduction. Mirror of RT.
 *     - Negative values cannot exceed normal 45% cap.
 *
 *   Four buttons (+ shape, directional pad):
 *     - A (north button):  deliberate arming gesture. Must be pressed AFTER
 *       release; requires fresh calibrated IMU, neutral sticks/triggers,
 *       and a valid entry attitude. Never auto-arms on startup/reconnect.
 *     - X (south button):  disarm / kill. Latches disarm; takes priority.
 *     - Y (west button):  emergency kill / immediate full disarm.
 *     - B (east button):   reserved / future boost override control.
 *
 *   LB / RB (shoulder buttons):
 *     - LB: manual disarm / reset calibration state.
 *     - RB: future companion/authoritative-mode override.
 *
 * This interface uses a 22-byte binary command frame shared with the RP2040
 * handheld controller (see the WIRE PROTOCOL v1 definition above). The frame
 * carries: sequence | roll (deg/scale) | pitch (deg/scale) | yaw_rate (deg/s) |
 * throttle [0,1] | LT [0,1] | RT [0,1] | buttons | CRC-16.
 *
 * Motor cap enforcement (user revision 2026-10-28, enforced in flight_control_step):
 *   - No throttle: 10% per-motor idle (centered stick)
 *   - Normal manual input: 45% maximum per-motor command
 *   - RT boost: 65% maximum per-motor command; boost releases immediately
 *     restore 45% cap (no lingering high-power state)
 *
 * Safety rules preserved:
 *   - Never arm on startup/reconnect; only deliberate A-press after release.
 *   - Stale controller (>250 ms since last frame) forces disarm.
 *   - Invalid numerical commands (NaN, non-finite, out-of-range) close to 0.
 *   - Calibration/NVS writes only while disarmed.
 *   - Physical motor mapping, ESC protocol, and wiring remain unvalidated;
 *     these commands remain mathematical demands until a verified adapter
 *     connects them to physical ESC/GPIO pins.
 */
