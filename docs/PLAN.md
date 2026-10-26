# Current software plan

**FC target: 1,000 Hz.** Runtime stays in one Arduino-style main.cpp, mostly C-style functions, Allman braces and Serial.printf. Motor outputs/physical arming remain disabled. BMP280 is unavailable and deferred. ROS2, Xbox, PCB and simulation work are outside this small sensor/timing stage.

| Stage | Current evidence | Next action |
|---|---|---|
| Accelerometer calibration and boot restoration | Real six-pose/tilt/XYZ/reset evidence preserved; reference uncertainty remains | Preserve coefficients |
| Gyro bias and boot restoration | Real stationary, reset and full-power-cycle recordings checked; residual bias remains | Preserve measured constants; do not refit validation records |
| Stationary repeatability | Assistant-run originals and covariance reports saved; support condition assumed | No more repeated low-rate sampling unless it resolves a specific issue |
| 1 kHz FIFO/state processing | Prepared, ESP32 build + 41 unique offline checks + 14 actual-source host cases pass | New hardware upload, then automatic real throughput/backlog/deadline tests |
| Gyro body-axis mapping | Not physically verified | Known rotations require operator action |
| Attitude filter | Not yet implemented/accepted | After acquisition and frame checks |
| Rate/angle PIDs and mixer | Not implemented in this branch | Add one controlled stage at a time, then simulation/gain validation |
| Altitude/vertical control | BMP280 unavailable | Defer; do not claim hover capability |

[Current 1 kHz guide](fifo1000-stage.md) describes the exact runtime, test commands and limits. High average sample rate is not a complete 1 kHz FC or hard deadline proof. Physical tests must identify actual engine/source/configuration and retain failures.

The user delegates fixed stationary tests to the assistant and prefers concise results after the work. No upload is authorized by that standing delegation; the user previously reserved flashing and a separate upload-choice question is pending. Moving/reference tests still need physical operator action. Do not fabricate level, stationarity, dynamics or successful hardware operation.

After each meaningful change, run relevant tests, commit and push without rewriting history. If a hardware stage fails, repair that stage and rebuild later work on it. Keep original evidence private; publish reviewed derived summaries. No empty commits.

[Earlier detailed plan/history](../archive/plan-history/2026-10-26-before-1khz.md) and the Git history preserve earlier accepted work. Archived references to a 500 Hz target are superseded by the user's 1,000 Hz request.
