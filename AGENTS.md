# Stepwise IntelQCopter agreement

- Work on `codex/stepwise-esp32`; preserve `main` and `esp32-xbox`.
- Current scope: software teaching in very small increments. The user designs/makes the PCB; do not work on PCB design unless explicitly asked again.
- Use PlatformIO/Arduino libraries, C-style functions/structs, one firmware main.cpp.
- Implement the smallest next capability. Commit plan, implementation and validation evidence separately where practical. Push each completed iteration; no empty commits or force-push.
- Physical motor arming and outputs stay disabled. No PWM/Servo/ESC pulses or actuator tests without a later explicit instruction changing that restriction.
- Physical acceptance requires real measurements, original evidence, exact source fingerprint, hardware identity and an operator attestation. Never fabricate success or treat compiler/simulation output as hardware evidence.
- A failure, evidence change, hardware-config change or source change invalidates affected acceptances and downstream work. Preserve old attempts; repair in new commits. Do not delete historical failed logs.
- Offline work is permitted but cannot bypass acceptance dependencies. Planned later stages must not silently become validated.
- User instructions supersede local guidelines. Keep friend assistance isolated from project publishing.
- Use project.json/qcopter.ps1, grouped tools, and important_sources links. Exclude environments, SDKs, credentials and personal files from commits. Source checkouts stay outside build/ and .cache/.

- Function style: opening and closing braces on separate lines (Allman function braces), four-space indentation. Do not place a function opening brace after its signature.

- Readability: one statement per line, four-space indentation, opening/closing braces on separate lines for functions and control blocks. Expand compact if/for/while bodies. Formatting must preserve C++ token spelling/order and behaviour.

- Teaching pace: keep the first firmware about20lines. Aim for3â€“10 added lines and one new idea per later step. Explain the delta and expected real result. Do not automatically expand future lessons or re-import archived diagnostics; wait for the user to follow the current lesson.

- The user also builds the Gazebo simulation. Offer commands and small explanations; do not generate worlds/models/complete simulation unless explicitly requested. Keep simulation learning separate from the tiny active MCU lesson.

- Serial output style: use Serial.printf with format specifiers and explicit newline escapes, following the user's examples.

- Calibration teaching: prefer explicit named scalar corrections and simple library calls. For fixed measured constants, firmware flash is sufficient persistence; do not reintroduce a complex NVS record layer unless runtime recalibration needs it. Preserve accurate frame mapping and distinguish fitted candidates from physically validated calibration.

- Earlier calibrated firmware at a1d3c78 used Adafruit MPU6050. Its existing Adafruit BusIO dependency writes accelerometer bias registers relative to the reset factory baseline, preserving bit 0 and verifying readback. No second MPU library and no duplicate software bias subtraction. Scale/mounting remains in software. Gyro sensor-coordinate bias was measured in the 25 October stationary level capture; proposed hardware trim still needs post-upload verification. Config changes require readback and physical checks; never label enum codes as physical units.

- The 25 October level trim is a constrained rotation based on a user-confirmed level stationary capture, not a new full accelerometer calibration. Gyro biases from that capture are restored to volatile MPU gyro offset registers at boot. Post-upload hardware verification and independent orientation checks remain required. Do not equate offline replay with a new physical measurement.

- The sensor remains rigidly mounted to the drone. Never accept calibration from one level pose or a same-pose temporal holdout alone. Require repeated multi-pose measurements and independent tilted-orientation validation; follow docs/multi-pose-calibration.md. Keep reference alignment distinct from bias/scale calibration.

- Earlier measurement firmware (checkpoint ad6b19b) was RAW POSE1000_V1 capture: Adafruit initialization, no user offset writes or calibration matrix. Use docs/record-18-poses.md and record-poses, not historical corrected-firmware checks. 18 runs: Z+, Z-, X+, X-, Y+, Y- repeated three times; preserve all raw samples and failed attempts. The user flashes; do not upload or start real measurements unasked. Earlier calibration implementation remains at a1d3c78. Future code must avoid applying hardware and software bias twice.

- On 25 October the user reported that the FC detached during the 18-pose session. That mounting-dependent dataset is invalid for calibration; its session 20261025T123623069142Z and derived serial-resync report were deleted at the user's request. Begin a new session after securing the final mounting; do not reuse earlier alignment coefficients without new validation.

- New session 20261025T132707615977Z has 18 complete records/360000 samples. Offline analysis uses repetitions 1-2 for fitting and reserves repetition 3. See measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md. Coefficients remain provisional; transients and temporal correlation prohibit blanket IID/Gaussian claims. Next physical step is independently measured tilted poses, not PID. Firmware stays raw until the validation/implementation lessons.

- User selected ±45-degree roll/pitch validation. record-tilts reuses raw POSE1000_V1 firmware, prompts four 20-second poses, records reference angles/method/uncertainty, freezes the candidate and checks source/board/offset continuity. Defaults reference the completed local session. No reflash or automatic acceptance; keep tilt originals private and require mounting continuity. See docs/record-45-degree-tilts.md.

- Tilt session 20261025T174502114509Z: user confirmed actual raised-side order +X,-X,+Y,-Y after original labels were swapped relative to the recorder convention. User estimates about ±1° with a hand-held 45° ruler; not a Gaussian sigma/certified bound. Preserve originals; clarified derived report is measurements/mpu6050/2026-10-25-tilt-validation/report.md. No refit. Differences 1.061°,0.254°,0.420°,0.209° support the next provisional bench-implementation lesson, not sub-degree/flight acceptance. Future prompts state raised side explicitly.

- Historical lesson C5a at 4f837bc was accelerometer SENSOR-X bias only. main.cpp restores target -2348 from recorded post-reset baseline -2074, using measured b_x=1.308683406 m/s² and an even decrement of 274. Require baseline/config/readback checks; no cumulative subtraction or duplicate software bias. Other biases/matrix/gyro are not implemented. record-bias-x prepares raw/corrected/raw bench evidence, not a physical pass. The user uploads; do not open serial or flash unasked. Raw recorder source is preserved byte-for-byte at archive/bench-checkpoints/raw-pose1000/main.cpp.txt. See docs/bench-x-bias.md.

- User explicitly requested applying the complete coupled XYZ correction now, overriding sequential single-axis implementation. Active firmware restores all three bias words and applies C to the coherent XYZ vector at every orientation. Only register-rounding remainder is subtracted in software; never subtract full bias twice or force the vector norm/pose to g/level. Targets X/Y/Z=-2348/2993/1212 from known baselines. record-accel is the current arbitrary-orientation collector; old X-only/raw protocols are historical. Full implementation is prepared/compiled, physical response and reset validation remain pending. See docs/full-accelerometer-calibration.md.

- First real XYZ session 20261025T185849159245Z: 2727 source-matched MCU samples at 90.906 Hz; original hash and full matrix output verified, maximum float arithmetic difference 1.542e-6 m/s². User reports motionless on support, not independently level. Mean vector indicates ~1.908° tilt, not a demonstrated calibration error; norm is 9.86573 m/s² (~+0.602% vs standard g), retained without refit/normalization. Targets are -2348/2993/1212; one boot is observed, repeated reset/full-power-cycle validation is next and still pending. See measurements/mpu6050/2026-10-25-first-live-xyz/report.md. record-accel accepts --note/--condition to document future runs.

- Restart restoration checked in operator-labeled sessions 20261025T200450965913Z (ESP32 reset) and 20261025T200531245399Z (full power cycle), 910 samples each. Same board/source/profile, original SHA and every matrix output verified. Boot words [-2074,2979,1074], targets [-2348,2993,1212], no accumulation observed in these runs. C6 may progress; absolute accuracy/temperature/flight validation remain separate. Report: measurements/mpu6050/2026-10-25-restart-validation/report.md. Next small lesson is gyro XYZ observation/bias; no gyro firmware change in this review. MCU setup resets MPU too, and supply removal is operator-reported.

- Current active source is IMU_XYZ_V1 gyro observation, preserving the complete XYZ accelerometer correction. Use record-gyro; record-accel is historical ACCEL_XYZ_V1. Gyro user-offset words are read only, candidate bias is PC-only with temporal holdout; no gyro correction applied. Do not reuse earlier pre-remount gyro coefficients. Build/offline checks are not hardware acceptance. See docs/gyro-observation.md.
