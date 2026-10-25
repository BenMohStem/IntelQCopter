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

- Active firmware is temporarily RAW POSE1000_V1 capture: Adafruit initialization, no user offset writes or calibration matrix. Use docs/record-18-poses.md and record-poses, not historical corrected-firmware checks. 18 runs: Z+, Z-, X+, X-, Y+, Y- repeated three times; preserve all raw samples and failed attempts. The user flashes; do not upload or start real measurements unasked. Earlier calibration implementation remains at a1d3c78. Future code must avoid applying hardware and software bias twice.

- On 25 October the user reported that the FC detached during the 18-pose session. That mounting-dependent dataset is invalid for calibration; its session 20261025T123623069142Z and derived serial-resync report were deleted at the user's request. Begin a new session after securing the final mounting; do not reuse earlier alignment coefficients without new validation.

- New session 20261025T132707615977Z has 18 complete records/360000 samples. Offline analysis uses repetitions 1-2 for fitting and reserves repetition 3. See measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md. Coefficients remain provisional; transients and temporal correlation prohibit blanket IID/Gaussian claims. Next physical step is independently measured tilted poses, not PID. Firmware stays raw until the validation/implementation lessons.

- User selected ±45-degree roll/pitch validation. record-tilts reuses raw POSE1000_V1 firmware, prompts four 20-second poses, records reference angles/method/uncertainty, freezes the candidate and checks source/board/offset continuity. Defaults reference the completed local session. No reflash or automatic acceptance; keep tilt originals private and require mounting continuity. See docs/record-45-degree-tilts.md.

- Tilt session 20261025T174502114509Z: user confirmed actual raised-side order +X,-X,+Y,-Y after original labels were swapped relative to the recorder convention. User estimates about ±1° with a hand-held 45° ruler; not a Gaussian sigma/certified bound. Preserve originals; clarified derived report is measurements/mpu6050/2026-10-25-tilt-validation/report.md. No refit. Differences 1.061°,0.254°,0.420°,0.209° support the next provisional bench-implementation lesson, not sub-degree/flight acceptance. Future prompts state raised side explicitly.
