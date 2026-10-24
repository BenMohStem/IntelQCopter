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

- Current MPU readings/configuration use Adafruit MPU6050. Its existing Adafruit BusIO dependency writes accelerometer bias registers relative to the reset factory baseline, preserving bit 0 and verifying readback. No second MPU library and no duplicate software bias subtraction. Scale/mounting remains in software. Gyro bias is unmeasured. Config changes require readback and physical checks; never label enum codes as physical units.
