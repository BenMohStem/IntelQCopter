# Stepwise IntelQCopter agreement

- Work on `codex/stepwise-esp32`; preserve `main` and `esp32-xbox`.
- Current scope: bench hardware diagnostics before control or a new PCB.
- Use PlatformIO/Arduino libraries, C-style functions/structs, one firmware main.cpp.
- Implement the smallest next capability. Commit plan, implementation and validation evidence separately where practical. Push each completed iteration; no empty commits or force-push.
- Physical motor arming and outputs stay disabled. No PWM/Servo/ESC pulses or actuator tests without a later explicit instruction changing that restriction.
- Physical acceptance requires real measurements, original evidence, exact source fingerprint, hardware identity and an operator attestation. Never fabricate success or treat compiler/simulation output as hardware evidence.
- A failure, evidence change, hardware-config change or source change invalidates affected acceptances and downstream work. Preserve old attempts; repair in new commits. Do not delete historical failed logs.
- Offline work is permitted but cannot bypass acceptance dependencies. Planned later stages must not silently become validated.
- User instructions supersede local guidelines. Keep friend assistance isolated from project publishing.
- Use project.json/qcopter.ps1, grouped tools, and important_sources links. Exclude environments, SDKs, credentials and personal files from commits. Source checkouts stay outside build/ and .cache/.

- Function style: opening and closing braces on separate lines (Allman function braces), four-space indentation. Do not place a function opening brace after its signature.
