# D-0001: Current project contract

Status: active summary, 2026-10-26. Sources: latest user instructions, AGENTS.md,
docs/PLAN.md and evidence E-FIFO/E-FILTER. Supersedes: historical assumptions only,
not a deletion of previous evidence. Future changes add another decision record.

- Active work is codex/stepwise-esp32 in QCopter-Stepwise. Pico main and ESP32-Xbox
  history remain separate. One C-like Arduino main.cpp, Allman braces, printf.
- Current FC target is 1000 Hz, superseding the earlier 500 Hz target.
- User authorizes relevant bench flashing/stationary tests. Earlier upload-only-by-user
  restrictions are superseded. This does not authorize motor pulses or physical motion.
- Motor outputs/arming remain disabled. Physical gyro rotations are deferred.
- Sensor calibration constants stay frozen unless a documented new fit is requested.
  MPU user trims are volatile: restore at boot and remove only rounding remainder in
  software. Never apply full bias twice or confuse factory OTP with writable persistence.
- BMP280 unavailable. No altitude, yaw heading, PID or flight acceptance in this branch.
- Current filter is PC-only and provisional; smooth output is not an accuracy proof.
- User handles PCB/simulation construction for now. Do not resume archived broad work.
- Work in small reviewable steps. Commit and push meaningful changes; retain failures,
  original evidence and history. Never force-push or publish credentials/private logs.

Why: these choices preserve known calibration/evidence, avoid hidden acceptance
changes, keep teaching understandable, and allow independent software work while
the physical rotation gate is pending.
