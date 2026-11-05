# AGENTS.md — IntelQCopter local AI coding instructions

This file is intended to be read by AI coding assistants working in the unpacked handoff folder or a fresh IntelQCopter checkout.

## Mandatory first reads

1. `PROJECT_HANDOFF.md` (hardware, architecture, evidence, unresolved faults, exact UART packet format, next work).
2. `NEXT_ACTIONS.md` (smallest executable next actions).
3. `reference_packages/IntelQCopter_QMC5883P_AutoDiagnostics_v18_NoPurchase_Triage.zip` source, especially `SESSION_v17_FINDINGS.md` and `src/main.cpp`.
4. `reference_packages/IntelQCopter_100Hz_UART_MAG_REPAIR_v3.zip` full calibration source and its `platformio.ini`.
5. `reference_packages/SESSION_20261103_184453_23344_SEND_THIS.zip` actual v17 logs; do not guess sample counts.

## Boundaries

- Current PCB is ESP32-S3 (not the older conceptual RP2350). Sensor addresses MPU6050 0x68, BMP280 0x76 via MPU XDA/XCL bypass, QMC5883P 0x2C (ID 0x80); SDA GPIO9 and SCL GPIO8. Keep MPU auxiliary bypass through `USER_CTRL=0x00` and `INT_PIN_CFG=0x02`; no physical INT/DRDY wires required.
- User wants complete calibration (not MAG-only) at QMC normal mode 100 Hz `0x0A=0xC9`, ctrl2 `0x0B=0x08`, sign `0x29=0x06`, I2C 100k initial, Wi-Fi disabled, UART 921600 and existing 52-byte CRC binary protocol. Do not mix UART diagnostic 115200 with binary 921600 code, nor mix v13 MAG-only with full v3.
- User requests per-stage samples gyro 30000; accel 5000 each of 6 faces; mag yaw/pitch/roll 2000 each; tumble 8000; BMP 1000; settle 3 seconds. Optional LiDAR 300 samples at 25/50/100/200cm and skip in GUI. Avoid sacrificing full calibration stages to quick diagnostic shortcuts.
- Windows PowerShell 5.1, PlatformIO espressif32@7.1.3, CH343 COM3/COM4 (auto-detect), `-UploadFirmware` opt-in. Avoid script encoding/pipeline mistakes. Close COM ports in `try/finally` and do not run two serial monitors simultaneously.
- No purchases (especially YKUSH), no fake cold power cycling, no arbitrary timeouts, user wants automatic headless diagnostics and exact log ZIPs. Work with existing USB bridge.
- **Strict safety:** propellers removed; ESC signal wires disconnected for sensor/bench work; motor GPIOs INPUT. Do not add live motor output, arming, or claim flight readiness just to meet competition date; safe flight needs independent validation.
- Not verified: integrated QMC freshness, v18 physical repair, FC 500 Hz achieved timing, calibration quality, motor order/ESC interface, flight.

## Practice

- Each code change: separate clean directory/commit, one tested hypothesis, full original inputs preserved, proper README/CHANGELOG, build and tests when tools available, clearly stated unverified hardware behavior.
- Distinguish device communication (ACK), register readbacks (possible failure), data-ready status, fresh conversions, measurement timing and calibration quality.
- Prioritize v18 `T0 F1 H0 A0 G5 T1 A1` differential read diagnostic (or a smaller well-designed alternative), then minimal change in full v3 calibration if evidence supports it.
- When receiving new results, produce a table of hypothesis, evidence, outcome, and next experiment; archive logs unchanged. If no sensor data, fail closed rather than generate calibration parameters.
- Source of truth is the most recent **measured logs**, not generated summary prose. If this handoff conflicts with future logs, update it with the evidence and date.
