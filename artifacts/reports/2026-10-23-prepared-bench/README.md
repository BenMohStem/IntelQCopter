# Prepared sensor/calibration bench - 23 October 2026

This is **offline software evidence**. No hardware was available, no upload occurred, and no physical sensor/calibration step passed. Motor GPIOs remain input/pulldown; there is no flight loop or motor driver.

The final single-file firmware compiled with PlatformIO Espressif32 7.1.3 / Arduino-ESP32 2.0.17 and pinned MPU6050/Adafruit libraries. Reported static RAM19616bytes; firmware flash333837bytes. See `summary.json` for exact source/binary hashes and the final original compiler-log path. The embedded source/configuration SHA is actually present in firmware.bin. The final source-inputs ZIP preserves the bytes matching the build report.

The failed sampling build (wrong BMP library standby enum) and its repaired build are retained along with all earlier attempts. Source-identity-only compilation had an unused-helper warning, resolved when sampling used that helper. No original attempt was erased.

Nine acceptance-ledger tests passed in temporary synthetic fixtures. They exercise dependency order, failure, parent re-pass, source/config/evidence changes, missing attestation and ledger tampering. They are not sensor measurements. The repair command's dry-run checked the actual step03 commit and did not create or switch worktrees.

Prepared diagnostics: board identity/uptime; I2C/chip IDs; fresh raw MPU observations and Welford statistics; forced BMP pressure/temperature; H43-gated raw magnetic field; passive checksummed TOFSense frames; gyro bias and six-face diagonal acceleration fit; independent bench Preferences namespace with CRC/board/version/readback; corrected residual observations. Scope/limits remain in `summary.json` and the bench procedure.

Next actual work: perform material/supply inspection, then board, bus, sensor and calibration tests in order on the real drone. A compiler success is not their acceptance. Only after reviewing those observations should Xbox/estimator/controller stages be added.
