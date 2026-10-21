# Scope and original failures

The raw `hardening-check-a.log` retains a test-only duplicate ctypes type error; corrected in the test, then final checks passed. `hardening-report.log` retains a report encoding-name error; corrected without altering original measurements. The first host rebuild was blocked by an existing simulation holding the DLL; the live session was gracefully stopped and its recording preserved before compilation. No trial data was overwritten.

Intermediate and final manual/navigation/QEMU trials remain separate directories. Final source matching is checked in the hardening report. The intentional companion-loss trial is expected to fail mission completion, and its original video/logs are retained. Windows hardware builds include upstream NimBLE warnings; new project-source warnings were removed. No physical flashing, BLE/I2C/UART sensor-device, ESC output, measured propulsion or real-time flight proof is claimed.
