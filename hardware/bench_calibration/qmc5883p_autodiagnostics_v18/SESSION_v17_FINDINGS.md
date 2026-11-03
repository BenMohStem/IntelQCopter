# Review of the uploaded v17 diagnostic session

**Evidence:** `SESSION_20261103_184453_23344_SEND_THIS.zip`, uploaded 2026-11-03, plus console paste. This document describes the data, not hardware-proven causes.

- **21 planned, 7 measured (all zero samples), 14 upload errors.** A0 A1 A2 A3 A4 B1 B2 uploaded successfully and produced UART diagnostics. C1 through F3 never uploaded; PlatformIO `esptool.py` could not open COM3 (`PermissionError(13)`, `Access is denied`). These are **not** sensor-failure results.
- QMC `0x2C` identified with chip ID `0x80`; MPU `0x68` and BMP `0x76` also responded to scan. MPU auxiliary bypass successfully enabled. QMC suspend, sign `0x29=0x06`, and set/reset/range `0x0B=0x08` were acknowledged; control-register PRE_START_VERIFY passed.
- After writing control register `0x0A`, all three verification reads (ID, C1, C2) returned zero bytes, accompanied by `requestFrom(): i2cWriteReadNonStop returned Error -1` from the Arduino-ESP32 `Wire` library. The write ACK is not proof of a functioning sensor measurement state.
- A0 (50Hz), A1 (100Hz), A2 (200Hz), A3 (200Hz low OSR), A4 (200Hz with recovery), B1 (50Hz Wi-Fi AP delayed), B2 (50Hz scans) each had 0 CSV rows and 0.0Hz measured fresh rate. Wi-Fi was not a necessary condition for failure since A0 already failed with Wi-Fi OFF. Rate-dependent behavior cannot be judged until communication works.
- A0/A1/A2/A3: each 2846 status-read failures and 5 verification failures; A4: 456 status failures and recovery exhaustion; B1 3755, B2 3778 status failures. Lower A4 count is from recovery activity, not evidence of success.
- COM3 access failed starting with C1. A competing monitor, Windows serial-port state, an unclosed handle, or driver behavior are possibilities, **not proven**. v18 adds short retries and abort-on-persistent-busy rather than running 14 more invalid uploads.

## Next diagnostic question

Does the QMC5883P stop responding only through **repeated START**, while STOP+START reads still work, or do other devices also become unreadable after the QMC measurement-mode write? v18's early STOP-vs-repeated-START tests and post-start MPU/QMC probes answer this more efficiently than the full stress suite.

## Limits

This session provides no successful magnetometer XYZ samples and therefore cannot support noise assessment, ellipsoid fitting, full 3x3 soft-iron calibration, frequency-response estimates or yaw accuracy claims. It also does not demonstrate a true power cycle; all recorded tests used warm resets.
