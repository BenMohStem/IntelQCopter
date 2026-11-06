# ESP32-S3 DSH flight-controller review candidate

**Status: experimental source review only. Not flight-ready and not hardware-validated.**

This folder preserves the DSH rewrite that had replaced the active calibration
firmware entry point. Its source was copied before the calibration project was
restored; the copied pre-review `src/main.cpp` SHA-256 was
`58DA6D40A34931FC19822AE5E904C7B2522DE861D28A5C3223A3E3F3E1BDD38A`.

## Scoped corrections in this candidate

- Restores the calibration GUI's fixed 52-byte version-3 packet layout and raw
  sensor field order. The previous DSH telemetry builder attempted to serialize
  more than 52 bytes, silently truncating fields before the CRC.
- Removes text logging from the binary sensor UART.
- Reads the QMC5883P register pointer with STOP then START, matching the
  transaction used by the November 4 captured firmware.
- Converts raw QMC counts with the GUI's current ±8 G QMC5883P scale
  (37.5 counts/µT) before calibration; the DSH rewrite's `raw * 0.29` conversion
  was inconsistent with the GUI's unit convention.
- Runs QMC transactions at 100 kHz and returns the shared I²C bus to the
  configured 400 kHz rate for MPU/BMP operations. The 400 kHz QMC path remains
  unverified and is not claimed here.
- Clears the MAG `fresh` flag on every loop and only sets it after DRDY plus a
  successful XYZ read. A cached sample expires after the configured stale age.

The active calibration application remains at
`hardware/bench_calibration/esp32s3_qmc5883p_100hz`; this candidate is isolated
so the calibration launcher does not compile flight-controller code.

## Known blockers

- No build, host test, board test, or flash has been performed for this copy.
- The DSH rewrite still has significant issues outside this scoped patch,
  including the LiDAR parser/baud mismatch with TOFSense-F2 Mini, questionable
  BMP conversion freshness and duplicate reads, MPU FIFO/temp configuration
  conflicts, attitude-estimator math errors, and unsafe altitude-reference
  behavior. The virtual motor mixer is not an actuator interface.
- GPIO 11/12/13/14 remain inputs. Physical motor arming and ESC output are not
  enabled or authorized. The USB Xbox path is not a verified flight receiver.
- The code has not been compared against a direct hardware register dump or
  captured UART session. The November 4 QMC rate and this source's behavior are
  separate evidence claims.

Do not upload this candidate for flight. Keep the known calibration firmware
and the evidence captures as the rollback reference.
