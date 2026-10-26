# Testing progress: 26 October

## Completed

| Work | Engine/evidence | Result |
|---|---|---|
| 1000 Hz acquisition/state-processing implementation | ESP32-S3 Arduino cross compilation; source fingerprint in binary | Builds; no upload yet |
| Calibration and FIFO protocol regression | Python, 41 unique tests | Pass |
| Actual firmware normal/fault execution | MSVC x64, fake Arduino/I2C/UART peripherals, 14 cases | Pass; not ESP32 emulation or timing proof |
| Native complete payload verification | Three host-source streams of 20000 model samples each | Sequence, checksum and 200 diagnostics per stream verified; synthetic data |
| Additional fixed-support recording | Actual COM3 V2, 300 seconds, 25001 selected samples, 83.33 Hz | Original hashes/metadata/samples/math/timing verified |

No calibration refit, motor pulse, arming or firmware upload was performed. Original successful and failed hardware/compiler/model logs remain local; only reviewed summaries are published.

## Hardware result

The additional five-minute run had full-run sensor gyro means [-0.000874, -0.004471, +0.008122] degrees/s and sample standard deviations [0.093142, 0.090656, 0.090477] degrees/s. PC gravity tilt means were roll +0.03418 degrees, pitch +1.90448 degrees. No independent level/stillness reference was instrumented; these are observations, not absolute calibration errors. Chip temperature ranged 27.259-28.483 C. No temperature compensation was fitted.

[Actual data, covariance derivation and graph](../measurements/mpu6050/2026-10-26-extended-stationary-check/report.md).

## 1 kHz implementation

MPU FIFO drives every calibrated state update. Compact raw integer records replace long per-sample float output. Diagnostic output is 10 Hz, offset checks are staggered, and fast math uses single-precision constants. Overflow, timeout and I2C/calibration failures stop acquisition. Sample sequence, FNV checksum and sparse MCU calculation checks preserve evidence. FIFO occupancy, maximum service gap, calculation time and full handler time are separate measurements.

[Source and test guide](fifo1000-stage.md) ? [Preparation verification](../measurements/mpu6050/2026-10-26-fifo1000-preparation/verification.json).

## Gate still pending

The physically connected board remains on known V2 source at commit 3799249; the new FIFO1000_V1 build has not run on it. The user previously reserved uploads. A separate upload-choice question was asked and remains unanswered. Consequently there is no physical 1 kHz capture or hard 1 ms deadline result to report.

When explicitly authorized to upload, or after the user uploads it, the next operation is the prepared real 20000-sample FIFO capture. Hardware evidence must pass before adding attitude fusion/PIDs. No further low-rate repetition substitutes for that gate. BMP280 remains deferred and physical arming remains disabled.
