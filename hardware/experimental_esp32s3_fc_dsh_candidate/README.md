# ESP32-S3 DSH flight-controller review candidate

**Status: static review only. Not built, flashed, or hardware-validated. Not
flight-ready.** This isolated candidate is separate from the active ESP32
calibration firmware at
`hardware/bench_calibration/esp32s3_qmc5883p_100hz`.

## Candidate settings

- Shared I²C target: 400 kHz for MPU6050, QMC5883P and BMP280.
- MPU6050: FIFO with temperature + accel + gyro, ±8 g, ±500 °/s,
  `CONFIG.DLPF_CFG=1`, `SMPLRT_DIV=1` (nominal 500 Hz).
- QMC5883P: address `0x2C`, 200 Hz `CTRL1=0x3D`, `CTRL2=0x08`,
  `XYZ_SIGN=0x06`; data are fresh only after DRDY and a successful read.
  The last calibrated value is reused by the attitude update between new 200 Hz
  conversions and rejected after 25 ms without a successful read.
- BMP280: normal mode, pressure/temperature ×1, IIR ×2 and library standby
  enum `STANDBY_MS_1`; host polling is scheduled at 6 ms and does not wait for
  conversion completion. Actual fresh rate and pressure-read age are unknown.
- LiDAR remains disabled and is not read or fused for the current competition
  calibration/control phase.
- Telemetry remains the calibration-compatible 52-byte version-3 frame. It
  contains no per-sensor sample-age or service-time fields, so it cannot support
  the requested per-sensor latency plots or a verified `<2 ms` claim.

These are source constants and intended behavior, not verified register values
or measured rates on the repaired PCB. Historical 400 kHz QMC runs have mixed
results; this candidate does not establish that the shared bus is reliable.

## Static corrections in this review

- Corrected the accelerometer path to retain gravity in the body frame for the
  attitude estimator, rotate specific force into world vertical, then subtract
  gravity for vertical acceleration.
- Integrates each fresh FIFO frame once, uses the measured loop interval for
  control timing and sample timestamps for IMU integration, and does not mark a
  cached MPU frame as fresh.
- Removed automatic barometer zeroing on disarm/startup. Barometric height now
  uses the saved pressure reference only; missing calibration keeps vertical
  control unavailable.
- Limits BMP compensation to completed, newly-read observations and marks the
  observation time after pressure/temperature compensation. BMP initialization
  is independent of MPU discovery.
- Initializes MAG calibration output deterministically and applies a new field
  before the attitude update that consumes it.
- Corrected the BMP library standby enum name and moved the command CRC
  declaration into the namespace used by its implementation.
- Corrected the optional 22-byte command parser to search the `51 43` hand-
  controller frame, validate CRC/ranges/sequences and resynchronize after a bad
  frame. The receiver remains compiled off because there is no verified flight
  adapter or Xbox/LoRa link.
- Sensor/calibration/PID/hover prerequisites now latch a failsafe and disarm the
  software controller. Physical GPIO motor outputs remain absent; configured
  motor pins are inputs.

## Remaining blockers

- No calibration file is loaded by this candidate: accel, gyro, MAG and baro
  calibration-valid flags remain false. PID gains and hover collective also
  remain unset. The controller therefore remains in failsafe with zero mixer
  output.
- The Xbox packet parser is only a protocol parser, not a C++ Xbox controller
  library integration or verified receiver. A Windows USB/serial connection is
  not a flight receiver.
- BMP compensation uses the Adafruit driver separately from the raw burst read;
  those library transactions are not included in `I2cTransactionManager` timing.
  This source cannot substantiate sensor service latency.
- No hardware register readback, sensor-age distribution, timing distribution,
  full-load 400 kHz test, motor output adapter, or flight-control validation is
  available for this candidate.

Do not upload this source for flight or enable physical arming. Keep the active
calibration firmware and its captures as the recovery reference. The DSH working
tree snapshot taken before this review is preserved at
`archive/review-candidates/2026-11-06/flight-controller-working-tree-20261106.c`.
