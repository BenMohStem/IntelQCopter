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
  Calibrated chip-frame values are retained for telemetry; heading fusion is
  disabled until the chip-to-body rotation and handedness are verified.
- BMP280: normal mode, pressure/temperature ×1, IIR ×2 and library standby
  enum `STANDBY_MS_1`; host polling is scheduled at 6 ms and does not wait for
  conversion completion. Actual fresh rate and pressure-read age are unknown.
- LiDAR remains disabled and is not read or fused for the current competition
  calibration/control phase.
- Motor-command policy in this candidate: 10% armed-idle floor, 35% final
  per-motor normal ceiling, and 50% final per-motor boost ceiling. These are
  normalized software command fractions, not measured RPM or thrust limits.
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
- PID readiness now rejects non-finite or out-of-range gains and limits before
  the controller can leave failsafe. The software arm gesture requires holding
  LB + RB + Y together continuously for 500 ms; this parser is still disabled
  and is not a flight input path.
- The arm transition no longer invalidates the attitude and vertical estimators
  immediately before the control prerequisite check; PID and mixer state still
  reset when the software arm state changes to armed.

## Remaining blockers

- The Nov 6 accel, gyro and MAG candidate values are now loaded into the
  isolated software estimator path, and all seven simulation PID candidates
  are configured. This is source-level activation only; the candidate has not
  been built or checked against live sensor axes/registers.
- Barometer calibration remains invalid, hover collective remains unset, and
  the command receiver is disabled. These gates keep `computeControl()` in
  failsafe. All motor pins are inputs and there is no ESC output driver.
- The gyro candidate may double-subtract an unknown MPU hardware offset. The
  accel fit may contain manual pose/chip alignment error. MAG hard/soft-iron
  correction is applied in chip coordinates, while MAG heading fusion remains
  disabled until chip-to-body axes and handedness are verified. BMP pressure
  units and a portable ground-pressure reference remain unresolved.
- The Xbox packet parser is only a protocol parser, not a C++ Xbox controller
  library integration or verified receiver. A Windows USB/serial connection is
  not a flight receiver.
- BMP compensation uses the Adafruit driver separately from the raw burst read;
  those library transactions are not included in `I2cTransactionManager` timing.
  This source cannot substantiate sensor service latency.
- No hardware register readback, sensor-age distribution, timing distribution,
  full-load 400 kHz test, motor output adapter, or flight-control validation is
  available for this candidate.

## Nov 6 recalibration/PID audit ingestion

The three-session analysis is represented in
`src/nov06_recalibration_candidates.hpp` as typed, compiled-in research data.
`main.cpp` loads the accel bias/matrix, gyro bias, MAG hard/soft-iron fit and all
seven model PID candidates into the isolated software estimator/controller.
The capture firmware was not changed.

The analysis confirms the assumed raw scales already present here: 4096
counts/g, 65.5 counts/(deg/s), and 37.5 counts/µT. The source applies these
candidate fits to the software samples, with the assumptions still attached:
the large gyro rest bias may conflict with MPU hardware offsets; the accel fit
is confounded with face placement and chip mounting; and the MAG fit is in chip
coordinates. The MAG field correction is calculated, but heading correction
is gated off until body axes and handedness are verified. BMP calibration
remains invalid because the CSV pressure unit and ground reference are not
verified.

All seven flight-loop gains are simulation-only but are now installed in the
software controller path. The controller converts its angle/rate signals to
radians and radians/second, applies a 20 Hz derivative low-pass, and converts
inner-loop torque outputs from N·m to normalized mixer demand using the audit's
assumed roll/pitch/yaw torque authorities. Those authorities, outer-loop gains,
thrust response and motor lag are unmeasured; this conversion makes the units
consistent, not the actuator model validated. The research motor-acceleration
PID remains disabled because this build has no per-motor RPM telemetry; IMU
acceleration is not rotor acceleration.

Software output limits used for this candidate are ±0.749933 N·m for roll and
pitch rate, ±0.070808 N·m for yaw rate, ±π rad/s for roll/pitch angle-loop rate
requests, ±2 m/s for altitude-loop vertical-speed requests, and ±5 m/s² for
vertical-speed-loop acceleration. Integrator state is clamped to ±0.5 in each
loop's integrated-error units. The inner-loop torque limits come from an
assumption-based model; outer-loop limits and integrator clamps are software
bounds, not flight-identified values. Exact gains are listed in the candidate
header.

The inherited linear-thrust model has a median hover fraction near 45% and a
95% interval near 30–89%; the user-selected 35% normal motor ceiling may be
below hover demand. The model is assumption-based, so this is a recorded risk,
not a reason to raise the cap. No propeller-powered test or motor-output
enablement is authorized by this code change.

Do not upload this source for flight or enable physical arming. Keep the active
calibration firmware and its captures as the recovery reference. The DSH working
tree snapshot taken before this review is preserved at
`archive/review-candidates/2026-11-06/flight-controller-working-tree-20261106.c`.
