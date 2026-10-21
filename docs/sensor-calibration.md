# Sensor calibration

The routines below are implemented inside `firmware/flight_controller/flight_controller.c`. They operate on measurements supplied by future sensor drivers; they do not communicate with an I²C device. No real coefficients have been collected. A versioned two-slot flash journal now loads validated fits at boot, and the raw-sample integration hook verifies/recalculates gyro bias when stationary samples become available. The six-face calibration UI and physical acquisition remain unfinished. Firmware motor outputs remain disabled. See [flash storage and boot behavior](calibration-flash.md).

Initialize each collector/result with `{0}`. Supply each fresh measurement once; the numerical collectors do not check timestamps. Begin with the sensor's configured counts-to-physical-units conversion and correct device identification. Collector and finish functions reject calls with `disarmed=false`. Rejected fits leave previously accepted coefficients intact; the caller must track that the requested new capture failed. Apply fitted results in the same sensor axes and units used during capture, then rotate into the airframe axes.

## MPU6050 gyro

Warm up the electronics, leave the vehicle stationary, and use `imu_stationary_cal_add` with acceleration in g and angular velocity in degrees/second. At least 500 samples are required. `imu_gyro_cal_finish` rejects high sample variance, a mean acceleration inconsistent with gravity, and implausible gyro bias. The present limits (gyro sample sigma at most 0.1 deg/s, acceleration sample sigma at most 0.02 g, gyro bias at most 5 deg/s per axis) are initial engineering limits requiring characterization on the installed board.

The result stores gyro bias, sample variance, bias-estimate variance (`sample variance / count`), and count. Subtract the accepted bias from all three gyro axes. Stationary variance checks cannot prove that the vehicle was stationary: slow constant rotation can resemble bias. Do this on a stationary support. Gyro scale, axis misalignment, temperature drift, and bias random walk are not calibrated by this procedure.

## MPU6050 accelerometer

Capture six stationary orientations in sensor axes: +X, -X, +Y, -Y, +Z, -Z. The named axis points upward so that its expected specific force is +1 g or -1 g. Feed each pose with `imu_accel_face_add`, with at least 100 samples per pose. `imu_accel_cal_finish` checks orientation, span, and sample variance before deriving each axis's offset and gain:

`offset = (positive_mean + negative_mean) / 2`

`gain = 2 / (positive_mean - negative_mean)`

`corrected_g = (measured_g - offset) * gain`

The fit records approximate offset/gain variances from the two pose means. It corrects three offsets and three scale factors; it does not estimate a full axis-misalignment matrix. Capturing only the level pose cannot separate accelerometer offset from tilt and is deliberately insufficient.

`sensor_sample_apply_imu_calibration` requires both accelerometer and gyro fits, corrects the sample, and marks `imu_calibrated=true`. It rejects a second application to the same sample. `flight_control_step` rejects an IMU sample without this flag. The driver must derive world-up acceleration from calibrated readings and the attitude/frame rotation; the correction function does not invent that acceleration. Synthetic host fixtures explicitly mark their already-calibrated measurements.

## HMC5883L magnetometer

Use `mag_cal_add` while slowly rotating the assembled vehicle through a complete range of three-dimensional orientations, with propulsion off and away from nearby magnetic objects. Pass microtesla values after the sensor's configured sensitivity conversion. `mag_cal_finish` requires at least 300 samples, confirmation that the full rotation was performed, sufficient span, and non-planar covariance coverage. A horizontal or tilted circular sweep alone is rejected.

This is a preliminary min/max hard-iron offset and diagonal scale correction. It does not fit the full soft-iron ellipsoid/misalignment matrix, verify all directional coverage, or compensate current-dependent fields from motors and ESC wiring. It is not yet connected to a heading estimator or yaw control. Validate corrected field magnitude and heading against independent orientations before using it. ST's [magnetometer calibration note](https://www.st.com/resource/en/application_note/an3192-3axis-magnetometer-calibration-and-compensation-stmicroelectronics.pdf) describes the broader hard/soft-iron problem.

## BMP280 pressure and altitude zero

There are two separate steps:

1. Read the 24 factory trim bytes beginning at register 0x88. `bmp280_trim_decode` checks/decodes them; `bmp280_compensate` converts raw 20-bit pressure and temperature with those coefficients. This follows section 3.11.3 of the [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf). Reading device registers is not implemented yet.
2. With the vehicle stationary after warm-up, collect at least 100 compensated pressure samples through `baro_reference_add`. `baro_reference_finish` uses a caller-selected maximum noise sigma to accept a ground-pressure mean. It stores sample variance and the variance of the mean. `baro_relative_height` gives relative height and propagates those variances into a height sigma.

The reference sets the current location to zero; it does not establish altitude above sea level. The standard-atmosphere pressure-height formula is an approximation. Temperature, weather, prop wash, and pressure-port shielding can cause drift or bias that is absent from the reported short-term sigma.

## TOFSense-F2 Mini, pack voltage, and current

`linear_cal_add` collects measured/reference pairs. `linear_cal_finish` fits gain and offset with at least three samples and rejects insufficient input span or a residual sigma above the caller's limit. `linear_cal_apply` performs the correction. Capture references across the intended operating range, using repeated measurements at several distinct points.

- Lidar: use measured fixture-to-target distances and account for sensor mounting offset. Validate more distances and surfaces after fitting. Ground slopes, vehicle tilt, invalid returns, and range limits require separate handling; a gain/offset fit does not solve them.
- Voltage divider: compare the ADC-derived reading with a trusted multimeter at several pack voltages. Keep ADC input limits within the board's electrical range.
- Current: capture zero current and several independently measured loads, defining positive current as discharge. A current sensor is still required for coulomb counting; a voltage divider cannot measure current.

The regression records residual sigma in reference units. It does not include reference-instrument error, nonlinearity outside the fitted range, ADC reference drift, or the battery capacity's uncertainty. These corrections are not yet wired into hardware acquisition or battery monitoring.

## Interpreting uncertainty and saved results

The variance-of-mean estimates assume independent samples. Filtering and correlated sensor output reduce the effective sample count and can make these estimates optimistic. Stationary sample variance is not the same quantity as the process-noise density or bias random-walk term in a Kalman model. The code does not overwrite filter tuning with stationary variances automatically.

Earlier host tests injected known biases, gains, noise and bad captures to check correction math. Their historical logs do not establish calibration accuracy on the drone or verify the new flash layer. Versioned checksummed persistent storage is now implemented and compiled. Physical acquisition, guided capture, withheld-reference validation, and hardware persistence/power-loss checks remain unfinished.
