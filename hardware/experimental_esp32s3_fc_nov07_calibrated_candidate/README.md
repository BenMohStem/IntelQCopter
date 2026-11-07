# IntelQCopter Nov 7 calibrated ESP32-S3 candidate

This is an isolated copy of the DSH ESP32-S3 review candidate with the Nov 7
completed-session calibration applied to its software estimator. The original
DSH candidate and the active Wi-Fi work remain untouched.

## Status

- Research candidate only. It has not been built, flashed, or checked on the
  repaired PCB.
- Physical motor output is disabled; motor pins stay INPUT.
- The mathematical motor mixer keeps the configured 10% idle floor, 35%
  normal ceiling, and 50% boost ceiling. These are software command fractions.
- Command input is disabled.
- Barometric altitude remains invalid until a stationary launch pressure
  reference is captured. Hover collective remains unset, so control stays in
  failsafe.
- Magnetometer correction is computed in chip coordinates. Heading fusion stays
  disabled until the chip-to-body rotation and handedness are verified.
- LiDAR data and use are excluded.
- Wi-Fi is disabled in this copy. DSH's Wi-Fi implementation remains separate.

## Source sessions

The fit uses only these three sessions, all marked complete with no missing
calibration stages:

- calibration_20261107_021542_146786_complete.csv
- calibration_20261107_024540_453344_complete.csv
- calibration_20261107_032549_896941_complete.csv

Project data path:
measurements/calibration/esp32s3/sessions/complete/

There are 295,000 accepted records total. File-name times are Lagos local;
session_started_utc in the CSV is one hour earlier.

## Sensor calibration loaded by this candidate

MPU6050 settings are ±8 g and ±500 deg/s. The code converts using 4096 counts/g
and 65.5 counts/(deg/s), applies the Nov 7 accelerometer matrix and bias, and
subtracts the robust pooled stationary gyro bias. At initialization, it checks
that the MPU user gyro offset registers read zero after reset and verifies the
gyro/accelerometer full-scale bits. A failed readback prevents MPU startup.

The accelerometer correction is:
a_cal = C * (raw_counts * 9.80665 / 4096 - bias)

Bias in m/s^2:
[+0.25928109, -0.08780301, -0.89685228]

Correction matrix:
[[+0.99463969, -0.04280700, +0.04798329],
 [+0.04255186, +0.99493002, +0.02387155],
 [-0.05126050, -0.03758379, +0.98640514]]

The six-face fit residual is 0.0061374 g RMS (0.06019 m/s^2). The fit is still
in the capture sensor frame; physical body-axis mapping and placement need
confirmation.

Gyro bias in deg/s, calculated from 89,409 robustly accepted REST records:
[-11.04461742, +6.99774234, -1.10236200]

The QMC5883P conversion assumes ±8 G, or 37.5 counts per microtesla. The
candidate verifies the configured QMC range, ODR register, and sign register
readbacks. The Nov 7 hard/soft-iron correction is applied to fresh MAG samples.
The captures measured about 166.8 fresh readings/s, so the configured 200 Hz
ODR is nominal rather than a measured 200 Hz stream.

Hard-iron offset in microtesla:
[-7.34180573, -13.99272901, -22.84747418]

Soft-iron matrix:
[[+0.97174713, -0.01740207, +0.03634599],
 [-0.01740207, +0.96045521, -0.02183163],
 [+0.03634599, -0.02183163, +1.07361879]]

The field-magnitude fit does not establish compass heading accuracy or motor
current immunity.

BMP280 records use pascals. Stationary pressure SD was 3.01-3.29 Pa, equivalent
to approximately 0.25-0.28 m barometric height noise. The candidate stores a
conservative pressure variance of 10.81 Pa^2, but leaves the pressure reference
invalid and altitude control unavailable. It does not use 101325 Pa as a
takeoff reference.

## PID values

The report did not derive flight gains from the calibration records. The
candidate carries forward the earlier Nov 6 model-only PID set for software
review; these are not flight-identified gains. The rate-loop model outputs
torque, and the torque authority used to convert torque into mixer demand is
assumed.

| Loop | Kp | Ki | Kd | Status |
|---|---:|---:|---:|---|
| Roll angle | 0.85 | 0.012 | 0.008 | Carried-forward simulation value |
| Pitch angle | 0.85 | 0.012 | 0.008 | Carried-forward simulation value |
| Roll rate torque | 0.0723794083 | 0.0342335039 | 0.0000440145 | Model-only, N m output |
| Pitch rate torque | 0.0768854934 | 0.0363647604 | 0.0000467547 | Model-only, N m output |
| Yaw rate torque | 0.0709297215 | 0.0191701950 | 0.0000575106 | Model-only, N m output |
| Altitude position | 0.35 | 0.003 | 0.010 | Model-only; altitude gate is closed |
| Vertical speed | 0.75 | 0.040 | 0.018 | Model-only; altitude gate is closed |

The Nov 7 static calibration data cannot identify mass, inertia, thrust curves,
ESC/motor response, or motor torque authority. Do not describe these values as
tuned or flight-validated.

## Build boundary

This standalone PlatformIO project is configured for an ESP32-S3 DevKitC-1.
Build command from the project root:

    pio run -d hardware/experimental_esp32s3_fc_nov07_calibrated_candidate

Only a source review has been done here. Do not upload it as a flight image.
