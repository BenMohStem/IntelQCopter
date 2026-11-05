# 4 November 2026 — successful integrated raw calibration session

## What was actually delivered

User archive: `IntelQCopter_Calibration_20261104.zip`, containing `iq_500hz_20261104_100907.csv`, `calibration_20261104_100907.json`, a MAG image, original `calibrate_sensors.py`, skipped-stages metadata and README. The independent audit reprocessed the raw CSV without copying its calibration coefficients.

## Sample inventory

| Stream/stage | Measured count or reported property | Interpretation |
|---|---|---|
| Total received rows | **98,868** | Combined session, not all static gyro or full-mag training samples |
| UART sampling | ~500 Hz class, 2 ms timestamp targets | Must distinguish timestamp target from timing jitter |
| Gyro rest | 30,000 | ~60 s stationary bias estimation |
| Six-face accelerometer | 30,000 | 5,000 each face; assumes nominal orthogonal poses |
| MAG rotation training | 14,000 | 2,000 each yaw/pitch/roll plus 8,000 3D tumble |
| Fresh MAG flags over entire raw session | 38,652 (prior review) | ~196 Hz over recorded windows; not 500 Hz independent observations |
| BMP rest | 1,000 accepted | Pressure SD 0.640 Pa, altitude SD 0.0548 m |
| LiDAR | 0 accepted | Not measured, postponed |

## Important actual settings

Despite some ZIP folder names saying “100 Hz”, the successful firmware snapshot configured **QMC Control1 `0x3D`** (nominal 200 Hz) and observed about **195 Hz fresh**. The source used the QMC5883P chip and 921600-baud CH343 binary UART. A separate v13 firmware had earlier worked with `0xC9` (nominal 100 Hz). These are distinct experiments.

## Main conclusions

- Real integrated MPU/MAG/BMP acquisition was demonstrated during the session.
- Independent raw-register conversions agreed with reported engineering units within serialization/rounding precision.
- The original saved MAG ellipsoid fitter had a defective/trivial solution construction; independent refit gave candidate 3×3 parameters.
- Sensor alignment to aircraft axes, thermal drift and operating with motors running were not validated.
- No claim of flight-controller or ESC functionality follows from this calibration capture.
