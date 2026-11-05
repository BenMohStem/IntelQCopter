# MPU6050 gyroscope — stationary calibration and precision

## Independent 4 November estimates

30,000 stationary samples, using original calibrated engineering-unit stream:

| Axis | Mean zero-rate bias, deg/s | Within-run SD, deg/s |
|---|---:|---:|
| X | +0.161968974 | 0.047727251 |
| Y | −0.304950151 | 0.044855293 |
| Z | −0.877433074 | 0.043199902 |

In software, with matching units and full-scale configuration, `omega_cal = omega_raw - b_g`. For the attitude estimator, convert degrees/s to radians/s **after or consistently with bias correction**. These numbers are sensor-frame values; apply proper `R_BS` before interpreting as body p/q/r.

## Hardware offset version

FC v2 writes coarse gyro compensation to MPU6050 offset registers, reads back, and samples ~600 stationary readings after that write. The correct residual subtraction is **post-write measured residual**, not original raw bias again. Log original register words, written words, post-write mean and SD to detect wrong LSB scaling or temperature-compensation-bit corruption.

## Precision is not accuracy

Stationary white-noise SD is small, but it is not a complete Allan deviation or temperature bias model. Other 4 November sessions showed between-session biases larger than within-run uncertainty. A large 30k sample count does not cancel temperature drift or systematic offset. A brief stationary check at the actual board temperature remains justified.

## Saturation warning

Fast MAG tumble recorded some gyro samples near ±250°/s limit (~21.7% of samples with at least one axis close to that limit in previous analysis). The owner plans to revisit the full-scale range. Changing the range changes sensitivity and potentially noise; log the new range and redo any required scale verification rather than reusing ±250°/s constants blindly.
