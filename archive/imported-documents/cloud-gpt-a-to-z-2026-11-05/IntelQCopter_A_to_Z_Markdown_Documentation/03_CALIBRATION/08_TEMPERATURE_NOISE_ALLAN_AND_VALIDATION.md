# Calibration error budget, temperature effects and better validation

## Error hierarchy

1. Random sensor white noise (measurable from stationary data).
2. Temporally correlated noise, vibration and aliasing (not reduced as 1/sqrt(N) when samples are correlated).
3. Bias shift with sensor temperature, regulator heat, startup time and PCB strain.
4. Accelerometer face/alignment error, magnetometer orientation and hard/soft iron drift.
5. Electrical interference, motor current, pressure flow and time-alignment errors.
6. Quantization, ADC full-scale saturation and uncompensated manufacturer trim.

## Data-collection improvements

Record sensor temperature and elapsed time alongside bias. Repeat stationary gyro sessions after warm-up and after power cycles. Use Allan deviation or bias stability curves to separate white noise, bias instability and random walk, with clear observation-window assumptions. Collect held-out six-face and independent MAG rotation sessions; do not train and validate on the exact same samples.

## How to report uncertainty

State whether intervals represent: within-session SD, bootstrap interval over blocks, between-session variation, setup error estimate or assumed Monte Carlo prior. The independent gyro 30k bias is estimated with high numerical precision, but that precision should not be reported as temperature-invariant physical accuracy.

## Trigger for recalibration

Change of sensor mounting, full-scale ranges, magnetometer distance to power conductors, firmware filter settings, large persistent innovation residuals, changed ground-frame orientation or shock/damage. Make versioned calibration files with date, sensors, config registers, firmware hash and raw-capture checksum.
