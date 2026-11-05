# BMP280 — pressure altitude, filter and real output rate

BMP280 is a pressure/temperature sensor (typically `0x76`) behind MPU6050 XDA/XCL bypass on this PCB. Raw ADC pressure and temperature require device-specific factory compensation coefficients. FC v2 caches those trims and reads a single burst of raw data, then uses the Bosch integer compensation routines.

## Actual v2 profile

- Normal measurement mode.
- Temperature oversampling ×1; pressure oversampling ×1.
- IIR filter ×4.
- Standby 0.5 ms.
- Typical **166.7 Hz** effective pressure output according to Bosch's mode table, not 200 independent new conversions per second.
- Firmware polls approximately once every 6 ms, ignores duplicated raw conversion values and records freshness separately.

## Why not simply maximize smoothing?

Pressure oversampling and IIR reduce high-frequency noise but increase conversion time, group delay and transient response time. The vertical estimator uses inertial acceleration for prediction and slow pressure for drift correction; lag cannot be wished away by “more Kalman” or higher polling frequency. Test actual ascent/step response before choosing higher IIR.

## Relative height

Pressure altitude relative to a boot baseline P0 is estimated by:

\[
h = 44330\left[1-\left(\frac{P}{P_0}\right)^{0.190294958}\right].
\]

This is an atmospheric approximation and assumes the baseline reference remains meaningful. It does not correct propwash, local airflow, thermal drift, weather variations or aircraft-specific pressure-port placement.

## Calibration and re-zero

At boot, collect stable pressure measurements, compute P0 and its statistical spread, then establish z=0. v2 also supports D-pad Up+LB+RB disarmed-only re-zero; never allow an in-flight uncommanded baseline jump. Air pressure variation with temperature is not direct evidence of true vertical motion.

Reference: Bosch BMP280 datasheet, <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf>.
