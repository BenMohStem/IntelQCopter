# MPU6050 accelerometer — full six-face matrix calibration

## Independent candidate

Bias vector, in m/s²:

```text
b_a = [ 1.264655572, -0.059103429, -0.678744839 ]
```

Direct correction matrix `C_a`, dimensionless:

```text
[ 0.999284313  -0.038750762  -0.032967877 ]
[ 0.041351387   0.992793975   0.040449121 ]
[ 0.042481183  -0.040866969   0.987204883 ]
```

Use `a_cal = C_a · (a_SI - b_a)` in sensor coordinates, with the same original ±2 g ADC scaling. The cross-axis entries are not optional if reproducing the fitted solution; a single scale factor cannot implement this 3×3 map.

## Fit statistics and limitation

30,000 samples across six nominal face orientations (5,000 per face). Reanalysis obtained **0.05331 m/s²** component RMS fit residual and **0.05334 m/s²** chronological within-pose holdout. Corrected gravity-magnitude RMS was approximately **0.0240 m/s²**. Leave-one-face-out errors in an earlier audit reached ~0.356 m/s², demonstrating sensitivity to face placement and possible axis misalignment.

## Important interpretation

Small within-pose residuals prove repeatability and the numerical fit, not an exact physical mounting rotation. Six hand-held or manually placed faces may not be perfectly orthogonal; those positioning errors are conflated with accelerometer cross-axis coefficients. For high precision, repeat with an alignment jig or surveyed orientations if available.

## Hardware accel offsets

MPU6050 hardware offset registers can remove a coarse shift but **cannot implement cross-axis calibration**. If hardware offsets are written at startup, adjust the remaining bias based on empirically observed register effect so there is no double correction. Preserve factory temperature compensation bits. Validate corrected norm and direction after all transforms before enabling attitude feedback.
