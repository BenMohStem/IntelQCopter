# QMC5883P magnetometer — independently fitted 3×3 calibration

## Independent candidate from raw 4 Nov rotations

Hard-iron bias in microtesla:

```text
b_m = [ -29.482301006, -18.397983738, -35.474272846 ]
```

Relative full soft-iron correction, symmetric, determinant normalized to 1:

```text
C_m =
[ 0.973366985   0.005036971   0.038277934 ]
[ 0.005036971   0.964534610  -0.017911741 ]
[ 0.038277934  -0.017911741   1.067011358 ]
```

Apply **in the sensor frame**:

\[
m_{cal}= C_m(m_{raw\,µT}-b_m).
\]

The fitted norm is **~40.743 µT** in the relative scaling, with SD **1.2504 µT** (coefficient of variation **3.069%**). Full-matrix held-out magnitude RMS error was ~3.46%, versus ~5.06% for a nonlinear diagonal-only model.

## Independent cross-checks

Two mathematical fits (robust nonlinear and algebraic ellipsoid) agreed on offset to better than ~0.1 µT per axis in the earlier audit. Validation on independent pre-10:09 sessions produced ~3.79% and ~3.46% relative RMS magnitude error. That is evidence the correction generalizes *for those recording conditions*, not proof of accurate heading in a motor-powered environment.

## Original fitting defect

The prior saved `calibrate_sensors.py` attempted an ellipsoid regression admitting a trivial constant solution. It also labeled the inverse-squared ellipsoid equation matrix as though it were the direct correction. These errors produced degenerate coefficients and a nearly collapsed plot. The independent fit did **not** discard 970 legitimate low uncorrected-norm samples simply for being below 20 µT.

## Physical limits

Ellipsoid fitting from rotation gives offset and relative shape but not absolute Earth-field scale, body-frame orientation, magnetic declination, or current-induced disturbances. Use a physically proper `R_BS` with det+1 and test the vector with real motor currents in a controlled environment. Disable yaw magnetic correction if field norm, coherence, freshness or inclination gates fail.
