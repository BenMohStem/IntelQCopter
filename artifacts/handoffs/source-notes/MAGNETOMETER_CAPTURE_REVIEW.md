# Real QMC5883P capture review — 2 November 2026

This review was calculated from the user-provided `captures.7z`, not from a synthetic test. The full 36 MB CSV is **not** copied into this project; it remains in the user's original archive. This is a snapshot, not a flight-validated calibration.

## Summary

- 112,377 recorded CSV rows in the main file; 11,000 **fresh, phase-accepted** magnetic readings (2,000 yaw, 2,000 pitch, 2,000 roll, 5,000 3D tumble). The other two CSV parts were headers only.
- Magnetometer: ID-verified QMC5883P, reported in sensor-chip XYZ (not the GUI's X/Y display swap).
- **Raw range (µT):** X −91.57 to +19.52; Y −73.76 to +32.48; Z −57.25 to +2.13.
- Existing estimated hard-iron center (µT): **[−36.30, −22.98, −27.83]**. These are from the fitted model, not an independently measured reference field.
- Existing approximate fitted field radius: **45.46 µT** (not independently confirmed by geomagnetic reference).
- Relative radial scatter: **12.95% before ellipsoid correction**, **1.15% after correction**, computed on the **same data used to fit** the model. This is in-sample fit quality, not independent heading validation.
- The corrected sample directions reach near both positive and negative poles on all three axes. However, direction density is uneven: positive-X 34%, positive-Y 75%, positive-Z 33%.
- Sphere coverage using 128 nearly uniform direction targets: about **87.5%** have a measurement within **18°**. All eight octants were sampled, but the least-populated corrected octant contains fewer than 1% of readings.
- Measured within-stage MPU logging is ~490 Hz, not exactly 500 Hz, despite the configured 500 Hz target. Magnetic fresh-sample counts are around 94 per second during the dynamic stages; verify ODR and new-data status behavior before concluding true independent sample rate.
- The **accelerometer six-face diagonal fit** has ~**0.60 m/s²** RMS residual across the six nominal poses. This suggests the freehand poses and/or offsets need independent checking. The experimental full 3×3 fit is not automatically preferable because slight pose errors can be misfit as cross-axis error.

## Practical interpretation

The compass calibration is promising but **not yet validated**. The biggest improvement is NOT holding your wrist perfectly around a fixed mathematical center; it is covering more orientation directions while keeping the magnetic environment stationary. Hold the whole assembly at a sensible distance from steel tables, power bricks, speakers, screws, vehicles, and high-current wires. Rotate slowly through diagonals, upside-down orientations and different headings. Move the sensor in space only a little; exact center-of-rotation is not required in a uniform field. Avoid changing the position of nearby magnetic objects during a run.

If you want better proof of a calibration, save one complete magnetic run for fitting and record a **separate, withheld** 3D tumbling run for validation. Compare corrected field magnitude scatter and yaw-heading repeatability on the withheld run, with the sensor mounted in its final arrangement. The current data cannot determine the exact magnetometer-to-drone rotation; the display-only X/Y swap has determinant −1 and cannot be a physical rotation matrix.

The QMC5883P's earlier `OVERFLOW status=0x1B` came from a different text test/configuration. Overflow samples must not enter calibration. The CSV reviewed here contains data marked fresh and valid by the firmware, but does not include an independent raw sensor status byte to re-audit every conversion.

## Graphic

`MAGNETOMETER_CAPTURE_REVIEW.png` shows the real 3D raw cloud and the fitted correction, colored by phase, with equal scales and numeric 3D grids. The coordinate axes are **sensor-chip axes**. The displayed arrows in the graph indicate increasing X/Y/Z directions, not verified aircraft mounting orientation.
