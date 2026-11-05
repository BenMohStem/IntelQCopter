# Independent sensor-calibration validation experiments

## Gyro

Repeat bias at cold/warm conditions, measure time-dependent bias, and estimate Allan variance when enough stationary records exist. Read hardware offset registers and verify residual means. Exclude obvious movement using accelerometer norm and gyro variance. Do not assume first few boot samples represent fully warmed sensor.

## Accelerometer

Use precisely oriented six faces if possible and independent known tilts. Compare raw 3D values and corrected magnitude `|a|≈g` for all six poses. Cross-axis matrix must be tested against orientations not used to fit; otherwise fit may absorb placement error. Validate after any ±2g range change.

## Magnetometer

Collect separate full 3D rotations after remounting, compare corrected norm distribution and held-out angle/heading, and verify direction mapping. Compare motor power off versus controlled powered conditions once safety architecture supports guarded motor tests. Check field magnitude trend for EMI.

## Barometer

Compare pressure-derived relative height at actual ruler-controlled positions, in still air, then near gentle flow/temperature variation without propulsion. Log sample age, IIR response and zero-reference drift. Confirm the disarmed D-pad reset works only once per chord.

## LiDAR

When connected, known 25/50/100/200 cm distances, varying reflectivity and tilt; measure status/variance and fusion innovation. No accepted 4 Nov LiDAR samples means no claim of successful calibration today.
