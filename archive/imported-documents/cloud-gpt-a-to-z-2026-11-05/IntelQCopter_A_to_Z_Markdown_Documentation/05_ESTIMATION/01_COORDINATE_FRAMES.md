# Coordinate frames and physical interpretation

## v2 convention

Mathematics assumes right-handed **FLU** body axes: +X forward, +Y left, +Z up. World also uses +Z up. The unit quaternion `q` maps vectors from body to world. Euler angles are for display and outer small-angle loops, not the core integration state.

## Transform sequence

- `gyro_sensor` -> gyro hardware offset / measured residual -> deg/s -> rad/s -> MPU sensor-to-body rotation -> body p,q,r.
- `accel_sensor` -> m/s² -> software bias and 3×3 matrix -> MPU sensor-to-body rotation -> proper specific force.
- `mag_sensor` -> µT -> hard-iron offset and 3×3 soft-iron matrix -> MAG sensor-to-body rotation -> heading observation.
- LiDAR range frame -> aligned downward ray -> height correction after tilt compensation and reference offset.

## Source-level hazard

`Cal::RbodyImu` and `Cal::RbodyMag` are identity placeholders in `include/flight_core.hpp`. This is **not an asserted physical measurement**. A single swapped sign can turn a nominally stable PID loop into positive feedback.

## Coordinate checks

At rest level with +Z up, specific force should be approximately +g along body Z; manually roll/pitch/yaw and check the derivative sign of each reported Euler axis and body rate. Confirm `R_BS` orthonormality and determinant +1; display-only mirrored axes must never enter the control orientation matrix.
