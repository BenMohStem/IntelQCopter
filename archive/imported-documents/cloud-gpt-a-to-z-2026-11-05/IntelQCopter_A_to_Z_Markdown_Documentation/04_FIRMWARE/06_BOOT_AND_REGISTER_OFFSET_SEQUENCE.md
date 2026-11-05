# Boot sequence, register offsets and zero references

## FC v2 boot intent

1. Keep motor GPIOs in INPUT state, disable Wi-Fi, start UART diagnostics.
2. Start I²C on verified GPIOs; detect MPU and establish AUX bypass (`0x6A/0x37`).
3. Initialize QMC5883P (preserved successful profile); initialize BMP280 with factory trims and compensation.
4. Apply coarse MPU gyro/accelerometer **register** offsets; read back to verify writes.
5. Acquire ~600 stationary observations *after* writing offsets. Estimate residual gyro bias and assess stability; reject motion.
6. Initialize quaternion roll/pitch from calibrated gravity (yaw initially arbitrary); check up-axis plausibility.
7. Average fresh stable BMP pressure and set P0, relative zero altitude, Kalman initial state.
8. Enable/reset FIFO only **after** stationary calibration; then start nominal 500 Hz loop.
9. Print boot health/model and preserve motor disabled state.

## Double-correction hazard

If hardware offset removes part of original bias, software must subtract only the remaining offset. `flight_core.hpp` provides `setResidualBias()` and `setHardwareAccelDelta()` to adjust the remaining software state. Because accelerometer cross-axis matrix cannot be written into MPU registers, it remains a per-sample software transform.

## Validation priority

Offset register scaling is partly a source-level *theoretical conversion* in v2 and is not verified on this actual board. Record before/after raw stationary counts, temperature, register readbacks, transformed gravity, gyro SD and any clipping. Wrong register signs or factory compensation bits can make an apparently precise calibration worse.

## During a flight-worthy future boot

A stationary gyro condition must be enforced before arming and stale P0 must be rejected. Current v2 never arms. The controller is a bench core and should not be represented as a completed arming state machine.
