# MPU6050 + BMP280-only FC — build evidence, 22 October 2026

**Actual engine:** PlatformIO espressif32@7.1.3 and its Arduino ESP32 cross-compiler. These are real compile/link logs, not captured hardware telemetry or a flight simulation.

- [Initial optional-companion compile](esp32-s3-build.log): successful; retained before final small source cleanup.
- [Final builds](final-builds.log): both esp32-s3 and esp32-s3-standalone successful.
- [Final source hashes and scope](summary.json).

The canonical FC is still one flight_controller.cpp. Only MPU6050 and BMP280 are required physical sensors. Detection tries0x68/0x69 and0x76/0x77 respectively, validates MPU range/filter/rate readback, and logs the result. Missing lidar/magnetometer does not block operation; missing required sensors still produces a fault. Original estimators, gains, control loops, calibration/NVS and stale-input checks are retained. Physical motor outputs remain disabled.

No hardware was flashed. Actual I2C operation, sensor axes, timing and flight behavior remain unmeasured. No new emulator or Gazebo execution was performed. Prior Gazebo evidence uses synthetic sensors and ideal localization and cannot validate these physical I2C paths.

[Wiring and calibration guide](../../../docs/esp32-mpu6050-bmp280.md).
