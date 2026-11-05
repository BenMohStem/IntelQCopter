# Hardware test plan — do not attach propellers

1. Inspect actual pinout and power voltages; ground, I2C pull-ups, UART levels and layout. Check MPU6050 and BMP280 path through XDA/XCL bypass.
2. On Windows run `tools/run_fc.ps1 -UploadFirmware` with ESP powered by USB only, ESC signal wires disconnected. Cold-boot if CH343 reset state is inconsistent.
3. Confirm UART 921600 and `FC_BOOT accel_hw=1 gyro_hw=1 fifo=1` as well as stationary gyro residual and quaternion pitch/roll. If accel hw offset fails, record raw registers and compare with the original calibration; don't assume its theoretical effect.
4. Check `FC` fields: `hz` near 500, increasing `imu`, `mag`, `bmp`, `fifo`; `FC_TIMING imu_max_us/mag_max_us/bmp_max_us/loop_max_us`, `qmc_slow`, `fifo_lost`, `fifo_over`. Every reported frequency is a *commanded or observed* quantity, not a hardware guarantee. No 20ms blocks or rising FIFO overflows.
5. Move board slowly about verified body axes; roll, pitch and yaw must have expected signs and valid gravity. Do not conflate sensor and body coordinates.
6. Check barometer at ground (z≈0), then at measured ~0.5, 1, 2m elevations, no motors. Check filter sigma, NIS rejections and long-term drift. D-pad Up+LB+RB may re-zero only when always-disarmed.
7. Connect Xbox to Windows, run `--inspect`, check all axis IDs, then `run_fc.ps1 -Xbox` and view virtual mixer (ESC GPIO still INPUT). Disconnect to force stale watchdog.
8. Separately identify LiDAR RX wiring, default UART 921600 50Hz, 16-byte frame, checksum. Verify reported distances with a ruler and tilt angles before enabling `IQ_LIDAR_ENABLE`.
9. Weigh each aircraft part; validate mass, COM and inertia for the actual assembled configuration. Do not include N150 profile unless it is actually mounted.
10. Measure rotor/ESC thrust curve and actuator delay under restrained, guarded, calibrated load; only then retune PIDs and assess motor safe-start/kill architecture. No free flight without independent hardware failsafe and verified propulsion safety.
