[Current real-data report and graphs](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Updated small-step plan](../docs/PLAN.md).

# Current task: review calibration, then validate tilted poses

[Upload once and automatically record all 18 poses](../docs/record-18-poses.md). [Active one-file firmware](../firmware/main.cpp).

## Earlier calibration references

1. [main.cpp — current tiny programme](../firmware/main.cpp)
2. [Current lesson explanation](../docs/accelerometer-calibration.md)
3. [The small-step plan](../docs/PLAN.md)

The files in tools/ manage builds and history; they are not flight code and need not be studied yet. Older diagnostics are in the archive and Git history, not the current firmware.

[Actual offline calibration/build results](../docs/calibration-test-results.md) - numerical/compile evidence only, not a hardware pass.

[MPU6050 datasheet and editable configuration guide](../docs/mpu6050-configuration.md).

[Measure standard deviation for Kalman filtering](../docs/imu-noise.md).

[Actual live sensor recording analysis and precision refinement](../docs/precision-2026-10-25.md).

[Next: calibration valid across drone orientations](../docs/multi-pose-calibration.md).

[Run the four 45-degree tilt measurements](../docs/record-45-degree-tilts.md).

[Completed 45-degree tilt review and graph](../measurements/mpu6050/2026-10-25-tilt-validation/report.md): original labels clarified by the operator, candidate unchanged; reference accuracy about ±1°.
