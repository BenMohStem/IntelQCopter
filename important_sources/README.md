# Read these for the current step

1. [main.cpp: one-axis X-bias lesson](../firmware/main.cpp).
2. [Explanation, expected readings and automatic bench test](../docs/bench-x-bias.md).
3. [Current small-step plan](../docs/PLAN.md).

[Previous raw-recorder checkpoint](../archive/bench-checkpoints/raw-pose1000/README.md) · [Six-pose report](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Tilt report](../measurements/mpu6050/2026-10-25-tilt-validation/report.md).

Supporting references: [MPU configuration](../docs/mpu6050-configuration.md), [historical calibration math](../docs/accelerometer-calibration.md), [recording six poses](../docs/record-18-poses.md), [recording tilts](../docs/record-45-degree-tilts.md), [ROS2/Gazebo commands](../docs/start-ros-gazebo.md).

PC tools are in tools/calibration/. They are not flight code and do not need to be read to follow main.cpp. The raw recorder and old firmware are preserved; no broad controller was imported into this lesson.

[Where 1.312 m/s² comes from: complete calculation report](../docs/bias-x-provenance.html) · [LaTeX Markdown source](../docs/bias-x-provenance.md).
