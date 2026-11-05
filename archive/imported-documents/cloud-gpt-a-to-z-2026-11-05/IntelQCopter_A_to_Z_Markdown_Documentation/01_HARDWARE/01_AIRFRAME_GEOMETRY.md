# Airframe geometry and coordinate conventions

The flight project is described as an F450-class X quadcopter with two red and two white arms. “450” usually designates nominal motor-to-motor diagonal dimension; the actual span, motor positions, arm height and board mounts have not been measured for these files. Some older descriptions called the vehicle “5-inch”; do not mix those dimensions with the F450 mass model.

## Working frame convention

- Body X forward, Y left, Z up (FLU), right-handed.
- World Z up in v2 estimator/control model.
- Sensor-frame raw XYZ must be rotated into body coordinates using a **determinant +1** orthogonal rotation matrix.
- Motor positions used by the geometric model are at approximate X/Y components ±0.1591 m for 0.450 m diagonal span; positions should be remeasured on the actual assembly.
- The mass model reports COM relative to its declared origin near the lower PCB; this is *not automatically* the IMU origin or a ROS base_link origin.

## Physical placement known from project description

3S 4200 mAh pack mounted on/above lower center PCB; ESP32-S3 custom sensor PCB approximately 75 × 100 mm; ESCs about midway along each arm, positioned beneath; four orange A2212/10T-class motors at arm ends. Downward TOFSense F2 Mini around center of lower board. Optional stripped Samsung N150 board intended near center, with horizontal COM aligned to frame center.

## What to measure

Measure motor center coordinates, diagonal, horizontal COM, vertical distances of battery/sensors/ESCs, propeller clearance, and the exact LiDAR optical-center-to-reference offset. Photograph and label nose direction, motor 1–4 order and sensor XYZ arrows. Once the real dimensions exist, update the physics model and URDF; do not carry zero off-diagonal inertia as a fact if battery or wiring is asymmetric.
