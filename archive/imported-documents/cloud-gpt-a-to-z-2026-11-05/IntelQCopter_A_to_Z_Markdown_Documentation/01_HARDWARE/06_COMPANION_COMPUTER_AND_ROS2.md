# Optional companion computer, ROS2, vision and autonomy

The project contemplates removing the screen and battery from a Samsung N150-class portable computer and mounting its remaining electronics near the airframe center for ROS2, a camera and YOLO processing. This is a **second proposed configuration**; the successful 4 Nov sensor calibration is not evidence of integrated ROS2 or YOLO flight.

## Resource and safety constraints

An older single/dual-core Atom-class computer with approximately 1 GB RAM (as discussed in prior planning) cannot be assumed to run contemporary full ROS2 + PyTorch inference with low, deterministic latency. A minimal headless Linux configuration, lightweight inference, measured memory use and resource-limited processes may help, but the companion PC must never own the primary 500 Hz stabilization timing. A loss of ROS2, USB or camera must not keep a motor armed.

## Intended separation

- ESP32-S3: IMU acquisition, calibration, attitude/altitude estimation, validated motor control, failsafes.
- Companion computer: high-latency mission planning, object detection, logging, mapping and optional setpoint suggestions.
- Interface: versioned CRC-protected UART or another measured link, with timestamps and command authority rules.

## Specific study-document trap

The Claude volume mentions an HMC5883L in parts of its hardware overview; the measured board has QMC5883P. Similarly, avoid generic RPLidar `LaserScan` drivers for the TOFSense F2 Mini: this is a downward time-of-flight ranger using its own message format, not necessarily a 360° scanning LiDAR. Camera/YOLO tasks must be evaluated against actual CPU/RAM capabilities rather than a generic tutorial.
