# Companion computer setup

## Common software boundary

Pi 4/5 and Jetson use the same C++ sources in `qcopter_autonomy`. Start with CPU
OpenCV DNN and the 320 px ONNX panel model; acceleration is a platform-specific
deployment choice. No TensorFlow is required. The current Windows C++ build and
DDS tests are evidence of compilation/integration, not an ARM performance test.

The RP2350 retains roll/pitch angle cascades, yaw rate and centered-stick vertical
speed/altitude hold. Companion outputs remain bounded flight-mode setpoints;
they never directly drive ESCs or grant physical arming. The physical serial
protocol/driver and pilot-versus-companion authority switch still need integration.

## OS selection is pending hardware identification

- Pi 4/5: Ubuntu 24.04 arm64 is compatible with the official Jazzy package platform.
  Install Jazzy ROS base, development tools, cv_bridge/OpenCV and camera driver.
  Keep RViz and Gazebo on the workstation; they need not run aboard the aircraft.
- Jetson: exact model and JetPack version are required before selecting an image.
  JetPack 6.2 uses an Ubuntu 22.04 root filesystem, whereas official Jazzy debs
  target Ubuntu 24.04. Do not replace the vendor GPU stack with desktop CUDA.
  A compatible container or supported platform/release combination must be chosen
  after identifying the board. No physical computer was reflashed or configured.

[Official Jazzy Ubuntu targets](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)
and [JetPack 6.2 platform information](https://developer.nvidia.com/embedded/jetpack-sdk-62).

## Pi build recipe after installing official Jazzy

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-robot-state-publisher libopencv-dev python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
cd IntelQCopter/companion/ros2_ws
colcon build --packages-select qcopter_description qcopter_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=64
```

This is a deployment recipe; it has not been run on a physical Pi. Obtain the
exported ONNX from this workstation and verify its SHA256; downloaded/trained
weights are excluded from Git. Keep hardware missions inactive initially.

## Required inputs and decisions

1. Exact companion model, OS/JetPack and available RAM.
2. Camera model and USB/CSI connection. Select its driver, calibrate intrinsics,
   rectify images and measure camera-to-body translation/rotation. The current
   panel node has a simulation-specific .18 m forward/.02 m up mounting transform.
3. A real metric local pose/velocity source. Current tests use simulated odometry.
   RGB + IMU/barometer does not automatically provide drift-free A-to-B navigation.
4. TOFSense exact model/interface, frame and timestamp handling. Current simulation
   uses a downward analytical floor ray; there is no validated physical lidar driver.
5. USB/UART wiring, rate, CRC, timeouts and FC authority/heartbeat. Those require
   the actual RP2350 board. Hardware motor arming remains disabled.

Current ROS odometry consumers require **world-frame linear velocity**, explicitly
signalled by matching `child_frame_id` to `header.frame_id`. This is a special
adapter contract, unlike conventional base-frame Odometry twist. Convert the
localization provider's twist explicitly before using it; do not simply relabel
frames. Pose and camera stamps must be synchronized. Keep camera/PnP processing
below the freshness deadlines; test dropout and companion restart before flight.

## Test order

1. Camera frames/calibration and timestamps; save a rosbag and actual image samples.
2. Replay recorded garden RGB through C++ YOLO/PnP, verify outputs and rejection cases.
3. Local pose source with motors disabled; verify coordinate axes and velocity frame.
4. Complete standard-topic Gazebo integration, then the physical serial bridge.
5. Sensor/communication loss, low battery, pilot override and landing tests.
6. Only after hardware is complete, measured and reviewed: explicitly decide arming.

The known-size blue-panel demo is not general arbitrary-obstacle avoidance. A
single forward lidar ray cannot certify the complete rotor corridor; a downward
mount primarily helps altitude. The mounting choice remains open for real hardware.
