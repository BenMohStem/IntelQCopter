# ROS 2 companion: Pi / Jetson C++ sources

The ROS package is ros2_ws/src/qcopter_autonomy. Its portable C++ mission and A*
navigation core is compiled and tested. The ROS C++ wrappers have **not** been
built: this PC has runtime DDS/Python bindings, without ROS development headers.
Real DDS adapter nodes were exercised separately; see the garden evidence.

## Current garden route

The companion selects start/finish, supervises spool, takeoff, hover, navigation,
approach, descent and landed states. It emits the existing roll/pitch angle,
yaw-rate and centered vertical-speed setpoints. No seventh flight PID is added.
Approach holds horizontal position before descending. Hardware arming is disabled.
RT boost is inhibited during simulated approach/landing.

Actual RGB YOLO plus known-size blue-panel PnP updates the observed obstacle map
online; A* replans around those observations. The runtime has no true obstacle map
or depth image. One downward lidar aids height. This requires calibrated 1.09 x
1.69 m blue panel interiors and simulated odometry/contact; general monocular
obstacle ranging, VIO, real landed detection and arbitrary-obstacle coverage remain
unfinished. A* is bounded to x[-2,10], y[-4,4], 0.2 m cells with static observations.

## ROS interfaces (source only until a full ROS build)

- Input odometry: local ENU; adapter must transform twist into the world frame
  and explicitly set child_frame_id equal to header.frame_id. Ordinary base_link
  twist is rejected. Stale pose invalidates commands; real fallback is unfinished.
- camera/image and camera/camera_info: synchronized rectified RGB/calibration.
- perception/semantics: custom stamped YOLO detections. yolo_node.cpp accepts a
  raw YOLO ONNX export with no embedded NMS, configured input_size (320 here).
- MetricObstacles: timestamped measured centres/radii, image_processed marker.
- mission/request: RequestMission service, start/finish Point and cruise height.
- mission/land: std_srvs/Trigger request to land at current XY.
- autonomy/setpoint: atomic custom AutonomySetpoint with validity/time stamp.
  A physical downstream bridge must enforce freshness, authority and fallback.

Build on a complete ROS 2 development system:

```bash
colcon build --packages-select qcopter_autonomy
source install/setup.bash
ros2 launch qcopter_autonomy garden_mission.launch.py panel_model:=/path/best.onnx enable_simulation_missions:=true known_fixture_model_confirmed:=true
```

Those flags default false. mission_client takes seven metre values. The executed
Windows Python DDS selector is documented in ../sim/garden/README.md. This adapter
is evidence of DDS transport and common control-core execution, not a compiled ROS
C++ deployment or a physical UART connection.

Legacy simulation.launch.py/static-map and depth_geometry.py remain reference
experiments. The new garden uses single RGB, not depth_geometry.py. Real-world
unknown obstacles require additional validated sensing/localization work.
