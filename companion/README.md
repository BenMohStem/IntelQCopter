# Companion autonomy — simulation development

`ros2_ws/src/qcopter_autonomy` is a C++17 ROS 2 package for the same Pi/Jetson build.
The portable A* and tracking core is compiled and exercised by the CPU simulation.
The ROS-specific C++ wrapper still requires a complete ROS 2 development installation
and has **not** been compiled on this PC. The bundled Isaac runtime supplies Python
message bindings and DDS libraries, not the complete C++ development environment.

## Contract

- Input `odometry`: `nav_msgs/Odometry`, local map/odom ENU, velocity transformed
  into that world frame by the adapter. Standard Odometry twist is normally in
  `child_frame_id`; transform it before using this adapter. Full VIO/GPS localization
  is not implemented. The adapter sets child_frame_id equal to header.frame_id to
  explicitly indicate this transformed velocity convention; ordinary base_link
  twist is rejected. Goals must match that exact frame.
- Input `goal`: `PoseStamped`, same frame as odometry. Static circles are parameters.
- Output `autonomy/setpoint`: custom `AutonomySetpoint` message: desired roll/pitch
  radians, yaw radians/s, normalized throttle (0.5 means hold), timestamp, valid flag
  and reached flag in a single sample. Invalid on stale odometry >250ms.
  The downstream adapter must time out this stream, reject old/out-of-order stamps,
  require pilot mode authorization, and fall back to flight-controller control.
  This node cannot arm motors.

Build on a configured ROS 2 system: `colcon build --packages-select qcopter_autonomy`.
Launch `ros2 launch qcopter_autonomy simulation.launch.py`. A* bounds are currently
x[-2,10], y[-4,4], 0.2m cells; this is a bounded static map baseline, not a general
dynamic-obstacle planner. A new goal triggers replanning. Out-of-map goals fail.

## Perception status

YOLO inference has its own recorded reference test; it is not driving the successful
route in the current videos. `perception/depth_geometry.py` implements tested metric
back-projection and semantic depth annotation. A camera + YOLO alone supplies no
metric range or reliable unknown-obstacle coverage. The downward 1D lidar can help
height; a front 1D lidar measures only one ray. Neither replaces depth/stereo or a
mapped obstacle sensor for this planner. Camera synchronization, extrinsics, depth
map updates, VIO and live replanning are remaining integration work.

`yolo_node.cpp` adds a ROS C++ semantic detector using OpenCV DNN and a raw
YOLO11 ONNX export (640 square, no embedded NMS). Parameters: `onnx_model`
(required path), `use_cuda` (default false; needs an OpenCV CUDA build).
Topics: `camera/image`, `camera/depth_registered` (32FC1 metres or 16UC1
millimetres), and `perception/semantics` (custom stamped detections).
Range annotation requires matching dimensions/frame and timestamps within 50ms.
These semantics are not connected to the static map. The C++ ROS wrapper is
unbuilt; the actual OpenCV inference/preprocessing contract was separately
executed through Python, with original JSON and logs saved.

Export the tested model with `YOLO('build/perception/yolo11n.pt').export(
format='onnx',opset=12,simplify=False,dynamic=False)` in the project environment.
Weights and ONNX files are generated under ignored `build/perception`.
