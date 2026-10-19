# Isaac Sim and ROS 2 interface contract

This is the first integration boundary for the companion computer and the Isaac Sim scene. The RP2350 remains responsible for stabilization, arming, motor mixing, and loss-of-link failsafes. ROS 2 traffic is for simulation and bounded high-level commands; it must never bypass the flight controller to drive ESCs.

## ROS 2 topics

Use ROS 2 standard messages and SI units. Topic names are relative to `/qcopter` so a namespace can be applied per vehicle.

| Topic | Type | Direction | Rate / notes |
|---|---|---|---|
| `imu` | `sensor_msgs/msg/Imu` | Isaac → FC adapter | Physics rate; acceleration m/s², angular velocity rad/s, frame `base_link` |
| `barometer` | `sensor_msgs/msg/FluidPressure` | Isaac → FC adapter | 25–50 Hz; pressure Pa; adapter derives relative altitude after ground calibration |
| `magnetic_field` | `sensor_msgs/msg/MagneticField` | Isaac → FC adapter | 10–50 Hz; tesla, calibrated/perturbed world field in sensor frame |
| `range` | `sensor_msgs/msg/Range` | Isaac → companion / FC adapter | Sensor-specific rate; metres; invalid/out-of-range readings preserved as invalid |
| `pilot/setpoint` | `geometry_msgs/msg/TwistStamped` | pilot/companion → FC | Bounded body-frame setpoint; linear.z vertical speed m/s, angular.x/y roll/pitch radians, angular.z yaw rate rad/s |
| `pilot/enable` | `std_msgs/msg/Bool` | safety supervisor → FC | Enable request only; FC still applies arming interlocks |
| `actuators/motors` | `std_msgs/msg/Float32MultiArray` | FC → Isaac | Exactly 4 normalized values `[0,1]`, logical order FL, FR, RR, RL; reject wrong length/non-finite values |
| `status` | `diagnostic_msgs/msg/DiagnosticArray` | FC → companion | Health, mode, sensor age, link age, loop timing, and failsafe reason |

Every sensor and command uses the message header timestamp. Reject stale messages and commands; the adapter must not silently replace missing sensor data with zeros. Use best-effort sensor QoS for high-rate streams and reliable, shallow-depth QoS for commands/status. Confirm the Isaac bridge supports the selected ROS 2 distribution before choosing QoS settings in a runnable launch file.

## Coordinate and timing rules

- ROS uses right-handed FLU (`x` forward, `y` left, `z` up); the flight firmware convention must be declared and converted once at the adapter boundary.
- Use ROS timestamps for simulation correlation. Firmware control `dt` is measured on-device and remains independent of ROS wall-clock timing.
- Simulated motor commands are normalized controls, not thrust in newtons. A vehicle-specific actuator model maps command to RPM/thrust after motor and propeller data are measured.
- The ROS 2 companion path is noncritical. If its topic or process stops, the flight controller continues its local stabilizer and applies its configured setpoint timeout/failsafe.
- Do not connect this simulated actuator topic to physical ESC outputs. A separate explicit hardware build/backend is required.

## Next implementation boundary

The firmware currently has a control-math scaffold, not a runnable 500 Hz task, shared sensor abstraction, or simulation protocol. Implement those first behind a `SIM_IO` backend and define a framed, checksummed serial transport for HIL. Then build an Isaac bridge that converts ROS sensor messages to that transport and maps returned actuator values into the vehicle model. Until then, this document is an interface contract only, not a running bridge.
