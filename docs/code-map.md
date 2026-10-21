# Active code and evidence map

| Domain | Active source |
|---|---|
| Flight control, calibration, estimators, battery math | firmware/src/flight_controller.cpp |
| Public platform boundary | firmware/include/fc_api.h |
| I²C, Xbox, scheduling, telemetry and NVS | firmware/src/flight_controller.cpp |
| Xtensa CPU replay | simulation/emulation/ |
| Native flight plant, worlds and input transport | simulation/gazebo/ |
| Drone ROS2 mission/perception/UART | companion/ros2_ws/src/qcopter_autonomy/ |
| Ground-PC map | companion/pc_mission_map.py |
| Mathematical plant/gain calculation | tools/model/model_lti.py |
| Knowledge validation/search | tools/knowledge/ |
| Offline guide rendering | tools/docs/ |
| UART CSV plotting | tools/reports/ |
| PlatformIO dependency adaptation | tools/platformio/ |
| Historical snapshot index and cache recovery | archive/ and tools/archive/ |

One `flight_controller.cpp` holds C-style flight math and the minimal C++ hardware-library boundary. The emulator compiles that same C++ source with the hardware boundary excluded;
it does not maintain a separate flight-controller implementation.

`project.json` declares domains and commands. Run `qcopter.ps1 help` to list them.
Open IntelQCopter.code-workspace for the active ESP32 tree, or the workspace file
in the sibling QCopter directory to see both architectures.

Build products and search indexes live in ignored build/. Historical duplicate
payloads live in ignored .cache/. Machine-specific runtime paths live in .local/.
Original execution logs and failed runs stay in versioned artifacts/.
The active worktree is QCopter-ESP32-Xbox, a sibling of QCopter; neither checkout
is inside a disposable build directory.
