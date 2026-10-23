# Important sources — start here

A small reading shelf of the files that matter. **These are byte-identical reference copies**, with source locations, branch/commit and SHA-256 in [manifest.json](manifest.json). Edit the canonical files linked below, then refresh these copies. Builds and the live simulator continue to use canonical paths, so edits in this reading shelf will not silently affect a flight.

## Current ESP32 sensor setup

Use the original FC with **MPU6050 + BMP280 only**; lidar and magnetometer may remain unplugged. The updated copy is [flight_controller.cpp](firmware/esp32/flight_controller.cpp). [Wiring, build and first-use calibration](../docs/esp32-mpu6050-bmp280.md). Both optional-ROS2 and Xbox-only versions compile; physical motor outputs remain disabled.

## 1. Flight controllers

| Read here | Active source / role |
|---|---|
| [ESP32 flight_controller.cpp](firmware/esp32/flight_controller.cpp) | [Canonical source](../firmware/src/flight_controller.cpp): C-style estimators, calibration, six control loops, Xbox decoding, optional companion link; Arduino library adapter at the end |
| [fc_api.h](firmware/esp32/fc_api.h) | [Canonical declarations](../firmware/include/fc_api.h), used by host simulation and firmware |
| [PlatformIO config](firmware/esp32/platformio.ini) | [Canonical config](../platformio.ini): Arduino framework, pinned sensor/Xbox libraries |
| [RP2350 flight_controller.c](firmware/rp2350/flight_controller.c) | Immutable snapshot from **main**, preserved long-term Pico aircraft architecture |
| [RP2040 handler_controller.c](firmware/rp2040/handler_controller.c) | Same main snapshot: two-stick handheld and radio architecture |

The two Pico files are reference material, not ESP32 build inputs. Their full Pico SDK project stays on main. All physical motor outputs remain disabled. [Arduino hardware-library guide](../docs/esp32-arduino-interfaces.md).

## 2. Drone ROS2 computer

The [qcopter_autonomy package](ros2/qcopter_autonomy/) keeps its normal ROS package layout, CMake, interfaces and launch files. Open these first:

1. [pilot_mission_node.cpp](ros2/qcopter_autonomy/src/pilot_mission_node.cpp): active Xbox-gated multi-node route execution and companion heartbeat.
2. [navigation.cpp](ros2/qcopter_autonomy/src/navigation.cpp) and [mission.cpp](ros2/qcopter_autonomy/src/mission.cpp): navigation and mission math.
3. [yolo_node.cpp](ros2/qcopter_autonomy/src/yolo_node.cpp): OpenCV DNN / ONNX inference from RGB frames. The trained model is an external prerequisite, not included here.
4. [panel_pose_node.cpp](ros2/qcopter_autonomy/src/panel_pose_node.cpp): distance from confirmed known panel dimensions, not general monocular depth.
5. [uart_bridge_node.cpp](ros2/qcopter_autonomy/src/uart_bridge_node.cpp): optional physical 3.3V UART to ESP32, TX/RX crossed plus common GND.
6. [esp32_companion.launch.py](ros2/qcopter_autonomy/launch/esp32_companion.launch.py): current companion entry point; requires model argument.
7. [PC mission map](ros2/pc_mission_map.py): ground-station route input.

Other retained nodes/launch files support earlier package entry points and remain because CMake references them; the current simulator uses pilot_mission_node. Without ROS2 connected, the ESP32 still accepts stable Xbox flight. Navigation requires an explicit mode request and a fresh companion; a lost link returns to stable control.

## 3. Gazebo, geometry and gains

| File | Meaning |
|---|---|
| [engine.py](simulation/gazebo/engine.py) | Native Gazebo physics integration |
| [fc_binding.py](simulation/gazebo/fc_binding.py) | Actual host-compiled FC core; synthetic sensors and ideal localization are explicit simulation assumptions |
| [build_world.py](simulation/gazebo/build_world.py) | SDF generator: garden, panels, F450-style geometry, cameras |
| [environments.json](simulation/gazebo/environments.json) | Ten garden obstacle arrangements |
| [parameters.json](simulation/model/parameters.json) | Provisional plant mass/inertia/thrust/lag values; not measured hardware data |
| [F450 URDF](drone_model/qcopter_description/urdf/f450.urdf) | Main-branch illustrative visual/inertial model; not the live Gazebo dynamic plant |
| [RViz config](drone_model/qcopter_description/rviz/f450.rviz) | Main-branch robot visualization settings |

Session controls, bridge, live-session management and settings are next to the Gazebo files. This shelf is not a replacement runtime checkout: keep launching from the project root using [the launch guide](../docs/run-simulation.md). [Explained math](../docs/current-model-and-fc-integration.html).

## 4. PCB — fabrication files

[KiCad project](../hardware/home-etched-fc/esp32_home_etch.kicad_pro) · [Board](../hardware/home-etched-fc/esp32_home_etch.kicad_pcb) · [Three-copy print PDF](../hardware/home-etched-fc/output/pdf/etch-mask-3-copies-A4-1to1.pdf) · [Complete CAD ZIP](../hardware/home-etched-fc/output/IntelQCopter-ESP32-home-etch-KiCad.zip) · [Original power/mask audit](../artifacts/hardware/solid-ground-power-audit-20261021/README.md).

## Keep this shelf current

From the ESP32 project root:

```powershell
.\qcopter.ps1 important-sources
.\qcopter.ps1 important-check
```

Refresh refuses to overwrite a hand-edited reference file. Copy your edits into its canonical source first. Downloaded SDKs, environments, credentials, videos and old trial output stay outside this shelf; original evidence remains under artifacts and history under archive.

Latest PCB edition: [manufacturer-reference dimensions and fit](../hardware/home-etched-fc/component-dimensions.md), with the current native KiCad package and three actual-size masks linked above.

## Iteration policy

Refresh this folder after project source changes and verify it before each project commit/push. The recent friend assistance and ESP32 rework are excluded from publishing unless explicitly included later. Keep the independent friend export outside this reading shelf and the project history.
