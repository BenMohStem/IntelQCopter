# Ground PC and drone computer

Active aircraft launch: `qcopter_autonomy/launch/esp32_companion.launch.py`.
Active simulator entry: `simulation/gazebo/run_suite.py`.
Ground-PC endpoint UI: `pc_mission_map.py`.

The portable C++ aircraft role consists of `pilot_mission_node`, `navigation.cpp`,
`mission.cpp`, `yolo_node`, `panel_pose_node`, and optional `uart_bridge_node`.
The older `navigator_node`, `mission_node`, `simulation.launch.py` and
`garden_mission.launch.py` are retained reference integrations from the Pico
project. They do not implement the current Xbox authority protocol; do not use
them to launch the ESP32 suite.

`mission_client` and the PC map call the same endpoint service. Pi/Jetson choice,
camera driver, localization and electrical UART are pending hardware decisions.
The active simulator needs no actual companion board.

See [simulation instructions](../simulation/gazebo/README.md) and
[binary protocol and controls](../docs/xbox-ros2-protocol.md).
