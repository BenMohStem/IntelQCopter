# Simulation integration — next step

No ESP32-specific emulation or flight recording is claimed.
The C core was ported from main, but its Pico host ABI/launchers are not the active ESP32 build.

Retained references:

- F450 URDF: ../archive/rp2350/companion/ros2_ws/src/qcopter_description/urdf/f450.urdf
- Gazebo scripts/assets: ../archive/rp2350/sim/
- Original recordings/logs: ../archive/rp2350/artifacts/
- Mathematical paper: ../archive/rp2350/output/pdf/IntelQcopter-model-guide.pdf

Next: build a host adapter for fc_api.h, supply timestamped virtual sensor/pilot samples,
keep synthetic calibration distinct from measured calibration, reconnect ROS/Gazebo feedback,
then record new logs, parameters, graphs and video. Later compare actual ESP32/Xbox timing.
Do not relabel old Pico/emulator/Gazebo evidence as ESP32 execution.
