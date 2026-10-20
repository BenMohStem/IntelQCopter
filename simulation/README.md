# Simulation integration — next step

ESP32-S3 CPU emulation now runs: see [emulation/README.md](../emulation/README.md).
It replays synthetic inputs and checks the active C code; it is not a flight plant.
No new ESP32/Gazebo flight recording is claimed.
The C core was ported from main, but its Pico host ABI/launchers are not the active ESP32 build.

Retained references:

- F450 URDF: ../archive/rp2350/companion/ros2_ws/src/qcopter_description/urdf/f450.urdf
- Gazebo scripts/assets: ../archive/rp2350/sim/
- Original recordings/logs: ../archive/rp2350/artifacts/
- Mathematical paper: ../archive/rp2350/output/pdf/IntelQcopter-model-guide.pdf

The active [linear/discrete guide](../docs/current-model-and-fc-integration.html)
and `tools/model_lti.py` provide separate calculated hover responses and a 16-state
plant model, with assumed parameters clearly recorded.

Next: build a host adapter for fc_api.h, supply timestamped virtual sensor/pilot samples,
keep synthetic calibration distinct from measured calibration, reconnect ROS/Gazebo feedback,
then record new logs, parameters, graphs and video. Later compare actual ESP32/Xbox timing.
Do not relabel old Pico/emulator/Gazebo evidence as ESP32 execution.
