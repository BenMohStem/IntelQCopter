# Important sources

This fresh branch links canonical files instead of duplicating them.

- [Step-by-step plan](../docs/PLAN.md)
- [Hardware bench procedure](../docs/bench-tests.md)
- [Actual hardware configuration](../config/hardware.json)
- [Stage dependencies](../validation/stages.json)
- [Single Arduino-style main.cpp](../firmware/main.cpp), [PlatformIO configuration](../platformio.ini), [validation/repair guide](../docs/validation.md), and [serial capture tool](../tools/capture.py).

Earlier complete firmware, ROS2, Gazebo and PCB sources remain on `esp32-xbox`; RP2350/RP2040 remain on `main`. They are not silently imported into this clean rebuild or described as physically accepted.
