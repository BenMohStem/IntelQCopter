# Important sources

This fresh branch links canonical files instead of duplicating them.

- [Step-by-step plan](../docs/PLAN.md)
- [Hardware bench procedure](../docs/bench-tests.md)
- [Actual hardware configuration](../config/hardware.json)
- [Stage dependencies](../validation/stages.json)
- Firmware, capture tools and ledger commands are added in the next implementation commits.

Earlier complete firmware, ROS2, Gazebo and PCB sources remain on `esp32-xbox`; RP2350/RP2040 remain on `main`. They are not silently imported into this clean rebuild or described as physically accepted.
