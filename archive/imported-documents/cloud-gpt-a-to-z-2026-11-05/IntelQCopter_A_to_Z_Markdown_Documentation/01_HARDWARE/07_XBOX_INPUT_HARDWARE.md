# Xbox input hardware and communications reality

The pilot interface target is an Xbox gamepad. The **ESP32-S3 supports BLE, not Bluetooth Classic**, and actual Xbox firmware/software pairing compatibility must be verified. The current FC v2 avoids assuming a reliable direct wireless connection by using a Windows PC gamepad -> CH343 UART bridge for bench tests only.

## v2 virtual commands

- Left stick X: yaw-rate request.
- Left stick Y: vertical-speed request.
- Right stick X/Y: roll/pitch angle requests.
- D-pad Up + LB + RB: barometer height re-zero request **only disarmed**.
- PC sends CRC-protected serial command frames; a 200 ms freshness watchdog invalidates stale input.

## Other repository branch

`esp32-xbox` documented D-pad Down + A/B hold gestures for stable/navigation mode selection, and D-pad Down + LT for latched disarm. Those branch conventions must not be silently spliced into the newer v2 command parser without a coherent authority and safety review.

## For real aircraft use

Requires tested controller power-up and disconnect behavior, independent cut-off/arming state machine, radio quality assessment, stick center calibration, dead zones, bound setpoints, command precedence, lost-link policy and watchdog that has actually been tested while the propulsion system is restrained. The current USB bench bridge is **not a flight radio**.
