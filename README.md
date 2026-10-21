# IntelQCopter · ESP32-S3 + Xbox

**Branch: esp32-xbox** · ESP32-S3 on the aircraft · Xbox Series X/S over BLE.

The [main branch](https://github.com/BenMohStem/IntelQCopter/tree/main) keeps the
RP2350 flight controller and RP2040 handheld. This branch replaces both custom
radio endpoints with a commercial Xbox controller connected directly to the ESP32-S3.

> **Bench prototype. Physical motor outputs are disabled.**
> No ESC pulse driver or arming action is included. Gains, axes and timing are not validated on this airframe.

**Start here:** [Illustrated F450 linear/discrete guide](docs/current-model-and-fc-integration.html) ·
[Run the ESP32-S3 emulator](simulation/emulation/README.md) · [Code map](docs/code-map.md).

## What is active

| Part | Implementation |
|---|---|
| Flight control | One C file: firmware/src/flight_controller.c |
| Hardware boundary | Small C++ adapter for Arduino, I²C sensors, Xbox BLE and NVS |
| Controller | XboxSeriesXControllerESP32_asukiaaa 1.1.3 with NimBLE 2.5.1 |
| Sensors | MPU6050 acquisition; BMP280 pressure acquisition and session ground reference |
| Control mode | Roll/pitch angle cascades, yaw rate, vertical-speed throttle with height hold |
| Scheduling | 2 ms timer requests; measured elapsed time; missed/late cycles and execution time reported |
| Calibration | Stationary gyro verification, six-face accel capture, versioned/CRC/board-bound NVS record |
| CPU emulation | Espressif QEMU ESP32-S3; 19 passing checks on the active C core with synthetic inputs |
| Battery policy | 35% estimated-SOC warning, 20% latched disarm; voltage/current acquisition unfinished |

## Controls

| Xbox input | Command |
|---|---|
| Left stick horizontal | Yaw rate |
| Left stick vertical | Climb/descent; center holds height |
| Right stick horizontal / vertical | Roll / pitch angle |
| RT | Smooth climb/collective boost allowance |
| LB + RB + A / B | Log takeoff / landing request only; no mission or arming action |
| LT, D-pad, other buttons | Reserved |

## Build

Install [PlatformIO](https://docs.platformio.org/en/latest/core/installation/index.html), then from this branch checkout:

    pio pkg install
    pio run -e esp32-s3

Dependency installation precedes the guarded Xbox callback hook.
Build is separate from flashing. See [setup, wiring and calibration](docs/setup.md).

## Clean project map

| Folder | Purpose |
|---|---|
| firmware/ | Active C control core and minimal C++ platform adapter |
| docs/ | Illustrated LTI/discrete model, setup, code map and current status |
| simulation/emulation/ | Reproducible ESP32-S3 CPU replay and original-check runner |
| knowledge/ | Versioned facts, evidence sources and dependency relationships |
| simulation/ | Flight simulation integration plan; QEMU replay is separate |
| artifacts/ | Original evidence produced for this branch |
| archive/reference-drone/ | Unmodified source snapshot from the user's Drone folder |
| archive/ | Historical Git snapshot index and original Drone reference |
| .cache/archive/ | Verified local snapshot payload; recoverable from Git |

**Read next:** [Current status](docs/status.md) · [Knowledge index](knowledge/README.md) ·
[Review of Drone](docs/drone-review.md).

Use `.\qcopter.ps1 help` for the command catalog. See [architecture and migration](docs/architecture.md).
Open `IntelQCopter.code-workspace` for an editor view focused on active files.
Generated tools, firmware images and the searchable SQLite index live in ignored `build/`.

## Build evidence

The latest ESP32-S3 hardware firmware compiles successfully: **31,732 bytes static RAM** and
**588,741 bytes reported program flash**. These size figures do not include every
runtime heap/task allocation. [Latest original logs and checks](artifacts/2026-10-21/README.md).
No flashing, pairing, physical calibration or flight has been performed.

QEMU executed 4,982 logical steps and passed 19 checks. [Original UART log](artifacts/emulation/verified-replay/uart.log) ·
[Recorded samples](artifacts/emulation/verified-replay/telemetry.png).
The replay uses stationary synthetic sensors; motor demands do not drive a flight plant.

The [mathematical paper](https://github.com/BenMohStem/IntelQCopter/blob/8432cf33300fc1f92aeab97f01548aac2b1d2e48/archive/rp2350/output/pdf/IntelQcopter-model-guide.pdf)
remains useful for airframe/control equations. Its RP2350 flash and old runtime descriptions
are historical and do not describe the ESP32 NVS/platform layer.
