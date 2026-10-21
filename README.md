# IntelQCopter · ESP32-S3 + Xbox

**Branch: esp32-xbox** · ESP32-S3 on the aircraft · Xbox Series X/S over BLE.

The [main branch](https://github.com/BenMohStem/IntelQCopter/tree/main) keeps the
RP2350 flight controller and RP2040 handheld. This branch replaces both custom
radio endpoints with a commercial Xbox controller connected directly to the ESP32-S3.

> **Bench prototype. Physical motor outputs are disabled.**
> No ESC pulse driver or arming action is included. Gains, axes and timing are not validated on this airframe.

**Start here:** [Illustrated F450 linear/discrete guide](docs/current-model-and-fc-integration.html) ·
[Run the ESP32-S3 emulator](simulation/emulation/README.md) · [Code map](docs/code-map.md).

## Run the simulation

**[Complete step-by-step launch guide →](docs/run-simulation.md)** — manual Xbox flight, 2–64-node navigation, automatic recordings, fresh-clone prerequisites and troubleshooting.

### Quick start on the configured Windows PC

Connect your Xbox Series X/S controller. Open **PowerShell in the ESP32 checkout**, then run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-ESP32-Xbox
.\qcopter.ps1 xbox-check
.\qcopter.ps1 station
```

1. Open **[http://127.0.0.1:8088/](http://127.0.0.1:8088/)**.
2. Choose **FC only · real Xbox** or **FC + onboard ROS2 navigation**, then choose a garden.
3. Click **Start simulation**. The flight opens automatically in a new tab when ready.
4. **Manual:** select **Enable manual flight**, wait for the mode gesture, then gently push the **left stick up**. The right stick controls roll/pitch angles. Gentle is the default.
5. **Navigation:** add start → any intermediate nodes → finish in the embedded route planner, then **Review route → Approve and fly**. The drone's ROS2 computer handles the mission through the C FC.
6. Click **End session** when finished. Recording is automatic; the video finalizes, the flight tab closes and the home page remains ready for another run. Closing a tab alone does not stop the simulator.

Only one simulation runs at a time. Startup may take up to 90 seconds. No physical drone/ESP32 board is needed; physical motor outputs remain disabled.

| Local page | Purpose |
|---|---|
| [8088 — workshop](http://127.0.0.1:8088/) | Start/end, scene selection, next-launch settings and recording library |
| [8087 — flight station](http://127.0.0.1:8087/) | Live cameras, Xbox controls, graphs and embedded route planner during a session |
| [Current proof report](http://127.0.0.1:8088/hardening-proof/) | Final videos, measured graphs, command-authority audit and fault tests |

These localhost links work on the PC running the server. GitHub itself does not run the simulation or interactive HTML reports.

### Rebuild after code changes

End the active simulation first, then:

```powershell
.\qcopter.ps1 build-host       # FC Windows DLL used by Gazebo
.\qcopter.ps1 build-companion  # ROS2 C++ nodes; only needed for companion changes/navigation
.\qcopter.ps1 station
```

For a **fresh clone**, first use the [dependency/configuration instructions](docs/run-simulation.md#6-fresh-clone--another-pc). Installed environments, build outputs and the trained panel ONNX model are not bundled. Manual-only mode does not need ROS2 or YOLO. The Windows wrappers contain workstation-specific paths that must be adapted on another PC. TensorFlow and Isaac Sim are not needed.

The engine is **native Gazebo + the active C FC host build**. The ESP32-S3 CPU emulator is a separate workflow. Simulated sensors/calibration, ideal localization and assumed aircraft parameters do not establish physical flight readiness.

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
| CPU emulation | Espressif QEMU ESP32-S3; 63 passing checks on the current C core with synthetic inputs |
| Battery policy | 35% estimated-SOC warning, 20% latched disarm; voltage/current acquisition unfinished |

## Controls

| Xbox input | Command |
|---|---|
| Left stick horizontal | Yaw rate |
| Left stick vertical | Climb/descent; center holds height |
| Right stick horizontal / vertical | Roll / pitch angle |
| RT / LT | Smooth climb/collective boost / reduction |
| D-pad Down + A, held 5 s | Select stable angle/yaw-rate/vertical-speed flight |
| D-pad Down + B, held 5 s | Select drone ROS2 navigation authority |
| D-pad Down + LT at least 90%, held 2 s | Latched disarm; restart required |
| LB / RB and remaining buttons | Reserved |

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
| simulation/gazebo/ | Native physics, cameras, raw Xbox/ROS2 transport and scenarios |
| companion/ | ROS2 C++ drone navigation, YOLO/PnP, UART bridge and PC endpoint map |
| artifacts/ | Original evidence produced for this branch |
| archive/reference-drone/ | Unmodified source snapshot from the user's Drone folder |
| archive/ | Historical Git snapshot index and original Drone reference |
| .cache/archive/ | Verified local snapshot payload; recoverable from Git |

**Read next:** [Current status](docs/status.md) · [Knowledge index](knowledge/README.md) ·
[Review of Drone](docs/drone-review.md).

Use `.\qcopter.ps1 help` for the command catalog. See [architecture and migration](docs/architecture.md).
Open `IntelQCopter.code-workspace` for an editor view focused on active files.
Generated tools, firmware images and the searchable SQLite index live in ignored `build/`.

## Current and historical build evidence

Current standard, standalone and level-3 debug firmware builds pass; QEMU passes **63 checks**. [Current raw build logs](artifacts/maintenance/hardening-20261021/) · [Current emulator UART log](artifacts/emulation/hardening-final-20261021/uart.log).

The following size figures and baseline logs are historical snapshots.

An earlier ESP32-S3 hardware firmware build compiled successfully: **31,836 bytes static RAM** and
**590,573 bytes reported program flash**. These size figures do not include every
runtime heap/task allocation. [Latest compiler and interface logs](artifacts/validation/2026-10-21/).
No flashing, pairing, physical calibration or flight has been performed.

An earlier QEMU input replay passed 39 checks. [Latest original UART log](artifacts/emulation/uart-mode-gestures-2026-10-21/uart.log).
The original baseline executed 4,982 logical steps and passed 19 checks. [Original UART log](artifacts/emulation/verified-replay/uart.log) ·
[Recorded samples](artifacts/emulation/verified-replay/telemetry.png).
The replay uses stationary synthetic sensors; motor demands do not drive a flight plant.

The [mathematical paper](https://github.com/BenMohStem/IntelQCopter/blob/8432cf33300fc1f92aeab97f01548aac2b1d2e48/archive/rp2350/output/pdf/IntelQcopter-model-guide.pdf)
remains useful for airframe/control equations. Its RP2350 flash and old runtime descriptions
are historical and do not describe the ESP32 NVS/platform layer.

## Native Gazebo flights

[Run the 10-world suite and PC map](simulation/gazebo/README.md) · [Controls and UART protocol](docs/xbox-ros2-protocol.md).

Gazebo drives the active C core at 500 Hz through simulated IMU/barometer samples.
The ground PC sends Xbox-format reports or a ROS2 endpoint request. The drone
C++ companion handles navigation and RGB perception. Physical ESC outputs stay disabled.
CPU emulation and host flight simulation are separate evidence categories.

## Earlier 10-world suite evidence · 21 October 2026

| Check | Result |
|---|---|
| ESP32-S3 hardware build | Pass; no flashing |
| ESP32-S3 QEMU | 39/39 checks |
| Native Gazebo manual Xbox flights | 5/5; zero companion flight packets |
| Native Gazebo ROS2 navigation | 10/10 layouts; no native panel contacts |
| Additional PC map mission | Custom endpoint/height accepted over ROS2; landed/disarmed |

[Results, limitations and measurements](docs/gazebo-results-2026-10-21.md) ·
[Video and graph index — open locally](artifacts/reports/gazebo-verified-20261021/index.html) ·
[Launch and choose your own endpoints](simulation/gazebo/README.md#pc-endpoint-selection).

![Actual native Gazebo navigation trajectories](artifacts/reports/gazebo-verified-20261021/navigation-trajectories.png)

Mean simulated landing error: **9.49 cm manual**, **1.53 cm navigation**. This
uses unmeasured aircraft parameters, synthetic sensors and simulated localization.
RGB distance is specific to known panel fixtures; physical interfaces remain unvalidated.


### Simulation workshop and seven-node authority audit

Use `.\qcopter.ps1 station` → http://127.0.0.1:8088 for start/stop, automatic recordings, next-launch FC/Kalman/model/ROS2 settings and a mathematical guide. Live cameras, route approval and actual C telemetry graphs are on port 8087 during a session.

[Control workshop guide](docs/control-workbench.md) · [Seven-node original video, graphs and authority audit](artifacts/reports/authority-20261021/index.html). Native Gazebo + actual C host core + ROS2; synthetic calibration/sensors and perfect simulated localization. Physical ESC output remains disabled. Old failed trials are retained.


### Manual control and automatic session tabs

Recording starts automatically. Home Start opens the flight in a new tab; End session finalizes the recording and closes that tab. Manual roll/pitch command tilt angles, then the C FC brakes and holds the release position using explicit simulated horizontal localization. Yaw direction is corrected. Real IMU/barometer alone cannot provide horizontal position hold; physical outputs remain disabled.

[Current angle/braking/yaw video and graphs, including preserved failure](artifacts/reports/manual-control-20261021/index.html). Native Gazebo/C host tests and 54 QEMU checks passed; no physical-flight claim.

### Gentle default and current hardening evidence

Gentle is the firmware/UI default. RT retains up to an 1800 µs-equivalent collective ceiling with PID headroom; LT reduces. The standard FC works without a companion and supports optional ROS2 navigation. Physical outputs remain disabled.

[Current videos, measured graphs and fault tests](artifacts/reports/hardening-20261021/index.html) · [Debug levels, profiling and real-flight preparation](docs/debugging-and-flight-readiness.html).
