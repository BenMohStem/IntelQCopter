# IntelQCopter · Pico architecture

C11 flight-control project for an F450/Q450 quadcopter. **Main:** RP2350 aircraft + RP2040 handheld.
[ESP32-S3/Xbox alternative](https://github.com/BenMohStem/IntelQCopter/tree/esp32-xbox).

**Prototype:** physical arming and ESC outputs remain disabled. The project has build,
CPU/emulator and simulation evidence; physical sensors, radio and flight are unvalidated.

## Start here

```powershell
.\qcopter.ps1 help
.\qcopter.ps1 build-flight
.\qcopter.ps1 build-handler
.\qcopter.ps1 build-sitl
.\qcopter.ps1 view-model
.\qcopter.ps1 garden -RunPrefix my-next-test
```

`garden` launches the existing native Gazebo/ROS workflow. It creates a new named run;
choose a unique prefix. Machine dependencies are documented in [Windows setup](docs/windows-jazzy-setup.md).
See [architecture and migration map](docs/architecture.md) before using an older command.

## Project map

| Area | Location |
|---|---|
| RP2350 / RP2040 C runtimes | [firmware/](firmware/README.md) |
| ROS2 C++ packages and perception | companion/ |
| Gazebo, host SITL, RP2350 emulator and plant parameters | sim/ |
| PCB draft and wiring | hardware/ |
| Guides and mathematical source | [docs/](docs/README.md) |
| Measurement procedures and blank worksheets | [measurements/](measurements/README.md) |
| Original logs, recordings and failed trials | artifacts/ |
| Retired Isaac and one-off setup/report scripts | [archive/](archive/README.md) |
| Build products / regenerable data | build/ (ignored) |
| Download/snapshot caches / machine settings | .cache/ / .local/ (ignored) |

Tools are grouped into build, simulation, model, docs, reports, ROS2, setup and validation.
`project.json` is the command/domain catalog; `qcopter.ps1` dispatches those commands.
Open `IntelQCopter.code-workspace` for both local architecture worktrees.

## Flight design and hardware

Roll and pitch angle cascades, yaw rate control, centered-stick vertical-speed commands
with height hold; six controller objects. RT provides boost allowance. The attitude
loop target is 500 Hz; physical timing has not been measured. Battery policy: warn at
35% estimated SOC, latch disarm at 20%, with no automatic battery landing.

F450/Q450, 1400 KV motors (exact model unknown), 9047 props, four 40 A ESCs,
3S 4200 mAh / 120C LiPo. The battery's roughly 150 g is an estimate. Complete mass,
inertia, thrust/current curves and sensor noise still need measurement.
MPU6050, BMP280 and HMC5883L are planned over I²C; LoRa is the primary handheld link,
with up to two optional CC1101 channels. TOFSense F2 mini placement remains undecided.

## Documentation and original evidence

- [Formatted model/integration guide](docs/current-model-and-fc-integration.html)
- [Detailed mathematical paper](docs/reports/IntelQcopter-model-guide.pdf)
- [Calibration and persistent storage](docs/calibration-flash.md)
- [Controller input design](docs/controller-inputs.md)
- [Current missing information](docs/missing-information-and-problems.md)
- [Recorded native Gazebo video](artifacts/simulation/2026-10-20-engines/jazzy-migration-v4-gazebo.mp4)
- [Recorded graphs](artifacts/simulation/2026-10-20-engines/jazzy-migration-v4-gazebo-graphs.png)
- [Original test/emulator log index](artifacts/test-runs/README.md)

The recorded native flight used an **assumed 1.2 kg** plant and fixture-style panel
perception with simulated localization. The separate 0.740 kg component URDF is also
an assumption. Earlier PyBullet and Isaac results retain their original engine labels.
The old line-by-line walkthrough is pinned to a historical source commit.

The ESP32 branch's newer linear/discrete tutorial and QEMU results are separate from
these Pico results. Select that worktree for its active code and launch commands.
