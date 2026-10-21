# Simulation and emulation

- [ESP32-S3 QEMU replay](emulation/README.md): active C core running on the emulated
  Xtensa CPU with deterministic synthetic inputs. Use qcopter.ps1 emulator-build,
  then emulator-run --name UNIQUE-NAME. This does not close a flight-physics loop.
- [Linear/discrete hover guide](../docs/current-model-and-fc-integration.html):
  independent numerical LTI response calculations with assumed aircraft parameters.
- The Pico branch's existing Gazebo/ROS workflow remains in the sibling QCopter
  checkout. Its historical source snapshot is recoverable through archive/README.md.

No ESP32/Gazebo flight recording is claimed. Next integration work is to connect
fc_api.h to timestamped plant samples and pilot commands, then record flight-loop
results with their actual engine, parameters and interface limits.
