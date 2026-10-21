# Simulation and emulation

- [Native Gazebo suite](gazebo/README.md): 10 authored worlds, five virtual Xbox manual flights and ten PC-requested navigation flights. Actual Gazebo physics/OGRE2, actual ROS2 DDS/C++ companion, active C flight core compiled for the host.
- [ESP32-S3 QEMU replay](emulation/README.md): active C core on the emulated Xtensa CPU, synthetic sensors and protocol tests. This is separate from Gazebo; Gazebo does not run inside QEMU.
- [LTI/discrete hover guide](../docs/current-model-and-fc-integration.html): numerical response calculations with assumed parameters.

No physical Bluetooth, sensor acquisition, ESC output or aircraft flight has been validated.
