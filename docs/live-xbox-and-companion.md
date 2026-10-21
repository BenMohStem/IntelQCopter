# Live Xbox and companion simulation

This older entry point now leads to the maintained guides:

- [Launch, manual controls, route planning and automatic recording](run-simulation.md).
- [Settings, actual command authority and tuning workflow](control-workbench.md).
- [F450 math and C implementation](current-model-and-fc-integration.html).
- [Diagnostics and remaining hardware limitations](debugging-and-flight-readiness.md).

The engine is native Gazebo with the actual C FC compiled for the host and actual ROS2 C++ processes. Ideal simulated localization remains an explicit assumption. Physical motor outputs remain disabled. [Earlier page retained in Git](../archive/documentation/README.md).
