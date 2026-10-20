# Isaac Sim — historical source, application removed

The user requested discontinuing Isaac Sim on 20 October 2026. The standalone
application at `C:\IsaacSim` was removed and current launchers use Gazebo with
standalone Jazzy. These files and original recordings are retained for provenance.
Later DirectX/PhysX garden flights did succeed before removal; the earlier failure
notes below are chronological diagnostics, not the final status.

## DirectX 12 retry — 20 October 2026

Explicitly setting `/app/vulkan=false` (in addition to `--d3d12`) detects the
RTX 3050, reaches `app ready`, and opens the local drone USD successfully.
The earlier plain d3d12 option did not override the base app's Vulkan setting.
Two camera probes still return an empty RGB array. This is progress beyond GPU
enumeration, but it does not yet establish working capture or vehicle physics.
Original logs: `artifacts/test-runs/2026-10-20-isaac-retry/`.
The repeatable probe is `tools/retry_isaac.ps1`; physical arming stays disabled.

## Installation and launch result

The shared standalone installation is at **C:\IsaacSim\app**, moved from Public
Documents as requested. It is separate from the older project Python environment
used for CPU/YOLO tests. The standalone package identifies as 6.2.0-rc.14.

Two launches on 20 October 2026 failed physical GPU enumeration and GPU Foundation
initialization. The remote Python server never became healthy. Original
[launch](../../artifacts/test-runs/2026-10-20/isaac-launch.log),
[second attempt](../../artifacts/test-runs/2026-10-20/isaac-d3d12-launch.log), and
[health probe](../../artifacts/test-runs/2026-10-20/isaac-health.log) logs are retained.
Adding the d3d12 option did not resolve the failure. This host is Windows 10,
RTX 3050 Laptop 4 GB, driver 555.99. No Isaac vehicle physics, camera capture, ROS
bridge scene or recording succeeded here.

## New USD model

Run `sim/isaac/build_stage.py` in the project environment to author
[assets/qcopter_test.usda](assets/qcopter_test.usda). It has 18 prims: rigid body
with assumed 1.2 kg mass and explicit diagonal inertia, red/white arms, four 9047
rotor discs, conservative 0.34 m collision envelope, ground, obstacles, light,
overview camera and gravity scene. Parameter source:
[simulation_parameters.json](../model/simulation_parameters.json).

[USD validation](../../artifacts/test-runs/2026-10-20/usd-validation.json) confirms
successful parser reopening and mass/scene attributes. It does not establish
PhysX stepping, contact correctness, sensor rendering or a flight-force adapter.
The older URDF remains a separate visual starter with older placeholders; it is
not the parameter source for this USD or the CPU tests.

## Working baseline and remaining integration

The [CPU simulation](../sitl/run.py) uses the production C flight equations,
common C++ planner and documented six-DOF plant. Two bundled ROS2 Jazzy Python
adapter nodes separately complete the route through actual UDP DDS.
[Video, graphs and data](../../artifacts/simulation/2026-10-20/README.md) explicitly
label that engine and its assumptions.

On a working Isaac host, first pass renderer/server health, explicitly select
PhysX (this package also contains Newton), open the USD and verify mass, contact
and frames. Force/torque actuator adaptation, sensor sampling, ROS clock/topics
and real camera capture still need implementation/testing in that live environment.
This source does not claim those adapters are connected.
