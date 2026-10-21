# Original native Gazebo trials

Final selected evidence: **five manual flights** from `suite-20261021` plus
**ten navigation flights** from `verified-a-20261021` (env01–03),
`verified-b-20261021` (env04–06), and `verified-c-20261021` (env07–10).
The final report selects those explicit groups, including any failure if present.

Each original run directory retains its native recording, flight CSV, raw Xbox
reports, process logs, exact SDF and result metadata when the run completed.
Current navigation runs additionally retain wire-frame and perception JSONL.
An interrupted run can have an incomplete video and no final transport summary;
its files remain unchanged and do not establish a completed flight.

## Development history retained

| Group | Purpose / problem found |
|---|---|
| development-01 | Native engine/camera startup probe, no commanded flight |
| trial-a | Initial raw-Xbox manual flight; exposed lateral landing drift |
| trial-b | Camera nodes failed to load incompatible Windows OpenCV/cv_bridge DLLs; aircraft hovered |
| trial-c | Endpoint lay inside inflated obstacle margin; navigation held position |
| trial-d | Development navigation with endpoint beyond the obstacle margin |
| suite-20261021 | Five final manual cases; early navigation recording serializer failures preserved |
| navigation-20261021 | Found report-gap mode-selection failure and false ranges from occluded panel silhouettes; final attempt interrupted |
| verified-a/b/c-20261021 | Rechecks after acknowledgement-based virtual input hold and known-fixture height consistency |

Development results used their recorded criteria. They are not substituted for
the final selected set. In particular, trial-a's original pass does not establish
the later <20 cm landing criterion. No original result or failed trial was edited
to produce a pass.

## Scope

Engine: native Gazebo 10.5, OGRE2 RGB, active ESP32 C source as a **host DLL**,
actual independent ROS2 processes. QEMU CPU emulation is separate evidence.
The 1.2 kg mass, inertia, thrust/lag, synthetic sensor noise and calibration are
assumptions, not physical measurements. Localization is Gazebo odometry. RGB
distance estimates depend on known-size blue panels standing on a known flat
floor, including their centre-height prior. No physical BLE, UART, I²C, ESC,
real-time deadline or arbitrary-object obstacle avoidance has been validated.

Physical motor outputs remain disabled. The simulator's explicit motor gate
allows these flight-physics tests without changing that hardware instruction.
