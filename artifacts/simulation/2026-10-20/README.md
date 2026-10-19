# Modelling and autonomy evidence — 20 October 2026

Start with [the local evidence viewer](index.html), [the mathematical PDF](../../../docs/modelisation/main.pdf)
or [the editable LaTeX source](../../../docs/modelisation/main.tex).

[Self-contained LaTeX source bundle with figures](mathematical-report-source.zip).

## Actual results

The [best flight video](best-flight-cpu.mp4) is an original visualization of logged
CPU six-DOF physics with the **production C flight core** and common C++ A* planner.
It uses a known static obstacle map and simulated local odometry. It is **not Isaac
footage, hardware flight, or YOLO-driven navigation**. Playback is one simulated
second per second; no claim of a live hard-real-time simulation is made.

Selected nominal gain scale 1.3: 21.5s route, 0.01199m altitude RMS error,
0.14165m final 3D goal error, 0.30058m minimum surface clearance including the
9047 propeller envelope. All three nominal candidates and three held-out cases
completed without collision. See [summary](summary.json); the score and all
parameters are saved. The complete vehicle mass 1.2kg and motor dynamics are
assumptions, not identified hardware values.

| Evidence | Original files | Scope |
|---|---|---|
| Flight route, attitude, motor proxies | [CSV](nominal-1.3.csv), [graphs](flight-results.png), [video](best-flight-cpu.mp4) | C equations compiled for host, not RP2350 timing |
| Monte Carlo distributions | [Angle CSV](angle-mc.csv), [height CSV](altitude-mc.csv), [graph](kalman-monte-carlo.png) | 500 trials each, matched synthetic noise models |
| Time-varying gains | [Graph](kalman-gains.png), [metrics](estimation-metrics.json) | Two states per single-angle filter; [height,speed] altitude |
| Algebra, bad input, depth geometry, refinement | [Report](analysis-tests.json) | 2/1/0.5ms physical integration, fixed 500Hz C control |
| ROS 2 DDS route | [Report](ros2/summary.json), [CSV](ros2/dds-route.csv), [latency samples](ros2/dds-latencies-ms.csv) | 10,751 round trips; two same-process Python adapter nodes over UDP DDS |
| Actual GPU YOLO inference | [Report](perception/yolo-result.json), [annotated image](perception/yolo-reference.jpg) | Official bus reference image; separate from navigation |
| Actual OpenCV ONNX inference | [Report](perception/onnx-result.json), [image](perception/opencv-onnx-reference.jpg) | Portable inference contract; ROS C++ wrappers remain unbuilt |
| USD drone scene | [USD](../../../sim/isaac/assets/qcopter_test.usda), [validation](../../test-runs/2026-10-20/usd-validation.json) | Parser and mass attributes checked; no Isaac physics/render execution |
| Firmware build | [Build log](../../test-runs/2026-10-20/pico2-build.log) | Pico2/RP2350 build, no flashing |
| RP2350JS instruction check | [Original emulator log](../../test-runs/2026-10-20/rp2350js-boot.log) | 1,000,000 instruction steps; repeated wait-loop PC and unsupported peripherals, not successful control-loop or I/O validation |
| Isaac failure | [Vulkan log](../../test-runs/2026-10-20/isaac-launch.log), [second launch](../../test-runs/2026-10-20/isaac-d3d12-launch.log), [health](../../test-runs/2026-10-20/isaac-health.log) | GPU initialization failed before healthy remote server; no Isaac video exists |

ROS 2 DDS does not carry a hardware actuator command in these tests. The production
hardware loop still has sensor/radio stubs and no ESC output driver. A complete
camera → metric map → dynamic replanning → ROS C++ node → physical FC pipeline
has not yet been validated. The dated [original log directory](../../test-runs/2026-10-20/)
preserves both successful tests and failed setup/launch attempts.

Reproduce using [the runner](../../../tools/run_simulation_evidence.ps1) in the
configured local environments. Model assets and wrappers are reviewable source.
