# Isaac Sim status and scene starter

## Actual status on this workstation

**No Isaac Sim aircraft simulation has run.** This checkout does not contain an Isaac Sim Python environment; its earlier startup attempt on the Windows 10 / RTX 3050 Laptop (4 GB VRAM) workstation failed before any vehicle scene was loaded. The former setup notes incorrectly said a local environment was installed; that claim has been removed. The QCopter project does not need TensorFlow, and its TensorFlow virtual environment has been removed.

The available artifacts are a starter URDF at [`assets/qcopter.urdf`](assets/qcopter.urdf), a parametric visual CAD model at [`../model/qcopter_f450.scad`](../model/qcopter_f450.scad), and scenario/interface planning files. They are **not** an Isaac Sim stage, calibrated multirotor dynamics model, or physics-simulation result. The URDF's mass and inertia are placeholders. It has no rotor thrust dynamics, sensor simulation, or connection to the RP2350 firmware.

Hardware limits and simulator release requirements change over time. Before installing or relying on Isaac Sim, check the official [Isaac Sim system requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) for the version you intend to use. If this PC cannot run a supported version, Gazebo is a more practical local route; the URDF still needs inertial/rotor/sensor models for meaningful flight tests.

## What has actually been tested

- The URDF parses as XML. This only verifies file syntax.
- The RP2350 emulator runs the firmware image for a fixed instruction count. It does not execute or validate an Isaac Sim scene.
- The C Kalman test is a seeded synthetic sensor/attitude math experiment. It is not vehicle physics or HIL.

See the [evidence index](../../artifacts/test-runs/README.md) for original logs, CSVs, and limitations. No vehicle-simulation video is included because no vehicle physics simulation has completed.

## Next scene work

1. Measure the real frame, motor locations, components, and assembled mass.
2. Replace placeholder URDF inertias with measured mass properties.
3. Add four motor/propeller thrust and torque models, actuator limits, and a documented coordinate convention.
4. Add sensor models with measured rates, noise, bias, and dropout.
5. Connect the model to a simulator-supported flight-control interface, then validate one behavior at a time and preserve the simulator's raw output, version, scene, and playback capture.
