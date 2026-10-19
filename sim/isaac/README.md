# Isaac Sim status and scene starter

## Actual status on this workstation

**Isaac Sim files are installed for all Windows accounts, but no Isaac Sim aircraft simulation has run.** The official Windows standalone 6.2.0 package is extracted at `C:\Users\Public\Documents\IsaacSim\app`; the archive MD5 matched NVIDIA's published checksum. Its package `VERSION` reads `6.2.0-rc.14+release.52805.02189a84.gl`. A shared install note is at `C:\Users\Public\Documents\IsaacSim\README-ALL-USERS.txt`.

The simulator has **not** been launched on this PC. Isaac Sim 6.2.0's current minimum lists Windows 11, GeForce RTX 4080, 16 GB VRAM, 32 GB RAM, and 50 GB SSD. This workstation is Windows 10 with an RTX 3050 Laptop GPU and 4 GB VRAM, so installation only stages the files; it does not make this PC a supported or usable simulator host. See NVIDIA's [system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html) and [standalone workstation install instructions](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_workstation.html). The QCopter project does not need TensorFlow, and its TensorFlow virtual environment has been removed.

The available artifacts are a starter URDF at [`assets/qcopter.urdf`](assets/qcopter.urdf), a parametric visual CAD model at [`../model/qcopter_f450.scad`](../model/qcopter_f450.scad), and scenario/interface planning files. They are **not** an Isaac Sim stage, calibrated multirotor dynamics model, or physics-simulation result. The URDF's mass and inertia are placeholders. It has no rotor thrust dynamics, sensor simulation, or connection to the RP2350 firmware.

Hardware limits and simulator release requirements change over time. Before running this installation, check the official requirements again. If this PC cannot run a supported version, Gazebo is a more practical local route; the URDF still needs inertial/rotor/sensor models for meaningful flight tests.

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
