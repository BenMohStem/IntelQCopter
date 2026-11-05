# Software tracks, Git branches and artifact families

The IntelQCopter repository history includes multiple independent tracks. A new version number in one track should not silently supersede another.

## Branch families

- `main`: older RP2350 flight control/modeling/PCB work; its contents are not the November ESP32 sensor calibration source by default.
- `esp32-xbox`: ESP32-S3 Xbox and ROS2/Gazebo architecture, host/simulated flight-control logic; physical motor arming disabled in reviewed source.
- `codex/stepwise-esp32`: incremental hardware bench and verified sensor acquisition.
- `codex/qmc-200hz-continuous-diagnosis`: magnetometer problem investigation; check actual branch head before cherry-picks.

## Local artifact families

1. Calibration GUI and sensor source v5-v11; v13 transplant; `100Hz_Original_v13_Driver_Full_Calibration.zip`.
2. Magnetometer-only forensic v12-v18 (standalone nonbinary experiments).
3. Raw calibration and independent audit from 4 Nov.
4. FC bench v1, integrated FC core v1 and Fast 500 Hz FC v2.
5. Academic study (`Volume I Edition 2`) and uncertainty-model packages.

## Version-control discipline

Before committing, compare `platformio.ini`, `src/main.cpp` and `include/` against the target branch. Do not merge the v13 MAG-only `main.cpp` as the FC `main.cpp`: it would remove the other sensors and/or binary transport. Keep a golden calibration capture/config tag. Use a separate `docs/` branch or project `docs` folder for this knowledge base if desired; no GitHub changes are implied by this archive.
