# Fast FC v2 — actual source architecture

**Archive:** `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip`. This is the most recent supplied source for a 500 Hz mathematical FC, configured **motor-off at compile time**.

## Key files

| Path inside archive | Responsibility |
|---|---|
| `src/main.cpp` | GPIO/input safety, I²C sensors, boot, FIFO, BMP/MAG updates, scheduler, UART and diagnostics |
| `include/flight_core.hpp` | Vector/matrix/quaternion math, calibration constants, airframe model, 7 PIDs, virtual motor mixer, health bits |
| `include/pid.hpp` | Filtered derivative, clamping, anti-windup |
| `include/altitude_kf.hpp` | 3-state [height, velocity, accel bias] Kalman with Joseph update |
| `include/bmp280_comp.hpp` | Bosch raw pressure/temperature compensation |
| `include/control_link.hpp` | $IQ2 CRC serial command decoder |
| `include/tofsense_frame0.hpp` | Optional LiDAR frame parser |
| `platformio.ini` | Build environment and safety flags |
| `tools/run_fc.ps1` | Upload vs capture, CH343 detection |
| `tools/xbox_uart_bridge.py` | Windows Xbox -> UART commands for virtual bench only |
| `tests/test_core.cpp`, `tests/test_project.py` | Host numerical and code-path tests |

## Control data path

`MPU FIFO + QMC + BMP (+optional LiDAR) -> SI conversion -> hardware residual / 3×3 matrices -> body alignment -> quaternion + height KF -> seven cascaded PIDs -> virtual X mixer -> FC log`.

It is not a real four-motor closed loop. **IQ_MOTOR_OUTPUTS_ENABLE=0** is checked and the controller explicitly refuses to clear itself for flight. Compile-time motor lock remains an essential safety constraint.

## Observability

`FC_BOOT`, `BOOT_BIAS`, `BOOT_BARO`, `FC`, `FC_TIMING`, and `FC_MODEL` contain health and timing fields. These are diagnostic telemetry, not an approval signal. Onboard timing and sensor success of v2 have not been supplied in the same way that the 4 Nov calibration raw capture was.
