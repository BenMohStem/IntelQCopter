# Review of the existing Drone project

Read-only source: C:\Users\Mohamed\Desktop\Work\Drone. Git baseline: 7bd54c1.
The inspected working main.cpp contains pre-existing uncommitted edits;
the snapshot preserves that working version, not just the older Git blob.
Inspected working files are copied with SHA256 provenance under archive/reference-drone/.
No environment, library cache, editor state or credentials were imported.

## Reused

ESP32-S3/Arduino target, Xbox BLE and MPU/BMP libraries, proposed wiring as an
unverified assumption, and the board-local nonvolatile calibration concept.

## Corrected or excluded

| Original sketch | New branch |
|---|---|
| RT directly drives throttle | Left vertical stick controls vertical speed; RT is boost |
| Five PID objects, no altitude PID | Six-loop C model from main, including altitude |
| Direct 50 Hz servo/ESC pulses | No servo library or physical outputs |
| Boolean timer flag loses tick count | Counted notifications, missed-cycle telemetry |
| Fixed .002 s dt | Measured dt with control-step rejection |
| Complementary expression with different weights | Main's independent angle/bias Kalman filters |
| Unprotected multi-variable pilot sharing | Complete callback snapshot under critical section |
| Connection state stands in for freshness | Timestamp of actual report; provisional age watchdog |
| Gyro-only independent sign flips | Identity transform explicitly awaiting mounting verification |
| One stored calibration-valid flag | Schema/profile/board/CRC/range checks and boot gyro verification |
| Level pose cannot identify accel scale | Six-face capture with variance/geometric checks |
| No complete burst validity gate | Data-ready and complete 14-byte register read checks |

The old attitude expression uses different gyro and error weights. At zero gyro rate,
it does not generally preserve a constant nonzero angle at equilibrium.
Its gains use servo-microsecond output units; the C core uses normalized demands.
Those numbers cannot be copied directly between implementations.
These are source-review findings, not flight experiments.

## Sources

- [Xbox library](https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32).
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino).
- [MPU6050 driver](https://github.com/ElectronicCats/mpu6050).
- [BMP280 driver](https://github.com/adafruit/Adafruit_BMP280_Library).
- [Preferences/NVS](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html).

Bluepad32 is an alternative C/ESP-IDF direction, not a second active stack.
Its [compatibility notes](https://bluepad32.readthedocs.io/en/latest/FAQ/) explain
why controller firmware and ESP32-S3 BLE-only support matter.
