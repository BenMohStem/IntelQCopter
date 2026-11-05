# Chronological engineering timeline

This is a version map of *parallel subprojects*, not a single monotonically increasing release line.

| When | Development/event | Result and important qualification |
|---|---|---|
| Earlier | RP2350/LoRa and ESP32-S3/Xbox/ROS2/Gazebo proposals | Distinct branches and prototypes; avoid conflating pinouts and runtimes |
| Oct 2026 | PCB work, modeling, preliminary motor limits and control scaffolding | Research/simulation; no flight demonstrated |
| Nov 1 | Engineering study of dynamics, math, filters and control | Several numbers marked synthetic or assumed |
| Nov 2 | 500 Hz full GUI sensor-calibration work | Binary stream, calibration sessions, PC analysis tooling |
| Nov 3 | QMC v12/v13 magnetometer isolation | **v13 100 Hz `0xC9` worked** at ~90 Hz in some logs, 200 Hz failed then |
| Nov 3 | QMC v14-v18 experiments | Slow/fast I²C, Wi-Fi interference, power cycles, COM and transaction failures; no stable universal repair |
| Nov 3 evening | Original v13 driver transplanted to full calibration | Restored a real path to successful integrated captures |
| Nov 4 morning | Complete calibration `100907` recording | 98,868 full data frames; live QMC, IMU, BMP; no LiDAR measurements |
| Nov 4 | Independent numeric calibration audit | Updated gyro/accel/mag constants, holdout tests, warnings about cross-axis confidence |
| Nov 4 | First 500 Hz FC integration v1 | Motor-disabled estimation/control model, initial mathematical tests |
| Nov 4/5 | Fast FC v2 | 500 Hz FIFO, BMP fast path, 400 kHz primary bus, 7 PIDs, UART Xbox bench bridge; no ESC |
| Nov 5 | This Markdown documentation snapshot | Documentation only; no firmware changes or flight test |

## Magnetometer turning points

**v13 100 Hz** and **Nov 4 `0x3D` 200 Hz** are different confirmed successful *sessions*, not a contradiction to erase: silicon configuration, bus mode, firmware scheduling and power state differed. Multiple intermediate combinations failed. Read `../02_SENSORS/02_QMC5883P.md` and `../07_TESTING/04_HISTORIC_FAILURES.md` before rewriting the driver.

## Project deadline context

The project owner previously reported a competition on 5 November 2026 and that the quadcopter had not yet flown. Do not use this deadline to lower arming, receiver-loss, motor-direction or thrust-margin safety requirements.
