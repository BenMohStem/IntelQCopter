# Sensor suite, buses and multirate schedule

The complete current FC is a four-sensor design: MPU6050 gyro/accelerometer, QMC5883P three-axis magnetic field, BMP280 pressure/temperature, and optional Nooploop TOFSense F2 Mini downward range. These measurements cannot all be fresh at 500 Hz.

| Sensor | Nominal physical setting in FC v2 | Host scheduling | Purpose |
|---|---|---|---|
| MPU6050 | 500 Hz FIFO at 188/184 Hz DLPF | 2 ms loop, drain bounded backlog | Angular rate and 3D specific force |
| QMC5883P | `0x3D`, ~200 Hz ODR | Poll status every 4 ms | Heading observation, anomaly-aware |
| BMP280 | 166.7 Hz typical ×1/×1 normal mode | Poll ~6 ms | Relative pressure-derived altitude |
| TOFSense F2 Mini | Up to 50 Hz | Optional UART parser | Ground range with quality and tilt gating |

## Time should be carried with data

Each measurement needs an acquisition timestamp, validity flag, freshness/age, configuration ID and optional covariance or quality statistic. A 500 Hz main loop can use the **most recent valid** magnetometer/pressure sample and apply an estimator correction only on fresh arrivals. It must not count 500 repeated barometer values as 500 independent observations.

## Bus topology

ESP32-S3 main I²C GPIO9 SDA / GPIO8 SCL; the BMP resides behind the MPU auxiliary bypass on XDA/XCL. The QMC shares the primary electrical bus. FC v2 is configured for 400 kHz for MPU/BMP and falls back to 100 kHz for QMC read transactions after problems. This is a source-level feature, **not a measured optimal bus profile**.

## Startup order and lock

No ESC output until all prerequisite sensors are recognized, calibrated and assigned physically verified body axes, while input and loop timing meet defined limits. Current v2 enforces a permanent motor-off compile-time guard.
