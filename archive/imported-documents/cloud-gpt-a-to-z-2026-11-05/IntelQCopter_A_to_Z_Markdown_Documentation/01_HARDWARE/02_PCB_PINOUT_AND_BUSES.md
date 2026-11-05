# ESP32-S3 PCB and present pinout knowledge

| Signal | Source-level configuration or observation | Confidence |
|---|---|---|
| Main I²C SDA | ESP32 GPIO9 | In calibration/FC source; confirm schematic |
| Main I²C SCL | ESP32 GPIO8 | In calibration/FC source; confirm schematic |
| MPU6050 | I²C `0x68` (alternate `0x69` can be supported) | Observed at 0x68 |
| QMC5883P | I²C `0x2C`, ID `0x80` | Observed |
| BMP280 | `0x76`, optional `0x77`; attached on MPU XDA/XCL aux pins | Observed/implemented; depends on bypass |
| CH343 | Windows USB serial COM3 or COM4 | Observed; number not guaranteed |
| Historical motor GPIOs | FL, FR, RR, RL = GPIO12,11,14,13 | **Code annotation only**, output disabled |
| MPU INT / QMC DRDY | Not required for current software-polling path | No dedicated line necessary |
| LiDAR RX on ESP32 | Undefined in safe v2: `IQ_LIDAR_RX_PIN=-1` | OPEN |

## MPU bypass requirements

MPU6050 `USER_CTRL` register `0x6A=0x00` disables its internal I²C master. `INT_PIN_CFG` **register** `0x37=0x02` enables access to XDA/XCL through bypass. This is an I²C register named “INT_PIN_CFG”; it does not imply a physical interrupt wire.

## Design checks

Use a continuity tester, not only an IDE pin macro, to determine actual on-PCB routes. Confirm I²C pull-ups terminate at a legal logic voltage, not battery. Check BMP280 breakout CSB/SDO strapping if not detected. Verify all ground references including UART adapter ground. No motor pin should be connected to an unknown driver until safe-start behavior is established.
