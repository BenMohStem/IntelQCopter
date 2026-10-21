# ESP32 hardware interfaces — Arduino libraries

One application file: [flight_controller.cpp](../firmware/src/flight_controller.cpp). The mathematical core uses C-style structs/functions; its final guarded section provides setup/loop and existing Arduino libraries. PlatformIO uses framework=arduino on ESP32-S3.

| Interface | Library/API used |
|---|---|
| I²C sensors | Wire, 400kHz, SDA9/SCL8; bounded transaction timeout |
| MPU6050 | Electronic Cats MPU6050; Wire read status checked in acquisition |
| BMP280 | Adafruit BMP280 / BusIO / Unified Sensor |
| Xbox Series X/S | XboxSeriesXControllerESP32_asukiaaa + NimBLE |
| Calibration flash | Arduino Preferences / NVS, versioned CRC-protected record |
| Optional ROS2 UART | Arduino HardwareSerial (Serial2), RX18/TX17,115200,3.3V |
| Motor pins today | Arduino pinMode(INPUT_PULLDOWN); no pulses, attach or arm |
| PWM when hardware is authorized | Arduino-ESP32 built-in LEDC PWM; no separate PWM package required |

The installed Arduino core exposes ledcSetup, ledcAttachPin, ledcWrite and ledcDetachPin (confirmed in its local esp32-hal-ledc.h). We do not invoke them on motor pins while physical outputs are disabled. A future core upgrade must adapt these API calls to its installed version.

An ESC accepts its specified pulse protocol, not a servo angle. If the actual ESC supports conventional servo-style pulses, the future output adapter can use an ESP32-compatible Servo library or built-in LEDC with explicit microsecond mapping. Do not assume its PWM frequency from the 500Hz PID update rate; the ESC model/protocol is still unknown. This request changes the preferred interface style; it does not enable motor outputs or claim a working physical ESC driver.

The optional companion cannot bypass the FC: stable Xbox control works without it; selected navigation commands pass through the same control core. The real ADC, magnetometer and lidar drivers remain unfinished. The simulator uses synthetic sensor inputs and ideal localization, not those unfinished physical interfaces.
