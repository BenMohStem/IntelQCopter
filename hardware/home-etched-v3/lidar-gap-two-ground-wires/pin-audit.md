# ESP32-S3 pin and interface audit

**Result: the selected pins are electrically capable of their assigned roles.**
This is a primary-documentation + CAD audit, not a hardware waveform or sensor test.
The native schematic and every PCB pad are checked independently.

| Function | ESP GPIO | Carrier pad | Check |
|---|---|---|---|
| MPU6050 SDA, Wire/controller 0 | 9 | U1.15 | Bidirectional GPIO, open-drain I2C through GPIO matrix |
| MPU6050 SCL, Wire/controller 0 | 8 | U1.12 | Bidirectional GPIO, I2C clock |
| BMP280 + GY-271 SDA, Wire1/controller 1 | 41 | U1.38 | I2C capable; do not also use external JTAG on this pin |
| BMP280 + GY-271 SCL, Wire1/controller 1 | 40 | U1.37 | I2C capable; external JTAG must not compete |
| MPU INT | 13 | U1.19 | Input/interrupt capable |
| Compass DRDY | 47 | U1.28 | Input/interrupt capable; actual compass chip identity remains to be verified |
| FC UART RX, from lidar TX (J1.3) | 10 | U1.16 | UART input routable through GPIO matrix |
| FC UART TX, to lidar RX (J1.4) | 11 | U1.17 | UART output routable through GPIO matrix |
| ESC1 PWM, J5.3 | 4 | U1.4 | Output capable; LEDC PWM routable |
| ESC2 PWM, J2.3 | 1 | U1.41 | Output capable; LEDC PWM routable |
| ESC3 PWM, J4.3 | 14 | U1.20 | Output capable; LEDC PWM routable |
| ESC4 PWM, J3.3 | 21 | U1.27 | Output capable; LEDC PWM routable |

Carrier numbers U1.1..44 identify this footprint, not two manufacturer connector
numberings or GPIO numbers. The actual signal names above match Espressif's header
tables. None of the selected signals uses strapping GPIO0/3/45/46, memory GPIO26..37,
USB GPIO19/20, default UART0 GPIO43/44 or the variant's GPIO38/48 RGB LED pins.
GPIO40/41 share alternate external JTAG functions; assign them to I2C and keep
external JTAG probes disconnected from those pins. Native USB pins remain free.

ESP32-S3 has two hardware I2C controllers and eight LEDC channels: two I2C buses
and four independent ESC PWM duties fit. This board does not require SPI. Each
I2C bus needs effective pullups to 3.3 V. Modules commonly have them, but inspect
the actual boards and measure bus-high voltage/rise time; parallel pullups combine.
Do not use 5 V I2C pullups on the ESP32. Start the slow bus at 100 kHz and retain
the verified MPU bus at 400 kHz, subject to real electrical checks.

MPU AD0 (module pin 7) is grounded: 0x68. BMP CSB (module pin 5) is tied to 3V3
for I2C selection. BMP SDO (module pin 6) is grounded for 0x76 and is a common-GND
wire attachment pad. BMP supply ground (module pin 2) joins it through G1. Compass
VCC/GND/SCL/SDA/DRDY follows the supplied five-pin board photo; HMC/QMC clone
identity/address/driver is not established by that label alone.

## What still needs software or hardware confirmation

The active firmware is a sensor bench, not a flight runtime. Its
`Wire.begin(9, 8, 400000)` matches this MPU bus, and it generates no motor PWM.
The existing motor INPUT_PULLDOWN list is 12,1,2,14; this PCB's future ESC map
is **4,1,14,21**. That safety-pin list and future output mapping must be updated
before connecting/enabling ESC signals. Do not load older flight firmware assuming
it matches this PCB. Calibration and the current bench source were not changed.

Future Arduino initialization uses `Wire.begin(9,8,400000)` and
`Wire1.begin(41,40,100000)`. Lidar RX10/TX11 must be passed explicitly to the
chosen HardwareSerial instance. The companion computer has no dedicated connector
in the user's nine-part sketch; GPIO16 RX / GPIO17 TX are free candidates via the
DevKit headers, with a separate supply and common GND. This is not an implemented
or tested ROS link. Old companion GPIO4 must not be reused: it is ESC1 here.

Four PWM outputs can use separate LEDC channels at one supported ESC pulse rate.
The 1 kHz FC calculation rate does NOT imply 1 kHz conventional ESC pulse frames:
a 1 ms frame cannot contain a 2 ms pulse. The exact 40 A ESC model/protocol,
accepted update rate and 3.3 V input threshold must be confirmed. Motor arming
and all PWM output remain disabled; no electrical capability test substitutes
for those hardware measurements.

## Primary sources checked

- [Espressif DevKitC-1 header and exclusive power options](https://documentation.espressif.com/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)
- [ESP32-S3 GPIO matrix and reserved pins](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)
- [Two I2C controllers, pullups and clock limits](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html)
- [Arduino Wire API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html)
- [Arduino UART pin assignment](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- [LEDC PWM and ESP32-S3 channel count](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [Bosch BMP280 I2C mode/address straps](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)

New Arduino 3.x LEDC attachment API differs from the project's Arduino 2.0.17;
pin capability is unchanged. Future implementation must use the installed core's API.
