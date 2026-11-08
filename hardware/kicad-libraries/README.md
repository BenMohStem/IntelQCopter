# QCopter KiCad add-on library

This folder supplements the KiCad 10 libraries installed on this PC. It does not copy or fork the standard libraries.

## Sensors

| Part | KiCad library entry | What it represents | Package dimensions |
|---|---|---|---|
| BMP280 IC | `Sensor_Pressure:BMP280` | Bare BMP280 IC, 8-pad LGA | 2.0 × 2.5 × 0.95 mm |
| BMP280 6-pin module | `QCopter_Sensor_Modules:BMP280_Module_6Pin` | Generic numbered header row; 2.54 mm pitch | Board outline intentionally omitted |
| QMC5883P IC | `QCopter_Sensors:QMC5883P_IC` | Bare QMC5883P IC, 16-pad LGA | 3.0 × 3.0 × 0.9 mm |
| Magnetometer 5-pin module | `QCopter_Sensor_Modules:Magnetometer_Module_5Pin` | Generic numbered header row; linked product is HMC5883L GY-273 | Board outline intentionally omitted |
| MPU6050 IC | `Sensor_Motion:MPU-6050` | Bare MPU-6050 IC, 24-pad QFN | 4.0 × 4.0 × 0.9 mm |
| MPU6050 8-pin module | `QCopter_Sensor_Modules:MPU6050_Module_8Pin` | Generic numbered header row; 2.54 mm pitch | Board outline intentionally omitted |

The module symbols are `Magnetometer_Module_5Pin`, `BMP280_Module_6Pin`, and `MPU6050_Module_8Pin` in `QCopter_Sensor_Modules`. Their footprints are in `QCopter_Sensor_Modules.pretty` and use the stated pin count in one straight row. Pin names are generic (`Pin 1`, etc.) so they can be mapped to the actual board silkscreen.

All three module headers use **2.54 mm center-to-center pitch** between adjacent pins, the standard 0.1-inch header spacing requested. The first-to-last pin-center spans are 10.16 mm for 5 pins, 12.70 mm for 6 pins, and 17.78 mm for 8 pins. The linked magnetometer seller explicitly specifies 2.54 mm pitch; its BMP280 and MPU6050 product pages do not publish a pitch, so those two footprints use the common header standard and should be checked against the physical boards if they are clone variants. Each footprint reuses KiCad's standard through-hole pad geometry and matching straight-header 3D model. The 3D model shows the header only; the breakout PCB outline and sensor body are intentionally omitted because the product pages do not provide verified mechanical drawings for all three modules.

The magnetometer product link identifies an HMC5883L GY-273 module, not a QMC5883P. The schematic symbol therefore uses the generic magnetometer name. The linked seller specifies a 16 × 18 mm board and 2.54 mm pin pitch. The other two product pages identify BMP280 and GY-521/MPU6050 modules but do not publish complete mechanical drawings. These assets represent the requested pin rows, not a guaranteed board perimeter or an assertion that the magnetometer IC is QMC5883P.

## Raspberry Pi Pico modules

KiCad's `MCU_Module:RaspberryPi_Pico` and `MCU_Module:RaspberryPi_Pico_W` symbols and `Module:RaspberryPi_Pico_Common_THT` footprint cover Pico/Pico W layouts; the common 40-pin footprint also matches Pico 2's header locations. `Module:RaspberryPi_Pico_Common_SMD` and the dedicated hand-solder footprints are available for surface-mounted variants. KiCad includes Pico and Pico W STEP models. The official Pico 2 STEP supplied here is under `3dmodels/` with its upstream license. Do not assign that model to Pico 2 W: the downloaded file identifies the non-W Pico 2, and the radio/antenna layout differs.

The user-provided official Pico 2 datasheet specifies a **micro-USB** connector, not USB Type-C. It is the correct source for the official Pico 2's 51 × 21 mm board and 40-pin layout, but it cannot validate a third-party Type-C board's connector outline. The common Pico footprint is therefore the official micro-USB layout; use a separate board drawing for a Type-C clone before relying on its connector clearance.

Board-size verification for the Pico family: (1) Raspberry Pi mechanical drawings specify 51 × 21 mm; (2) the standard KiCad common footprint outline measures 21.00 mm across by 51.00 mm long from its F.Fab board-edge geometry; (3) the official Pico 2 STEP is retained for a CAD overlay check before it is assigned to a footprint. Pico 2 W's antenna keepout is 14 × 9 mm and must remain clear. The module footprint must not be reduced to just the 2.54 mm header grid.

## Headers

Use standard `Connector_Generic:Conn_01x02` through `Conn_01x10` schematic symbols and `Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical` through `...1x10...` for single-row 2.54 mm through-hole headers. The matching KiCad footprints include 3D header models. This add-on does not replace those standard entries.

## Dimension validation status

Validation: the sensor footprints have exact pad counts and 2.54 mm spacing, measured from the KiCad pad coordinates and matched to the standard 2.54 mm header family; the magnetometer seller page independently states 2.54 mm pitch. Each row's first-to-last span is also checked as `(pin count − 1) × 2.54 mm`. KiCad CLI exported each new symbol and footprint successfully. Do not use these pad-row footprints as the complete breakout-board perimeter; add exact board outlines only when the seller supplies a dimensioned drawing or CAD model.

## Upstream references

- [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [QST QMC5883P datasheet](https://cdn-shop.adafruit.com/product-files/6388/C2847467.pdf)
- [TDK/InvenSense MPU-6050 product specification](https://www.es.co.th/Schemetic/PDF/MPU6000.PDF)
- [Raspberry Pi Pico documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [Raspberry Pi Pico 2 datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf)
- [Raspberry Pi Pico 2 W datasheet](https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf)
- [Power Lab BMP280 module](https://powerlab.dz/product/module-bmp280-capteur-de-pression-barometrique-ambiante/)
- [Power Lab GY-521 MPU6050 module](https://powerlab.dz/product/module-accelerometre-gyroscope-3-axes-mpu6050-6/)
- [Power Lab HMC5883L GY-273 magnetometer module](https://powerlab.dz/product/module-boussole-3-axes-hmc5883l/)
- [Pico 2 STEP source](https://pip-assets.raspberrypi.com/categories/1005-raspberry-pi-pico-2/documents/RP-009061-CA-2-Pico%202%20-%203D%20STEP.zip)
- [GY-BMP280 module example: 16 × 12 × 3 mm](https://kamami.pl/en/pressure-sensors/561435-gy-bmp280-module-with-bosch-mems-bmp280-sensor-5906623455270.html)
- [QMC5883P module example: Adafruit product 6388](https://www.adafruit.com/product/6388)
- [GY-521 module example](https://www.electronotics.com/static/datasheets/gy-521_mpu-6050_3-axis_gyroscope_and_acceleration_sensor_en_3.pdf)
