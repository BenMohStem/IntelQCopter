# QCopter KiCad add-on library

This folder supplements the KiCad 10 libraries installed on this PC. It does not copy or fork the standard libraries.

## Sensors

| Part requested | KiCad library entry | What it represents | Package dimensions |
|---|---|---|---|
| BMP280, 6-pin breakout | `Sensor_Pressure:BMP280` | Bare BMP280 IC, 8-pad LGA | 2.0 × 2.5 × 0.95 mm |
| QMC5883P, 5-pin breakout | `QCopter_Sensors:QMC5883P_IC` | Bare QMC5883P IC, 16-pad LGA | 3.0 × 3.0 × 0.9 mm |
| MPU6050, 8-pin breakout | `Sensor_Motion:MPU-6050` | Bare MPU-6050 IC, 24-pad QFN | 4.0 × 4.0 × 0.9 mm |

These are chip symbols and footprints, not breakout-board outlines. The six-, five-, and eight-pin modules have no single standard outline: published boards with the same pin count differ in board size, pin order, hole position, and component placement. The breakout modules are deliberately not assigned invented dimensions or 3D models. Once the exact board/product links or photos with a ruler are known, add variant-specific footprints and models.

## Raspberry Pi Pico modules

KiCad's `MCU_Module:RaspberryPi_Pico` and `MCU_Module:RaspberryPi_Pico_W` symbols and `Module:RaspberryPi_Pico_Common_THT` footprint cover Pico/Pico W layouts; the common 40-pin footprint also matches Pico 2's header locations. `Module:RaspberryPi_Pico_Common_SMD` and the dedicated hand-solder footprints are available for surface-mounted variants. KiCad includes Pico and Pico W STEP models. The official Pico 2 STEP supplied here is under `3dmodels/` with its upstream license. Do not assign that model to Pico 2 W: the downloaded file identifies the non-W Pico 2, and the radio/antenna layout differs.

Board-size verification for the Pico family: (1) Raspberry Pi mechanical drawings specify 51 × 21 mm; (2) the standard KiCad common footprint outline measures 21.00 mm across by 51.00 mm long from its F.Fab board-edge geometry; (3) the official Pico 2 STEP is retained for a CAD overlay check before it is assigned to a footprint. Pico 2 W's antenna keepout is 14 × 9 mm and must remain clear. The module footprint must not be reduced to just the 2.54 mm header grid.

## Headers

Use standard `Connector_Generic:Conn_01x02` through `Conn_01x10` schematic symbols and `Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical` through `...1x10...` for single-row 2.54 mm through-hole headers. The matching KiCad footprints include 3D header models. This add-on does not replace those standard entries.

## Dimension validation status

For every new or copied footprint, dimensions must agree across (1) manufacturer mechanical drawing/datasheet, (2) CAD or STEP geometry where available, and (3) a numeric measurement of the KiCad footprint's board-edge and pad coordinates. Pin pitch and mounting-hole center coordinates are checked separately. IC package dimensions are not substituted for breakout-board dimensions. Sensor breakout footprints remain pending exact manufacturer variants; do not fabricate a PCB from the IC footprints when the design intends to mount a breakout module.

## Upstream references

- [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [QST QMC5883P datasheet](https://cdn-shop.adafruit.com/product-files/6388/C2847467.pdf)
- [TDK/InvenSense MPU-6050 product specification](https://www.es.co.th/Schemetic/PDF/MPU6000.PDF)
- [Raspberry Pi Pico documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [Raspberry Pi Pico 2 datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf)
- [Raspberry Pi Pico 2 W datasheet](https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf)
- [Pico 2 STEP source](https://pip-assets.raspberrypi.com/categories/1005-raspberry-pi-pico-2/documents/RP-009061-CA-2-Pico%202%20-%203D%20STEP.zip)
- [GY-BMP280 module example: 16 × 12 × 3 mm](https://kamami.pl/en/pressure-sensors/561435-gy-bmp280-module-with-bosch-mems-bmp280-sensor-5906623455270.html)
- [QMC5883P module example: Adafruit product 6388](https://www.adafruit.com/product/6388)
- [GY-521 module example](https://www.electronotics.com/static/datasheets/gy-521_mpu-6050_3-axis_gyroscope_and_acceleration_sensor_en_3.pdf)
