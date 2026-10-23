# Current sensor-header correction

The23October photo-matched revision replaces generic four-wire sensor ports with8/6/5-pin2.54mm header landings. Module bodies stay offboard on wires; neither body dimensions nor mounting holes can be measured precisely from these photos. See [current wiring and orientation guide](README.md). The earlier manufacturer-reference dimensions below still apply to the DevKit and passives; previous statements about four-pin sensor ports are superseded.

# Selected component dimensions and fit

All dimensions are millimetres. These are explicit published reference choices, selected because the exact purchased modules cannot be identified. They are not measurements of the user's parts. Sources are manufacturer documents, not marketplace guesses.

## Parts actually fitted to the carrier

| Part / reference | Published mechanical data | Carrier implementation |
|---|---|---|
| Espressif ESP32-S3-DevKitC-1 V1 | Body25.40 x62.74; 22 pins per row,2.54 pitch | Row centres22.86 apart; body rectangle corrected |
| Samtec SSQ THT socket reference | 2.54 pitch; recommended PCB hole1.02 | 1.10 manual drill,1.94 pads;0.42 annulus /0.60 gap |
| Vishay MRS25 / DIN0207 resistor reference | Body maximum6.5 x diameter2.5; nominal lead0.6; minimum bent span10.0 | 10.16 lead span;1.2 drill/3.2 pad |
| Vishay K-series size15 H5 X7R100nF reference | Body maximum4.0 x2.6 x4.0; lead0.5; formed lead pitch5.0 | C1/C2 pitch5.00,1.2 drill/3.2 pad; C2 reversed left for USB access |
| Wire and jumper solder landings | These are designed wire holes, not bought connector footprints | External3.2/1.2 pad/drill; insulated jumpers2.8/1.2 |

The MCU row spacing follows25.40 -2(1.27) =22.86. Its first pad-to-last-pad distance is21(2.54) =53.34. From the mechanical drawing, top edge-to-first-pad is62.74 -8.00 -53.34 =1.40. Relative to carrier first pad(40,14), the body spans(38.73,12.60) to(64.13,75.34). The antenna can overhang the base PCB; the larger RF notch is retained. The red USB service rectangle is a conservative design envelope, not a measured plug body.

All44 carrier socket positions are independently compared with Espressif's footprint (tolerance0.01mm accommodates tiny coordinate rounding in that file). Hole and copper pad dimensions are intentionally enlarged for this non-plated hand-etched board rather than copied from a factory pad.

## Remote sensor examples — wire-connected, not mounted by these holes

| Published reference | Body size | Consequence |
|---|---|---|
| SparkFun SEN-11028 MPU6050 | 25.5 x15.2 x2.48 | External pigtail; not a GY-521 footprint claim |
| Adafruit2651 BMP280 reference | 19.2 x17.9 x2.9 | Revision-dependent shape; wire port avoids clone geometry assumptions |
| DFRobot SEN0102 HMC5883L (retired) | 17.78 x17.78 | Optional wire-connected reference only |
| Nooploop TOFSense-F2 Mini V1.0 | 19 x12 x10.3 | External cable; connector pitch is not carrier hole pitch |

No sensor-module mounting holes are inferred from these dimensions. Module pin order, orientation, pull-ups and supply compatibility must be mapped to GND/3V3/SDA/SCL (or specified UART pins). The two-sensor firmware needs only MPU6050 and BMP280. F450 carrier mounting, actual module/plug height, RF behavior and fabricated board accuracy remain unmeasured.

## Primary documents

- [Espressif mechanical drawing](https://dl.espressif.com/dl/DXF_ESP32-S3-DevKitC-1_V1_20210312CB.pdf) and [Espressif KiCad footprint](https://raw.githubusercontent.com/espressif/kicad-libraries/main/footprints/Espressif.pretty/ESP32-S3-DevKitC.kicad_mod).
- [Samtec recommended socket layout](https://suddendocs.samtec.com/prints/ssq-1xx-xx-xx-x-xx-xxx-xx-xx.pdf).
- [Vishay MRS25 drawing](https://www.vishay.com/docs/28724/mrs16m25.pdf) and [Vishay K-series drawing/order codes](https://www.vishay.com/docs/45171/kseries.pdf).
- [SparkFun MPU6050](https://www.sparkfun.com/sparkfun-triple-axis-accelerometer-and-gyro-breakout-mpu-6050.html), [Adafruit BMP280](https://www.adafruit.com/product/2651), [DFRobot magnetometer](https://www.dfrobot.com/product-535.html), [Nooploop F2 Mini](https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf).
