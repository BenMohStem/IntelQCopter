# ESP32-S3 single-sided flight-controller carrier

## Current edition: photographed sensor headers, 23 October 2026

This replaces the earlier four-wire sensor ports. The old PCB, mask and README are preserved in `../../archive/hardware/2026-10-23-before-photo-headers/previous-carrier.zip`. Old extracted ZIP folders are **not** current fabrication files.

- [Open native KiCad PCB](esp32_home_etch.kicad_pcb) and [schematic](esp32_home_etch.kicad_sch).
- [Current complete KiCad ZIP](output/IntelQCopter-ESP32-home-etch-KiCad.zip).
- [Three identical text-free A4 masks, 1:1](output/pdf/etch-mask-3-copies-A4-1to1.pdf).
- [Assembly view](assembly-preview.png), [pin map](connector-map.md), [jumper list](wire-links.csv), [drill coordinates](drill-table.csv).
- [Current original audit results](../../artifacts/hardware/photo-headers-20261023/README.md).

**PCB: 100 x 100 mm; route and etch B.Cu only.** KiCad stores a two-copper-layer board because its file format uses paired copper layers; there are no front-copper routes, vias or plated-layer connections required. All crossings use insulated top wires. Ground is a large solid B.Cu fill, with solid pad joins and circular drill openings, no thermal crosses. Copper under the DevKit antenna region is removed.

## Headers and the photographs

Component-side view. Pin1 of each sensor row is its leftmost pad at x=8 mm; pin numbers increase to the right. These are **2.54 mm solder/header landings** in the photographed signal order, not verified body/mounting-hole models. Mount the sensor boards separately and solder wires signal-to-signal. Do not flip a module onto the carrier just because its pin count matches: the photo may show its opposite face, and its body may overlap neighbouring parts.

| Carrier | Module | Pins1 through last, left to right |
|---|---|---|
| J4, y=45 mm | GY-521 MPU6050, 8 pins | VCC=3.3V; GND; SCL=GPIO8; SDA=GPIO9; XDA unused; XCL unused; AD0=GND; INT unused |
| J5, y=55 mm | GY-BM280, 6 pins | VCC=3.3V; GND; SCL=GPIO8; SDA=GPIO9; CSB=3.3V **through W24**; SDO=GND |
| J6, y=75 mm | GY-271/HW-246, 5 pins | VCC=3.3V; GND; SCL=GPIO8; SDA=GPIO9; DRDY unused |

The MPU screenshot reads INT,AD0,XCL,XDA,SDA,SCL,GND,VCC from its shown left to right. Start wiring from the printed **VCC** end; do not count from the photograph's left edge. The barometer photo reads VCC,GND,SCL,SDA,CSB,SDO from top to bottom. The magnetometer in the hand photo reads VCC,GND,SCL,SDA,DRDY top to bottom. Unused pins get isolated solder pads; they are not joined to the ground plane.

BMP280 CSB must be high for I2C; W24 provides that connection. SDO low selects0x76. MPU AD0 low selects0x68. The GY-271 label does not establish which magnetometer silicon a seller fitted; confirm HMC5883L versus QMC5883L by chip ID/address before selecting its driver. No firmware driver completion is claimed here.

### LiDAR J13: four large wire-solder holes

The official TOFSense F2 Mini uses four wires. This carrier keeps generous5.08mm spacing,3.2mm round pads and1.2mm drills, so no tiny GH connector is needed on the PCB.

| J13 pin | Carrier signal | Connect LiDAR wire labelled |
|---|---|---|
| 1 | Common GND | GND |
| 2 | Regulated5V via W23 | VCC |
| 3 | FC RX / GPIO6 | LiDAR TX |
| 4 | FC TX / GPIO7 | LiDAR RX |

This is a wire landing order, **not** the pin order of the tiny LiDAR plug. Follow the signal labels and official plug diagram. The real F2 Mini reference is four pins; a different five-wire device needs its model/label checked before its fifth wire is assigned. Do not guess a fifth power or mode signal.

## ESP32-S3 assignments

The carrier fits the genuine/reference **Espressif ESP32-S3-DevKitC-1, 2x22 pins**,22.86mm row spacing,2.54mm pitch. A different ESP32-S3 clone is not guaranteed to fit. The chip datasheet specifies GPIO functions; the **DevKit** documentation determines header positions. All four published DevKit GND header pins connect directly to the common copper.

| Function | GPIO | Notes |
|---|---|---|
| Shared I2C SDA / SCL | 9 / 8 | 3.3V pull-ups only |
| ESC FL / FR / RR / RL signal | 12 / 1 / 2 / 14 | Through220ohm; ESC grounds common, no ESC power carried |
| Companion UART TX / RX | 17 / 18 | J12 pins1/2; companion RX/TX respectively; pin3GND |
| LiDAR UART RX / TX | 6 / 7 | J13 pins3/4; UART levels3.3V |
| Battery voltage ADC / current ADC | 4 / 5 | Low-current measurement only |

Assignments keep the existing project firmware GPIO mapping; **no firmware changes are included in this PCB revision**. Avoid using a raw ESP32 chip pin number as a DevKit header number. Boot straps, USB and flash/PSRAM pins are not used for these signals. The current project LiDAR/magnetometer hardware interfaces remain unvalidated. Motor arming remains disabled.

## Supply and assembly

1. Fit **all14 insulated wire links** exactly as `wire-links.csv`, including W24 for BMP280 CSB. A colour line in the assembly image is a real insulated wire, never a copper track on the component side.
2. J3 accepts an **external regulated5V** source. Never connect a3S pack directly there. Motor/ESC current goes through the separate rated PDB. This carrier is not a high-current battery switch or BMS.
3. W21 connects external5V to the DevKit5V input. **Remove W21 before powering the DevKit by USB**. W23 supplies LiDAR5V from the external regulator, not from the3.3V sensor rail.
4. Sensor VCC and I2C logic are3.3V. Check voltage at each actual module's chip supply after its onboard regulator; unknown breakout LDOs may drop too much voltage from3.3V input. No5V I2C pull-up is acceptable at ESP32 GPIOs. Check combined parallel module pull-ups; remove redundant ones if the measured equivalent resistance exceeds the bus sink-current capability. At3.3V,1kohm draws about2.9mA at a0.4V low level.
5. J7 is pack-voltage **sense only**.47kohm/10kohm gives approximately2.21V at12.6V pack voltage, followed by1kohm/100nF filtering. Disconnect sense when the MCU is unpowered; no powered-off clamp isolation is provided. J14 expects a conditioned0-2.5V current signal; its external transducer has not been selected.
6. Use the fit/drill sheet and actual parts before etching.2.54mm rows require smaller1.94mm pads/1.1mm holes to preserve0.60mm copper gaps. Other wire pads are3.2mm/1.2mm. Tracks are at least0.8mm; signal tracks normally1mm. Hand drilling still needs centring.

## Print and etch

Print the **current** PDF with **Actual size /100%**, no fit-to-page. It contains3 copies, no text. Black is retained copper, white is removed copper, and drill centres stay white. The artwork is component-view orientation; pressing the toner side face-down onto bottom copper supplies the required reversal. Do not apply a second mirror. Measure22.86mm between the two DevKit header rows, or the separate20mm calibration ruler before transfer; there is no board-edge line in the text-free mask. Digital1:1 scale is checked; your printer's physical output is not measured here.

Refresh the native B.Cu zone in KiCad with `B` after editing. For a native plot use B.Cu only, black positive, board scale1.0, disable footprint values/references and plot drill marks. The delivered PDF is generated directly from filled native copper and subtracts complete white drill circles; it does not include silkscreen or thermal relief crosses.

## Evidence and remaining limits

Native KiCad DRC/ERC, schematic/netlist comparison and rendered-mask checks report actual results in the dated evidence directory. They check the digital board and explicit wire links. They do not validate unknown module internals, physical fit, regulator performance, a manufactured board, RF behaviour or flight safety. No flight/hardware test was performed for this PCB correction.

References: [Espressif DevKit header map](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html), [ESP32-S3 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf), [Bosch BMP280 I2C circuit](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf), [Nooploop wire sequence](https://support.nooploop.com/tofsense/hardware/). Sensor breakout signal order additionally follows the user photos; personal screenshots are not published.
