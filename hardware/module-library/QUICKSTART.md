# QCopter: modules ready for your own PCB design

The schematic and PCB contain **nine unwired/unrouted parts**. Local libraries
travel with the project; no global KiCad library installation is needed.

| Reference | Part | Physical interface |
|---|---|---|
| U1 | ESP32-S3-DevKitC-1 reference development board | 2 x 22 pins, 2.54 mm pitch, 22.86 mm between rows |
| U2 | MPU6050 GY-521-style module | 1 x 8 pins, 2.54 mm pitch |
| U3 | Purple GY-BMP280-style module | 1 x 6 pins, 2.54 mm pitch |
| U4 | Your photographed GY-271-style magnetometer module | 1 x 5 pins, 2.54 mm pitch |
| J1 | Remote lidar wire solder holes | 4 holes, 5.08 mm spacing, 3.2 mm pads, 1.2 mm drills |
| J2-J5 | Four ESC control plugs | 1 x 3 pins each, 2.54 mm pitch |

Header/module pads have 2.0 mm copper diameters and 1.1 mm drills. Pad 1 is
square to identify orientation. Other pads are round; drill holes are empty.
The three sensor footprints now include sourced nominal body rectangles and
2 mm placement margins. Exact clone fit/header offsets remain unverified. See
[module dimensions and orientation](module-dimensions.md). No guessed mounting holes were added.

## Open and use

1. Open `QCopter.kicad_pro`, then the schematic or PCB editor.
2. The parts are already placed. Connect the schematic with **W**, then **F8** to
   update PCB nets. The placed footprints have matching schematic UUID paths.
3. To add another symbol: press **A** in the schematic and search `QCopterModules`.
4. To add a footprint directly: press **A** in the PCB editor and search the same
   library. In the Footprint Editor, select the project library `QCopterModules`.
5. Move with **M**, rotate with **R**. Enable **F.Fab** in the layer display to see
   the per-pad signal labels, including every ESP32 pin.
6. For your single-copper-face process, route on **B.Cu** and later plot that
   copper layer. Through-hole footprints normally declare both copper sides;
   that does not require you to route a front copper layer. Choose the outline,
   GND pour and routing yourself before fabrication.

No board outline, copper traces, GND pour or electrical connections are supplied.
This is your starting component palette, not a fabrication-ready board. ERC/DRC
unconnected-pin reports are expected until you complete the design.

## Pin order: top/component-side view, pad numbers as shown in KiCad

- **U2 MPU6050:** 1 VCC, 2 GND, 3 SCL, 4 SDA, 5 XDA, 6 XCL, 7 AD0, 8 INT.
- **U3 BMP280:** 1 VCC, 2 GND, 3 SCL, 4 SDA, 5 CSB, 6 SDO.
- **U4 magnetometer:** 1 VCC, 2 GND, 3 SCL, 4 SDA, 5 DRDY. Unlike the BMP280,
  this photographed module has five header positions. The footprint does not
  determine whether a purchased clone contains an HMC5883L or a different chip.
- **J1 lidar:** 1 VCC_5V, 2 GND, 3 TX_FROM_LIDAR (connect to FC RX),
  4 RX_TO_LIDAR (connect to FC TX). This is our freely wired carrier order,
  NOT a claim about the lidar's own connector/cable order. Map each wire by signal.
- **J2-J5 ESC:** 1 GND, 2 BEC_5V_OPTIONAL, 3 SIGNAL. Check your plug orientation.
  These are the **control cables**, not the 40 A motor/battery connections.
  Decide which BEC, if any, supplies the board; do not automatically join four
  independent BEC outputs. An ESC without a BEC may have no useful middle wire.

U1 pin numbers match Espressif's reference footprint: left top-to-bottom 1-22;
right top-to-bottom 44-23. Its 44 pad coordinates were checked against that source
within 0.004 mm. Exact identification of your physical DevKit clone is still
unconfirmed: compare the two header rows before drilling. In particular, don't
substitute the bare ESP32-S3-WROOM module footprint for this development board.

The photo-matched sensor orders come from your supplied BMP280/GY-521/GY-271
images. Check the text printed on the actual modules before soldering; a chip
datasheet alone does not specify a third-party breakout board's dimensions.

## Sources and verification

- [Espressif official reference footprint](https://github.com/espressif/kicad-libraries/blob/main/footprints/Espressif.pretty/ESP32-S3-DevKitC.kicad_mod)
- [Espressif board pin guide](https://documentation.espressif.com/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)
- [Espressif mechanical drawing](https://dl.espressif.com/dl/DXF_ESP32-S3-DevKitC-1_V1_20210312CB.pdf)
- [AZ-Delivery GY-521 module pin guide](https://www.electronotics.com/static/datasheets/gy-521_mpu-6050_3-axis_gyroscope_and_acceleration_sensor_en_3.pdf)
- [Nooploop F2 Mini datasheet](https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf): 4.3-5.2 V supply, 3.3 V interface levels.

KiCad 10.0.6 natively loads the schematic, exports its netlist and reloads all
nine footprints/79 pads. `parts.json` records the pin mappings and reference hash.
Native SVG/PNG previews are in `preview/`. Original project files are preserved
in the timestamped `backups/before-module-palette-*` directory.

From the Stepwise software checkout, you can repeat the native CAD check:

```powershell
.\qcopter.ps1 check-pcb-palette
.\qcopter.ps1 check-pcb-palette --project C:\Users\Mohamed\Documents\Projects\PCB\QCopter
```

The second command checks your project. After you add wires/traces, the palette's
zero-routing assertions no longer apply; use your completed-design ERC/DRC instead.

The library uses passive electrical pin types as an interface palette. Refine electrical pin types for full ERC once you choose the power architecture; matching pad numbers alone does not validate electrical wiring.

For the current body outlines and how to update your open board, read [module-dimensions.md](module-dimensions.md).
