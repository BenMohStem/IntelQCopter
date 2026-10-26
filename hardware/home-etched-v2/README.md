# Single-sided ESP32-S3 carrier PCB - revision 2

**Start with [the interactive assembly guide](output/assembly-guide.html).**
Open `QCopter.kicad_pro` in KiCad, then its PCB editor. This is a new routed project;
the previous connected reference and the user's original Documents project are preserved.

| Deliverable | File |
|---|---|
| Native KiCad project | [QCopter.kicad_pro](QCopter.kicad_pro) |
| PCB | [QCopter.kicad_pcb](QCopter.kicad_pcb) |
| Connected schematic, including every wire link | [QCopter.kicad_sch](QCopter.kicad_sch) |
| Three identical actual-size toner-transfer masks, no text | [Print PDF](output/etch-mask-3-copies-A4-1to1.pdf) |
| Component view and selectable wire endpoints | [Assembly guide](output/assembly-guide.html) |
| Drill coordinates | [drill-map.csv](output/drill-map.csv) |
| Native electrical result | [DRC](validation/native-drc.json), [graph check](verification.json) |

## Board and fabrication

- Board: **90 x 90 mm**, copper on the underside **B.Cu only**; no top tracks or vias.
- Signals: 0.6 mm tracks. Power/ground/BEC tracks: 1.0 mm. Copper clearance: 0.5 mm minimum.
- Module/header pads: 2.0 mm round copper, 1.1 mm drill. Wire links: 2.4 mm copper, 1.1 mm drill.
- Remote lidar: four 3.2 mm solder pads, 1.2 mm drills, 5.08 mm pitch.
- Four 3.2 mm mounting holes at (4,4), (86,4), (4,86), (86,86) mm. This is a carrier pattern;
  **it is not a measured F450 mounting pattern**. Use a suitable adapter/support.
- Large GND pour with **solid** pad connections. White holes are empty; no copper crosshairs.
- **W1-W27 are mandatory insulated wires**, placed on the component side and soldered underneath.
  They replace crossings that a one-face etch cannot make. There are 27 wires, not a second copper face.
  In particular W1/W2 also bridge ground sections: do not omit them because a pad looks surrounded by copper.
- Fit the ESP32 and all three sensor modules into **raised female sockets**. Keep wires flat,
  insulated and mechanically secured; verify **at least 3 mm free space above every wire**.
  Choose socket height from the actual underside parts; nominal 8.5 mm sockets are a starting point.
  The KiCad courtyards describe the sockets at carrier level; full module outlines remain on F.Fab.
  A flat wire under an elevated module occupies a different height. Direct flush-mounted modules are unsuitable.
- Keep the marked USB plug corridor clear and the antenna area free of added copper, wires and metal.
  The mask has no copper beneath the nominal antenna footprint. Keep motor wiring away from the compass/IMU.
- KiCad represents through-hole pads on both face layers, but **only B.Cu is etched**. Holes are manually
  drilled and unplated. Solder every pin on the underside. Do not interpret the file as a two-layer manufacturing order.

## Printing the mask

Use **Actual size / 100%**, A4, no Fit/Shrink, and disable any extra printer mirroring.
Black preserves copper. White is etched away or drilled out. The page contains only three masks.
The artwork uses component-side coordinates: printing it unchanged and placing the **toner face down
against the underside copper** makes the one required reflection. No further mirror operation is needed.
This orientation applies to the iron/toner-transfer process requested by the user.

The PDF's vectors were checked at exactly 72/25.4 points per mm, three copies, A4,
`PrintScaling=None`, zero text and 156 drill voids per copy. A printer can still introduce scale error:
before transfer, measure **22.86 mm between the ESP socket rows**, **53.34 mm from first to last pin**,
and **2.54 mm between adjacent module pins**. Place the real parts against the paper to verify fit and polarity.
The asymmetric antenna blank area and lidar pads help compare orientation with the assembly view.

## Power - one verified BEC

ESC motor/battery wires go to a separate power distribution board. **No 40 A path runs through this PCB.**
ESC J2-J5 have pin 1 GND, pin 2 that ESC's BEC output, pin 3 signal. Connector order is chosen here;
verify each ESC cable's actual order before connecting it.

All JP1-JP4 links are open initially. After measuring a regulated **5 V** BEC, fit **at most one** link:
JP1 selects ESC1, JP2 selects ESC2, etc. The other BEC red wires remain electrically separate.
An ESC's 40 A rating does not guarantee a BEC exists or that its voltage/current is suitable.
Remove **all** BEC links before USB power/programming. The DevKit's 5 V, 3.3 V and USB supply options
are mutually exclusive according to Espressif. The companion computer uses a separate power supply.

The chosen 5 V rail powers DevKit 5V and lidar. The DevKit's regulated 3V3 rail powers the sensors;
I2C/UART remain 3.3 V logic. Never connect 3S LiPo voltage directly to these rails.
R1/R2 are optional 4.7 kohm I2C pullups and **DNP initially**: module pullups already combine in parallel.
Verify actual module input limits, regulator output and I2C high levels; clone modules differ.

## Connector map, component side

| Part | Pins in order from pad 1 | ESP GPIO / detail |
|---|---|---|
| U2 MPU6050 GY-521, 8 pins | 3V3, GND, SCL, SDA, AUX_SDA, AUX_SCL, GND, INT | SCL8, SDA9, INT16; address 0x68 |
| U3 BMP280, 6 pins | 3V3, GND, SCL, SDA, 3V3, GND | CSB high selects I2C; SDO low selects 0x76 |
| U4 GY-271, 5 pins | 3V3, GND, SCL, SDA, DRDY | DRDY17; actual compass chip/address unverified |
| J1 remote lidar wires | 5V, GND, TX from lidar, RX to lidar | FC RX6, TX7; **not a factory cable color/order** |
| J2 ESC1 | GND, BEC1, signal | GPIO12 |
| J3 ESC2 | GND, BEC2, signal | GPIO1 |
| J4 ESC3 | GND, BEC3, signal | GPIO2 |
| J5 ESC4 | GND, BEC4, signal | GPIO14 |
| JP1-JP4 | corresponding BEC, FC_5V | Fit at most one; all removed for USB |
| J6 MPU auxiliary interface | 3V3, GND, AUX_SCL, AUX_SDA | Separate from the main host I2C bus |
| J7 companion UART | GND, FC TX, FC RX | GPIO5 to companion RX; GPIO4 from companion TX; 3.3 V only |

U1 is the **44-pin Espressif ESP32-S3-DevKitC-1 reference**, 22 pins per row,
2.54 mm pitch, rows 22.86 mm apart. Its pad numbers are header numbers, not GPIO numbers.
See the numbered assembly guide, native schematic and connections.json for all 44 pads.

## Mechanical assumptions and hardware acceptance

Nominal bodies: MPU6050 21.2 x 16.4 mm, BMP280 15.5 x 11.8 mm, GY-271 13.5 x 14.8 mm.
For sensors, the first dimension is along the pin row. The exact clone header-to-edge distance
and row centering are still assumptions. Refer to [the source-linked dimensions](module-dimensions.md).
The ESP body is 25.40 x 62.74 mm, with its last header pin 8.00 mm before the USB-end edge;
the antenna overhang is drawn separately. The actual purchased clone and cable housings must be checked.

Native checks show **0 DRC violations / 0 unconnected items**, 44 schematic components plus
4 mounting holes, 156 pads/holes, and all original logical nets connected through the fitted wires.
Voltage rails and BEC feeds remain distinct. These are CAD checks, **not a manufactured-board continuity test**.
After etching/drilling, inspect tracks and annuli, check 5V-to-GND and 3V3-to-GND for shorts, then
check each ground and connector end-to-end before inserting modules or connecting power.

The current bench firmware remains unchanged and has motor outputs disabled. This PCB does not
validate slow sensor drivers, Xbox, companion communications, flight control or a BEC under load.
Battery voltage/current sensing, cell monitoring and high-current distribution are not included in this carrier revision.

## Recheck and reproduce

From QCopter-Stepwise:

```powershell
.\qcopter.ps1 check-home-etched-pcb
```

This saves original KiCad DRC/netlist logs under `.local/home-etched-checks/` and performs
an independent native schematic/PCB comparison plus a graph check through every W link.
Original failed routing reports are retained in `validation/failed-trials/`.
The native board is authoritative; `route-model.json` is its routing input, not an acceptance test.
Tools live in `tools/hardware/`: build_home_etched_pcb.py, check_routed_pcb.py,
export_routed_geometry.py, make_routed_outputs.py and verify_home_etched_pcb.py.
The builder requires KiCad Python and a **new output directory**. PDF generation requires ReportLab/pypdf.
Use isolated dependencies; do not install them into KiCad's Python or the FC environment.

Primary references:
- [Espressif DevKit header/power guide](https://documentation.espressif.com/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)
- [Espressif mechanical drawing](https://dl.espressif.com/dl/DXF_ESP32-S3-DevKitC-1_V1_20210312CB.pdf)
- [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
