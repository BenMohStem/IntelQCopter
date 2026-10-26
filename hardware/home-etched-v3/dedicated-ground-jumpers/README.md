# Current PCB: three dedicated ground jumpers

Open [QCopter.kicad_pro](QCopter.kicad_pro), then [the PCB](QCopter.kicad_pcb).
Press **Alt+3** for the included 3D assembly. The previous two-wire/shared-pin
revision is preserved separately; its mask does not include these six new holes.

## What changed

The isolated grounds now have short **1.2 mm copper branches** ending at new
wire holes. Fit **three separate insulated wires**, each between its own G1/G2/G3
pad pair. No shared ESC/module header hole, solder overlay or middle tap.
Each jumper land is **3.2 mm diameter with a 1.2 mm drill**. There are **85 holes**
total: all 79 original module/header holes, unchanged, plus six jumper holes.
The single B.Cu layer retains 1.2/1.4 mm tracks and 0.5 mm minimum clearance.

| Jumper | Pad 1 | Pad 2 | Purpose |
|---|---|---|---|
| G1 | (112.0,79.0) mm | (118.5,60.5) mm | Join BMP280/GY-271 shared return to common GND |
| G2 | (103.0,108.6) mm | (103.0,117.4) mm | Join right ESP ground and ESC4 return to common GND |
| G3 | (55.0,108.0) mm | (55.0,117.4) mm | Join isolated J4/ESC3 GND to common GND |

Coordinates are KiCad component-side coordinates. Follow the G-number pairs in
the guide. Fit wires above the carrier and solder below. G1 passes beneath the
raised BMP module; fit it first and confirm actual vertical clearance.

[Labelled jumper guide](output/assembly-guide.html) ·
[Component-side drawing](output/assembly-component-side.png) ·
[Three 1:1 A4 masks](output/etch-mask-3-copies-A4-1to1.pdf) ·
[Complete portable KiCad ZIP](output/IntelQCopter-dedicated-ground-jumpers-KiCad.zip)

![Jumper placement](output/assembly-component-side.png)

## Checks and limits

[Native DRC](validation/native-drc.json), schematic parity and unconnected items:
**all zero**. The [independent pad/wire graph check](verification.json) confirms
all 14 logical GND endpoints share common ground **when all three wires are
fitted**. The new common lands are on one connected GND pour, not isolated fill.
All three jumper footprints have exactly two separate drilled pads. All original
79 header/module pad positions, nets and holes are [preserved](output/preserved-interfaces.json).
Rejected placement trials and original logs are retained in validation/failed-trials.

Before power, verify continuity from every GND to J1/lidar GND with a multimeter,
and check the 5 V / 3.3 V rails for shorts. CAD checks do not establish physical
continuity, exact module fit, successful etching, printer scale or supply quality.

Print the new mask at **Actual size / 100%**, no Fit to page or additional mirror.
Three vector 92 x 82 mm copies on A4; black preserves copper, white drill holes,
no printed text. [PDF scale checks](output/print-verification.json) are exact in
the file; measure a paper print before transfer. Do not use the old 79-hole mask.

Only J4/ESC3 pin 2 supplies FC_5V from a verified regulated 5 V BEC. Other BEC
positives stay isolated; disconnect the selected positive before USB powering.
Motor/battery current stays on a separate PDB. Sensors and I2C pullups use 3.3 V.
MPU AD0 stays grounded for address 0x68; BMP SDO stays grounded for 0x76.
No firmware or motor-output changes. The future PWM map still needs adaptation;
see [pin audit](pin-audit.md). Physical motor arming remains disabled.

[3D top preview](output/3d-top.png) · [Model assumptions](models3d/README.md)

Recheck from the Stepwise checkout: `.\qcopter.ps1 check-current-pcb`.
