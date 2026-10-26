# Lidar gap and common ground - current CAD revision

**Open [the assembly guide](output/assembly-guide.html) before building.**
The old power jumper is removed. Two mandatory insulated GND wires connect every
ground after assembly. They reuse existing header solder pins, adding no holes.
G2 is one continuous wire with a middle solder tap: three connection points.

| File | Purpose |
|---|---|
| [QCopter.kicad_pro](QCopter.kicad_pro) | Open the complete KiCad project |
| [QCopter.kicad_pcb](QCopter.kicad_pcb) | Single-sided B.Cu PCB |
| [QCopter.kicad_sch](QCopter.kicad_sch) | Matching native schematic, including G1/G2 |
| [Three 1:1 masks](output/etch-mask-3-copies-A4-1to1.pdf) | A4, three copies, black copper, white drill voids, no text |
| [Assembly guide](output/assembly-guide.html) | Readable component and mirrored solder-side views |
| [Pin capability audit](pin-audit.md) | ESP32-S3 I2C, UART, PWM and firmware limits |
| [Native DRC](validation/native-drc.json) / [independent checks](verification.json) | Original CAD evidence |
| [Drill map](output/drill-map.csv) | 79 actual holes; no zero-diameter overlay lands |

## What changed

The remote lidar uses free-position wire lands, not a fixed-pitch connector.
Pin 1/5V moved to (47.00,95.50) mm; pin 2/GND to (55.00,95.50). Pin 3/TX from
lidar stays (59.00,95.50); pin 4/RX to lidar stays (64.08,95.50). Coordinates use
the KiCad component-side origin. The first pair now has 8 mm centre spacing.
The 1.4 mm MPU_INT trace passes through this gap. The former W1 power bridge
and its two drill holes are gone. Do not follow old W1 instructions on this board.

The slow I2C clock track was moved, a lower data track was shifted, and 1.2 mm
ground return tracks were added to make the ground connections practical.
The antenna-area power track remains on the lower/right route, clear of the
nominal antenna copper cutout. All routed tracks are 1.2 or 1.4 mm. Copper
clearance is explicitly 0.5 mm, with no waived electrical violations.

## Fit both GND wires

| Wire | Connections, in order |
|---|---|
| G1 | BMP280 module pin 2 (GND) -> BMP280 module pin 6 (SDO already strapped to common GND) |
| G2 | ESP carrier pad 23 (bottom-right GND) -> carrier pad 22 (bottom-left GND, middle tap) -> J4/ESC3 pin 1 (GND) |

G1 is about 10.2 mm between attachment points. G2 has two spans totalling about
40.4 mm; add a little length for soldering/routing. Use insulated wire and keep
the insulation intact over crossed tracks. Solder to the exposed ends of existing
header pins on the underside; do not force a second lead into an occupied hole.
G2 needs one small stripped window for its middle tap. The assembly guide marks
the exact GND pins, away from neighbouring 5V pins. Never omit either wire.

These wire footprints contain overlay solder lands coincident with the existing
pins. They do NOT add physical drill holes or hidden copper shorts. The native
board has 84 electrical pad records but only **79 real drilled holes**. The checker
verifies each wire attachment coincides with exactly one existing header pad.
The schematic models the two actual wires, including G2's third tap. All 14 GND
points become one electrical node **when the documented wires are soldered**.

## Power and connectors

Only J4/ESC3 pin 2 is connected to FC_5V. Verify that this ESC actually supplies
regulated 5 V with enough current for the DevKit and lidar. The other three BEC
positive outputs are isolated and must stay isolated. A 40 A ESC rating does not
identify its BEC voltage/current. All four ESC signal grounds join common GND.
Disconnect the selected BEC positive lead before USB powering/programming.
Motor/battery current stays on a separate PDB, not these header tracks.

Each ESC header is pin 1 GND / pin 2 BEC / pin 3 signal. Match cable order to pin
numbers rather than assuming every cable uses the same direction. Sensors and
GPIO logic use 3.3 V. The lidar supply is 5 V; verify the actual sensor's TX level
is compatible with 3.3 V GPIO. No battery divider/current measurement is included.

## Native CAD result and physical limits

KiCad 10: **0 violations, 0 unconnected items, 0 schematic parity issues**.
The independent native netlist/pad/wire graph check confirms common GND, separated
3V3/5V rails, unselected BEC isolation and GPIO assignments. No motor test or
firmware flashing occurred. Physical motor arming remains disabled.

Actual clone geometry/header offsets, etched continuity, printed scale, BEC load,
ESC input threshold and sensor communication still need physical checks. The
reference is a 44-pin ESP32-S3 DevKitC-1, not every board sold as ESP32-S3. The BMP280
module is rotated relative to the other sensor modules: match pin 1/VCC carefully.
Paper-fit the real modules before etching, inspect/drill/solder, fit G1/G2, then
check every GND with a multimeter and test for 5V/3V3 shorts before power.

## Printing

Print the PDF at **Actual Size / 100%**, A4, no Fit/Shrink and no extra mirroring.
Put the toner face down against the underside copper; that contact makes the
required reflection. Black preserves copper. White is etched away or drilled.
The vector transforms are checked at exactly 72/25.4 points per mm, three copies,
92 x 82 mm board coordinates, 79 white drill voids each, and zero mask text.
Physically verify 22.86 mm ESP row spacing, 53.34 mm first-to-last pin spacing,
and 2.54 mm sensor/header pitch. Printer scaling remains a physical check.

## Repeat the checks

```powershell
.\qcopter.ps1 check-current-pcb
```

Original logs are saved in `.local/lidar-gap-checks/`. Historical failed reports
are retained in validation/. The earlier checkpoint masks are obsolete for this
revision; use only the PDF paired with this PCB. Firmware is unchanged; see the
pin audit for the prepared map and the remaining driver/integration work.
