# Can this single-sided layout satisfy the specification?

Use **necessary tests first, a complete routed witness second, and physical
measurements last**. A failed autorouter run means that run failed. It does not
prove that every placement, GPIO assignment or continuous route is impossible.

## Requirements that this gate checks

| Requirement | CAD acceptance |
|---|---|
| Required parts | Exactly U1 ESP32, U2 MPU, U3 BMP, U4 compass, J1 lidar and J2–J5 ESC headers |
| Manufactured interface | 79 unique through holes, correct module pin ordering/pitch, round solder lands at least 2 mm, annulus at least 0.45 mm |
| Home fabrication | B.Cu tracks/pours only; tracks at least 1.2 mm; explicit clearance at least 0.5 mm; no vias or hidden copper graphics |
| Zero wire bridges | No extra jumper components, net ties, internal pad-jumper declarations or duplicate pin tricks |
| Ground | Every required GND endpoint belongs to one **actual carrier-copper** component |
| Power | 3.3 V sensors and I2C; 5 V FC/lidar; exactly one ESC BEC positive feeds 5 V; other BEC positives isolated |
| GPIO | Unique eligible ESP32-S3 GPIO for each signal; exclude boot, USB, flash/PSRAM and board-LED conflicts |
| Antenna | No copper intersects the nominal antenna projection; permanent F.Cu/B.Cu rule area covers the entire nominal antenna projection and blocks tracks, vias, pads and pours |
| Native tools | Zero native DRC, unconnected items and schematic parity issues; native XML agrees with PCB nets |

BMP280 SDO is deliberately selectable: `--bmp-address 0x76` requires SDO=GND
and counts 14 ground endpoints. `--bmp-address 0x77` requires SDO=3V3 and counts
13. Bosch documents both choices; the future driver must use the selected
address. SDO cannot float and CSB remains at 3.3 V for I2C. This option does not
change the current bench firmware. [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)

The carrier numbering is checked against the genuine DevKitC-1 header map,
including the two 22-pin rows and 22.86 mm spacing. Candidate GPIO reassignment
is allowed, but changing a net name cannot turn an unsuitable GPIO into a valid
output. GPIO40/41 must not simultaneously serve external JTAG. Actual clone
variant and purchased dimensions remain unverified.
[Espressif header reference](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)

An explicitly selected `--single-3v3-pad 1` or `--single-3v3-pad 2` may use one
of the DevKit's duplicate 3.3 V output headers and leave the other **unconnected
on the carrier**. Both are on the same internal DevKit supply rail; using one
output is an ordinary carrier power choice, not a hidden ground bridge.
The default still requires both carrier connections. The optional mode checks
the unused pad has no touching carrier copper and checks that every sensor
VCC/CSB/selected-SDO endpoint is physically connected to the used output by
carrier copper. All ground endpoints remain mandatory bare-copper connections.
[Espressif DevKit schematic](https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-DevKitC-1_V1.1_20220413.pdf)

## 1. Reject impossible pin escapes before routing

For neighbouring round pads, let pitch be `p`, diameter `d`, track width `w`
and copper clearance `c`. The available gap is

\[
g=p-d.
\]

A different-net track can pass through that gap only if

\[
w+2c\leq g.
\]

Here

\[
g=2.54-2.00=0.54\ \mathrm{mm},\qquad
w+2c=1.20+2(0.50)=2.20\ \mathrm{mm}.
\]

Therefore these header banks **cannot be crossed between adjacent pads** by a
1.2 mm different-net track. Escape through the appropriate terminal or around
the bank ends. Raising a sensor permits routing beneath its body; it does not
create additional space between the solder pads.

## 2. Test narrow channels and terminal order

If `n` distinct nets are forced through a channel of usable width `W`, then a
necessary packing condition is

\[
nw+(n-1)c+2b\leq W,
\qquad
n\leq\left\lfloor\frac{W-2b+c}{w+c}\right\rfloor,
\]

where `b` is clearance to each channel boundary. With `W=6`, `w=1.2`, `c=b=0.5`
mm, at most three such tracks fit. Four forced nets require 7.3 mm and cannot
use that fixed channel. This is a certificate **only if an alternative path
around the channel is excluded**. Moving a module or changing a GPIO can remove
the forced cut. Published routing work uses channel density as a lower bound,
not as a general feasibility certificate.
[Princeton routing research](https://www.cs.princeton.edu/research/techreps/746),
[Berkeley routing lecture](https://people.eecs.berkeley.edu/~keutzer/classes/244fa2005/lectures/4-routing.pdf)

For two two-terminal nets constrained to one simply connected region, alternating
boundary order `A1,B1,A2,B2` forces a crossing: an A1–A2 path divides the region,
and the two B terminals lie on opposite sides. This does not apply if a route
may leave the region, pass under an assumed obstacle, or use an allowed alternate
terminal. It is a local, stated-boundary test.

Do **not** collapse every multi-pin module into one graph vertex and call a
nonplanar component-net graph a proof of impossibility. Independent module pins
are not shorted together, and legal traces may pass beneath raised modules.
The actual ports, obstacles, channel boundaries and net-tree choices matter.

## 3. Reserve power and ground corridors

Choose placements and GPIOs that put sensor bus terminals in compatible orders.
Reserve escape corridors before committing long signal trees. Ground-first or
signal-first routing are heuristics; neither guarantees success. After every
complete signal-route/RF-keepout iteration, refill native zones and reconstruct
ground connectivity again. A signal path or antenna cutout can split a ground
pour even when every pad still displays the label `GND`.

The independent graph in `geometry.py` uses actual round pads, finite-width
segments and individual filled-zone contours/holes. Separate islands receive
separate vertices. It does not add edges for ESP internal wiring, net labels,
jumper pad settings, schematic components or 3D models. Native DRC/parity is
also mandatory: the independent geometry calculation is an additional check,
not a replacement for KiCad's engine.

KiCad explicitly distinguishes copper net ties from pad jumpers that assume
off-board component connections. Both are rejected for this zero-jumper carrier
test. This prevents an apparently clean ratsnest from hiding a wire or module
ground path. [KiCad 10 pad connection documentation](https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#_pad_connections_net_ties_and_jumper_pads)

## 4. Require a complete witness, not a promising partial board

Run from the Stepwise checkout, with KiCad's bundled Python:

```powershell
& "$env:LOCALAPPDATA\Programs\KiCad\10.0\bin\python.exe" `
  tools\pcb_zero_jumper\method\validate_zero.py `
  hardware\zero-jumper-study\layout-a\YOUR-CANDIDATE `
  --report-dir hardware\zero-jumper-study\method\runs\candidate-a
```

For the deliberately high BMP SDO option, add `--bmp-address 0x77`.
If the approved duplicate power-output option is used, also add
`--single-3v3-pad 1` (or `2`, matching the candidate's used output).
The report directory retains the exact native DRC JSON, XML netlist, original
native tool output, board SHA256 and every independent check. Exit code zero
means scoped CAD acceptance only. `--skip-native` is useful during construction
but intentionally **cannot** return final acceptance.

An exact SMT/ILP/exhaustive router could prove `UNSAT` for a fully specified
finite grid, placements, net-tree choices and rules. That proof excludes only
that formal model. It does not exclude continuous routes, other grid spacing,
other legal GPIOs or other placements unless those were represented. A timeout
or bounded heuristic search gives no impossibility proof.

## Recorded controls and limits

- [14 analytic controls](analytic-controls.json) pass: disjoint same-labelled
  copper stays disconnected, a real track joins pads, polygon holes remain
  holes, and channel/order lower bounds reject the intended impossible cases.
- [Current jumper board negative control](runs/current-jumper-negative-control/report.json)
  is correctly **rejected** as zero-jumper even though its native DRC/parity
  pass. It has real wire components, additional holes and physically separated
  carrier ground groups. Its assembled three-wire acceptance remains a separate
  claim; this test does not invalidate that documented assembly.

No new zero-jumper candidate has been accepted by this method yet. Layout A/B
remain separate search tasks until complete native results are supplied.

Nominal antenna projection is not a certified 15 mm air/copper clearance. The
preferred module placement puts the antenna beyond the carrier edge; verify
housing, installed modules and cables, then measure actual RF range. Copper
clearance alone cannot predict antenna performance.
[Espressif antenna guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/pcb-layout-design.html)

Even final CAD acceptance does not establish actual clone pinout/dimensions,
print scale, etching, drilling, solder continuity, current capacity, supply noise,
I2C rise times/pullups, lidar logic levels, ESC PWM thresholds or flight safety.
Measure these on the real assembly before enabling motor outputs. Motor arming
remains disabled; this study performs no flashing or actuator operations.

## Additional 3.3 V and antenna-rule controls

The optional single-output policy has [seven native-board pin/short controls](runs/single-3v3-controls/controls.json)
and [three synthetic copper-power controls](runs/single-3v3-power-controls/controls.json).
These checks include both selected-pin choices, the default two-pin policy, a
wrong selection, an unused pad touched by carrier copper, and an isolated sensor
VCC. All pass. The test boards are intentionally incomplete, native DRC is skipped
for these focused checks, and none is approved for manufacturing.

The antenna gate subtracts the actual qualifying KiCad rule polygons from the
nominal antenna projection. Merely having a rule area elsewhere cannot pass it.
The [RF rule-coverage controls](runs/rf-rule-coverage-controls.json) exercise
complete, partial, unrelated, union-of-two, and hole-in-keepout cases. RF range,
the real module antenna shape, nearby housing and the recommended physical
clearance around the antenna remain separate measurements.

The strengthened validator was also run on the [current RF-clearance carrier](runs/current-rf-jumper-negative-final/report.json).
Native DRC, unconnected items and schematic parity are all zero, and its antenna
rule covers the nominal projection; it is correctly rejected by the separate
zero-wire criterion because it physically uses ground jumpers. This confirms
that an electrically valid assembled three-wire board is not mistaken for a
bare-copper zero-jumper design.

Each inline sensor header is checked for a straight, ordered, uniformly spaced
row, not merely for distances from pin 1. [Header controls](runs/inline-header-controls.json)
include rotated valid rows and curved or misordered invalid rows. Actual module
outline, pin labels and physical orientation still require checking the purchased
boards; this geometric check cannot certify a clone's body dimensions.

Re-run the focused controls from the Stepwise checkout with the KiCad Python
interpreter. They save new timestamped evidence and never overwrite earlier runs:

```powershell
$kicadPython = "$env:LOCALAPPDATA\Programs\KiCad\10.0\bin\python.exe"
& $kicadPython tools\pcb_zero_jumper\method\test_single_3v3.py
& $kicadPython tools\pcb_zero_jumper\method\test_single_3v3_power.py
& $kicadPython tools\pcb_zero_jumper\method\test_rf_coverage.py
& $kicadPython tools\pcb_zero_jumper\method\test_inline_header.py
```

The generated test boards are **synthetic, incomplete fixtures**. They are not
finished PCB candidates, and no native DRC acceptance is claimed for them.
