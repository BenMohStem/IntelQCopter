# ESP32-S3 competition carrier — home etched prototype

![Component-side assembly](assembly-preview.png)

**Review and fit prototype. Physical motor arming remains disabled.** Native KiCad checks establish the carrier geometry and connectivity, not electrical operation or flight readiness. No physical board has been built or measured.

## Ready-to-print mask and KiCad download

- [Three identical masks on A4, no text, 1:1](output/pdf/etch-mask-3-copies-A4-1to1.pdf).
- [Complete self-contained KiCad package](output/IntelQCopter-ESP32-home-etch-KiCad.zip).

The PDF contains **only three identical copper masks**: no labels, dimensions, text, silkscreen or jumpers. Each represents the 100 x 100 mm board at actual size. Black is copper to preserve, including the large connected GND plane; white is etched away. It is a positive **face-down toner-transfer master**. The printed toner face goes against the copper; that contact produces the one required reversal. Do not enable printer mirroring. This requires heat-fusible toner, not ordinary inkjet ink.

Print **Actual size / 100%**; turn off fit-to-page and borderless enlargement. The corresponding left edges of the two upper masks are **100 mm apart** on the printed sheet. The ground-plane outer span is 95 mm because it stays 2.5 mm from each board edge. Check this before transferring. Use the separate fit/drill sheet for component and mounting dimensions.

The zone has 0.8 mm isolation clearance, 0.8 mm minimum fill width and solid GND pad connections (no thermal spokes). Isolated zone islands are removed. The zone has an antenna-clearance notch covering x=36..67 mm, y=0..17 mm at the reference ESP32 antenna; actual board/frame RF clearance still needs checking. The 13 insulated top wires are still required.

Native KiCad DRC reports zero violations, unrouted items and schematic-parity issues after the ground fill. Exact filled polygons, tracks and pads are extracted from the native board for the PDF. Both original labelled PDFs and earlier worksheet-bearing SVGs are retained as **superseded** evidence under `artifacts/hardware/print-mask-20261021/`. Use the new three-copy PDF.

## What to open

- `esp32_home_etch.kicad_pro`: KiCad 10 project, schematic, board and bundled libraries.
- [Connector map](connector-map.md), [assembly BOM](assembly-bom.csv), [wire links](wire-links.csv), [drill table](drill-table.csv).
- `fit-and-drill-top.svg`: 100 × 100 mm paper fit sheet. Print **100% / actual size**; check its 20 mm ruler and 22.86 mm socket-row spacing.
- `plots/copper-component-view.svg`: bottom-copper artwork viewed through the board from the component side.
- `plots/copper-solder-side-view.svg`: mirrored artwork as seen looking directly at the solder side.
- [Process coupon](process-coupon/README.md): trial widths, gaps and enlarged pads.
- [Original CAD reports and failed iterations](../../artifacts/hardware/home-etch-20261021/).

## Construction rules

| Feature | Size / method |
|---|---|
| Carrier | 100 × 100 mm; mounting pattern must be checked on the real F450 |
| Copper | One etched bottom layer; **13 insulated component-side wire links** |
| Signal tracks | 1.0 mm; minimum rule 0.8 mm |
| Logic power/ground | 1.5–2.0 mm; minimum copper gap 0.6 mm |
| DevKit socket pads | 1.94 mm diameter, 1.1 mm drill, 2.54 mm pitch |
| External connector/passive pads | 3.2 mm diameter, 1.2 mm drill |
| Wire-link pads | 2.8 mm diameter, 1.2 mm drill |
| Mounting holes | 3.2 mm, non-plated |

The two dense DevKit socket rows constrain pad enlargement: 1.94 mm pads leave 0.60 mm between pads at 2.54 mm pitch. External wiring uses much larger pads. KiCad represents through-hole pads on two copper layers, but **there are no routed top-copper tracks**. Only B.Cu, including the GND zone, is fabricated; holes are not plated. Solder from the bottom. Wire crossings require insulation.

## Reference components and power

The socket reference is the genuine **Espressif ESP32-S3-DevKitC-1**, two 22-pin rows separated by 22.86 mm. A generic board advertised as “ESP32-S3” may not fit. Compare the printed sheet with the purchased board before etching. The grey body now follows the published 25.40 x 62.74 mm DevKitC-1 reference outline. RF and USB service envelopes are separate design clearances; purchased clone fit is not established. Sensor modules connect by pigtail, so their clone pin orders are not assumed.

J3 takes an **external regulated 5 V supply**, never the 3S pack. W21 connects that supply to the DevKit 5 V pin: **remove W21 before USB powering/programming**. Use one board power source at a time. Motor and ESC power must remain on the separate rated power-distribution system; no 40 A path exists here. The carrier does not implement a battery cutoff or hardware BMS.

I2C: SDA GPIO9, SCL GPIO8; ports for MPU6050, BMP280 and HMC5883L. Choose breakouts with verified 3.3 V logic and compatible pull-ups. Do not connect an unknown 5 V I2C module. HMC5883L firmware integration is pending. Board antenna clearance and frame mounting require physical checking.

ESC signal assignments retain the adapter map: FL GPIO12, FR GPIO1, RR GPIO2, RL GPIO14 through 220 Ω resistors. These are signal-only connectors. Their logic threshold, ESC protocol and physical motor ordering have not been validated; physical motor outputs remain disabled.

Optional companion: GPIO17 TX, GPIO18 RX, common ground, 3.3 V UART; cross TX to RX. No companion-computer power feed is provided. The FC must remain usable without the companion.

## Battery voltage and current provisions

J7 is a low-current pack-voltage **sense input**. R1 = 47 kΩ and R2 = 10 kΩ give

`V_ADC = V_pack × 10 / 57`: 12.6 V becomes approximately 2.211 V.

R3 = 1 kΩ and C1 = 100 nF filter the GPIO4 input. With the divider's Thevenin resistance, the approximate time constant is `(47k || 10k + 1k) × 100nF = 0.925 ms`. Resistor tolerances, ADC calibration and wiring affect accuracy. Disconnect raw battery sense while the MCU supply is off; power-off input protection is not implemented. A calibrated firmware ADC driver is still needed.

J14 provides a **future external current-sensor** input on GPIO5 with R8/C2 filtering. Only a verified 0–2.5 V output and compatible supply are allowed. No current sensor or high-current shunt is selected or placed. Voltage alone cannot measure consumed capacity accurately. The requested 35% warning / 20% disarm policy needs validated voltage/current measurements and a charge estimator; this carrier does not make that policy physically ready. A balance charger does not protect cells during discharge.

Lidar UART provision: GPIO6 RX, GPIO7 TX. J13 pin 2 now receives regulated external 5V through W23. The exact TOFSense-F2 Mini V1.0 datasheet specifies 4.3–5.2V supply and 3.3V logic. Our pigtail order is GND / VCC / sensor TX / sensor RX; map by signal, not vendor cable pin number. The actual purchased revision and firmware driver still need verification.

## Assembly order and review

1. Print the fit sheet at actual size. Confirm board/module/frame dimensions and available drill bits.
2. Use the coupon to establish reliable gaps and pad annuli for your etching/drilling process.
3. Choose artwork orientation for your transfer method. A face-down toner transfer reverses the printed image: use the component-view artwork for that reversal and compare the final copper against the solder-side view. Check an asymmetric connector before etching the full board; never print both masks on the same board.
4. Inspect etched tracks and drill alignment. Install passives, mapped connectors and all 13 insulated links from the wire-link table. Keep links flat under the removable module/socket gap.
5. Before inserting modules or connecting a battery, check continuity against the connector map and absence of shorts. Verify the regulated supply and divided voltage with a meter.
6. Fit the ESP32 and individually verified sensor modules. Motor power/output remains disabled pending later bench integration.

## Reproducibility and evidence limits

`generate_board.py` runs with KiCad's Python/pcbnew. `generate_schematic.py` builds the corresponding schematic; `render_assembly.py` independently compares its exported netlist with the component model and creates the assembly sheets. Native reports, source hashes and comparison counts are in `verification-summary.json` under the evidence folder. Original failing reports remain there.

Current board/print provenance and independent mask power checks: [solid-ground power audit](../../artifacts/hardware/solid-ground-power-audit-20261021/README.md). Earlier versions and their original hashes remain in their original evidence folders.

The final native checks report **zero DRC violations, zero unrouted connections, zero schematic parity issues and zero ERC violations**. These checks treat external modules as passive connectors; they cannot prove voltage-domain compatibility, regulator capacity, ADC accuracy, electromagnetic behavior or flight performance.

## Primary references

- [Espressif board layout](https://dl.espressif.com/dl/PCB_ESP32-S3-DevKitC-1_V1_20210312CB.pdf) and [official footprint](https://raw.githubusercontent.com/espressif/kicad-libraries/main/footprints/Espressif.pretty/ESP32-S3-DevKitC.kicad_mod).
- [Espressif DevKitC-1 power/pin guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html).
- [Espressif ADC schematic guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html).
- [Nooploop module downloads](https://support.nooploop.com/cn/tofsense/download/): exact purchased lidar revision still needs checking.

The long-term RP2350/RP2040 + LoRa/CC1101 architecture remains on main. This carrier serves the separate ESP32-S3 competition branch only.


## Direct ESP32 common ground — corrected edition

The older carrier used `GND_MCU` and W22. That jumper dependency is removed. Carrier J1 pin 22 and J2 pins 1, 21 and 22 are now **GND**, on the same net as J3 power-input ground, sensor grounds, ESC signal returns and the bottom fill. These correspond to the ground pins in the [official Espressif header table](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html#header-block); our right socket is called J2, while Espressif calls that board header J3.

There are direct 1.5–2.5 mm copper routes plus the large B.Cu plane. ESP32 ground pads have **solid zone connections**; all other GND pads now also use solid connections (no cross-shaped thermal reliefs). W22 is not fitted; 13 other insulated links remain. The four ESP32 grounds pass a separate 300 dpi printed-mask connected-copper check against the power-input GND pad, with no non-GND pad shorted to that region. This is digital continuity evidence, not a measurement of an etched board.

Current dimension, native and print reports are in `artifacts/hardware/reference-dimensions-20261023/`. The solid-ground-power-audit folder retains the earlier edition. The direct-common-ground folder retains the earlier edition. Earlier reports remain under their original folders. Use the regenerated PDF and ZIP for this edition.

### Reproduce the pour/export in KiCad 10

1. Open `esp32_home_etch.kicad_pro` and its PCB. Select **B.Cu**.
2. Edit the filled zone: net **GND**, clearance **0.8 mm**, minimum width **0.8 mm**, island removal **Always**, **Pad connections: Solid**. Thermal gap/spoke values are inactive in this mode.
3. For every GND pad, open Pad Properties → Clearance Overrides → Zone connection and keep **Solid**. Verify their net is **GND**, not an unconnected net or `GND_MCU`.
4. Press **B** to refill zones, run DRC with schematic parity and save. The antenna notch remains intentionally unfilled by the zone.
5. In Plot, select only **B.Cu**, PDF, black/white, scale **1**, actual drill marks, no drawing sheet, no references/values and **no mirror** for face-down toner transfer. Do not select top copper, solder mask or silkscreen. Printer settings must also stay at 100% with mirroring and borderless enlargement off.

Equivalent native single-mask export from this folder:

```powershell
kicad-cli pcb export pdf --mode-single --layers B.Cu --black-and-white --exclude-refdes --exclude-value --drill-shape-opt 2 --scale 1 --check-zones --output bottom-copper-single.pdf esp32_home_etch.kicad_pcb
```

The native single-layer export uses the board/page origin and may place copper close to a printer margin; it is useful for layer review. The supplied A4 three-copy sheet positions the masks inside the page and is the recommended print master.

The supplied three-copy PDF is composed from those native copper primitives and checked filled polygons, with no printed text. `tools/hardware/export_etch_geometry.py` uses KiCad Python; `tools/hardware/make_etch_prints.py` uses Python with ReportLab and pypdf. Refill/export after any board edit so the print matches the current board.

## Solid pads and clean drill holes — current edition

All **19 GND pads** use **solid** zone connections. The mask preserves continuous black GND copper around a round white drill opening; there are no four-spoke thermal relief rings. This needs more soldering heat than thermal pads. Non-GND pads retain their insulating clearances: do not fill these white rings with copper.

To reproduce in KiCad 10: open the board, select the B.Cu GND zone, press **E**, set **Pad connections → Solid**, and refill with **B**. GND pad overrides are also Solid. Plot **B.Cu only**, black and white, no drawing sheet, with actual drill marks (not small centre crosses). For the requested three-copy sheet, use the supplied PDF; normal KiCad plotting produces one board per sheet.

Both reference DevKit left pins 1 and 2 are 3.3V. The four GND pins and input GND share copper directly. The board has **13 insulated top links**, including W23 for lidar regulated5V. W22 is absent. W21 remains removable: disconnect it before USB powering the DevKit. The companion computer uses its own supply, with signal GND shared.

Before inserting any modules, meter-check the etched board: each GND hole to J3.1 should read essentially probe-lead resistance; no direct short between GND, 3.3V, 5V or battery-sense positive. Test with a current-limited regulated5V supply, and measure sensor rails before fitting sensors. Paper fit and purchased module pinout checks are still required.

## Manufacturer-reference dimensions — current 23 October edition

[Selected parts, dimensions and sources](component-dimensions.md) · [Machine-readable references](reference-dimensions.json) · [Original dimension/native/print audit](../../artifacts/hardware/reference-dimensions-20261023/README.md).

The dense DevKit sockets now have **1.1mm holes and 1.94mm round pads**, allowing the selected socket's recommended1.02mm hole to be met with a standard manual drill. This leaves0.42mm nominal annulus and0.60mm between adjacent pads. The minimum annulus rule is0.4mm; external3.2mm pads keep their much larger1.0mm annulus. Accurate centring remains essential.

Both100nF capacitors now use exact **5.00mm lead spacing** for the selected formed-lead X7R reference. C2 points left, placing its body and pads outside the USB service area. Axial resistor spacing remains10.16mm, suitable for the documented6.5mm-body MRS25 reference. Copper clearances, common GND, three no-text masks and one bottom copper layer remain as before.

**J3–J14 are soldered-wire ports**, not generic plug-in module footprints or unspecified screw-terminal blocks. Their5.08mm hole spacing deliberately allows large pads. Map sensor/power pigtails by signal; do not push a2.54mm module header directly into them. Lidar/magnetometer ports may remain unused in the current two-sensor setup. This is a reference-defined prototype, not proof that every board sold as ESP32-S3/GY-521/BMP280 fits.
