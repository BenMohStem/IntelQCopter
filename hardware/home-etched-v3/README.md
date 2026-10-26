# Current PCB work - DRAFT, NOT READY TO ETCH

The user's current sketch is the starting point; the earlier 27-link v2 layout
is superseded as the target. Required: one copper face, 1.4 mm preferred / 1.2 mm
minimum routed tracks, zero jumpers preferred / at most two, all GND common.

## Requested checkpoint before removing W1

- [KiCad PCB](checkpoint-one-jumper/QCopter.kicad_pcb)
- [Project](checkpoint-one-jumper/QCopter.kicad_pro)
- [Three 1:1 masks, one A4 sheet, no copper text](checkpoint-one-jumper/output/DRAFT-one-jumper-3-masks-A4-1to1.pdf)
- [Original native DRC](checkpoint-one-jumper/native-drc.json)

The 3V3 track that ran above the antenna now passes through the ESP interior and
around the lower/right perimeter at 1.4 mm. The antenna copper cutout remains.
This is the user's opened one-jumper trial5 with its existing exploratory GPIO
map, NOT the unrelated later routing searches. No firmware changes or flashing.

**Native result: zero clearance/short violations, FOUR unconnected GND items.**
Not accepted for fabrication or power. A poured area is not proof of common GND.
Every GND must pass native connectivity checks before producing a release mask.
The supplied PDF is an explicit draft/paper inspection checkpoint requested by
the user before the W1 removal attempt. Use Actual Size / 100%, no extra mirror,
toner face down on underside copper. Verify the print physically with the parts.

The later zero-jumper perimeter attempt shorted MPU_INT and was rejected; see
rejected-attempts. W1 remains fitted in the checkpoint. Neither revision is flight
hardware acceptance. The user's open Documents original project is preserved.
