# RF clearance verification

Final native DRC, unconnected items and schematic parity: zero. Independent
native graph validates all fourteen GND endpoints through three physical wires.
The rule area covers both copper layers and forbids pads, tracks, vias and fill.
Native geometry checks prove no copper in y<49 mm, including the 11 mm edge band.
Original narrower antenna cutout is now also protected by the rule area.

Rejected trials preserved: removing the top return exposed a disconnected GND
pour; an attempted internal return crossed slow SDA and was rejected. Those
extra tracks are NOT in the delivered board. Three wires are retained by giving
G2 a third dedicated RF-ground hole at (124,60.5) mm, with a middle tap at G2.1.
No invisible module-internal connections or ignored DRC errors are used.
Actual RF gain/range, module fit, printer scale and etched continuity untested.
