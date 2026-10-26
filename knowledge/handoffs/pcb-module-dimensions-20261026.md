# Sensor body outlines / coordinator / 2026-10-26

User asked for module dimensions to prevent collisions/reversal. Added sourced
nominal body rectangles and a chosen 2 mm placement courtyard. VCC at pin 1,
components facing up, bodies right of vertical header. Header edge offset 1.27 mm
and along-edge centering are assumptions, not measurements of the actual clones.
Sources and geometric limits: hardware/module-library/module-dimensions.md.

Native KiCad checks preserve all 79 pads, nets, placement and schematic links.
The early shape-mutation SWIG attempt failed in staging; final graphics-only text
patches are natively reloaded and checked. Reapplication checks use temporary
copies. Original user library and PCB saved in timestamped external backup.
Windows UI helper failed startup after retry/reset: active user board not edited
behind the application. Updated copy supplied as QCopter-dimensions-preview PCB;
current live board needs Tools > Update Footprints from Library. Firmware untouched.
