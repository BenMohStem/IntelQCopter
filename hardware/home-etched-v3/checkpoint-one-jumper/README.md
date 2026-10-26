# Draft checkpoint: antenna route below, W1 retained

**NOT READY TO ETCH.** Native KiCad reports zero clearance/short violations but
four unconnected GND items. The user requires every ground common; this has NOT passed.
The three-copy PDF is supplied as a requested inspection/scale checkpoint before
attempting to remove W1. It is not a fabrication release or validated FC design.

Board 92 x 82 mm. Actual-size A4 PDF, three copies, no text on copper masks.
Black retains copper; white holes remain open. Print Actual Size / 100%; no extra
mirroring. Put toner face down on the underside copper. Verify 22.86 mm between
ESP header rows and 2.54 mm pin pitch against a ruler before any transfer.
The upper 3V3 route was replaced by a 1.4 mm path through the ESP interior and
around the lower/right perimeter. The antenna cutout remains clear of copper.
W1 remains a mandatory 4.8 mm insulated bridge in this checkpoint.

This copy is based on the earlier one-jumper trial5, not the original Documents
sketch. Its exploratory GPIO map is in connections.json; it differs from the old
FC map and has not been accepted or flashed. No firmware or motor-output changes.
The user's open original project is preserved. Do not overwrite it from this draft.
