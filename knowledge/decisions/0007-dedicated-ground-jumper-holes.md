# D-0007: Three dedicated two-hole GND jumpers

2026-10-26. User requests short copper branches from isolated ESC GND to its
own drilled wire land, plus clear jumper pairs for the other isolated grounds.
Supersedes D-0006's shared header-pin / middle-tap assembly, not its pin map or
single-layer width rules. Previous CAD and its masks are preserved as history.

Three independent wires G1/G2/G3, six 1.2 mm holes with 3.2 mm round pads.
All 79 original module/header holes and pad nets preserved. Native DRC/parity/
unconnected zero; independent graph checks all GND points common after fitting.
All new common lands connect to one native GND fill contour. Rejected positions
and primary failure logs are retained. G1 must be fitted before the raised BMP
module and kept below it. No new hardware, firmware or physical arming acceptance.
