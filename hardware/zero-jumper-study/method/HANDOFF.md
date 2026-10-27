# Validator update handoff

The independent zero-jumper gate remains conservative: actual bare-carrier GND
and power copper connectivity, no bridge components/netties/pad-jumper shortcuts,
one copper track layer, widths/clearances, eligible GPIOs, sensor pin maps, native
KiCad DRC/parity and full nominal antenna keepout coverage. A sensor header must
be a straight, ordered 2.54 mm row. Nominal geometry and CAD are not physical RF,
clone-fit, etching, supply integrity, sensor operation or flight acceptance.

The previously added optional `--single-3v3-pad 1|2` leaves the other duplicate
output header pad electrically unused on the carrier, while all sensor 3.3 V
endpoints must still be connected to the chosen pad. Default: both outputs.

Evidence: 14 analytic controls, seven pin/short fixture controls, three synthetic
power-connectivity controls, seven KiCad polygon RF rule controls, seven header
geometry controls. Final negative control runs on the actual current three-wire RF
board: native DRC/parity 0, RF rule covers nominal antenna, but the separate
bare-copper/zero-wire check rejects it. All focused synthetic fixtures are
incomplete and not manufacture-ready; original test outputs and prior failed
designs remain preserved. An unrouteable bounded trial is not an impossibility proof.

Root may stage this method scope; the two other agents own their layout scopes.
