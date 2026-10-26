# Three dedicated drilled ground jumper pairs

Task pcb-drilled-jumpers complete. New canonical project:
hardware/home-etched-v3/dedicated-ground-jumpers. Prior revision untouched.
Engine: KiCad 10.0.6 native DRC + parity + XML netlist, independent assembled-net
graph, native raytraced 3D, exact-vector ReportLab masks with pypdf scale checks.
85 PTH holes, 12 components, 349 B.Cu tracks >=1.2mm, one GND fill contour.
Zero violations/unconnected/parity; all fourteen logical GND endpoints common
with three separate wires. Original 79 module/header pad interfaces unchanged.
PDF and top/assembly previews visually inspected. Three rejected placements
preserved under validation/failed-trials with native evidence; full trial source
copies private under .local. No hardware, firmware, serial, motor or arming work.
Remaining: actual clone/paper fit, 9mm raised module clearance above G1, etched
continuity and shorts, BEC supply and future PWM pin mapping integration.
