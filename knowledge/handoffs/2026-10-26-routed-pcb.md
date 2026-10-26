# Routed PCB handoff

Scope: hardware/home-etched-v2, tools/hardware, command and knowledge pointers.
Engine: native KiCad 10.0.6, exact native XML netlist/PCB pad comparison, wire-link
graph check; native filled copper extracted into ReportLab PDF, pypdf scale checks.
Result: 44 schematic components + 4 mounts / 156 holes / 27 insulated links;
0 native violations and 0 unconnected items. No top copper tracks or vias.
Final geometry respects separate BEC feeds and 3V3/5V/GND; all logical connections
match the prior connected reference when mandatory W wires are fitted.
Failed trials preserved in hardware/home-etched-v2/validation/failed-trials;
full intermediate boards and routing search failures remain locally under QCopter/build/route-pcb.
Physical gates: actual sensor/body/header fit, raised-socket clearances, printed scale,
etch/drill continuity, BEC output/current and sensor function. Firmware/arming unchanged.
