# Lidar gap CAD handoff

Engine: native KiCad 10 DRC + schematic parity + exported XML. Independent
pad/graph checker verifies 14 common GND nodes when G1/G2 are fitted, 3V3/5V
separation, 3 isolated unused BEC positives, all selected GPIOs and 79 real drills.
Result: all native counts 0, track minimum1.2mm, explicit copper clearance0.5mm.
G2 is one continuous wire with a middle tap; no extra holes or hidden top copper.
Reuses solder lands on existing header pins; schematic models actual wires.
Source pin capabilities checked against primary Espressif/Bosch docs; current
bench MPU9/8 matches but future ESC4/1/14/21 and UART10/11/slow bus41/40 require
integration. Existing bench pulldown list differs; no code/flash/PWM changes.
Failed original routing logs remain. Previous draft masks are obsolete. Physical
fit, printer scale, manufacture/continuity, BEC load and sensor/ESC behaviour
remain unvalidated; CAD pass is not flight acceptance.
