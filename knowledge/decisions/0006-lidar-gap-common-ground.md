# D-0006 - free lidar lands and two ground wires

2026-10-26. User explicitly permits moving nonuniform lidar solder holes and
one/two GND wires to make all grounds common. Supersedes D-0005's failed current
ground acceptance, preserving its width/single-face constraints and old evidence.
Lidar first gap is 8mm; 1.4mm INT copper passes between 5V and GND lands.
Old W1 power bridge removed. Two actual insulated GND wires reuse existing
header pins: G1 has two solder ends; G2 one continuous wire with three taps.
They are real assembly components, not hidden copper or ignored connectivity.
SMD overlay lands model solder attachment on existing pins and add no drills.
Native DRC, parity and unconnected items all 0 with explicit 0.5mm clearance and
1.2mm minimum tracks. Independent graph confirms all 14 GND nodes common when
the two wires are installed. Actual manufacture, clone fit, BEC/ESC thresholds,
driver use and the changed future PWM pin map remain unvalidated. No firmware,
calibration, flashing or physical arming changes.
