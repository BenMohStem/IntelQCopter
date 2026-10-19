# Flight-controller carrier PCB — draft 0

This first editable PCB is a **logic/sensor carrier** for a Raspberry Pi Pico 2 (RP2350) module. It is not a 40 A power-distribution board. The four ESCs must take battery current directly from a separately rated power-distribution board; this board only routes ESC signal and signal ground. Do not route motor current through this PCB.

## Mechanical assumption

The best match for the described red/white quad is an F450/Q450-style frame. The DJI FlameWheel F450 manual gives a 450 mm diagonal wheelbase, 282 g frame weight, and 9.4 × 5.0 inch propeller dimensions. The latter is a 9.4 inch diameter and 5 inch pitch; it is not a 5 inch racing prop. The board is provisionally 95 × 85 mm with a 30.5 mm square M3 mounting pattern, intended to fit inside the center plates. Verify the specific clone's plate opening and hole coordinates with calipers before ordering a PCB.

## Electrical design

- Host MCU: removable Pico 2 module on two 1×20, 2.54 mm female sockets. External buck module supplies 5.0 V to Pico 2 VSYS from the 3S pack. Never connect the LiPo directly to VSYS.
- Sensors: three 1×4 I²C headers for MPU6050, BMP280, and HMC5883L breakouts; bus is GP4 SDA / GP5 SCL. Breakouts must use 3.3 V logic. Keep sensor leads short, twisted with ground where practical, and away from ESC phase/battery wires.
- Radio: shared SPI0 (GP16 MISO, GP17 LoRa NSS, GP18 SCK, GP19 MOSI), plus individual CS / reset / DIO or GDO lines for one LoRa module and two CC1101 breakouts. All radio interfaces are 3.3 V only; verify exact module pinouts before connecting.
- ESC: four two-pin signal/ground headers in provisional FL, FR, RR, RL order on GP2, GP3, GP6, GP7. The PCB does not carry ESC BEC +5 V pins.
- Battery sense: a 100 kΩ / 27 kΩ divider scales 3S pack voltage to ADC0/GP26; nominal 12.6 V produces about 2.68 V. It must be checked against the actual resistor tolerances and Pico 2 ADC reference before powering.
- UART: one 1×4, 3.3 V-only auxiliary UART on GP8/GP9. The exact SenseTF Mini model and voltage interface must be checked before use; add level shifting if its signal levels require it.
- Four mounting holes are non-plated M3 clearances. The ground plane, decoupling, connector keepouts, mounting plate, and return-current paths still need a reviewed routed layout.

## Current files and validation

`generate_pcb.py` uses the installed KiCad 10.0.6 footprint libraries to generate `qcopter_fc_carrier.kicad_pcb` with footprints, nets, board outline, and mounting holes. KiCad DRC currently reports zero geometric/clearance violations; all 65 electrical connections remain unrouted, so this is an unrouted placement/net-assignment draft and **not ready for fabrication**. There is no completed schematic, routed copper, electrical ERC sign-off, or assembly drawing yet. The pin map and preliminary BOM are adjacent CSV files.

Before manufacturing, confirm the exact F450/Q450 clone, mounting pattern, all breakout-module models, ESC input protocol, switch/arming design, power supply, Pico module orientation, battery connector, and lidar electrical interface. Then create the authoritative schematic, route the board, run DRC/ERC, review 3D clearances, and bench-test rails/current draw with no propellers.
