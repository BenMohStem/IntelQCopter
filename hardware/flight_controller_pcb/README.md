# Flight-controller carrier PCB — draft 0

This first editable PCB is a **logic/sensor carrier** for a Raspberry Pi Pico 2 (RP2350) module. It is not a 40 A power-distribution board. The four ESCs must take battery current directly from a separately rated power-distribution board; this board only routes ESC signal and signal ground. Do not route motor current through this PCB.

## Mechanical assumption

The best match for the described red/white quad is an F450/Q450-style frame. The DJI FlameWheel F450 manual gives a 450 mm diagonal wheelbase, 282 g frame weight, and 9.4 × 5.0 inch propeller dimensions. The latter is a 9.4 inch diameter and 5 inch pitch; it is not a 5 inch racing prop. The board is provisionally 95 × 85 mm with a 30.5 mm square M3 mounting pattern, intended to fit inside the center plates. Verify the specific clone's plate opening and hole coordinates with calipers before ordering a PCB.

## Electrical design

- Host MCU: removable Pico 2 module on two 1×20, 2.54 mm female sockets. External buck module supplies 5.0 V to Pico 2 VSYS from the 3S pack. Never connect the LiPo directly to VSYS.
- Sensors: three 1×4 I²C headers for MPU6050, BMP280, and HMC5883L breakouts; bus is GP4 SDA / GP5 SCL. Breakouts must use 3.3 V logic. Keep sensor leads short, twisted with ground where practical, and away from ESC phase/battery wires.
- Radio: shared SPI0 (GP16 MISO, GP17 LoRa NSS, GP18 SCK, GP19 MOSI), plus individual CS / reset / DIO or GDO lines for one LoRa module and two CC1101 breakouts. All radio interfaces are 3.3 V only; verify exact module pinouts before connecting.
- ESC: four two-pin signal/ground headers in provisional FL, FR, RR, RL order on GP2, GP3, GP6, GP7. The PCB does not carry ESC BEC +5 V pins.
- Battery voltage: the existing 100 kΩ / 27 kΩ divider scales total 3S pack voltage to ADC0/GP26; nominal 12.6 V produces about 2.68 V. It does **not** measure individual cells or state of charge directly. Verify the divider, ADC reference, resistor tolerances, and open-wire behavior before connecting a pack.
- Battery current: GP27/ADC1 receives a filtered 0–3.3 V analog output from an **external current sensor installed in the separately rated PDB**. The flight-controller board carries only the low-voltage sensor signal; battery/ESC current must never flow through it. Select and calibrate the Hall sensor against measured peak current. Do not connect a 5 V sensor output directly to the Pico.
- Cell safety interface: GP28 is `BMS_ALERT`; J17 carries 3.3 V, ground, shared I²C, and alert from an external 3S cell-monitor/protection module connected to the pack's balance lead. The module and its cell-tap wiring are not selected or included. This carrier contains no battery disconnect MOSFETs, fuse, balancing circuit, or independent over/undervoltage protection.
- UART: one 1×4, 3.3 V-only auxiliary UART on GP8/GP9. The exact SenseTF Mini model and voltage interface must be checked before use; add level shifting if its signal levels require it.
- Four mounting holes are non-plated M3 clearances. The ground plane, decoupling, connector keepouts, mounting plate, and return-current paths still need a reviewed routed layout.

## Current files and validation

`generate_pcb.py` uses the installed KiCad 10.0.6 footprint libraries to generate `qcopter_fc_carrier.kicad_pcb` with footprints, nets, board outline, and mounting holes. The current-sense and cell-monitor connections are interface placeholders. The regenerated draft passes the KiCad geometry DRC (0 geometry violations) but has 76 unconnected pads because no copper has been routed. This is **not ready for fabrication**. There is no completed schematic, routed copper, electrical ERC sign-off, or assembly drawing yet. The pin map and preliminary BOM are adjacent CSV files. The saved DRC report is not a full connectivity/routability sign-off.

The battery software in `flight_controller/flight_controller.c` filters total-pack voltage with a scalar Kalman filter and estimates SOC with a Kalman predictor driven by coulomb counting. An optional SOC measurement correction must come from a pack-specific rested-OCV lookup; no such lookup is supplied yet, and loaded pack voltage is never treated as an SOC measurement. SOC uncertainty is reported as standard deviation. Cell voltage and BMS alert inputs are optional data from an external balance-lead monitor/protector; without valid per-cell data, the software cannot detect a weak individual cell. Thresholds must be set from the exact pack and sensor datasheets. The action is a landing request while airborne and disarm only after a separate landed-state indication; firmware does not cut battery or ESC power.

Before manufacturing, confirm the exact F450/Q450 clone, mounting pattern, all breakout-module models, ESC input protocol, switch/arming design, power supply, Pico module orientation, battery connector, external current-sensor output range and peak rating, cell-monitor/BMS module, and lidar electrical interface. Then create the authoritative schematic, route the board, run DRC/ERC, review 3D clearances, and bench-test rails/current draw with no propellers.

The flight code may provide **software monitoring and low-battery flight modes**, but those do not replace pack-level protection. A software decision depends on the MCU, sensor, firmware, and power remaining operational; it cannot open the battery circuit unless a separately engineered, current-rated hardware protector exists. Do not hard-cut motor power in flight on a low-voltage warning; the intended policy is warn/request a controlled landing and only disarm after landing. Thresholds remain unset pending the exact cell datasheet and load measurements.
