# Connected I2C reference: all current modules and spare interfaces

Open **QCopter.kicad_pro in this folder**. The schematic is connected through named
nets. The PCB has the same nets assigned to pads and shows airwires; it has **no
copper routes, board outline or ground pour**. Arrange and route it yourself.
This separate project preserves your existing open palette/layout.

## Sensor choice

Keep MPU6050, BMP280 and GY-271 on **I2C**, SDA GPIO9 and SCL GPIO8. BMP280 can use
SPI, but its low sample rate does not justify extra wires here; SPI does not
improve its pressure measurement accuracy. A shared-bus timing check is still
required when slow sensors are integrated into the 1 kHz acquisition firmware.
The MPU6050 and HMC5883L are I2C devices; do not mistake the SPI-capable MPU6000
for the MPU6050.

The physical interface is prepared. The current sensor-test firmware remains
MPU6050-only; this project does not implement new drivers, PID or ROS navigation.

## Connections

| Device | Power | Signals and extra pins |
|---|---|---|
| U2 MPU6050 | 3V3 / GND | SCL GPIO8, SDA GPIO9, INT GPIO16, AD0 GND; XDA/XCL to J6 auxiliary header |
| U3 BMP280 | 3V3 / GND | SCL GPIO8, SDA GPIO9; CSB to 3V3, SDO to GND: I2C address 0x76 |
| U4 GY-271 | 3V3 / GND | SCL GPIO8, SDA GPIO9, DRDY GPIO17 |
| J1 remote lidar | FC_5V / GND | Sensor TX to FC RX GPIO6; sensor RX from FC TX GPIO7 |
| J2 ESC1 | GND; own BEC net | Signal GPIO12; BEC through JP1 only |
| J3 ESC2 | GND; own BEC net | Signal GPIO1; BEC through JP2 only |
| J4 ESC3 | GND; own BEC net | Signal GPIO2; BEC through JP3 only |
| J5 ESC4 | GND; own BEC net | Signal GPIO14; BEC through JP4 only |
| J6 MPU auxiliary | 3V3 / GND | Pin3 AUX SCL from MPU XCL; pin4 AUX SDA from MPU XDA; separate from main I2C bus |
| J7 companion UART | GND, no power pin | Pin2 FC TX GPIO5 to computer RX; pin3 FC RX GPIO4 from computer TX |

J1 carrier wire order remains: 1 regulated 5 V, 2 GND, 3 TX_FROM_LIDAR, 4 RX_TO_LIDAR.
This is not a claim about the lidar's original cable order; identify wires by signal.
J7 requires **3.3 V UART logic**, not RS-232 or 5 V serial. Verify your companion
carrier's actual UART level and power it separately. The FC can operate without
the companion; its future serial driver must not be a dependency for manual flight.
UART and interrupt wiring do not mean those firmware interfaces are already tested.

Unused DevKit GPIOs remain accessible on its existing headers. Their schematic
no-connect markers mean "unused in this design", not that you physically lose
access. Every sensor/connector pin has a defined connection. J6 exposes the
MPU auxiliary bus rather than wrongly joining it to the host I2C bus.

## Power selection

**JP1-JP4 are removable two-pin links. Fit at most ONE**, and only for an ESC that
has a verified regulated **5 V BEC output**. A 40 A rating does not establish the
presence, voltage or capacity of a BEC. All four BEC wires reach separate link
inputs; the outputs reach FC_5V. They are not joined unless someone fits multiple
links. All links are initially open; the diagram does not magically select a source.

FC_5V powers the DevKit 5V pin and lidar. The DevKit regulator supplies sensor 3V3.
All ground references are connected. The high-current ESC battery/motor wiring
stays on a separate rated power distribution board, not through these connectors.
Remove **all BEC links before USB powering/programming**, per the reference
DevKit's mutually exclusive supply guidance. Voltage must be measured before use.

R1/R2 are optional **4.7k I2C pull-up footprints, DNP initially**. Modules may already
contain pullups; assess their combined resistance/rise time before fitting more.
All I2C pullups must go to 3V3. Generic passive symbols are interface placeholders;
their ERC result alone is not proof of a valid power architecture or correct assembly.

## Validation and use

The independent native netlist/PCB check verifies 17 components, 98 pad numbers,
all assigned nets, UART directions, I2C sharing, BMP straps, GPIO allocation and
separate BEC feeds. It cannot validate clone identities, physical fit, BEC output,
power noise, bus timing or completed copper routing. Those remain real tests.

From the Stepwise software checkout:

```powershell
.\qcopter.ps1 check-connected-pcb
```

In PCB Editor, enable F.Fab/F.CrtYd to see module bodies. Press M/R to place parts,
route the bottom copper for your single-sided process, and run DRC after routing.
Existing sensor body dimensions remain nominal references with assumed header
offsets; see module-dimensions.md. No physical motor arming is enabled.

References: [Bosch BMP280 interface selection](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf),
[Espressif DevKit power and headers](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html),
[Hobbywing BEC backfeed guidance](https://support.hobbywingdirect.com/hc/en-us/articles/19157002618131-Using-a-Servo-Tester-to-operate-your-Electronic-Speed-Control).

[Original native connection-check output](validation.log) and [reviewed DRC findings](DRC-summary.json) are retained. DRC currently reports the missing outline and 52 unrouted items; no copper shorts/clearance violations were reported for this unrouted reference.
