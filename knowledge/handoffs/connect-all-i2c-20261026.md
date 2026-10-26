# Connected I2C reference / root / 2026-10-26

User's final choice: I2C where SPI offers no practical benefit; connect all useful
interfaces. Created self-contained connected reference beside original user
project, preserving open files. Shared I2C, MPU INT GPIO16, mag DRDY GPIO17,
lidar UART6/7, companion UART4/5, motor controls12/1/2/14. MPU auxiliary bus
exposed separately. All module pins assigned; unused MCU GPIOs remain on DevKit.

Four BEC links are initially OPEN; at most one verified 5 V source may be linked.
R1/R2 DNP optional pullups. No routing, motor current path, firmware driver
implementation or physical acceptance. Original failed netlist trial had local
names /GND vs PCB GND; fixed with explicit global labels. Failed staging preserved.
Final native netlist independently checks all pad nets and power separation.
Review the README/validation JSON and actual test output before routing.
