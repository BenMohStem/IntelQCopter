# D-0003: Connect useful interfaces; retain I2C

Date: 2026-10-26. Sources: user asks to connect everything, then requests I2C if
SPI offers no worthwhile benefit. Extends D-0002 from a palette to a connected
schematic/net-assigned PCB; user still handles PCB arrangement/routing.

MPU6050/BMP280/GY-271 use shared 3.3 V I2C on GPIO9/8. SPI does not improve BMP
pressure accuracy; shared-bus timing still needs actual measurement on integration.
Expose MPU INT, magnetometer DRDY, auxiliary bus header, lidar and companion UART.
Wire four ESC control headers with separate BEC feeds through four selectable
links; never auto-parallel supplies. Fit at most one verified 5 V BEC link.

The connected project is self-contained in hardware/connected-i2c and installed
as Documents/Projects/PCB/QCopter/connected-I2C. Original open project retained.
No MCU flash, actuator enablement or new hardware/ROS driver validation occurs.
