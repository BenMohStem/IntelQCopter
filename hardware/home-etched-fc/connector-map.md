# Connector pin map

Viewed from the component side. Pin numbers refer to this carrier, not an arbitrary sensor module. Map each module with a labelled pigtail. Empty socket pins are intentionally unused.

| Reference | Function | Pin: net |
|---|---|---|
| J1 | DevKitC-1 LEFT 1x22 | 1: 3V3; 2: NC; 3: NC; 4: GPIO4; 5: GPIO5; 6: GPIO6; 7: GPIO7; 8: NC; 9: NC; 10: GPIO17; 11: GPIO18; 12: GPIO8; 13: NC; 14: NC; 15: GPIO9; 16: NC; 17: NC; 18: GPIO12; 19: NC; 20: GPIO14; 21: 5V_MCU; 22: GND_MCU |
| J2 | DevKitC-1 RIGHT 1x22 | 1: NC; 2: NC; 3: NC; 4: GPIO1; 5: GPIO2; 6: NC; 7: NC; 8: NC; 9: NC; 10: NC; 11: NC; 12: NC; 13: NC; 14: NC; 15: NC; 16: NC; 17: NC; 18: NC; 19: NC; 20: NC; 21: NC; 22: NC |
| J3 | REGULATED 5V ONLY | 1: GND; 2: 5V_EXT |
| J4 | MPU6050 | 1: GND; 2: SENS_3V3; 3: I2C_SDA; 4: I2C_SCL |
| J5 | BMP280 | 1: GND; 2: SENS_3V3; 3: I2C_SDA; 4: I2C_SCL |
| J6 | HMC5883L | 1: GND; 2: SENS_3V3; 3: I2C_SDA; 4: I2C_SCL |
| J7 | SENSE ONLY | 1: VBAT_SENSE; 2: GND |
| J8 | ESC_FL | 1: ESC_FL; 2: GND |
| J9 | ESC_FR | 1: ESC_FR; 2: GND |
| J10 | ESC_RR | 1: ESC_RR; 2: GND |
| J11 | ESC_RL | 1: ESC_RL; 2: GND |
| J12 | OPTIONAL 3V3 UART | 1: COMP_TX; 2: COMP_RX; 3: GND |
| J13 | F2 MINI: VCC TBD | 1: GND; 2: NC; 3: LIDAR_RX_FC; 4: LIDAR_TX_FC |
| J14 | 0-2.5V current input | 1: GND; 2: SENS_3V3; 3: CURRENT_IN |
