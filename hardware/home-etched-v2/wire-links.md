# Mandatory insulated wire links

Use the interactive assembly guide to highlight a pair of holes. Coordinates: mm, component side, origin top left. Route wires flat and clear of socket contacts, USB and antenna. Do not use bare wire. W1-W27 are fitted; JP1-JP4 are separate power-selection links.

| Link | Logical net | A: X, Y mm | B: X, Y mm | Straight distance mm |
|---|---|---|---|---|
| W1 | GND | 47.244, 15.240 | 45.720, 26.924 | 11.78 |
| W2 | GND | 69.850, 15.748 | 69.850, 25.146 | 9.40 |
| W3 | 3V3 | 63.500, 33.274 | 63.500, 37.846 | 4.57 |
| W4 | 3V3 | 46.736, 49.276 | 46.736, 40.894 | 8.38 |
| W5 | 3V3 | 32.766, 12.700 | 45.466, 12.700 | 12.70 |
| W6 | FC_5V | 77.470, 56.896 | 77.470, 86.106 | 29.21 |
| W7 | I2C_SCL | 67.056, 19.050 | 64.516, 43.180 | 24.26 |
| W8 | I2C_SCL | 62.484, 18.034 | 71.882, 18.034 | 9.40 |
| W9 | I2C_SCL | 33.020, 26.416 | 38.100, 26.416 | 5.08 |
| W10 | I2C_SDA | 62.484, 52.578 | 65.532, 45.974 | 7.27 |
| W11 | I2C_SDA | 60.452, 20.320 | 69.850, 20.320 | 9.40 |
| W12 | I2C_SDA | 33.020, 51.816 | 48.514, 51.816 | 15.49 |
| W13 | ESC1_SIGNAL | 32.766, 61.214 | 44.450, 78.486 | 20.85 |
| W14 | ESC2_BEC | 56.896, 80.772 | 56.896, 76.708 | 4.06 |
| W15 | ESC2_SIGNAL | 53.594, 76.454 | 42.164, 18.034 | 59.53 |
| W16 | ESC3_BEC | 67.056, 80.772 | 67.056, 76.708 | 4.06 |
| W17 | ESC3_SIGNAL | 63.754, 76.454 | 39.624, 20.574 | 60.87 |
| W18 | ESC4_BEC | 77.216, 80.772 | 77.216, 76.708 | 4.06 |
| W19 | ESC4_SIGNAL | 23.876, 61.214 | 73.914, 76.454 | 52.31 |
| W20 | FC_TX_TO_LIDAR | 77.216, 66.548 | 24.130, 36.068 | 61.21 |
| W21 | FC_UART_RX | 21.844, 65.786 | 21.844, 34.036 | 31.75 |
| W22 | FC_UART_TX | 38.862, 66.040 | 4.572, 66.040 | 34.29 |
| W23 | LIDAR_TO_FC_RX | 77.216, 63.500 | 9.906, 23.368 | 78.37 |
| W24 | MAG_DRDY | 9.906, 35.306 | 63.246, 48.006 | 54.83 |
| W25 | MPU_AUX_SCL | 48.260, 25.400 | 48.006, 45.720 | 20.32 |
| W26 | MPU_AUX_SDA | 49.784, 22.860 | 49.784, 48.260 | 25.40 |
| W27 | MPU_INT | 48.006, 30.226 | 9.906, 30.226 | 38.10 |

Distance is not the wire cut length: allow for routing, insertion and soldering. Fit every W link before testing continuity.
