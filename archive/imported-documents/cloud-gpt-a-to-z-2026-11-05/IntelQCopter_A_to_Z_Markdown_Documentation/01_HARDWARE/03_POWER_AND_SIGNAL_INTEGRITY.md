# Power, PCB integrity and electrical risks

## Known power system

Battery: 3-cell 4200 mAh lithium-polymer pack. Four nominal 40 A ESCs distribute high transient motor currents. The ESP32-S3, IMU, MAG, and BMP run from low-voltage regulated rails whose detailed schematic, brownout thresholds, and rail decoupling have not been independently examined.

## Historic QMC problem

Older tests detected `0x2C` with chip ID `0x80` and wrote `CTRL1`, then lost I²C reads after entering measurement mode. Possible root causes included sensor-specific repeated-START compatibility, supply integrity, reset timing, MPU bypass interference or bus loading. Wi-Fi transmission introduced interference at some configurations, but failure also appeared with Wi-Fi off. Do not assert “Wi-Fi was the cause.”

## What to inspect with instrument

1. Verify regulated 3.3 V at the sensor *pins* at boot, radio-disabled operation, and configuration changes.
2. Observe 3.3 V droop and ripple on an oscilloscope during motor/ESC switching (later controlled lab test).
3. Check SDA/SCL rise times, stuck-low events and pull-ups with a logic analyzer or scope; judge whether 400 kHz is physically supported.
4. Shorten noisy lines, maintain grounds, route magnetometer away from power conductors and motors where practical.
5. Record cold power cycle versus warm reset; a USB-only disconnect may not depower the sensor if another supply remains connected.
6. Keep LiPo connections isolated from any direct ESP32 GPIO or 3.3 V pin.

## Acceptance

No unexplained resetting, voltage collapse, I²C bus lockup, serial loss, or sensor invalidity in a sustained bench run. Strong magnetic fields and current-generated offsets must be checked again with controlled motor-power tests; a static compass calibration is not the full story.
