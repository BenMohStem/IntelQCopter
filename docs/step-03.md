# Step 03 - start I2C with Wire

## One new idea

I2C is the shared communication bus for our sensors. SDA carries data; SCL carries the clock. Arduino's Wire library manages that bus. This lesson only starts the ESP32 I2C controller; it does not ask a sensor for anything.

## The small change

At the top, add:

```cpp
#include <Wire.h>
```

Wire is included with the installed Arduino framework. No extra library download or platformio.ini change is needed.

After `Serial.begin(115200);`, add:

```cpp
    bool i2c_started = Wire.begin(9, 8, 100000);
    Serial.print("I2C controller started (1=yes, 0=no): ");
    Serial.println(i2c_started);
```

Read the first instruction from right to left:

1. `Wire.begin(9, 8, 100000)` starts I2C: SDA is GPIO9, SCL is GPIO8, clock is 100000 Hz (100 kHz).
2. It returns `true` if the local I2C controller starts, or `false` if it fails.
3. `bool i2c_started` stores that yes/no result.
4. The next two instructions print the label and result. Serial prints a boolean as `1` or `0`.

`setup()` runs once, so the I2C result is printed once. The existing uptime messages continue in `loop()` without changes. Motor outputs remain disabled.

## Wiring assumptions

GPIO numbers refer to the board's labelled GPIOs, not header hole numbers. These are the pins currently specified in config/hardware.json; they are not confirmed against your actual assembly.

| ESP32-S3 connection | Sensor connection |
|---|---|
| GPIO9 | SDA |
| GPIO8 | SCL |
| GND | Common GND |

Use a suitable 3.3 V supply for the identified sensor modules and keep SDA/SCL pull-ups at 3.3 V. Do not connect 5 V logic to ESP32 GPIOs. I2C needs pull-up resistors; modules may already contain them. Confirm the module wiring and power before connecting, with power off.

## Your test

If step02 has not worked yet, finish that test first. Then close Monitor, upload step03 yourself, and reopen Monitor at 115200 baud. Keep propellers removed. Press RESET/EN after opening Monitor if you missed the startup line.

Expected example, not measured evidence:

```text
I2C controller started (1=yes, 0=no): 1
ESP32 is running, uptime = 25 ms
ESP32 is running, uptime = 1025 ms
```

Check that the startup result is `1` and uptime continues increasing. If it is `0`, keep the full startup output and stop here for diagnosis.

**A result of 1 does not prove a sensor is present, correctly wired or calibrated.** It can occur with no sensor attached. The next lesson checks the MPU6050 response.

## Status

Prepared only. No build, upload or physical test performed by the assistant. The catalogue's `implemented: true` records code presence, not physical acceptance. Preserve your actual output locally and report what you see.
