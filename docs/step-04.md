# Step 04 - does the MPU6050 initialize?

## Goal

Use the existing Arduino library to find and initialize the MPU6050. No acceleration readings, gyro readings or calibration yet. Your change moving Serial.begin earlier is preserved.

## Library configuration

platformio.ini now contains:

```ini
lib_deps =
    adafruit/Adafruit MPU6050@2.2.9
```

PlatformIO downloads this version and its declared dependencies when you build/upload. The library metadata also declares display-related dependencies used by its examples; this lesson does not use a display. Our firmware includes no display code.

## Read the new source instructions

```cpp
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
```

The include exposes the library. The second line creates one object named mpu, which manages our sensor. This is the small amount of C++ needed to use this Arduino library. It sits outside setup and loop so both functions can use the same sensor later.

After the Wire startup result, setup adds:

```cpp
    bool mpu_started = false;
    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.print("MPU6050 initialized (1=yes, 0=no): ");
    Serial.println(mpu_started);
```

1. Start with a result of false: success must be established.
2. `if (i2c_started)` attempts initialization only if the previous Wire setup succeeded.
3. `0x68` is a hexadecimal I2C address, not a GPIO number. It is the usual MPU6050 address with AD0 low. AD0 high selects 0x69; check your module wiring before changing the address.
4. `&Wire` passes the already configured I2C bus to the library. The ampersand means the address of the Wire object; it does not select a physical pin.
5. `mpu.begin(...)` checks communication and the chip identification register, then resets/configures the sensor with the library defaults. This is initialization, not just a passive bus scan.
6. Its returned true/false value replaces mpu_started. The last two instructions print the result as 1 or 0.

The loop still prints only uptime. It stays alive even when the sensor is missing, so you can read the failure message. No motor output is added.

## Your physical check

Finish step03 first if its I2C startup failed. With power off, verify module power, common ground, SDA to GPIO9 and SCL to GPIO8, with I2C logic at 3.3 V. Keep propellers removed. Close Monitor, upload yourself, reopen Monitor at 115200 baud, and press RESET/EN if needed to see startup output.

Expected example, not actual test evidence:

```text
I2C controller started (1=yes, 0=no): 1
MPU6050 initialized (1=yes, 0=no): 1
ESP32 is running, uptime = 500 ms
```

The exact uptime varies. If the MPU result is 0, stop and send the full startup log. It can indicate wiring, address, power, bus or chip-identity problems; do not assume one specific cause. A 1 is a useful connection/initialization result, not proof of correct axes, noise, calibration or flight suitability. You can repeat a negative test with the sensor disconnected only after powering off, then power off again before reconnecting.

## Status and sources

Prepared only. No build, flash or real sensor test performed by the assistant. implemented=true means code exists, not physical acceptance. Original hardware logs remain yours to preserve and report; no pass was invented.

Reference: [Adafruit MPU6050 library](https://github.com/adafruit/Adafruit_MPU6050), including its begin implementation and basic_readings example. Only initialization is used in this lesson. Next: one acceleration component.
