#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

void setup(void)
{
    Serial.begin(115200);

    // Keep motor signal pins inactive. No motor commands are generated.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    bool i2c_started = Wire.begin(9, 8, 100000);
    Serial.print("I2C controller started (1=yes, 0=no): ");
    Serial.println(i2c_started);

    bool mpu_started = false;
    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.print("MPU6050 initialized (1=yes, 0=no): ");
    Serial.println(mpu_started);
}

void loop(void)
{
    Serial.print("ESP32 is running, uptime = ");
    Serial.print(millis());
    Serial.println(" ms");
    delay(1000);
}