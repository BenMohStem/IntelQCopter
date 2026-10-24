#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
double avrg_xacc =  3.350011f;
double avrg_yacc = -0.411099f;
double avrg_zacc = -0.716789f;
bool mpu_started = false;
bool i2c_started = false;

void setup(void)
{
    Serial.begin(115200);

    // Keep motor signal pins inactive. No motor commands are generated.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    i2c_started = Wire.begin(9, 8, 100000);
    Serial.printf("I2C controller started : %s.\n", (i2c_started) ? "Yes" : "No");

    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.printf("MPU6050 initialized : %s.\n", (mpu_started) ? "Yes" : "No");
}

void loop(void)
{
    Serial.printf("ESP32 is running, uptime = %lu ms\n", millis());
    if (i2c_started && mpu_started)
    {
        sensors_event_t acceleration;
        mpu.getAccelerometerSensor()->getEvent(&acceleration);
        Serial.printf("Acceleration x = %g m/s^2\n", acceleration.acceleration.x - avrg_xacc);
        Serial.printf("Acceleration y = %g m/s^2\n", acceleration.acceleration.y - avrg_yacc);
        Serial.printf("Acceleration z = %g m/s^2\n", acceleration.acceleration.z - avrg_zacc);

        sensors_event_t rotation;
        mpu.getGyroSensor()->getEvent(&rotation);
        Serial.printf("Gyro Z = %.3f rad/s\n", rotation.gyro.z);
    }
    delay(1000);
}