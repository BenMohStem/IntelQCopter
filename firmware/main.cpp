#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
bool mpu_started = false;

// Six-pose accelerometer biases, in MPU sensor axes and m/s^2.
// These constants are stored with the firmware in ESP32 flash.
static const float accel_bias_x = 1.293084598f;
static const float accel_bias_y = -0.079166657f;
static const float accel_bias_z = -0.639507624f;

void setup(void)
{
    Serial.begin(115200);

    // No motor commands: signal pins remain inputs pulled low.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    bool i2c_started = Wire.begin(9, 8, 100000);
    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.printf("I2C started : %s.\n", i2c_started ? "Yes" : "No");
    Serial.printf("MPU6050 initialized : %s.\n", mpu_started ? "Yes" : "No");
    Serial.printf("Calibration : six-pose candidate, stored in firmware flash.\n");
}

void loop(void)
{
    if (mpu_started)
    {
        sensors_event_t acceleration;
        mpu.getAccelerometerSensor()->getEvent(&acceleration);

        // First remove the bias, keeping MPU sensor axes.
        float sensor_x = acceleration.acceleration.x - accel_bias_x;
        float sensor_y = acceleration.acceleration.y - accel_bias_y;
        float sensor_z = acceleration.acceleration.z - accel_bias_z;

        // Then correct scale/mounting: output is in DRONE axes.
        float drone_x = 0.046659658f * sensor_x + 0.992539802f * sensor_y - 0.045743511f * sensor_z;
        float drone_y = -0.955388252f * sensor_x + 0.053316466f * sensor_y + 0.289431328f * sensor_z;
        float drone_z = 0.272504170f * sensor_x + 0.023832247f * sensor_y + 0.951963666f * sensor_z;

        Serial.printf("RAW_SENSOR X=%.4f Y=%.4f Z=%.4f m/s^2\n",
            acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z);
        Serial.printf("DRONE X=%.4f Y=%.4f Z=%.4f m/s^2\n", drone_x, drone_y, drone_z);
    }
    delay(1000);
}
