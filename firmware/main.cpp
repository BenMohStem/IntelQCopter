#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu(0x68);
bool mpu_started = false;

// Measured biases in m/s^2, stored with the firmware in ESP32 flash.
static const float accel_bias_x = 1.293084598f;
static const float accel_bias_y = -0.079166657f;
static const float accel_bias_z = -0.639507624f;

static int32_t corrected_offset(int16_t factory_offset, float bias)
{
    // At +/-2g: 8 output counts per offset-register count.
    // An even change preserves the temperature-compensation bit (bit 0).
    int32_t change = 2 * lroundf(bias * 1024.0f / 9.80665f);
    return (int32_t)factory_offset - change;
}

static bool apply_accel_offsets(void)
{
    int16_t factory_x = mpu.getXAccelOffset();
    int16_t factory_y = mpu.getYAccelOffset();
    int16_t factory_z = mpu.getZAccelOffset();

    int32_t offset_x = corrected_offset(factory_x, accel_bias_x);
    int32_t offset_y = corrected_offset(factory_y, accel_bias_y);
    int32_t offset_z = corrected_offset(factory_z, accel_bias_z);
    if (offset_x < -32768 || offset_x > 32767 || offset_y < -32768 || offset_y > 32767 || offset_z < -32768 || offset_z > 32767)
    {
        Serial.printf("MPU_OFFSETS,out_of_range\n");
        return false;
    }

    mpu.setXAccelOffset((int16_t)offset_x);
    mpu.setYAccelOffset((int16_t)offset_y);
    mpu.setZAccelOffset((int16_t)offset_z);

    int16_t read_x = mpu.getXAccelOffset();
    int16_t read_y = mpu.getYAccelOffset();
    int16_t read_z = mpu.getZAccelOffset();
    Serial.printf("MPU_BASE_OFFSETS X=%d Y=%d Z=%d\n", factory_x, factory_y, factory_z);
    Serial.printf("MPU_APPLIED_OFFSETS X=%d Y=%d Z=%d\n", read_x, read_y, read_z);
    return read_x == offset_x && read_y == offset_y && read_z == offset_z;
}

void setup(void)
{
    Serial.begin(115200);

    // No motor commands: signal pins remain inputs pulled low.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    bool i2c_started = Wire.begin(9, 8, 100000);
    Serial.printf("I2C started : %s.\n", i2c_started ? "Yes" : "No");
    if (i2c_started && mpu.testConnection())
    {
        // Restore sensor defaults before reading its baseline offset registers.
        mpu.reset();
        delay(100);
        mpu.initialize();
        delay(100);
        mpu_started = mpu.testConnection();
        if (mpu_started)
        {
            mpu_started = mpu.getFullScaleAccelRange() == MPU6050_ACCEL_FS_2;
        }
        if (mpu_started)
        {
            mpu_started = apply_accel_offsets();
        }
    }
    Serial.printf("MPU initialized and accel offsets verified : %s.\n", mpu_started ? "Yes" : "No");
}

void loop(void)
{
    if (mpu_started)
    {
        int16_t raw_x, raw_y, raw_z;
        mpu.getAcceleration(&raw_x, &raw_y, &raw_z);

        // Hardware already applies the bias: do NOT subtract it again.
        float sensor_x = raw_x * (9.80665f / 16384.0f);
        float sensor_y = raw_y * (9.80665f / 16384.0f);
        float sensor_z = raw_z * (9.80665f / 16384.0f);

        // Scale and sensor-to-drone mounting correction remain in software.
        float drone_x = 0.046659658f * sensor_x + 0.992539802f * sensor_y - 0.045743511f * sensor_z;
        float drone_y = -0.955388252f * sensor_x + 0.053316466f * sensor_y + 0.289431328f * sensor_z;
        float drone_z = 0.272504170f * sensor_x + 0.023832247f * sensor_y + 0.951963666f * sensor_z;

        Serial.printf("DRONE X=%.4f Y=%.4f Z=%.4f m/s^2\n", drone_x, drone_y, drone_z);
    }
    delay(100);
}
