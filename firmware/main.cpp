#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <math.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

// Editable configuration. These defaults match the previous Adafruit setup.
static const uint8_t mpu_address = 0x68;
static const int sda_pin = 9;
static const int scl_pin = 8;
static const uint32_t i2c_clock_hz = 100000;
static const mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G;
static const mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_500_DEG;
static const mpu6050_bandwidth_t filter_bandwidth = MPU6050_BAND_260_HZ;
static const uint8_t sample_divider = 0;
static const mpu6050_clock_select_t clock_source = MPU6050_PLL_GYROX;

Adafruit_MPU6050 mpu;
Adafruit_I2CDevice register_device(mpu_address, &Wire);
bool mpu_started = false;

// Six-pose accelerometer biases, in MPU sensor axes and m/s^2.
// These constants are stored with the firmware in ESP32 flash.
static const float accel_bias_x = 1.293084598f;
static const float accel_bias_y = -0.079166657f;
static const float accel_bias_z = -0.639507624f;

// Accelerometer offset words: X=0x06, Y=0x08, Z=0x0A.
// BusIO is already used by Adafruit; no second MPU library is needed.
static bool write_accel_bias(uint8_t address, float bias, const char *axis)
{
    Adafruit_BusIO_Register offset(&register_device, address, 2, MSBFIRST);
    uint16_t baseline_bits;
    if (!offset.read(&baseline_bits))
    {
        Serial.printf("MPU_OFFSET %s: baseline read failed\n", axis);
        return false;
    }

    int16_t baseline = (int16_t)baseline_bits;
    int32_t change = 2 * lroundf(bias * 1024.0f / 9.80665f);
    int32_t requested = (int32_t)baseline - change;
    if (requested < -32768 || requested > 32767)
    {
        return false;
    }
    // Even changes preserve the factory temperature-compensation bit.
    if (!offset.write((uint16_t)requested))
    {
        return false;
    }
    uint16_t readback;
    if (!offset.read(&readback))
    {
        return false;
    }
    Serial.printf("MPU_OFFSET %s: baseline=%d requested=%d readback=%d\n",
        axis, baseline, (int)requested, (int16_t)readback);
    return readback == (uint16_t)requested;
}

void setup(void)
{
    Serial.begin(115200);

    // No motor commands: signal pins remain inputs pulled low.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    bool i2c_started = Wire.begin(sda_pin, scl_pin, i2c_clock_hz);
    if (i2c_started)
    {
        mpu_started = mpu.begin(mpu_address, &Wire);
    }
    Serial.printf("I2C started : %s.\n", i2c_started ? "Yes" : "No");
    Serial.printf("MPU6050 initialized : %s.\n", mpu_started ? "Yes" : "No");
    if (mpu_started)
    {
        mpu.setAccelerometerRange(accel_range);
        mpu.setGyroRange(gyro_range);
        mpu.setFilterBandwidth(filter_bandwidth);
        mpu.setSampleRateDivisor(sample_divider);
        mpu.setClock(clock_source);

        bool config_ok = mpu.getAccelerometerRange() == accel_range;
        config_ok = config_ok && mpu.getGyroRange() == gyro_range;
        config_ok = config_ok && mpu.getFilterBandwidth() == filter_bandwidth;
        config_ok = config_ok && mpu.getSampleRateDivisor() == sample_divider;
        config_ok = config_ok && mpu.getClock() == clock_source;

        mpu_started = config_ok && register_device.begin();
        if (mpu_started)
        {
            mpu_started = write_accel_bias(0x06, accel_bias_x, "X");
        }
        if (mpu_started)
        {
            mpu_started = write_accel_bias(0x08, accel_bias_y, "Y");
        }
        if (mpu_started)
        {
            mpu_started = write_accel_bias(0x0A, accel_bias_z, "Z");
        }
        Serial.printf("MPU_CONFIG accel=%u gyro=%u dlpf=%u divider=%u clock=%u verified=%s\n",
            (unsigned)accel_range, (unsigned)gyro_range, (unsigned)filter_bandwidth,
            (unsigned)sample_divider, (unsigned)clock_source, mpu_started ? "Yes" : "No");
    }
}

void loop(void)
{
    if (mpu_started)
    {
        sensors_event_t acceleration;
        mpu.getAccelerometerSensor()->getEvent(&acceleration);

        // Bias is already corrected in the MPU registers; do not subtract again.
        float sensor_x = acceleration.acceleration.x;
        float sensor_y = acceleration.acceleration.y;
        float sensor_z = acceleration.acceleration.z;

        // Then correct scale/mounting: output is in DRONE axes.
        float drone_x = 0.046659658f * sensor_x + 0.992539802f * sensor_y - 0.045743511f * sensor_z;
        float drone_y = -0.955388252f * sensor_x + 0.053316466f * sensor_y + 0.289431328f * sensor_z;
        float drone_z = 0.272504170f * sensor_x + 0.023832247f * sensor_y + 0.951963666f * sensor_z;

        Serial.printf("MPU_BIAS_CORRECTED X=%.4f Y=%.4f Z=%.4f m/s^2\n",
            acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z);
        float magnitude = sqrtf(drone_x * drone_x + drone_y * drone_y + drone_z * drone_z);
        Serial.printf("DRONE X=%.4f Y=%.4f Z=%.4f norm=%.4f m/s^2\n", drone_x, drone_y, drone_z, magnitude);
    }
    delay(100);
}
