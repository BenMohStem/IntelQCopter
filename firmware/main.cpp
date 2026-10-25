#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <math.h>

#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "unknown"
#endif

// Fixed calibration from repetitions 1-2. Valid for the unchanged sensor mounting.
static const float accel_bias[3] = {1.308683406f, -0.067669726f, -0.657188793f};
static const float accel_matrix[3][3] = {
    {0.953457734f, -0.085144142f, -0.287271311f},
    {0.088080052f, 0.990761548f, -0.002594316f},
    {0.287429504f, -0.030297733f, 0.947299771f}
};
static const int16_t expected_baseline[3] = {-2074, 2979, 1074};

Adafruit_MPU6050 mpu;
Adafruit_I2CDevice sensor(0x68, &Wire);
bool sensor_ready = false;
int16_t applied_offset[3] = {};
int16_t boot_offset[3] = {};
float remaining_bias[3] = {};

static bool restore_accel_bias(void)
{
    // Accept the original baseline or our own retained target, never a new unknown value.
    // Validate every axis before writing; every write uses an absolute target.
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        int32_t decrement = 2 * lroundf(accel_bias[axis] * 1024.0f / 9.80665f);
        int32_t target = (int32_t)expected_baseline[axis] - decrement;
        if (target < -32768 || target > 32767)
        {
            return false;
        }
        applied_offset[axis] = (int16_t)target;
        remaining_bias[axis] = accel_bias[axis] - decrement * (9.80665f / 2048.0f);
        Adafruit_BusIO_Register reg(&sensor, 0x06 + 2 * axis, 2, MSBFIRST);
        uint16_t observed;
        if (!reg.read(&observed))
        {
            return false;
        }
        boot_offset[axis] = (int16_t)observed;
        if (boot_offset[axis] != expected_baseline[axis] && boot_offset[axis] != applied_offset[axis])
        {
            Serial.printf("ERROR,UNEXPECTED_BOOT_OFFSET,axis=%u,value=%d\n", axis, boot_offset[axis]);
            return false;
        }
    }
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        Adafruit_BusIO_Register reg(&sensor, 0x06 + 2 * axis, 2, MSBFIRST);
        uint16_t readback;
        if (!reg.write((uint16_t)applied_offset[axis]) || !reg.read(&readback) || (int16_t)readback != applied_offset[axis])
        {
            return false;
        }
    }
    delay(20);
    return true;
}

static bool verify_offsets(void)
{
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        Adafruit_BusIO_Register reg(&sensor, 0x06 + 2 * axis, 2, MSBFIRST);
        uint16_t readback;
        if (!reg.read(&readback) || (int16_t)readback != applied_offset[axis])
        {
            return false;
        }
    }
    return true;
}

static void correct_acceleration(const int16_t counts[3], float drone[3])
{
    float centred[3];
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        centred[axis] = counts[axis] * (9.80665f / 4096.0f) - remaining_bias[axis];
    }
    for (uint8_t row = 0; row < 3; row++)
    {
        drone[row] = 0.0f;
        for (uint8_t column = 0; column < 3; column++)
        {
            drone[row] += accel_matrix[row][column] * centred[column];
        }
    }
}

static void report(void)
{
    if (!sensor_ready || !verify_offsets())
    {
        sensor_ready = false;
        Serial.printf("ERROR,SENSOR_OR_OFFSET_CHECK_FAILED\n");
        return;
    }
    Serial.printf("HELLO,ACCEL_XYZ_V1,%s\n", BENCH_SOURCE_SHA);
    Serial.printf("CONFIG,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=xyz_mpu_plus_matrix\n");
    Serial.printf("BOARD,%s,%012llX\n", ESP.getChipModel(), ESP.getEfuseMac());
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        Serial.printf("BIAS,%u,%.9f\n", axis, accel_bias[axis]);
        Serial.printf("BOOT_OFFSET,%u,%d\n", axis, boot_offset[axis]);
        Serial.printf("ROW,%u,%.9f,%.9f,%.9f\n", axis,
            accel_matrix[axis][0], accel_matrix[axis][1], accel_matrix[axis][2]);
        Serial.printf("OFFSET,%u,%d,%d,%.9f\n", axis,
            expected_baseline[axis], applied_offset[axis], remaining_bias[axis]);
    }
    Serial.printf("READY,ACCEL_XYZ_V1\n");
}

void setup(void)
{
    Serial.begin(921600);
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    sensor_ready = Wire.begin(9, 8, 400000) && mpu.begin(0x68, &Wire) && sensor.begin();
    if (sensor_ready)
    {
        Wire.setClock(400000);
        Wire.setTimeOut(10);
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        mpu.setSampleRateDivisor(7);
        mpu.setClock(MPU6050_PLL_GYROX);
        sensor_ready = mpu.getAccelerometerRange() == MPU6050_RANGE_8_G;
        sensor_ready = sensor_ready && mpu.getGyroRange() == MPU6050_RANGE_500_DEG;
        sensor_ready = sensor_ready && mpu.getFilterBandwidth() == MPU6050_BAND_260_HZ;
        sensor_ready = sensor_ready && mpu.getSampleRateDivisor() == 7;
        sensor_ready = sensor_ready && mpu.getClock() == MPU6050_PLL_GYROX;
        sensor_ready = sensor_ready && Wire.getClock() == 400000;
    }
    sensor_ready = sensor_ready && restore_accel_bias();
    report();
}

void loop(void)
{
    if (Serial.available() && Serial.read() == '?')
    {
        Serial.printf("QUERY_REPLY\n");
        report();
    }
    if (sensor_ready)
    {
        uint8_t address = 0x3B;
        uint8_t bytes[6];
        sensor_ready = verify_offsets() && sensor.write_then_read(&address, 1, bytes, sizeof(bytes));
        if (sensor_ready)
        {
            int16_t counts[3];
            float drone[3];
            for (uint8_t axis = 0; axis < 3; axis++)
            {
                counts[axis] = (int16_t)(((uint16_t)bytes[2 * axis] << 8) | bytes[2 * axis + 1]);
            }
            correct_acceleration(counts, drone);
            Serial.printf("ACCEL,%lu,%d,%d,%d,%.6f,%.6f,%.6f\n", (unsigned long)millis(),
                counts[0], counts[1], counts[2], drone[0], drone[1], drone[2]);
        }
    }
    if (!sensor_ready)
    {
        Serial.printf("ERROR,SENSOR_OR_CALIBRATION_FAILED\n");
    }
    delay(10);
}
