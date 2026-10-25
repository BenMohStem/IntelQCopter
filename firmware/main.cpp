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

// Frozen from gyro runs 1-2; run 3 was held out. Units: sensor rad/s.
static const float gyro_bias[3] = {0.003098404f, -0.004985037f, -0.014435425f};
static const int16_t gyro_baseline[3] = {0, 0, 0};
int16_t gyro_applied_offset[3] = { 0 };
int16_t gyro_boot_offset[3] = { 0 };
float gyro_remaining_bias[3] = { 0 };

Adafruit_MPU6050 mpu;
Adafruit_I2CDevice sensor(0x68, &Wire);
bool sensor_ready = false;
int16_t applied_offset[3] = { 0 };
int16_t boot_offset[3] = { 0 };
float remaining_bias[3] = { 0 };

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

static bool restore_gyro_bias(void)
{
    // Offset words use nominal 32.8 counts/(degree/s), not the output scale 65.5.
    // Validate all axes first; write absolute targets to avoid accumulating trim.
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        int32_t decrement = lroundf(gyro_bias[axis] * (180.0f / PI) * 32.8f);
        int32_t target = (int32_t)gyro_baseline[axis] - decrement;
        if (target < -32768 || target > 32767)
        {
            return false;
        }
        gyro_applied_offset[axis] = (int16_t)target;
        gyro_remaining_bias[axis] = gyro_bias[axis] - decrement * (PI / (180.0f * 32.8f));
        Adafruit_BusIO_Register reg(&sensor, 0x13 + 2 * axis, 2, MSBFIRST);
        uint16_t observed;
        if (!reg.read(&observed))
        {
            return false;
        }
        gyro_boot_offset[axis] = (int16_t)observed;
        if (gyro_boot_offset[axis] != gyro_baseline[axis] && gyro_boot_offset[axis] != gyro_applied_offset[axis])
        {
            Serial.printf("ERROR,UNEXPECTED_GYRO_BOOT_OFFSET,axis=%u,value=%d\n", axis, gyro_boot_offset[axis]);
            return false;
        }
    }
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        Adafruit_BusIO_Register reg(&sensor, 0x13 + 2 * axis, 2, MSBFIRST);
        uint16_t readback;
        if (!reg.write((uint16_t)gyro_applied_offset[axis]) || !reg.read(&readback) || (int16_t)readback != gyro_applied_offset[axis])
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
        Adafruit_BusIO_Register gyro_reg(&sensor, 0x13 + 2 * axis, 2, MSBFIRST);
        if (!gyro_reg.read(&readback) || (int16_t)readback != gyro_applied_offset[axis])
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
    Serial.printf("HELLO,IMU_XYZ_V2,%s\n", BENCH_SOURCE_SHA);
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
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        Serial.printf("GYRO_BIAS,%u,%.9f\n", axis, gyro_bias[axis]);
        Serial.printf("GYRO_BOOT_OFFSET,%u,%d\n", axis, gyro_boot_offset[axis]);
        Serial.printf("GYRO_OFFSET,%u,%d\n", axis, gyro_applied_offset[axis]);
        Serial.printf("GYRO_TRIM,%u,%d,%d,%.9f\n", axis,
            gyro_baseline[axis], gyro_applied_offset[axis], gyro_remaining_bias[axis]);
    }
    Serial.printf("GYRO_SCALE,65.5,counts_per_degree_per_second,frame=sensor,correction=mpu_plus_remainder\n");
    Serial.printf("READY,IMU_XYZ_V2\n");
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
    sensor_ready = sensor_ready && restore_gyro_bias();
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
        uint8_t bytes[14];
        sensor_ready = verify_offsets() && sensor.write_then_read(&address, 1, bytes, sizeof(bytes));
        if (sensor_ready)
        {
            int16_t counts[7];
            float drone[3];
            float gyro[3];
            // Packet order: accel XYZ, temperature, gyro XYZ.
            for (uint8_t axis = 0; axis < 7; axis++)
            {
                counts[axis] = (int16_t)(((uint16_t)bytes[2 * axis] << 8) | bytes[2 * axis + 1]);
            }
            correct_acceleration(counts, drone);
            for (uint8_t axis = 0; axis < 3; axis++)
            {
                gyro[axis] = counts[axis + 4] * (PI / (180.0f * 65.5f)) - gyro_remaining_bias[axis];
            }
            float temperature = counts[3] / 340.0f + 36.53f;
            Serial.printf("IMU,%lu,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.3f\n",
                (unsigned long)millis(), counts[0], counts[1], counts[2], counts[3],
                counts[4], counts[5], counts[6], drone[0], drone[1], drone[2],
                gyro[0], gyro[1], gyro[2], temperature);
        }
    }
    if (!sensor_ready)
    {
        Serial.printf("ERROR,SENSOR_OR_CALIBRATION_FAILED\n");
    }
    delay(10);
}
