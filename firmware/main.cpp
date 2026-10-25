#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <math.h>

#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "unknown"
#endif

// Lesson C5a: sensor X bias only. Y/Z, scale and mounting are not corrected yet.
static const float accel_bias_x = 1.308683406f;
static const int16_t expected_baseline_x = -2074;
static const int32_t offset_decrement = 2 * lroundf(accel_bias_x * 1024.0f / 9.80665f);
static const int32_t corrected_offset_x = expected_baseline_x - offset_decrement;

Adafruit_MPU6050 mpu;
Adafruit_I2CDevice sensor(0x68, &Wire);
Adafruit_BusIO_Register x_offset(&sensor, 0x06, 2, MSBFIRST);
Adafruit_BusIO_Register x_reading(&sensor, 0x3B, 2, MSBFIRST);
bool sensor_ready = false;
int16_t active_offset_x = expected_baseline_x;

static bool select_x_offset(int32_t requested)
{
    if (requested < -32768 || requested > 32767)
    {
        return false;
    }
    uint16_t readback;
    if (!x_offset.write((uint16_t)requested) || !x_offset.read(&readback))
    {
        return false;
    }
    if (readback != (uint16_t)requested)
    {
        return false;
    }
    active_offset_x = (int16_t)readback;
    Serial.printf("MODE,%d\n", active_offset_x);
    return true;
}

static void report(void)
{
    uint16_t readback;
    if (!sensor_ready || !x_offset.read(&readback) || (int16_t)readback != active_offset_x)
    {
        sensor_ready = false;
        Serial.printf("ERROR,OFFSET_OR_SENSOR_CHECK_FAILED\n");
        return;
    }
    Serial.printf("HELLO,BIAS_X_V1,%s\n", BENCH_SOURCE_SHA);
    Serial.printf("OFFSET_X,%d,%ld,%d\n", expected_baseline_x, (long)corrected_offset_x, (int16_t)readback);
    Serial.printf("CONFIG,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=accel_x_mpu_only\n");
    Serial.printf("READY,BIAS_X_V1\n");
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
    // Require the recorded post-reset baseline, preventing accumulated correction.
    uint16_t baseline = 0;
    sensor_ready = sensor_ready && x_offset.read(&baseline);
    Serial.printf("BASELINE_X,expected=%d,readback=%d,read_ok=%u\n", expected_baseline_x, (int16_t)baseline, (unsigned)sensor_ready);
    sensor_ready = sensor_ready && (int16_t)baseline == expected_baseline_x;
    sensor_ready = sensor_ready && select_x_offset(corrected_offset_x);
    report();
}

void loop(void)
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == 'r' && sensor_ready)
        {
            sensor_ready = select_x_offset(expected_baseline_x);
        }
        if (command == 'c' && sensor_ready)
        {
            sensor_ready = select_x_offset(corrected_offset_x);
        }
        if (command == '?')
        {
            Serial.printf("QUERY_REPLY\n");
            report();
        }
    }
    if (sensor_ready)
    {
        uint16_t bits;
        sensor_ready = x_reading.read(&bits);
        if (sensor_ready)
        {
            // ±8 g gives 4096 counts/g; bias is already handled in the MPU.
            int16_t counts = (int16_t)bits;
            float x = counts * (9.80665f / 4096.0f);
            Serial.printf("AX,%lu,%d,%d,%.6f\n", (unsigned long)millis(), active_offset_x, counts, x);
        }
    }
    if (!sensor_ready)
    {
        Serial.printf("ERROR,SENSOR_OR_OFFSET_FAILED\n");
    }
    delay(100);
}
