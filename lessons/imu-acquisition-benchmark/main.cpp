#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>

#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "unknown"
#endif

// Measurement harness only. No PWM/Servo/ESC/BLE, control or calibration writes.
Adafruit_MPU6050 mpu;
Adafruit_I2CDevice imu(0x68, &Wire);
static bool ready = false;
static const uint32_t window_us = 4000000;
static const uint16_t capacity = 5000;

struct Sample
{
    uint32_t service_start;
    uint32_t ready_observed;
    uint32_t read_start;
    uint32_t read_end;
    uint16_t fifo_bytes;
    int16_t raw[7];
};
static Sample samples[capacity];

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    return imu.write(bytes, 2);
}

static bool read_registers(uint8_t reg, uint8_t *bytes, uint8_t length)
{
    return imu.write_then_read(&reg, 1, bytes, length);
}

static void report(void)
{
    uint8_t who = 0;
    read_registers(0x75, &who, 1);
    Serial.printf("ACQ_BENCH_V1,%s,WHO=%u,READY=%u,WINDOW_US=%lu\n",
        BENCH_SOURCE_SHA, who, ready, (unsigned long)window_us);
    Serial.printf("SETTINGS,ACCEL=8g,GYRO=500dps,DLPF=0,DIV=7,I2C=400000,PACKET=14,NO_MOTOR_OUTPUT\n");
}

static bool check_config(void)
{
    uint8_t bytes[4];
    uint8_t power;
    if (!read_registers(0x19, bytes, 4) || !read_registers(0x6B, &power, 1))
    {
        return false;
    }
    return bytes[0] == 7 && (bytes[1] & 7) == 0
        && (bytes[2] & 0x18) == 0x08 && (bytes[3] & 0x18) == 0x10
        && (power & 0x7F) == 1 && Wire.getClock() == 400000;
}

static void capture(char mode)
{
    bool fifo = mode == 'f' || mode == 'F';
    bool pauses = mode == 'D' || mode == 'F';
    if (!ready || !check_config())
    {
        Serial.printf("ERROR,CONFIG_OR_SENSOR\n");
        return;
    }
    bool ok = write_register(0x23, 0) && write_register(0x6A, 0x04);
    delay(2);
    ok = write_register(0x6A, fifo ? 0x40 : 0) && ok;
    delay(20);
    uint8_t status = 0;
    ok = read_registers(0x3A, &status, 1) && ok;
    Serial.printf("START,%c\n", mode);
    Serial.flush();
    ok = write_register(0x23, fifo ? 0xF8 : 0) && ok;

    uint16_t count = 0;
    uint16_t queued = 0;
    uint32_t checksum = 2166136261u;
    uint32_t injected = 0;
    uint32_t pause_at = 100000;
    uint32_t started = micros();
    const char *failure = nullptr;
    while (ok && (uint32_t)(micros() - started) < window_us)
    {
        if (pauses && (uint32_t)(micros() - started) >= pause_at)
        {
            delayMicroseconds(2500);
            pause_at += 100000;
            injected++;
        }
        Sample row = {};
        row.service_start = micros() - started;
        if (!read_registers(0x3A, &status, 1))
        {
            failure = "STATUS_READ";
            break;
        }
        if (status & 0x10)
        {
            failure = "FIFO_OVERFLOW";
            break;
        }
        if (!(status & 1) && queued == 0)
        {
            continue;
        }
        row.ready_observed = micros() - started;
        if (fifo)
        {
            uint8_t size[2];
            if (!read_registers(0x72, size, 2))
            {
                failure = "FIFO_COUNT_READ";
                break;
            }
            row.fifo_bytes = ((uint16_t)size[0] << 8) | size[1];
            if (row.fifo_bytes >= 1024)
            {
                failure = "FIFO_FULL";
                break;
            }
            queued = row.fifo_bytes / 14;
            if (queued == 0)
            {
                continue;
            }
        }
        if (count >= capacity)
        {
            failure = "RECORD_BUFFER_FULL";
            break;
        }
        uint8_t bytes[14];
        row.read_start = micros() - started;
        if (!read_registers(fifo ? 0x74 : 0x3B, bytes, 14))
        {
            failure = "DATA_READ";
            break;
        }
        row.read_end = micros() - started;
        for (uint8_t i = 0; i < 7; i++)
        {
            row.raw[i] = (int16_t)(((uint16_t)bytes[2 * i] << 8) | bytes[2 * i + 1]);
        }
        for (uint8_t i = 0; i < 14; i++)
        {
            checksum = (checksum ^ bytes[i]) * 16777619u;
        }
        samples[count++] = row;
        if (fifo)
        {
            queued--;
        }
    }
    uint32_t elapsed = micros() - started;
    uint8_t tail_size[2] = {};
    uint16_t tail = 0;
    if (fifo && read_registers(0x72, tail_size, 2))
    {
        tail = ((uint16_t)tail_size[0] << 8) | tail_size[1];
    }
    ok = write_register(0x23, 0) && write_register(0x6A, 0) && ok;
    ok = check_config() && ok;
    Serial.printf("DATA,%c,%u,%lu,%lu,%u,%08lX\n", mode, count,
        (unsigned long)elapsed, (unsigned long)injected, tail, (unsigned long)checksum);
    for (uint16_t i = 0; i < count; i++)
    {
        const Sample &row = samples[i];
        Serial.printf("S,%u,%lu,%lu,%lu,%lu,%u", i,
            (unsigned long)row.service_start, (unsigned long)row.ready_observed,
            (unsigned long)row.read_start, (unsigned long)row.read_end, row.fifo_bytes);
        for (uint8_t j = 0; j < 7; j++)
        {
            Serial.printf(",%d", row.raw[j]);
        }
        Serial.printf("\n");
    }
    Serial.printf("DONE,%c,%s\n", mode, failure ? failure : (ok ? "OK" : "I2C_OR_CONFIG_FAILURE"));
    Serial.flush();
}

void setup(void)
{
    const uint8_t pins[] = {12, 1, 2, 14, 4, 21};
    for (uint8_t i = 0; i < sizeof(pins); i++)
    {
        pinMode(pins[i], INPUT_PULLDOWN);
    }
    Serial.begin(921600);
    Wire.begin(9, 8, 400000);
    Wire.setTimeOut(10);
    ready = mpu.begin(0x68, &Wire) && imu.begin();
    if (ready)
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        mpu.setSampleRateDivisor(7);
        mpu.setClock(MPU6050_PLL_GYROX);
        Wire.setClock(400000);
        ready = write_register(0x37, 0) && write_register(0x38, 0x11) && check_config();
    }
    report();
}

void loop(void)
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == '?')
        {
            report();
        }
        if (command == 'd' || command == 'f' || command == 'D' || command == 'F')
        {
            capture(command);
        }
    }
    delay(1);
}
