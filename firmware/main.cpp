#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

// Measurement firmware only. No user calibration, attitude filter or motor driver.
// The build script supplies the exact source SHA; no generated header is needed.
#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "unknown"
#endif

static const uint32_t sample_target = 20000;
static const uint32_t serial_baud = 921600;
static const uint8_t fifo_enable = 0x23;
static const uint8_t interrupt_status = 0x3A;
static const uint8_t user_control = 0x6A;
static const uint8_t fifo_count = 0x72;
static const uint8_t fifo_data = 0x74;

Adafruit_MPU6050 mpu;
Adafruit_I2CDevice sensor(0x68, &Wire);
bool sensor_ready = false;
bool recording = false;
uint32_t sample_count = 0;
uint32_t started_us = 0;
uint16_t maximum_fifo = 0;
uint32_t data_hash = 2166136261u;
int64_t sums[7] = {};

// Small checked helpers: Adafruit has no public FIFO sample-reading function.
static bool write_register(uint8_t address, uint8_t value)
{
    Adafruit_BusIO_Register reg(&sensor, address);
    return reg.write(value);
}

static bool read_registers(uint8_t address, uint8_t *bytes, uint8_t length)
{
    return sensor.write_then_read(&address, 1, bytes, length);
}

static void stop_recording(const char *reason)
{
    recording = false;
    bool stopped = write_register(fifo_enable, 0);
    stopped = write_register(user_control, 0) && stopped;
    Serial.printf("ERROR,%s,samples=%lu,stop_ok=%u\n", reason,
        (unsigned long)sample_count, (unsigned)stopped);
}

static bool report_configuration(void)
{
    uint8_t config[4];
    uint8_t power;
    if (!read_registers(0x19, config, 4) || !read_registers(0x6B, &power, 1))
    {
        return false;
    }
    // Divider 7 with DLPF mode 0: 8000 / (7 + 1) = 1000 samples/second.
    if (config[0] != 7 || config[1] != 0 || config[2] != 8 || config[3] != 16 || power != 1)
    {
        return false;
    }
    Serial.printf("HELLO,POSE1000_V1,%s\n", BENCH_SOURCE_SHA);
    Serial.printf("CONFIG,hz=1000,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=none\n");
    Serial.printf("BOARD,%s,%012llX\n", ESP.getChipModel(), ESP.getEfuseMac());
    // Preserve the actual post-reset offset words in the evidence. Do not write them.
    const uint8_t offsets[6] = {0x06, 0x08, 0x0A, 0x13, 0x15, 0x17};
    for (uint8_t index = 0; index < 6; index++)
    {
        uint8_t bytes[2];
        if (!read_registers(offsets[index], bytes, 2))
        {
            return false;
        }
        int16_t value = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
        Serial.printf("OFFSET,%02X,%d\n", offsets[index], value);
    }
    Serial.printf("READY,send_s_to_record_20000_samples\n");
    return true;
}

static void start_recording(void)
{
    // Disable sources, reset the FIFO, then start with a new complete packet.
    bool ok = write_register(fifo_enable, 0);
    ok = write_register(user_control, 0) && ok;
    ok = write_register(user_control, 0x04) && ok;
    delay(2);
    uint8_t status;
    ok = read_registers(interrupt_status, &status, 1) && ok;
    ok = write_register(user_control, 0x40) && ok;
    if (!ok)
    {
        stop_recording("FIFO_START_FAILED");
        return;
    }
    sample_count = 0;
    maximum_fifo = 0;
    data_hash = 2166136261u;
    for (uint8_t axis = 0; axis < 7; axis++)
    {
        sums[axis] = 0;
    }
    Serial.printf("BEGIN,20000\n");
    Serial.flush();
    started_us = micros();
    // Packet order: accel XYZ, temperature, gyro XYZ; seven signed 16-bit words.
    recording = write_register(fifo_enable, 0xF8);
    if (!recording)
    {
        stop_recording("FIFO_ENABLE_FAILED");
    }
}

static void read_one_sample(void)
{
    uint8_t status;
    uint8_t count_bytes[2];
    if (!read_registers(interrupt_status, &status, 1) || !read_registers(fifo_count, count_bytes, 2))
    {
        stop_recording("I2C_READ_FAILED");
        return;
    }
    uint16_t available = ((uint16_t)count_bytes[0] << 8) | count_bytes[1];
    maximum_fifo = max(maximum_fifo, available);
    if ((status & 0x10) || available >= 980)
    {
        // Stop rather than silently lose or misalign samples in the 1024-byte FIFO.
        stop_recording("FIFO_OVERFLOW_OR_BACKLOG");
        return;
    }
    if (available < 14)
    {
        delayMicroseconds(50);
        return;
    }
    uint8_t bytes[14];
    if (!read_registers(fifo_data, bytes, sizeof(bytes)))
    {
        stop_recording("FIFO_READ_FAILED");
        return;
    }
    int16_t values[7];
    for (uint8_t axis = 0; axis < 7; axis++)
    {
        values[axis] = (int16_t)(((uint16_t)bytes[2 * axis] << 8) | bytes[2 * axis + 1]);
        sums[axis] += values[axis];
    }
    for (uint8_t index = 0; index < sizeof(bytes); index++)
    {
        // FNV-1a checksum: the PC checks the full transmitted sample payload.
        data_hash = (data_hash ^ bytes[index]) * 16777619u;
    }
    Serial.printf("S,%lu,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)sample_count,
        values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
    sample_count++;
    if (sample_count == sample_target)
    {
        uint32_t elapsed_us = micros() - started_us;
        bool stopped = write_register(fifo_enable, 0);
        stopped = read_registers(interrupt_status, &status, 1) && stopped;
        stopped = write_register(user_control, 0) && stopped;
        recording = false;
        if (!stopped || (status & 0x10))
        {
            stop_recording("FINAL_FIFO_CHECK_FAILED");
            return;
        }
        for (uint8_t axis = 0; axis < 7; axis++)
        {
            Serial.printf("MEAN_COUNTS,%u,%.9f\n", axis, (double)sums[axis] / sample_target);
        }
        Serial.printf("END,%lu,%lu,%u,%08lX\n", (unsigned long)sample_count,
            (unsigned long)elapsed_us, maximum_fifo, (unsigned long)data_hash);
    }
}

void setup(void)
{
    Serial.setTxBufferSize(4096);
    Serial.begin(serial_baud);
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    // 400 kHz is needed to drain complete FIFO packets at 1000 Hz.
    sensor_ready = Wire.begin(9, 8, 400000);
    Wire.setTimeOut(10);
    sensor_ready = sensor_ready && mpu.begin(0x68, &Wire) && sensor.begin();
    if (sensor_ready)
    {
        Wire.setClock(400000);
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        mpu.setSampleRateDivisor(7);
        mpu.setClock(MPU6050_PLL_GYROX);
        sensor_ready = (Wire.getClock() == 400000) && write_register(0x38, 0x10);
        sensor_ready = write_register(fifo_enable, 0) && sensor_ready;
        sensor_ready = write_register(user_control, 0) && sensor_ready;
        sensor_ready = report_configuration() && sensor_ready;
    }
    if (!sensor_ready)
    {
        Serial.printf("ERROR,INITIALIZATION_OR_CONFIG_FAILED\n");
    }
}

void loop(void)
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == 'x')
        {
            stop_recording("USER_ABORT");
        }
        if (command == '?' && !recording)
        {
            Serial.printf("QUERY_REPLY\n");
            if (sensor_ready)
            {
                sensor_ready = report_configuration();
            }
            if (!sensor_ready)
            {
                Serial.printf("ERROR,SENSOR_NOT_READY\n");
            }
        }
        if (command == 's' && !recording && sensor_ready)
        {
            start_recording();
        }
    }
    if (recording)
    {
        if ((uint32_t)(micros() - started_us) > 25000000u)
        {
            stop_recording("RECORDING_TIMEOUT");
            return;
        }
        read_one_sample();
    }
    else
    {
        delay(1);
    }
}
