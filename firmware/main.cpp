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
static const float radians_to_degrees = 57.29577951308232f;
static const float degrees_to_radians = 0.01745329251994329577f;
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
        int32_t decrement = lroundf(gyro_bias[axis] * radians_to_degrees * 32.8f);
        int32_t target = (int32_t)gyro_baseline[axis] - decrement;
        if (target < -32768 || target > 32767)
        {
            return false;
        }
        gyro_applied_offset[axis] = (int16_t)target;
        gyro_remaining_bias[axis] = gyro_bias[axis] - decrement * (degrees_to_radians / 32.8f);
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

static const uint32_t control_rate_hz = 1000;
static const uint32_t capture_samples = control_rate_hz * 20;
static const uint32_t diagnostic_divider = control_rate_hz / 10;
bool recording = false;
uint32_t sequence = 0;
uint32_t capture_started_us = 0;
uint32_t previous_service_us = 0;
uint32_t maximum_service_gap_us = 0;
uint32_t maximum_processing_us = 0;
uint32_t payload_hash = 2166136261u;
uint16_t maximum_fifo_bytes = 0;
uint32_t maximum_handler_us = 0;
uint32_t maximum_i2c_us = 0;
uint32_t maximum_logging_us = 0;
uint32_t maximum_offset_us = 0;
uint32_t handler_overruns = 0;
// Observable latest state is updated for EVERY FIFO sample, not only diagnostics.
volatile float latest_drone_accel[3] = { 0 };
volatile float latest_sensor_gyro[3] = { 0 };
volatile float latest_roll_deg = 0;
volatile float latest_pitch_deg = 0;

static bool write_register(uint8_t address, uint8_t value)
{
    Adafruit_BusIO_Register reg(&sensor, address);
    return reg.write(value);
}

static bool read_registers(uint8_t address, uint8_t *bytes, uint8_t length)
{
    return sensor.write_then_read(&address, 1, bytes, length);
}

static void stop_acquisition(const char *reason)
{
    bool stopped = write_register(0x23, 0);
    stopped = write_register(0x6A, 0) && stopped;
    recording = false;
    sensor_ready = false;
    Serial.printf("ERROR,%s,stopped=%u\n", reason, (unsigned)stopped);
}

static bool reset_fifo(void)
{
    bool ok = write_register(0x23, 0);
    ok = write_register(0x6A, 0) && ok;
    ok = write_register(0x6A, 0x04) && ok;
    delay(2);
    uint8_t status;
    ok = read_registers(0x3A, &status, 1) && ok;
    ok = write_register(0x6A, 0x40) && ok;
    return ok;
}

static bool enable_fifo(void)
{
    // Packet order: accel XYZ, temperature, gyro XYZ (14 bytes).
    if (!write_register(0x23, 0xF8))
    {
        return false;
    }
    uint8_t sources;
    uint8_t control;
    return read_registers(0x23, &sources, 1) && read_registers(0x6A, &control, 1)
        && sources == 0xF8 && (control & 0x40) != 0;
}

static void report(void)
{
    if (!sensor_ready || !verify_offsets())
    {
        sensor_ready = false;
        Serial.printf("ERROR,SENSOR_OR_OFFSET_CHECK_FAILED\n");
        return;
    }
    Serial.printf("HELLO,FIFO1000_V1,%s\n", BENCH_SOURCE_SHA);
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
    Serial.printf("RATE,hz=1000,packet_bytes=14,clock=mpu_fifo,diagnostic_divider=100\n");
    Serial.printf("READY,FIFO1000_V1\n");
}

void setup(void)
{
    Serial.setTxBufferSize(4096);
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
    sensor_ready = sensor_ready && write_register(0x38, 0x10);
    report();
    Serial.flush();
    sensor_ready = sensor_ready && reset_fifo() && enable_fifo();
    if (!sensor_ready)
    {
        stop_acquisition("INITIALIZATION_FAILED");
    }
}

static void start_capture(void)
{
    if (!verify_offsets() || !reset_fifo())
    {
        stop_acquisition("CAPTURE_START_FAILED");
        return;
    }
    sequence = 0;
    previous_service_us = 0;
    maximum_service_gap_us = 0;
    maximum_processing_us = 0;
    maximum_handler_us = 0;
    maximum_i2c_us = 0;
    maximum_logging_us = 0;
    maximum_offset_us = 0;
    handler_overruns = 0;
    maximum_fifo_bytes = 0;
    payload_hash = 2166136261u;
    Serial.printf("BEGIN,%lu\n", (unsigned long)capture_samples);
    Serial.flush();
    capture_started_us = micros();
    recording = enable_fifo();
    if (!recording)
    {
        stop_acquisition("FIFO_ENABLE_FAILED");
    }
}

static bool verify_next_offset(void)
{
    static uint8_t index = 0;
    uint8_t axis = index % 3;
    uint8_t address = index < 3 ? 0x06 + 2 * axis : 0x13 + 2 * axis;
    int16_t expected = index < 3 ? applied_offset[axis] : gyro_applied_offset[axis];
    Adafruit_BusIO_Register reg(&sensor, address, 2, MSBFIRST);
    uint16_t observed;
    bool ok = reg.read(&observed) && (int16_t)observed == expected;
    index = (index + 1) % 6;
    return ok;
}

static void process_one_sample(void)
{
    uint32_t handler_started_us = micros();
    uint8_t status;
    uint8_t count_bytes[2];
    if (!read_registers(0x72, count_bytes, 2))
    {
        stop_acquisition("FIFO_STATUS_READ_FAILED");
        return;
    }
    uint16_t available = ((uint16_t)count_bytes[0] << 8) | count_bytes[1];
    maximum_fifo_bytes = max(maximum_fifo_bytes, available);
    // Occupancy guard stops well before 1024-byte overflow. Check status at boundaries.
    if (available >= 980)
    {
        stop_acquisition("FIFO_OVERFLOW_OR_BACKLOG");
        return;
    }
    if (available < 14)
    {
        delayMicroseconds(30);
        return;
    }
    uint8_t bytes[14];
    if (!read_registers(0x74, bytes, sizeof(bytes)))
    {
        stop_acquisition("FIFO_PACKET_READ_FAILED");
        return;
    }
    uint32_t service_us = micros();
    maximum_i2c_us = max(maximum_i2c_us, service_us - handler_started_us);
    if (previous_service_us != 0)
    {
        maximum_service_gap_us = max(maximum_service_gap_us, service_us - previous_service_us);
    }
    previous_service_us = service_us;
    int16_t counts[7];
    float drone[3];
    float gyro[3];
    for (uint8_t axis = 0; axis < 7; axis++)
    {
        counts[axis] = (int16_t)(((uint16_t)bytes[2 * axis] << 8) | bytes[2 * axis + 1]);
    }
    correct_acceleration(counts, drone);
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        gyro[axis] = counts[axis + 4] * (degrees_to_radians / 65.5f) - gyro_remaining_bias[axis];
    }
    float yz = hypotf(drone[1], drone[2]);
    float roll = yz > 0.001f ? atan2f(drone[1], drone[2]) * radians_to_degrees : NAN;
    float pitch = yz > 0.001f ? atan2f(-drone[0], yz) * radians_to_degrees : NAN;
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        latest_drone_accel[axis] = drone[axis];
        latest_sensor_gyro[axis] = gyro[axis];
    }
    latest_roll_deg = roll;
    latest_pitch_deg = pitch;
    maximum_processing_us = max(maximum_processing_us, (uint32_t)(micros() - service_us));
    uint32_t logging_started_us = micros();
    if (recording)
    {
        for (uint8_t index = 0; index < sizeof(bytes); index++)
        {
            payload_hash = (payload_hash ^ bytes[index]) * 16777619u;
        }
        Serial.printf("S,%lu,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)sequence,
            counts[0], counts[1], counts[2], counts[3], counts[4], counts[5], counts[6]);
    }
    // Slow diagnostics are separate from the per-sample calculations.
    if (sequence % diagnostic_divider == 0)
    {
        Serial.printf("D,%lu,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.3f,%.3f\n",
            (unsigned long)sequence, drone[0], drone[1], drone[2], gyro[0], gyro[1], gyro[2], roll, pitch);
    }
    maximum_logging_us = max(maximum_logging_us, (uint32_t)(micros() - logging_started_us));
    sequence++;
    if (sequence % diagnostic_divider == 0)
    {
        uint32_t offset_started_us = micros();
        if (!verify_next_offset())
        {
            stop_acquisition("LIVE_OFFSETS_CHANGED");
            return;
        }
        maximum_offset_us = max(maximum_offset_us, (uint32_t)(micros() - offset_started_us));
    }
    uint32_t handler_us = micros() - handler_started_us;
    maximum_handler_us = max(maximum_handler_us, handler_us);
    if (handler_us > 1000)
    {
        handler_overruns++;
    }
    if (recording && sequence == capture_samples)
    {
        uint32_t elapsed_us = micros() - capture_started_us;
        bool ok = write_register(0x23, 0);
        ok = read_registers(0x3A, &status, 1) && ok;
        ok = verify_offsets() && ok;
        if (!ok || (status & 0x10))
        {
            stop_acquisition("FINAL_FIFO_OR_OFFSET_CHECK_FAILED");
            return;
        }
        recording = false;
        Serial.printf("PROFILE,%lu,%lu,%lu,%lu\n", (unsigned long)maximum_i2c_us,
            (unsigned long)maximum_logging_us, (unsigned long)maximum_offset_us, (unsigned long)handler_overruns);
        Serial.printf("END,%lu,%lu,%u,%lu,%lu,%lu,%08lX\n", (unsigned long)sequence,
            (unsigned long)elapsed_us, (unsigned)maximum_fifo_bytes, (unsigned long)maximum_service_gap_us,
            (unsigned long)maximum_processing_us, (unsigned long)maximum_handler_us, (unsigned long)payload_hash);
        Serial.flush();
        sensor_ready = reset_fifo() && enable_fifo();
        if (!sensor_ready)
        {
            stop_acquisition("FIFO_RESTART_FAILED");
        }
    }
}

void loop(void)
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == 'x')
        {
            stop_acquisition("USER_ABORT");
        }
        if (command == '?' && !recording && sensor_ready)
        {
            sensor_ready = reset_fifo();
            Serial.printf("QUERY_REPLY\n");
            report();
            Serial.flush();
            sensor_ready = sensor_ready && reset_fifo() && enable_fifo();
        }
        if (command == 's' && !recording && sensor_ready)
        {
            start_capture();
        }
    }
    if (recording && (uint32_t)(micros() - capture_started_us) > 25000000u)
    {
        stop_acquisition("CAPTURE_TIMEOUT");
    }
    if (sensor_ready)
    {
        process_one_sample();
    }
    else
    {
        delay(1);
    }
}
