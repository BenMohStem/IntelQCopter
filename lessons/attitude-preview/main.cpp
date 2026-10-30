#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <math.h>
#include <Adafruit_AHRS_Mahony.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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
// Provisional mounting rotation only; physical gyro-axis validation is pending.
static const float gyro_to_drone[3][3] = {
    {0.953336617f, -0.085320686f, -0.289602617f},
    {0.089416989f, 0.995993856f, 0.000917157f},
    {0.288364175f, -0.026769754f, 0.957146532f},
};
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
volatile float latest_drone_gyro[3] = { 0 };
volatile float latest_roll_deg = 0;
volatile float latest_pitch_deg = 0;

// Diagnostic roll-rate PID, zero setpoint. No actuator consumes this value.
// Illustrative normalized-output gains, NOT physical flight settings.
struct PidPreview
{
    float p;
    float i;
    float d;
    float output;
    float previous_rate;
    bool initialized;
};
static PidPreview roll_pid = {};

static void update_pid_preview(PidPreview *pid, float rate)
{
    const float dt = 0.001f;
    const float limit = 0.2f;
    const float error = -rate;
    pid->p = 0.1f * error;
    float slope = pid->initialized ? (rate - pid->previous_rate) / dt : 0.0f;
    float derivative = -0.001f * slope;
    pid->d += dt / (0.02f + dt) * (derivative - pid->d);
    float candidate_i = fmaxf(-limit, fminf(limit, pid->i + 0.05f * error * dt));
    float sum = pid->p + candidate_i + pid->d;
    // Keep integration when it helps leave saturation; reject outward growth.
    if (!((sum > limit && error > 0.0f) || (sum < -limit && error < 0.0f)))
    {
        pid->i = candidate_i;
    }
    pid->output = fmaxf(-limit, fminf(limit, pid->p + pid->i + pid->d));
    pid->previous_rate = rate;
    pid->initialized = true;
}

// Core 1 owns sensor state. Core 0 owns slow serial formatting/transmission.
struct TelemetryPacket
{
    uint32_t sequence;
    int16_t counts[7];
    float acceleration[3];
    float gyro[3];
    float body_gyro[3];
    float roll;
    float pitch;
    float quaternion[4];
    float pid_terms[4];
    bool capture;
    bool diagnostic;
    bool barrier;
};
static QueueHandle_t telemetry_queue = nullptr;
static SemaphoreHandle_t telemetry_drained = nullptr;
static uint32_t maximum_queue_depth = 0;

// Preview setpoints only: there is no PID, PWM, ESC or motor output here.
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;
struct TargetPacket
{
    uint32_t timestamp;
    bool valid;
    float roll;
    float pitch;
    float yaw;
    float vertical;
};
static QueueHandle_t target_queue;


static float stick_command(uint16_t raw)
{
    const float deadband = 0.08f;
    float value = ((float)raw - 32767.5f) / 32767.5f;
    float magnitude = fabsf(value);
    if (magnitude <= deadband)
    {
        return 0.0f;
    }
    float shaped = (magnitude - deadband) / (1.0f - deadband);
    shaped = 0.6f * shaped + 0.4f * shaped * shaped * shaped;
    return value < 0.0f ? -shaped : shaped;
}

static void report_targets(void)
{
    static bool neutral_seen = false;
    static uint32_t previous_report = 0;
    uint32_t now = millis();
    bool recent = xbox.isConnected() && !xbox.isWaitingForFirstNotification()
        && (uint32_t)(now - xbox.getReceiveNotificationAt()) <= 300;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float vertical = 0.0f;
    if (recent)
    {
        roll = stick_command(xbox.xboxNotif.joyRHori);
        pitch = stick_command(xbox.xboxNotif.joyRVert);
        yaw = -stick_command(xbox.xboxNotif.joyLHori);
        vertical = -stick_command(xbox.xboxNotif.joyLVert);
        if (roll == 0.0f && pitch == 0.0f && yaw == 0.0f && vertical == 0.0f)
        {
            neutral_seen = true;
        }
    }
    else
    {
        neutral_seen = false;
    }
    bool valid = recent && neutral_seen;
    if ((uint32_t)(now - previous_report) >= 100)
    {
        TargetPacket target = {now, valid, valid ? 20.0f * roll : 0.0f,
            valid ? 20.0f * pitch : 0.0f, valid ? 30.0f * yaw : 0.0f,
            valid ? vertical : 0.0f};
        xQueueOverwrite(target_queue, &target);
        previous_report = now;
    }
}

static void xbox_worker(void *)
{
    xbox.begin();
    NimBLEDevice::getScan()->setActiveScan(true);
    for (;;)
    {
        xbox.onLoop();
        report_targets();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void telemetry_worker(void *)
{
    TelemetryPacket packet;
    TargetPacket target = {};
    uint32_t previous_target_report = 0;
    for (;;)
    {
        xQueueReceive(target_queue, &target, 0);
        uint32_t now = millis();
        if ((uint32_t)(now - previous_target_report) >= 100)
        {
            bool valid = target.valid && (uint32_t)(now - target.timestamp) <= 300;
            Serial.printf("T,%u,%.3f,%.3f,%.3f,%.3f\n", valid,
                valid ? target.roll : 0.0f, valid ? target.pitch : 0.0f,
                valid ? target.yaw : 0.0f, valid ? target.vertical : 0.0f);
            previous_target_report = now;
        }
        if (xQueueReceive(telemetry_queue, &packet, pdMS_TO_TICKS(5)) != pdTRUE)
        {
            continue;
        }
        if (packet.barrier)
        {
            xSemaphoreGive(telemetry_drained);
            continue;
        }
        if (packet.capture)
        {
            Serial.printf("S,%lu,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)packet.sequence,
                packet.counts[0], packet.counts[1], packet.counts[2], packet.counts[3],
                packet.counts[4], packet.counts[5], packet.counts[6]);
        }
        if (packet.diagnostic)
        {
            Serial.printf("C,%lu,%.6f,%.6f,%.6f,%.6f\n", (unsigned long)packet.sequence,
                packet.pid_terms[0], packet.pid_terms[1], packet.pid_terms[2], packet.pid_terms[3]);
            Serial.printf("Q,%lu,%.8f,%.8f,%.8f,%.8f\n", (unsigned long)packet.sequence,
                packet.quaternion[0], packet.quaternion[1],
                packet.quaternion[2], packet.quaternion[3]);
            Serial.printf("D,%lu,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.3f,%.3f,%.7f,%.7f,%.7f\n",
                (unsigned long)packet.sequence, packet.acceleration[0], packet.acceleration[1],
                packet.acceleration[2], packet.gyro[0], packet.gyro[1], packet.gyro[2], packet.roll, packet.pitch,
                packet.body_gyro[0], packet.body_gyro[1], packet.body_gyro[2]);
        }
    }
}

static bool start_telemetry(void)
{
    telemetry_queue = xQueueCreate(128, sizeof(TelemetryPacket));
    telemetry_drained = xSemaphoreCreateBinary();
    target_queue = xQueueCreate(1, sizeof(TargetPacket));
    return telemetry_queue && telemetry_drained && target_queue
        && xTaskCreatePinnedToCore(xbox_worker, "xbox", 6144, nullptr, 1, nullptr, 0) == pdPASS
        && xTaskCreatePinnedToCore(telemetry_worker, "telemetry", 4096, nullptr, 1, nullptr, 0) == pdPASS;
}

static bool flush_telemetry(void)
{
    TelemetryPacket barrier = {};
    barrier.barrier = true;
    return xQueueSend(telemetry_queue, &barrier, pdMS_TO_TICKS(2000)) == pdTRUE
        && xSemaphoreTake(telemetry_drained, pdMS_TO_TICKS(2000)) == pdTRUE;
}

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
    Serial.printf("HELLO,PID_DEMO_V1,%s\n", BENCH_SOURCE_SHA);
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
    for (uint8_t row = 0; row < 3; row++)
    {
        Serial.printf("GYRO_ROT,%u,%.9f,%.9f,%.9f\n", (unsigned)row,
            gyro_to_drone[row][0], gyro_to_drone[row][1], gyro_to_drone[row][2]);
    }
    Serial.printf("GYRO_SCALE,65.5,counts_per_degree_per_second,frame=sensor,correction=mpu_plus_remainder\n");
    Serial.printf("LOG,acquisition_core=%d,serial_core=0,queue_capacity=128\n", xPortGetCoreID());
    Serial.printf("RATE,hz=1000,packet_bytes=14,clock=mpu_fifo,diagnostic_divider=100\n");
    Serial.printf("READY,PID_DEMO_V1\n");
}

void setup(void)
{
    Serial.setTxBufferSize(4096);
    Serial.begin(921600);
    // Keep legacy and current carrier ESC pins inactive; no motor driver.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);
    pinMode(4, INPUT_PULLDOWN);
    pinMode(21, INPUT_PULLDOWN);

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
    sensor_ready = sensor_ready && start_telemetry() && xPortGetCoreID() == 1;
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
    if (!verify_offsets() || !reset_fifo() || !flush_telemetry())
    {
        stop_acquisition("CAPTURE_START_FAILED");
        return;
    }
    sequence = 0;
    previous_service_us = 0;
    maximum_service_gap_us = 0;
    maximum_processing_us = 0;
    maximum_handler_us = 0;
    maximum_queue_depth = 0;
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

// Quaternion complementary preview. Gains are provisional, not flight gains.
static Adafruit_Mahony attitude;
static bool attitude_started = false;
static float attitude_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

static bool update_attitude(const float acceleration[3], const float rate[3])
{
    float magnitude = sqrtf(acceleration[0] * acceleration[0]
        + acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2]);
    bool gravity_usable = isfinite(magnitude) && fabsf(magnitude - 9.80665f) < 1.5f;
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        if (!isfinite(rate[axis]) || !isfinite(acceleration[axis]))
        {
            return false;
        }
    }
    if (!attitude_started)
    {
        if (!gravity_usable)
        {
            return false;
        }
        float roll = atan2f(acceleration[1], acceleration[2]);
        float pitch = atan2f(-acceleration[0], hypotf(acceleration[1], acceleration[2]));
        attitude.begin(1000.0f);
        attitude.setKp(0.5f);
        attitude.setKi(0.0f);
        attitude.setQuaternion(cosf(roll / 2) * cosf(pitch / 2),
            sinf(roll / 2) * cosf(pitch / 2), cosf(roll / 2) * sinf(pitch / 2),
            -sinf(roll / 2) * sinf(pitch / 2));
        attitude_started = true;
    }
    // FIFO sample period, not host receive timing. Library expects gyro deg/s.
    attitude.updateIMU(rate[0] * radians_to_degrees,
        rate[1] * radians_to_degrees, rate[2] * radians_to_degrees,
        gravity_usable ? acceleration[0] : 0.0f,
        gravity_usable ? acceleration[1] : 0.0f,
        gravity_usable ? acceleration[2] : 0.0f, 0.001f);
    attitude.getQuaternion(&attitude_q[0], &attitude_q[1], &attitude_q[2], &attitude_q[3]);
    float norm = sqrtf(attitude_q[0] * attitude_q[0] + attitude_q[1] * attitude_q[1]
        + attitude_q[2] * attitude_q[2] + attitude_q[3] * attitude_q[3]);
    if (!isfinite(norm) || norm < 0.5f || norm > 1.5f)
    {
        return false;
    }
    for (uint8_t component = 0; component < 4; component++)
    {
        attitude_q[component] /= norm;
    }
    attitude.setQuaternion(attitude_q[0], attitude_q[1], attitude_q[2], attitude_q[3]);
    return true;
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
    float body_gyro[3] = { 0 };
    for (uint8_t row = 0; row < 3; row++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            body_gyro[row] += gyro_to_drone[row][axis] * gyro[axis];
        }
        latest_drone_gyro[row] = body_gyro[row];
    }
    if (!update_attitude(drone, body_gyro))
    {
        stop_acquisition("ATTITUDE_INVALID_OR_STARTUP_ACCELERATION");
        return;
    }
    const float w = attitude_q[0];
    const float x = attitude_q[1];
    const float y = attitude_q[2];
    const float z = attitude_q[3];
    float roll = atan2f(2.0f * (w * x + y * z),
        1.0f - 2.0f * (x * x + y * y)) * radians_to_degrees;
    float sine_pitch = 2.0f * (w * y - z * x);
    sine_pitch = fmaxf(-1.0f, fminf(1.0f, sine_pitch));
    float pitch = asinf(sine_pitch) * radians_to_degrees;
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        latest_drone_accel[axis] = drone[axis];
        latest_sensor_gyro[axis] = gyro[axis];
    }
    update_pid_preview(&roll_pid, body_gyro[0]);
    latest_roll_deg = roll;
    latest_pitch_deg = pitch;
    maximum_processing_us = max(maximum_processing_us, (uint32_t)(micros() - service_us));
    uint32_t logging_started_us = micros();
    bool diagnostic = sequence % diagnostic_divider == 0;
    if (recording)
    {
        for (uint8_t index = 0; index < sizeof(bytes); index++)
        {
            payload_hash = (payload_hash ^ bytes[index]) * 16777619u;
        }
    }
    if (recording || diagnostic)
    {
        TelemetryPacket packet = {};
        packet.sequence = sequence;
        packet.capture = recording;
        packet.diagnostic = diagnostic;
        for (uint8_t axis = 0; axis < 7; axis++)
        {
            packet.counts[axis] = counts[axis];
        }
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            packet.acceleration[axis] = drone[axis];
            packet.gyro[axis] = gyro[axis];
            packet.body_gyro[axis] = body_gyro[axis];
        }
        for (uint8_t component = 0; component < 4; component++)
        {
            packet.quaternion[component] = attitude_q[component];
        }
        packet.pid_terms[0] = roll_pid.p;
        packet.pid_terms[1] = roll_pid.i;
        packet.pid_terms[2] = roll_pid.d;
        packet.pid_terms[3] = roll_pid.output;
        packet.roll = roll;
        packet.pitch = pitch;
        if (xQueueSend(telemetry_queue, &packet, 0) != pdTRUE)
        {
            stop_acquisition("TELEMETRY_QUEUE_FULL");
            return;
        }
        maximum_queue_depth = max(maximum_queue_depth, (uint32_t)uxQueueMessagesWaiting(telemetry_queue));
    }
    maximum_logging_us = max(maximum_logging_us, (uint32_t)(micros() - logging_started_us));
    sequence++;
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
        if (!flush_telemetry())
        {
            stop_acquisition("TELEMETRY_DRAIN_FAILED");
            return;
        }
        Serial.flush();
        Serial.printf("PROFILE,%lu,%lu,%lu,%lu,%lu\n", (unsigned long)maximum_i2c_us,
            (unsigned long)maximum_logging_us, (unsigned long)maximum_offset_us, (unsigned long)handler_overruns, (unsigned long)maximum_queue_depth);
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
            sensor_ready = reset_fifo() && flush_telemetry();
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
