#include <Arduino.h>
#include <esp32-hal-rgb-led.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_I2CDevice.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "unknown"
#endif

// Measurement harness only. No PWM/Servo/ESC, PID control or calibration writes.
Adafruit_MPU6050 mpu;
Adafruit_I2CDevice imu(0x68, &Wire);
static bool ready = false;
static const uint32_t window_us = 4000000;
static const uint16_t capacity = 4200;

struct Sample
{
    uint32_t service_start;
    uint32_t ready_observed;
    uint32_t read_start;
    uint32_t read_end;
    uint16_t fifo_bytes;
    int16_t raw[7];
};
struct RotationSample
{
    uint16_t read_gap_us;
    int16_t gyro[3];
};
static const uint16_t rotation_capacity = 20200;
static const uint32_t rotation_window_us = 20000000;
static union
{
    Sample samples[capacity];
    RotationSample rotations[rotation_capacity];
} records;
static Sample (&samples)[capacity] = records.samples;

struct Trial
{
    char mode;
    uint16_t count;
    uint32_t elapsed;
    uint32_t injected;
    uint16_t tail;
    uint32_t checksum;
    const char *failure;
};
static QueueHandle_t requests;
static QueueHandle_t completed;
static SemaphoreHandle_t buffer_free;
static EventGroupHandle_t events;
static const EventBits_t abort_bit = 1;
static uint8_t identity = 0;
static int acquisition_core = -1;
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;

// DevKitC-1 addressable RGB LED. The separate power LED cannot be controlled.
// Override at build time for boards whose RGB LED is wired to another GPIO.
#ifndef XBOX_LED_GPIO
#define XBOX_LED_GPIO 48
#endif

static void connection_led(uint32_t now, bool connected, bool fresh)
{
    static int previous = -1;
    // Search: 80 ms per edge. Connected but waiting/stale: 500 ms per edge.
    bool on = fresh || ((now / (connected ? 500 : 80)) % 2 == 0);
    if ((int)on != previous)
    {
        neopixelWrite(XBOX_LED_GPIO, 0, on ? 24 : 0, 0);
        previous = on;
    }
}

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    return imu.write(bytes, 2);
}

static bool read_registers(uint8_t reg, uint8_t *bytes, uint8_t length)
{
    return imu.write_then_read(&reg, 1, bytes, length);
}

// Cached identity only: the slow core must never touch the sensor I2C bus.
static void report(void)
{
    Serial.printf("ACQ_MENU_V1,%s,WHO=%u,READY=%u,WINDOW_US=%lu\n",
        BENCH_SOURCE_SHA, identity, ready, (unsigned long)window_us);
    Serial.printf("CORE,acquisition=%d,slow=%d,heap=%u,min_heap=%u,PID=NOT_IMPLEMENTED\n",
        acquisition_core, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap());
    Serial.printf("MENU,Menu=SHOW_OR_NEXT,Dpad=SELECT,A=RUN_SELECTED,B=CANCEL,Y=INFO,MOTORS=DISABLED\n");
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

// Rotation-only FIFO capture: retain gyro, not the full timing benchmark rows.
static void capture_rotation(void)
{
    Trial result = {};
    result.mode = 'r';
    result.checksum = 2166136261u;
    bool ok = ready && check_config();
    ok = ok && write_register(0x23, 0) && write_register(0x6A, 0x04);
    delay(2);
    ok = ok && write_register(0x6A, 0x40) && write_register(0x23, 0xF8);
    uint32_t started = micros();
    uint32_t previous = 0;
    while (ok && (uint32_t)(micros() - started) < rotation_window_us)
    {
        if (xEventGroupGetBits(events) & abort_bit)
        {
            result.failure = "USER_ABORT";
            break;
        }
        uint8_t status;
        uint8_t size[2];
        ok = read_registers(0x3A, &status, 1) && read_registers(0x72, size, 2);
        if (!ok)
        {
            break;
        }
        uint16_t queued = ((uint16_t)size[0] << 8) | size[1];
        if ((status & 0x10) || queued >= 1024)
        {
            result.failure = "FIFO_OVERFLOW";
            break;
        }
        if (queued < 14)
        {
            continue;
        }
        if (result.count >= rotation_capacity)
        {
            result.failure = "RECORD_BUFFER_FULL";
            break;
        }
        uint8_t bytes[14];
        ok = read_registers(0x74, bytes, 14);
        if (!ok)
        {
            break;
        }
        uint32_t now = micros() - started;
        if (now - previous > 65535)
        {
            result.failure = "READ_GAP_TOO_LONG";
            break;
        }
        RotationSample &row = records.rotations[result.count++];
        row.read_gap_us = now - previous;
        previous = now;
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            row.gyro[axis] = (int16_t)(((uint16_t)bytes[8 + 2 * axis] << 8) | bytes[9 + 2 * axis]);
        }
        for (uint8_t j = 8; j < 14; j++)
        {
            result.checksum = (result.checksum ^ bytes[j]) * 16777619u;
        }
    }
    result.elapsed = micros() - started;
    uint8_t tail[2] = {};
    ok = read_registers(0x72, tail, 2) && ok;
    result.tail = ((uint16_t)tail[0] << 8) | tail[1];
    ok = write_register(0x23, 0) && write_register(0x6A, 0) && ok;
    if (!ok && !result.failure)
    {
        result.failure = "I2C_OR_CONFIG_FAILURE";
    }
    xQueueSend(completed, &result, 0);
}

static void capture(char mode)
{
    if (mode == 'r')
    {
        capture_rotation();
        return;
    }
    bool fifo = mode == 'f' || mode == 'F';
    bool pauses = mode == 'D' || mode == 'F';
    if (!ready || !check_config())
    {
        Trial rejected = {};
        rejected.mode = mode;
        rejected.failure = "CONFIG_OR_SENSOR";
        xQueueSend(completed, &rejected, 0);
        return;
    }
    bool ok = write_register(0x23, 0) && write_register(0x6A, 0x04);
    delay(2);
    ok = write_register(0x6A, fifo ? 0x40 : 0) && ok;
    delay(20);
    uint8_t status = 0;
    ok = read_registers(0x3A, &status, 1) && ok;
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
        if (xEventGroupGetBits(events) & abort_bit)
        {
            failure = "USER_ABORT";
            break;
        }
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
    Trial result = {mode, count, elapsed, injected, tail, checksum,
        failure ? failure : (ok ? nullptr : "I2C_OR_CONFIG_FAILURE")};
    xQueueSend(completed, &result, 0);
}

static void request_test(char mode)
{
    if (uxSemaphoreGetCount(buffer_free) == 0 || uxQueueMessagesWaiting(requests))
    {
        Serial.printf("MENU,BUSY_NO_NEW_TEST\n");
        return;
    }
    xEventGroupClearBits(events, abort_bit);
    if (xQueueSend(requests, &mode, 0) == pdTRUE)
    {
        Serial.printf("START,%c\n", mode);
    }
}

// Core0 owns BLE, menu, Serial and chunked post-capture logging.
static void slow_worker(void *unused)
{
    const char modes[4] = {'d', 'f', 'D', 'F'};
    const char *names[4] = {"DIRECT_NORMAL", "FIFO_NORMAL", "DIRECT_PAUSES", "FIFO_PAUSES"};
    uint8_t selected = 0;
    bool released = false;
    uint8_t previous = 0;
    bool dumping = false;
    uint16_t dump_index = 0;
    Trial result = {};
    uint32_t last_status = 0;
    uint32_t last_input = 0;
    bool inputs_only = false;
    report();
    connection_led(millis(), false, false);
    Serial.printf("MENU,SELECTED=DIRECT_NORMAL,COMMAND=d,A=RUN,DpadLeftRight=SELECT,B=CANCEL\n");
    xbox.begin();
    // Request scan-response data as well as advertisements for discovery.
    NimBLEDevice::getScan()->setActiveScan(true);
    Serial.printf("XBOX_LED,GPIO=%u,SEARCH_EDGE_MS=80,WAIT_EDGE_MS=500,READY=STEADY_GREEN\n", XBOX_LED_GPIO);
    for (;;)
    {
        xbox.onLoop();
        uint32_t now = millis();
        bool connected = xbox.isConnected();
        bool fresh = connected && !xbox.isWaitingForFirstNotification()
            && (uint32_t)(now - xbox.getReceiveNotificationAt()) <= 300;
        bool input_ready = connected && !xbox.isWaitingForFirstNotification();
        connection_led(now, connected, input_ready);
        uint8_t buttons = 0;
        if (input_ready)
        {
            // Operator observed these two library fields exchanged on this controller.
            buttons = (xbox.xboxNotif.btnY ? 1 : 0)
                | (xbox.xboxNotif.btnA ? 2 : 0)
                | (xbox.xboxNotif.btnB ? 4 : 0)
                | (xbox.xboxNotif.btnStart ? 8 : 0)
                | (xbox.xboxNotif.btnDirRight ? 16 : 0)
                | (xbox.xboxNotif.btnDirLeft ? 32 : 0);
            if (buttons == 0)
            {
                released = true;
            }
        }
        else
        {
            released = false;
        }
        uint8_t pressed = released ? buttons & ~previous : 0;
        previous = buttons;
        if (pressed & (1 | 16 | 32))
        {
            if (pressed & 32)
            {
                selected = (selected + 3) % 4;
            }
            else
            {
                selected = (selected + 1) % 4;
            }
            Serial.printf("MENU,SELECTED=%s,COMMAND=%c\n", names[selected], modes[selected]);
        }
        if ((pressed & 2) && !inputs_only)
        {
            request_test(modes[selected]);
        }
        if ((pressed & 4) && !inputs_only)
        {
            xEventGroupSetBits(events, abort_bit);
            Serial.printf("MENU,CANCEL_REQUESTED\n");
        }
        if (pressed & 8)
        {
            report();
            Serial.printf("MENU,SELECTED=%s,COMMAND=%c,A=RUN\n", names[selected], modes[selected]);
        }
        if (Serial.available())
        {
            char command = Serial.read();
            if (command == 'i' || command == 't')
            {
                inputs_only = command == 'i';
                Serial.printf("INPUT_MODE,%s\n", inputs_only ? "MONITOR_ONLY_NO_TESTS" : "TEST_MENU");
            }
            if (command == '?')
            {
                report();
            }
            if (command == 'x')
            {
                xEventGroupSetBits(events, abort_bit);
            }
            if (!inputs_only && (command == 'd' || command == 'f' || command == 'D' || command == 'F' || command == 'r'))
            {
                request_test(command);
            }
        }
        if (!dumping && xQueueReceive(completed, &result, 0) == pdTRUE)
        {
            Serial.printf("%s,%c,%u,%lu,%lu,%u,%08lX\n", result.mode == 'r' ? "RDATA" : "DATA", result.mode, result.count,
                (unsigned long)result.elapsed, (unsigned long)result.injected,
                result.tail, (unsigned long)result.checksum);
            dump_index = 0;
            dumping = true;
        }
        if (dumping)
        {
            uint16_t limit = min((uint16_t)(dump_index + 32), result.count);
            while (dump_index < limit)
            {
                if (result.mode == 'r')
                {
                    const RotationSample &row = records.rotations[dump_index];
                    Serial.printf("G,%u,%u,%d,%d,%d\n", dump_index, row.read_gap_us,
                        row.gyro[0], row.gyro[1], row.gyro[2]);
                    dump_index++;
                    continue;
                }
                const Sample &row = samples[dump_index];
                Serial.printf("S,%u,%lu,%lu,%lu,%lu,%u", dump_index,
                    (unsigned long)row.service_start, (unsigned long)row.ready_observed,
                    (unsigned long)row.read_start, (unsigned long)row.read_end, row.fifo_bytes);
                for (uint8_t j = 0; j < 7; j++)
                {
                    Serial.printf(",%d", row.raw[j]);
                }
                Serial.printf("\n");
                dump_index++;
            }
            if (dump_index == result.count)
            {
                Serial.printf("DONE,%c,%s\n", result.mode, result.failure ? result.failure : "OK");
                dumping = false;
                xSemaphoreGive(buffer_free);
            }
        }
        if (inputs_only && input_ready && (uint32_t)(now - last_input) >= 200)
        {
            // Library field names are raw decoder labels, not yet physical acceptance.
            const auto &input = xbox.xboxNotif;
            Serial.printf("INPUT,rawA=%u,rawB=%u,rawX=%u,rawY=%u,rawStart=%u,LB=%u,RB=%u,UP=%u,DOWN=%u,LEFT=%u,RIGHT=%u,LX=%u,LY=%u,RX=%u,RY=%u,LT=%u,RT=%u\n",
                input.btnA, input.btnB, input.btnX, input.btnY, input.btnStart,
                input.btnLB, input.btnRB, input.btnDirUp, input.btnDirDown,
                input.btnDirLeft, input.btnDirRight, input.joyLHori, input.joyLVert,
                input.joyRHori, input.joyRVert, input.trigLT, input.trigRT);
            last_input = now;
        }
        if (!dumping && (uint32_t)(now - last_status) >= 5000)
        {
            Serial.printf("XBOX,CONNECTED=%u,RECENT_INPUT=%u,CORE=%d,HEAP=%u,MIN_HEAP=%u,SCANNING=%u\n",
                connected, fresh, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                NimBLEDevice::getScan()->isScanning());
            last_status = now;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
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
    read_registers(0x75, &identity, 1);
    acquisition_core = xPortGetCoreID();
    requests = xQueueCreate(1, sizeof(char));
    completed = xQueueCreate(1, sizeof(Trial));
    buffer_free = xSemaphoreCreateBinary();
    events = xEventGroupCreate();
    if (!requests || !completed || !buffer_free || !events)
    {
        ready = false;
        Serial.printf("ERROR,TASK_RESOURCES\n");
        return;
    }
    xSemaphoreGive(buffer_free);
    if (xTaskCreatePinnedToCore(slow_worker, "xbox_slow", 6144, nullptr, 1, nullptr, 0) != pdPASS)
    {
        ready = false;
        Serial.printf("ERROR,SLOW_TASK_START\n");
    }
}

void loop(void)
{
    char command;
    if (ready && requests && xQueueReceive(requests, &command, 0) == pdTRUE)
    {
        if (xSemaphoreTake(buffer_free, 0) == pdTRUE)
        {
            capture(command);
        }
    }
    delay(1);
}
