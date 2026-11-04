from pathlib import Path
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/xbox-test-menu';p.mkdir(parents=True,exist_ok=True)
s=(r/'lessons/imu-acquisition-benchmark/main.cpp').read_text(encoding='utf-8')
s=s.replace('#include <Adafruit_I2CDevice.h>', '''#include <Adafruit_I2CDevice.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>''')
s=s.replace('No PWM/Servo/ESC/BLE, control or calibration writes.', 'No PWM/Servo/ESC, PID control or calibration writes.')
s=s.replace('capacity = 5000','capacity = 4200')
at=s.index('static bool write_register')
s=s[:at]+'''struct Trial
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

''' + s[at:]
start=s.index('static void report(void)');end=s.index('static bool check_config',start)
s=s[:start]+'''// Cached identity only: the slow core must never touch the sensor I2C bus.
static void report(void)
{
    Serial.printf("ACQ_MENU_V1,%s,WHO=%u,READY=%u,WINDOW_US=%lu\\n",
        BENCH_SOURCE_SHA, identity, ready, (unsigned long)window_us);
    Serial.printf("CORE,acquisition=%d,slow=%d,heap=%u,min_heap=%u,PID=NOT_IMPLEMENTED\\n",
        acquisition_core, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap());
    Serial.printf("MENU,Menu=OPEN_OR_NEXT,A=RUN,B=CANCEL,Y=INFO,MOTORS=DISABLED\\n");
}

''' +s[end:]
s=s.replace('''        Serial.printf("ERROR,CONFIG_OR_SENSOR\\n");
        return;''','''        Trial rejected = {};
        rejected.mode = mode;
        rejected.failure = "CONFIG_OR_SENSOR";
        xQueueSend(completed, &rejected, 0);
        return;''')
s=s.replace('''    Serial.printf("START,%c\\n", mode);
    Serial.flush();
''','')
s=s.replace('''    {
        if (pauses &&''','''    {
        if (xEventGroupGetBits(events) & abort_bit)
        {
            failure = "USER_ABORT";
            break;
        }
        if (pauses &&''',1)
start=s.index('    Serial.printf("DATA,%c,');end=s.index('\nvoid setup(void)',start)
s=s[:start]+'''    Trial result = {mode, count, elapsed, injected, tail, checksum,
        failure ? failure : (ok ? nullptr : "I2C_OR_CONFIG_FAILURE")};
    xQueueSend(completed, &result, 0);
}

static void request_test(char mode)
{
    if (uxSemaphoreGetCount(buffer_free) == 0 || uxQueueMessagesWaiting(requests))
    {
        Serial.printf("MENU,BUSY_NO_NEW_TEST\\n");
        return;
    }
    xEventGroupClearBits(events, abort_bit);
    if (xQueueSend(requests, &mode, 0) == pdTRUE)
    {
        Serial.printf("START,%c\\n", mode);
    }
}

// Core0 owns BLE, menu, Serial and chunked post-capture logging.
static void slow_worker(void *unused)
{
    const char modes[4] = {'d', 'f', 'D', 'F'};
    const char *names[4] = {"DIRECT_NORMAL", "FIFO_NORMAL", "DIRECT_PAUSES", "FIFO_PAUSES"};
    uint8_t selected = 0;
    bool menu_open = false;
    bool released = false;
    uint8_t previous = 0;
    bool dumping = false;
    uint16_t dump_index = 0;
    Trial result = {};
    uint32_t last_status = 0;
    report();
    xbox.begin();
    for (;;)
    {
        xbox.onLoop();
        uint32_t now = millis();
        bool connected = xbox.isConnected();
        bool fresh = connected && !xbox.isWaitingForFirstNotification()
            && (uint32_t)(now - xbox.getReceiveNotificationAt()) <= 300;
        uint8_t buttons = 0;
        if (fresh)
        {
            buttons = (xbox.xboxNotif.btnStart ? 1 : 0)
                | (xbox.xboxNotif.btnA ? 2 : 0)
                | (xbox.xboxNotif.btnB ? 4 : 0)
                | (xbox.xboxNotif.btnY ? 8 : 0);
            if (buttons == 0)
            {
                released = true;
            }
        }
        else
        {
            released = false;
            menu_open = false;
        }
        uint8_t pressed = released ? buttons & ~previous : 0;
        previous = buttons;
        if (pressed & 1)
        {
            if (menu_open)
            {
                selected = (selected + 1) % 4;
            }
            menu_open = true;
            Serial.printf("MENU,SELECTED=%s,COMMAND=%c\\n", names[selected], modes[selected]);
        }
        if ((pressed & 2) && menu_open)
        {
            request_test(modes[selected]);
            menu_open = false;
        }
        if (pressed & 4)
        {
            menu_open = false;
            xEventGroupSetBits(events, abort_bit);
            Serial.printf("MENU,CANCEL_REQUESTED\\n");
        }
        if (pressed & 8)
        {
            report();
        }
        if (Serial.available())
        {
            char command = Serial.read();
            if (command == '?')
            {
                report();
            }
            if (command == 'x')
            {
                xEventGroupSetBits(events, abort_bit);
            }
            if (command == 'd' || command == 'f' || command == 'D' || command == 'F')
            {
                request_test(command);
            }
        }
        if (!dumping && xQueueReceive(completed, &result, 0) == pdTRUE)
        {
            Serial.printf("DATA,%c,%u,%lu,%lu,%u,%08lX\\n", result.mode, result.count,
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
                const Sample &row = samples[dump_index];
                Serial.printf("S,%u,%lu,%lu,%lu,%lu,%u", dump_index,
                    (unsigned long)row.service_start, (unsigned long)row.ready_observed,
                    (unsigned long)row.read_start, (unsigned long)row.read_end, row.fifo_bytes);
                for (uint8_t j = 0; j < 7; j++)
                {
                    Serial.printf(",%d", row.raw[j]);
                }
                Serial.printf("\\n");
                dump_index++;
            }
            if (dump_index == result.count)
            {
                Serial.printf("DONE,%c,%s\\n", result.mode, result.failure ? result.failure : "OK");
                dumping = false;
                xSemaphoreGive(buffer_free);
            }
        }
        if (!dumping && (uint32_t)(now - last_status) >= 1000)
        {
            Serial.printf("XBOX,CONNECTED=%u,FRESH=%u,CORE=%d,HEAP=%u,MIN_HEAP=%u\\n",
                connected, fresh, xPortGetCoreID(), ESP.getFreeHeap(), ESP.getMinFreeHeap());
            last_status = now;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
''' +s[end:]
s=s.replace('''    report();
}

void loop(void)''','''    read_registers(0x75, &identity, 1);
    acquisition_core = xPortGetCoreID();
    requests = xQueueCreate(1, sizeof(char));
    completed = xQueueCreate(1, sizeof(Trial));
    buffer_free = xSemaphoreCreateBinary();
    events = xEventGroupCreate();
    if (!requests || !completed || !buffer_free || !events)
    {
        ready = false;
        Serial.printf("ERROR,TASK_RESOURCES\\n");
        return;
    }
    xSemaphoreGive(buffer_free);
    if (xTaskCreatePinnedToCore(slow_worker, "xbox_slow", 6144, nullptr, 1, nullptr, 0) != pdPASS)
    {
        ready = false;
        Serial.printf("ERROR,SLOW_TASK_START\\n");
    }
}

void loop(void)''')
start=s.index('void loop(void)')
s=s[:start]+'''void loop(void)
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
'''
(p/'main.cpp').write_text(s,encoding='utf-8',newline='\n')
ini=(r/'lessons/imu-acquisition-benchmark/platformio.ini').read_text(encoding='utf-8')
ini=ini.replace('default_envs = acquisition','default_envs = xbox-menu').replace('[env:acquisition]','[env:xbox-menu]')
ini=ini.replace('lib_deps = adafruit/Adafruit MPU6050@2.2.9','''lib_deps =
    adafruit/Adafruit MPU6050@2.2.9
    asukiaaa/XboxSeriesXControllerESP32_asukiaaa@1.1.3
    h2zero/NimBLE-Arduino@2.5.1
    asukiaaa/XboxControllerNotificationParser@1.0.5
    asukiaaa/XboxSeriesXHIDReportBuilder_asukiaaa@1.0.1''')
(p/'platformio.ini').write_text(ini,encoding='utf-8',newline='\n')
(p/'source_stamp.py').write_bytes((r/'lessons/imu-acquisition-benchmark/source_stamp.py').read_bytes())
print('Prepared separate two-core Xbox menu; historical benchmark and production sources preserved.')
