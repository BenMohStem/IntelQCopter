from pathlib import Path
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise');p=r/'lessons/attitude-preview/main.cpp';s=p.read_text(encoding='utf-8')
s=s.replace('static XboxSeriesXControllerESP32_asukiaaa::Core xbox;', '''static XboxSeriesXControllerESP32_asukiaaa::Core xbox;
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
''')
old='''        Serial.printf("T,%u,%.3f,%.3f,%.3f,%.3f\\n", valid,
            valid ? 20.0f * roll : 0.0f, valid ? 20.0f * pitch : 0.0f,
            valid ? 30.0f * yaw : 0.0f, valid ? vertical : 0.0f);'''
new='''        TargetPacket target = {now, valid, valid ? 20.0f * roll : 0.0f,
            valid ? 20.0f * pitch : 0.0f, valid ? 30.0f * yaw : 0.0f,
            valid ? vertical : 0.0f};
        xQueueOverwrite(target_queue, &target);'''
assert old in s;s=s.replace(old,new)
s=s.replace('static void telemetry_worker(void *)', '''static void xbox_worker(void *)
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

static void telemetry_worker(void *)''')
s=s.replace('''    TelemetryPacket packet;
    xbox.begin();
    NimBLEDevice::getScan()->setActiveScan(true);''','''    TelemetryPacket packet;
    TargetPacket target = {};
    uint32_t previous_target_report = 0;''')
s=s.replace('''        xbox.onLoop();
        report_targets();
        if (xQueueReceive(telemetry_queue''','''        xQueueReceive(target_queue, &target, 0);
        uint32_t now = millis();
        if ((uint32_t)(now - previous_target_report) >= 100)
        {
            bool valid = target.valid && (uint32_t)(now - target.timestamp) <= 300;
            Serial.printf("T,%u,%.3f,%.3f,%.3f,%.3f\\n", valid,
                valid ? target.roll : 0.0f, valid ? target.pitch : 0.0f,
                valid ? target.yaw : 0.0f, valid ? target.vertical : 0.0f);
            previous_target_report = now;
        }
        if (xQueueReceive(telemetry_queue''')
s=s.replace('''    return telemetry_queue && telemetry_drained
        && xTaskCreatePinnedToCore''','''    target_queue = xQueueCreate(1, sizeof(TargetPacket));
    return telemetry_queue && telemetry_drained && target_queue
        && xTaskCreatePinnedToCore(xbox_worker, "xbox", 6144, nullptr, 1, nullptr, 0) == pdPASS
        && xTaskCreatePinnedToCore''')
p.write_text(s,encoding='utf-8',newline='\n')
print('BLE and serial tasks isolated with timestamped one-slot target queue.')
