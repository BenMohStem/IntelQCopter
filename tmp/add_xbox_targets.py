from pathlib import Path
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/attitude-preview/main.cpp';s=p.read_text(encoding='utf-8')
s=s.replace('#include <Adafruit_AHRS_Mahony.h>','#include <Adafruit_AHRS_Mahony.h>\n#include <XboxSeriesXControllerESP32_asukiaaa.hpp>')
code='''// Preview setpoints only: there is no PID, PWM, ESC or motor output here.
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;

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
        Serial.printf("T,%u,%.3f,%.3f,%.3f,%.3f\\n", valid,
            valid ? 20.0f * roll : 0.0f, valid ? 20.0f * pitch : 0.0f,
            valid ? 30.0f * yaw : 0.0f, valid ? vertical : 0.0f);
        previous_report = now;
    }
}

'''
s=s.replace('static void telemetry_worker(void *)',code+'static void telemetry_worker(void *)')
s=s.replace('''    TelemetryPacket packet;
    for (;;)''','''    TelemetryPacket packet;
    xbox.begin();
    NimBLEDevice::getScan()->setActiveScan(true);
    for (;;)''')
s=s.replace('''        if (xQueueReceive(telemetry_queue, &packet, portMAX_DELAY) != pdTRUE)''','''        xbox.onLoop();
        report_targets();
        if (xQueueReceive(telemetry_queue, &packet, pdMS_TO_TICKS(5)) != pdTRUE)''')
s=s.replace('FUSION_PREVIEW_V2','FUSION_TARGETS_V1')
p.write_text(s,encoding='utf-8',newline='\n')
p=r/'lessons/attitude-preview/platformio.ini';s=p.read_text();s+='\n    asukiaaa/XboxSeriesXControllerESP32_asukiaaa@1.1.3\n    h2zero/NimBLE-Arduino@2.5.1\n    asukiaaa/XboxControllerNotificationParser@1.0.5\n    asukiaaa/XboxSeriesXHIDReportBuilder_asukiaaa@1.0.1\n';p.write_text(s)
p=r/'tools/visualization/attitude_preview.py';s=p.read_text(encoding='utf-8')
s=s.replace('qhistory=[deque(maxlen=300) for _ in range(5)]','qhistory=[deque(maxlen=300) for _ in range(5)]\ntarget_history=[deque(maxlen=300) for _ in range(4)]')
s=s.replace("axes[0].set_ylim(-180,180)\nstatus=", "axes[0].set_ylim(-180,180)\ntarget_lines=[axes[0].plot([],[],'--',label=name)[0] for name in ['Roll target','Pitch target']]\ntarget_lines.append(axes[1].plot([],[],'--',label='Yaw-rate target')[0])\ntarget_text=axes[1].text(0.01,0.02,'Xbox: waiting',transform=axes[1].transAxes,fontsize=9)\nstatus=",1)
s=s.replace("        if verified and value.startswith('Q,'):", """        if verified and value.startswith('T,'):
            fields=value.split(',')
            if len(fields)==6:
                for buf,v in zip(target_history,[time.monotonic()-started]+[float(v) for v in fields[2:5]]):buf.append(v)
                target_text.set_text('Xbox valid='+fields[1]+' | vertical request='+fields[5]+' (normalized, no altitude control)')
        if verified and value.startswith('Q,'):""")
s=s.replace('    for i,line in enumerate(qlines):line.set_data(qhistory[0],qhistory[i+1])','    for i,line in enumerate(qlines):line.set_data(qhistory[0],qhistory[i+1])\n    for i,line in enumerate(target_lines):line.set_data(target_history[0],target_history[i+1])')
s=s.replace('return lines+qlines','return lines+qlines+target_lines')
p.write_text(s,encoding='utf-8',newline='\n');compile(s,str(p),'exec')
out=r/'measurements/software/2026-10-31-xbox-targets';out.mkdir(exist_ok=True)
(out/'README.md').write_text('''# Next small feature: Xbox targets, motors disabled

One-file preview now reads Xbox on core0 alongside telemetry; IMU/fusion remains core1. Same pinned Xbox library as the earlier bench lesson. Xbox connection attempts can block its worker: telemetry queue overflow stops acquisition rather than hiding loss; integration timing must be measured on ESP32. No hard deadline guarantee.

Right stick: roll/pitch targets within±20deg. Left horizontal: yaw rate within±30deg/s. Left vertical: normalized vertical request±1 (no altitude estimator or control; barometer absent). 8%deadband rescaled to full travel;0.6s+0.4s³ gives gentler response. Intended signs follow existing angle convention, but physical stick directions still require operator confirmation. RT/LT boost and mode gestures deferred.

New T records contain valid, roll_deg, pitch_deg, yaw_deg_s, vertical_normalized. A recent report within300ms and sticks observed neutral are required before accepting values. Disconnect/stale data zeros all display targets and clears neutral readiness. Idle controllers may send infrequent reports: if valid drops at rest this conservative bench guard must be diagnosed from notification cadence before flight, not relaxed blindly. Recenter to requalify. Connection alone does not authorize motors. Currently these values only go to graphs; there is no PID/mixer/output adapter.

Upload-attitude-preview then view-attitude commands unchanged. Close existing viewer first. One short session: connect, center sticks, move each separately and release, then disconnect. Check dashed targets, neutral return and validity. No sensor calibration needed. This is planned operator validation, not an already measured pass.
''',encoding='utf-8')
print('Added bounded Xbox target telemetry and dashed target plots, without motor outputs.')
