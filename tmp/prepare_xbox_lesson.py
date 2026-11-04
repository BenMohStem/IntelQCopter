from pathlib import Path
import json

root=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
def write(path,text):
    p=root/path;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(text,encoding='utf-8',newline='\n')
def write_json(path,data):
    write(path,json.dumps(data,indent=2,ensure_ascii=False)+'\n')

write('lessons/xbox-input/main.cpp',r'''#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Input-only lesson: no sensor changes, PWM, Servo, PID or motor commands.
XboxSeriesXControllerESP32_asukiaaa::Core xbox;
static uint32_t last_print_ms = 0;

void setup(void)
{
    // Keep signal pins from both previous carrier layouts as inputs.
    const uint8_t motor_pins[] = {12, 1, 2, 14, 4, 21};
    for (uint8_t pin : motor_pins)
    {
        pinMode(pin, INPUT_PULLDOWN);
    }
    Serial.begin(115200);
    delay(1000);
    Serial.printf("XBOX_INPUT_LESSON_V1,MOTOR_OUTPUTS_DISABLED\n");
    xbox.begin();
}

void loop(void)
{
    xbox.onLoop();
    uint32_t now = millis();
    if ((uint32_t)(now - last_print_ms) < 200)
    {
        delay(1);
        return;
    }
    last_print_ms = now;

    if (!xbox.isConnected())
    {
        Serial.printf("XBOX,CONNECTED=0,INPUT_READY=0\n");
        return;
    }
    if (xbox.isWaitingForFirstNotification())
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,WAITING_FOR_DATA\n");
        return;
    }

    uint32_t age_ms = now - (uint32_t)xbox.getReceiveNotificationAt();
    if (age_ms > 300)
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,STALE,AGE_MS=%lu\n",
            (unsigned long)age_ms);
        return;
    }

    Serial.printf("XBOX,CONNECTED=1,INPUT_READY=1,AGE_MS=%lu\n%s\n",
        (unsigned long)age_ms, xbox.xboxNotif.toString().c_str());
}
'''.replace('\\n','\n') if False else r'''#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Input-only lesson: no sensor changes, PWM, Servo, PID or motor commands.
XboxSeriesXControllerESP32_asukiaaa::Core xbox;
static uint32_t last_print_ms = 0;

void setup(void)
{
    // Keep signal pins from both previous carrier layouts as inputs.
    const uint8_t motor_pins[] = {12, 1, 2, 14, 4, 21};
    for (uint8_t pin : motor_pins)
    {
        pinMode(pin, INPUT_PULLDOWN);
    }
    Serial.begin(115200);
    delay(1000);
    Serial.printf("XBOX_INPUT_LESSON_V1,MOTOR_OUTPUTS_DISABLED\n");
    xbox.begin();
}

void loop(void)
{
    xbox.onLoop();
    uint32_t now = millis();
    if ((uint32_t)(now - last_print_ms) < 200)
    {
        delay(1);
        return;
    }
    last_print_ms = now;

    if (!xbox.isConnected())
    {
        Serial.printf("XBOX,CONNECTED=0,INPUT_READY=0\n");
        return;
    }
    if (xbox.isWaitingForFirstNotification())
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,WAITING_FOR_DATA\n");
        return;
    }

    uint32_t age_ms = now - (uint32_t)xbox.getReceiveNotificationAt();
    if (age_ms > 300)
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,STALE,AGE_MS=%lu\n",
            (unsigned long)age_ms);
        return;
    }

    Serial.printf("XBOX,CONNECTED=1,INPUT_READY=1,AGE_MS=%lu\n%s\n",
        (unsigned long)age_ms, xbox.xboxNotif.toString().c_str());
}
''')

write('lessons/xbox-input/platformio.ini','''[platformio]
default_envs = xbox-input
src_dir = .

[env:xbox-input]
platform = espressif32@7.1.3
board = esp32-s3-devkitc-1
framework = arduino
build_src_filter = +<main.cpp>
monitor_speed = 115200
lib_deps =
    asukiaaa/XboxSeriesXControllerESP32_asukiaaa@1.1.3
    h2zero/NimBLE-Arduino@2.5.1
    asukiaaa/XboxControllerNotificationParser@1.0.5
    asukiaaa/XboxSeriesXHIDReportBuilder_asukiaaa@1.0.1
''')

write('docs/revision-and-xbox.md',r'''# Resume with your own tests - 30 October 2026

Current hardware, user confirmed: old ESP32-S3 PCB with MPU6050 only. The new PCB is in progress, not substituted or accepted. Windows inventory currently lists COM3, not COM4; port identity must be checked by the operator. No serial port was opened for this lesson preparation.

## Our next small lesson: Xbox input only

One 57-line Arduino/C-style main.cpp uses the existing asukiaaa Xbox library. This is an isolated PlatformIO project; firmware/main.cpp and its local gyro-matrix edit are preserved. This firmware executes ON THE ESP32-S3. PC/MSVC tests were optional software checks, never deployment targets.

Build without flashing:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 build-xbox
```

When you choose to upload: remove propellers and disconnect ESC signal plugs for this input-only exercise. Close any Serial Monitor. Confirm the actual ESP32 port by unplugging/replugging its USB and checking PlatformIO Devices. Uploading replaces the currently running MPU firmware; its source/calibration constants remain saved in the project.

```powershell
.\qcopter.ps1 upload-xbox --upload-port COM3
.\qcopter.ps1 monitor-xbox --port COM3
```

Replace COM3 if your confirmed port is different. No upload was performed during preparation.

1. Turn on the Xbox Series X/S controller and hold its pairing button until the logo flashes rapidly. Disconnect it from the PC/console first; it must connect directly to the ESP32 over BLE.
2. Observe CONNECTED=1 and INPUT_READY=1. Move each stick, press each button, and move LT/RT. The library prints raw values; there is no throttle or flight interpretation yet.
3. Release every control: verify sticks return approximately to their centre and triggers to rest. Record observed ranges before choosing deadbands.
4. Turn the controller off: input must become disconnected/stale/not-ready. Turn it back on and inspect reconnection. A lack of fresh notifications is not necessarily a broken radio: an unchanged control may not generate reports; move a stick to check.
5. Keep a short serial log and your observations. Pairing, axis directions, ranges and reconnect remain PENDING until this test is done on your controller.

The 300 ms freshness threshold is only a bench display rule, not an accepted flight failsafe. BLE connect/reconnect code may block. Do not insert xbox.onLoop into the 1 kHz acquisition task. Later integration needs a separate slow task, a synchronized latest-command snapshot, measured latency and source-bound timing regression. No PWM, Servo, arming, PID, mixer or motor output is included.

Library: https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 (MIT). Upstream reports testing controller firmware5.13.3143.0; other firmware compatibility must be verified, not assumed. Dependencies are pinned in the lesson platformio.ini. No controller firmware update is required automatically.

## Revision order: do one, inspect its evidence, then continue

Historical passed results stay preserved. These rows are fresh OPERATOR REVISION gates, all pending. We are not deleting commits, refitting calibration or declaring previous tests invalid merely because you did not run them personally.

| Revision | What you do | What we learn |
|---|---|---|
| R0 - current source | Review the local gyro_to_drone coefficient difference before uploading the MPU firmware | Avoid applying historical rotation proofs to changed coefficients |
| R1 - Xbox input | Do the five checks above | Controller pairing, raw controls and link behaviour on your ESP32 |
| R2 - restore MPU bench | Upload the reviewed MPU firmware; inspect identity, ranges and offset readback | Correct sensor and boot correction, no duplicate bias subtraction |
| R3 - stationary recording | Run one20s record-1khz capture | Sample integrity, corrected values, actual timing; independently level it only if testing level |
| R4 - known rotations | Positive/negative X/Y/Z movements with an angle reference | Physical gyro signs, frame and scale; do not use these for stationary bias fitting |
| R5 - replay yourself | Run the existing offline tests and saved-record analysis with explanations | Understand54software tests and19host cases; host timing is not MCU timing |
| R6 - small attitude update | Only after the frame gate: add the estimator to MCU and test it | Motion response, acceleration sensitivity and combined1kHz timing |
| R7 - control | Add one PID capability at a time and validate in your simulation | Rate yaw, angle roll/pitch; altitude waits for a real altitude sensor |

For R3, after the existing MPU source is reviewed/restored:

```powershell
.\qcopter.ps1 record-1khz --port COM3 --condition stationary --note "old PCB; MPU only; operator revision"
```

For R5, after reviewing the local gyro coefficient:

```powershell
.\qcopter.ps1 check-all --native
```

This command builds/runs software checks on the PC, never flashes the board. A failure stops our progression; repair that stage, preserve the failed evidence, then rebuild its dependent steps. No blanket re-calibration is needed if the sensor and mounting are unchanged.

## FIFO versus direct register reads

At1kHz the MPU produces one sample each1ms. Direct reads give the register contents at read time; a late ESP32 read can skip intermediate samples. A coherent burst read avoids mixing bytes but cannot reconstruct a skipped sample.

FIFO queues successive sensor samples. Our MPU packet is14bytes (acceleration, temperature, gyro); the1024-byte FIFO holds73complete such packets, about73ms at1kHz. This is theoretical storage, not an allowed flight delay: old samples are stale and overflow must be rejected. FIFO preserves acquisition history; it does not guarantee a fresh1kHz PID output. Sample rate, sensor filter delay, FIFO backlog, CPU execution time and motor update rate are separate measurements.

The PC was used to analyse logs and inject software faults. Actual firmware deployment remains ESP32-S3/Arduino/PlatformIO.
''')

write('docs/battery-current-sensor.md',r'''# Battery current sensing - selection proposal, 30 October 2026

Use ONE sensor in the main battery feed before the power distribution branches, so it measures all ESCs AND avionics. Four ESC-current sensors add weight, wiring and ADC channels; useful later for motor diagnostics, not necessary for initial charge accounting. Do not put a single sensor after only one ESC branch and call it battery current.

```text
3S LiPo positive -> current-sensor primary conductor -> distribution -> 4 ESCs + regulators
3S LiPo negative ----------------------------------> common power return
Sensor signal output -> filtered/protected ADC input (pin not assigned yet)
```

The main high-current wiring is separate from the home-etched signal PCB. No motor phase current is being measured here.

## Range and references

Four40A ESC labels sum to160A, but this is neither measured consumption nor a guarantee that wiring/battery/motors safely deliver160A. Include avionics and transient headroom; a200A-class sensing range is a conservative starting candidate until propulsion tests establish demand. A chip measuring200A does not automatically make a cheap module or connector safe at200A continuously.

Preferred component candidate: **Allegro ACS773ECB-200B-PFF-T**, bidirectional±200A, nominal3.3V supply, ratiometric output; nominal6.6mV/A and1.65V zero-current output at3.3V. This is a component reference, not a reviewed ready-to-wire breakout. Confirm exact suffix, genuine component, PCB/conductor thermal rating, decoupling, output conditioning and actual module output range before buying/wiring. Supply/output/ADC limits need validation; do not assume every module labelled ACS773 accepts the same wiring.

Practical ready-board alternative: **Matek HCS-150A**, rated0-150A DC, advertised0-3.3V current/voltage outputs, sensitivity20mV/A, approximately7g. It senses the magnetic field around the designated battery conductor. Use it only if measured/design peak current stays below its150A range with margin; it is not a160A-range solution. Manufacturer specifies5% current-output error, so do not call its charge estimate laboratory-accurate. Keep magnetometer placement away from power-current fields and the current sensor's magnetic assembly.

ACS758200A modules are an older possible option, but Allegro marks ACS758 NOT FOR NEW DESIGN and directs new designs to ACS772/ACS773. A5V-powered module may output above the ESP32 ADC limit: inspect its datasheet/circuit before connecting. No sensor has been purchased/installed/accepted by this recommendation.

## Measurement and charge accounting

For the ratiometric200B candidate at nominal supply:

I[A] = (Vout[V] - Vzero[V]) / 0.0066[V/A]

Measure the actual zero offset and slope with a known current/reference meter. Use calibrated ADC voltage, check range/noise/temperature, and sample battery current in a slow task rather than blocking the1kHz IMU task. Small persistent zero bias accumulates into charge error.

Used capacity[mAh] = sum(I[A] * delta_t[s]) / 3.6

Remaining fraction approximately = initial_fraction - used_capacity / usable_capacity

Usable capacity is measured, not automatically the4200mAh label. Full battery voltage, load sag, temperature, charge history and current-sensor bias also matter. Kalman filtering cannot create missing reference information. Battery voltage alone cannot accurately identify20% charge under arbitrary load. This document does not implement a motor-disarm threshold or alter the disabled-output policy.

Sources checked30October2026:
- https://www.allegromicro.com/~/media/files/datasheets/acs773-datasheet.pdf
- https://www.allegromicro.com/en/products/sense/current-sensor-ics/integrated-current-sensors/acs758
- https://www.mateksys.com/?portfolio=hcs-150a
''')

p=root/'docs/PLAN.md';old=p.read_text(encoding='utf-8')
intro='''# Current software plan

## Operator revision priority - 30 October 2026

User is still on the OLD ESP32-S3 PCB with MPU6050 only; new PCB is in progress. Resume by reviewing the current source and personally repeating small steps. Historical evidence remains historical, not a fresh pass for changed source.

**Next implementation: isolated Xbox BLE input-only lesson**, before combining inputs with acquisition/control. [New revision sequence and commands](revision-and-xbox.md) is the current entry point. It preserves firmware/main.cpp and its uncommitted gyro coefficient, keeps motors disabled, and requires no ROS computer. [Battery current-sensor selection](battery-current-sensor.md) proposes one main-feed sensor; no hardware/ADC pin has been accepted.

The following chronological sections preserve earlier planning. Where they say Xbox is a later stage or physical tests are deferred, this operator revision sequence supersedes the scheduling, without bypassing physical acceptance dependencies.
'''
assert old.startswith('# Current software plan');write('docs/PLAN.md',intro+old[len('# Current software plan'):])

p=root/'project.json';d=json.loads(p.read_text())
for name,target,desc in [('build-xbox','build','Compile isolated ESP32-S3 Xbox input lesson; no upload or sensor changes'),('upload-xbox','upload','Manually upload input-only Xbox lesson; replaces running firmware; specify --upload-port'),('monitor-xbox','monitor','Open Xbox input Serial Monitor at115200; specify --port')]:
    args=['run','--project-dir','lessons/xbox-input','--environment','xbox-input']
    if target=='upload':args+=['--target','upload']
    if target=='monitor':args=['device','monitor','--baud','115200']
    d['commands'][name]={'runtime':'pio','path':'lessons/xbox-input/platformio.ini','args':args,'description':desc}
write_json('project.json',d)
p=root/'important_sources/README.md';write('important_sources/README.md',p.read_text(encoding='utf-8')+'\n\n## Current operator revision - 30 October 2026\n\n[Small Xbox input-only main.cpp](../lessons/xbox-input/main.cpp), [PlatformIO configuration](../lessons/xbox-input/platformio.ini), [revision guide](../docs/revision-and-xbox.md). Temporary independent lesson; replaces running firmware only when YOU upload it. No motors/PID; original MPU runtime remains in firmware/main.cpp. [Battery-current sensor choices](../docs/battery-current-sensor.md).\n')

write_json('knowledge/tasks/xbox-input-bench.json',{'schema':1,'id':'xbox-input-bench','status':'ready','title':'Operator revision: isolated ESP32-S3 Xbox BLE input lesson','depends_on':[],'scopes':['lessons/xbox-input','docs/revision-and-xbox.md'],'evidence':[],'acceptance':['ESP32-S3 target build and retained compiler log; no host build presented as hardware','Operator verifies stick/trigger/button ranges and disconnect/reconnect before integration','No PWM/Servo/motor output or calibration change; preserve original MPU source','Integrated BLE plus IMU timing remains a separate pending gate']})
domain={'schema':2,'id':'operator-revision','updated':'2026-10-30','entries':[
 {'id':'revision.old-pcb','status':'user_preference','summary':'Current hardware is old ESP32-S3 PCB with MPU6050 only; new PCB in progress. User wants to personally repeat earlier assistant-only steps without losing evidence.','sources':[{'kind':'user','date':'2026-10-30','reference':'Latest resumption and revision instructions'}]},
 {'id':'revision.xbox-next','status':'user_preference','summary':'Next small implementation is Xbox Series X/S BLE library input-only on ESP32-S3, independent from ROS and before combined FC integration. No new motor arming authorization.','sources':[{'kind':'user','date':'2026-10-30','reference':'Make Xbox library implementation next step; return to ESP32-S3 and repeat tests together'}]},
 {'id':'revision.port','status':'pending','summary':'User suggests COM4; Windows port inventory currently lists COM3. No serial device opened or flashed; operator must identify the ESP32.','sources':[{'kind':'file','path':'docs/revision-and-xbox.md'}]},
 {'id':'revision.calibration-review','status':'pending','summary':'Existing uncommitted gyro-matrix coefficient differs from tested snapshot. Preserve it and review before MPU upload/tests; do not silently refresh fingerprints or recalibrate from validation data.','sources':[{'kind':'file','path':'docs/revision-and-xbox.md'}]}
]}
write_json('knowledge/domains/operator-revision.json',domain)
p=root/'knowledge/index.json';d=json.loads(p.read_text());d['updated']='2026-10-30';d['domains'].append({'id':'operator-revision','path':'domains/operator-revision.json'});write_json('knowledge/index.json',d)
write_json('knowledge/events/2026-10-30-operator-revision-xbox.json',{'schema':2,'id':'2026-10-30-operator-revision-xbox','date':'2026-10-30','domains':['operator-revision'],'summary':'User requests FIFO explanation, ESP32-S3 Xbox next step, battery-current sensor choice and personal repetition of assistant-only validations.','outcome':'Prepared isolated Arduino Xbox input lesson and ordered revision guide, preserving MPU source/calibration/history. No flash, COM access or motor operation. Complete build result recorded in lesson validation after compilation.'})
print('Prepared isolated Xbox input lesson, operator revision plan and current-sensor proposal.')
