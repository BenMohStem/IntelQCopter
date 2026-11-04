from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/attitude-preview/main.cpp'
s=p.read_text(encoding='utf-8').replace('#include <math.h>','#include <math.h>\n#include <Adafruit_AHRS_Mahony.h>')
helper='''// Quaternion complementary preview. Gains are provisional, not flight gains.
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

'''
assert 'static bool update_attitude' not in s
s=s.replace('static void process_one_sample(void)',helper+'static void process_one_sample(void)')
old='''    float yz = hypotf(drone[1], drone[2]);
    float roll = yz > 0.001f ? atan2f(drone[1], drone[2]) * radians_to_degrees : NAN;
    float pitch = yz > 0.001f ? atan2f(-drone[0], yz) * radians_to_degrees : NAN;'''
new='''    if (!update_attitude(drone, body_gyro))
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
    float pitch = asinf(sine_pitch) * radians_to_degrees;'''
assert old in s;s=s.replace(old,new)
s=s.replace('FIFO1000_V2','FUSION_PREVIEW_V1')
p.write_text(s,encoding='utf-8',newline='\n')
p=r/'lessons/attitude-preview/platformio.ini'
s=p.read_text();s=s.replace('lib_deps = adafruit/Adafruit MPU6050@2.2.9','lib_deps =\n    adafruit/Adafruit MPU6050@2.2.9\n    adafruit/Adafruit AHRS@2.4.0');p.write_text(s)
p=r/'tools/visualization/attitude_preview.py'
s=p.read_text(encoding='utf-8').replace('Calibrated gravity tilt preview - motors disabled','Quaternion fusion preview - motors disabled').replace("'Gravity tilt (deg)'","'Fused Euler angles (deg)'").replace('Near vertical pitch: roll is poorly observable from gravity','Near vertical pitch: Euler roll is ambiguous').replace('Gravity tilt only - axes provisional','Mahony fusion - axes provisional')
p.write_text(s,encoding='utf-8',newline='\n')
out=r/'measurements/software/2026-10-31-fusion';out.mkdir(exist_ok=True)
(out/'README.md').write_text('''# Quaternion fusion preview — 31 October 2026

Adafruit AHRS2.4.0 Mahony IMU algorithm replaces direct gravity-angle output in lessons/attitude-preview/main.cpp. This is a quaternion complementary filter, not Kalman. Existing sensor trims, biases and historical proper rotation remain unchanged; production firmware/main.cpp user edit remains preserved separately. Motors disabled; Xbox not integrated in this preview.

At startup initialize a unit quaternion from gravity roll/pitch, yaw zero (a reference, not a heading observation). Each FIFO sample propagates at nominal0.001s using body gyro converted rad/s to deg/s as the library expects. Kp=0.5, Ki=0 are provisional estimator gains, unrelated to flight PID gains. Accelerometer correction is skipped when |norm(a)-g|>=1.5m/s². This heuristic cannot reject every translational disturbance. Normalize q and reject invalid state. Euler pitch conversion is clamped before asin to prevent floating-point domain errors. Stop on startup acceleration inconsistent with gravity; keep assembly resting when starting.

Quaternion continuity does not make Euler angles unique near vertical pitch or prevent roll wrap at±180degrees. Warnings remain. MPU-only fusion cannot correct absolute yaw drift. No claim of flight accuracy or hard1ms full-controller deadlines. Source change invalidates old hardware computation/timing acceptance.

Build: .\\qcopter.ps1 build-attitude-preview
Upload: .\\qcopter.ps1 upload-attitude-preview --upload-port COM3
Display: .\\qcopter.ps1 view-attitude --port COM3

One short check: startup resting, gentle tilt/return about each physical axis, inspect recovery and existing logged diagnostics. Stop when behavior is understood; no new calibration campaign. Physical signed directions still require reference before motor feedback.

Primary source: https://github.com/adafruit/Adafruit_AHRS/tree/2.4.0 . Actual installed source is checked by the host tests; synthetic cases are not hardware validation. Runtime remains one main.cpp plus Arduino libraries.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-31-fusion-preview','date':'2026-10-31','domains':['operator-revision'],'summary':'User requests next implementation after quaternion explanation.','outcome':'Prepared motor-disabled Mahony quaternion fusion using pinned Arduino library, acceleration gating, normalization and Euler warnings; no Kalman/physical flight acceptance implied.'}
(r/'knowledge/events/2026-10-31-fusion-preview.json').write_text(json.dumps(event,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:f.write('\n[Quaternion fusion preview and limits](../measurements/software/2026-10-31-fusion/README.md). Adafruit Mahony, not Kalman.\n')
