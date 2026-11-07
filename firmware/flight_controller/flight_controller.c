/*
 * QCopter Flight Controller
 * ============================================================================
 *
 * Target: RP2350 Pico 2 firmware core (see the repository CMake target)
 * Control period target: 500 Hz (2 ms); timing is supplied by the caller
 * Scope: estimation/control math only; sensor transport and ESC output are
 *        not implemented in this source file
 *
 * SAFETY - Physical motor arming remains DISABLED:
 *   - Motor GPIOs 12/11/14/13 = INPUT ONLY
 *   - No auto-arm on startup/reconnect
 *   - Internal arming state does not enable physical motor output
 *   - All motor outputs are mathematical demands only; no physical output
 *
 * Control architecture (per axis, ESC-command fractions 0..1):
 *   Roll:  angle PID -> rate demand -> rate PID -> torque demand
 *   Pitch: angle PID -> rate demand -> rate PID -> torque demand
 *   Yaw:   rate PID directly (rate demand -> torque demand)
 *   Altitude: position PID -> vertical speed demand -> vertical speed PID
 *             -> vertical acceleration command -> collective thrust
 *   Vertical speed: throttle stick -> vertical speed demand
 *
 * Motor mixing (X-frame): collective +/- roll torque +/- pitch torque +/- yaw torque
 *
 * SAFETY: Physical motor arming remains DISABLED. This code produces
 * mathematical demands only; no motor outputs are ever generated.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <float.h>

/* BEGIN WIRE PROTOCOL v1: keep identical in both firmware files. */
/* Fixed-size, endian-stable command frame shared by the two Pico firmwares. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define QCOPTER_FRAME_SIZE 22u
#define QCOPTER_PROTOCOL_VERSION 1u
#define QCOPTER_AXIS_SCALE 10000
#define QCOPTER_TRIGGER_SCALE 1000u
#define QCOPTER_THROTTLE_SCALE 10000u

enum {
    QCOPTER_BUTTON_LB = 1u << 0,
    QCOPTER_BUTTON_RB = 1u << 1,
    QCOPTER_BUTTON_NORTH = 1u << 2,   /* Y button - arm/disarm */
    QCOPTER_BUTTON_EAST = 1u << 3,    /* B button */
    QCOPTER_BUTTON_SOUTH = 1u << 4,   /* A button - emergency disarm */
    QCOPTER_BUTTON_WEST = 1u << 5     /* X button */
};

typedef struct {
    uint16_t sequence;
    int16_t roll;
    int16_t pitch;
    int16_t yaw_rate;
    uint16_t throttle;
    uint16_t lt;
    uint16_t rt;
    uint16_t buttons;
} qcopter_command_frame_t;

/* ===========================================================================
   CALIBRATION DATA (auto-generated)
   =========================================================================== */
#include "qcopter_calibration.h"

/* ===========================================================================
   CONTROL LIMITS (ESC-command fractions 0..1, not RPM/thrust)
   =========================================================================== */
#define FC_ARMED_IDLE_FRACTION   0.10f   /* 10% armed idle without throttle */
#define FC_NORMAL_MOTOR_CEILING  0.45f   /* 45% maximum normal command */
#define FC_BOOST_MOTOR_CEILING   0.65f   /* 65% maximum boost command */
#define FC_MAX_VERTICAL_SPEED_MPS 2.0f
#define FC_MAX_CLIMB_MPS 1.5f
#define FC_MAX_DESCENT_MPS 1.0f
#define FC_MAX_ROLL_DEG 35.0f
#define FC_MAX_PITCH_DEG 35.0f
#define FC_MAX_YAW_RATE_DPS 180.0f
#define FC_MAX_COLLECTIVE_ACCEL_MPS2 3.0f
#define FC_MAX_SLEW_PER_SECOND 10.0f     /* max motor command change per second */
#define FC_LOOP_HZ 500.0f
#define FC_LOOP_PERIOD_US 2000u
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f
#define FC_RADIO_TIMEOUT_US 150000u
#define FC_GRAVITY_MPS2 9.80665f

/* ===========================================================================
   SENSOR SAMPLE STRUCTURE
   =========================================================================== */
typedef struct {
    bool valid;
    bool fresh;
    uint32_t timestamp_us;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;
    float accel_x_g, accel_y_g, accel_z_g;   /* m/s^2, calibrated */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; /* deg/s, calibrated */
    float temperature_c;
    float barometer_height_m;   /* meters relative to zero */
    bool barometer_valid;
    bool barometer_updated;
    float barometer_raw_pa;
    float barometer_temperature_c;
    float magnetometer_x_uT, magnetometer_y_uT, magnetometer_z_uT;
    bool magnetometer_valid;
    bool magnetometer_fresh;
    float vertical_acceleration_mps2; /* gravity compensated */
} sensor_sample_t;

typedef struct {
    bool armed;
    bool sensor_fault;
    float motor[4];   /* FL, FR, RR, RL: ESC command fractions 0..1 */
} control_output_t;

/* ===========================================================================
   GLOBAL STATE
   =========================================================================== */
static bool armed = false;
static bool failsafe = false;
static bool controller_connected = false;
static uint32_t last_controller_ms = 0;
static float hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;
static bool hover_collective_valid = false;
static float baro_zero_offset_m = 0.0f;
static bool baro_zeroed = false;
static float lt_reduce_state = 0.0f;
static float rt_boost_state = 0.0f;
static float altitude_ref_m = 0.0f;
static bool altitude_ref_initialized = false;

/* Calibration data */
static float cal_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float cal_accel_scale[3][3] = {{1.0f, 0.0f, 0.0f},
                                       {0.0f, 1.0f, 0.0f},
                                       {0.0f, 0.0f, 1.0f}};
static float cal_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float cal_mag_hard_iron[3] = {0.0f, 0.0f, 0.0f};
static float cal_mag_soft_iron[3][3] = {{1.0f, 0.0f, 0.0f},
                                         {0.0f, 1.0f, 0.0f},
                                         {0.0f, 0.0f, 1.0f}};
static bool calibration_loaded = false;

/* ===========================================================================
   PID CONTROLLER
   =========================================================================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_measurement;
    float integrator_limit;
    float output_limit;
    bool initialized;
} pid_t;

static void pid_reset(pid_t *pid) {
    pid->integrator = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->initialized = false;
}

static float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {
    if (!(dt > 0.0f) || dt > FC_MAX_CONTROL_DT) {
        pid_reset(pid);
        return 0.0f;
    }
    const float error = setpoint - measurement;
    const float derivative = pid->initialized
        ? (measurement - pid->previous_measurement) / dt : 0.0f;
    const float candidate_i = fmaxf(-pid->integrator_limit,
        fminf(pid->integrator_limit, pid->integrator + error * dt));
    const float raw = pid->kp * error + pid->ki * candidate_i
                    - pid->kd * derivative;
    const float output = fmaxf(-pid->output_limit,
                               fminf(pid->output_limit, raw));
    /* Conditional integration (anti-windup): freeze integrator when saturated
       and driving further into saturation */
    if (raw == output || (raw > output && error < 0.0f)
        || (raw < output && error > 0.0f)) {
        pid->integrator = candidate_i;
    }
    pid->previous_measurement = measurement;
    pid->initialized = true;
    return output;
}

/* PID instances - Placeholder gains. Replace with measured values after
 * calibration flight testing. */
static pid_t pid_roll_angle = { .kp = 0.65f, .ki = 0.02f, .kd = 0.03f,
    .integrator_limit = 2.0f, .output_limit = 30.0f };
static pid_t pid_pitch_angle = { .kp = 0.65f, .ki = 0.02f, .kd = 0.03f,
    .integrator_limit = 2.0f, .output_limit = 30.0f };
static pid_t pid_roll_rate = { .kp = 0.078f, .ki = 0.039f, .kd = 0.00006f,
    .integrator_limit = 15.0f, .output_limit = 0.35f };
static pid_t pid_pitch_rate = { .kp = 0.083f, .ki = 0.041f, .kd = 0.00006f,
    .integrator_limit = 15.0f, .output_limit = 0.35f };
static pid_t pid_yaw_rate = { .kp = 0.087f, .ki = 0.028f, .kd = 0.00012f,
    .integrator_limit = 8.0f, .output_limit = 0.35f };
static pid_t pid_altitude = { .kp = 0.40f, .ki = 0.005f, .kd = 0.015f,
    .integrator_limit = 2.0f, .output_limit = 1.0f };
static pid_t pid_vertical_speed = { .kp = 1.0f, .ki = 0.06f, .kd = 0.025f,
    .integrator_limit = 5.0f, .output_limit = 1.0f };

/* ===========================================================================
   QUATERNION ATTITUDE ESTIMATOR
   =========================================================================== */
typedef struct {
    float w, x, y, z;
} quat_t;

static void quat_normalize(quat_t *q) {
    float n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (n > 0.001f) { q->w/=n; q->x/=n; q->y/=n; q->z/=n; }
    else { q->w=1.0f; q->x=0.0f; q->y=0.0f; q->z=0.0f; }
}

static void quat_to_euler(const quat_t *q, float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * FC_RAD_TO_DEG;
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch_deg = copysignf(90.0f, sinp);
    else
        *pitch_deg = asinf(sinp) * FC_RAD_TO_DEG;
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * FC_RAD_TO_DEG;
}

typedef struct {
    quat_t q;
    float gyro_bias[3];
    float roll_deg, pitch_deg, yaw_deg;
    float roll_rate_dps, pitch_rate_dps, yaw_rate_dps;
    bool initialized;
    float kp_roll, kp_pitch, kp_yaw;
} attitude_t;

static attitude_t attitude = {0};

static void attitude_init(attitude_t *att, const float accel[3]) {
    float roll = atan2f(accel[1], accel[2]) * FC_RAD_TO_DEG;
    float pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * FC_RAD_TO_DEG;
    float cr = cosf(roll * FC_DEG_TO_RAD), sr = sinf(roll * FC_DEG_TO_RAD);
    float cp = cosf(pitch * FC_DEG_TO_RAD), sp = sinf(pitch * FC_DEG_TO_RAD);
    att->q.w = cr*cp*1.0f + sr*sp*0.0f;
    att->q.x = sr*cp*1.0f - cr*sp*0.0f;
    att->q.y = cr*sp*1.0f + sr*cp*0.0f;
    att->q.z = cr*cp*0.0f - sr*sp*0.0f;
    quat_normalize(&att->q);
    att->roll_deg = roll;
    att->pitch_deg = pitch;
    att->yaw_deg = 0.0f;
    att->initialized = true;
}

static void attitude_update(attitude_t *att, const float gyro_dps[3],
                            const float accel_mps2[3],
                            const float mag_uT[3], bool mag_valid, float dt) {
    if (!att->initialized) return;
    float gx = gyro_dps[0] - att->gyro_bias[0];
    float gy = gyro_dps[1] - att->gyro_bias[1];
    float gz = gyro_dps[2] - att->gyro_bias[2];
    float hdt = dt * 0.5f;
    float qx = gx * FC_DEG_TO_RAD * hdt;
    float qy = gy * FC_DEG_TO_RAD * hdt;
    float qz = gz * FC_DEG_TO_RAD * hdt;
    float nqd = sqrtf(qx*qx + qy*qy + qz*qz);
    quat_t qd; qd.w = cosf(nqd);
    if (nqd > 0.001f) { float s = sinf(nqd)/nqd; qd.x = qx*s; qd.y = qy*s; qd.z = qz*s; }
    quat_t qn;
    qn.w = att->q.w*qd.w - att->q.x*qd.x - att->q.y*qd.y - att->q.z*qd.z;
    qn.x = att->q.w*qd.x + att->q.x*qd.w + att->q.y*qd.z - att->q.z*qd.y;
    qn.y = att->q.w*qd.y - att->q.x*qd.z + att->q.y*qd.w + att->q.z*qd.x;
    qn.z = att->q.w*qd.z + att->q.x*qd.y - att->q.y*qd.x + att->q.z*qd.w;
    att->q = qn; quat_normalize(&att->q);

    /* Accelerometer correction (roll/pitch only) */
    if (accel_mps2) {
        float n = sqrtf(accel_mps2[0]*accel_mps2[0] + accel_mps2[1]*accel_mps2[1] + accel_mps2[2]*accel_mps2[2]);
        if (n > 0.5f && n < 2.0f) {
            float ax = accel_mps2[0]/n, ay = accel_mps2[1]/n, az = accel_mps2[2]/n;
            float vx = 2.0f*(att->q.x*att->q.z - att->q.w*att->q.y);
            float vy = 2.0f*(att->q.y*att->q.z + att->q.w*att->q.x);
            float vz = att->q.w*att->q.w - att->q.x*att->q.x - att->q.y*att->q.y + att->q.z*att->q.z;
            float ex = ay*vz - az*vy;
            float ey = az*vx - ax*vz;
            float ez = ax*vy - ay*vx;
            gx += att->kp_roll * ex;
            gy += att->kp_roll * ey;
            gz += att->kp_roll * ez;
        }
    }

    /* Magnetometer correction (yaw only) */
    if (mag_valid && mag_uT) {
        float mx = mag_uT[0], my = mag_uT[1], mz = mag_uT[2];
        float emx = 2.0f*(att->q.w*att->q.x + att->q.y*att->q.z)*mx
                  + 2.0f*(att->q.w*att->q.y - att->q.x*att->q.z)*my
                  + (att->q.w*att->q.w - att->q.x*att->q.x - att->q.y*att->q.y + att->q.z*att->q.z)*mz;
        float emy = 2.0f*(att->q.x*att->q.y - att->q.w*att->q.z)*mx
                  + (att->q.w*att->q.w + att->q.x*att->q.x - att->q.y*att->q.y - att->q.z*att->q.z)*my
                  + 2.0f*(att->q.w*att->q.x + att->q.y*att->q.z)*mz;
        float heading = atan2f(-emy, emx) * FC_RAD_TO_DEG;
        float err = heading - att->yaw_deg;
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        gz += att->kp_yaw * err;
    }

    /* Re-integrate with corrected rates */
    qx = gx * FC_DEG_TO_RAD * hdt;
    qy = gy * FC_DEG_TO_RAD * hdt;
    qz = gz * FC_DEG_TO_RAD * hdt;
    nqd = sqrtf(qx*qx + qy*qy + qz*qz);
    qd.w = cosf(nqd);
    if (nqd > 0.001f) { float s = sinf(nqd)/nqd; qd.x = qx*s; qd.y = qy*s; qd.z = qz*s; }
    qn.w = att->q.w*qd.w - att->q.x*qd.x - att->q.y*qd.y - att->q.z*qd.z;
    qn.x = att->q.w*qd.x + att->q.x*qd.w + att->q.y*qd.z - att->q.z*qd.y;
    qn.y = att->q.w*qd.y - att->q.x*qd.z + att->q.y*qd.w + att->q.z*qd.x;
    qn.z = att->q.w*qd.z + att->q.x*qd.y - att->q.y*qd.x + att->q.z*qd.w;
    att->q = qn; quat_normalize(&att->q);
    quat_to_euler(&att->q, &att->roll_deg, &att->pitch_deg, &att->yaw_deg);
    att->roll_rate_dps = gx;
    att->pitch_rate_dps = gy;
    att->yaw_rate_dps = gz;
}

/* ===========================================================================
   VERTICAL KALMAN FILTER
   =========================================================================== */
typedef struct {
    float height_m;
    float velocity_mps;
    float p00, p01, p10, p11;
    float accel_noise;
    float baro_noise;
    float lidar_noise;
    bool initialized;
} altitude_kf_t;

static void kf_init(altitude_kf_t *kf, float height, float velocity) {
    kf->height_m = height;
    kf->velocity_mps = velocity;
    kf->p00 = 0.04f; kf->p01 = 0.0f; kf->p10 = 0.0f; kf->p11 = 0.1f;
    kf->accel_noise = 1.5f;
    kf->baro_noise = 0.0025f;
    kf->lidar_noise = 0.01f;
    kf->initialized = true;
}

static void kf_predict(altitude_kf_t *kf, float accel_up_mps2, float dt) {
    if (!kf->initialized) return;
    float a = accel_up_mps2; /* already gravity compensated */
    kf->height_m += kf->velocity_mps * dt + 0.5f * a * dt * dt;
    kf->velocity_mps += a * dt;
    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, q = kf->accel_noise*kf->accel_noise;
    kf->p00 = kf->p00 + dt*(kf->p01+kf->p10) + dt2*kf->p11 + 0.25f*dt4*q;
    kf->p01 = kf->p01 + dt*kf->p11 + 0.5f*dt3*q;
    kf->p10 = kf->p10 + dt*kf->p11 + 0.5f*dt3*q;
    kf->p11 = kf->p11 + dt2*q;
}

static void kf_update_baro(altitude_kf_t *kf, float height_m, float sigma_m) {
    if (!kf->initialized || sigma_m <= 0.0f) return;
    float r = sigma_m*sigma_m; if (r < 1e-6f) r = 1e-6f;
    float innovation = height_m - kf->height_m;
    float innovation_var = kf->p00 + r;
    float k0 = kf->p00/innovation_var;
    float k1 = kf->p10/innovation_var;
    kf->height_m += k0 * innovation;
    kf->velocity_mps += k1 * innovation;
    float z = 1.0f - k0;
    kf->p00 = z*z*kf->p00 + k0*k0*r;
    kf->p01 = z*(kf->p01 - k1*kf->p00) + k0*k1*r;
    kf->p10 = kf->p01;
    kf->p11 = kf->p11 - 2.0f*k1*kf->p01 + k1*k1*kf->p00 + k1*k1*r;
}

static void kf_update_lidar(altitude_kf_t *kf, float height_m, float sigma_m) {
    if (!kf->initialized || sigma_m <= 0.0f) return;
    float r = sigma_m*sigma_m; if (r < 1e-6f) r = 1e-6f;
    float innovation = height_m - kf->height_m;
    float innovation_var = kf->p00 + r;
    float k0 = kf->p00/innovation_var;
    float k1 = kf->p10/innovation_var;
    kf->height_m += k0 * innovation;
    kf->velocity_mps += k1 * innovation;
    float z = 1.0f - k0;
    kf->p00 = z*z*kf->p00 + k0*k0*r;
    kf->p01 = z*(kf->p01 - k1*kf->p00) + k0*k1*r;
    kf->p10 = kf->p01;
    kf->p11 = kf->p11 - 2.0f*k1*kf->p01 + k1*k1*kf->p00 + k1*k1*r;
}

/* ===========================================================================
   MOTOR MIXER (X-frame: FL, FR, RR, RL)
   =========================================================================== */
typedef struct {
    float last_cmd[4];
    float max_slew_per_second;
} motor_mixer_t;

static void motor_mixer_init(motor_mixer_t *m) {
    for (int i = 0; i < 4; ++i) m->last_cmd[i] = 0.0f;
    m->max_slew_per_second = g_max_slew_per_second;
}

static void motor_mixer_mix(motor_mixer_t *m,
                            float collective,
                            float roll_torque, float pitch_torque, float yaw_torque,
                            float ceiling, float idle_fraction, float dt,
                            float out[4]) {
    /* X-frame mixer: collective +/- roll +/- pitch +/- yaw */
    float corr[4] = {
        roll_torque - pitch_torque - yaw_torque,   /* FL */
       -roll_torque - pitch_torque + yaw_torque,   /* FR */
       -roll_torque + pitch_torque - yaw_torque,   /* RR */
        roll_torque + pitch_torque + yaw_torque     /* RL */
    };
    float low = corr[0], high = corr[0];
    for (int i = 1; i < 4; ++i) {
        if (corr[i] < low) low = corr[i];
        if (corr[i] > high) high = corr[i];
    }
    float span = ceiling - idle_fraction;
    float scale = (high - low > span) ? span / (high - low) : 1.0f;
    float allocated = fmaxf(idle_fraction, fminf(ceiling - high * scale, collective));
    allocated = fminf(allocated, ceiling - low * scale);

    float raw[4];
    for (int i = 0; i < 4; ++i) {
        raw[i] = fmaxf(idle_fraction, fminf(ceiling, allocated + scale * corr[i]));
    }

    /* Slew-rate limit to prevent violent motor acceleration */
    float max_delta = m->max_slew_per_second * dt;
    for (int i = 0; i < 4; ++i) {
        float delta = raw[i] - m->last_cmd[i];
        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;
        m->last_cmd[i] = fmaxf(idle_fraction, fminf(ceiling, m->last_cmd[i] + delta));
        out[i] = m->last_cmd[i];
    }
}

static motor_mixer_t mixer;

/* ===========================================================================
   PILOT COMMAND PROCESSING
   =========================================================================== */
typedef struct {
    bool valid;
    uint32_t timestamp_ms;
    uint8_t buttons;
    float roll;      /* -1..1 right stick X */
    float pitch;     /* -1..1 right stick Y */
    float yaw;       /* -1..1 left stick X */
    float throttle;  /* -1..1 left stick Y */
    float rt;        /* 0..1 right trigger */
    float lt;        /* 0..1 left trigger */
} pilot_command_t;

static float deadzonef(float v, float zone) {
    if (fabsf(v) < zone) return 0.0f;
    return v > 0.0f ? (v - zone) / (1.0f - zone) : (v + zone) / (1.0f - zone);
}

static float smooth_state(float *state, float target, float dt, float tau) {
    float alpha = dt / (tau + dt);
    *state += alpha * (target - *state);
    return *state;
}

static bool pilot_button_pressed(uint8_t buttons, uint8_t button, bool *prev_state) {
    bool pressed = (buttons & button) != 0;
    bool changed = pressed && !*prev_state;
    *prev_state = pressed;
    return pressed;
}

/* ===========================================================================
   FLIGHT CONTROL STEP
   =========================================================================== */
static control_output_t flight_control_step(const sensor_sample_t *s,
                                             const pilot_command_t *cmd,
                                             float dt) {
    control_output_t out = { {0.0f,0.0f,0.0f,0.0f}, false, false };

    /* Safety: if not armed, all motors zero */
    if (!armed) {
        out.armed = false;
        return out;
    }

    /* Failsafe: if failsafe active, zero motors */
    if (failsafe) {
        out.armed = false;
        out.sensor_fault = true;
        return out;
    }

    /* =========================================================================
       STEP 1: Apply calibration to sensor readings
       ========================================================================= */
    float accel_x = s->accel_x_g;
    float accel_y = s->accel_y_g;
    float accel_z = s->accel_z_g;
    float gyro_x = s->gyro_x_dps;
    float gyro_y = s->gyro_y_dps;
    float gyro_z = s->gyro_z_dps;
    float mag_x = s->magnetometer_x_uT;
    float mag_y = s->magnetometer_y_uT;
    float mag_z = s->magnetometer_z_uT;

    if (calibration_loaded) {
        /* Apply accelerometer calibration: calibrated = scale * (raw - bias) */
        float ax = accel_x, ay = accel_y, az = accel_z;
        accel_x = cal_accel_scale[0][0]*(ax - cal_accel_bias[0])
                + cal_accel_scale[0][1]*(ay - cal_accel_bias[1])
                + cal_accel_scale[0][2]*(az - cal_accel_bias[2]);
        accel_y = cal_accel_scale[1][0]*(ax - cal_accel_bias[0])
                + cal_accel_scale[1][1]*(ay - cal_accel_bias[1])
                + cal_accel_scale[1][2]*(az - cal_accel_bias[2]);
        accel_z = cal_accel_scale[2][0]*(ax - cal_accel_bias[0])
                + cal_accel_scale[2][1]*(ay - cal_accel_bias[1])
                + cal_accel_scale[2][2]*(az - cal_accel_bias[2]);

        /* Apply gyro calibration: calibrated = raw - bias */
        gyro_x -= cal_gyro_bias[0];
        gyro_y -= cal_gyro_bias[1];
        gyro_z -= cal_gyro_bias[2];

        /* Apply magnetometer calibration: calibrated = soft_iron * (raw - hard_iron) */
        float mx_c = mag_x - cal_mag_hard_iron[0];
        float my_c = mag_y - cal_mag_hard_iron[1];
        float mz_c = mag_z - cal_mag_hard_iron[2];
        mag_x = cal_mag_soft_iron[0][0]*mx_c + cal_mag_soft_iron[0][1]*my_c + cal_mag_soft_iron[0][2]*mz_c;
        mag_y = cal_mag_soft_iron[1][0]*mx_c + cal_mag_soft_iron[1][1]*my_c + cal_mag_soft_iron[1][2]*mz_c;
        mag_z = cal_mag_soft_iron[2][0]*mx_c + cal_mag_soft_iron[2][1]*my_c + cal_mag_soft_iron[2][2]*mz_c;
    }

    /* =========================================================================
       STEP 2: Attitude estimation
       ========================================================================= */
    if (!attitude.initialized) {
        attitude_init(&attitude, accel_x, accel_y, accel_z);
    }
    attitude_update(&attitude, gyro_x, gyro_y, gyro_z,
                    mag_x, mag_y, mag_z,
                    s->magnetometer_valid && s->magnetometer_fresh, dt);

    /* =========================================================================
       STEP 2: Vertical Kalman filter
       ========================================================================= */
    if (!alt_kf_initialized) {
        kf_init(&alt_kf, s->barometer_height_m, 0.0f);
        alt_kf_initialized = true;
    }
    kf_predict(&alt_kf, s->vertical_acceleration_mps2, dt);
    if (s->barometer_valid && s->barometer_updated) {
        float baro_h = s->barometer_height_m - baro_zero_offset_m;
        kf_update_baro(&alt_kf, baro_h, 1.0f);
    }
    /* LiDAR optional - not wired yet */

    /* =========================================================================
       STEP 3: Pilot command processing
       ========================================================================= */
    if (!cmd->valid) {
        out.armed = false;
        return out;
    }

    /* Arm/disarm state machine:
       - Hold LB + RB, press Y (NORTH) for 500 ms to arm
       - Press A (SOUTH) to emergency disarm */
    static bool arm_seq = false;
    static uint32_t arm_hold_ms = 0;
    static bool last_north = false;
    static bool last_south = false;

    bool north_pressed = (cmd->buttons & QCOPTER_BUTTON_NORTH) != 0;
    bool south_pressed = (cmd->buttons & QCOPTER_BUTTON_SOUTH) != 0;
    bool north_rising = north_pressed && !last_north;
    bool south_rising = south_pressed && !last_south;
    last_north = north_pressed;
    last_south = south_pressed;

    if (!armed) {
        bool lt_rb_held = (cmd->buttons & QCOPTER_BUTTON_LB) != 0
                       && (cmd->buttons & QCOPTER_BUTTON_RB) != 0;
        if (lt_rb_held) {
            if (!arm_seq) {
                arm_seq = true;
                arm_hold_ms = cmd->timestamp_ms;
            }
            if (arm_seq && north_rising
                && (cmd->timestamp_ms - arm_hold_ms >= 500u)) {
                armed = true;
                arm_seq = false;
                /* Reset control state on arm */
                motor_mixer_init(&mixer);
                pid_reset(&pid_roll_angle);
                pid_reset(&pid_pitch_angle);
                pid_reset(&pid_roll_rate);
                pid_reset(&pid_pitch_rate);
                pid_reset(&pid_yaw_rate);
                pid_reset(&pid_altitude);
                pid_reset(&pid_vertical_speed);
                attitude.initialized = false;
                alt_kf_initialized = false;
                altitude_ref_initialized = false;
                lt_reduce_state = 0.0f;
                rt_boost_state = 0.0f;
                altitude_ref_m = alt_kf.height_m;
                altitude_ref_initialized = true;
            }
        } else {
            arm_seq = false;
            arm_hold_ms = 0;
        }
    } else {
        bool disarm_cmd = south_rising;
        bool controller_lost = !controller_connected;
        if (disarm_cmd || controller_lost) {
            armed = false;
            failsafe = controller_lost;
            motor_mixer_init(&mixer);
            pid_reset(&pid_roll_angle);
            pid_reset(&pid_pitch_angle);
            pid_reset(&pid_roll_rate);
            pid_reset(&pid_pitch_rate);
            pid_reset(&pid_yaw_rate);
            pid_reset(&pid_altitude);
            pid_reset(&pid_vertical_speed);
            attitude.initialized = false;
            alt_kf_initialized = false;
            altitude_ref_initialized = false;
            lt_reduce_state = 0.0f;
            rt_boost_state = 0.0f;
        }
    }

    if (!armed) {
        out.armed = false;
        return out;
    }

    /* =========================================================================
       STEP 4: Control computation
       ========================================================================= */

    /* Roll/pitch angle setpoints from right stick */
    float roll_setpoint = cmd->roll * FC_MAX_ROLL_DEG;
    float pitch_setpoint = cmd->pitch * FC_MAX_PITCH_DEG;

    /* Angle P controllers -> rate setpoints */
    float roll_rate_setpoint = pid_update(&pid_roll_angle, roll_setpoint,
                                          attitude.roll_deg, dt);
    float pitch_rate_setpoint = pid_update(&pid_pitch_angle, pitch_setpoint,
                                           attitude.pitch_deg, dt);

    /* Inner rate PIDs -> torque demands */
    float roll_torque = pid_update(&pid_roll_rate, roll_rate_setpoint,
                                   attitude.roll_rate_dps, dt);
    float pitch_torque = pid_update(&pid_pitch_rate, pitch_rate_setpoint,
                                    attitude.pitch_rate_dps, dt);

    /* Yaw rate PID: left stick X -> yaw rate demand */
    float yaw_rate_setpoint = cmd->yaw * FC_MAX_YAW_RATE_DPS;
    float yaw_torque = pid_update(&pid_yaw_rate, yaw_rate_setpoint,
                                  attitude.yaw_rate_dps, dt);

    /* Vertical speed demand from left stick Y */
    float vs_setpoint = cmd->throttle * FC_MAX_VERTICAL_SPEED_MPS;

    /* RT boost / LT reduce: trim vertical speed */
    float rt_boost = smooth_state(&rt_boost_state, cmd->rt, dt, 0.2f);
    float lt_reduce = smooth_state(&lt_reduce_state, cmd->lt, dt, 0.2f);
    float trigger_trim = (rt_boost - lt_reduce) * 1.0f;
    vs_setpoint += trigger_trim;
    if (vs_setpoint > FC_MAX_CLIMB_MPS) vs_setpoint = FC_MAX_CLIMB_MPS;
    if (vs_setpoint < -FC_MAX_DESCENT_MPS) vs_setpoint = -FC_MAX_DESCENT_MPS;

    /* Vertical speed PID -> vertical acceleration command */
    float accel_cmd = pid_update(&pid_vertical_speed, vs_setpoint,
                                 alt_kf.velocity_mps, dt);

    /* Altitude hold: when throttle near center, hold altitude */
    bool altitude_hold_active = fabsf(cmd->throttle) < 0.1f;
    if (altitude_hold_active && altitude_ref_initialized) {
        float alt_correction = pid_update(&pid_altitude, altitude_ref_m,
                                          alt_kf.height_m, dt);
        accel_cmd += alt_correction;
    } else {
        if (!altitude_ref_initialized) {
            altitude_ref_m = alt_kf.height_m;
            altitude_ref_initialized = true;
        }
        altitude_ref_m += vs_setpoint * dt;
    }

    /* Convert vertical acceleration command to collective thrust */
    float cos_roll = cosf(attitude.roll_deg * FC_DEG_TO_RAD);
    float cos_pitch = cosf(attitude.pitch_deg * FC_DEG_TO_RAD);
    float tilt_factor = fmaxf(0.5f, cos_roll * cos_pitch);
    float collective = (hover_collective + accel_cmd * 0.05f) / tilt_factor;
    collective = fmaxf(FC_ARMED_IDLE_FRACTION,
                       fminf(FC_BOOST_MOTOR_CEILING, collective));

    /* Motor mixing */
    float ceiling = FC_NORMAL_MOTOR_CEILING
                  + (FC_BOOST_MOTOR_CEILING - FC_NORMAL_MOTOR_CEILING) * rt_boost;
    motor_mixer_mix(&mixer, collective,
                    roll_torque, pitch_torque, yaw_torque,
                    ceiling, FC_ARMED_IDLE_FRACTION, dt,
                    out.motor);

    out.armed = true;
    out.sensor_fault = failsafe;
    return out;
}

/* ===========================================================================
   PUBLIC API
   =========================================================================== */

void fc_init(void) {
    motor_mixer_init(&mixer);
    pid_reset(&pid_roll_angle);
    pid_reset(&pid_pitch_angle);
    pid_reset(&pid_roll_rate);
    pid_reset(&pid_pitch_rate);
    pid_reset(&pid_yaw_rate);
    pid_reset(&pid_altitude);
    pid_reset(&pid_vertical_speed);
    attitude.initialized = false;
    alt_kf_initialized = false;
    altitude_ref_initialized = false;
    armed = false;
    failsafe = false;
    controller_connected = false;
    last_controller_ms = 0;
    hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;
    baro_zero_offset_m = 0.0f;
    baro_zeroed = false;
    lt_reduce_state = 0.0f;
    rt_boost_state = 0.0f;
    altitude_ref_m = 0.0f;
    altitude_ref_initialized = false;
}

static pilot_command_t pilot_cmd;
static bool pilot_cmd_valid = false;

void fc_apply_command(const qcopter_command_frame_t *frame, uint64_t now_ms) {
    if (!frame) return;
    controller_connected = true;
    last_controller_ms = now_ms;

    pilot_cmd.roll_deg = (float)frame->roll * 35.0f / QCOPTER_AXIS_SCALE;
    pilot_cmd.pitch_deg = (float)frame->pitch * 35.0f / QCOPTER_AXIS_SCALE;
    pilot_cmd.yaw_rate_dps = (float)frame->yaw_rate * 180.0f / QCOPTER_AXIS_SCALE;
    pilot_cmd.throttle = (float)frame->throttle / QCOPTER_THROTTLE_SCALE;
    pilot_cmd.rt = (float)frame->rt / QCOPTER_TRIGGER_SCALE;
    pilot_cmd.lt = (float)frame->lt / QCOPTER_TRIGGER_SCALE;
    pilot_cmd.valid = true;
    pilot_cmd.timestamp_ms = now_ms;
    pilot_cmd.buttons = frame->buttons;
    pilot_cmd_valid = true;
}

control_output_t fc_get_output(const sensor_sample_t *s,
                               const pilot_command_t *cmd,
                               float dt) {
    /* If no external command provided, use the stored UART command */
    const pilot_command_t *c = cmd;
    if (c == NULL && pilot_cmd_valid) {
        c = &pilot_cmd;
    }
    if (c == NULL) {
        control_output_t out = { {0.0f,0.0f,0.0f,0.0f}, false, false };
        if (armed) {
            armed = false;
            failsafe = true;
            controller_connected = false;
            motor_mixer_init(&mixer);
            out.sensor_fault = true;
        }
        return out;
    }

    if (!control_clock_initialized) {
        control_clock_ms = now_ms;
        control_clock_initialized = true;
    } else {
        const uint32_t clock_now_ms = (uint32_t)control_clock_ms;
        const uint32_t command_ahead_ms = c->timestamp_ms - clock_now_ms;
        if (command_ahead_ms != 0u && command_ahead_ms < 0x80000000u)
            control_clock_ms += command_ahead_ms;
    }
    if (isfinite(dt) && dt > 0.0f && dt <= FC_MAX_CONTROL_DT)
        control_clock_ms += (uint64_t)lroundf(dt * 1000.0f);

    const uint32_t command_age_ms = (uint32_t)control_clock_ms - c->timestamp_ms;
    if (!c->valid || command_age_ms > FC_RADIO_TIMEOUT_US / 1000u) {
        control_output_t out = { {0.0f,0.0f,0.0f,0.0f}, false, true };
        armed = false;
        failsafe = true;
        controller_connected = false;
        pilot_cmd_valid = false;
        motor_mixer_init(&mixer);
        return out;
    }
    return flight_control_step(s, c, dt);
}

bool fc_is_armed(void) {
    return armed;
}

bool fc_is_failsafe(void) {
    return failsafe;
}

bool fc_controller_connected(void) {
    return controller_connected;
}

void fc_apply_calibration(const float accel_bias[3],
                          const float accel_scale[3][3],
                          const float gyro_bias[3],
                          const float mag_hard_iron[3],
                          const float mag_soft_iron[3][3]) {
    if (accel_bias == NULL || accel_scale == NULL || gyro_bias == NULL
        || mag_hard_iron == NULL || mag_soft_iron == NULL) return;

    for (int i = 0; i < 3; ++i) {
        if (!isfinite(accel_bias[i]) || !isfinite(gyro_bias[i])
            || !isfinite(mag_hard_iron[i])) return;
        for (int j = 0; j < 3; ++j) {
            if (!isfinite(accel_scale[i][j]) || !isfinite(mag_soft_iron[i][j]))
                return;
        }
    }

    const float accel_det =
          accel_scale[0][0] * (accel_scale[1][1] * accel_scale[2][2]
                              - accel_scale[1][2] * accel_scale[2][1])
        - accel_scale[0][1] * (accel_scale[1][0] * accel_scale[2][2]
                              - accel_scale[1][2] * accel_scale[2][0])
        + accel_scale[0][2] * (accel_scale[1][0] * accel_scale[2][1]
                              - accel_scale[1][1] * accel_scale[2][0]);
    const float mag_det =
          mag_soft_iron[0][0] * (mag_soft_iron[1][1] * mag_soft_iron[2][2]
                                - mag_soft_iron[1][2] * mag_soft_iron[2][1])
        - mag_soft_iron[0][1] * (mag_soft_iron[1][0] * mag_soft_iron[2][2]
                                - mag_soft_iron[1][2] * mag_soft_iron[2][0])
        + mag_soft_iron[0][2] * (mag_soft_iron[1][0] * mag_soft_iron[2][1]
                                - mag_soft_iron[1][1] * mag_soft_iron[2][0]);
    if (!isfinite(accel_det) || !isfinite(mag_det)
        || fabsf(accel_det) < 1.0e-6f || fabsf(mag_det) < 1.0e-6f)
        return;

    for (int i = 0; i < 3; ++i) {
        cal_accel_bias[i] = accel_bias[i];
        cal_gyro_bias[i] = gyro_bias[i];
        cal_mag_hard_iron[i] = mag_hard_iron[i];
        for (int j = 0; j < 3; ++j) {
            cal_accel_scale[i][j] = accel_scale[i][j];
            cal_mag_soft_iron[i][j] = mag_soft_iron[i][j];
        }
    }
    calibration_loaded = true;
}

float fc_get_hover_collective(void) {
    return hover_collective;
}

void fc_set_hover_collective(float collective) {
    if (!isfinite(collective)
        || collective < FC_ARMED_IDLE_FRACTION
        || collective > FC_NORMAL_MOTOR_CEILING) {
        hover_collective_valid = false;
        hover_collective = 0.0f;
        armed = false;
        failsafe = true;
        motor_mixer_init(&mixer);
        return;
    }
    hover_collective = collective;
    hover_collective_valid = true;
}

float fc_get_baro_zero_offset(void) {
    return baro_zero_offset_m;
}

float fc_get_altitude_ref(void) {
    return altitude_ref_m;
}
