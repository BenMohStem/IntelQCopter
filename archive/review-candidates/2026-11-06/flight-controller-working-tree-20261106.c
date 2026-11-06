/*
 * QCopter flight-controller foundation.
 *
 * This file contains hardware-independent estimation and control logic so the
 * same C implementation is used in the native Gazebo host simulation.
 * Hardware sensor and ESC drivers are intentionally not enabled: board pins,
 * ESC protocol, frame orientation, and motor order have not been confirmed.
 * Therefore this firmware remains disarmed and never produces motor outputs.
 *
 * Control architecture (per axis, ESC-command fractions 0..1 only):
 *   Roll: angle PID -> rate demand -> rate PID -> torque demand
 *   Pitch: angle PID -> rate demand -> rate PID -> torque demand
 *   Yaw: rate PID directly (rate demand -> torque demand)
 *   Altitude: position PID -> vertical speed demand -> vertical speed PID
 *            -> vertical acceleration demand -> collective thrust
 *   Vertical speed: throttle stick -> vertical speed command
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
    QCOPTER_BUTTON_NORTH = 1u << 2,
    QCOPTER_BUTTON_EAST = 1u << 3,
    QCOPTER_BUTTON_SOUTH = 1u << 4,
    QCOPTER_BUTTON_WEST = 1u << 5
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
   MPU6050 CONFIGURATION TABLE (configurable from calibration)
   =========================================================================== */
#define MPU_GYRO_FS_500 0x08   /* +/-500 deg/s, 65.5 LSB/(deg/s) */
#define MPU_ACCEL_FS_8G 0x10   /* +/-8 g, 4096 LSB/g */
#define MPU_DLPF_CFG_1  0x01   /* 188 Hz gyro, 184 Hz accel */
#define MPU_SAMPLE_DIV_1 0x01  /* 1 kHz / 2 = 500 Hz */
#define MPU_GYRO_SENSITIVITY 65.5f   /* LSB per degree/s for 500 dps */
#define MPU_ACCEL_SENSITIVITY 4096.0f /* LSB per g for +/-8 g */

/* QMC5883P: CTRL1=0x3D, CTRL2=0x08, XYZ_SIGN=0x06, +/-8 G = 3750 LSB/G */
#define MAG_COUNTS_PER_UT 3750.0f
#define MAG_UT_PER_COUNT (1.0f / 3750.0f)

/* ===========================================================================
   CONTROL LIMITS (ESC-command fractions 0..1, not RPM/thrust)
   =========================================================================== */
#define FC_ARMED_IDLE_FRACTION 0.10f   /* 10% armed idle without throttle */
#define FC_NORMAL_MOTOR_CEILING 0.45f  /* 45% maximum normal command */
#define FC_BOOST_MOTOR_CEILING 0.65f   /* 65% maximum boost command */
#define FC_MAX_VERTICAL_SPEED_MPS 2.0f
#define FC_MAX_CLIMB_MPS 1.5f
#define FC_MAX_DESCENT_MPS 1.0f
#define FC_MAX_ROLL_DEG 35.0f
#define FC_MAX_PITCH_DEG 35.0f
#define FC_MAX_YAW_RATE_DPS 180.0f
#define FC_MAX_COLLECTIVE_ACCEL_MPS2 3.0f   /* max vertical accel from collective */
#define FC_LOOP_HZ 500.0f
#define FC_LOOP_PERIOD_US 2000u
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f
#define FC_HOVER_THROTTLE_PLACEHOLDER 0.50f
#define FC_RADIO_TIMEOUT_US 150000u

/* GRAVITY */
#define FC_GRAVITY_MPS2 9.80665f

/* ===========================================================================
   WIRE PROTOCOL v1
   =========================================================================== */
static inline uint16_t qcopter_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void qcopter_put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)(value >> 8);
}

static inline uint16_t qcopter_get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline bool qcopter_frame_values_valid(const qcopter_command_frame_t *f) {
    return f != NULL
        && f->roll >= -QCOPTER_AXIS_SCALE && f->roll <= QCOPTER_AXIS_SCALE
        && f->pitch >= -QCOPTER_AXIS_SCALE && f->pitch <= QCOPTER_AXIS_SCALE
        && f->yaw_rate >= -QCOPTER_AXIS_SCALE && f->yaw_rate <= QCOPTER_AXIS_SCALE
        && f->throttle <= QCOPTER_THROTTLE_SCALE
        && f->lt <= QCOPTER_TRIGGER_SCALE && f->rt <= QCOPTER_TRIGGER_SCALE
        && (f->buttons & (uint16_t)~0x003Fu) == 0u;
}

static inline bool qcopter_encode_frame(const qcopter_command_frame_t *f,
                                        uint8_t out[QCOPTER_FRAME_SIZE]) {
    if (out == NULL || !qcopter_frame_values_valid(f)) return false;
    out[0] = 0x51u;
    out[1] = 0x43u;
    out[2] = QCOPTER_PROTOCOL_VERSION;
    out[3] = 0u;
    qcopter_put_u16(&out[4], f->sequence);
    qcopter_put_u16(&out[6], (uint16_t)f->roll);
    qcopter_put_u16(&out[8], (uint16_t)f->pitch);
    qcopter_put_u16(&out[10], (uint16_t)f->yaw_rate);
    qcopter_put_u16(&out[12], f->throttle);
    qcopter_put_u16(&out[14], f->lt);
    qcopter_put_u16(&out[16], f->rt);
    qcopter_put_u16(&out[18], f->buttons);
    qcopter_put_u16(&out[20], qcopter_crc16(out, 20u));
    return true;
}

static inline bool qcopter_decode_frame(const uint8_t *data, size_t length,
                                        qcopter_command_frame_t *out) {
    if (data == NULL || out == NULL || length != QCOPTER_FRAME_SIZE
        || data[0] != 0x51u || data[1] != 0x43u
        || data[2] != QCOPTER_PROTOCOL_VERSION || data[3] != 0u
        || qcopter_get_u16(&data[20]) != qcopter_crc16(data, 20u))
        return false;
    qcopter_command_frame_t f = {
        .sequence = qcopter_get_u16(&data[4]),
        .roll = (int16_t)qcopter_get_u16(&data[6]),
        .pitch = (int16_t)qcopter_get_u16(&data[8]),
        .yaw_rate = (int16_t)qcopter_get_u16(&data[10]),
        .throttle = qcopter_get_u16(&data[12]),
        .lt = qcopter_get_u16(&data[14]),
        .rt = qcopter_get_u16(&data[16]),
        .buttons = qcopter_get_u16(&data[18])
    };
    if (!qcopter_frame_values_valid(&f)) return false;
    *out = f;
    return true;
}

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
    const float candidate_i = clampf(pid->integrator + error * dt,
                                     -pid->integrator_limit,
                                     pid->integrator_limit);
    const float raw = pid->kp * error + pid->ki * candidate_i
                    - pid->kd * derivative;
    const float output = clampf(raw, -pid->output_limit, pid->output_limit);
    if (raw == output || (raw > output && error < 0.0f)
        || (raw < output && error > 0.0f)) {
        pid->integrator = candidate_i;
    }
    pid->previous_measurement = measurement;
    pid->initialized = true;
    return output;
}

static inline float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

/* ===========================================================================
   ATTITUDE ESTIMATOR (complementary filter quaternion)
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

    /* Accelerometer correction */
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

    /* Re-integrate */
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
    float a = accel_up_mps2; /* already gravity-compensated */
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
   GLOBAL STATE
   =========================================================================== */
typedef struct {
    float roll_deg, pitch_deg, yaw_deg;
    float roll_rate_dps, pitch_rate_dps, yaw_rate_dps;
    float accel_x_g, accel_y_g, accel_z_g;
    float baro_height_m;
    bool imu_valid, imu_calibrated;
    bool baro_valid, baro_updated;
    bool mag_valid, mag_fresh;
    float mag_x_uT, mag_y_uT, mag_z_uT;
} sensor_sample_t;

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

typedef struct {
    float motor[4];   /* FL, FR, RR, RL: ESC command fractions 0..1 */
    bool armed;
    bool sensor_fault;
} control_output_t;

/* Static state */
static attitude_t attitude = {0};
static altitude_kf_t alt_kf = {0};
static bool alt_kf_initialized = false;

/* PIDs */
static pid_t pid_roll_angle = {.kp = 0.65f, .ki = 0.02f, .kd = 0.03f,
    .integrator_limit = 2.0f, .output_limit = 0.35f};
static pid_t pid_pitch_angle = {.kp = 0.65f, .ki = 0.02f, .kd = 0.03f,
    .integrator_limit = 2.0f, .output_limit = 0.35f};
static pid_t pid_roll_rate = {.kp = 0.078254f, .ki = 0.039127f, .kd = 0.00005869f,
    .integrator_limit = 15.0f, .output_limit = 0.35f};
static pid_t pid_pitch_rate = {.kp = 0.083150f, .ki = 0.041575f, .kd = 0.00006236f,
    .integrator_limit = 15.0f, .output_limit = 0.35f};
static pid_t pid_yaw_rate = {.kp = 0.087424f, .ki = 0.027607f, .kd = 0.00011503f,
    .integrator_limit = 8.0f, .output_limit = 0.35f};
static pid_t pid_altitude = {.kp = 0.40f, .ki = 0.005f, .kd = 0.015f,
    .integrator_limit = 2.0f, .output_limit = 0.35f};
static pid_t pid_vertical_speed = {.kp = 1.0f, .ki = 0.06f, .kd = 0.025f,
    .integrator_limit = 5.0f, .output_limit = 1.0f};

/* State */
static bool armed = false;
static bool failsafe = false;
static bool controller_connected = false;
static uint32_t last_controller_ms = 0;
static float hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;
static float baro_zero_offset_m = 0.0f;
static bool baro_zeroed = false;
static float lt_reduce_state = 0.0f;
static float rt_boost_state = 0.0f;
static float altitude_ref_m = 0.0f;
static bool altitude_ref_initialized = false;

/* ===========================================================================
   MOTOR MIXER (X-frame: FL, FR, RR, RL)
   =========================================================================== */
typedef struct {
    float last_cmd[4];
    float max_slew_per_second;
} motor_mixer_t;

static void motor_mixer_init(motor_mixer_t *m) {
    for (int i = 0; i < 4; ++i) m->last_cmd[i] = 0.0f;
    m->max_slew_per_second = 10.0f;
}

static void motor_mixer_mix(motor_mixer_t *m,
                            float collective,
                            float roll_torque, float pitch_torque, float yaw_torque,
                            float ceiling, float idle_fraction, float dt,
                            float out[4]) {
    /* X-frame mixer */
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

    /* Slew-rate limit */
    float max_delta = m->max_slew_per_second * dt;
    for (int i = 0; i < 4; ++i) {
        float delta = raw[i] - m->last_cmd[i];
        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;
        m->last_cmd[i] = fmaxf(idle_fraction, fminf(ceiling, m->last_cmd[i] + delta));
        out[i] = m->last_cmd[i];
    }
}

static void motor_mixer_reset(motor_mixer_t *m) {
    for (int i = 0; i < 4; ++i) m->last_cmd[i] = 0.0f;
}

static motor_mixer_t mixer;

/* ===========================================================================
   CALIBRATION APPLICATION
   =========================================================================== */
static bool cal_loaded = false;
static float cal_accel_bias[3] = {0,0,0};
static float cal_accel_scale[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
static float cal_gyro_bias[3] = {0,0,0};
static float cal_mag_hard_iron[3] = {0,0,0};
static float cal_mag_soft_iron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

static bool apply_accel_cal(const float raw[3], float out[3]) {
    if (!cal_loaded) return false;
    for (int i = 0; i < 3; ++i)
        out[i] = cal_accel_scale[i][0]*(raw[0]-cal_accel_bias[0])
               + cal_accel_scale[i][1]*(raw[1]-cal_accel_bias[1])
               + cal_accel_scale[i][2]*(raw[2]-cal_accel_bias[2]);
    return true;
}

static bool apply_gyro_cal(const float raw[3], float out[3]) {
    if (!cal_loaded) return false;
    for (int i = 0; i < 3; ++i)
        out[i] = raw[i] - cal_gyro_bias[i];
    return true;
}

static bool apply_mag_cal(const float raw_uT[3], float out_uT[3]) {
    if (!cal_loaded) return false;
    float centered[3] = {
        raw_uT[0] - cal_mag_hard_iron[0],
        raw_uT[1] - cal_mag_hard_iron[1],
        raw_uT[2] - cal_mag_hard_iron[2]
    };
    for (int i = 0; i < 3; ++i)
        out_uT[i] = cal_mag_soft_iron[i][0]*centered[0]
                  + cal_mag_soft_iron[i][1]*centered[1]
                  + cal_mag_soft_iron[i][2]*centered[2];
    return true;
}

/* ===========================================================================
   PILOT COMMAND PROCESSING
   =========================================================================== */
static bool pilot_button_pressed(uint16_t buttons, uint8_t button, bool *prev_state) {
    bool pressed = (buttons & button) != 0;
    bool changed = pressed && !*prev_state;
    *prev_state = pressed;
    return pressed;
}

static float deadzonef(float v, float zone) {
    if (fabsf(v) < zone) return 0.0f;
    return v > 0.0f ? (v - zone) / (1.0f - zone) : (v + zone) / (1.0f - zone);
}

static float smooth_state(float *state, float target, float dt, float tau) {
    float alpha = dt / (tau + dt);
    *state += alpha * (target - *state);
    return *state;
}

/* ===========================================================================
   FLIGHT CONTROL STEP
   =========================================================================== */
static control_output_t flight_control_step(const sensor_sample_t *s,
                                             const pilot_command_t *cmd,
                                             float dt) {
    control_output_t out = { {0.0f,0.0f,0.0f,0.0f}, false, false };

    /* Safety: if not armed, always zero motors */
    if (!armed) {
        out.armed = false;
        return out;
    }

    /* Failsafe: if not armed or failsafe, zero motors */
    if (!armed || failsafe) {
        out.armed = false;
        out.sensor_fault = true;
        return out;
    }

    /* =========================================================================
       STEP 1: Attitude estimation
       ========================================================================= */
    if (!attitude.initialized) {
        attitude_init(&attitude, s->accel_x_g, s->accel_y_g, s->accel_z_g);
    }
    attitude_update(&attitude, s->gyro_x_dps, s->gyro_y_dps, s->gyro_z_dps,
                    s->mag_x_uT, s->mag_y_uT, s->mag_z_uT,
                    s->mag_valid && s->mag_fresh, dt);

    /* =========================================================================
       STEP 2: Altitude estimation (vertical Kalman filter)
       ========================================================================= */
    if (!alt_kf_initialized) {
        kf_init(&alt_kf, s->barometer_height_m, 0.0f);
        alt_kf_initialized = true;
    }
    kf_predict(&alt_kf, s->vertical_acceleration_mps2, dt);
    if (s->barometer_valid && s->barometer_updated) {
        float baro_h = s->barometer_height_m - baro_zero_offset_m;
        kf_update_baro(&alt_kf, baro_h, 1.0f); /* sigma 1.0 m */
    }
    /* LiDAR optional - not wired yet */

    /* =========================================================================
       STEP 3: Pilot command processing
       ========================================================================= */
    if (!cmd->valid) {
        /* No valid command: hold disarmed */
        out.armed = false;
        return out;
    }

    /* Arm/disarm state machine:
       - Hold LB + RB, press Y (NORTH) for 500 ms to arm
       * Press A (SOUTH) to emergency disarm
       */
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
        /* Disarmed: check arming sequence */
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
        /* Armed: check disarm */
        bool disarm_cmd = south_rising; /* A button = emergency disarm */
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

    /* If not armed after all this, return zeros */
    if (!armed) {
        out.armed = false;
        return out;
    }

    /* =========================================================================
       STEP 4: Control computation
       ========================================================================= */

    /* --- Roll/pitch angle demands from right stick --- */
    float roll_setpoint_deg = cmd->roll * FC_MAX_ROLL_DEG;
    float pitch_setpoint_deg = cmd->pitch * FC_MAX_PITCH_DEG;

    /* --- Angle P controllers -> rate setpoints --- */
    float roll_rate_setpoint = pid_update(&pid_roll_angle, roll_setpoint_deg,
                                          attitude.roll_deg, dt);
    float pitch_rate_setpoint = pid_update(&pid_pitch_angle, pitch_setpoint_deg,
                                           attitude.pitch_deg, dt);

    /* --- Inner rate PIDs -> torque demands --- */
    float roll_torque = pid_update(&pid_roll_rate, roll_rate_setpoint,
                                   attitude.roll_rate_dps, dt);
    float pitch_torque = pid_update(&pid_pitch_rate, pitch_rate_setpoint,
                                    attitude.pitch_rate_dps, dt);

    /* --- Yaw rate PID: left stick X -> yaw rate demand --- */
    float yaw_rate_setpoint = cmd->yaw * FC_MAX_YAW_RATE_DPS;
    float yaw_torque = pid_update(&pid_yaw_rate, yaw_rate_setpoint,
                                  attitude.yaw_rate_dps, dt);

    /* --- Vertical control --- */
    /* Vertical speed demand from left stick Y (-1..1) */
    float vs_setpoint = cmd->throttle * FC_MAX_VERTICAL_SPEED_MPS;

    /* RT boost / LT reduce: trim vertical speed */
    float rt_boost = smooth_state(&rt_boost_state, cmd->rt, dt, 0.2f);
    float lt_reduce = smooth_state(&lt_reduce_state, cmd->lt, dt, 0.2f);
    float trigger_trim = (rt_boost - lt_reduce) * 1.0f; /* +/- 1 m/s trim */
    vs_setpoint += trigger_trim;
    if (vs_setpoint > FC_MAX_CLIMB_MPS) vs_setpoint = FC_MAX_CLIMB_MPS;
    if (vs_setpoint < -FC_MAX_DESCENT_MPS) vs_setpoint = -FC_MAX_DESCENT_MPS;

    /* --- Vertical speed PID -> vertical acceleration command --- */
    float accel_cmd = pid_update(&pid_vertical_speed, vs_setpoint,
                                 alt_kf.velocity_mps, dt);

    /* --- Altitude hold: when throttle near center, hold altitude --- */
    bool altitude_hold_active = fabsf(cmd->throttle) < 0.1f;
    if (altitude_hold_active && altitude_ref_initialized) {
        float alt_correction = pid_update(&pid_altitude, altitude_ref_m,
                                          alt_kf.height_m, dt);
        accel_cmd += alt_correction;
    } else {
        /* Update altitude reference when throttle moves */
        if (!altitude_ref_initialized) {
            altitude_ref_m = alt_kf.height_m;
            altitude_ref_initialized = true;
        }
        altitude_ref_m += vs_setpoint * dt;
    }

    /* --- Convert vertical acceleration command to collective thrust --- */
    /* Tilt compensation: divide by cos(roll)*cos(pitch) */
    float cos_roll = cosf(attitude.roll_deg * FC_DEG_TO_RAD);
    float cos_pitch = cosf(attitude.pitch_deg * FC_DEG_TO_RAD);
    float tilt_factor = fmaxf(0.5f, cos_roll * cos_pitch);
    float collective = (hover_collective + accel_cmd * 0.05f) / tilt_factor;
    collective = fmaxf(FC_ARMED_IDLE_FRACTION,
                       fminf(FC_BOOST_MOTOR_CEILING, collective));

    /* --- Motor mixing --- */
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

/* Initialize flight controller (call once at boot, disarmed) */
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

/* Apply pilot command from UART frame */
void fc_apply_command(const qcopter_command_frame_t *frame, uint64_t now_ms) {
    if (!frame) return;
    controller_connected = true;
    last_controller_ms = now_ms;

    pilot_command_t cmd = {0};
    cmd.roll_deg = (float)frame->roll * 35.0f / QCOPTER_AXIS_SCALE;
    cmd.pitch_deg = (float)frame->pitch * 35.0f / QCOPTER_AXIS_SCALE;
    cmd.yaw_rate_dps = (float)frame->yaw_rate * 180.0f / QCOPTER_AXIS_SCALE;
    cmd.throttle = (float)frame->throttle / QCOPTER_THROTTLE_SCALE;
    cmd.boost = (float)frame->rt / QCOPTER_TRIGGER_SCALE;
    cmd.lt = (float)frame->lt / QCOPTER_TRIGGER_SCALE;
    cmd.valid = true;
    cmd.timestamp_ms = now_ms;
    cmd.buttons = frame->buttons;

    /* Check arming buttons */
    bool lb_pressed = (frame->buttons & QCOPTER_BUTTON_LB) != 0;
    bool rb_pressed = (frame->buttons & QCOPTER_BUTTON_RB) != 0;
    bool north_pressed = (frame->buttons & QCOPTER_BUTTON_NORTH) != 0;
    bool south_pressed = (frame->buttons & QCOPTER_BUTTON_SOUTH) != 0;

    /* Update arm state (also done inside flight_control_step, but
       we keep a local copy here for the command structure) */
    cmd.arm_request = false; /* armed via hold LB+RB + press Y */

    /* Store command for flight_control_step */
    /* NOTE: flight_control_step takes cmd by pointer; we need to store
       the last command somewhere. For now, flight_control_step is called
       with the same cmd each loop. */
}

/* Get the current control output */
control_output_t fc_get_output(const sensor_sample_t *s,
                               const pilot_command_t *cmd,
                               float dt) {
    return flight_control_step(s, cmd, dt);
}

/* Get the current armed state */
bool fc_is_armed(void) {
    return armed;
}

/* Check if failsafe is active */
bool fc_is_failsafe(void) {
    return failsafe;
}

/* Check controller connection status */
bool fc_controller_connected(void) {
    return controller_connected && controller_lost_ms == 0;
}

/* Check controller lost */
static uint64_t controller_lost_ms = 0;
bool fc_controller_lost(void) {
    return controller_lost_ms > 0;
}

/* Apply calibration data to flight controller */
void fc_apply_calibration(const float accel_bias[3],
                          const float accel_scale[3][3],
                          const float gyro_bias[3],
                          const float mag_hard_iron[3],
                          const float mag_soft_iron[3][3]) {
    for (int i = 0; i < 3; ++i) {
        cal_accel_bias[i] = accel_bias[i];
        cal_gyro_bias[i] = gyro_bias[i];
        cal_mag_hard_iron[i] = mag_hard_iron[i];
        for (int j = 0; j < 3; ++j) {
            cal_accel_scale[i][j] = accel_scale[i][j];
            cal_mag_soft_iron[i][j] = mag_soft_iron[i][j];
        }
    }
    cal_loaded = true;
}

/* Get current hover collective */
float fc_get_hover_collective(void) {
    return hover_collective;
}

/* Set hover collective (from calibration) */
void fc_set_hover_collective(float collective) {
    hover_collective = fmaxf(FC_ARMED_IDLE_FRACTION,
                             fminf(FC_BOOST_MOTOR_CEILING, collective));
}

/* Get current baro zero offset */
float fc_get_baro_zero_offset(void) {
    return baro_zero_offset_m;
}

/* Get current altitude reference */
float fc_get_altitude_ref(void) {
    return altitude_ref_m;
}