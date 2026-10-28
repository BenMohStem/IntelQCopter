/*
 * QCopter flight-controller foundation.
 *
 * This file contains hardware-independent estimation and control logic so the
 * same C implementation is used in the native Gazebo host simulation.
 * Hardware sensor and ESC drivers are intentionally not enabled: board pins,
 * ESC protocol, frame orientation, and motor order have not been confirmed.
 * Therefore this firmware remains disarmed and never produces motor outputs.
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

/* Valid for sequence gaps shorter than half the 16-bit sequence space. */
static inline bool qcopter_sequence_is_newer(uint16_t value, uint16_t previous) {
    return (int16_t)(value - previous) > 0;
}
/* END WIRE PROTOCOL v1 */


#ifndef QCOPTER_HOST_TEST
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#endif

/* Battery monitoring: types and implementation stay in this subproject file. */
#include <stdbool.h>

typedef enum {
    BATTERY_ACTION_NORMAL = 0,
    BATTERY_ACTION_WARN,
    BATTERY_ACTION_DISARM,
    BATTERY_ACTION_CONFIGURATION_FAULT,
    BATTERY_ACTION_SENSOR_FAULT
} battery_action_t;

typedef struct {
    float pack_voltage_v;
    float pack_current_a;       /* positive means battery discharging */
    float minimum_cell_voltage_v;
    float rested_soc_observation_percent; /* from pack-specific rested OCV curve */
    bool pack_voltage_valid;
    bool pack_current_valid;
    bool cell_voltage_valid;
    bool rested_soc_observation_valid;
    bool bms_alert;
    bool aircraft_landed;
} battery_sample_t;

typedef struct {
    float capacity_ah;
    float initial_soc_percent;  /* initialize from verified fully charged pack */
    float low_cell_voltage_v;
    float critical_cell_voltage_v;
    float low_pack_voltage_v;
    float maximum_discharge_current_a;
    float voltage_measurement_variance_v2;
    float voltage_process_variance_v2;
    float initial_soc_variance_percent2;
    float soc_process_variance_percent2_per_s;
    float soc_measurement_variance_percent2;
    float maximum_sample_dt_s;
} battery_config_t;

typedef struct {
    float filtered_pack_voltage_v;
    float pack_voltage_sigma_v;
    float estimated_soc_percent; /* coulomb count, initialized by caller */
    float estimated_soc_sigma_percent;
    battery_action_t action;
    bool voltage_filter_initialized;
    bool soc_initialized;
    bool soc_filter_initialized;
    bool sensor_fault;
} battery_status_t;

/* User-selected policy: warning at 35%, immediate latched disarm at 20%.
 * No automatic landing. Only a deliberate disarmed reset starts a new pack/session.
 * These flags do not create sensor data or enable a physical ESC output driver. */
#define BATTERY_SOC_WARNING_PERCENT 35.0f
#define BATTERY_SOC_DISARM_PERCENT 20.0f
static bool battery_warning_active;
static bool battery_disarm_latched;

bool battery_policy_reset(bool disarmed) {
    if (!disarmed) return false;
    battery_warning_active = false;
    battery_disarm_latched = false;
    return true;
}

static void battery_apply_soc_policy(float soc_percent) {
    battery_warning_active = soc_percent <= BATTERY_SOC_WARNING_PERCENT;
    if (soc_percent <= BATTERY_SOC_DISARM_PERCENT) battery_disarm_latched = true;
}

void battery_monitor_init(battery_status_t *status, float initial_soc_percent);
battery_action_t battery_monitor_update(battery_status_t *status,
                                        const battery_sample_t *sample,
                                        const battery_config_t *config,
                                        float dt_s);
/* Configurable battery telemetry / warning logic. No hardware cut-off path. */


#include <math.h>
#include <stddef.h>

static bool positive_finite(float value) {
    return isfinite(value) && value > 0.0f;
}

static bool config_is_valid(const battery_config_t *c) {
    return c != NULL && positive_finite(c->capacity_ah)
        && isfinite(c->initial_soc_percent)
        && c->initial_soc_percent >= 0.0f && c->initial_soc_percent <= 100.0f
        && positive_finite(c->critical_cell_voltage_v)
        && c->low_cell_voltage_v >= c->critical_cell_voltage_v
        && positive_finite(c->low_cell_voltage_v)
        && positive_finite(c->low_pack_voltage_v)
        && positive_finite(c->maximum_discharge_current_a)
        && positive_finite(c->voltage_measurement_variance_v2)
        && c->voltage_process_variance_v2 >= 0.0f
        && isfinite(c->voltage_process_variance_v2)
        && positive_finite(c->initial_soc_variance_percent2)
        && c->soc_process_variance_percent2_per_s >= 0.0f
        && isfinite(c->soc_process_variance_percent2_per_s)
        && positive_finite(c->soc_measurement_variance_percent2)
        && positive_finite(c->maximum_sample_dt_s);
}

void battery_monitor_init(battery_status_t *status, float initial_soc_percent) {
    if (status == NULL) return;
    *status = (battery_status_t){0};
    if (isfinite(initial_soc_percent)
        && initial_soc_percent >= 0.0f && initial_soc_percent <= 100.0f) {
        status->estimated_soc_percent = initial_soc_percent;
        status->soc_initialized = true;
    }
}

battery_action_t battery_monitor_update(battery_status_t *status,
                                        const battery_sample_t *sample,
                                        const battery_config_t *config,
                                        float dt_s) {
    if (status == NULL) return BATTERY_ACTION_CONFIGURATION_FAULT;
    status->sensor_fault = false;
    if (!config_is_valid(config)) {
        status->action = BATTERY_ACTION_CONFIGURATION_FAULT;
        return status->action;
    }
    if (sample == NULL || !isfinite(dt_s) || dt_s <= 0.0f
        || dt_s > config->maximum_sample_dt_s
        || !sample->pack_voltage_valid || !sample->pack_current_valid
        || !isfinite(sample->pack_voltage_v) || sample->pack_voltage_v <= 0.0f
        || !isfinite(sample->pack_current_a)
        || (sample->cell_voltage_valid
            && (!isfinite(sample->minimum_cell_voltage_v)
                || sample->minimum_cell_voltage_v <= 0.0f))
        || (sample->rested_soc_observation_valid
            && (!isfinite(sample->rested_soc_observation_percent)
                || sample->rested_soc_observation_percent < 0.0f
                || sample->rested_soc_observation_percent > 100.0f))) {
        status->sensor_fault = true;
        status->action = BATTERY_ACTION_SENSOR_FAULT;
        return status->action;
    }

    /* Scalar Kalman estimate of noisy total pack voltage; covariance in V^2. */
    if (!status->voltage_filter_initialized) {
        status->filtered_pack_voltage_v = sample->pack_voltage_v;
        status->pack_voltage_sigma_v =
            sqrtf(config->voltage_measurement_variance_v2);
        status->voltage_filter_initialized = true;
    } else {
        float p = status->pack_voltage_sigma_v * status->pack_voltage_sigma_v;
        p += config->voltage_process_variance_v2;
        const float gain = p / (p + config->voltage_measurement_variance_v2);
        status->filtered_pack_voltage_v += gain
            * (sample->pack_voltage_v - status->filtered_pack_voltage_v);
        p *= 1.0f - gain;
        status->pack_voltage_sigma_v = sqrtf(fmaxf(p, 0.0f));
    }

    /* SOC Kalman predict: coulomb count is the process model. */
    if (!status->soc_initialized) {
        status->estimated_soc_percent = config->initial_soc_percent;
        status->soc_initialized = true;
    }
    if (!status->soc_filter_initialized) {
        status->estimated_soc_sigma_percent =
            sqrtf(config->initial_soc_variance_percent2);
        status->soc_filter_initialized = true;
    }
    status->estimated_soc_percent -= sample->pack_current_a * dt_s
        / (config->capacity_ah * 3600.0f) * 100.0f;
    float soc_variance = status->estimated_soc_sigma_percent
                       * status->estimated_soc_sigma_percent
                       + config->soc_process_variance_percent2_per_s * dt_s;
    /* Optional correction comes only from a separately calibrated, rested-pack
       OCV-to-SOC table. Loaded pack voltage is not a valid SOC measurement. */
    if (sample->rested_soc_observation_valid) {
        const float gain = soc_variance
            / (soc_variance + config->soc_measurement_variance_percent2);
        status->estimated_soc_percent += gain
            * (sample->rested_soc_observation_percent
               - status->estimated_soc_percent);
        soc_variance *= 1.0f - gain;
    }
    status->estimated_soc_percent = fminf(100.0f,
        fmaxf(0.0f, status->estimated_soc_percent));
    status->estimated_soc_sigma_percent = sqrtf(fmaxf(soc_variance, 0.0f));

    battery_apply_soc_policy(status->estimated_soc_percent);
    /* Optional external critical alerts also latch disarm. Ordinary voltage/current
       warnings notify; there is intentionally no landing action in this policy. */
    if (sample->bms_alert
        || (sample->cell_voltage_valid
            && sample->minimum_cell_voltage_v <= config->critical_cell_voltage_v)) {
        battery_disarm_latched = true;
    }
    if (status->filtered_pack_voltage_v <= config->low_pack_voltage_v
        || (sample->cell_voltage_valid
            && sample->minimum_cell_voltage_v <= config->low_cell_voltage_v)
        || sample->pack_current_a > config->maximum_discharge_current_a) {
        battery_warning_active = true;
    }
    status->action = battery_disarm_latched ? BATTERY_ACTION_DISARM
                   : battery_warning_active ? BATTERY_ACTION_WARN : BATTERY_ACTION_NORMAL;
    return status->action;
}

#define FC_LOOP_HZ 500.0f
#define FC_LOOP_PERIOD_US 2000u
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f
#define FC_HOVER_THROTTLE_PLACEHOLDER 0.50f
#define FC_MAX_VERTICAL_SPEED_MPS 2.0f
#define FC_RADIO_TIMEOUT_US 150000u
#define FC_SENSOR_LOOP_HZ 1000u

/* SENSOR CALIBRATION -------------------------------------------------------
 * Physical units after sensor factory conversion: g, deg/s, microtesla, Pa.
 * These collectors accept fresh samples only while disarmed. Results live in
 * RAM while collecting; validated fits can be persisted by the flash journal
 * below. Sensor acquisition, temperature modelling and a guided six-face UI
 * remain unfinished. No synthetic fixture is installed as a flight calibration.
 */
typedef struct {
    uint32_t n;
    double mean[3], m2[3];
} cal_stats3_t;

typedef struct {
    float accel_bias_g[3], accel_gain[3];
    float accel_bias_variance_g2[3], accel_gain_variance[3];
    float gyro_bias_dps[3], gyro_variance_dps2[3];
    float gyro_bias_variance_dps2[3];
    uint32_t gyro_samples;
    bool accel_valid, gyro_valid;
} imu_calibration_t;

typedef struct { cal_stats3_t accel, gyro; } imu_stationary_cal_t;

static bool cal_vector_finite(const float v[3]) {
    return v != NULL && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void cal_stats3_add(cal_stats3_t *s, const float v[3]) {
    s->n++;
    for (unsigned i = 0; i < 3; ++i) {
        const double delta = v[i] - s->mean[i];
        s->mean[i] += delta / s->n;
        s->m2[i] += delta * (v[i] - s->mean[i]);
    }
}

static double cal_variance(const cal_stats3_t *s, unsigned axis) {
    return s->n > 1u ? s->m2[axis] / (s->n - 1u) : INFINITY;
}

bool imu_stationary_cal_add(imu_stationary_cal_t *s, const float accel_g[3],
                            const float gyro_dps[3], bool disarmed) {
    if (s == NULL || !disarmed || !cal_vector_finite(accel_g)
        || !cal_vector_finite(gyro_dps) || s->accel.n == UINT32_MAX
        || s->gyro.n == UINT32_MAX) return false;
    const float norm2 = accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1]
                      + accel_g[2]*accel_g[2];
    if (norm2 < 0.64f || norm2 > 1.44f) return false;
    for (unsigned i = 0; i < 3; ++i)
        if (fabsf(gyro_dps[i]) > 10.0f) return false;
    cal_stats3_add(&s->accel, accel_g);
    cal_stats3_add(&s->gyro, gyro_dps);
    return true;
}

bool imu_gyro_cal_finish(const imu_stationary_cal_t *s, imu_calibration_t *out,
                         bool disarmed) {
    if (s == NULL || out == NULL || !disarmed || s->gyro.n < 500u
        || s->accel.n != s->gyro.n) return false;
    double norm2 = 0.0;
    for (unsigned i = 0; i < 3; ++i) {
        norm2 += s->accel.mean[i] * s->accel.mean[i];
        if (!isfinite(s->gyro.mean[i]) || fabs(s->gyro.mean[i]) > 5.0
            || !isfinite(cal_variance(&s->accel, i))
            || !isfinite(cal_variance(&s->gyro, i))
            || cal_variance(&s->accel, i) < 0.0
            || cal_variance(&s->gyro, i) < 0.0
            || cal_variance(&s->accel, i) > 0.0004
            || cal_variance(&s->gyro, i) > 0.01) return false;
    }
    if (!isfinite(norm2) || norm2 < 0.81 || norm2 > 1.21) return false;
    for (unsigned i = 0; i < 3; ++i) {
        out->gyro_bias_dps[i] = (float)s->gyro.mean[i];
        out->gyro_variance_dps2[i] = (float)cal_variance(&s->gyro, i);
        out->gyro_bias_variance_dps2[i] = out->gyro_variance_dps2[i] / s->gyro.n;
    }
    out->gyro_samples = s->gyro.n;
    out->gyro_valid = true;
    return true;
}

/* Six poses in sensor axes: +X, -X, +Y, -Y, +Z, -Z (specific force). */
bool imu_accel_face_add(cal_stats3_t faces[6], unsigned face,
                        const float accel_g[3], bool disarmed) {
    if (faces == NULL || face >= 6u || !disarmed
        || !cal_vector_finite(accel_g) || faces[face].n == UINT32_MAX)
        return false;
    cal_stats3_add(&faces[face], accel_g);
    return true;
}

bool imu_accel_cal_finish(const cal_stats3_t faces[6], imu_calibration_t *out,
                          bool disarmed) {
    if (faces == NULL || out == NULL || !disarmed) return false;
    float bias[3], gain[3];
    for (unsigned face = 0; face < 6u; ++face) {
        if (faces[face].n < 100u) return false;
        for (unsigned axis = 0; axis < 3u; ++axis) {
            if (!isfinite(faces[face].mean[axis])
                || !isfinite(cal_variance(&faces[face], axis))
                || cal_variance(&faces[face], axis) < 0.0
                || cal_variance(&faces[face], axis) > 0.0004) return false;
            if (axis != face / 2u && fabs(faces[face].mean[axis]) > 0.25)
                return false;
        }
    }
    for (unsigned axis = 0; axis < 3u; ++axis) {
        const double pos = faces[axis * 2u].mean[axis];
        const double neg = faces[axis * 2u + 1u].mean[axis];
        const double span = pos - neg;
        if (pos < 0.7 || neg > -0.7 || span < 1.5 || span > 2.5)
            return false;
        bias[axis] = (float)((pos + neg) * 0.5);
        gain[axis] = (float)(2.0 / span);
        if (fabsf(bias[axis]) > 0.25f) return false;
    }
    for (unsigned i = 0; i < 3u; ++i) {
        out->accel_bias_g[i] = bias[i];
        out->accel_gain[i] = gain[i];
        const cal_stats3_t *pos = &faces[2u*i], *neg = &faces[2u*i+1u];
        const double mean_variance = cal_variance(pos, i)/pos->n
                                   + cal_variance(neg, i)/neg->n;
        const double span = pos->mean[i] - neg->mean[i];
        out->accel_bias_variance_g2[i] = (float)(0.25 * mean_variance);
        out->accel_gain_variance[i] = (float)(4.0 * mean_variance / pow(span, 4.0));
    }
    out->accel_valid = true;
    return true;
}

bool imu_cal_apply(const imu_calibration_t *c, const float accel_g[3],
                    const float gyro_dps[3], float corrected_accel_g[3],
                    float corrected_gyro_dps[3]) {
    if (c == NULL || !c->accel_valid || !c->gyro_valid
        || !cal_vector_finite(accel_g) || !cal_vector_finite(gyro_dps)
        || corrected_accel_g == NULL || corrected_gyro_dps == NULL)
        return false;
    float a[3], g[3];
    for (unsigned i = 0; i < 3u; ++i) {
        if (!isfinite(c->accel_gain[i]) || c->accel_gain[i] <= 0.0f
            || !isfinite(c->accel_bias_g[i]) || !isfinite(c->gyro_bias_dps[i]))
            return false;
        a[i] = (accel_g[i] - c->accel_bias_g[i]) * c->accel_gain[i];
        g[i] = gyro_dps[i] - c->gyro_bias_dps[i];
        if (!isfinite(a[i]) || !isfinite(g[i])) return false;
    }
    for (unsigned i = 0; i < 3u; ++i) {
        corrected_accel_g[i] = a[i];
        corrected_gyro_dps[i] = g[i];
    }
    return true;
}

typedef struct {
    cal_stats3_t stats;
    float minimum_ut[3], maximum_ut[3];
    double cross_m2[3]; /* xy, xz, yz: reject a rotation confined to a plane */
} mag_sweep_cal_t;

typedef struct { float offset_ut[3], gain[3]; bool valid; } mag_calibration_t;

bool mag_cal_add(mag_sweep_cal_t *s, const float field_ut[3], bool disarmed) {
    if (s == NULL || !disarmed || !cal_vector_finite(field_ut)
        || s->stats.n == UINT32_MAX) return false;
    double delta[3];
    for (unsigned i = 0; i < 3u; ++i) {
        if (fabsf(field_ut[i]) > 1000.0f) return false;
        delta[i] = field_ut[i] - s->stats.mean[i];
    }
    for (unsigned i = 0; i < 3u; ++i) {
        if (s->stats.n == 0u || field_ut[i] < s->minimum_ut[i])
            s->minimum_ut[i] = field_ut[i];
        if (s->stats.n == 0u || field_ut[i] > s->maximum_ut[i])
            s->maximum_ut[i] = field_ut[i];
    }
    cal_stats3_add(&s->stats, field_ut);
    s->cross_m2[0] += delta[0] * (field_ut[1] - s->stats.mean[1]);
    s->cross_m2[1] += delta[0] * (field_ut[2] - s->stats.mean[2]);
    s->cross_m2[2] += delta[1] * (field_ut[2] - s->stats.mean[2]);
    return true;
}

/* Min/max hard-iron + diagonal scale fit, NOT a full soft-iron matrix fit. */
bool mag_cal_finish(const mag_sweep_cal_t *s, mag_calibration_t *out,
                     bool full_rotation_confirmed, bool disarmed) {
    if (s == NULL || out == NULL || !disarmed || !full_rotation_confirmed
        || s->stats.n < 300u) return false;
    double radius[3], mean_radius = 0.0;
    for (unsigned i = 0; i < 3u; ++i) {
        radius[i] = (s->maximum_ut[i] - s->minimum_ut[i]) * 0.5;
        if (!isfinite(radius[i]) || radius[i] < 5.0 || radius[i] > 100.0
            || !isfinite(s->stats.m2[i]) || s->stats.m2[i] <= 0.0)
            return false;
        mean_radius += radius[i] / 3.0;
    }
    const double xy = s->cross_m2[0] / sqrt(s->stats.m2[0]*s->stats.m2[1]);
    const double xz = s->cross_m2[1] / sqrt(s->stats.m2[0]*s->stats.m2[2]);
    const double yz = s->cross_m2[2] / sqrt(s->stats.m2[1]*s->stats.m2[2]);
    const double coverage = 1.0 + 2.0*xy*xz*yz - xy*xy - xz*xz - yz*yz;
    if (!isfinite(coverage) || coverage < 0.05) return false;
    for (unsigned i = 0; i < 3u; ++i)
        if (mean_radius / radius[i] < 0.5 || mean_radius / radius[i] > 2.0)
            return false;
    for (unsigned i = 0; i < 3u; ++i) {
        out->offset_ut[i] = (s->maximum_ut[i] + s->minimum_ut[i]) * 0.5f;
        out->gain[i] = (float)(mean_radius / radius[i]);
    }
    out->valid = true;
    return true;
}

bool mag_cal_apply(const mag_calibration_t *c, const float raw_ut[3],
                    float corrected_ut[3]) {
    if (c == NULL || !c->valid || !cal_vector_finite(raw_ut)
        || corrected_ut == NULL) return false;
    float corrected[3];
    for (unsigned i = 0; i < 3u; ++i) {
        if (!isfinite(c->offset_ut[i]) || !isfinite(c->gain[i])
            || c->gain[i] <= 0.0f) return false;
        corrected[i] = (raw_ut[i] - c->offset_ut[i]) * c->gain[i];
        if (!isfinite(corrected[i])) return false;
    }
    for (unsigned i = 0; i < 3u; ++i) corrected_ut[i] = corrected[i];
    return true;
}

/* BMP280 OTP coefficients are factory calibration, separate from ground zero.
 * Compensation equations: Bosch BMP280 datasheet, section 3.11.3.
 */
typedef struct {
    uint16_t t1, p1;
    int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9;
} bmp280_trim_t;

bool bmp280_trim_decode(const uint8_t *bytes, size_t length, bmp280_trim_t *out) {
    if (bytes == NULL || out == NULL || length != 24u) return false;
    const bmp280_trim_t t = {
        .t1 = qcopter_get_u16(bytes), .t2 = (int16_t)qcopter_get_u16(bytes + 2),
        .t3 = (int16_t)qcopter_get_u16(bytes + 4), .p1 = qcopter_get_u16(bytes + 6),
        .p2 = (int16_t)qcopter_get_u16(bytes + 8), .p3 = (int16_t)qcopter_get_u16(bytes + 10),
        .p4 = (int16_t)qcopter_get_u16(bytes + 12), .p5 = (int16_t)qcopter_get_u16(bytes + 14),
        .p6 = (int16_t)qcopter_get_u16(bytes + 16), .p7 = (int16_t)qcopter_get_u16(bytes + 18),
        .p8 = (int16_t)qcopter_get_u16(bytes + 20), .p9 = (int16_t)qcopter_get_u16(bytes + 22)
    };
    if (t.t1 == 0u || t.t1 == 0xFFFFu || t.p1 == 0u || t.p1 == 0xFFFFu)
        return false;
    *out = t;
    return true;
}

bool bmp280_compensate(const bmp280_trim_t *t, uint32_t raw_temperature,
                        uint32_t raw_pressure, double *temperature_c,
                        double *pressure_pa) {
    if (t == NULL || temperature_c == NULL || pressure_pa == NULL
        || t->t1 == 0u || t->p1 == 0u || raw_temperature > 0xFFFFFu
        || raw_pressure > 0xFFFFFu || raw_temperature == 0x80000u
        || raw_pressure == 0x80000u) return false;
    const double dt = raw_temperature / 131072.0 - t->t1 / 8192.0;
    const double fine = (raw_temperature / 16384.0 - t->t1 / 1024.0) * t->t2
                      + dt * dt * t->t3;
    const double temp = fine / 5120.0;
    double v1 = fine / 2.0 - 64000.0;
    double v2 = v1*v1*t->p6/32768.0 + v1*t->p5*2.0;
    v2 = v2/4.0 + t->p4*65536.0;
    v1 = (t->p3*v1*v1/524288.0 + t->p2*v1)/524288.0;
    v1 = (1.0 + v1/32768.0) * t->p1;
    if (!isfinite(v1) || fabs(v1) < 1e-12) return false;
    double pressure = (1048576.0 - raw_pressure - v2/4096.0) * 6250.0 / v1;
    v1 = t->p9*pressure*pressure/2147483648.0;
    v2 = pressure*t->p8/32768.0;
    pressure += (v1 + v2 + t->p7)/16.0;
    if (!isfinite(temp) || !isfinite(pressure) || temp < -40.0 || temp > 85.0
        || pressure < 30000.0 || pressure > 110000.0) return false;
    *temperature_c = temp;
    *pressure_pa = pressure;
    return true;
}

typedef struct { uint32_t n; double mean_pa, m2_pa2; } baro_reference_cal_t;
typedef struct {
    double ground_pressure_pa, reference_variance_pa2, measurement_variance_pa2;
    bool valid;
} baro_calibration_t;

bool baro_reference_add(baro_reference_cal_t *s, double compensated_pa,
                         bool disarmed) {
    if (s == NULL || !disarmed || !isfinite(compensated_pa)
        || compensated_pa < 30000.0 || compensated_pa > 110000.0
        || s->n == UINT32_MAX) return false;
    const double delta = compensated_pa - s->mean_pa;
    s->n++;
    s->mean_pa += delta / s->n;
    s->m2_pa2 += delta * (compensated_pa - s->mean_pa);
    return true;
}

bool baro_reference_finish(const baro_reference_cal_t *s, baro_calibration_t *out,
                            double maximum_sigma_pa, bool disarmed) {
    if (s == NULL || out == NULL || !disarmed || s->n < 100u
        || !isfinite(maximum_sigma_pa) || maximum_sigma_pa <= 0.0
        || !isfinite(s->mean_pa) || s->mean_pa < 30000.0
        || s->mean_pa > 110000.0) return false;
    const double variance = s->m2_pa2 / (s->n - 1u);
    if (!isfinite(variance) || variance < 0.0
        || variance > maximum_sigma_pa * maximum_sigma_pa) return false;
    *out = (baro_calibration_t){s->mean_pa, variance / s->n, variance, true};
    return true;
}

bool baro_relative_height(const baro_calibration_t *c, double compensated_pa,
                           double *height_m, double *sigma_m) {
    if (c == NULL || !c->valid || height_m == NULL || sigma_m == NULL
        || !isfinite(compensated_pa) || compensated_pa < 30000.0
        || compensated_pa > 110000.0 || !isfinite(c->ground_pressure_pa)
        || c->ground_pressure_pa < 30000.0 || c->ground_pressure_pa > 110000.0
        || !isfinite(c->reference_variance_pa2) || c->reference_variance_pa2 < 0.0
        || !isfinite(c->measurement_variance_pa2) || c->measurement_variance_pa2 < 0.0)
        return false;
    const double exponent = 0.190294957;
    const double ratio = compensated_pa / c->ground_pressure_pa;
    const double derivative = 44330.0 * exponent * pow(ratio, exponent - 1.0);
    const double dp = -derivative / c->ground_pressure_pa;
    const double dp0 = derivative * compensated_pa
                      / (c->ground_pressure_pa * c->ground_pressure_pa);
    *height_m = 44330.0 * (1.0 - pow(ratio, exponent));
    *sigma_m = sqrt(dp*dp*c->measurement_variance_pa2
                    + dp0*dp0*c->reference_variance_pa2);
    return isfinite(*height_m) && isfinite(*sigma_m);
}

/* Reference-pair regression for battery volts, current zero/gain, or lidar.
 * Example: input ADC volts, reference trusted multimeter volts / measured amps.
 * For lidar, reference a measured target distance; terrain is not a reference.
 */
typedef struct { uint32_t n; double x, y, xx, yy, xy; } linear_cal_fit_t;
typedef struct { double gain, offset, residual_sigma; bool valid; } linear_cal_t;

bool linear_cal_add(linear_cal_fit_t *s, double input, double reference,
                      bool disarmed) {
    if (s == NULL || !disarmed || !isfinite(input) || !isfinite(reference)
        || fabs(input) > 1e9 || fabs(reference) > 1e9 || s->n == UINT32_MAX)
        return false;
    const double dx = input - s->x, dy = reference - s->y;
    s->n++;
    s->x += dx / s->n;
    s->y += dy / s->n;
    s->xx += dx * (input - s->x);
    s->yy += dy * (reference - s->y);
    s->xy += dx * (reference - s->y);
    return true;
}

bool linear_cal_finish(const linear_cal_fit_t *s, linear_cal_t *out,
                         double maximum_residual_sigma, bool disarmed) {
    if (s == NULL || out == NULL || !disarmed || s->n < 3u
        || !isfinite(s->xx) || !isfinite(s->yy) || !isfinite(s->xy)
        || !isfinite(s->x) || !isfinite(s->y) || s->yy < 0.0 || s->xx <= 1e-8
        || !isfinite(maximum_residual_sigma) || maximum_residual_sigma <= 0.0)
        return false;
    const double gain = s->xy / s->xx, offset = s->y - gain * s->x;
    const double residual = sqrt(fmax(0.0, s->yy - gain*s->xy) / (s->n - 2u));
    if (!isfinite(gain) || fabs(gain) < 1e-12 || !isfinite(offset)
        || !isfinite(residual) || residual > maximum_residual_sigma)
        return false;
    *out = (linear_cal_t){gain, offset, residual, true};
    return true;
}

bool linear_cal_apply(const linear_cal_t *c, double raw, double *corrected) {
    if (c == NULL || !c->valid || corrected == NULL || !isfinite(raw)
        || !isfinite(c->gain) || !isfinite(c->offset)) return false;
    const double value = c->gain * raw + c->offset;
    if (!isfinite(value)) return false;
    *corrected = value;
    return true;
}

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

typedef struct {
    float angle_deg;
    float gyro_bias_dps;
    float p00;
    float p01;
    float p10;
    float p11;
    float gyro_noise;
    float bias_walk;
    float accel_noise;
    float gain_angle, gain_bias, innovation, innovation_variance;
} angle_kf_t;

typedef struct {
    float height_m;
    float velocity_mps;
    float p00;
    float p01;
    float p10;
    float p11;
    float accel_noise;
    float baro_noise;
    float gain_height, gain_velocity, innovation, innovation_variance;
} altitude_kf_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float throttle; /* [0,1], center requests zero vertical speed */
    float boost;    /* RT [0,1]: extra climb/collective allowance, same flight mode */
    bool valid;
    bool arm_request;
} pilot_command_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float vertical_acceleration_mps2; /* gravity compensated, world-up positive */
    float barometer_height_m;
    bool imu_valid;
    bool imu_calibrated; /* scaled + offset/gain corrected before control */
    bool barometer_valid;
    bool barometer_updated; /* apply each independent pressure sample once */
} sensor_sample_t;

bool sensor_sample_apply_imu_calibration(sensor_sample_t *s,
                                          const imu_calibration_t *cal) {
    if (s == NULL || !s->imu_valid || s->imu_calibrated) return false;
    const float accel[3] = {s->accel_x_g, s->accel_y_g, s->accel_z_g};
    const float gyro[3] = {s->gyro_x_dps, s->gyro_y_dps, s->gyro_z_dps};
    float a[3], g[3];
    if (!imu_cal_apply(cal, accel, gyro, a, g)) return false;
    s->accel_x_g = a[0]; s->accel_y_g = a[1]; s->accel_z_g = a[2];
    s->gyro_x_dps = g[0]; s->gyro_y_dps = g[1]; s->gyro_z_dps = g[2];
    s->imu_calibrated = true;
    /* Derive world-up acceleration AFTER this correction and frame rotation. */
    return true;
}

typedef struct {
    float motor[4]; /* normalized math result only; no ESC output is connected */
    bool armed;
    bool sensor_fault;
} control_output_t;

typedef struct {
    pilot_command_t latest;
    uint16_t last_sequence;
    uint64_t last_received_us;
    bool received;
} pilot_link_state_t;

static bool pilot_link_accept(pilot_link_state_t *link, const uint8_t *bytes,
                              size_t length, uint64_t received_us) {
    qcopter_command_frame_t frame;
    if (link == NULL || !qcopter_decode_frame(bytes, length, &frame)) return false;
    if (link->received
        && !qcopter_sequence_is_newer(frame.sequence, link->last_sequence))
        return false;
    link->latest = (pilot_command_t){
        .roll_deg = (float)frame.roll * 35.0f / QCOPTER_AXIS_SCALE,
        .pitch_deg = (float)frame.pitch * 35.0f / QCOPTER_AXIS_SCALE,
        .yaw_rate_dps = (float)frame.yaw_rate * 180.0f / QCOPTER_AXIS_SCALE,
        .throttle = (float)frame.throttle / QCOPTER_THROTTLE_SCALE,
        .boost = (float)frame.rt / QCOPTER_TRIGGER_SCALE,
        .valid = true,
        .arm_request = false /* no arming gesture is defined or enabled yet */
    };
    link->last_sequence = frame.sequence;
    link->last_received_us = received_us;
    link->received = true;
    return true;
}

static pilot_command_t pilot_link_current(const pilot_link_state_t *link,
                                          uint64_t now_us) {
    pilot_command_t command = {0};
    if (link != NULL && link->received && now_us >= link->last_received_us
        && now_us - link->last_received_us <= FC_RADIO_TIMEOUT_US)
        command = link->latest;
    else
        command.valid = false;
    return command;
}

static angle_kf_t roll_estimator = {
    .p00 = 1.0f, .p11 = 0.01f,
    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
};
static angle_kf_t pitch_estimator = {
    .p00 = 1.0f, .p11 = 0.01f,
    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
};
static altitude_kf_t altitude_estimator = {
    .p00 = 1.0f, .p11 = 1.0f,
    .accel_noise = 1.5f, .baro_noise = 2.0f
};

/* Gains are placeholders for simulation/bench tuning, not flight values. */
static pid_t roll_angle_pid = {
    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t pitch_angle_pid = {
    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t roll_rate_pid = {
    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,
    .integrator_limit = 15.0f, .output_limit = 0.35f
};
static pid_t pitch_rate_pid = {
    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,
    .integrator_limit = 15.0f, .output_limit = 0.35f
};
static pid_t yaw_rate_pid = {
    .kp = 0.005375614f, .ki = 0.008362066f, .kd = 0.0f,
    .integrator_limit = 8.0f, .output_limit = 0.35f
};
static pid_t altitude_pid = {
    .kp = 0.108045f, .ki = 0.03087f, .kd = 0.1225f,
    .integrator_limit = 2.0f, .output_limit = 0.35f
};
static float altitude_hold_reference_m;
static float hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;
/* User policy, 2026-10-28: normalized ESC command fractions, not measured RPM. */
#define FC_ARMED_IDLE_FRACTION 0.10f
#define FC_NORMAL_MOTOR_CEILING 0.45f
#define FC_BOOST_MOTOR_CEILING 0.65f
static float boost_state;
static bool altitude_reference_initialized;

static float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void pid_reset(pid_t *pid) {
    pid->integrator = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->initialized = false;
}

static void reset_control_state(void) {
    boost_state=0;
    pid_reset(&roll_angle_pid);
    pid_reset(&pitch_angle_pid);
    pid_reset(&roll_rate_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
    pid_reset(&altitude_pid);
    altitude_reference_initialized = false;
}

/* Derivative is taken on the measurement to avoid setpoint kick. */
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
    /* Conditional integration prevents windup when saturation is driven farther. */
    if (raw == output || (raw > output && error < 0.0f)
                      || (raw < output && error > 0.0f)) {
        pid->integrator = candidate_i;
    }
    pid->previous_measurement = measurement;
    pid->initialized = true;
    return output;
}

/* Joseph scalar-observation covariance update, H=[1,0]. */
static void covariance_correct(float *p00, float *p01, float *p10, float *p11,
                                float k0, float k1, float r) {
    const float a = *p00, b = (*p01 + *p10)*0.5f, d = *p11;
    const float z = 1.0f-k0;
    *p00 = z*z*a + k0*k0*r;
    *p01 = z*(b-k1*a) + k0*k1*r;
    *p10 = *p01;
    *p11 = d-2.0f*k1*b+k1*k1*a+k1*k1*r;
}

static void angle_kf_update(angle_kf_t *kf, float gyro_dps,
                            float accel_angle_deg, float dt) {
    kf->angle_deg += dt*(gyro_dps-kf->gyro_bias_dps);
    const float a=kf->p00, b=kf->p01, c=kf->p10, d=kf->p11;
    const float qb=kf->bias_walk;
    /* gyro_noise is per-sample (deg/s)^2; bias_walk is (deg/s)^2/s. */
    kf->p00=a-dt*(b+c)+dt*dt*d+kf->gyro_noise*dt*dt+qb*dt*dt*dt/3.0f;
    kf->p01=b-dt*d-qb*dt*dt/2.0f;
    kf->p10=c-dt*d-qb*dt*dt/2.0f;
    kf->p11=d+qb*dt;
    kf->innovation_variance=kf->p00+kf->accel_noise;
    kf->gain_angle=kf->p00/kf->innovation_variance;
    kf->gain_bias=kf->p10/kf->innovation_variance;
    kf->innovation=accel_angle_deg-kf->angle_deg;
    kf->angle_deg+=kf->gain_angle*kf->innovation;
    kf->gyro_bias_dps+=kf->gain_bias*kf->innovation;
    covariance_correct(&kf->p00,&kf->p01,&kf->p10,&kf->p11,
                       kf->gain_angle,kf->gain_bias,kf->accel_noise);
}

/* 2-state constant-acceleration filter: state = [height, vertical speed]. */
static void altitude_kf_predict(altitude_kf_t *kf, float accel_up_mps2,
                                float dt) {
    kf->height_m += kf->velocity_mps * dt
                  + 0.5f * accel_up_mps2 * dt * dt;
    kf->velocity_mps += accel_up_mps2 * dt;

    const float p00 = kf->p00;
    const float p01 = kf->p01;
    const float p10 = kf->p10;
    const float p11 = kf->p11;
    const float q = kf->accel_noise * kf->accel_noise;
    kf->p00 = p00 + dt * (p01 + p10) + dt * dt * p11
            + 0.25f * dt * dt * dt * dt * q;
    kf->p01 = p01 + dt * p11 + 0.5f * dt * dt * dt * q;
    kf->p10 = p10 + dt * p11 + 0.5f * dt * dt * dt * q;
    kf->p11 = p11 + dt * dt * q;
}

static void altitude_kf_baro_update(altitude_kf_t *kf, float height_m) {
    kf->innovation_variance=kf->p00+kf->baro_noise;
    kf->gain_height=kf->p00/kf->innovation_variance;
    kf->gain_velocity=kf->p10/kf->innovation_variance;
    kf->innovation=height_m-kf->height_m;
    kf->height_m+=kf->gain_height*kf->innovation;
    kf->velocity_mps+=kf->gain_velocity*kf->innovation;
    covariance_correct(&kf->p00,&kf->p01,&kf->p10,&kf->p11,
                       kf->gain_height,kf->gain_velocity,kf->baro_noise);
}

/* Exactly one altitude PID: D uses commanded/measured vertical velocity. */
static float altitude_pid_update(float reference, float height, float v_cmd,
                                  float velocity, float dt) {
    const float error=reference-height;
    const float candidate=clampf(altitude_pid.integrator+error*dt,
        -altitude_pid.integrator_limit,altitude_pid.integrator_limit);
    const float raw=altitude_pid.kp*error+altitude_pid.ki*candidate
                    +altitude_pid.kd*(v_cmd-velocity);
    const float output=clampf(raw,-altitude_pid.output_limit,altitude_pid.output_limit);
    if (raw==output || (raw>output && error<0.0f) || (raw<output && error>0.0f))
        altitude_pid.integrator=candidate;
    return output;
}

static bool sample_is_healthy(const sensor_sample_t *s) {
    return s != NULL && s->imu_valid && s->imu_calibrated
        && isfinite(s->accel_x_g) && isfinite(s->accel_y_g)
        && isfinite(s->accel_z_g) && isfinite(s->gyro_x_dps)
        && isfinite(s->gyro_y_dps) && isfinite(s->gyro_z_dps);
}

/*
 * This function is currently a math/control core only. Mixer motor order and
 * signs must be validated against the chosen frame before its outputs are used.
 */
static control_output_t flight_control_step(const sensor_sample_t *s,
                                             const pilot_command_t *cmd,
                                             float dt) {
    control_output_t out = { .motor = {0.0f, 0.0f, 0.0f, 0.0f},
                             .armed = false, .sensor_fault = false };
    if (battery_disarm_latched) {
        reset_control_state();
        return out; /* Explicit user policy: stop mathematical demands, even airborne. */
    }
    if (!sample_is_healthy(s) || !isfinite(dt) || dt <= 0.0f
        || dt > FC_MAX_CONTROL_DT) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }

    const float accel_roll_deg = atan2f(s->accel_y_g, s->accel_z_g)
                               * FC_RAD_TO_DEG;
    const float accel_pitch_deg = atan2f(-s->accel_x_g,
        sqrtf(s->accel_y_g * s->accel_y_g + s->accel_z_g * s->accel_z_g))
        * FC_RAD_TO_DEG;
    angle_kf_update(&roll_estimator, s->gyro_x_dps, accel_roll_deg, dt);
    angle_kf_update(&pitch_estimator, s->gyro_y_dps, accel_pitch_deg, dt);

    if (s->barometer_valid && isfinite(s->barometer_height_m)
        && isfinite(s->vertical_acceleration_mps2)) {
        altitude_kf_predict(&altitude_estimator,
                            s->vertical_acceleration_mps2, dt);
        if (s->barometer_updated)
            altitude_kf_baro_update(&altitude_estimator, s->barometer_height_m);
    }

    if (cmd == NULL || !cmd->valid || !isfinite(cmd->roll_deg) || !isfinite(cmd->pitch_deg)
        || !isfinite(cmd->yaw_rate_dps) || !isfinite(cmd->throttle) || !isfinite(cmd->boost)) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }
    /* The only implemented mode requires valid height/rate aiding. */
    if (!s->barometer_valid || !isfinite(s->barometer_height_m)
        || !isfinite(s->vertical_acceleration_mps2)) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }

    /* Bound pilot inputs before they enter the control math. */
    const float roll_setpoint = clampf(cmd->roll_deg, -35.0f, 35.0f);
    const float pitch_setpoint = clampf(cmd->pitch_deg, -35.0f, 35.0f);
    const float yaw_rate_setpoint = clampf(cmd->yaw_rate_dps, -180.0f, 180.0f);
    const float throttle = clampf(cmd->throttle, 0.0f, 1.0f);
    const float boost_target=clampf(cmd->boost,0,1);
    boost_state+=clampf(boost_target-boost_state,-10*dt,4*dt);
    const float boost=boost_state;
    const float roll_rate_setpoint = pid_update(&roll_angle_pid, roll_setpoint,
                                                 roll_estimator.angle_deg, dt);
    const float pitch_rate_setpoint = pid_update(&pitch_angle_pid, pitch_setpoint,
                                                  pitch_estimator.angle_deg, dt);
    const float roll = pid_update(&roll_rate_pid, roll_rate_setpoint,
                                  s->gyro_x_dps, dt);
    const float pitch = pid_update(&pitch_rate_pid, pitch_rate_setpoint,
                                   s->gyro_y_dps, dt);
    const float yaw = pid_update(&yaw_rate_pid, yaw_rate_setpoint,
                                 s->gyro_z_dps, dt);

    if (!altitude_reference_initialized) {
        altitude_hold_reference_m = altitude_estimator.height_m;
        pid_reset(&altitude_pid);
        altitude_reference_initialized = true;
    }
    const float pilot_vertical_speed = (throttle - 0.5f)
                                     * 2.0f * (FC_MAX_VERTICAL_SPEED_MPS+boost);
    altitude_hold_reference_m += pilot_vertical_speed * dt;
    altitude_hold_reference_m=clampf(altitude_hold_reference_m,
        altitude_estimator.height_m-2.0f,altitude_estimator.height_m+2.0f);
    const float collective_correction=altitude_pid_update(altitude_hold_reference_m,
        altitude_estimator.height_m,pilot_vertical_speed,altitude_estimator.velocity_mps,dt);
    const float tilt_factor=fmaxf(0.5f,cosf(roll_estimator.angle_deg*FC_DEG_TO_RAD)
                                     *cosf(pitch_estimator.angle_deg*FC_DEG_TO_RAD));
    const float ceiling=FC_NORMAL_MOTOR_CEILING+(FC_BOOST_MOTOR_CEILING-FC_NORMAL_MOTOR_CEILING)*fminf(boost,boost_target);
    const float collective=clampf((hover_collective+collective_correction)/tilt_factor,FC_ARMED_IDLE_FRACTION,ceiling);

    /* Provisional X-frame math in logical order FL, FR, RR, RL. The result is
       for simulation only until motor order, spin directions, and frame axes
       are verified. No PWM/DShot/ESC driver consumes these values. */
    float correction[4]={roll-pitch-yaw,-roll-pitch+yaw,-roll+pitch-yaw,roll+pitch+yaw};
    float low=correction[0],high=correction[0];
    for(unsigned i=1;i<4;i++){low=fminf(low,correction[i]);high=fmaxf(high,correction[i]);}
    // Uniformly scale only infeasible torque combinations, then shift collective
    // into the feasible interval. Independent clipping would distort torques.
    const float span=ceiling-FC_ARMED_IDLE_FRACTION;
    const float scale=high-low>span ? span/(high-low) : 1;
    const float allocated=clampf(collective,FC_ARMED_IDLE_FRACTION-low*scale,ceiling-high*scale);
    for(unsigned i=0;i<4;i++)out.motor[i]=clampf(allocated+scale*correction[i],FC_ARMED_IDLE_FRACTION,ceiling);

    /* There is deliberately no hardware arming/output path in this scaffold. */
    out.armed = false;
    return out;
}

#ifdef QCOPTER_HOST_TEST
/* Host-only ABI: same C control equations, no physical ESC/arming path. */
#ifdef QCOPTER_SITL_DLL
#define QC_SIM_EXPORT __declspec(dllexport)
#else
#define QC_SIM_EXPORT
#endif
QC_SIM_EXPORT void qc_sitl_reset(float hover, float height, const float *gains) {
    (void)battery_policy_reset(true); /* New simulated vehicle/battery session. */
    boost_state=0;
    roll_estimator=(angle_kf_t){.p00=1.0f,.p11=0.01f,
        .gyro_noise=0.04f,.bias_walk=0.0001f,.accel_noise=4.0f};
    pitch_estimator=roll_estimator;
    altitude_estimator=(altitude_kf_t){.height_m=height,.p00=0.04f,.p11=0.1f,
        .accel_noise=1.5f,.baro_noise=0.0025f};
    pid_t *loops[6]={&roll_angle_pid,&roll_rate_pid,&pitch_angle_pid,
                     &pitch_rate_pid,&yaw_rate_pid,&altitude_pid};
    for(unsigned i=0;i<6u;++i) {
        if(gains!=NULL) {
            loops[i]->kp=gains[3u*i]; loops[i]->ki=gains[3u*i+1u];
            loops[i]->kd=gains[3u*i+2u];
        }
        pid_reset(loops[i]);
    }
    hover_collective=clampf(hover,0.05f,0.85f);
    altitude_reference_initialized=false;
}

QC_SIM_EXPORT void qc_sitl_step(const float *input, const float *command,
                                float dt, unsigned flags, float *motors,
                                float *diagnostics) {
    const sensor_sample_t s={
        .accel_x_g=input[0],.accel_y_g=input[1],.accel_z_g=input[2],
        .gyro_x_dps=input[3],.gyro_y_dps=input[4],.gyro_z_dps=input[5],
        .vertical_acceleration_mps2=input[6],.barometer_height_m=input[7],
        .barometer_updated=input[8]>0.5f,.imu_valid=(flags&1u)!=0u,
        .imu_calibrated=(flags&1u)!=0u,.barometer_valid=(flags&2u)!=0u};
    const pilot_command_t c={.roll_deg=command[0],.pitch_deg=command[1],
        .yaw_rate_dps=command[2],.throttle=command[3],.boost=(flags&8u)!=0u?command[4]:0,.valid=(flags&4u)!=0u};
    const control_output_t out=flight_control_step(&s,&c,dt);
    for(unsigned i=0;i<4u;++i) motors[i]=out.motor[i];
    const float d[16]={roll_estimator.angle_deg,pitch_estimator.angle_deg,
        altitude_estimator.height_m,altitude_estimator.velocity_mps,
        roll_estimator.gain_angle,roll_estimator.gain_bias,
        altitude_estimator.gain_height,altitude_estimator.gain_velocity,
        sqrtf(fmaxf(0.0f,roll_estimator.p00)),
        sqrtf(fmaxf(0.0f,altitude_estimator.p00)),
        roll_estimator.innovation,altitude_estimator.innovation,
        altitude_hold_reference_m,(float)out.armed,(float)out.sensor_fault,
        roll_estimator.innovation_variance};
    for(unsigned i=0;i<16u;++i) diagnostics[i]=d[i];
}
QC_SIM_EXPORT void qc_sitl_height_observation(float height,float variance) {
    if(isfinite(height)&&isfinite(variance)&&variance>0) {
        const float previous=altitude_estimator.baro_noise;
        altitude_estimator.baro_noise=variance;
        altitude_kf_baro_update(&altitude_estimator,height);
        altitude_estimator.baro_noise=previous;
    }
}
/* Optional simulation battery input. Existing flight scenarios do not fabricate
 * battery measurements; a caller must supply a valid estimated SOC explicitly. */
QC_SIM_EXPORT void qc_sitl_set_battery_soc(float soc_percent) {
    if (isfinite(soc_percent) && soc_percent >= 0 && soc_percent <= 100)
        battery_apply_soc_policy(soc_percent);
}
QC_SIM_EXPORT unsigned qc_sitl_battery_flags(void) {
    return (battery_warning_active ? 1u : 0u) | (battery_disarm_latched ? 2u : 0u);
}
#endif

#ifndef QCOPTER_HOST_TEST
/* CALIBRATION FLASH JOURNAL -------------------------------------------------
 * Two reserved 4096-byte sectors; 512-byte data + final 256-byte commit page.
 * Explicit little-endian serialization, CRC32, board/config binding, A/B updates.
 * Never persist volatile ground-pressure zero, battery SOC, or live PID integrals.
 * Core 0 owns RAM fits and writes; flash operations are disarmed-only.
 */
#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/unique_id.h"

#define FC_CAL_PROFILE_ID 0x00010001u /* Bump for units, mounting, IMU range/DLPF or sensor replacement. */
#define FC_CAL_REGION_BYTES 8192u
#define FC_CAL_DATA_BYTES 512u
#define FC_CAL_COMMIT_BYTES 256u
#define FC_CAL_MAGIC 0x4C414351u
#define FC_CAL_COMMIT_MAGIC 0x54494D43u
#define FC_CAL_SCHEMA 1u
enum { CAL_ACCEL=1u, CAL_GYRO=2u, CAL_MAG=4u, CAL_VOLTAGE=8u, CAL_CURRENT=16u, CAL_LIDAR=32u };
typedef struct {
    uint32_t mask;
    imu_calibration_t imu;
    mag_calibration_t mag;
    linear_cal_t voltage, current, lidar;
} calibration_bundle_t;
typedef enum { CAL_EMPTY, CAL_LOADED, CAL_RECALIBRATE, CAL_IO_ERROR } calibration_store_status_t;
static calibration_bundle_t live_calibration;
static calibration_store_status_t calibration_store_status;
static imu_stationary_cal_t boot_gyro_capture;
static bool boot_gyro_verified;
static uint64_t calibration_board_id;
static bool calibration_dirty;
static unsigned calibration_auto_save_failures;
_Static_assert(sizeof(float)==4 && FLT_MANT_DIG==24, "Flash schema requires binary32 float");
_Static_assert(sizeof(double)==8 && DBL_MANT_DIG==53, "Flash schema requires binary64 double");
_Static_assert(FLASH_SECTOR_SIZE==4096 && FLASH_PAGE_SIZE==256, "Unexpected flash geometry");
_Static_assert(PICO_FLASH_SIZE_BYTES > FC_CAL_REGION_BYTES, "Flash too small for calibration region");

static uint32_t cal_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static void cal_put32(uint8_t *p, uint32_t x) {
    for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(x>>(8*i));
}
static uint64_t cal_u64(const uint8_t *p) {
    uint64_t x=0; for(unsigned i=0;i<8;i++) x|=(uint64_t)p[i]<<(8*i); return x;
}
static void cal_put64(uint8_t *p, uint64_t x) {
    for(unsigned i=0;i<8;i++) p[i]=(uint8_t)(x>>(8*i));
}
static void cal_float_write(uint8_t *p, float x) {
    uint32_t bits; memcpy(&bits,&x,4); cal_put32(p,bits);
}
static float cal_float_read(const uint8_t *p) {
    uint32_t bits=cal_u32(p); float x; memcpy(&x,&bits,4); return x;
}
static void cal_double_write(uint8_t *p, double x) {
    uint64_t bits; memcpy(&bits,&x,8); cal_put64(p,bits);
}
static double cal_double_read(const uint8_t *p) {
    uint64_t bits=cal_u64(p); double x; memcpy(&x,&bits,8); return x;
}
static uint32_t cal_crc32(const uint8_t *p, size_t n) {
    uint32_t crc=0xFFFFFFFFu;
    for(size_t i=0;i<n;i++) {
        crc^=p[i];
        for(unsigned j=0;j<8;j++) crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
    }
    return ~crc;
}
static bool calibration_values_valid(const calibration_bundle_t *c) {
    if(c==NULL || (c->mask & ~63u)) return false;
    for(unsigned i=0;i<3;i++) {
        if((c->mask&CAL_ACCEL) && (!isfinite(c->imu.accel_bias_g[i])
            || fabsf(c->imu.accel_bias_g[i])>.25f || !isfinite(c->imu.accel_gain[i])
            || c->imu.accel_gain[i]<.5f || c->imu.accel_gain[i]>2.f
            || !isfinite(c->imu.accel_bias_variance_g2[i]) || c->imu.accel_bias_variance_g2[i]<0
            || !isfinite(c->imu.accel_gain_variance[i]) || c->imu.accel_gain_variance[i]<0)) return false;
        if((c->mask&CAL_GYRO) && (c->imu.gyro_samples<500u
            || !isfinite(c->imu.gyro_bias_dps[i]) || fabsf(c->imu.gyro_bias_dps[i])>5.f
            || !isfinite(c->imu.gyro_variance_dps2[i]) || c->imu.gyro_variance_dps2[i]<0
            || c->imu.gyro_variance_dps2[i]>.01f
            || !isfinite(c->imu.gyro_bias_variance_dps2[i]) || c->imu.gyro_bias_variance_dps2[i]<0)) return false;
        if((c->mask&CAL_MAG) && (!isfinite(c->mag.offset_ut[i]) || fabsf(c->mag.offset_ut[i])>1000.f
            || !isfinite(c->mag.gain[i]) || c->mag.gain[i]<.5f || c->mag.gain[i]>2.f)) return false;
    }
    const linear_cal_t *linear[3]={&c->voltage,&c->current,&c->lidar};
    for(unsigned i=0;i<3;i++) if((c->mask&(CAL_VOLTAGE<<i)) &&
        (!isfinite(linear[i]->gain) || fabs(linear[i]->gain)<1e-12
        || !isfinite(linear[i]->offset) || !isfinite(linear[i]->residual_sigma)
        || linear[i]->residual_sigma<0)) return false;
    return true;
}
static void calibration_pack(uint8_t page[FC_CAL_DATA_BYTES], const calibration_bundle_t *c, uint32_t seq) {
    memset(page,0xFF,FC_CAL_DATA_BYTES);
    cal_put32(page,FC_CAL_MAGIC); cal_put32(page+4,FC_CAL_SCHEMA);
    cal_put32(page+8,seq); cal_put32(page+12,FC_CAL_PROFILE_ID);
    cal_put64(page+16,calibration_board_id); cal_put32(page+24,c->mask);
    cal_put32(page+28,c->imu.gyro_samples);
    const float *vectors[]={c->imu.accel_bias_g,c->imu.accel_gain,
        c->imu.accel_bias_variance_g2,c->imu.accel_gain_variance,
        c->imu.gyro_bias_dps,c->imu.gyro_variance_dps2,c->imu.gyro_bias_variance_dps2,
        c->mag.offset_ut,c->mag.gain};
    unsigned at=32;
    for(unsigned v=0;v<9;v++) for(unsigned a=0;a<3;a++,at+=4) cal_float_write(page+at,vectors[v][a]);
    const linear_cal_t *linear[]={&c->voltage,&c->current,&c->lidar};
    for(unsigned i=0;i<3;i++) {
        cal_double_write(page+at,linear[i]->gain); at+=8;
        cal_double_write(page+at,linear[i]->offset); at+=8;
        cal_double_write(page+at,linear[i]->residual_sigma); at+=8;
    }
    cal_put32(page+508,cal_crc32(page,508));
}
static bool calibration_unpack(const uint8_t *page, calibration_bundle_t *c) {
    if(cal_u32(page+12)!=FC_CAL_PROFILE_ID || cal_u64(page+16)!=calibration_board_id) return false;
    *c=(calibration_bundle_t){0}; c->mask=cal_u32(page+24); c->imu.gyro_samples=cal_u32(page+28);
    float *vectors[]={c->imu.accel_bias_g,c->imu.accel_gain,
        c->imu.accel_bias_variance_g2,c->imu.accel_gain_variance,
        c->imu.gyro_bias_dps,c->imu.gyro_variance_dps2,c->imu.gyro_bias_variance_dps2,
        c->mag.offset_ut,c->mag.gain};
    unsigned at=32;
    for(unsigned v=0;v<9;v++) for(unsigned a=0;a<3;a++,at+=4) vectors[v][a]=cal_float_read(page+at);
    linear_cal_t *linear[]={&c->voltage,&c->current,&c->lidar};
    for(unsigned i=0;i<3;i++) {
        linear[i]->gain=cal_double_read(page+at); at+=8;
        linear[i]->offset=cal_double_read(page+at); at+=8;
        linear[i]->residual_sigma=cal_double_read(page+at); at+=8;
        linear[i]->valid=(c->mask&(CAL_VOLTAGE<<i))!=0;
    }
    c->imu.accel_valid=(c->mask&CAL_ACCEL)!=0; c->imu.gyro_valid=(c->mask&CAL_GYRO)!=0;
    c->mag.valid=(c->mask&CAL_MAG)!=0;
    return calibration_values_valid(c);
}
static uint32_t calibration_offset(unsigned slot) {
    return PICO_FLASH_SIZE_BYTES-FC_CAL_REGION_BYTES+slot*FLASH_SECTOR_SIZE;
}
static const uint8_t *calibration_page(unsigned slot) {
    return (const uint8_t *)(XIP_BASE+calibration_offset(slot));
}
static bool calibration_raw_committed(const uint8_t *p) {
    const uint8_t *commit=p+FC_CAL_DATA_BYTES;
    const uint32_t crc=cal_u32(p+508);
    return cal_u32(p)==FC_CAL_MAGIC && cal_u32(p+4)==FC_CAL_SCHEMA
        && crc==cal_crc32(p,508) && cal_u32(commit)==FC_CAL_COMMIT_MAGIC
        && cal_u32(commit+4)==cal_u32(p+8) && cal_u32(commit+8)==crc
        && cal_u32(commit+12)==~crc;
}
static int calibration_latest_slot(void) {
    bool a=calibration_raw_committed(calibration_page(0)),b=calibration_raw_committed(calibration_page(1));
    if(!a) return b?1:-1;
    if(!b) return 0;
    uint32_t delta=cal_u32(calibration_page(1)+8)-cal_u32(calibration_page(0)+8);
    return delta!=0 && delta<0x80000000u ? 1:0;
}
static void calibration_boot_load(void) {
    pico_unique_board_id_t id; pico_get_unique_board_id(&id); calibration_board_id=cal_u64(id.id);
    live_calibration=(calibration_bundle_t){0}; boot_gyro_verified=false;
    boot_gyro_capture=(imu_stationary_cal_t){0}; calibration_dirty=false;
    int slot=calibration_latest_slot();
    calibration_store_status=CAL_EMPTY;
    if(slot>=0) {
        calibration_bundle_t decoded;
        if(calibration_unpack(calibration_page((unsigned)slot),&decoded)) {
            live_calibration=decoded;
            calibration_store_status=decoded.mask ? CAL_LOADED:CAL_RECALIBRATE;
        } else calibration_store_status=CAL_RECALIBRATE;
    }
}
typedef struct { uint32_t offset; uint8_t data[FC_CAL_DATA_BYTES],commit[FC_CAL_COMMIT_BYTES]; } calibration_write_t;
static void __not_in_flash_func(calibration_flash_write)(void *arg) {
    calibration_write_t *job=(calibration_write_t *)arg;
    flash_range_erase(job->offset,FLASH_SECTOR_SIZE);
    flash_range_program(job->offset,job->data,FC_CAL_DATA_BYTES);
    flash_range_program(job->offset+FC_CAL_DATA_BYTES,job->commit,FC_CAL_COMMIT_BYTES);
}
bool fc_calibration_persist(bool disarmed) {
    extern char __flash_binary_end;
    if(!disarmed || get_core_num()!=0 || !multicore_lockout_victim_is_initialized(1)
        || (uintptr_t)&__flash_binary_end > XIP_BASE+calibration_offset(0)
        || !calibration_values_valid(&live_calibration)) return false;
    int old=calibration_latest_slot();
    uint32_t seq=old<0 ? 1u:cal_u32(calibration_page((unsigned)old)+8)+1u;
    calibration_write_t job;
    job.offset=calibration_offset(old==0 ? 1u:0u);
    calibration_pack(job.data,&live_calibration,seq);
    /* Unchanged fits do not spend an erase cycle. Sequence/CRC are intentionally excluded. */
    if(old>=0 && memcmp(calibration_page((unsigned)old)+12,job.data+12,496)==0) {
        calibration_dirty=false;
        calibration_store_status=live_calibration.mask ? CAL_LOADED:CAL_RECALIBRATE;
        return true;
    }
    memset(job.commit,0xFF,sizeof(job.commit));
    cal_put32(job.commit,FC_CAL_COMMIT_MAGIC); cal_put32(job.commit+4,seq);
    uint32_t crc=cal_u32(job.data+508); cal_put32(job.commit+8,crc); cal_put32(job.commit+12,~crc);
    int rc=flash_safe_execute(calibration_flash_write,&job,500u);
    /* A timeout may occur after writing; reread integrity before reporting success. */
    const uint8_t *written=(const uint8_t *)(XIP_BASE+job.offset);
    if(rc!=PICO_OK || !calibration_raw_committed(written) || memcmp(written,job.data,FC_CAL_DATA_BYTES)!=0) {
        calibration_store_status=CAL_IO_ERROR; return false;
    }
    calibration_dirty=false;
    calibration_store_status=live_calibration.mask ? CAL_LOADED:CAL_RECALIBRATE;
    return true;
}
/* Capture/UI drivers can submit validated accel/gyro fits, then persist while disarmed.
 * Magnetic and linear fits use this complete-bundle entry point as well. */
bool fc_calibration_install(const calibration_bundle_t *fit, bool disarmed) {
    if(!disarmed || !calibration_values_valid(fit)) return false;
    live_calibration=*fit;
    live_calibration.imu.accel_valid=(fit->mask&CAL_ACCEL)!=0;
    live_calibration.imu.gyro_valid=(fit->mask&CAL_GYRO)!=0;
    live_calibration.mag.valid=(fit->mask&CAL_MAG)!=0;
    live_calibration.voltage.valid=(fit->mask&CAL_VOLTAGE)!=0;
    live_calibration.current.valid=(fit->mask&CAL_CURRENT)!=0;
    live_calibration.lidar.valid=(fit->mask&CAL_LIDAR)!=0;
    boot_gyro_verified=false; boot_gyro_capture=(imu_stationary_cal_t){0};
    calibration_dirty=true;
    calibration_auto_save_failures=0;
    return true;
}
static void calibration_process_imu(sensor_sample_t *sample, bool disarmed) {
    if(sample==NULL || !sample->imu_valid || sample->imu_calibrated) return;
    const float a[3]={sample->accel_x_g,sample->accel_y_g,sample->accel_z_g};
    const float g[3]={sample->gyro_x_dps,sample->gyro_y_dps,sample->gyro_z_dps};
    if(!boot_gyro_verified && disarmed) {
        if(!imu_stationary_cal_add(&boot_gyro_capture,a,g,true)) {
            boot_gyro_capture=(imu_stationary_cal_t){0};
        } else if(boot_gyro_capture.gyro.n>=500u) {
            imu_calibration_t fresh=live_calibration.imu;
            if(imu_gyro_cal_finish(&boot_gyro_capture,&fresh,true)) {
                bool refresh=(live_calibration.mask&CAL_GYRO)==0;
                for(unsigned i=0;i<3;i++) {
                    float uncertainty=sqrtf(fresh.gyro_bias_variance_dps2[i]
                                           +live_calibration.imu.gyro_bias_variance_dps2[i]);
                    if(fabsf(fresh.gyro_bias_dps[i]-live_calibration.imu.gyro_bias_dps[i])
                        >fmaxf(.05f,5.f*uncertainty)) refresh=true;
                }
                if(refresh) { live_calibration.imu=fresh; live_calibration.mask|=CAL_GYRO; calibration_dirty=true; }
                boot_gyro_verified=true;
                puts(refresh ? "calibration: gyro refreshed; save pending":"calibration: stored gyro verified at boot");
            } else boot_gyro_capture=(imu_stationary_cal_t){0};
        }
    }
    if(boot_gyro_verified && (live_calibration.mask&(CAL_ACCEL|CAL_GYRO))==(CAL_ACCEL|CAL_GYRO))
        (void)sensor_sample_apply_imu_calibration(sample,&live_calibration.imu);
}
static void calibration_console(int key, bool disarmed) {
    if(key=='c' || key=='C')
        printf("calibration: store=%u mask=0x%02lx gyro_verified=%u dirty=%u; missing accel needs six-face capture\n",
            (unsigned)calibration_store_status,(unsigned long)live_calibration.mask,boot_gyro_verified,calibration_dirty);
    if((key=='r' || key=='R') && disarmed) {
        calibration_bundle_t clear={0};
        (void)fc_calibration_install(&clear,true);
        /* A newest committed empty record prevents resurrection of an older fit. */
        puts(fc_calibration_persist(true) ? "calibration: cleared in flash; recapture required":"calibration: clear NOT persisted; retry");
    }
    if((key=='s' || key=='S') && disarmed) {
        calibration_auto_save_failures=0;
        if(live_calibration.mask==0) puts("calibration: nothing measured to save");
        else puts(fc_calibration_persist(true) ? "calibration: saved":"calibration: save failed");
    }
}

typedef struct {
    uint8_t bytes[QCOPTER_FRAME_SIZE];
    size_t length;
    uint64_t received_us;
} radio_rx_item_t;

static queue_t radio_rx_queue;

/* Nonblocking placeholder: connect this to the selected LoRa module driver. */
static bool radio_receive_frame(uint8_t *bytes, size_t capacity, size_t *length) {
    (void)bytes;
    (void)capacity;
    (void)length;
    return false;
}

static void slow_core(void) {
    if(!flash_safe_execute_core_init()) return;
    for (;;) {
        /*
         * Core 1 owns nonblocking radio and slower sensor acquisition.
         * The radio stub returns no data until a module driver is selected.
         */
        radio_rx_item_t item = {0};
        if (radio_receive_frame(item.bytes, sizeof(item.bytes), &item.length)
            && item.length <= sizeof(item.bytes)) {
            item.received_us = time_us_64();
            (void)queue_try_add(&radio_rx_queue, &item);
        }
        sleep_ms(1u);
    }
}
#endif

#ifndef QCOPTER_HOST_TEST
int main(void) {
    stdio_init_all();
    calibration_boot_load();
    (void)flash_safe_execute_core_init();
    queue_init(&radio_rx_queue, sizeof(radio_rx_item_t), 4u);
    multicore_launch_core1(slow_core);
    sleep_ms(1500);
    puts("QCopter RP2350 FC: 500 Hz loop, radio/sensors/ESC outputs not connected.");
    puts("Core 1 radio receive is a stub; outputs stay disarmed.");
    calibration_console('c',true);
    puts("USB commands: C=calibration status, R=clear/recalculate, S=save measured fits. Battery: warn35%, disarm20%.");

    pilot_link_state_t pilot_link = {0};
    uint64_t previous_us = time_us_64();
    absolute_time_t next_cycle = delayed_by_us(get_absolute_time(),
                                                FC_LOOP_PERIOD_US);
    uint32_t loop_count = 0u;
    uint32_t missed_deadlines = 0u;
    for (;;) {
        sleep_until(next_cycle);
        next_cycle = delayed_by_us(next_cycle, FC_LOOP_PERIOD_US);
        const uint64_t now_us = time_us_64();
        const uint64_t scheduled_us = to_us_since_boot(next_cycle)
                                    - FC_LOOP_PERIOD_US;
        if (now_us > scheduled_us + 100u) missed_deadlines++;
        const float dt = (float)(now_us - previous_us) * 1e-6f;
        previous_us = now_us;

        radio_rx_item_t item;
        while (queue_try_remove(&radio_rx_queue, &item))
            (void)pilot_link_accept(&pilot_link, item.bytes, item.length,
                                    item.received_us);

        /* Sensor acquisition is not implemented: safe zero outputs by design. */
        sensor_sample_t no_sensors = {0};
        /* Future driver must set imu_valid only for a fresh raw sample. No fake calibration input. */
        calibration_process_imu(&no_sensors,true);
        const pilot_command_t command = pilot_link_current(&pilot_link, now_us);
        const control_output_t status = flight_control_step(&no_sensors,
                                                             &command, dt);
        (void)status;
        int key=getchar_timeout_us(0);
        if(key>=0) calibration_console(key,true);
        /* Only a newly measured/changed fit is saved; never erase continuously. */
        static uint64_t last_calibration_save_attempt_us;
        if(calibration_dirty && live_calibration.mask!=0
            && calibration_auto_save_failures<3u
            && now_us-last_calibration_save_attempt_us>=5000000u) {
            last_calibration_save_attempt_us=now_us;
            if(!fc_calibration_persist(true) && ++calibration_auto_save_failures==3u)
                puts("calibration: automatic saves stopped after three failures; S retries manually");
        }
        static bool previous_warning,previous_disarm;
        if(battery_warning_active && !previous_warning) puts("BATTERY WARNING: SOC <=35% or voltage/current alert");
        if(battery_disarm_latched && !previous_disarm) puts("BATTERY DISARM: latched motor inhibit; no landing");
        previous_warning=battery_warning_active; previous_disarm=battery_disarm_latched;
        if (++loop_count == FC_LOOP_HZ) {
            printf("fc_loop=500Hz target; missed=%lu; command=%s; motors=DISABLED\n",
                   (unsigned long)missed_deadlines,
                   command.valid ? "fresh" : "failsafe");
            loop_count = 0u;
            missed_deadlines = 0u;
        }
    }
}
#endif
