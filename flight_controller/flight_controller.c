/*
 * QCopter flight-controller foundation.
 *
 * This file contains hardware-independent estimation and control logic so the
 * same C implementation can be connected to a future Isaac Sim HIL backend.
 * Hardware sensor and ESC drivers are intentionally not enabled: board pins,
 * ESC protocol, frame orientation, and motor order have not been confirmed.
 * Therefore this firmware remains disarmed and never produces motor outputs.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
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
    BATTERY_ACTION_LAND,
    BATTERY_ACTION_LANDED_DISARM,
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
    float low_soc_percent;      /* request landing at/below this estimate */
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
        && isfinite(c->low_soc_percent)
        && c->low_soc_percent >= 0.0f && c->low_soc_percent <= 100.0f
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

    /* BMS alerts and cell-level undervoltage outrank pack-level warnings. */
    if (sample->bms_alert
        || (sample->cell_voltage_valid
            && sample->minimum_cell_voltage_v <= config->critical_cell_voltage_v)) {
        status->action = sample->aircraft_landed
            ? BATTERY_ACTION_LANDED_DISARM : BATTERY_ACTION_LAND;
    } else if (status->estimated_soc_percent <= config->low_soc_percent
        || status->filtered_pack_voltage_v <= config->low_pack_voltage_v
        || (sample->cell_voltage_valid
            && sample->minimum_cell_voltage_v <= config->low_cell_voltage_v)
        || sample->pack_current_a > config->maximum_discharge_current_a) {
        status->action = sample->aircraft_landed
            ? BATTERY_ACTION_LANDED_DISARM : BATTERY_ACTION_LAND;
    } else {
        status->action = BATTERY_ACTION_NORMAL;
    }
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
 * RAM; sensor drivers, guided UI, temperature modelling and flash storage are
 * not implemented. No test fixture is ever installed as a flight calibration.
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
} altitude_kf_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float throttle; /* [0,1], center requests zero vertical speed */
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
    .kp = 4.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t pitch_angle_pid = {
    .kp = 4.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t roll_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t pitch_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t yaw_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t vertical_rate_pid = {
    .kp = 0.08f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 0.2f
};
static float altitude_hold_reference_m;
static const float altitude_position_to_rate = 0.6f; /* placeholder */
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
    pid_reset(&roll_angle_pid);
    pid_reset(&pitch_angle_pid);
    pid_reset(&roll_rate_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
    pid_reset(&vertical_rate_pid);
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

static void angle_kf_update(angle_kf_t *kf, float gyro_dps,
                            float accel_angle_deg, float dt) {
    const float corrected_rate = gyro_dps - kf->gyro_bias_dps;
    kf->angle_deg += dt * corrected_rate;

    const float p00 = kf->p00;
    const float p01 = kf->p01;
    const float p10 = kf->p10;
    const float p11 = kf->p11;
    kf->p00 = p00 + dt * (dt * p11 - p01 - p10 + kf->gyro_noise);
    kf->p01 = p01 - dt * p11;
    kf->p10 = p10 - dt * p11;
    kf->p11 = p11 + kf->bias_walk * dt;

    const float innovation_variance = kf->p00 + kf->accel_noise;
    const float k0 = kf->p00 / innovation_variance;
    const float k1 = kf->p10 / innovation_variance;
    const float innovation = accel_angle_deg - kf->angle_deg;
    kf->angle_deg += k0 * innovation;
    kf->gyro_bias_dps += k1 * innovation;

    const float updated_p00 = kf->p00;
    const float updated_p01 = kf->p01;
    kf->p00 -= k0 * updated_p00;
    kf->p01 -= k0 * updated_p01;
    kf->p10 -= k1 * updated_p00;
    kf->p11 -= k1 * updated_p01;
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
    const float innovation_variance = kf->p00 + kf->baro_noise;
    const float k0 = kf->p00 / innovation_variance;
    const float k1 = kf->p10 / innovation_variance;
    const float innovation = height_m - kf->height_m;
    const float p00 = kf->p00;
    const float p01 = kf->p01;
    kf->height_m += k0 * innovation;
    kf->velocity_mps += k1 * innovation;
    kf->p00 -= k0 * p00;
    kf->p01 -= k0 * p01;
    kf->p10 -= k1 * p00;
    kf->p11 -= k1 * p01;
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
        altitude_kf_baro_update(&altitude_estimator, s->barometer_height_m);
    }

    if (cmd == NULL || !cmd->valid || !isfinite(cmd->roll_deg) || !isfinite(cmd->pitch_deg)
        || !isfinite(cmd->yaw_rate_dps) || !isfinite(cmd->throttle)) {
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
        pid_reset(&vertical_rate_pid);
        altitude_reference_initialized = true;
    }
    const float pilot_vertical_speed = (throttle - 0.5f)
                                     * 2.0f * FC_MAX_VERTICAL_SPEED_MPS;
    altitude_hold_reference_m += pilot_vertical_speed * dt;
    const float altitude_velocity_correction = clampf(
        altitude_position_to_rate
            * (altitude_hold_reference_m - altitude_estimator.height_m),
        -FC_MAX_VERTICAL_SPEED_MPS, FC_MAX_VERTICAL_SPEED_MPS);
    const float vertical_speed_setpoint = clampf(
        pilot_vertical_speed + altitude_velocity_correction,
        -FC_MAX_VERTICAL_SPEED_MPS, FC_MAX_VERTICAL_SPEED_MPS);
    const float collective_correction = pid_update(&vertical_rate_pid,
        vertical_speed_setpoint, altitude_estimator.velocity_mps, dt);
    const float collective = clampf(FC_HOVER_THROTTLE_PLACEHOLDER
                                    + collective_correction, 0.0f, 1.0f);

    /* Provisional X-frame math in logical order FL, FR, RR, RL. The result is
       for simulation only until motor order, spin directions, and frame axes
       are verified. No PWM/DShot/ESC driver consumes these values. */
    out.motor[0] = clampf(collective + roll + pitch - yaw, 0.0f, 1.0f);
    out.motor[1] = clampf(collective - roll + pitch + yaw, 0.0f, 1.0f);
    out.motor[2] = clampf(collective - roll - pitch - yaw, 0.0f, 1.0f);
    out.motor[3] = clampf(collective + roll - pitch + yaw, 0.0f, 1.0f);

    /* There is deliberately no hardware arming/output path in this scaffold. */
    out.armed = false;
    return out;
}

#ifndef QCOPTER_HOST_TEST
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
    queue_init(&radio_rx_queue, sizeof(radio_rx_item_t), 4u);
    multicore_launch_core1(slow_core);
    sleep_ms(1500);
    puts("QCopter RP2350 FC: 500 Hz loop, radio/sensors/ESC outputs not connected.");
    puts("Core 1 radio receive is a stub; outputs stay disarmed.");

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
        const sensor_sample_t no_sensors = {0};
        const pilot_command_t command = pilot_link_current(&pilot_link, now_us);
        const control_output_t status = flight_control_step(&no_sensors,
                                                             &command, dt);
        (void)status;
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
