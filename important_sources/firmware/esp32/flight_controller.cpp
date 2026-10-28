/* IntelQCopter ESP32 branch: one C-style C++ firmware translation unit.
 * C-style control/calibration/estimation below; minimal C++ library boundary at end.
 * Required physical sensors: MPU6050 + BMP280 only, sharing Arduino Wire I2C.
 * No lidar/magnetometer is required; no absolute heading or onboard XY localization.
 * Physical arming/output is disabled. No scenario gains are certified for flight.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "fc_api.h"
#ifdef __cplusplus
static_assert(sizeof(float)==4, "NVS schema requires a 32-bit float");
#else
_Static_assert(sizeof(float)==4, "NVS schema requires a 32-bit float");
#endif
static uint16_t qcopter_get_u16(const uint8_t *p) {
    return (uint16_t)p[0]|(uint16_t)((uint16_t)p[1]<<8);
}
/* 1. Battery estimation and requested warning/disarm policy.
 * Monitor math is retained for the future aircraft voltage/current acquisition. */

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
    *status = (battery_status_t){};
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

/* 2. Sensor calibration, uncertainty and pressure compensation. */
#define FC_LOOP_HZ 500.0f
#define FC_LOOP_PERIOD_US 2000u
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f
#define FC_HOVER_THROTTLE_PLACEHOLDER 0.50f
#define FC_MAX_VERTICAL_SPEED_MPS 2.0f

/* SENSOR CALIBRATION -------------------------------------------------------
 * Physical units after sensor factory conversion: g, deg/s, microtesla, Pa.
 * These collectors accept fresh samples only while disarmed. Results live in
 * RAM while collecting; the ESP32 platform persists versioned NVS records.
 * Temperature modelling and physical calibration validation remain unfinished. No synthetic fixture is installed as a flight calibration.
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
        .t1 = qcopter_get_u16(bytes), .p1 = qcopter_get_u16(bytes + 6),
        .t2 = (int16_t)qcopter_get_u16(bytes + 2), .t3 = (int16_t)qcopter_get_u16(bytes + 4),
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

/* 3. Controller and estimator state; parameters are provisional. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_measurement;
    float integrator_limit;
    float output_limit;
    bool initialized;
} fc_pid_t;

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
    float boost;    /* RT [0,1]: extra climb/collective allowance */
    float reduce;   /* LT [0,1]: symmetric reduced climb/collective allowance */
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

static const angle_kf_t angle_estimator_defaults = {
    .p00 = 1.0f, .p11 = 0.01f,
    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
};
static angle_kf_t roll_estimator, pitch_estimator;
static const altitude_kf_t altitude_estimator_defaults = {
    .p00 = 1.0f, .p11 = 1.0f,
    .accel_noise = 1.5f, .baro_noise = 2.0f
};
static altitude_kf_t altitude_estimator;

/* Gains are placeholders for simulation/bench tuning, not flight values. */
static fc_pid_t roll_angle_pid = {
    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static fc_pid_t pitch_angle_pid = {
    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static fc_pid_t roll_rate_pid = {
    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,
    .integrator_limit = 15.0f, .output_limit = 0.35f
};
static fc_pid_t pitch_rate_pid = {
    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,
    .integrator_limit = 15.0f, .output_limit = 0.35f
};
static fc_pid_t yaw_rate_pid = {
    .kp = 0.005375614f, .ki = 0.008362066f, .kd = 0.0f,
    .integrator_limit = 8.0f, .output_limit = 0.35f
};
static fc_pid_t altitude_pid = {
    .kp = 0.108045f, .ki = 0.03087f, .kd = 0.1225f,
    .integrator_limit = 2.0f, .output_limit = 0.35f
};
#ifdef FC_BUILD_DLL
static float sim_commands[4];
static uint32_t sim_control_calls;
#endif
/* A localization observation is a sensor input, never a motor/pose override.
 * Horizontal control stays here in the C FC, including in FC-only simulation.
 * The real IMU/barometer alone cannot supply this drift-free observation. */
static struct {
    float x,y,vx,vy,yaw,target_x,target_y,ax,ay;
    uint64_t acquired_us;
    bool present,holding,fresh;
} horizontal;
static bool accel_angle_correction_enabled=true;
static float altitude_hold_reference_m;
static float hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;
static float boost_state;
static fc_debug_t debug={.level=2,.gentle=1};
static fc_profile_t execution_profile;
/* Status sigma changes only with the calibration variances, not at 500 Hz. */
static float gyro_variance_cache[3],gyro_sigma_cache[3];
static fc_trace_t debug_trace[64];
static uint32_t debug_trace_next,debug_trace_count;
static uint64_t debug_trace_us;
static bool gentle_profile=true;
static const float armed_idle=.10f,normal_ceiling=.45f,boost_ceiling=.65f; /* User 2026-10-28 ESC command limits; not measured RPM. */
static bool altitude_reference_initialized;
static bool user_disarm_latched;

static float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void pid_reset(fc_pid_t *pid) {
    pid->integrator = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->initialized = false;
}

static void reset_control_state(void) {
    horizontal.holding=false;
    boost_state=0;
    pid_reset(&roll_angle_pid);
    pid_reset(&pitch_angle_pid);
    pid_reset(&roll_rate_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
    pid_reset(&altitude_pid);
    altitude_reference_initialized = false;
}

/* 4. Discrete PID and Kalman update equations.
 * Derivative is taken on the measurement to avoid setpoint kick. */
static float pid_update(fc_pid_t *pid, float setpoint, float measurement, float dt) {
    if (!(dt > 0.0f) || dt > FC_MAX_CONTROL_DT) {
        pid_reset(pid);
        return 0.0f;
    }
    const float error = setpoint - measurement;
    const float derivative = pid->initialized && pid->kd != 0.0f
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
    kf->innovation=accel_angle_deg-kf->angle_deg;
    /* An accelerometer measures gravity PLUS translation. During measured
     * manual horizontal acceleration, treating it as gravity makes a held
     * angle slowly creep. Keep gyro prediction/covariance propagation, but
     * withhold the invalid gravity correction until translation settles.
     * This relies on explicitly fresh external localization; no IMU-only
     * drift-free attitude/position claim is made. Navigation is unaffected. */
    if(!accel_angle_correction_enabled){kf->gain_angle=kf->gain_bias=0;return;}
    kf->gain_angle=kf->p00/kf->innovation_variance;
    kf->gain_bias=kf->p10/kf->innovation_variance;
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

/* 5. One control step and F450 X allocation.
 * This function is currently a math/control core only. Mixer motor order and
 * signs must be validated against the chosen frame before its outputs are used.
 */
static control_output_t flight_control_step(const sensor_sample_t *s,
                                             const pilot_command_t *cmd,
                                             float dt) {
    control_output_t out = { .motor = {0.0f, 0.0f, 0.0f, 0.0f},
                             .armed = false, .sensor_fault = false };
    if (battery_disarm_latched || user_disarm_latched) {
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
    const float boost_target=clampf(cmd->boost-cmd->reduce,-1,1);
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
                                     * 2.0f * (FC_MAX_VERTICAL_SPEED_MPS+fabsf(boost))
                                     +boost*.6f;
    altitude_hold_reference_m += pilot_vertical_speed * dt;
    altitude_hold_reference_m=clampf(altitude_hold_reference_m,
        altitude_estimator.height_m-2.0f,altitude_estimator.height_m+2.0f);
#ifdef FC_BUILD_DLL
    sim_commands[0]=roll_setpoint;sim_commands[1]=pitch_setpoint;
    sim_commands[2]=yaw_rate_setpoint;sim_commands[3]=pilot_vertical_speed;
#endif
    const float altitude_integrator_before=altitude_pid.integrator;
    const float collective_correction=altitude_pid_update(altitude_hold_reference_m,
        altitude_estimator.height_m,pilot_vertical_speed,altitude_estimator.velocity_mps,dt);
    const float tilt_factor=fmaxf(0.5f,cosf(roll_estimator.angle_deg*FC_DEG_TO_RAD)
                                     *cosf(pitch_estimator.angle_deg*FC_DEG_TO_RAD));
    const float ceiling=normal_ceiling+(boost_ceiling-normal_ceiling)*clampf(boost,0,1);
    const float requested=(hover_collective+collective_correction)/tilt_factor;
    const float collective=clampf(requested,armed_idle,ceiling);
    /* Allocation-aware altitude antiwindup: undo integration when the actual
     * collective ceiling opposes altitude error. Allow unwinding. */
    if((requested>ceiling && altitude_hold_reference_m>altitude_estimator.height_m)
       ||(requested<0 && altitude_hold_reference_m<altitude_estimator.height_m))
        altitude_pid.integrator=altitude_integrator_before;

    /* Provisional X-frame math in logical order FL, FR, RR, RL. The result is
       for simulation only until motor order, spin directions, and frame axes
       are verified. No PWM/DShot/ESC driver consumes these values. */
    float correction[4]={roll-pitch-yaw,-roll-pitch+yaw,-roll+pitch-yaw,roll+pitch+yaw};
    float low=correction[0],high=correction[0];
    for(unsigned i=1;i<4;i++){low=fminf(low,correction[i]);high=fmaxf(high,correction[i]);}
    // Uniformly scale only infeasible torque combinations, then shift collective
    // into the feasible interval. Independent clipping would distort torques.
    const float span=ceiling-armed_idle;
    const float scale=high-low>span ? span/(high-low) : 1;
    const float allocated=clampf(collective,armed_idle-low*scale,ceiling-high*scale);
    if(debug.level){
        debug.saturation_cycles+=requested<0 || requested>ceiling || fabsf(allocated-collective)>1e-6f;
        debug.torque_scaled_cycles+=scale<.99999f;
        if(debug.level>=2){debug.collective_requested=requested;debug.collective_allocated=allocated;
            debug.collective_ceiling=ceiling;debug.torque_scale=scale;}
    }
    for(unsigned i=0;i<4;i++)out.motor[i]=clampf(allocated+scale*correction[i],armed_idle,ceiling);

    /* There is deliberately no hardware arming/output path in this scaffold. */
    out.armed = false;
    return out;
}

/* 6. ESP32-facing C API and calibration serialization.
 * Only the control task owns these states. */
static imu_calibration_t saved_imu;
static imu_stationary_cal_t verification;
static cal_stats3_t accel_faces[6];
static baro_reference_cal_t pressure_capture;
static baro_calibration_t ground_reference;
static uint32_t last_pressure_sequence;
static uint64_t calibration_board_id;
static bool gyro_verified,cal_dirty;
static int8_t capture_face=-1;
static fc_pilot_t received_pilot;
static uint8_t selected_mode,held_gesture;
static bool companion_enabled=FC_ENABLE_COMPANION!=0;
static uint64_t gesture_started_us,previous_report_us,companion_received_us;
static uint32_t received_reports,mode_events,companion_sequence;
static bool gesture_fired,have_companion;
static bool navigation_denied_hold;
static fc_link_stats_t link_stats;
static bool companion_is_fresh(uint64_t now) {
    return companion_enabled && have_companion && now>=companion_received_us
        && now-companion_received_us<=150000u;
}
void fc_link_snapshot(fc_link_stats_t *out){if(out)*out=link_stats;}
static float companion_values[4];

static uint32_t record_u32(const uint8_t *p) {
    return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;
}
static void record_put32(uint8_t *p,uint32_t v) {
    for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i));
}
static uint32_t record_crc(const uint8_t *p,unsigned n) {
    uint32_t crc=0xffffffffu;
    for(unsigned i=0;i<n;i++) {
        crc^=p[i];
        for(unsigned j=0;j<8;j++) crc=(crc>>1)^((crc&1u)?0xedb88320u:0);
    }
    return ~crc;
}
static bool imu_fit_valid(const imu_calibration_t *c) {
    if(c->gyro_valid && c->gyro_samples<500u) return false;
    for(unsigned i=0;i<3;i++) {
        if(c->gyro_valid && (!isfinite(c->gyro_bias_dps[i]) || fabsf(c->gyro_bias_dps[i])>5
            || !isfinite(c->gyro_variance_dps2[i]) || c->gyro_variance_dps2[i]<0
            || c->gyro_variance_dps2[i]>.01f || !isfinite(c->gyro_bias_variance_dps2[i])
            || c->gyro_bias_variance_dps2[i]<0)) return false;
        if(c->accel_valid && (!isfinite(c->accel_bias_g[i]) || fabsf(c->accel_bias_g[i])>.25f
            || !isfinite(c->accel_gain[i]) || c->accel_gain[i]<.5f || c->accel_gain[i]>2
            || !isfinite(c->accel_bias_variance_g2[i]) || c->accel_bias_variance_g2[i]<0
            || !isfinite(c->accel_gain_variance[i]) || c->accel_gain_variance[i]<0)) return false;
    }
    return true;
}
bool fc_horizontal_observation(const float v[5],uint64_t acquired_us) {
    if(!v){horizontal.present=horizontal.holding=horizontal.fresh=false;return false;}
    for(unsigned i=0;i<5;i++)if(!isfinite(v[i])){
        horizontal.present=horizontal.holding=horizontal.fresh=false;return false;
    }
    if(fabsf(v[0])>100000 || fabsf(v[1])>100000 || fabsf(v[2])>50 || fabsf(v[3])>50
       || fabsf(v[4])>100000 || (horizontal.present && acquired_us<horizontal.acquired_us))return false;
    if(horizontal.present && acquired_us>horizontal.acquired_us) {
        const float h=(acquired_us-horizontal.acquired_us)*1e-6f;
        if(h<=.1f) {
            horizontal.ax+=.25f*(clampf((v[2]-horizontal.vx)/h,-20,20)-horizontal.ax);
            horizontal.ay+=.25f*(clampf((v[3]-horizontal.vy)/h,-20,20)-horizontal.ay);
        } else horizontal.ax=horizontal.ay=0;
    }
    horizontal.x=v[0];horizontal.y=v[1];horizontal.vx=v[2];horizontal.vy=v[3];horizontal.yaw=v[4];
    horizontal.acquired_us=acquired_us;horizontal.present=true;return true;
}
void fc_init(uint64_t board_id) {
    memset(&horizontal,0,sizeof(horizontal));accel_angle_correction_enabled=true;
    debug=(fc_debug_t){.level=2,.gentle=1};gentle_profile=true;
    debug_trace_next=debug_trace_count=0;debug_trace_us=0;
    execution_profile=(fc_profile_t){};
    for(unsigned i=0;i<3;i++){gyro_variance_cache[i]=-1;gyro_sigma_cache[i]=0;}
    calibration_board_id=board_id;
#ifdef FC_BUILD_DLL
    sim_control_calls=0;memset(sim_commands,0,sizeof(sim_commands));
    const float baseline[6][4]={{3,0,0,0},{3,0,0,0},{.000829338f,.003870245f,0,15},{.000829338f,.003870245f,0,15},{.005375614f,.008362066f,0,8},{.108045f,.03087f,.1225f,2}};
    fc_pid_t *reset_pids[]={&roll_angle_pid,&pitch_angle_pid,&roll_rate_pid,&pitch_rate_pid,&yaw_rate_pid,&altitude_pid};
    for(unsigned i=0;i<6;i++){reset_pids[i]->kp=baseline[i][0];reset_pids[i]->ki=baseline[i][1];reset_pids[i]->kd=baseline[i][2];reset_pids[i]->integrator_limit=baseline[i][3];}
#endif
    roll_estimator=angle_estimator_defaults;
    pitch_estimator=angle_estimator_defaults;
    altitude_estimator=altitude_estimator_defaults;
    hover_collective=FC_HOVER_THROTTLE_PLACEHOLDER;
    received_pilot=(fc_pilot_t){}; selected_mode=held_gesture=0; companion_enabled=FC_ENABLE_COMPANION!=0;
    gesture_started_us=previous_report_us=companion_received_us=0;
    received_reports=mode_events=companion_sequence=0;
    gesture_fired=have_companion=user_disarm_latched=navigation_denied_hold=false;
    link_stats=(fc_link_stats_t){};
    saved_imu=(imu_calibration_t){};
    verification=(imu_stationary_cal_t){};
    memset(accel_faces,0,sizeof(accel_faces));
    ground_reference=(baro_calibration_t){};
    pressure_capture=(baro_reference_cal_t){};
    last_pressure_sequence=0; gyro_verified=false; cal_dirty=false; capture_face=-1;
    (void)battery_policy_reset(true);
    reset_control_state();
}
bool fc_cal_load(const uint8_t record[FC_CAL_RECORD_BYTES]) {
    if(record==NULL || record_u32(record)!=0x45494351u || record_u32(record+4)!=1u
        || record_u32(record+8)!=FC_CAL_PROFILE || record_u32(record+12)!=(uint32_t)calibration_board_id
        || record_u32(record+16)!=(uint32_t)(calibration_board_id>>32)
        || record_u32(record+156)!=record_crc(record,156)) return false;
    uint32_t flags=record_u32(record+20);
    if(flags&~3u) return false;
    imu_calibration_t fit={};
    fit.accel_valid=(flags&1u)!=0; fit.gyro_valid=(flags&2u)!=0;
    fit.gyro_samples=record_u32(record+24);
    float *vectors[]={fit.accel_bias_g,fit.accel_gain,fit.accel_bias_variance_g2,
        fit.accel_gain_variance,fit.gyro_bias_dps,fit.gyro_variance_dps2,fit.gyro_bias_variance_dps2};
    unsigned offset=28;
    for(unsigned v=0;v<7;v++) for(unsigned a=0;a<3;a++,offset+=4) {
        uint32_t bits=record_u32(record+offset); memcpy(&vectors[v][a],&bits,4);
    }
    if(!imu_fit_valid(&fit)) return false;
    saved_imu=fit; gyro_verified=false; verification=(imu_stationary_cal_t){};
    return true;
}
bool fc_cal_take_pending(uint8_t record[FC_CAL_RECORD_BYTES]) {
    if(!cal_dirty || record==NULL || !imu_fit_valid(&saved_imu)) return false;
    memset(record,0xff,FC_CAL_RECORD_BYTES);
    record_put32(record,0x45494351u); record_put32(record+4,1u);
    record_put32(record+8,FC_CAL_PROFILE);
    record_put32(record+12,(uint32_t)calibration_board_id);
    record_put32(record+16,(uint32_t)(calibration_board_id>>32));
    record_put32(record+20,(saved_imu.accel_valid?1u:0u)|(saved_imu.gyro_valid?2u:0u));
    record_put32(record+24,saved_imu.gyro_samples);
    const float *vectors[]={saved_imu.accel_bias_g,saved_imu.accel_gain,saved_imu.accel_bias_variance_g2,
        saved_imu.accel_gain_variance,saved_imu.gyro_bias_dps,saved_imu.gyro_variance_dps2,
        saved_imu.gyro_bias_variance_dps2};
    unsigned offset=28;
    for(unsigned v=0;v<7;v++) for(unsigned a=0;a<3;a++,offset+=4) {
        uint32_t bits; memcpy(&bits,&vectors[v][a],4); record_put32(record+offset,bits);
    }
    record_put32(record+156,record_crc(record,156));
    cal_dirty=false;
    return true;
}
void fc_cal_command(char key) {
    if(key=='R' || key=='r') {
        saved_imu=(imu_calibration_t){}; verification=(imu_stationary_cal_t){};
        memset(accel_faces,0,sizeof(accel_faces)); gyro_verified=false; capture_face=-1;
        cal_dirty=true; reset_control_state();
    } else if(key=='S' || key=='s') cal_dirty=true;
    else if(key>='0' && key<='5') {
        capture_face=(int8_t)(key-'0');
        accel_faces[(unsigned)capture_face]=(cal_stats3_t){};
    }
}
static void update_calibration(const fc_sample_t *s) {
    if(!s->imu_valid) return;
    if(!gyro_verified) {
        if(!imu_stationary_cal_add(&verification,s->accel_g,s->gyro_dps,true))
            verification=(imu_stationary_cal_t){};
        else if(verification.gyro.n>=500u) {
            imu_calibration_t fresh=saved_imu;
            if(imu_gyro_cal_finish(&verification,&fresh,true)) {
                bool refresh=!saved_imu.gyro_valid;
                for(unsigned i=0;i<3;i++)
                    if(fabsf(fresh.gyro_bias_dps[i]-saved_imu.gyro_bias_dps[i])
                        >fmaxf(.05f,5*sqrtf(fresh.gyro_bias_variance_dps2[i]+saved_imu.gyro_bias_variance_dps2[i])))
                        refresh=true;
                if(refresh) { saved_imu=fresh; cal_dirty=true; }
                gyro_verified=true;
            } else verification=(imu_stationary_cal_t){};
        }
    }
    if(capture_face>=0) {
        unsigned face=(unsigned)capture_face;
        (void)imu_accel_face_add(accel_faces,face,s->accel_g,true);
        if(accel_faces[face].n>=200u) {
            capture_face=-1;
            if(imu_accel_cal_finish(accel_faces,&saved_imu,true)) {
                cal_dirty=true; reset_control_state();
            }
        }
    }
}
void fc_set_estimated_soc(float percent) {
    if(isfinite(percent) && percent>=0 && percent<=100) battery_apply_soc_policy(percent);
}
void fc_step(const fc_sample_t *raw,const fc_pilot_t *pilot,uint64_t now,float dt,fc_status_t *status) {
    if(status==NULL) {reset_control_state();return;}
    *status=(fc_status_t){};
    if(raw==NULL || pilot==NULL || !isfinite(dt) || dt<=0 || dt>FC_MAX_CONTROL_DT) {
        reset_control_state();status->sensor_fault=true;return;
    }
    update_calibration(raw);
    bool pressure_updated=raw->pressure_sequence!=last_pressure_sequence;
    float height=altitude_estimator.height_m;
    if(pressure_updated) {
        last_pressure_sequence=raw->pressure_sequence;
        if(raw->pressure_valid && isfinite(raw->pressure_pa)) {
            if(!ground_reference.valid) {
                (void)baro_reference_add(&pressure_capture,raw->pressure_pa,true);
                if(pressure_capture.n>=100u
                    && !baro_reference_finish(&pressure_capture,&ground_reference,3.0,true))
                    pressure_capture=(baro_reference_cal_t){};
            }
            if(ground_reference.valid) {
                double h,sigma;
                if(baro_relative_height(&ground_reference,raw->pressure_pa,&h,&sigma)) height=(float)h;
            }
        }
    }
    sensor_sample_t s={.accel_x_g=raw->accel_g[0],.accel_y_g=raw->accel_g[1],.accel_z_g=raw->accel_g[2],
        .gyro_x_dps=raw->gyro_dps[0],.gyro_y_dps=raw->gyro_dps[1],.gyro_z_dps=raw->gyro_dps[2],
        .barometer_height_m=height,.imu_valid=raw->imu_valid,
        .barometer_valid=raw->pressure_valid && ground_reference.valid,
        .barometer_updated=pressure_updated};
    if(gyro_verified && saved_imu.accel_valid)
        (void)sensor_sample_apply_imu_calibration(&s,&saved_imu);
    if(s.imu_calibrated) {
        float phi=roll_estimator.angle_deg*FC_DEG_TO_RAD,theta=pitch_estimator.angle_deg*FC_DEG_TO_RAD;
        const float sin_phi=sinf(phi),cos_phi=cosf(phi);
        const float sin_theta=sinf(theta),cos_theta=cosf(theta);
        s.vertical_acceleration_mps2=9.80665f*(-sin_theta*s.accel_x_g
            +sin_phi*cos_theta*s.accel_y_g+cos_phi*cos_theta*s.accel_z_g-1);
    }
    bool fresh=pilot->connected && now>=pilot->received_us
        && now-pilot->received_us<=250000u
        && isfinite(pilot->roll) && isfinite(pilot->pitch) && isfinite(pilot->yaw)
        && isfinite(pilot->vertical) && isfinite(pilot->boost) && isfinite(pilot->reduce);
    pilot_command_t cmd={.roll_deg=35*pilot->roll,.pitch_deg=35*pilot->pitch,
        .yaw_rate_dps=180*pilot->yaw,.throttle=.5f+.5f*pilot->vertical,.boost=pilot->boost,.reduce=pilot->reduce,.valid=fresh};
    horizontal.fresh=horizontal.present && now>=horizontal.acquired_us
        && now-horizontal.acquired_us<=100000u;
    /* A deflected right stick still commands ANGLE, not horizontal speed/rate.
     * On release capture the current point, briefly counter-tilt to brake,
     * then hold that point. No instantaneous stopping/position reset is used.
     * Yaw and vertical commands retain their own control loops while holding XY. */
    accel_angle_correction_enabled=!(selected_mode==0 && horizontal.fresh
        && hypotf(horizontal.ax,horizontal.ay)>.25f);
    const bool centered=fabsf(pilot->roll)<.001f && fabsf(pilot->pitch)<.001f;
    if(selected_mode==0 && fresh && !user_disarm_latched && gyro_verified
       && s.imu_calibrated && horizontal.fresh && altitude_estimator.height_m>.15f) {
        if(!centered)horizontal.holding=false;
        else {
            if(!horizontal.holding){horizontal.target_x=horizontal.x;horizontal.target_y=horizontal.y;horizontal.holding=true;}
            const float ax=clampf(.9f*(horizontal.target_x-horizontal.x)-1.9f*horizontal.vx,-1.2f,1.2f);
            const float ay=clampf(.9f*(horizontal.target_y-horizontal.y)-1.9f*horizontal.vy,-1.2f,1.2f);
            const float cy=cosf(horizontal.yaw),sy=sinf(horizontal.yaw);
            const float bx=cy*ax+sy*ay,by=-sy*ax+cy*ay;
            cmd.roll_deg=clampf(-atan2f(by,9.80665f)/FC_DEG_TO_RAD,-7,7);
            cmd.pitch_deg=clampf(atan2f(bx,9.80665f)/FC_DEG_TO_RAD,-7,7);
        }
    } else horizontal.holding=false;
    control_output_t result=flight_control_step(&s,&cmd,dt);
    memcpy(status->motor_demand,result.motor,sizeof(status->motor_demand));
    status->roll_deg=roll_estimator.angle_deg; status->pitch_deg=pitch_estimator.angle_deg;
    status->height_m=altitude_estimator.height_m; status->vertical_mps=altitude_estimator.velocity_mps;
    status->sensor_fault=result.sensor_fault; status->pilot_fresh=fresh;
    status->gyro_verified=gyro_verified; status->accel_calibrated=saved_imu.accel_valid;
    status->barometer_ready=ground_reference.valid; status->capturing_face=capture_face;
    for(unsigned i=0;i<3;i++) {
        const float variance=saved_imu.gyro_variance_dps2[i];
        if(variance!=gyro_variance_cache[i]) {
            gyro_variance_cache[i]=variance;
            gyro_sigma_cache[i]=sqrtf(fmaxf(0,variance));
        }
        status->gyro_sigma_dps[i]=gyro_sigma_cache[i];
    }
    for(unsigned f=0;f<6;f++) if(accel_faces[f].n>=200u) status->faces_done|=1u<<f;
    status->battery_warning=battery_warning_active; status->battery_disarm=battery_disarm_latched;
    status->flight_mode=selected_mode;status->user_disarm=user_disarm_latched;
    status->xbox_reports=received_reports;status->mode_events=mode_events;
}

/* 7. Xbox raw HID reports and bounded companion authority.
 * Holds require continuous fresh reports; gaps/release/conflicts restart timing.
 * Physical arming stays absent. A user disarm is latched until session restart. */
static float xbox_axis(uint16_t value) {
    float v=((int32_t)value-32768)/32768.0f;
    if(fabsf(v)<=.12f)return 0;
    return copysignf(fminf(1,(fabsf(v)-.12f)/.88f),v);
}
bool fc_xbox_report(const uint8_t *p,unsigned length,uint64_t now) {
    if(!p || length!=16u || qcopter_get_u16(p+8)>1023 || qcopter_get_u16(p+10)>1023 || p[12]>8
       ||(received_reports && now<previous_report_us)){if(debug.level)debug.hid_rejected++;return false;}
    bool continuous=received_reports && now>=previous_report_us && now-previous_report_us<=100000u;
    fc_pilot_t next={.roll=xbox_axis(qcopter_get_u16(p+4)),.pitch=-xbox_axis(qcopter_get_u16(p+6)),
        .yaw=-xbox_axis(qcopter_get_u16(p)),.vertical=-xbox_axis(qcopter_get_u16(p+2)),
        .boost=qcopter_get_u16(p+10)/1023.0f,.reduce=qcopter_get_u16(p+8)/1023.0f,
        .received_us=now,.connected=true};
    if(gentle_profile){
        next.roll=copysignf(next.roll*next.roll*(5.f/35),next.roll);
        next.pitch=copysignf(next.pitch*next.pitch*(5.f/35),next.pitch);
        next.yaw=copysignf(next.yaw*next.yaw*.25f,next.yaw);
        next.vertical=copysignf(next.vertical*next.vertical*.25f,next.vertical);
    }
    bool down=p[12]>=4 && p[12]<=6;
    uint8_t gesture=down && next.reduce>=.9f ? 3 : down && (p[13]&3u)==1 ? 1 : down && (p[13]&3u)==2 ? 2 : 0;
    const bool denied=gesture==2 && !companion_is_fresh(now);
    if(denied && !navigation_denied_hold)link_stats.navigation_denied++;
    navigation_denied_hold=denied;
    if(denied)gesture=0; /* No navigation selection until actual valid UART traffic. */
    if(!continuous || gesture!=held_gesture) {held_gesture=gesture;gesture_started_us=now;gesture_fired=false;}
    if(gesture && !gesture_fired && now>=gesture_started_us
        && now-gesture_started_us>=(gesture==3?2000000u:5000000u)) {
        gesture_fired=true;mode_events++;
        if(gesture==3)user_disarm_latched=true;
        else {selected_mode=gesture==2?1:0;reset_control_state();}
    }
    received_pilot=next;previous_report_us=now;received_reports++;
    return true;
}
void fc_configure_companion(bool enabled) {
    companion_enabled=enabled && FC_ENABLE_COMPANION;
    if(!companion_enabled){selected_mode=0;have_companion=false;link_stats.available=false;reset_control_state();}
}
static bool companion_parse(const uint8_t *p,unsigned length,uint64_t now) {
    if(!companion_enabled || (have_companion && now<companion_received_us))return false;
    if(!p || length!=FC_COMPANION_FRAME_BYTES || p[0]!='Q' || p[1]!='N' || p[2]!=1 || (p[3]!=1 && p[3]!=3)
        || record_u32(p+24)!=record_crc(p,24))return false;
    uint32_t seq=record_u32(p+4);
    if(have_companion && now>=companion_received_us && now-companion_received_us<1000000u
        && (int32_t)(seq-companion_sequence)<=0)return false;
    float values[4];
    for(unsigned i=0;i<4;i++){uint32_t bits=record_u32(p+8+4*i);memcpy(values+i,&bits,4);if(!isfinite(values[i]))return false;}
    if(fabsf(values[0])>35 || fabsf(values[1])>35 || fabsf(values[2])>180 || fabsf(values[3])>3)return false;
    memcpy(companion_values,values,sizeof(values));companion_sequence=seq;companion_received_us=now;have_companion=true;
    if((p[3]&2u) && selected_mode==1){user_disarm_latched=true;reset_control_state();}
    return true;
}
bool fc_companion_report(const uint8_t *p,unsigned length,uint64_t now) {
    const bool ok=companion_parse(p,length,now);if(!ok && debug.level)debug.uart_rejected++;return ok;
}
static void control_step_impl(const fc_sample_t *raw,uint64_t now,float dt,fc_status_t *status) {
#ifdef FC_BUILD_DLL
    sim_control_calls++;
#endif
    fc_pilot_t pilot=received_pilot;
    bool fresh=companion_is_fresh(now);link_stats.available=fresh;
    if(selected_mode==1 && !fresh) {
        selected_mode=0;mode_events++;link_stats.stable_fallbacks++;
        reset_control_state(); /* Recapture height and leave UART commands behind. */
    }
    if(selected_mode==1) {
        pilot.boost=pilot.reduce=0; /* Navigation owns vertical speed; triggers cannot silently modify it. */
        pilot.roll=fresh?companion_values[0]/35:0;pilot.pitch=fresh?companion_values[1]/35:0;
        pilot.yaw=fresh?companion_values[2]/180:0;pilot.vertical=fresh?companion_values[3]/2:0;
    }
    fc_step(raw,&pilot,now,dt,status);
    if(status)status->companion_fresh=fresh;
}
bool fc_debug_configure(unsigned level){if(level>3 || received_reports)return false;debug.level=(uint8_t)level;return true;}
bool fc_configure_gentle(bool gentle){if(received_reports)return false;gentle_profile=gentle;debug.gentle=gentle;return true;}
void fc_debug_snapshot(fc_debug_t *out){if(out)*out=debug;}
unsigned fc_debug_trace(fc_trace_t *out,unsigned capacity){
    if(!out)return 0;
    const unsigned n=capacity<debug_trace_count?capacity:debug_trace_count;
    const unsigned start=(debug_trace_next+64-debug_trace_count)%64;
    for(unsigned i=0;i<n;i++)out[i]=debug_trace[(start+i)%64];
    return n;
}
void fc_profile_snapshot(fc_profile_t *out){if(out)*out=execution_profile;}
void fc_profile_record(uint32_t elapsed){
    if(!debug.level)return;
    if(!execution_profile.samples || elapsed<execution_profile.minimum_us)execution_profile.minimum_us=elapsed;
    if(elapsed>execution_profile.maximum_us)execution_profile.maximum_us=elapsed;
    execution_profile.samples++;execution_profile.total_us+=elapsed;execution_profile.last_us=elapsed;
    if(elapsed>2000 && execution_profile.over_budget<UINT32_MAX)execution_profile.over_budget++;
    if(elapsed>debug.max_execution_us)debug.max_execution_us=elapsed;
    debug.deadline_misses+=elapsed>2000;
    static const uint32_t bins[]={50,100,250,500,1000,1500,2000};
    unsigned i=0;while(i<7 && elapsed>bins[i])i++;debug.execution_bins[i]++;
}
void fc_control_step(const fc_sample_t *raw,uint64_t now,float dt,fc_status_t *status){
    if(!raw || !status){reset_control_state();if(debug.level)debug.invalid_calls++;if(status){*status=(fc_status_t){};status->sensor_fault=true;}return;}
    control_step_impl(raw,now,dt,status);
    bool numeric=false;
    for(unsigned i=0;i<4;i++)numeric|=!isfinite(status->motor_demand[i]);
    numeric|=!isfinite(status->roll_deg)||!isfinite(status->pitch_deg)||!isfinite(status->height_m)||!isfinite(status->vertical_mps);
    if(numeric){memset(status->motor_demand,0,sizeof(status->motor_demand));status->sensor_fault=true;reset_control_state();}
    if(!debug.level)return;
    debug.steps++;debug.numeric_faults+=numeric;debug.sensor_faults+=status->sensor_fault;
    debug.stale_pilot+=!status->pilot_fresh;debug.stale_companion+=selected_mode==1&&!status->companion_fresh;
    if(isfinite(dt)&&dt>debug.max_dt_s)debug.max_dt_s=dt;
    if(debug.level>=2){
        debug.motor_min=debug.motor_max=status->motor_demand[0];
        for(unsigned i=1;i<4;i++){debug.motor_min=fminf(debug.motor_min,status->motor_demand[i]);debug.motor_max=fmaxf(debug.motor_max,status->motor_demand[i]);}
        debug.roll_nis=roll_estimator.innovation*roll_estimator.innovation/fmaxf(roll_estimator.innovation_variance,1e-9f);
        debug.pitch_nis=pitch_estimator.innovation*pitch_estimator.innovation/fmaxf(pitch_estimator.innovation_variance,1e-9f);
        debug.height_nis=altitude_estimator.innovation*altitude_estimator.innovation/fmaxf(altitude_estimator.innovation_variance,1e-9f);
        debug.min_covariance=fminf(fminf(roll_estimator.p00,roll_estimator.p11),fminf(fminf(pitch_estimator.p00,pitch_estimator.p11),fminf(altitude_estimator.p00,altitude_estimator.p11)));
    }
    if(debug.level>=3 && (now<debug_trace_us || now-debug_trace_us>=20000)){
        debug_trace_us=now;fc_trace_t *t=&debug_trace[debug_trace_next];
        *t=(fc_trace_t){.now_us=now,.roll=status->roll_deg,.pitch=status->pitch_deg,.height=status->height_m,
            .collective=debug.collective_allocated,.scale=debug.torque_scale,
            .flags=(status->sensor_fault?1u:0)|(status->pilot_fresh?2u:0)|(status->companion_fresh?4u:0)|(numeric?8u:0)};
        memcpy(t->motor,status->motor_demand,sizeof(t->motor));debug_trace_next=(debug_trace_next+1)%64;
        if(debug_trace_count<64)debug_trace_count++;
        debug.trace_written++;
    }
}
void fc_status_frame(const fc_status_t *s,uint8_t p[16]) {
    if(!s || !p)return;
    memset(p,0,16);p[0]='Q';p[1]='S';p[2]=1;p[3]=s->flight_mode;
    p[4]=s->user_disarm;p[5]=s->pilot_fresh;p[6]=s->companion_fresh;p[7]=companion_enabled?1:0;
    record_put32(p+8,s->xbox_reports);record_put32(p+12,record_crc(p,12));
}

#ifdef FC_BUILD_DLL
/* Simulator workbench only. Refuse changes after the first pilot packet.
 * 6*(Kp,Ki,Kd,integral bound), 3 angle-KF noises, 2 altitude-KF noises, hover.
 * Configuration is validated completely before any state is modified. */
FC_API bool fc_sim_configure(const float *v,unsigned n) {
    if(!v || n!=30 || received_reports)return false;
    for(unsigned i=0;i<n;i++)if(!isfinite(v[i])||v[i]<0)return false;
    for(unsigned i=0;i<6;i++){
        for(unsigned j=0;j<3;j++)if(v[4*i+j]>(i<2?20.0f:i==5?5.0f:1.0f))return false;
        if(v[4*i+3]>1000)return false;
    }
    for(unsigned i=24;i<29;i++)if(v[i]<=0||v[i]>1000)return false;
    if(v[29]<.1f||v[29]>.8f)return false;
    fc_pid_t *pids[]={&roll_angle_pid,&pitch_angle_pid,&roll_rate_pid,&pitch_rate_pid,&yaw_rate_pid,&altitude_pid};
    for(unsigned i=0;i<6;i++){pids[i]->kp=v[4*i];pids[i]->ki=v[4*i+1];pids[i]->kd=v[4*i+2];pids[i]->integrator_limit=v[4*i+3];}
    roll_estimator.gyro_noise=pitch_estimator.gyro_noise=v[24];
    roll_estimator.bias_walk=pitch_estimator.bias_walk=v[25];
    roll_estimator.accel_noise=pitch_estimator.accel_noise=v[26];
    altitude_estimator.accel_noise=v[27];altitude_estimator.baro_noise=v[28];hover_collective=v[29];
    reset_control_state();return true;
}
FC_API unsigned fc_sim_observe(float *v,unsigned n) {
    if(!v||n<23)return 0;
    float values[]={roll_estimator.angle_deg,pitch_estimator.angle_deg,altitude_estimator.height_m,altitude_estimator.velocity_mps,
      sqrtf(fmaxf(0,roll_estimator.p00)),sqrtf(fmaxf(0,pitch_estimator.p00)),sqrtf(fmaxf(0,altitude_estimator.p00)),sqrtf(fmaxf(0,altitude_estimator.p11)),
      roll_estimator.gain_angle,pitch_estimator.gain_angle,altitude_estimator.gain_height,altitude_estimator.gain_velocity,
      roll_estimator.innovation,pitch_estimator.innovation,altitude_estimator.innovation,altitude_estimator.innovation_variance,
      roll_estimator.gyro_bias_dps,pitch_estimator.gyro_bias_dps,sim_commands[0],sim_commands[1],sim_commands[2],sim_commands[3],(float)sim_control_calls};
    memcpy(v,values,sizeof(values));
    if(n>=31){float extra[]={horizontal.holding?1.f:0.f,horizontal.fresh?1.f:0.f,
        horizontal.target_x,horizontal.target_y,horizontal.x,horizontal.y,horizontal.vx,horizontal.vy};
        memcpy(v+23,extra,sizeof(extra));
        if(n>=33){v[31]=hypotf(horizontal.ax,horizontal.ay);v[32]=accel_angle_correction_enabled?1.f:0.f;return 33;}
        return 31;}
    return 23;
}
#endif


/* ===== ESP32 library boundary: excluded only by host/QEMU harness builds. ===== */
#if defined(ARDUINO) && !defined(FC_CORE_ONLY)
/* Minimal C++ boundary for Arduino, sensor drivers, NVS and Xbox BLE.
 * All estimators, six controllers and calibration math are above in this same file.
 * No ESC library, output pulse, or physical arming action exists in this adapter.
 */
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include <esp_timer.h>
#include "fc_api.h"
static void xbox_report(uint8_t*,size_t);
#define QCOPTER_XBOX_REPORT_HOOK xbox_report
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

static const int SDA_PIN=9,SCL_PIN=8,LED_PIN=42;
static const int MOTOR_PINS[4]={12,1,2,14}; // FL, FR, RR, RL; INPUT ONLY.
static const uint32_t CONTROL_PERIOD_US=2000;
static const int64_t PRESSURE_TIMEOUT_US=150000;
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;
// Existing library has constructor-selected addresses; keep both statically allocated.
static MPU6050 imu_devices[2]={MPU6050(0x68),MPU6050(0x69)};
static MPU6050 *mpu=&imu_devices[0];
static Adafruit_BMP280 bmp;
static bool imu_available,bmp_available;
static uint8_t imu_address=0,barometer_address=0;
static bool i2c_available;
static SemaphoreHandle_t i2c_mutex;
static TaskHandle_t control_handle;
static QueueHandle_t command_queue,calibration_queue;
static esp_timer_handle_t control_timer;
static portMUX_TYPE state_lock=portMUX_INITIALIZER_UNLOCKED;
static fc_status_t shared_status;
static fc_debug_t shared_debug;
static fc_profile_t shared_profile;
static fc_link_stats_t shared_link;
static uint32_t hid_drops,uart_drops,imu_failures;
static float shared_pressure;
static uint32_t pressure_sequence,missed_ticks,max_execution_us,late_cycles;
static int64_t pressure_received_us;
static bool pressure_ok;

struct HidPacket { uint8_t data[16]; uint64_t received_us; };
struct CompanionPacket { uint8_t data[FC_COMPANION_FRAME_BYTES]; uint64_t received_us; };
static QueueHandle_t hid_queue,companion_queue;
static void xbox_report(uint8_t *bytes,size_t length) {
    if(length!=16 || !hid_queue) return;
    HidPacket packet={};memcpy(packet.data,bytes,16);packet.received_us=esp_timer_get_time();
    // The control task owns all gesture timing and authority state.
    if(xQueueSend(hid_queue,&packet,0)!=pdTRUE){portENTER_CRITICAL(&state_lock);hid_drops++;portEXIT_CRITICAL(&state_lock);}
}
/* Boot only, before either core starts using Wire. Only these two sensors are
 * required. Never substitute dummy data when a required sensor is absent.
 * The register readback matches the scale factors in acquire_imu(). */
static bool initialize_required_sensors(void) {
    i2c_available=Wire.begin(SDA_PIN,SCL_PIN);
    if(!i2c_available) return false;
    Wire.setClock(400000); Wire.setTimeOut(2);
    for(unsigned i=0;i<2;i++) {
        imu_address=static_cast<uint8_t>(0x68+i); mpu=&imu_devices[i];
        if(!mpu->testConnection()) continue;
        mpu->initialize();
        mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
        mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_500);
        mpu->setDLPFMode(MPU6050_DLPF_BW_42); mpu->setRate(1);
        mpu->setIntDataReadyEnabled(true);
        imu_available=mpu->testConnection()
            && mpu->getFullScaleAccelRange()==MPU6050_ACCEL_FS_8
            && mpu->getFullScaleGyroRange()==MPU6050_GYRO_FS_500
            && mpu->getDLPFMode()==MPU6050_DLPF_BW_42 && mpu->getRate()==1;
        if(imu_available) break;
    }
    if(!imu_available) imu_address=0; // Log absence, not the last attempted address.
    for(unsigned i=0;i<2;i++) {
        const uint8_t address=static_cast<uint8_t>(0x76+i);
        if(!bmp.begin(address)) continue;
        bmp_available=true; barometer_address=address; break;
    }
    if(bmp_available) bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X1,Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::FILTER_X4,Adafruit_BMP280::STANDBY_MS_1);
    return imu_available && bmp_available;
}
static bool read_registers(uint8_t reg,uint8_t *data,size_t count) {
    Wire.beginTransmission(imu_address); Wire.write(reg);
    if(Wire.endTransmission(false)!=0) return false;
    if(Wire.requestFrom(imu_address,count,true)!=count) {
        while(Wire.available()) (void)Wire.read();
        return false;
    }
    for(size_t i=0;i<count;i++) data[i]=static_cast<uint8_t>(Wire.read());
    return true;
}
static int16_t imu_raw_word(const uint8_t *bytes,unsigned i) {
    return (int16_t)(((uint16_t)bytes[i]<<8)|bytes[i+1]);
}
static bool acquire_imu(fc_sample_t *sample) {
    if(!imu_available || xSemaphoreTake(i2c_mutex,pdMS_TO_TICKS(1))!=pdTRUE) return false;
    uint8_t ready=0,bytes[14];
    bool valid=read_registers(0x3a,&ready,1) && (ready&1)
        && read_registers(0x3b,bytes,sizeof(bytes));
    xSemaphoreGive(i2c_mutex);
    if(!valid) return false;
    for(unsigned i=0;i<3;i++) {
        sample->accel_g[i]=imu_raw_word(bytes,2*i)/4096.0f; // ±8g.
        sample->gyro_dps[i]=imu_raw_word(bytes,8+2*i)/65.5f; // ±500 deg/s.
    }
    return true; // Identity sensor-to-body transform is provisional and must be checked.
}
static void timer_tick(void*) {
    if(control_handle) xTaskNotifyGive(control_handle);
}
static void control_task(void*) {
    int64_t previous=esp_timer_get_time();
    for(;;) {
        uint32_t ticks=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        int64_t start=esp_timer_get_time();
        float dt=static_cast<float>(start-previous)*1e-6f; previous=start;
        fc_sample_t raw={}; fc_status_t status;
        portENTER_CRITICAL(&state_lock);
        raw.pressure_pa=shared_pressure;
        raw.pressure_sequence=pressure_sequence;
        raw.pressure_valid=pressure_ok && start>=pressure_received_us
            && start-pressure_received_us<=PRESSURE_TIMEOUT_US;
        portEXIT_CRITICAL(&state_lock);
        raw.imu_valid=acquire_imu(&raw);
        char command;
        for(unsigned budget=0;budget<8 && xQueueReceive(command_queue,&command,0)==pdTRUE;budget++) fc_cal_command(command);
        HidPacket report;
        for(unsigned budget=0;budget<8 && xQueueReceive(hid_queue,&report,0)==pdTRUE;budget++) fc_xbox_report(report.data,16,report.received_us);
        CompanionPacket companion;
        for(unsigned budget=0;budget<8 && xQueueReceive(companion_queue,&companion,0)==pdTRUE;budget++) fc_companion_report(companion.data,sizeof(companion.data),companion.received_us);
        fc_control_step(&raw,static_cast<uint64_t>(start),dt,&status);
        uint8_t record[FC_CAL_RECORD_BYTES];
        if(fc_cal_take_pending(record)) xQueueOverwrite(calibration_queue,record);
        // Deliberately no PWM/DShot writes, even when mathematical demands are nonzero.
        uint32_t elapsed=static_cast<uint32_t>(esp_timer_get_time()-start);
        portENTER_CRITICAL(&state_lock);
        fc_profile_record(elapsed);fc_debug_snapshot(&shared_debug);fc_profile_snapshot(&shared_profile);fc_link_snapshot(&shared_link);
        shared_status=status;imu_failures+=!raw.imu_valid;
        if(ticks>1) missed_ticks+=ticks-1;
        if(dt>.0021f) late_cycles++;
        if(elapsed>max_execution_us) max_execution_us=elapsed;
        portEXIT_CRITICAL(&state_lock);
    }
}
static void bluetooth_task(void*) {
    for(;;){xbox.onLoop();vTaskDelay(pdMS_TO_TICKS(2));}
}
static void slow_task(void*) {
    uint32_t last_pressure=0,last_log=0,last_save=0;
    uint8_t pending_record[FC_CAL_RECORD_BYTES]={};
    bool save_pending=false;
    unsigned failures=0;
    for(;;) {
        // BLE discovery/reconnection runs in its own low-priority task. Its
        // blocking library calls must not postpone pressure or UART servicing.
        // No synthetic freshness refresh on connection polling: only actual HID reports count.
        static uint8_t uart_frame[FC_COMPANION_FRAME_BYTES];static unsigned uart_used=0;
        for(unsigned budget=0;budget<112 && FC_ENABLE_COMPANION && Serial2.available();budget++) {
            uint8_t byte=static_cast<uint8_t>(Serial2.read());
            if(uart_used==0 && byte!='Q')continue;
            if(uart_used==1 && byte!='N'){uart_used=byte=='Q'?1:0;continue;}
            uart_frame[uart_used++]=byte;
            if(uart_used==sizeof(uart_frame)){
                CompanionPacket packet={};memcpy(packet.data,uart_frame,sizeof(uart_frame));packet.received_us=esp_timer_get_time();
                if(xQueueSend(companion_queue,&packet,0)!=pdTRUE)uart_drops++;
                uart_used=0;
            }
        }
        uint32_t now=millis();
        if(bmp_available && now-last_pressure>=20) {
            last_pressure=now;
            if(xSemaphoreTake(i2c_mutex,pdMS_TO_TICKS(2))==pdTRUE) {
                float pressure=bmp.readPressure();
                xSemaphoreGive(i2c_mutex);
                bool valid=isfinite(pressure) && pressure>=30000 && pressure<=110000;
                portENTER_CRITICAL(&state_lock);
                shared_pressure=pressure; pressure_ok=valid;
                if(valid) { pressure_sequence++; pressure_received_us=esp_timer_get_time(); }
                portEXIT_CRITICAL(&state_lock);
            }
        }
        if(Serial.available()) {
            char key=static_cast<char>(Serial.read());
            if(strchr("CcRrSs012345",key)) {
                if(xQueueSend(command_queue,&key,0)!=pdTRUE) Serial.println("command_queue=full");
                if(key=='C'||key=='c') Serial.println("C=status; R=clear; S=save; 0..5=+X,-X,+Y,-Y,+Z,-Z; motors=DISABLED");
            }
        }
        if(xQueueReceive(calibration_queue,pending_record,0)==pdTRUE) {
            save_pending=true; failures=0;
        }
        if(save_pending && failures<3 && now-last_save>=5000) {
            last_save=now;
            Preferences nvs;
            bool saved=false;
            if(nvs.begin("intelqcopter",false)) {
                saved=nvs.putBytes("imu-v1",pending_record,sizeof(pending_record))==sizeof(pending_record);
                uint8_t check[FC_CAL_RECORD_BYTES];
                saved=saved && nvs.getBytes("imu-v1",check,sizeof(check))==sizeof(check)
                    && memcmp(check,pending_record,sizeof(check))==0;
                nvs.end();
            }
            Serial.printf("calibration_nvs=%s; motors=DISABLED\n",saved?"saved_readback":"failed");
            if(saved) save_pending=false; else failures++;
        }
        if(now-last_log>=250) {
            last_log=now;
            fc_status_t s;fc_debug_t d;fc_profile_t profile;fc_link_stats_t link; uint32_t missed,execution,late,hd,im;
            portENTER_CRITICAL(&state_lock);
            s=shared_status;d=shared_debug;profile=shared_profile;link=shared_link;hd=hid_drops;im=imu_failures; missed=missed_ticks; execution=max_execution_us; late=late_cycles;
            portEXIT_CRITICAL(&state_lock);
            digitalWrite(LED_PIN,s.battery_warning || s.battery_disarm);
            Serial.printf("FC,ms=%lu,pilot=%d,gyro=%d,accel=%d,baro=%d,fault=%d,roll=%.3f,pitch=%.3f,h=%.3f,vz=%.3f,faces=0x%02x,capture=%d,missed=%lu,late=%lu,max_exec_us=%lu,warn35=%d,disarm20=%d,nvs_pending=%d,nvs_failures=%u,motors=DISABLED\n",
                static_cast<unsigned long>(now),s.pilot_fresh,s.gyro_verified,s.accel_calibrated,s.barometer_ready,
                s.sensor_fault,s.roll_deg,s.pitch_deg,s.height_m,s.vertical_mps,s.faces_done,s.capturing_face,
                static_cast<unsigned long>(missed),static_cast<unsigned long>(late),static_cast<unsigned long>(execution),
                s.battery_warning,s.battery_disarm,save_pending,failures);
            Serial.printf("MODE,selected=%u,user_disarm=%d,companion=%d,reports=%lu,events=%lu\n",s.flight_mode,s.user_disarm,s.companion_fresh,(unsigned long)s.xbox_reports,(unsigned long)s.mode_events);
            Serial.printf("DEBUG,level=%u,gentle=%u,steps=%lu,hid_reject=%lu,uart_reject=%lu,hid_drop=%lu,uart_drop=%lu,imu_fail=%lu,deadline_miss=%lu,collective=%.4f,ceiling=%.4f,torque_scale=%.4f,nis=%.3f/%.3f/%.3f,cov_min=%.6g,numeric_faults=%lu\n",
                d.level,d.gentle,(unsigned long)d.steps,(unsigned long)d.hid_rejected,(unsigned long)d.uart_rejected,
                (unsigned long)hd,(unsigned long)uart_drops,(unsigned long)im,(unsigned long)d.deadline_misses,
                d.collective_allocated,d.collective_ceiling,d.torque_scale,d.roll_nis,d.pitch_nis,d.height_nis,d.min_covariance,(unsigned long)d.numeric_faults);
            Serial.printf("LINK,available=%d,navigation_denied=%lu,stable_fallbacks=%lu\n",
                link.available,(unsigned long)link.navigation_denied,(unsigned long)link.stable_fallbacks);
            Serial.printf("PROFILE,samples=%llu,total_us=%llu,min_us=%lu,mean_us=%.2f,max_us=%lu,last_us=%lu,over_2ms=%lu,scope=loop_before_status_publish,motors=DISABLED\n",
                (unsigned long long)profile.samples,(unsigned long long)profile.total_us,
                (unsigned long)profile.minimum_us,profile.samples?(double)profile.total_us/profile.samples:0.0,
                (unsigned long)profile.maximum_us,(unsigned long)profile.last_us,(unsigned long)profile.over_budget);
            uint8_t telemetry[16];fc_status_frame(&s,telemetry);if(FC_ENABLE_COMPANION && Serial2.availableForWrite()>=16)Serial2.write(telemetry,sizeof(telemetry));
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
void setup() {
    Serial.begin(115200);
    if(FC_ENABLE_COMPANION){Serial2.setRxBufferSize(1024);
    Serial2.begin(115200,SERIAL_8N1,18,17);} // RX18, TX17; verify final PCB routing.
    for(unsigned i=0;i<4;i++) pinMode(MOTOR_PINS[i],INPUT_PULLDOWN);
    pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
    i2c_mutex=xSemaphoreCreateMutex();
    command_queue=xQueueCreate(8,sizeof(char));
    calibration_queue=xQueueCreate(1,FC_CAL_RECORD_BYTES);
    hid_queue=xQueueCreate(8,sizeof(HidPacket));companion_queue=xQueueCreate(8,sizeof(CompanionPacket));
    configASSERT(i2c_mutex && command_queue && calibration_queue && hid_queue && companion_queue);
    const bool required_sensors_present=initialize_required_sensors();
    Serial.printf("SENSORS,required=MPU6050+BMP280,i2c=%d,mpu=%d,mpu_addr=0x%02x,bmp=%d,bmp_addr=0x%02x,lidar=NOT_REQUIRED,magnetometer=NOT_REQUIRED,motors=DISABLED\n",
        i2c_available,imu_available,(unsigned)imu_address,bmp_available,(unsigned)barometer_address);
    if(!required_sensors_present)
        Serial.println("SENSOR_FAULT: check regulated3V3/commonGND/SDA9/SCL8 and MPU6050/BMP280 chip identity; fix wiring then reboot. No dummy sensor data.");
    Serial.println("AIDING,roll_pitch=accel_gyro,yaw=gyro_rate,height=relative_baro+vertical_accel,xy=NO_ONBOARD_LOCALIZATION");
    fc_init(ESP.getEfuseMac());
    fc_debug_configure(FC_DEBUG_LEVEL);
    uint8_t record[FC_CAL_RECORD_BYTES]; bool loaded=false;
    Preferences nvs;
    if(nvs.begin("intelqcopter",true)) {
        if(nvs.getBytesLength("imu-v1")==sizeof(record)
            && nvs.getBytes("imu-v1",record,sizeof(record))==sizeof(record)) loaded=fc_cal_load(record);
        nvs.end();
    }
    Serial.printf("IntelQCopter ESP32-S3/Xbox; imu=%d,bmp=%d,stored_cal=%d; MOTOR OUTPUTS DISABLED\n",
        imu_available,bmp_available,loaded);
    Serial.println("Mode2: LX=yaw, LY=vertical speed, RX=roll, RY=pitch, RT=boost, LT=reduce; Down+A/B 5s=stable/nav, Down+LT 2s=disarm; 0..5=six-face capture");
    xbox.begin();
    configASSERT(xTaskCreatePinnedToCore(control_task,"fc500",8192,NULL,5,&control_handle,0)==pdPASS);
    configASSERT(xTaskCreatePinnedToCore(slow_task,"sensor_uart_nvs",12288,NULL,2,NULL,1)==pdPASS);
    configASSERT(xTaskCreatePinnedToCore(bluetooth_task,"xbox_ble",8192,NULL,1,NULL,1)==pdPASS);
    esp_timer_create_args_t args={};
    args.callback=timer_tick; args.name="fc_tick"; args.dispatch_method=ESP_TIMER_TASK;
    ESP_ERROR_CHECK(esp_timer_create(&args,&control_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer,CONTROL_PERIOD_US));
}
void loop() { delay(1000); }

#endif
