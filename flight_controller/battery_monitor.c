/* Configurable battery telemetry / warning logic. No hardware cut-off path. */
#include "battery_monitor.h"

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

    /* BMS alerts and cell-level undervoltage outrank pack-level warnings. */
    if (sample->bms_alert
        || (sample->cell_voltage_valid
            && sample->minimum_cell_voltage_v <= config->critical_cell_voltage_v)) {
        status->action = sample->aircraft_landed
            ? BATTERY_ACTION_LANDED_DISARM : BATTERY_ACTION_LAND;
    } else if (status->filtered_pack_voltage_v <= config->low_pack_voltage_v
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
