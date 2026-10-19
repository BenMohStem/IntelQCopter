#ifndef QCOPTER_BATTERY_MONITOR_H
#define QCOPTER_BATTERY_MONITOR_H

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

#endif
