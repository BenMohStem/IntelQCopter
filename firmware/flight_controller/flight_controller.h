#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
   Flight Controller Public API
   =========================================================================== */

/* Sensor sample structure (defined in flight_controller.c) */
typedef struct {
    bool valid;
    bool fresh;
    uint32_t timestamp_us;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float temperature_c;
    float barometer_height_m;
    bool barometer_valid;
    bool barometer_updated;
    float barometer_raw_pa;
    float barometer_temperature_c;
    float magnetometer_x_uT, magnetometer_y_uT, magnetometer_z_uT;
    bool magnetometer_valid;
    bool magnetometer_fresh;
    float vertical_acceleration_mps2;
} sensor_sample_t;

/* Control output structure */
typedef struct {
    bool armed;
    bool sensor_fault;
    float motor[4];   /* FL, FR, RR, RL: ESC command fractions 0..1 */
} control_output_t;

/* Initialize flight controller (call once at boot) */
void fc_init(void);

/* Apply pilot command from UART frame */
void fc_apply_command(const void *frame, uint64_t now_ms);

/* Get control output for current sensor sample and pilot command */
control_output_t fc_get_output(const sensor_sample_t *s,
                               const void *cmd,
                               float dt);

/* Check if armed */
bool fc_is_armed(void);

/* Check if failsafe */
bool fc_is_failsafe(void);

/* Check controller connection */
bool fc_controller_connected(void);

/* Apply calibration data */
void fc_apply_calibration(const float accel_bias[3],
                          const float accel_scale[3][3],
                          const float gyro_bias[3],
                          const float mag_hard_iron[3],
                          const float mag_soft_iron[3][3]);

/* Get hover collective */
float fc_get_hover_collective(void);

/* Set hover collective */
void fc_set_hover_collective(float collective);

/* Get baro zero offset */
float fc_get_baro_zero_offset(void);

/* Get altitude reference */
float fc_get_altitude_ref(void);

/* ===========================================================================
   PID Tuning API (for WiFi tuner integration)
   =========================================================================== */

/* Get PID values for a given axis */
void fc_get_pid_values(float *kp, float *ki, float *kd, const char *axis);

/* Set PID values for a given axis */
bool fc_set_pid_values(const char *axis, float kp, float ki, float kd);

/* Get telemetry data */
void fc_get_telemetry(float *roll, float *pitch, float *yaw,
                       float *roll_rate, float *pitch_rate,
                       float *yaw_rate, float *altitude,
                       float *vert_speed, float *motor0,
                       float *motor1, float *motor2, float *motor3,
                       bool *armed, float *loop_rate);

/* Get loop period in seconds */
float fc_get_loop_dt(void);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROLLER_H */