#ifndef INTELQCOPTER_FC_API_H
#define INTELQCOPTER_FC_API_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define FC_CAL_RECORD_BYTES 160u
#define FC_CAL_PROFILE 0x00080001u /* ±8g, ±500dps, 42Hz DLPF, 500Hz; bump after mount/config changes. */
/* After boot setup, only the control task calls this API. Cross-task requests use queues. */
typedef struct {
    float accel_g[3],gyro_dps[3],pressure_pa;
    uint32_t pressure_sequence;
    bool imu_valid,pressure_valid;
} fc_sample_t;
typedef struct {
    float roll,pitch,yaw,vertical,boost;
    uint64_t received_us;
    bool connected;
} fc_pilot_t;
typedef struct {
    float motor_demand[4],roll_deg,pitch_deg,height_m,vertical_mps;
    float gyro_sigma_dps[3];
    bool sensor_fault,pilot_fresh,gyro_verified,accel_calibrated,barometer_ready;
    bool battery_warning,battery_disarm;
    uint8_t faces_done;
    int8_t capturing_face;
} fc_status_t;
void fc_init(uint64_t board_id);
void fc_step(const fc_sample_t*,const fc_pilot_t*,uint64_t now_us,float dt,fc_status_t*);
bool fc_cal_load(const uint8_t record[FC_CAL_RECORD_BYTES]);
void fc_cal_command(char command);
bool fc_cal_take_pending(uint8_t record[FC_CAL_RECORD_BYTES]);
void fc_set_estimated_soc(float percent); /* Future calibrated battery monitor; not controller battery. */
#ifdef __cplusplus
}
#endif
#endif
