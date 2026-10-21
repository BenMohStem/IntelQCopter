#ifndef INTELQCOPTER_FC_API_H
#define INTELQCOPTER_FC_API_H
#include <stdbool.h>
#include <stdint.h>
#if defined(FC_BUILD_DLL) && defined(_WIN32)
#define FC_API __declspec(dllexport)
#else
#define FC_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
#ifndef FC_ENABLE_COMPANION
#define FC_ENABLE_COMPANION 1
#endif
FC_API void fc_configure_companion(bool enabled); /* Boot/control task only. Compile-time disabled builds cannot enable it. */
#define FC_CAL_RECORD_BYTES 160u
#define FC_CAL_PROFILE 0x00080001u /* ±8g, ±500dps, 42Hz DLPF, 500Hz; bump after mount/config changes. */
/* After boot setup, only the control task calls this API. Cross-task requests use queues. */
typedef struct {
    float accel_g[3],gyro_dps[3],pressure_pa;
    uint32_t pressure_sequence;
    bool imu_valid,pressure_valid;
} fc_sample_t;
typedef struct {
    float roll,pitch,yaw,vertical,boost,reduce;
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
    uint8_t flight_mode; /* 0=stable, 1=ROS navigation */
    bool user_disarm,companion_fresh;
    uint32_t xbox_reports,mode_events;
} fc_status_t;
/* Optional horizontal localization input: world x/y (m), vx/vy (m/s),
 * body yaw (rad), acquisition time in the FC clock. Control-task owned.
 * No production hardware driver supplies this yet; simulation labels truth aiding.
 * NULL/invalid input revokes aiding; >100ms old observations cannot hold position. */
FC_API bool fc_horizontal_observation(const float world_xy_vxy_yaw[5],uint64_t acquired_us);
FC_API void fc_init(uint64_t board_id);
FC_API void fc_step(const fc_sample_t*,const fc_pilot_t*,uint64_t now_us,float dt,fc_status_t*);
FC_API bool fc_cal_load(const uint8_t record[FC_CAL_RECORD_BYTES]);
FC_API void fc_cal_command(char command);
FC_API bool fc_cal_take_pending(uint8_t record[FC_CAL_RECORD_BYTES]);
FC_API void fc_set_estimated_soc(float percent); /* Future calibrated battery monitor; not controller battery. */
/* Same decoded-report/authority path for hardware and simulation. Control-task owned. */
FC_API bool fc_xbox_report(const uint8_t *report, unsigned length, uint64_t now_us);
FC_API bool fc_companion_report(const uint8_t *frame, unsigned length, uint64_t now_us);
FC_API void fc_control_step(const fc_sample_t*,uint64_t now_us,float dt,fc_status_t*);
#ifdef FC_BUILD_DLL
FC_API bool fc_sim_configure(const float *values,unsigned count);
FC_API unsigned fc_sim_observe(float *values,unsigned capacity);
#endif
#define FC_COMPANION_FRAME_BYTES 28u
FC_API void fc_status_frame(const fc_status_t*,uint8_t frame[16]);
#ifdef __cplusplus
}
#endif
#endif
