/* Deterministic input replay on an emulated ESP32-S3. No plant or BLE model.
 * Every check evaluates the active flight_controller.c through its public API.
 * Logical dt is 2 ms; executing faster does not measure physical deadlines. */
#include "fc_api.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static fc_sample_t sample;
static fc_pilot_t pilot;
static fc_status_t status;
static uint64_t clock_us;
static unsigned ticks, failures, checks;
static void put32(uint8_t*p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static void seal(uint8_t*p){uint32_t c=~0u;for(unsigned j=0;j<24;j++){c^=p[j];for(unsigned k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}put32(p+24,~c);}

static void check(const char *name, bool passed) {
    ++checks;
    if (!passed) ++failures;
    printf("CHECK,%s,%s\n", name, passed ? "PASS" : "FAIL");
}
static void tick(bool fresh) {
    clock_us += 2000;
    if (++ticks % 10 == 0) ++sample.pressure_sequence;
    if (fresh) pilot.received_us = clock_us;
    fc_step(&sample, &pilot, clock_us, .002f, &status);
}
static void run(unsigned count, bool fresh) { while (count--) tick(fresh); }
static bool zero_outputs(void) {
    for (unsigned i=0; i<4; ++i) if (status.motor_demand[i] != 0) return false;
    return true;
}
static void row(const char *phase) {
    printf("DATA,%llu,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u\n",
        (unsigned long long)clock_us, phase, (double)status.roll_deg,
        (double)status.pitch_deg, (double)status.height_m,
        (double)status.motor_demand[0], (double)status.motor_demand[1],
        (double)status.motor_demand[2], (double)status.motor_demand[3],
        status.sensor_fault, status.battery_warning, status.battery_disarm);
}
/* Valid neutral UART presence, matching the idle ROS2 heartbeat. */
static uint32_t heartbeat_sequence=1000;
static void emulate_companion(uint64_t now) {
    uint8_t h[28]={'Q','N',1,1};put32(h+4,++heartbeat_sequence);seal(h);
    (void)fc_companion_report(h,28,now);
}
void run_scenario(void) {
    printf("QCOPTER_EMU_BEGIN,ESP32-S3,synthetic-input-replay,dt_us=2000\n");
    printf("FIELDS,time_us,phase,roll_deg,pitch_deg,height_m,FL,FR,RR,RL,fault,warn,disarm\n");
    fc_init(0x1122334455667788ull);
    sample = (fc_sample_t){.accel_g={0,0,1},.gyro_dps={.8f,-.2f,.1f},
        .pressure_pa=101325,.imu_valid=true,.pressure_valid=true};
    pilot=(fc_pilot_t){.connected=true};
    run(1100,true);
    check("gyro_stationary_verification",status.gyro_verified);
    check("barometer_ground_reference",status.barometer_ready);
    check("uncalibrated_accelerometer_blocks_demands",status.sensor_fault && zero_outputs());
    row("boot");
    for (unsigned f=0; f<6; ++f) {
        memset(sample.accel_g,0,sizeof(sample.accel_g));
        sample.accel_g[f/2]=(f%2)?-1:1;
        fc_cal_command((char)('0'+f));
        run(200,true);
    }
    sample.accel_g[0]=sample.accel_g[1]=0; sample.accel_g[2]=1;
    run(500,true);
    check("six_face_calibration",status.accel_calibrated && status.faces_done==63);
    check("healthy_calibrated_input",!status.sensor_fault);
    uint8_t record[FC_CAL_RECORD_BYTES], corrupt[FC_CAL_RECORD_BYTES];
    fc_cal_command('S');
    bool saved=fc_cal_take_pending(record);
    check("calibration_record_generated",saved);
    if(saved) {
        memcpy(corrupt,record,sizeof(record)); corrupt[40]^=1;
        check("crc_rejects_corruption",!fc_cal_load(corrupt));
        fc_init(99);
        check("record_rejects_wrong_board",!fc_cal_load(record));
        fc_init(0x1122334455667788ull);
        check("valid_record_load",fc_cal_load(record));
        tick(true);
        check("init_resets_estimator_state",status.roll_deg==0 && status.pitch_deg==0 && status.height_m==0);
        check("boot_requires_fresh_gyro_check",!status.gyro_verified && zero_outputs());
        run(1100,true);
        check("reload_recovers_after_verification",status.gyro_verified && status.accel_calibrated && !status.sensor_fault);
    }
    row("neutral");
    for(unsigned i=0;i<250;i++) {
        pilot.roll=.10f; tick(true);
        if(i%10==0) row("roll_request");
    }
    check("roll_mixer_sign",status.motor_demand[0]>status.motor_demand[1] && status.motor_demand[3]>status.motor_demand[2]);
    pilot.roll=0; run(300,true); row("recenter");
    pilot.vertical=.2f;
    for(unsigned i=0;i<250;i++) { tick(true); if(i%10==0) row("climb_request"); }
    pilot.vertical=0;
    run(126,false); row("stale_report");
    check("stale_report_zeroes_demands",!status.pilot_fresh && zero_outputs());
    run(50,true);
    sample.imu_valid=false; tick(true); row("imu_loss");
    check("imu_loss_zeroes_demands",status.sensor_fault && zero_outputs());
    sample.imu_valid=true; run(50,true);
    sample.pressure_valid=false; tick(true); row("barometer_loss");
    check("barometer_loss_zeroes_demands",status.sensor_fault && zero_outputs());
    sample.pressure_valid=true; run(50,true);
    fc_set_estimated_soc(35); tick(true); row("soc_35");
    check("soc_35_warns",status.battery_warning && !status.battery_disarm);
    fc_set_estimated_soc(20); tick(true); row("soc_20");
    check("soc_20_disarms",status.battery_disarm && zero_outputs());
    fc_set_estimated_soc(80); tick(true);
    check("battery_disarm_latches",status.battery_disarm && zero_outputs());
    /* Exercise the exact raw-report path used by BLE and Gazebo. */
    fc_init(0x1122334455667788ull);
    uint8_t hid[16]={0};for(unsigned i=0;i<4;i++){hid[2*i]=0;hid[2*i+1]=128;}
    hid[12]=5;hid[13]=2;
    uint64_t input_time=30000000u;
    for(unsigned i=0;i<250;i++){emulate_companion(input_time);fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);input_time+=20000;}
    check("navigation_requires_full_five_seconds",status.flight_mode==0);
    emulate_companion(input_time);fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("down_B_selects_navigation_after_five_seconds",status.flight_mode==1 && status.mode_events==1);
    for(unsigned i=0;i<50;i++){input_time+=20000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time);}
    fc_control_step(&sample,input_time,.002f,&status);
    check("held_gesture_fires_once",status.mode_events==1);
    hid[13]=1;input_time+=20000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time);
    input_time+=200000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time); // Report gap resets the hold.
    for(unsigned i=0;i<249;i++){input_time+=20000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time);}
    fc_control_step(&sample,input_time,.002f,&status);
    check("report_gap_restarts_mode_hold",status.flight_mode==1);
    input_time+=20000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("down_A_selects_stable",status.flight_mode==0 && status.mode_events==2);
    hid[13]=0;hid[8]=0xff;hid[9]=3;
    input_time+=20000;fc_xbox_report(hid,16,input_time);
    for(unsigned i=0;i<99;i++){input_time+=20000;fc_xbox_report(hid,16,input_time);}
    fc_control_step(&sample,input_time,.002f,&status);
    check("down_LT_requires_two_seconds",!status.user_disarm);
    input_time+=20000;fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("down_LT_disarms_after_two_seconds",status.user_disarm && zero_outputs());
    hid[12]=0;hid[8]=hid[9]=0;input_time+=20000;fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("user_disarm_is_latched",status.user_disarm && zero_outputs());
    check("malformed_HID_rejected",!fc_xbox_report(hid,15,input_time));
    hid[12]=9;check("invalid_hat_rejected",!fc_xbox_report(hid,16,input_time));
    uint8_t telemetry[16];fc_status_frame(&status,telemetry);
    check("UART_telemetry_identifies_mode_and_disarm",telemetry[0]=='Q' && telemetry[1]=='S' && telemetry[3]==0 && telemetry[4]==1);
    fc_configure_companion(false);fc_configure_companion(true); /* Reset protocol sequence test session. */
    uint8_t uart[28]={'Q','N',1,1};put32(uart+4,1);seal(uart);
    check("UART_valid_command_accepted",fc_companion_report(uart,28,input_time));
    check("UART_duplicate_sequence_rejected",!fc_companion_report(uart,28,input_time+1000));
    uart[8]^=1;check("UART_corrupt_CRC_rejected",!fc_companion_report(uart,28,input_time+2000));uart[8]^=1;
    put32(uart+4,2);put32(uart+8,0x7fc00000u);seal(uart);
    check("UART_NaN_rejected",!fc_companion_report(uart,28,input_time+3000));
    float outside=36;memcpy(uart+8,&outside,4);seal(uart);
    check("UART_out_of_range_angle_rejected",!fc_companion_report(uart,28,input_time+4000));
    put32(uart+8,0);seal(uart);
    check("UART_fresh_sequence_accepted",fc_companion_report(uart,28,input_time+5000));
    fc_control_step(&sample,input_time+5000,.002f,&status);
    check("UART_freshness_reported",status.companion_fresh);
    fc_control_step(&sample,input_time+156000,.002f,&status);
    check("UART_stale_command_expires",!status.companion_fresh);
    check("UART_wrong_length_rejected",!fc_companion_report(uart,27,input_time+157000));
    fc_configure_companion(false);
    fc_control_step(&sample,input_time+158000,.002f,&status);
    check("standalone_selects_stable",status.flight_mode==0 && !status.companion_fresh);
    put32(uart+4,3);seal(uart);
    check("standalone_rejects_companion_command",!fc_companion_report(uart,28,input_time+160000));
    hid[12]=5;hid[13]=2;input_time+=200000;
    for(unsigned i=0;i<260;i++){fc_xbox_report(hid,16,input_time);input_time+=20000;}
    fc_control_step(&sample,input_time,.002f,&status);
    check("standalone_rejects_navigation_gesture",status.flight_mode==0);
    fc_status_frame(&status,telemetry);
    check("standalone_telemetry_capability_off",(telemetry[7]&1)==0);
    fc_configure_companion(true);fc_status_frame(&status,telemetry);
    check("companion_telemetry_capability_on",(telemetry[7]&1)==1);
    // Completion/disarm is a CRC-protected companion request, scoped to nav.
    fc_init(0x3200);fc_configure_companion(true);input_time=1000000;
    memset(hid,0,sizeof(hid));for(unsigned i=0;i<4;i++){hid[2*i]=0;hid[2*i+1]=0x80;}
    memset(uart,0,sizeof(uart));uart[0]='Q';uart[1]='N';uart[2]=1;uart[3]=3;put32(uart+4,1);seal(uart);
    check("UART_completion_flag_valid",fc_companion_report(uart,28,input_time));
    fc_control_step(&sample,input_time,.002f,&status);
    check("stable_ignores_companion_disarm",!status.user_disarm);
    hid[12]=5;hid[13]=2;
    for(unsigned i=0;i<260;i++){emulate_companion(input_time);fc_xbox_report(hid,16,input_time);input_time+=20000;}
    fc_control_step(&sample,input_time,.002f,&status);
    check("completion_test_navigation_selected",status.flight_mode==1);
    uart[3]=7;put32(uart+4,++heartbeat_sequence);seal(uart);
    check("UART_unknown_flags_rejected",!fc_companion_report(uart,28,input_time));
    uart[3]=3;seal(uart);check("navigation_completion_request_accepted",fc_companion_report(uart,28,input_time));
    fc_control_step(&sample,input_time,.002f,&status);
    check("navigation_completion_disarms_FC",status.user_disarm&&zero_outputs());
    float xy[5]={0,0,0,0,0};
    check("horizontal_observation_finite_accepted",fc_horizontal_observation(xy,input_time));
    check("horizontal_observation_old_rejected",!fc_horizontal_observation(xy,input_time-1));
    xy[0]=NAN;check("horizontal_observation_NaN_rejected",!fc_horizontal_observation(xy,input_time+1));
    check("horizontal_observation_NULL_revokes",!fc_horizontal_observation(NULL,input_time+2));
    fc_init(0x3200);fc_debug_t d;fc_debug_snapshot(&d);
    check("boot_default_gentle_profile",d.gentle==1);
    check("boot_default_debug_statistics",d.level==2);
    check("invalid_debug_level_rejected",!fc_debug_configure(4));
    check("trace_debug_enabled_at_boot",fc_debug_configure(3));
    memset(hid,0,sizeof(hid));for(unsigned i=0;i<4;i++)hid[2*i+1]=0x80;
    fc_xbox_report(hid,16,1000000);
    check("runtime_profile_change_rejected",!fc_configure_gentle(false));
    check("runtime_debug_change_rejected",!fc_debug_configure(0));
    check("older_HID_rejected",!fc_xbox_report(hid,16,999999));
    fc_profile_record(2000);fc_profile_record(2001);fc_debug_snapshot(&d);
    check("deadline_histogram_boundary",d.deadline_misses==1&&d.max_execution_us==2001);
    fc_control_step(NULL,1000000,.002f,&status);
    check("null_sample_faults_zero_demands",status.sensor_fault&&zero_outputs());
    fc_init(0x3200);memset(hid,0,sizeof(hid));for(unsigned i=0;i<4;i++)hid[2*i+1]=0x80;
    hid[12]=5;hid[13]=2;input_time=60000000u;
    for(unsigned i=0;i<260;i++){fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);input_time+=20000;}
    check("absent_ROS_cannot_select_navigation",status.flight_mode==0);
    for(unsigned i=0;i<260;i++){emulate_companion(input_time);fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);input_time+=20000;}
    check("fresh_ROS_allows_explicit_navigation",status.flight_mode==1);
    hid[12]=hid[13]=0;input_time+=200000;fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    fc_link_stats_t links;fc_link_snapshot(&links);
    check("lost_ROS_falls_back_to_stable",status.flight_mode==0 && links.stable_fallbacks==1);
    input_time+=20000;emulate_companion(input_time);fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("reconnected_ROS_does_not_seize_control",status.flight_mode==0 && status.companion_fresh);
    printf("QCOPTER_EMU_END,checks=%u,failures=%u,ticks=%u\n",checks,failures,ticks);
    fflush(stdout);
}
