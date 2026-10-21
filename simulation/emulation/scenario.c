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
    for(unsigned i=0;i<250;i++){fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);input_time+=20000;}
    check("navigation_requires_full_five_seconds",status.flight_mode==0);
    fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
    check("down_B_selects_navigation_after_five_seconds",status.flight_mode==1 && status.mode_events==1);
    for(unsigned i=0;i<50;i++){input_time+=20000;fc_xbox_report(hid,16,input_time);}
    fc_control_step(&sample,input_time,.002f,&status);
    check("held_gesture_fires_once",status.mode_events==1);
    hid[13]=1;input_time+=20000;fc_xbox_report(hid,16,input_time);
    input_time+=200000;fc_xbox_report(hid,16,input_time); // Report gap resets the hold.
    for(unsigned i=0;i<249;i++){input_time+=20000;fc_xbox_report(hid,16,input_time);}
    fc_control_step(&sample,input_time,.002f,&status);
    check("report_gap_restarts_mode_hold",status.flight_mode==1);
    input_time+=20000;fc_xbox_report(hid,16,input_time);fc_control_step(&sample,input_time,.002f,&status);
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
    printf("QCOPTER_EMU_END,checks=%u,failures=%u,ticks=%u\n",checks,failures,ticks);
    fflush(stdout);
}
