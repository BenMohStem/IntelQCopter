/* Actual production core, deterministic synthetic samples; no simulator/hardware. */
#define FC_CORE_ONLY
#define FC_BUILD_DLL
#include "../../firmware/src/flight_controller.cpp"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1;}}while(0)
int main(void) {
    sensor_sample_t sample={};sample.accel_z_g=1;
    sample.imu_valid=sample.imu_calibrated=sample.barometer_valid=sample.barometer_updated=true;
    pilot_command_t cmd={};cmd.valid=true;cmd.throttle=1;
    uint32_t random=19;
    for(unsigned n=0;n<20000;n++) {
        fc_init(1000u+n);
        const float boost=(n%101)/100.f;boost_state=boost;cmd.boost=boost;
        random=random*1664525u+1013904223u;cmd.roll_deg=((int)(random%70001)-35000)/1000.f;
        random=random*1664525u+1013904223u;cmd.pitch_deg=((int)(random%70001)-35000)/1000.f;
        random=random*1664525u+1013904223u;cmd.yaw_rate_dps=((int)(random%360001)-180000)/1000.f;
        sample.gyro_x_dps=(n%2)?-500:500;sample.gyro_y_dps=(n%3)?-500:500;sample.gyro_z_dps=(n%5)?-500:500;
        const control_output_t out=flight_control_step(&sample,&cmd,.002f);
        const float ceiling=.45f+.20f*boost;
        CHECK(!out.sensor_fault && !out.armed);
        for(float value:out.motor) { if (!(isfinite(value)&&value>=.099999f&&value<=ceiling+.000001f)) { fprintf(stderr,"n=%u value=%f cap=%f boost=%f state=%f\n",n,value,ceiling,boost,boost_state); return 1; } }
    }
    cmd.valid=false;control_output_t out=flight_control_step(&sample,&cmd,.002f);
    CHECK(out.sensor_fault);for(float v:out.motor)CHECK(v==0);
    cmd.valid=true;sample.imu_valid=false;out=flight_control_step(&sample,&cmd,.002f);
    CHECK(out.sensor_fault);for(float v:out.motor)CHECK(v==0);
    sample.imu_valid=true;battery_disarm_latched=true;out=flight_control_step(&sample,&cmd,.002f);
    for(float v:out.motor)CHECK(v==0);
    puts("PASS: 20000 actual ESP32 core allocations respect 10% floor and interpolated 45%-65% final motor caps; bad commands/sensors/battery disarm output zero. Physical arming remains unimplemented.");
    return 0;
}
