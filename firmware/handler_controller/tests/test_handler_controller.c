#define QCOPTER_HANDLER_HOST_TEST
#include "../handler_controller.c"
#undef NDEBUG
#include <assert.h>
int main(void) {
    handler_calibration_t c={.minimum={0,0,0,0},.center={2048,2048,2048,2048},
      .maximum={4095,4095,4095,4095},.trigger_released={0,4095},.trigger_pressed={4095,0},
      .deadband=.04f,.expo=.25f,.filter_tau_s=.035f,.slew_per_s=3,.valid=true};
    handler_state_t s={0};uint16_t raw[6]={2048,2048,2048,2048,0,4095};qcopter_command_frame_t f;
    assert(handler_process(&s,&c,raw,0,.01f,&f));assert(f.throttle==5000 && f.roll==0);
    raw[2]=4095;assert(handler_process(&s,&c,raw,QCOPTER_BUTTON_RB,.01f,&f));assert(f.roll>0 && f.roll<=300);
    for(int i=0;i<200;i++)assert(handler_process(&s,&c,raw,0,.01f,&f));assert(f.roll==10000);
    raw[4]=4095;raw[5]=0;assert(handler_process(&s,&c,raw,0,.01f,&f));assert(f.lt==1000 && f.rt==1000);
    uint16_t seq=s.sequence;raw[0]=5000;assert(!handler_process(&s,&c,raw,0,.01f,&f));assert(s.sequence==seq);
    raw[0]=2048;assert(!handler_process(&s,&c,raw,0,.2f,&f));
    c.maximum[1]=c.center[1];assert(!handler_process(&s,&c,raw,0,.01f,&f));
    c.maximum[1]=4095;handler_state_t gestures={0};raw[2]=2048;raw[4]=0;raw[5]=4095;
    int events=0;for(int i=0;i<200;i++){assert(handler_process(&gestures,&c,raw,QCOPTER_BUTTON_RB|QCOPTER_BUTTON_NORTH,.01f,&f));events+=!!(gestures.mission_events&1);}
    assert(events==1);
    assert(handler_process(&gestures,&c,raw,0,.01f,&f));events=0;
    for(int i=0;i<150;i++){assert(handler_process(&gestures,&c,raw,QCOPTER_BUTTON_LB|QCOPTER_BUTTON_SOUTH,.01f,&f));events+=!!(gestures.mission_events&2);}
    assert(events==1);
    puts("Handler C tests passed: neutral, deadband, shaping, slew, endpoints, reversed triggers, invalid ADC/dt/calibration.");
}
