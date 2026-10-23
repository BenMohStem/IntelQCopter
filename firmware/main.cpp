/* Step01: board diagnostics only. No motor output and no flight loop. */
#include <Arduino.h>
#include <esp_system.h>
#ifndef BENCH_SOURCE_SHA
#define BENCH_SOURCE_SHA "UNSTAMPED"
#endif
static const int MOTOR_PINS[]={12,1,2,14};
static void board_report(void){
    Serial.printf("BOARD,uptime_ms=%lu,chip=%s,revision=%u,cores=%u,reset_reason=%d,heap=%u\n",
        (unsigned long)millis(),ESP.getChipModel(),ESP.getChipRevision(),ESP.getChipCores(),(int)esp_reset_reason(),ESP.getFreeHeap());
    Serial.printf("SOURCE_SHA,%s\n",BENCH_SOURCE_SHA);
    Serial.println("MOTOR_OUTPUTS,DISABLED_INPUT_PULLDOWN; no PWM/Servo/ESC driver");
}
void setup(void){
    for(unsigned i=0;i<4;i++)pinMode(MOTOR_PINS[i],INPUT_PULLDOWN);
    Serial.begin(115200);delay(1000);board_report();Serial.println("COMMAND,p=board report");
}
void loop(void){
    if(Serial.available() && Serial.read()=='p')board_report();
    static uint32_t previous;
    if(millis()-previous>=5000){previous=millis();Serial.printf("HEARTBEAT,%lu\n",(unsigned long)millis());}
    delay(1);
}
