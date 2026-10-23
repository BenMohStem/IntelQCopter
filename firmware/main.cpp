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
/* Step02: identities only; no sensor data invented and no flight outputs. */
#include <Wire.h>
#include <MPU6050.h>
static const int SDA_PIN=9,SCL_PIN=8;
static MPU6050 mpu68(0x68),mpu69(0x69);
static MPU6050 *mpu=&mpu68;
static uint8_t imu_address=0,baro_address=0;
static bool bus_ready=false;
static bool read_regs(uint8_t address,uint8_t reg,uint8_t *data,size_t count){
    if(!bus_ready || count>32)return false;
    Wire.beginTransmission(address);Wire.write(reg);
    if(Wire.endTransmission(false)!=0)return false;
    size_t got=Wire.requestFrom(address,(uint8_t)count,(uint8_t)true);
    if(got!=count){while(Wire.available())Wire.read();return false;}
    for(size_t i=0;i<count;i++)data[i]=(uint8_t)Wire.read();
    return true;
}
static bool write_reg(uint8_t address,uint8_t reg,uint8_t value){
    Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);
    return Wire.endTransmission(true)==0;
}
static void sensor_identities(void){
    imu_address=baro_address=0;
    if(!bus_ready){Serial.println("ERROR,I2C_BEGIN_FAILED");return;}
    Serial.println("SCAN,GPIO_SDA=9,GPIO_SCL=8,clock_hz=100000");
    for(uint8_t a=8;a<0x78;a++){
        Wire.beginTransmission(a);uint8_t rc=Wire.endTransmission();
        if(rc==0)Serial.printf("ACK,0x%02X\n",a);
    }
    for(unsigned i=0;i<2;i++){
        uint8_t id=0,a=0x68+i;
        if(read_regs(a,0x75,&id,1)){
            Serial.printf("MPU_WHO_AM_I,address=0x%02X,id=0x%02X,expected=0x68\n",a,id);
            MPU6050 *candidate=i?&mpu69:&mpu68;
            if(id==0x68 && candidate->testConnection()){
                if(!imu_address){imu_address=a;mpu=candidate;}
                else Serial.println("NOTICE,TWO_MPUS_PRESENT; using lower address");
            }
        }
        a=0x76+i;
        if(read_regs(a,0xD0,&id,1)){
            Serial.printf("BARO_ID,address=0x%02X,id=0x%02X,expected_BMP280=0x58\n",a,id);
            if(id==0x58 && !baro_address)baro_address=a;
        }
    }
    uint8_t id[3];
    if(read_regs(0x1E,0x0A,id,3))Serial.printf("MAG_HMC_ID,%02X,%02X,%02X,expected_H43=48,34,33\n",id[0],id[1],id[2]);
    Wire.beginTransmission(0x0D);
    if(Wire.endTransmission()==0)Serial.println("MAG,0x0D_ACK_ONLY; possible_QMC_NOT_IDENTIFIED; no_HMC_driver_assumed");
    Serial.printf("REQUIRED_IDENTITIES,mpu=%s,bmp=%s; observation_only_not_acceptance\n",imu_address?"FOUND":"MISSING",baro_address?"FOUND":"MISSING");
}

void setup(void){
    for(unsigned i=0;i<4;i++)pinMode(MOTOR_PINS[i],INPUT_PULLDOWN);
    Serial.begin(115200);delay(1000);board_report();
    bus_ready=Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(100000);Wire.setTimeOut(20);
    Serial.println("COMMAND,p=board report,i=I2C scan and identities");
}
void loop(void){
    if(Serial.available()){int c=Serial.read();if(c=='p')board_report();else if(c=='i')sensor_identities();}
    static uint32_t previous;
    if(millis()-previous>=5000){previous=millis();Serial.printf("HEARTBEAT,%lu\n",(unsigned long)millis());}
    delay(1);
}
