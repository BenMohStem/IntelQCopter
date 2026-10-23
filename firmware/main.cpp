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

/* Step03: observations only. Raw samples are fresh, checked I2C reads. */
#include <Adafruit_BMP280.h>
#include <math.h>
struct Statistics{uint32_t n;double mean,m2;};
static void add_sample(Statistics *s,double x){s->n++;double d=x-s->mean;s->mean+=d/s->n;s->m2+=d*(x-s->mean);}
static double sigma(const Statistics *s){return s->n>1?sqrt(s->m2/(s->n-1)):NAN;}
static int16_t signed_be(const uint8_t *v){return (int16_t)((uint16_t)v[0]<<8|v[1]);}
static bool configure_imu(void){
    if(!imu_address)sensor_identities();
    if(!imu_address)return false;
    mpu->initialize();mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu->setDLPFMode(MPU6050_DLPF_BW_42);mpu->setRate(9);mpu->setIntDataReadyEnabled(true);
    delay(100);
    bool ok=mpu->testConnection() && mpu->getFullScaleAccelRange()==MPU6050_ACCEL_FS_8
        && mpu->getFullScaleGyroRange()==MPU6050_GYRO_FS_500 && mpu->getRate()==9;
    Serial.printf("MPU_CONFIG,ok=%d,accel_g=8,gyro_dps=500,dlpf_hz=42,nominal_hz=100\n",ok);
    return ok;
}
static float last_imu_temperature;
static bool capture_imu(unsigned count,Statistics output[6]){
    memset(output,0,6*sizeof(Statistics));
    if(!configure_imu()){Serial.println("ERROR,MPU_CONFIG_FAILED");return false;}
    uint32_t began=millis(),previous=0,min_dt=UINT32_MAX,max_dt=0,errors=0,clipped=0;
    Serial.println("IMU_CSV,us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw");
    while(output[0].n<count && millis()-began<count*20+2000){
        uint8_t status,raw[14];
        if(!read_regs(imu_address,0x3A,&status,1)){errors++;delay(2);continue;}
        if(!(status&1)){delay(1);continue;}
        if(!read_regs(imu_address,0x3B,raw,14)){errors++;continue;}
        int16_t v[7];for(unsigned i=0;i<7;i++)v[i]=signed_be(raw+2*i);
        uint32_t now=micros();
        if(previous){uint32_t dt=now-previous;if(dt<min_dt)min_dt=dt;if(dt>max_dt)max_dt=dt;}previous=now;
        for(unsigned i=0;i<3;i++){
            add_sample(&output[i],v[i]/4096.0);add_sample(&output[i+3],v[i+4]/65.5);
            if(abs((int)v[i])>32700 || abs((int)v[i+4])>32700)clipped++;
        }
        last_imu_temperature=v[3]/340.0f+36.53f;
        Serial.printf("IMU,%lu,%d,%d,%d,%d,%d,%d,%d\n",(unsigned long)now,v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
    }
    for(unsigned i=0;i<6;i++)Serial.printf("IMU_STATS,axis=%u,n=%lu,mean=%.8f,sample_std=%.8f,unit=%s\n",i,(unsigned long)output[i].n,output[i].mean,sigma(&output[i]),i<3?"g":"deg/s");
    Serial.printf("IMU_END,read_errors=%lu,clipped=%lu,min_dt_us=%lu,max_dt_us=%lu; not_hardware_acceptance\n",(unsigned long)errors,(unsigned long)clipped,(unsigned long)(min_dt==UINT32_MAX?0:min_dt),(unsigned long)max_dt);
    return output[0].n==count && errors==0 && clipped==0;
}
static void sample_barometer(void){
    if(!baro_address)sensor_identities();
    Adafruit_BMP280 bmp;
    if(!baro_address || !bmp.begin(baro_address,0x58)){Serial.println("ERROR,BMP280_NOT_READY");return;}
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::SAMPLING_X1,Adafruit_BMP280::FILTER_OFF,Adafruit_BMP280::STANDBY_MS_1);
    Statistics stats[2]={};unsigned errors=0;
    Serial.println("BARO_CONFIG,forced_x1_x1,no_filter,nominal_hz=20; BARO_CSV,us,pressure_Pa,temp_C");
    for(unsigned i=0;i<100;i++){
        uint32_t start=millis();bool ok=write_reg(baro_address,0xF4,0x25);delay(2);
        uint8_t status=8;
        while(ok && (status&8) && millis()-start<50){ok=read_regs(baro_address,0xF3,&status,1);delay(1);}
        if(!ok || (status&8)){errors++;Serial.println("ERROR,BARO_CONVERSION");continue;}
        float pressure=bmp.readPressure(),temperature=bmp.readTemperature();
        if(!isfinite(pressure)||!isfinite(temperature)||pressure<30000||pressure>110000){errors++;Serial.println("ERROR,BARO_RANGE_OR_READ");continue;}
        add_sample(&stats[0],pressure);add_sample(&stats[1],temperature);
        Serial.printf("BARO,%lu,%.4f,%.4f\n",(unsigned long)micros(),pressure,temperature);
        while(millis()-start<50)delay(1);
    }
    for(unsigned i=0;i<2;i++)Serial.printf("BARO_STATS,axis=%u,n=%lu,mean=%.8f,sample_std=%.8f\n",i,(unsigned long)stats[i].n,stats[i].mean,sigma(&stats[i]));
    Serial.printf("BARO_END,errors=%u; no_automatic_acceptance_or_absolute_altitude_calibration\n",errors);
}
static void sample_magnetometer(void){
    uint8_t id[3];
    if(!read_regs(0x1E,0x0A,id,3)||id[0]!=0x48||id[1]!=0x34||id[2]!=0x33){Serial.println("ERROR,HMC_ID_NOT_H43; QMC_requires_its_own_driver");return;}
    if(!write_reg(0x1E,0,0x70)||!write_reg(0x1E,1,0x20)||!write_reg(0x1E,2,0)){Serial.println("ERROR,MAG_CONFIG");return;}
    Serial.println("MAG_CONFIG,HMC_only,8_average,15Hz,gain_1090_LSB_per_Gauss; no_heading_calibration");
    uint32_t start=millis();unsigned n=0,errors=0;
    while(n<64 && millis()-start<15000){
        uint8_t status,raw[6];
        if(!read_regs(0x1E,9,&status,1)){errors++;delay(10);continue;}
        if(!(status&1)){delay(2);continue;}
        if(!read_regs(0x1E,3,raw,6)){errors++;continue;}
        int16_t x=signed_be(raw),z=signed_be(raw+2),y=signed_be(raw+4);
        if(x==-4096||y==-4096||z==-4096){errors++;continue;}
        Serial.printf("MAG,%lu,%d,%d,%d\n",(unsigned long)micros(),x,y,z);n++;
    }
    Serial.printf("MAG_END,n=%u,errors=%u; raw_field_not_heading_or_calibration\n",n,errors);
}
static void sample_lidar(void){
    Serial1.setRxBufferSize(2048);Serial1.begin(921600,SERIAL_8N1,6,7);
    uint8_t frame[16];unsigned used=0,good=0,bad=0;uint32_t began=millis();
    Serial.println("LIDAR_CONFIG,passive_TOFSense_UART,baud=921600,FC_RX=6,FC_TX=7; no_transmitted_command");
    while(millis()-began<10000){
        if(!Serial1.available()){delay(1);continue;}
        uint8_t v=(uint8_t)Serial1.read();
        if((used==0 && v!=0x57)||(used==1 && v!=0)||(used==2 && v!=0xFF)){used=v==0x57?1:0;frame[0]=0x57;continue;}
        frame[used++]=v;if(used<16)continue;used=0;
        uint8_t sum=0;for(unsigned i=0;i<15;i++)sum+=frame[i];
        if(sum!=frame[15]){bad++;continue;}
        uint32_t bits=(uint32_t)frame[8]|(uint32_t)frame[9]<<8|(uint32_t)frame[10]<<16;
        int32_t distance=(bits&0x800000)?(int32_t)(bits|0xFF000000):((int32_t)bits);
        Serial.printf("LIDAR,%lu,id=%u,distance_mm=%ld,status=%u,strength=%u,raw=",(unsigned long)micros(),frame[3],(long)distance,frame[11],(unsigned)(frame[12]|(uint16_t)frame[13]<<8));
        for(unsigned i=0;i<16;i++){Serial.printf("%02X",frame[i]);}
        Serial.println();good++;
    }
    Serial1.end();Serial.printf("LIDAR_END,frames=%u,bad_checksums=%u; verify_model_status_and_measured_target_distance\n",good,bad);
}

void setup(void){
    for(unsigned i=0;i<4;i++)pinMode(MOTOR_PINS[i],INPUT_PULLDOWN);
    Serial.begin(115200);delay(1000);board_report();
    bus_ready=Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(100000);Wire.setTimeOut(20);
    Serial.println("COMMAND,p=board,i=identities,m=500_IMU,b=100_baro,h=HMC_raw,l=LiDAR_10s");
}
void loop(void){
    if(Serial.available()){int c=Serial.read();if(c=='p')board_report();else if(c=='i')sensor_identities();else if(c=='m'){Statistics stats[6];capture_imu(500,stats);}else if(c=='b')sample_barometer();else if(c=='h')sample_magnetometer();else if(c=='l')sample_lidar();}
    static uint32_t previous;
    if(millis()-previous>=5000){previous=millis();Serial.printf("HEARTBEAT,%lu\n",(unsigned long)millis());}
    delay(1);
}
