/* Minimal C++ boundary for Arduino, sensor drivers, NVS and Xbox BLE.
 * All estimators, six controllers and calibration math are in flight_controller.c.
 * No ESC library, output pulse, or physical arming action exists in this adapter.
 */
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include <esp_timer.h>
#include "fc_api.h"
static void xbox_report(uint8_t*,size_t);
#define QCOPTER_XBOX_REPORT_HOOK xbox_report
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

static constexpr int SDA_PIN=9,SCL_PIN=8,LED_PIN=42;
static constexpr int MOTOR_PINS[4]={12,1,2,14}; // FL, FR, RR, RL; INPUT ONLY.
static constexpr uint32_t CONTROL_PERIOD_US=2000;
static constexpr int64_t PRESSURE_TIMEOUT_US=150000;
static XboxSeriesXControllerESP32_asukiaaa::Core xbox;
static MPU6050 mpu;
static Adafruit_BMP280 bmp;
static bool imu_available,bmp_available;
static uint8_t imu_address=0x68;
static SemaphoreHandle_t i2c_mutex;
static TaskHandle_t control_handle;
static QueueHandle_t command_queue,calibration_queue;
static esp_timer_handle_t control_timer;
static portMUX_TYPE state_lock=portMUX_INITIALIZER_UNLOCKED;
static fc_pilot_t shared_pilot;
static fc_status_t shared_status;
static float shared_pressure;
static uint32_t pressure_sequence,missed_ticks,max_execution_us,late_cycles;
static int64_t pressure_received_us;
static bool pressure_ok;
static unsigned gesture_events;

static float stick(uint16_t value) {
    float x=(static_cast<int32_t>(value)-32768)/32768.0f;
    constexpr float deadband=.12f;
    if(fabsf(x)<=deadband) return 0;
    return copysignf(fminf(1,(fabsf(x)-deadband)/(1-deadband)),x);
}
static void xbox_report(uint8_t *bytes,size_t length) {
    if(length!=XboxControllerNotificationParser::expectedDataLen) return;
    XboxControllerNotificationParser report;
    if(report.update(bytes,length)!=0) return;
    fc_pilot_t p={};
    p.roll=stick(report.joyRHori); p.pitch=-stick(report.joyRVert);
    p.yaw=stick(report.joyLHori); p.vertical=-stick(report.joyLVert);
    p.boost=fminf(1,report.trigRT/1023.0f);
    p.received_us=static_cast<uint64_t>(esp_timer_get_time()); p.connected=true;
    // Requests are recorded only. They never arm or directly command motors.
    static bool was_takeoff,was_land;
    bool takeoff=report.btnLB && report.btnRB && report.btnA;
    bool land=report.btnLB && report.btnRB && report.btnB;
    portENTER_CRITICAL(&state_lock);
    shared_pilot=p;
    if(takeoff && !was_takeoff) gesture_events|=1;
    if(land && !was_land) gesture_events|=2;
    portEXIT_CRITICAL(&state_lock);
    was_takeoff=takeoff; was_land=land;
}
static bool read_registers(uint8_t reg,uint8_t *data,size_t count) {
    Wire.beginTransmission(imu_address); Wire.write(reg);
    if(Wire.endTransmission(false)!=0) return false;
    if(Wire.requestFrom(imu_address,count,true)!=count) {
        while(Wire.available()) (void)Wire.read();
        return false;
    }
    for(size_t i=0;i<count;i++) data[i]=static_cast<uint8_t>(Wire.read());
    return true;
}
static bool acquire_imu(fc_sample_t &sample) {
    if(!imu_available || xSemaphoreTake(i2c_mutex,pdMS_TO_TICKS(1))!=pdTRUE) return false;
    uint8_t ready=0,bytes[14];
    bool valid=read_registers(0x3a,&ready,1) && (ready&1)
        && read_registers(0x3b,bytes,sizeof(bytes));
    xSemaphoreGive(i2c_mutex);
    if(!valid) return false;
    auto raw=[&](unsigned i) { return static_cast<int16_t>((static_cast<uint16_t>(bytes[i])<<8)|bytes[i+1]); };
    for(unsigned i=0;i<3;i++) {
        sample.accel_g[i]=raw(2*i)/4096.0f; // ±8g.
        sample.gyro_dps[i]=raw(8+2*i)/65.5f; // ±500 deg/s.
    }
    return true; // Identity sensor-to-body transform is provisional and must be checked.
}
static void timer_tick(void*) {
    if(control_handle) xTaskNotifyGive(control_handle);
}
static void control_task(void*) {
    int64_t previous=esp_timer_get_time();
    for(;;) {
        uint32_t ticks=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        int64_t start=esp_timer_get_time();
        float dt=static_cast<float>(start-previous)*1e-6f; previous=start;
        fc_sample_t raw={}; fc_pilot_t pilot; fc_status_t status;
        portENTER_CRITICAL(&state_lock);
        pilot=shared_pilot; raw.pressure_pa=shared_pressure;
        raw.pressure_sequence=pressure_sequence;
        raw.pressure_valid=pressure_ok && start>=pressure_received_us
            && start-pressure_received_us<=PRESSURE_TIMEOUT_US;
        portEXIT_CRITICAL(&state_lock);
        raw.imu_valid=acquire_imu(raw);
        char command;
        while(xQueueReceive(command_queue,&command,0)==pdTRUE) fc_cal_command(command);
        fc_step(&raw,&pilot,static_cast<uint64_t>(start),dt,&status);
        uint8_t record[FC_CAL_RECORD_BYTES];
        if(fc_cal_take_pending(record)) xQueueOverwrite(calibration_queue,record);
        // Deliberately no PWM/DShot writes, even when mathematical demands are nonzero.
        uint32_t elapsed=static_cast<uint32_t>(esp_timer_get_time()-start);
        portENTER_CRITICAL(&state_lock);
        shared_status=status;
        if(ticks>1) missed_ticks+=ticks-1;
        if(dt>.0021f) late_cycles++;
        if(elapsed>max_execution_us) max_execution_us=elapsed;
        portEXIT_CRITICAL(&state_lock);
    }
}
static void slow_task(void*) {
    uint32_t last_pressure=0,last_log=0,last_save=0;
    uint8_t pending_record[FC_CAL_RECORD_BYTES]={};
    bool save_pending=false;
    unsigned failures=0;
    for(;;) {
        xbox.onLoop(); // Discovery/reconnection may block; it is outside the control task.
        bool connected=xbox.isConnected() && !xbox.isWaitingForFirstNotification();
        portENTER_CRITICAL(&state_lock);
        if(!connected) shared_pilot.connected=false;
        portEXIT_CRITICAL(&state_lock);
        uint32_t now=millis();
        if(bmp_available && now-last_pressure>=20) {
            last_pressure=now;
            if(xSemaphoreTake(i2c_mutex,pdMS_TO_TICKS(2))==pdTRUE) {
                float pressure=bmp.readPressure();
                xSemaphoreGive(i2c_mutex);
                bool valid=isfinite(pressure) && pressure>=30000 && pressure<=110000;
                portENTER_CRITICAL(&state_lock);
                shared_pressure=pressure; pressure_ok=valid;
                if(valid) { pressure_sequence++; pressure_received_us=esp_timer_get_time(); }
                portEXIT_CRITICAL(&state_lock);
            }
        }
        if(Serial.available()) {
            char key=static_cast<char>(Serial.read());
            if(strchr("CcRrSs012345",key)) {
                if(xQueueSend(command_queue,&key,0)!=pdTRUE) Serial.println("command_queue=full");
                if(key=='C'||key=='c') Serial.println("C=status; R=clear; S=save; 0..5=+X,-X,+Y,-Y,+Z,-Z; motors=DISABLED");
            }
        }
        if(xQueueReceive(calibration_queue,pending_record,0)==pdTRUE) {
            save_pending=true; failures=0;
        }
        if(save_pending && failures<3 && now-last_save>=5000) {
            last_save=now;
            Preferences nvs;
            bool saved=false;
            if(nvs.begin("intelqcopter",false)) {
                saved=nvs.putBytes("imu-v1",pending_record,sizeof(pending_record))==sizeof(pending_record);
                uint8_t check[FC_CAL_RECORD_BYTES];
                saved=saved && nvs.getBytes("imu-v1",check,sizeof(check))==sizeof(check)
                    && memcmp(check,pending_record,sizeof(check))==0;
                nvs.end();
            }
            Serial.printf("calibration_nvs=%s; motors=DISABLED\n",saved?"saved_readback":"failed");
            if(saved) save_pending=false; else failures++;
        }
        if(now-last_log>=250) {
            last_log=now;
            fc_status_t s; uint32_t missed,execution,late; unsigned events;
            portENTER_CRITICAL(&state_lock);
            s=shared_status; missed=missed_ticks; execution=max_execution_us; late=late_cycles;
            events=gesture_events; gesture_events=0;
            portEXIT_CRITICAL(&state_lock);
            digitalWrite(LED_PIN,s.battery_warning || s.battery_disarm);
            Serial.printf("FC,ms=%lu,pilot=%d,gyro=%d,accel=%d,baro=%d,fault=%d,roll=%.3f,pitch=%.3f,h=%.3f,vz=%.3f,faces=0x%02x,capture=%d,missed=%lu,late=%lu,max_exec_us=%lu,warn35=%d,disarm20=%d,nvs_pending=%d,nvs_failures=%u,motors=DISABLED\n",
                static_cast<unsigned long>(now),s.pilot_fresh,s.gyro_verified,s.accel_calibrated,s.barometer_ready,
                s.sensor_fault,s.roll_deg,s.pitch_deg,s.height_m,s.vertical_mps,s.faces_done,s.capturing_face,
                static_cast<unsigned long>(missed),static_cast<unsigned long>(late),static_cast<unsigned long>(execution),
                s.battery_warning,s.battery_disarm,save_pending,failures);
            if(events&1) Serial.println("gesture=TAKEOFF_REQUEST_ONLY; no mission/arming action connected");
            if(events&2) Serial.println("gesture=LAND_REQUEST_ONLY; no mission/arming action connected");
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
void setup() {
    Serial.begin(115200);
    for(int pin:MOTOR_PINS) pinMode(pin,INPUT_PULLDOWN);
    pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
    i2c_mutex=xSemaphoreCreateMutex();
    command_queue=xQueueCreate(8,sizeof(char));
    calibration_queue=xQueueCreate(1,FC_CAL_RECORD_BYTES);
    configASSERT(i2c_mutex && command_queue && calibration_queue);
    Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(2);
    mpu.initialize(); imu_available=mpu.testConnection();
    if(imu_available) {
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
        mpu.setDLPFMode(MPU6050_DLPF_BW_42); mpu.setRate(1);
        mpu.setIntDataReadyEnabled(true);
    }
    bmp_available=bmp.begin(0x76) || bmp.begin(0x77);
    if(bmp_available) bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X1,Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::FILTER_X4,Adafruit_BMP280::STANDBY_MS_1);
    fc_init(ESP.getEfuseMac());
    uint8_t record[FC_CAL_RECORD_BYTES]; bool loaded=false;
    Preferences nvs;
    if(nvs.begin("intelqcopter",true)) {
        if(nvs.getBytesLength("imu-v1")==sizeof(record)
            && nvs.getBytes("imu-v1",record,sizeof(record))==sizeof(record)) loaded=fc_cal_load(record);
        nvs.end();
    }
    Serial.printf("IntelQCopter ESP32-S3/Xbox; imu=%d,bmp=%d,stored_cal=%d; MOTOR OUTPUTS DISABLED\n",
        imu_available,bmp_available,loaded);
    Serial.println("Mode2: LX=yaw, LY=vertical speed, RX=roll, RY=pitch, RT=boost; 0..5=six-face capture");
    xbox.begin();
    configASSERT(xTaskCreatePinnedToCore(control_task,"fc500",8192,nullptr,5,&control_handle,0)==pdPASS);
    configASSERT(xTaskCreatePinnedToCore(slow_task,"xbox_slow",12288,nullptr,1,nullptr,1)==pdPASS);
    esp_timer_create_args_t args={};
    args.callback=timer_tick; args.name="fc_tick"; args.dispatch_method=ESP_TIMER_TASK;
    ESP_ERROR_CHECK(esp_timer_create(&args,&control_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer,CONTROL_PERIOD_US));
}
void loop() { delay(1000); }
