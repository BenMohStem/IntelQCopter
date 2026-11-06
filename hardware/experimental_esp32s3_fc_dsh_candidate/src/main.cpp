/*
 * IntelQCopter — ESP32-S3 Competition Flight Controller
 * ======================================================
 *
 * Target:  ESP32-S3 DevKitC-1 N8, 8 MB flash, no PSRAM
 * Framework: Arduino / PlatformIO
 * Control rate: 500 Hz deterministic (2 ms cycle)
 * UART: 921600 baud, 8N1, 52-byte CRC-protected v3 binary packets
 * I²C: 400 kHz for ALL sensors (MPU6050, BMP280, QMC5883P)
 * Motor outputs: DISABLED during upload, calibration, and diagnostics
 *
 * Architecture:
 *   500 Hz control loop (IMU, MAG, Baro, LiDAR, estimation, PID, mixing)
 *   Motor GPIOs 12/11/14/13 = INPUT ONLY until explicit arming
 *   No auto-arm on startup/reconnect
 *
 * Safety:
 *   Motor outputs never enabled in this build
 *   Arming requires deliberate Xbox button combination (LB+RB+Y hold)
 *   Failsafe on controller loss, sensor loss, estimator loss
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <climits>
#include <initializer_list>

#ifndef IQ_WIFI_ENABLED
#define IQ_WIFI_ENABLED 0
#endif
#ifndef IQ_I2C_HZ
#define IQ_I2C_HZ 400000
#endif

#if IQ_WIFI_ENABLED != 0
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

namespace cfg {

// I²C pins (ESP32-S3 DevKitC-1)
constexpr uint8_t SDA_PIN = 9;
constexpr uint8_t SCL_PIN = 8;

// All three sensors share the configured 400 kHz I²C bus.
constexpr uint32_t I2C_HZ = IQ_I2C_HZ;
constexpr uint32_t QMC_I2C_HZ = I2C_HZ;

// Sensor addresses
constexpr uint8_t MPU_PRIMARY_ADDRESS = 0x68;
constexpr uint8_t MPU_ALTERNATE_ADDRESS = 0x69;
constexpr uint8_t BMP_PRIMARY_ADDRESS = 0x76;
constexpr uint8_t BMP_ALTERNATE_ADDRESS = 0x77;
constexpr uint8_t MAG_ADDRESS = 0x2C;   // QMC5883P

// Motor pins: FL, FR, RR, RL — INPUT ONLY until armed
constexpr uint8_t MOTOR_PINS[4] = {12, 11, 14, 13};

// LiDAR UART pins
constexpr uint8_t LIDAR_RX = 1;
constexpr uint8_t LIDAR_TX = 2;

// Timing
constexpr uint32_t CONTROL_PERIOD_US = 2000;   // 500 Hz
constexpr uint32_t IMU_PERIOD_US = 2000;       // 500 Hz
constexpr uint32_t MAG_PERIOD_US = 5000;       // 200 Hz
constexpr uint32_t BMP_PERIOD_US = 6000;       // ~166.7 Hz poll target; conversion timing unverified
constexpr uint32_t LIDAR_PERIOD_US = 20000;    // 50 Hz
constexpr uint32_t MAG_STALE_MS = 25;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 250;
// Provisional software freshness limits; the board has not been measured.
constexpr uint32_t IMU_MAX_AGE_US = 4000;
constexpr uint32_t BMP_MAX_AGE_US = 20000;
constexpr float GRAVITY_MPS2 = 9.80665f;
// No verified Xbox/LoRa adapter is wired to this candidate's USB telemetry port.
constexpr bool COMMAND_INPUT_ENABLED = false;

// UART telemetry
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t PACKET_SIZE = 52;

// MPU6050 register map
constexpr uint8_t MPU_REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t MPU_REG_PWR_MGMT_2    = 0x6C;
constexpr uint8_t MPU_REG_CONFIG        = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t MPU_REG_FIFO_EN       = 0x23;
constexpr uint8_t MPU_REG_INT_PIN_CFG   = 0x37;
constexpr uint8_t MPU_REG_INT_ENABLE    = 0x38;
constexpr uint8_t MPU_REG_USER_CTRL     = 0x6A;
constexpr uint8_t MPU_REG_FIFO_COUNT_H  = 0x72;
constexpr uint8_t MPU_REG_FIFO_R_W      = 0x74;
constexpr uint8_t MPU_REG_WHO_AM_I      = 0x75;

// MPU6050 register values
constexpr uint8_t MPU_PWR_MGMT_1_RESET      = 0x80;
constexpr uint8_t MPU_PWR_MGMT_1_CLKSEL     = 0x01;
constexpr uint8_t MPU_DLPF_188HZ            = 0x01;   // 188/188 Hz
constexpr uint8_t MPU_GYRO_FS_500           = 0x08;   // ±500 deg/s
constexpr uint8_t MPU_ACCEL_FS_8G           = 0x10;   // ±8 g
constexpr uint8_t MPU_FIFO_TEMP_EN          = 0x80;
constexpr uint8_t MPU_FIFO_XG_EN            = 0x40;
constexpr uint8_t MPU_FIFO_YG_EN            = 0x20;
constexpr uint8_t MPU_FIFO_ZG_EN            = 0x10;
constexpr uint8_t MPU_FIFO_ACCEL_EN         = 0x08;
constexpr uint8_t MPU_USER_CTRL_FIFO_EN     = 0x40;
constexpr uint8_t MPU_USER_CTRL_FIFO_RESET  = 0x04;
constexpr uint8_t MPU_INT_PIN_CFG_BYPASS_EN = 0x02;
constexpr uint8_t MPU_INT_ENABLE_RAW_RDY_EN = 0x01;

// QMC5883P register map
constexpr uint8_t QMC_REG_CHIP_ID   = 0x00;
constexpr uint8_t QMC_REG_CTRL1     = 0x0A;
constexpr uint8_t QMC_REG_CTRL2     = 0x0B;
constexpr uint8_t QMC_REG_SET_PERIOD= 0x0C;
constexpr uint8_t QMC_REG_XYZ_SIGN  = 0x29;
constexpr uint8_t QMC_REG_DATA_XL   = 0x01;
constexpr uint8_t QMC_REG_STATUS    = 0x09;

constexpr uint8_t QMC_CHIP_ID       = 0x80;
constexpr uint8_t QMC_CTRL1_200HZ   = 0x3D;   // ODR=200Hz, OSR1=1, OSR2=1
constexpr uint8_t QMC_CTRL2_8GAUSS  = 0x08;   // ±8 gauss, set/reset
constexpr uint8_t QMC_XYZ_SIGN_VAL  = 0x06;
constexpr float QMC_COUNTS_PER_UT   = 37.5f; // QMC5883P, ±8 G GUI scale

// BMP280 registers
constexpr uint8_t BMP_REG_ID           = 0xD0;
constexpr uint8_t BMP_REG_STATUS       = 0xF3;
constexpr uint8_t BMP_REG_RESET        = 0xE0;
constexpr uint8_t BMP_REG_CTRL_MEAS    = 0xF4;
constexpr uint8_t BMP_REG_CONFIG       = 0xF5;
constexpr uint8_t BMP_REG_PRESSURE_MSB = 0xF7;

constexpr uint8_t BMP_CHIP_ID          = 0x58;
// Xbox button mapping
constexpr uint8_t XBOX_BTN_LB   = 1u << 0;
constexpr uint8_t XBOX_BTN_RB   = 1u << 1;
constexpr uint8_t XBOX_BTN_NORTH= 1u << 2;
constexpr uint8_t XBOX_BTN_EAST = 1u << 3;
constexpr uint8_t XBOX_BTN_SOUTH= 1u << 4;
constexpr uint8_t XBOX_BTN_WEST = 1u << 5;

constexpr uint8_t ARM_BUTTONS = XBOX_BTN_LB | XBOX_BTN_RB;
constexpr uint8_t COMMAND_MAGIC_0 = 0x51;
constexpr uint8_t COMMAND_MAGIC_1 = 0x43;
constexpr uint8_t COMMAND_VERSION = 1;

// Motor policy (ESC command fractions, not RPM)
constexpr float FC_ARMED_IDLE_FRACTION = 0.10f;
constexpr float FC_NORMAL_MOTOR_CEILING = 0.35f;
constexpr float FC_BOOST_MOTOR_CEILING = 0.50f;
constexpr float FC_MAX_VERTICAL_SPEED_MPS = 2.0f;
constexpr float FC_MAX_CLIMB_MPS = 1.5f;
constexpr float FC_MAX_DESCENT_MPS = 1.0f;

// Control limits
constexpr float MAX_ROLL_DEG = 35.0f;
constexpr float MAX_PITCH_DEG = 35.0f;
constexpr float MAX_YAW_RATE_DPS = 180.0f;

} // namespace cfg

// ============================================================================
// I²C TRANSACTION MANAGER
// ============================================================================

class I2cTransactionManager {
public:
  struct Stats {
    uint32_t total_transactions = 0;
    uint32_t successful = 0;
    uint32_t failures = 0;
    uint32_t timeouts = 0;
    uint32_t bus_recoveries = 0;
    uint32_t mpu_transactions = 0;
    uint32_t bmp_transactions = 0;
    uint32_t mag_transactions = 0;
    uint32_t max_transaction_us = 0;
    uint32_t total_transaction_us = 0;
  } stats;

  I2cTransactionManager() = default;

  bool begin() {
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN);
    Wire.setClock(cfg::I2C_HZ);
    Wire.setTimeOut(2);
    return true;
  }

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint32_t start = micros();
    bool ok = i2cWrite(addr, reg, &value, 1);
    recordTransaction(ok, micros() - start);
    return ok;
  }

  bool readRegs(uint8_t addr, uint8_t reg, uint8_t *out, size_t count) {
    uint32_t start = micros();
    bool ok = i2cRead(addr, reg, out, count);
    recordTransaction(ok, micros() - start);
    return ok;
  }

  // The QMC path uses a STOP between its register-pointer write and read.
  // The previous calibration capture recorded repeated-START failures here.
  bool readQmcRegs(uint8_t reg, uint8_t *out, size_t count) {
    const uint32_t start = micros();
    const bool ok = i2cReadStopStart(cfg::MAG_ADDRESS, reg, out, count);
    recordTransaction(ok, micros() - start);
    return ok;
  }

  bool readQmcByte(uint8_t reg, uint8_t &value) {
    return readQmcRegs(reg, &value, 1);
  }

  void setClock(uint32_t hz) { Wire.setClock(hz); }

  bool readByte(uint8_t addr, uint8_t reg, uint8_t &value) {
    return readRegs(addr, reg, &value, 1);
  }

  bool burstRead(uint8_t addr, uint8_t reg, uint8_t *out, size_t count) {
    return readRegs(addr, reg, out, count);
  }

  bool recoverBus() {
    stats.bus_recoveries++;
    pinMode(cfg::SCL_PIN, OUTPUT);
    pinMode(cfg::SDA_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(cfg::SCL_PIN, LOW);
      delayMicroseconds(5);
      digitalWrite(cfg::SCL_PIN, HIGH);
      delayMicroseconds(5);
    }
    digitalWrite(cfg::SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(cfg::SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(cfg::SDA_PIN, HIGH);
    delayMicroseconds(5);
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN);
    Wire.setClock(cfg::I2C_HZ);
    Wire.setTimeOut(2);
    return true;
  }

  bool isPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

private:
  bool i2cWrite(uint8_t addr, uint8_t reg, const uint8_t *values, size_t count) {
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) { Wire.endTransmission(); return false; }
    for (size_t i = 0; i < count; i++)
      if (Wire.write(values[i]) != 1) { Wire.endTransmission(); return false; }
    return Wire.endTransmission() == 0;
  }

  bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *out, size_t count) {
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) { Wire.endTransmission(); return false; }
    if (Wire.endTransmission(false) != 0) return false;
    int received = Wire.requestFrom((int)addr, (int)count);
    if (received != (int)count) {
      while (Wire.available()) (void)Wire.read();
      return false;
    }
    for (size_t i = 0; i < count; i++) {
      if (Wire.available() <= 0) return false;
      out[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
  }

  bool i2cReadStopStart(uint8_t addr, uint8_t reg, uint8_t *out, size_t count) {
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) {
      (void)Wire.endTransmission(true);
      return false;
    }
    if (Wire.endTransmission(true) != 0) return false;
    const int received = Wire.requestFrom((int)addr, (int)count);
    if (received != (int)count) {
      while (Wire.available() > 0) (void)Wire.read();
      return false;
    }
    for (size_t i = 0; i < count; ++i) {
      if (Wire.available() <= 0) return false;
      out[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
  }

  bool i2cPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

  void recordTransaction(bool ok, uint32_t duration_us) {
    stats.total_transactions++;
    if (ok) stats.successful++; else stats.failures++;
    if (duration_us > stats.max_transaction_us) stats.max_transaction_us = duration_us;
    stats.total_transaction_us += duration_us;
  }
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct CalVector3 { float x, y, z; };
struct CalMatrix3x3 { float m[3][3]; };

struct AccelCalibration {
  CalVector3 bias; CalMatrix3x3 scale; float rms_mps2; bool valid;
};
struct MagCalibration {
  CalVector3 hard_iron; CalMatrix3x3 soft_iron; float radius_mean_uT; bool valid;
};
struct BaroCalibration {
  float ground_pressure_pa, reference_variance_pa2, measurement_variance_pa2; bool valid;
};
struct GyroCalibration {
  CalVector3 bias_dps; float variance_dps2[3]; bool valid;
};

struct ImuSample {
  bool valid=false, fresh=false; uint32_t timestamp_us=0;
  int16_t accel_raw[3]={}, gyro_raw[3]={}, temp_raw=0;
  float accel[3]={}, gyro[3]={}, temperature_c=0.0f;
  float vertical_acceleration_world_mps2=0.0f;
  uint32_t fifo_count=0, overflows=0;
};
struct MagSample {
  bool valid=false, fresh=false; uint32_t timestamp_us=0;
  int16_t raw[3]={}; float calibrated[3]={}; uint8_t status=0;
};
struct BaroSample {
  bool valid=false, fresh=false; uint32_t timestamp_us=0;
  uint32_t raw_pressure=0, raw_temperature=0;
  float temperature_c=0.0f, pressure_pa=0.0f;
  float height_m=0.0f, height_sigma_m=0.0f;
};
struct LidarSample {
  bool valid=false, fresh=false; uint32_t timestamp_us=0;
  float distance_m=0.0f, quality=0.0f; bool tilt_valid=false;
};

struct SystemHealth {
  uint32_t loop_count=0, missed_deadlines=0, consecutive_ok=0;
  uint32_t imu_errors=0, mag_errors=0, baro_errors=0, lidar_errors=0;
  uint32_t controller_losses=0, arming_attempts=0, disarms=0, failsafes=0;
  uint32_t serial_drops=0, last_reset_reason=0, uptime_us=0;
};

struct ControlState {
  bool armed=false, arm_requested=false, disarmed=true, failsafe=false;
  bool controller_connected=false;
  uint32_t last_controller_ms=0, arm_time_ms=0, disarm_time_ms=0;
  float hover_collective=0.0f;
  bool hover_collective_valid=false;
  float lt_reduce_state=0.0f, boost_state=0.0f;
  float roll_angle_ref=0.0f, pitch_angle_ref=0.0f, yaw_heading_ref=0.0f;
  float altitude_ref_m=0.0f; bool altitude_ref_initialized=false;
};

struct PilotCommand {
  bool valid=false; uint32_t timestamp_ms=0; uint8_t buttons=0;
  float roll=0.0f, pitch=0.0f, yaw=0.0f, throttle=0.0f, rt=0.0f, lt=0.0f;
};

struct MotorOutput {
  float motor[4]={0,0,0,0}; bool armed=false, sensor_fault=false, failsafe=false;
};

// ============================================================================
// GLOBAL STATE (namespace iq)
// ============================================================================

namespace iq {

uint16_t crcCcittFalse(const uint8_t*d,size_t s);

I2cTransactionManager i2c_mgr;
Adafruit_BMP280 *bmp_driver = nullptr;

uint8_t mpu_address=0, bmp_address=0, mag_address=cfg::MAG_ADDRESS;
bool mpu_present=false, bmp_present=false, mag_configured=false, lidar_present=false;

ImuSample imu_sample;
MagSample mag_sample;
BaroSample baro_sample;
LidarSample lidar_sample;

SystemHealth health;
AccelCalibration cal_accel={};
MagCalibration cal_mag={};
BaroCalibration cal_baro={};
GyroCalibration cal_gyro={};
bool calibration_loaded=false;

ControlState control;
PilotCommand pilot;
MotorOutput motor_output;

struct Quaternion { float w,x,y,z; Quaternion():w(1),x(0),y(0),z(0) {}
  void normalize() { float n=sqrtf(w*w+x*x+y*y+z*z);
    if(n>0.001f){w/=n;x/=n;y/=n;z/=n;} else {w=1;x=0;y=0;z=0;} }
  void toEuler(float&roll,float&pitch,float&yaw) const {
    float sr=2*(w*x+y*y), cr=1-2*(x*x+y*y); roll=atan2f(sr,cr)*57.2957795f;
    float sp=2*(w*y-z*x);
    pitch = fabsf(sp)>=1.0f? copysignf(90.0f,sp) : asinf(sp)*57.2957795f;
    float sy=2*(w*z+x*y), cy=1-2*(y*y+z*z); yaw=atan2f(sy,cy)*57.2957795f;
  }
};

struct AttitudeEstimator {
  Quaternion q; float gyro_bias[3]={0,0,0}; float accel_bias[3]={0,0,0};
  float roll_deg=0, pitch_deg=0, yaw_deg=0;
  float roll_rate_dps=0, pitch_rate_dps=0, yaw_rate_dps=0;
  bool initialized=false; uint32_t last_update_us=0;
  float kp_roll=0.5f, kp_pitch=0.5f, kp_yaw=0.1f;

  void init(const float accel[3]) {
    float roll=atan2f(accel[1],accel[2])*57.2957795f;
    float pitch=atan2f(-accel[0],sqrtf(accel[1]*accel[1]+accel[2]*accel[2]))*57.2957795f;
    float cr=cosf(roll*0.0174532925f), sr=sinf(roll*0.0174532925f);
    float cp=cosf(pitch*0.0174532925f), sp=sinf(pitch*0.0174532925f);
    q.w=cr*cp*1+sr*sp*0; q.x=sr*cp*1-cr*sp*0; q.y=cr*sp*1+sr*cp*0; q.z=cr*cp*0-sr*sp*0;
    q.normalize();
    roll_deg=roll; pitch_deg=pitch; yaw_deg=0; initialized=true;
    last_update_us=esp_timer_get_time();
  }

  // q maps body-frame vectors into the local navigation frame.
  // The accelerometer input is specific force and still includes gravity.
  float worldVerticalSpecificForce(const float accel_body[3]) const {
    const float r20=2.0f*(q.x*q.z-q.w*q.y);
    const float r21=2.0f*(q.y*q.z+q.w*q.x);
    const float r22=1.0f-2.0f*(q.x*q.x+q.y*q.y);
    return r20*accel_body[0]+r21*accel_body[1]+r22*accel_body[2];
  }

  void update(const float gyro_dps[3], const float accel_mps2[3],
              const float mag_uT[3], bool mag_valid, float dt) {
    if(!initialized || !isfinite(dt) || dt<=0.0f || dt>0.02f) return;
    float gx=gyro_dps[0]-gyro_bias[0], gy=gyro_dps[1]-gyro_bias[1], gz=gyro_dps[2]-gyro_bias[2];
    if(accel_mps2) {
      float n=sqrtf(accel_mps2[0]*accel_mps2[0]+accel_mps2[1]*accel_mps2[1]+accel_mps2[2]*accel_mps2[2]);
      if(n>0.5f*cfg::GRAVITY_MPS2&&n<2.0f*cfg::GRAVITY_MPS2){
        float ax=accel_mps2[0]/n, ay=accel_mps2[1]/n, az=accel_mps2[2]/n;
        float vx=2*(q.x*q.z-q.w*q.y), vy=2*(q.y*q.z+q.w*q.x), vz=q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z;
        float ex=ay*vz-az*vy, ey=az*vx-ax*vz, ez=ax*vy-ay*vx;
        // The correction gain is in s^-1; convert its rad/s correction to deg/s.
        constexpr float RAD_TO_DEG=57.2957795f;
        gx+=kp_roll*ex*RAD_TO_DEG;
        gy+=kp_pitch*ey*RAD_TO_DEG;
        gz+=kp_roll*ez*RAD_TO_DEG;
      }
    }
    if(mag_valid&&mag_uT) {
      float mx=mag_uT[0], my=mag_uT[1], mz=mag_uT[2];
      float emx=2*(q.w*q.x+q.y*q.z)*mx+2*(q.w*q.y-q.x*q.z)*my+(q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z)*mz;
      float emy=2*(q.x*q.y-q.w*q.z)*mx+(q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)*my+2*(q.w*q.x+q.y*q.z)*mz;
      float heading=atan2f(-emy,emx)*57.2957795f;
      float err=heading-yaw_deg;
      while(err>180)err-=360; while(err<-180)err+=360;
      gz+=kp_yaw*err;
    }
    // Apply the corrected gyro rate once. The previous version first integrated
    // the raw rate, then integrated the corrected rate over the same dt again.
    const float half_dt=0.5f*dt*0.0174532925f;
    const float qx=gx*half_dt, qy=gy*half_dt, qz=gz*half_dt;
    const float angle=sqrtf(qx*qx+qy*qy+qz*qz);
    Quaternion dq; dq.w=cosf(angle);
    if(angle>1e-7f){const float scale=sinf(angle)/angle; dq.x=qx*scale; dq.y=qy*scale; dq.z=qz*scale;}
    Quaternion next;
    next.w=q.w*dq.w-q.x*dq.x-q.y*dq.y-q.z*dq.z;
    next.x=q.w*dq.x+q.x*dq.w+q.y*dq.z-q.z*dq.y;
    next.y=q.w*dq.y-q.x*dq.z+q.y*dq.w+q.z*dq.x;
    next.z=q.w*dq.z+q.x*dq.y-q.y*dq.x+q.z*dq.w;
    q=next; q.normalize();
    q.toEuler(roll_deg,pitch_deg,yaw_deg);
    roll_rate_dps=gx; pitch_rate_dps=gy; yaw_rate_dps=gz;
    last_update_us=esp_timer_get_time();
  }
};

struct VerticalKf {
  float height_m=0, velocity_mps=0, accel_bias_mps2=0;
  // Two-state [height, vertical velocity] covariance. Acceleration bias is a
  // fixed calibration input here; it is not estimated by this two-state filter.
  float p00=0.04f,p01=0,p10=0,p11=0.1f;
  float accel_noise=1.5f, baro_noise=0.0025f, lidar_noise=0.01f;
  bool initialized=false; uint32_t last_update_us=0;
  void init(float h,float v){height_m=h;velocity_mps=v;accel_bias_mps2=0;
    p00=0.04f;p01=0;p10=0;p11=0.1f;initialized=true;last_update_us=esp_timer_get_time();}
  void predict(float accel_up,float dt){
    if(!initialized||!isfinite(accel_up)||!isfinite(dt)||dt<=0.0f||dt>0.02f)return;
    float a=accel_up-accel_bias_mps2;
    height_m+=velocity_mps*dt+0.5f*a*dt*dt; velocity_mps+=a*dt;
    float dt2=dt*dt,dt3=dt2*dt,dt4=dt3*dt,q=accel_noise*accel_noise;
    p00+=dt*(p01+p10)+dt2*p11+0.25f*dt4*q;
    p01+=dt*p11+0.5f*dt3*q; p10+=dt*p11+0.5f*dt3*q; p11+=dt2*q;
    last_update_us=esp_timer_get_time();}
  void updateMeasurement(float measured_height_m,float sigma_m){
    if(!initialized||!isfinite(measured_height_m)||!isfinite(sigma_m)||sigma_m<=0.0f)return;
    const float r=fmaxf(sigma_m*sigma_m,1e-6f);
    const float innovation=measured_height_m-height_m;
    const float innovation_variance=p00+r;
    if(!isfinite(innovation_variance)||innovation_variance<=0.0f)return;
    // Reject implausible jumps instead of dragging the estimate across the map.
    if(fabsf(innovation)>5.0f*sqrtf(innovation_variance))return;
    const float k0=p00/innovation_variance;
    const float k1=p10/innovation_variance;
    height_m+=k0*innovation;
    velocity_mps+=k1*innovation;

    // Joseph covariance update, using the old covariance for every term.
    const float old_p00=p00, old_p01=p01, old_p10=p10, old_p11=p11;
    const float a=1.0f-k0;
    const float new_p00=a*a*old_p00+k0*k0*r;
    const float new_p01=a*(old_p01-k1*old_p00)+k0*k1*r;
    const float new_p10=a*(old_p10-k1*old_p00)+k1*k0*r;
    const float new_p11=old_p11-k1*old_p01-k1*old_p10
                       +k1*k1*old_p00+k1*k1*r;
    p00=fmaxf(new_p00,1e-8f);
    p01=p10=0.5f*(new_p01+new_p10);
    p11=fmaxf(new_p11,1e-8f);
    last_update_us=esp_timer_get_time();
  }
  void updateBaro(float h,float sigma){updateMeasurement(h,sigma);}
  void updateLidar(float h,float sigma){updateMeasurement(h,sigma);}
};

AttitudeEstimator attitude;
VerticalKf vert_kf;

struct PidController {
  float kp=0,ki=0,kd=0,integrator=0,prev_meas=0,integrator_limit=0,output_limit=0;
  bool initialized=false; float deriv_filt=0, deriv_alpha=0.7f;
  void reset(){integrator=0;prev_meas=0;initialized=false;deriv_filt=0;}
  float update(float sp,float meas,float dt){
    if(!isfinite(sp)||!isfinite(meas)||!isfinite(dt)||dt<=0.0f||dt>0.02f
        ||!isfinite(kp)||!isfinite(ki)||!isfinite(kd)
        ||!isfinite(integrator_limit)||integrator_limit<0.0f
        ||!isfinite(output_limit)||output_limit<0.0f
        ||!isfinite(deriv_alpha)||deriv_alpha<0.0f||deriv_alpha>1.0f){
      reset();return 0.0f;
    }
    float err=sp-meas;
    float der=0;
    if(initialized){der=(meas-prev_meas)/dt; deriv_filt=deriv_alpha*deriv_filt+(1-deriv_alpha)*der; der=deriv_filt;}
    float ci=integrator+err*dt; ci=fmaxf(-integrator_limit,fminf(integrator_limit,ci));
    float raw=kp*err+ki*ci-kd*der;
    if(!isfinite(raw)||!isfinite(ci)){reset();return 0.0f;}
    float out=fmaxf(-output_limit,fminf(output_limit,raw));
    if(raw==out||(raw>out&&err<0)||(raw<out&&err>0)) integrator=ci;
    prev_meas=meas; initialized=true; return out;}
};

PidController pid_roll_angle, pid_pitch_angle, pid_roll_rate, pid_pitch_rate;
PidController pid_yaw_rate, pid_altitude, pid_vertical_speed;

bool pidReady(const PidController&p){
  return isfinite(p.kp)&&p.kp>0.0f&&isfinite(p.ki)&&p.ki>=0.0f
      &&isfinite(p.kd)&&p.kd>=0.0f&&isfinite(p.integrator_limit)
      &&p.integrator_limit>=0.0f&&isfinite(p.output_limit)
      &&p.output_limit>0.0f&&isfinite(p.deriv_alpha)
      &&p.deriv_alpha>=0.0f&&p.deriv_alpha<=1.0f;
}

bool pidGainsConfigured(){
  return pidReady(pid_roll_angle)&&pidReady(pid_pitch_angle)
      &&pidReady(pid_roll_rate)&&pidReady(pid_pitch_rate)
      &&pidReady(pid_yaw_rate)&&pidReady(pid_altitude)
      &&pidReady(pid_vertical_speed);
}

struct MotorMixer {
  float last_cmd[4]={0,0,0,0}; float max_slew=10.0f;
  void mix(float coll,float roll,float pitch,float yaw,float ceil,float idle,float dt,float out[4]){
    if(out==nullptr||!isfinite(coll)||!isfinite(roll)||!isfinite(pitch)
        ||!isfinite(yaw)||!isfinite(ceil)||!isfinite(idle)||!isfinite(dt)
        ||dt<=0.0f||dt>0.02f||idle<0.0f||ceil>1.0f||ceil<idle){
      reset();
      if(out!=nullptr)for(int i=0;i<4;i++)out[i]=0.0f;
      return;
    }
    float cor[4]={roll-pitch-yaw,-roll-pitch+yaw,-roll+pitch-yaw,roll+pitch+yaw};
    float lo=cor[0],hi=cor[0]; for(int i=1;i<4;i++){lo=fminf(lo,cor[i]);hi=fmaxf(hi,cor[i]);}
    float span=ceil-idle; float scale=(hi-lo>span)?span/(hi-lo):1.0f;
    float alloc=fmaxf(idle,fminf(ceil-hi*scale,coll)); alloc=fminf(alloc,ceil-lo*scale);
    float maxd=max_slew*dt;
    for(int i=0;i<4;i++){
      float r=alloc+scale*cor[i]; r=fmaxf(idle,fminf(ceil,r));
      float d=r-last_cmd[i]; d=fmaxf(-maxd,fminf(maxd,d)); out[i]=fmaxf(idle,fminf(ceil,last_cmd[i]+d));
      last_cmd[i]=out[i];}}
  void reset(){for(int i=0;i<4;i++)last_cmd[i]=0;}
};

MotorMixer motor_mixer;

struct XboxInterface {
  HardwareSerial &serial; bool connected=false; uint32_t last_pkt_ms=0;
  uint8_t last_btn=0; bool arm_seq=false; uint32_t arm_hold_ms=0;
  uint16_t last_sequence=0; bool sequence_seen=false;
  float stick_dz=0.05f, trig_dz=0.02f;
  XboxInterface(HardwareSerial&s):serial(s){}
  bool parseCmd(const uint8_t*d,size_t len,PilotCommand&c){
    if(d==nullptr||len!=22u||d[0]!=cfg::COMMAND_MAGIC_0
        ||d[1]!=cfg::COMMAND_MAGIC_1||d[2]!=cfg::COMMAND_VERSION||d[3]!=0u)
      return false;
    const uint16_t received_crc=uint16_t(d[20])|(uint16_t(d[21])<<8);
    if(crcCcittFalse(d,20u)!=received_crc)return false;
    const uint16_t seq=uint16_t(d[4])|(uint16_t(d[5])<<8);
    const int16_t rr=(int16_t)(uint16_t(d[6])|(uint16_t(d[7])<<8));
    const int16_t pr=(int16_t)(uint16_t(d[8])|(uint16_t(d[9])<<8));
    const int16_t yr=(int16_t)(uint16_t(d[10])|(uint16_t(d[11])<<8));
    const uint16_t th=uint16_t(d[12])|(uint16_t(d[13])<<8);
    const uint16_t lt=uint16_t(d[14])|(uint16_t(d[15])<<8);
    const uint16_t rt=uint16_t(d[16])|(uint16_t(d[17])<<8);
    const uint16_t bt=uint16_t(d[18])|(uint16_t(d[19])<<8);
    if(rr < -10000 || rr > 10000 || pr < -10000 || pr > 10000
        || yr < -10000 || yr > 10000 || th > 10000u
        || lt > 1000u || rt > 1000u || (bt&uint16_t(~0x003Fu))!=0u)
      return false;
    if(sequence_seen){
      const uint16_t delta=uint16_t(seq-last_sequence);
      if(delta==0u||delta>=0x8000u)return false;
    }
    c.roll=dz((float)rr/10000.0f,stick_dz);
    // The hand-controller frame already applies the pitch stick sign.
    c.pitch=dz((float)pr/10000.0f,stick_dz);
    c.yaw=dz((float)yr/10000.0f,stick_dz);
    c.throttle=dz(((float)th-5000.0f)/5000.0f,stick_dz);
    c.rt=dz((float)rt/1000.0f,trig_dz); c.lt=dz((float)lt/1000.0f,trig_dz);
    c.buttons=(uint8_t)bt; c.timestamp_ms=millis(); c.valid=true;
    last_sequence=seq; sequence_seen=true;
    connected=true; last_pkt_ms=c.timestamp_ms;
    return true;}
  float dz(float v,float z){if(fabsf(v)<z)return 0; return v>0?(v-z)/(1-z):(v+z)/(1-z);}
  bool connectedOk(){return connected&&(millis()-last_pkt_ms<cfg::CONTROLLER_TIMEOUT_MS);}
  bool updateArm(PilotCommand&c,bool&armed,bool&changed){
    changed=false;
    if(!connectedOk()){
      if(armed){armed=false;changed=true;health.disarms++;health.failsafes++;}
      arm_seq=false; arm_hold_ms=0; last_btn=0;
      return false;
    }
    bool hold=(c.buttons&cfg::ARM_BUTTONS)==cfg::ARM_BUTTONS;
    const bool arm_combo=(c.buttons&(cfg::ARM_BUTTONS|cfg::XBOX_BTN_NORTH))
                       ==(cfg::ARM_BUTTONS|cfg::XBOX_BTN_NORTH);
    const bool south_pressed=(c.buttons&cfg::XBOX_BTN_SOUTH)
                          &&!(last_btn&cfg::XBOX_BTN_SOUTH);
    if(!armed){
      if(hold&&(c.buttons&cfg::XBOX_BTN_NORTH)){
        if(!arm_seq){arm_seq=true;arm_hold_ms=c.timestamp_ms;}
        if(arm_seq&&arm_combo
            &&uint32_t(c.timestamp_ms-arm_hold_ms)>=500u){
          armed=true; changed=true; control.arm_time_ms=c.timestamp_ms;
          health.arming_attempts++; arm_seq=false;
        }
      } else {
        arm_seq=false; arm_hold_ms=0;
      }
    } else if(south_pressed){
      armed=false; changed=true; control.disarm_time_ms=c.timestamp_ms;
      health.disarms++;
    }
    last_btn=c.buttons;
    return armed;
  }
};

XboxInterface xbox_interface(Serial);

float clampf(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}

bool timestampWithinUs(uint32_t timestamp_us,uint32_t maximum_age_us){
  return timestamp_us!=0u
      && uint32_t(micros()-timestamp_us)<=maximum_age_us;
}

uint16_t crcCcittFalse(const uint8_t*d,size_t s){
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<s;i++){crc^=d[i]<<8; for(uint8_t b=0;b<8;b++) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;}
  return crc;}

struct TelemetryPacket {
  uint8_t bytes[cfg::PACKET_SIZE]={}; size_t len=0;
  void u8(uint8_t v){ if(len<cfg::PACKET_SIZE) bytes[len++]=v; }
  void u16(uint16_t v){ u8(v&0xFF); u8((v>>8)&0xFF); }
  void i16(int16_t v){ u16((uint16_t)v); }
  void u32(uint32_t v){ u8(v&0xFF); u8((v>>8)&0xFF); u8((v>>16)&0xFF); u8((v>>24)&0xFF); }
};

void buildTelemetry(TelemetryPacket&pkt,const ImuSample&imu,const MagSample&mag,
                    const BaroSample&baro,const LidarSample&lidar,
                    const AttitudeEstimator&att,const VerticalKf&vkf,
                    const MotorOutput&out,uint32_t seq,uint32_t ts,uint16_t late,
                    const I2cTransactionManager::Stats&st){
  uint8_t flags=0;
  flags|=imu.valid?0x01:0; flags|=baro.valid?0x02:0; flags|=mag.valid?0x04:0;
  flags|=baro.fresh?0x08:0; flags|=mag.fresh?0x10:0;
  flags|=(late>250)?0x20:0; // Same flag layout consumed by the calibration GUI.
  flags|=0x40;              // Sensor identity: QMC5883P.
  pkt.len=0; pkt.u8(0xA5); pkt.u8(0x5A); pkt.u8(3); pkt.u8(flags&0xFF);
  pkt.u32(seq); pkt.u32(ts); pkt.u16(late);
  for(int i=0;i<3;i++) pkt.i16(imu.accel_raw[i]);
  for(int i=0;i<3;i++) pkt.i16(imu.gyro_raw[i]);
  pkt.i16(imu.temp_raw);
  for(int i=0;i<3;i++) pkt.i16(mag.raw[i]);
  pkt.u32(baro.raw_pressure); pkt.u32(baro.raw_temperature);
  pkt.i16(int16_t(baro.temperature_c*100.0f));
  pkt.u32(uint32_t(baro.pressure_pa*100.0f));
  pkt.u16(uint16_t(health.missed_deadlines));
  if(pkt.len!=cfg::PACKET_SIZE-2){pkt.len=0;return;}
  pkt.u16(crcCcittFalse(pkt.bytes,pkt.len));
}

// ============================================================================
// CALIBRATION MATH
// ============================================================================

bool applyAccelCal(const float raw[3],float out[3]){
  if(!cal_accel.valid)return false;
  for(int i=0;i<3;i++) out[i]=cal_accel.scale.m[i][0]*(raw[0]-cal_accel.bias.x)+cal_accel.scale.m[i][1]*(raw[1]-cal_accel.bias.y)+cal_accel.scale.m[i][2]*(raw[2]-cal_accel.bias.z);
  return true;}
bool applyGyroCal(const float raw[3],float out[3]){
  if(!cal_gyro.valid)return false;
  out[0]=raw[0]-cal_gyro.bias_dps.x; out[1]=raw[1]-cal_gyro.bias_dps.y; out[2]=raw[2]-cal_gyro.bias_dps.z;
  return true;}
bool applyMagCal(const float raw[3],float out[3]){
  if(!cal_mag.valid)return false;
  float c[3]={raw[0]-cal_mag.hard_iron.x,raw[1]-cal_mag.hard_iron.y,raw[2]-cal_mag.hard_iron.z};
  for(int i=0;i<3;i++) out[i]=cal_mag.soft_iron.m[i][0]*c[0]+cal_mag.soft_iron.m[i][1]*c[1]+cal_mag.soft_iron.m[i][2]*c[2];
  return true;}

bool loadCalDefaults(){
  cal_gyro.bias_dps={0,0,0}; cal_gyro.variance_dps2[0]=0.01f;cal_gyro.variance_dps2[1]=0.01f;cal_gyro.variance_dps2[2]=0.01f; cal_gyro.valid=false;
  cal_accel.bias={0,0,0}; cal_accel.scale={{{1,0,0},{0,1,0},{0,0,1}}}; cal_accel.rms_mps2=0.05f; cal_accel.valid=false;
  cal_mag.hard_iron={0,0,0}; cal_mag.soft_iron={{{1,0,0},{0,1,0},{0,0,1}}}; cal_mag.radius_mean_uT=40.0f; cal_mag.valid=false;
  // A sea-level default is not a measured ground reference. Keep altitude hold
  // unavailable until a real, stationary ground-pressure calibration is loaded.
  cal_baro.ground_pressure_pa=101325.0f; cal_baro.reference_variance_pa2=0; cal_baro.measurement_variance_pa2=10.0f; cal_baro.valid=false;
  calibration_loaded=false; return true;}

// ============================================================================
// MPU6050 FIFO DRIVER
// ============================================================================

bool initMpu(){
  i2c_mgr.setClock(cfg::I2C_HZ);
  mpu_address=0;
  if(i2c_mgr.isPresent(cfg::MPU_PRIMARY_ADDRESS))mpu_address=cfg::MPU_PRIMARY_ADDRESS;
  else if(i2c_mgr.isPresent(cfg::MPU_ALTERNATE_ADDRESS))mpu_address=cfg::MPU_ALTERNATE_ADDRESS;
  if(!mpu_address)return false;
  uint8_t id=0; if(!i2c_mgr.readByte(mpu_address,cfg::MPU_REG_WHO_AM_I,id)||id!=0x68)return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_PWR_MGMT_1,cfg::MPU_PWR_MGMT_1_RESET))return false; delay(100);
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_PWR_MGMT_1,cfg::MPU_PWR_MGMT_1_CLKSEL))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_PWR_MGMT_2,0x00))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_CONFIG,cfg::MPU_DLPF_188HZ))return false;
  if(!i2c_mgr.writeReg(mpu_address,0x19,0x01))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_GYRO_CONFIG,cfg::MPU_GYRO_FS_500))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_ACCEL_CONFIG,cfg::MPU_ACCEL_FS_8G))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_FIFO_EN,cfg::MPU_FIFO_TEMP_EN|cfg::MPU_FIFO_XG_EN|cfg::MPU_FIFO_YG_EN|cfg::MPU_FIFO_ZG_EN|cfg::MPU_FIFO_ACCEL_EN))return false;
  uint8_t ic=0; if(!i2c_mgr.readByte(mpu_address,cfg::MPU_REG_INT_PIN_CFG,ic))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_INT_PIN_CFG,ic|cfg::MPU_INT_PIN_CFG_BYPASS_EN))return false;
  uint8_t uc=0; if(!i2c_mgr.readByte(mpu_address,cfg::MPU_REG_USER_CTRL,uc))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_USER_CTRL,uc|cfg::MPU_USER_CTRL_FIFO_EN))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_USER_CTRL,uc|cfg::MPU_USER_CTRL_FIFO_EN|cfg::MPU_USER_CTRL_FIFO_RESET))return false;
  if(!i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_INT_ENABLE,cfg::MPU_INT_ENABLE_RAW_RDY_EN))return false;
  mpu_present=true; return true;}

bool readMpuFifo(ImuSample&s){
  s.fresh=false;
  i2c_mgr.setClock(cfg::I2C_HZ);
  if(!mpu_present)return false;
  uint8_t fc[2]={};
  if(!i2c_mgr.readRegs(mpu_address,cfg::MPU_REG_FIFO_COUNT_H,fc,2)){health.imu_errors++;return false;}
  const uint16_t fcnt=(uint16_t(fc[0])<<8)|fc[1];
  const uint16_t complete_records=fcnt/14u;
  s.fifo_count=complete_records;
  const uint32_t now_us=micros();

  if(fcnt>=1024u){
    ++s.overflows;
    ++health.imu_errors;
    (void)i2c_mgr.writeReg(mpu_address,cfg::MPU_REG_USER_CTRL,
        cfg::MPU_USER_CTRL_FIFO_EN|cfg::MPU_USER_CTRL_FIFO_RESET);
    s.valid=false;
    return false;
  }

  // Reuse the last coherent FIFO frame when no complete record is available.
  // Direct-register reads here used to be labelled fresh even when repeated.
  if(complete_records==0u){
    if(s.timestamp_us==0u
        || uint32_t(now_us-s.timestamp_us)>cfg::IMU_MAX_AGE_US)
      s.valid=false;
    return s.valid;
  }

  uint8_t d[14]={};
  if(!i2c_mgr.readRegs(mpu_address,cfg::MPU_REG_FIFO_R_W,d,sizeof(d))){
    health.imu_errors++;
    return false;
  }
  for(int i=0;i<3;i++){
    s.accel_raw[i]=(int16_t)((uint16_t(d[2*i])<<8)|d[2*i+1]);
    s.gyro_raw[i]=(int16_t)((uint16_t(d[8+2*i])<<8)|d[9+2*i]);
  }
  s.temp_raw=(int16_t)((uint16_t(d[6])<<8)|d[7]);

  // FIFO reads pop the oldest complete record. Estimate its age from the
  // queued record count; this uses the configured 500 Hz nominal period.
  const uint32_t backlog_age_us=(complete_records-1u)*cfg::IMU_PERIOD_US;
  s.timestamp_us=now_us-backlog_age_us;
  s.valid=true;
  s.fresh=true;
  return true;
}

// ============================================================================
// QMC5883P DRIVER
// ============================================================================

bool initMag(){
  i2c_mgr.setClock(cfg::QMC_I2C_HZ);
  mag_address=cfg::MAG_ADDRESS; if(!i2c_mgr.isPresent(mag_address))return false;
  uint8_t id=0; if(!i2c_mgr.readQmcByte(cfg::QMC_REG_CHIP_ID,id)||id!=cfg::QMC_CHIP_ID)return false;
  if(!i2c_mgr.writeReg(mag_address,cfg::QMC_REG_CTRL1,0x00))return false; delay(20);
  if(!i2c_mgr.writeReg(mag_address,cfg::QMC_REG_XYZ_SIGN,cfg::QMC_XYZ_SIGN_VAL))return false;
  if(!i2c_mgr.writeReg(mag_address,cfg::QMC_REG_CTRL2,cfg::QMC_CTRL2_8GAUSS))return false;
  if(!i2c_mgr.writeReg(mag_address,cfg::QMC_REG_CTRL1,cfg::QMC_CTRL1_200HZ))return false;
  mag_configured=true;
  i2c_mgr.setClock(cfg::I2C_HZ);
  return true;}

bool readMag(MagSample&s){
  s.fresh=false;
  const uint32_t now_us=micros();
  if(s.timestamp_us==0 || uint32_t(now_us-s.timestamp_us)>=cfg::MAG_STALE_MS*1000U)
    s.valid=false;
  if(!mag_configured)return false;
  i2c_mgr.setClock(cfg::QMC_I2C_HZ);
  uint8_t st=0; if(!i2c_mgr.readQmcByte(cfg::QMC_REG_STATUS,st)){health.mag_errors++;return false;}
  s.status=st; if(st&0x02){health.mag_errors++;return false;} if(!(st&0x01))return false;
  uint8_t d[6]={}; if(!i2c_mgr.readQmcRegs(cfg::QMC_REG_DATA_XL,d,6)){health.mag_errors++;return false;}
  for(int i=0;i<3;i++)s.raw[i]=(int16_t)((uint16_t(d[2*i+1])<<8)|d[2*i]);
  if((s.raw[0]==0&&s.raw[1]==0&&s.raw[2]==0)||(s.raw[0]==-1&&s.raw[1]==-1&&s.raw[2]==-1)||s.raw[0]==INT16_MIN||s.raw[1]==INT16_MIN||s.raw[2]==INT16_MIN){health.mag_errors++;return false;}
  s.timestamp_us=esp_timer_get_time(); s.valid=true; s.fresh=true; return true;}

// ============================================================================
// BMP280 DRIVER
// ============================================================================

bool initBmp(){
  i2c_mgr.setClock(cfg::I2C_HZ);
  bmp_address=0;
  if(i2c_mgr.isPresent(cfg::BMP_PRIMARY_ADDRESS))bmp_address=cfg::BMP_PRIMARY_ADDRESS;
  else if(i2c_mgr.isPresent(cfg::BMP_ALTERNATE_ADDRESS))bmp_address=cfg::BMP_ALTERNATE_ADDRESS;
  if(!bmp_address)return false;
  uint8_t id=0; if(!i2c_mgr.readByte(bmp_address,cfg::BMP_REG_ID,id)||id!=cfg::BMP_CHIP_ID)return false;
  bmp_present=true; return true;}

bool readBmp(BaroSample&s){
  s.fresh=false;
  i2c_mgr.setClock(cfg::I2C_HZ);
  if(!bmp_present)return false;
  const uint32_t now_us=micros();
  const uint32_t age_us=s.timestamp_us==0u?UINT32_MAX:uint32_t(now_us-s.timestamp_us);
  if(age_us>cfg::BMP_MAX_AGE_US)s.valid=false;
  if(s.timestamp_us!=0u&&age_us<cfg::BMP_PERIOD_US)return s.valid;
  uint8_t status=0;
  if(!i2c_mgr.readByte(bmp_address,cfg::BMP_REG_STATUS,status)){
    health.baro_errors++;
    return s.valid;
  }
  if((status&0x08u)!=0u)return s.valid; // measuring; reuse cache, never wait
  uint8_t r[6]={}; if(!i2c_mgr.readRegs(bmp_address,cfg::BMP_REG_PRESSURE_MSB,r,6)){health.baro_errors++;return false;}
  s.raw_pressure=((uint32_t(r[0])<<12)|((uint32_t(r[1])<<4)|(r[2]>>4)));
  s.raw_temperature=((uint32_t(r[3])<<12)|((uint32_t(r[4])<<4)|(r[5]>>4)));
  s.timestamp_us=micros(); s.valid=true; s.fresh=true; return true;}

bool compensateBmp(const BaroSample&raw,float&tc,float&pp){
  (void)raw;
  if(!bmp_driver)return false;
  tc=bmp_driver->readTemperature(); pp=bmp_driver->readPressure();
  return isfinite(tc)&&isfinite(pp)&&tc>-40&&tc<85&&pp>30000&&pp<110000;}

// ============================================================================
// LiDAR DRIVER
// ============================================================================

bool readLidar(LidarSample&s){
  if(!lidar_present)return false;
  static HardwareSerial ls(1); static uint8_t buf[9]; static size_t bi=0;
  while(ls.available()){
    uint8_t b=ls.read();
    if(bi==0&&b!=0x59)continue; if(bi==1&&b!=0x59){bi=0;continue;}
    buf[bi++]=b;
    if(bi==9){ uint8_t sum=0; for(int i=0;i<8;i++)sum+=buf[i];
      if(sum!=buf[8]){bi=0;health.lidar_errors++;continue;}
      uint16_t dc=(uint16_t(buf[1])<<8)|buf[0]; uint16_t sv=(uint16_t(buf[3])<<8)|buf[2];
      s.distance_m=dc/100.0f; s.quality=sv/65535.0f; s.timestamp_us=esp_timer_get_time();
      s.valid=true; s.fresh=true; bi=0; return true;}}
  return false;}

// ============================================================================
// CONTROL
// ============================================================================

void computeControl(const PilotCommand&cmd,const AttitudeEstimator&att,
                    const VerticalKf&vkf,float dt,MotorOutput&out){
  const bool imu_ok=calibration_loaded&&cal_accel.valid&&cal_gyro.valid
      &&imu_sample.valid
      &&timestampWithinUs(imu_sample.timestamp_us,cfg::IMU_MAX_AGE_US)
      &&att.initialized;
  const bool altitude_ok=cal_baro.valid&&baro_sample.valid
      &&timestampWithinUs(baro_sample.timestamp_us,cfg::BMP_MAX_AGE_US)
      &&vkf.initialized;
  const bool pilot_ok=cmd.valid
      &&uint32_t(millis()-cmd.timestamp_ms)<cfg::CONTROLLER_TIMEOUT_MS;
  if(!imu_ok||!altitude_ok||!pilot_ok||!pidGainsConfigured()
      ||!control.hover_collective_valid||!isfinite(dt)||dt<=0.0f||dt>0.01f){
    const bool newly_failed=!control.failsafe;
    if(control.armed){
      control.armed=false;
      control.disarm_time_ms=millis();
      health.disarms++;
    }
    control.failsafe=true;
    if(newly_failed)health.failsafes++;
    motor_mixer.reset();
    out.armed=false;
    out.sensor_fault=!imu_ok||!altitude_ok;
    out.failsafe=true;
    for(int i=0;i<4;i++)out.motor[i]=0.0f;
    return;
  }
  control.failsafe=false;
  out.armed=control.armed;
  out.sensor_fault=false;
  out.failsafe=false;
  if(!out.armed){for(int i=0;i<4;i++)out.motor[i]=0.0f;return;}
  float rs=cmd.roll*cfg::MAX_ROLL_DEG, ps=cmd.pitch*cfg::MAX_PITCH_DEG;
  float rrs=pid_roll_angle.update(rs,att.roll_deg,dt);
  float prs=pid_pitch_angle.update(ps,att.pitch_deg,dt);
  float rtq=pid_roll_rate.update(rrs,att.roll_rate_dps,dt);
  float ptq=pid_pitch_rate.update(prs,att.pitch_rate_dps,dt);
  float ytq=pid_yaw_rate.update(cmd.yaw*cfg::MAX_YAW_RATE_DPS,att.yaw_rate_dps,dt);
  float vs=cmd.throttle*cfg::FC_MAX_VERTICAL_SPEED_MPS;
  const float boost=clampf(control.boost_state,0.0f,1.0f);
  float tt=(boost-control.lt_reduce_state)*1.0f;
  vs+=tt; vs=fmaxf(-cfg::FC_MAX_DESCENT_MPS,fminf(cfg::FC_MAX_CLIMB_MPS,vs));
  float ac=pid_vertical_speed.update(vs,vkf.velocity_mps,dt);
  bool ah=fabsf(cmd.throttle)<0.1f;
  if(ah&&control.altitude_ref_initialized){float ac2=pid_altitude.update(control.altitude_ref_m,vkf.height_m,dt); ac+=ac2;}
  else{if(!control.altitude_ref_initialized){control.altitude_ref_m=vkf.height_m;control.altitude_ref_initialized=true;} control.altitude_ref_m+=vs*dt;}
  float cr=cosf(att.roll_deg*0.0174532925f), cp=cosf(att.pitch_deg*0.0174532925f);
  float tf=fmaxf(0.5f,cr*cp);
  float coll=(control.hover_collective+ac*0.05f)/tf;
  coll=fmaxf(cfg::FC_ARMED_IDLE_FRACTION,fminf(cfg::FC_BOOST_MOTOR_CEILING,coll));
  float ceil=cfg::FC_NORMAL_MOTOR_CEILING+(cfg::FC_BOOST_MOTOR_CEILING-cfg::FC_NORMAL_MOTOR_CEILING)*boost;
  motor_mixer.mix(coll,rtq,ptq,ytq,ceil,cfg::FC_ARMED_IDLE_FRACTION,dt,out.motor);
}

// ============================================================================
// HARDWARE INIT
// ============================================================================

void initHardware(){
  for(uint8_t p:cfg::MOTOR_PINS)pinMode(p,INPUT);
  i2c_mgr.begin(); loadCalDefaults();
  mpu_present=initMpu();
  if(mpu_present)delay(30);
  // Initialize the BMP independently; an MPU discovery failure must not hide
  // a working pressure sensor from telemetry or diagnostics.
  bmp_present=initBmp();
  if(bmp_present){
    bmp_driver=new Adafruit_BMP280(&Wire);
    if(!bmp_driver->begin(bmp_address)){
      delete bmp_driver; bmp_driver=nullptr; bmp_present=false;
    }else{
      bmp_driver->setSampling(Adafruit_BMP280::MODE_NORMAL,
          Adafruit_BMP280::SAMPLING_X1,Adafruit_BMP280::SAMPLING_X1,
          Adafruit_BMP280::FILTER_X2,Adafruit_BMP280::STANDBY_MS_1);
    }
  }
  mag_configured=initMag();
  // Competition build excludes LiDAR until its wiring/protocol are confirmed.
  lidar_present=false;
  health.last_reset_reason=esp_reset_reason();}

} // namespace iq

// ============================================================================
// Arduino entry points
// ============================================================================

using namespace iq;

void setup(){
  Serial.begin(cfg::SERIAL_BAUD);
  while(!Serial&&millis()<2000){}
  initHardware();
  health.uptime_us=esp_timer_get_time();
  // Keep this UART binary-only: the calibration GUI searches for 52-byte CRC
  // frames and arbitrary text can corrupt a frame already in its byte stream.
}

void loop(){
  static int64_t nd=0;
  static uint32_t seq=0,previous_loop_us=0,previous_imu_us=0;
  TelemetryPacket pkt;
  int64_t now=esp_timer_get_time();
  if(nd==0)nd=now+cfg::CONTROL_PERIOD_US;
  int64_t ud=nd-now;
  if(ud>300){delayMicroseconds((uint32_t)(ud-200));return;}
  if(ud>0)return;
  int64_t sa=esp_timer_get_time(); int64_t late=sa-nd;
  if(late>=(int64_t)cfg::CONTROL_PERIOD_US){uint64_t sk=(uint64_t)late/cfg::CONTROL_PERIOD_US;
    health.missed_deadlines+=(uint32_t)(sk>UINT32_MAX?UINT32_MAX:sk); nd+=(int64_t)sk*cfg::CONTROL_PERIOD_US;}
  const uint32_t loop_start_us=(uint32_t)sa;
  const float dt=previous_loop_us==0u
      ? cfg::CONTROL_PERIOD_US/1000000.0f
      : uint32_t(loop_start_us-previous_loop_us)/1000000.0f;
  previous_loop_us=loop_start_us;
  nd+=cfg::CONTROL_PERIOD_US;
  health.loop_count++; health.uptime_us=sa;

  bool imu_ok=readMpuFifo(imu_sample);
  bool mag_ok=readMag(mag_sample);
  bool baro_ok=readBmp(baro_sample);
  // This competition phase intentionally excludes LiDAR: it is not wired yet.
  // Only process each FIFO frame once; cached raw values remain available for
  // telemetry but must never be integrated as a second measurement.
  if(mag_ok&&mag_sample.fresh){
    float mru[3]={mag_sample.raw[0]/cfg::QMC_COUNTS_PER_UT,
                  mag_sample.raw[1]/cfg::QMC_COUNTS_PER_UT,
                  mag_sample.raw[2]/cfg::QMC_COUNTS_PER_UT};
    mag_sample.calibrated[0]=mag_sample.calibrated[1]=mag_sample.calibrated[2]=0.0f;
    if(cal_mag.valid)(void)applyMagCal(mru,mag_sample.calibrated);
  }
  if(imu_ok&&imu_sample.fresh){
    float ar[3]={imu_sample.accel_raw[0]/4096.0f*cfg::GRAVITY_MPS2,
                 imu_sample.accel_raw[1]/4096.0f*cfg::GRAVITY_MPS2,
                 imu_sample.accel_raw[2]/4096.0f*cfg::GRAVITY_MPS2};
    float gr[3]={imu_sample.gyro_raw[0]/65.5f,
                 imu_sample.gyro_raw[1]/65.5f,
                 imu_sample.gyro_raw[2]/65.5f};
    imu_sample.temperature_c=imu_sample.temp_raw/340.0f+36.53f;
    const bool accel_cal_ok=cal_accel.valid&&applyAccelCal(ar,imu_sample.accel);
    const bool gyro_cal_ok=cal_gyro.valid&&applyGyroCal(gr,imu_sample.gyro);
    if(accel_cal_ok&&gyro_cal_ok){
      const float imu_dt=previous_imu_us==0u
          ? cfg::IMU_PERIOD_US/1000000.0f
          : uint32_t(imu_sample.timestamp_us-previous_imu_us)/1000000.0f;
      previous_imu_us=imu_sample.timestamp_us;
      if(!attitude.initialized)attitude.init(imu_sample.accel);
      const bool mag_usable=cal_mag.valid&&mag_sample.valid
          &&timestampWithinUs(mag_sample.timestamp_us,cfg::MAG_STALE_MS*1000u);
      attitude.update(imu_sample.gyro,imu_sample.accel,
          mag_usable?mag_sample.calibrated:nullptr,mag_usable,imu_dt);
      if(vert_kf.initialized){
        const float world_specific_force=attitude.worldVerticalSpecificForce(imu_sample.accel);
        imu_sample.vertical_acceleration_world_mps2=world_specific_force-cfg::GRAVITY_MPS2;
        vert_kf.predict(imu_sample.vertical_acceleration_world_mps2,imu_dt);
      }
    }else{
      attitude.initialized=false;
      vert_kf.initialized=false;
    }
  }
  // Polling is non-blocking. Reuse a pressure observation only when the BMP
  // reports a completed conversion; do not compensate or timestamp a cached
  // sample again on every 500 Hz loop.
  if(baro_ok&&baro_sample.fresh){
    float tc=0.0f,pp=0.0f;
    if(compensateBmp(baro_sample,tc,pp)){
      baro_sample.temperature_c=tc;
      baro_sample.pressure_pa=pp;
      baro_sample.timestamp_us=micros();
      if(cal_baro.valid&&isfinite(cal_baro.ground_pressure_pa)
          &&cal_baro.ground_pressure_pa>30000.0f){
        const float ratio=pp/cal_baro.ground_pressure_pa;
        if(ratio>0.5f&&ratio<1.5f){
          baro_sample.height_m=44330.0f*(1.0f-powf(ratio,0.190294957f));
          const float pressure_variance=fmaxf(0.0f,cal_baro.reference_variance_pa2)
              +fmaxf(0.0f,cal_baro.measurement_variance_pa2);
          const float dh_dp=44330.0f*0.190294957f
              *powf(ratio,0.190294957f-1.0f)/cal_baro.ground_pressure_pa;
          const float calculated_sigma=fabsf(dh_dp)*sqrtf(pressure_variance);
          baro_sample.height_sigma_m=isfinite(calculated_sigma)
              ? fmaxf(0.15f,calculated_sigma):1.0f;
          if(!vert_kf.initialized)vert_kf.init(baro_sample.height_m,0.0f);
          else vert_kf.updateBaro(baro_sample.height_m,baro_sample.height_sigma_m);
        }else{
          baro_sample.valid=false;
        }
      }
    }else{
      baro_sample.valid=false;
    }
  }

  static uint8_t ub[22]; static size_t bi=0;
  if(cfg::COMMAND_INPUT_ENABLED){
    while(Serial.available()){
      const uint8_t b=(uint8_t)Serial.read();
      if(bi==0u){if(b==cfg::COMMAND_MAGIC_0)ub[bi++]=b;continue;}
      if(bi==1u){
        if(b==cfg::COMMAND_MAGIC_1)ub[bi++]=b;
        else if(b==cfg::COMMAND_MAGIC_0)ub[0]=b;
        else bi=0u;
        continue;
      }
      ub[bi++]=b;
      if(bi==sizeof(ub)){
        if(xbox_interface.parseCmd(ub,sizeof(ub),pilot))bi=0u;
        else{
          size_t next=1u;
          while(next+1u<sizeof(ub)
              &&!(ub[next]==cfg::COMMAND_MAGIC_0&&ub[next+1u]==cfg::COMMAND_MAGIC_1))
            ++next;
          if(next+1u<sizeof(ub)){
            const size_t remain=sizeof(ub)-next;
            memmove(ub,ub+next,remain);bi=remain;
          }else if(ub[sizeof(ub)-1u]==cfg::COMMAND_MAGIC_0){
            ub[0]=cfg::COMMAND_MAGIC_0;bi=1u;
          }else bi=0u;
        }
      }
    }
  }
  bool arm_changed=false; xbox_interface.updateArm(pilot,control.armed,arm_changed);
  if(arm_changed){if(control.armed){
    pid_roll_angle.reset();pid_pitch_angle.reset();pid_roll_rate.reset();pid_pitch_rate.reset();
    pid_yaw_rate.reset();pid_altitude.reset();pid_vertical_speed.reset();motor_mixer.reset();
    // Keep the live sensor estimators: computeControl requires them to remain
    // initialized on the same control tick that accepts the arm transition.
    control.altitude_ref_initialized=false;
    control.lt_reduce_state=0;control.boost_state=0;}}
  float lt_t=pilot.lt; control.lt_reduce_state+=(lt_t-control.lt_reduce_state)*0.2f; control.lt_reduce_state=clampf(control.lt_reduce_state,0,1);
  control.boost_state+=(pilot.rt-control.boost_state)*0.2f; control.boost_state=clampf(control.boost_state,0,1);
  MotorOutput mo; computeControl(pilot,attitude,vert_kf,dt,mo);
  uint16_t lu=(uint16_t)(late>65535?65535:(late>0?late:0));
  buildTelemetry(pkt,imu_sample,mag_sample,baro_sample,lidar_sample,attitude,vert_kf,mo,seq++,(uint32_t)sa,lu,i2c_mgr.stats);
  if(Serial.availableForWrite()>=(int)cfg::PACKET_SIZE)Serial.write(pkt.bytes,cfg::PACKET_SIZE);
  else health.serial_drops++;
}
