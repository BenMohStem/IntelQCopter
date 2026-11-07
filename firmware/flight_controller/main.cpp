/*
 * IntelQCopter — ESP32-S3 Competition Flight Controller
 * ======================================================
 *
 * Target:  ESP32-S3 DevKitC-1 N8, 8 MB flash, no PSRAM
 * Framework: Arduino / PlatformIO
 * Control rate: 500 Hz deterministic (2 ms cycle)
 * UART: 921600 baud, 8N1, 52-byte CRC-protected v3 binary packets
 * I²C: 400 kHz shared bus for MPU6050, BMP280, QMC5883P
 * WiFi: Access Point "IntelQCopter_FC", WebSocket on port 81 (PID tuning)
 * Motor outputs: DISABLED (pins 12/11/14/13 = INPUT ONLY)
 *
 * Architecture:
 *   500 Hz control loop (IMU, MAG, Baro, estimation, PID, mixing)
 *   Wireless PID tuning via WebSocket web server
 *   No auto-arm on startup/reconnect
 *   Physical motor arming remains disabled
 *
 * Calibration (Nov 7 completed-session fit, software candidate only):
 *   MPU6050: ±8 g / ±500 deg/s, software bias + matrix applied once
 *   QMC5883P: ±8 G = 37.5 counts/µT, hard/soft-iron applied once
 *   BMP280: no ground pressure reference; altitude control unavailable
 *   LiDAR: excluded (not wired)
 *   Magnetometer heading fusion: DISABLED until axes/handedness verified
 *
 * PID values: carried-forward simulation values only.
 *   Rate loops output N·m; not flight-identified. Not wired to the live
 *   mixer until units and torque-to-command conversion are reconciled.
 *
 * Motor policy (software command fractions, not RPM/thrust):
 *   Armed idle: 10% | Normal ceiling: 35% | Boost ceiling: 50%
 *   Slew-rate cap: 8.0 per second
 *
 * WiFi: SSID "IntelQCopter_FC", password "IntelQCopter_FC"
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

/* ESP-IDF WiFi and networking (direct, avoids Arduino WiFi lib conflicts) */
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ============================================================================
   CALIBRATION CONSTANTS (Nov 7 completed-session fit)
   ============================================================================
   Source: hardware/experimental_esp32s3_fc_nov07_calibrated_candidate/
   All constants are research-grade software candidates, not flight-validated.
   ============================================================================ */

namespace cal {

/* ---- Accelerometer (m/s², capture sensor chip frame) ---- */
/* Raw conversion: counts / 4096 * 9.80665 */
/* Apply once: calibrated = C * (raw_mps2 - bias) */
inline const float accel_bias_mps2[3] = {
    0.25928109f, -0.08780301f, -0.89685228f
};
inline const float accel_correction[3][3] = {
    { 0.99463969f, -0.04280700f,  0.04798329f},
    { 0.04255186f,  0.99493002f,  0.02387155f},
    {-0.05126050f, -0.03758379f,  0.98640514f}
};
inline const float accel_fit_residual_rms_mps2 = 0.06018733f;

/* ---- Gyroscope (deg/s) ---- */
/* Raw conversion: counts / 65.5 */
/* Apply once: calibrated = raw - bias */
inline const float gyro_bias_dps[3] = {
    -11.04461742f, 6.99774234f, -1.10236200f
};

/* ---- QMC5883P magnetometer (chip-frame microtesla) ---- */
/* Raw conversion: counts / 37.5 = µT (±8 G) */
/* Apply once: calibrated = S * (raw_µT - hard_iron) */
inline const float mag_counts_per_ut = 37.5f;
inline const float mag_hard_iron_ut[3] = {
    -7.34180573f, -13.99272901f, -22.84747418f
};
inline const float mag_soft_iron[3][3] = {
    { 0.97174713f, -0.01740207f,  0.03634599f},
    {-0.01740207f,  0.96045521f, -0.02183163f},
    { 0.03634599f, -0.02183163f,  1.07361879f}
};
inline const float mag_field_radius_ut = 40.8129f;

/* ---- Barometer (Pa) ---- */
/* No ground pressure reference recorded. Altitude control remains unavailable. */
inline const float baro_ground_pressure_pa = 0.0f;
inline const float baro_reference_variance_pa2 = 0.0f;
inline const float baro_measurement_variance_pa2 = 10.81f;
inline const float baro_height_variance_m2 = 0.08f;

/* ---- Gravity ---- */
inline const float gravity_mps2 = 9.80665f;

/* ---- PID gains (carried-forward simulation values, research-only) ---- */
/* Rate loops output N·m; not wired to live mixer until units reconciled. */
inline const float pid_roll_angle[3]   = { 0.85f,  0.012f, 0.008f  };
inline const float pid_pitch_angle[3]  = { 0.85f,  0.012f, 0.008f  };
inline const float pid_roll_rate[3]    = { 0.0723794083f, 0.0342335039f, 0.0000440145f };
inline const float pid_pitch_rate[3]   = { 0.0768854934f, 0.0363647604f, 0.0000467547f };
inline const float pid_yaw_rate[3]     = { 0.0709297215f, 0.0191701950f, 0.0000575106f };
inline const float pid_altitude[3]     = { 0.35f,  0.003f, 0.010f  };
inline const float pid_vert_speed[3]   = { 0.75f,  0.040f, 0.018f  };

/* ---- Motor policy (ESC command fractions) ---- */
inline const float motor_idle_fraction = 0.10f;
inline const float motor_normal_ceiling = 0.35f;
inline const float motor_boost_ceiling = 0.50f;

/* ---- Slew-rate cap (per second) ---- */
inline const float motor_max_slew_per_second = 8.0f;

/* ---- Control limits ---- */
inline const float max_roll_deg = 35.0f;
inline const float max_pitch_deg = 35.0f;
inline const float max_yaw_rate_dps = 180.0f;
inline const float max_vertical_speed_mps = 2.0f;
inline const float max_climb_mps = 1.5f;
inline const float max_descent_mps = 1.0f;

/* ---- Feature flags ---- */
inline const bool kEnableMagHeadingFusion = false;
inline const bool kEnableMotorAccelerationPid = false;
inline const bool kEnableLidar = false;
inline const bool kApplyCalibrationToRuntime = true;

/* ---- WiFi configuration ---- */
inline const char* wifi_ssid     = "IntelQCopter_FC";
inline const char* wifi_password = "IntelQCopter_FC";
inline const uint8_t wifi_channel = 1;
inline const uint8_t wifi_max_conn = 4;

/* ---- Network ports ---- */
inline const uint16_t http_port = 80;
inline const uint16_t ws_port   = 81;

/* ---- Telemetry ---- */
inline const uint32_t telemetry_interval_ms = 50;

/* ---- UART telemetry packet ---- */
inline const uint8_t packet_size = 52;
inline const uint8_t magic0 = 0x51;
inline const uint8_t magic1 = 0x43;
inline const uint8_t version = 1;

/* ---- Xbox buttons ---- */
inline const uint8_t btn_lb   = 1u << 0;
inline const uint8_t btn_rb   = 1u << 1;
inline const uint8_t btn_north= 1u << 2;
inline const uint8_t btn_east = 1u << 3;
inline const uint8_t btn_south= 1u << 4;
inline const uint8_t btn_west = 1u << 5;
inline const uint8_t arm_buttons = btn_lb | btn_rb;

} // namespace cal

/* ============================================================================
   SENSOR ADDRESSES / REGISTERS
   ============================================================================ */

static constexpr uint8_t kMpuPrimaryAddr   = 0x68;
static constexpr uint8_t kMpuAlternateAddr = 0x69;
static constexpr uint8_t kBmpPrimaryAddr   = 0x76;
static constexpr uint8_t kBmpAlternateAddr = 0x77;
static constexpr uint8_t kMagAddress       = 0x2C;

static constexpr uint8_t kMpuRegPwrMgmt1    = 0x6B;
static constexpr uint8_t kMpuRegPwrMgmt2    = 0x6C;
static constexpr uint8_t kMpuRegConfig      = 0x1A;
static constexpr uint8_t kMpuRegGyroConfig  = 0x1B;
static constexpr uint8_t kMpuRegAccelConfig = 0x1C;
static constexpr uint8_t kMpuRegFifoEn      = 0x23;
static constexpr uint8_t kMpuRegIntPinCfg   = 0x37;
static constexpr uint8_t kMpuRegIntEnable   = 0x38;
static constexpr uint8_t kMpuRegUserCtrl    = 0x6A;
static constexpr uint8_t kMpuRegFifoCountH  = 0x72;
static constexpr uint8_t kMpuRegFifoRW      = 0x74;
static constexpr uint8_t kMpuRegWhoAmI      = 0x75;
static constexpr uint8_t kMpuRegXgOffsUsrh  = 0x13;

static constexpr uint8_t kMpuPwrMgmt1Reset      = 0x80;
static constexpr uint8_t kMpuPwrMgmt1Clksel     = 0x01;
static constexpr uint8_t kMpuDlpf188Hz            = 0x01;
static constexpr uint8_t kMpuGyroFs500            = 0x08;
static constexpr uint8_t kMpuAccelFs8G            = 0x10;
static constexpr uint8_t kMpuFifoTempEn          = 0x80;
static constexpr uint8_t kMpuFifoXgEn            = 0x40;
static constexpr uint8_t kMpuFifoYgEn            = 0x20;
static constexpr uint8_t kMpuFifoZgEn            = 0x10;
static constexpr uint8_t kMpuFifoAccelEn         = 0x08;
static constexpr uint8_t kMpuUserCtrlFifoEn     = 0x40;
static constexpr uint8_t kMpuUserCtrlFifoReset  = 0x04;
static constexpr uint8_t kMpuIntPinCfgBypassEn = 0x02;
static constexpr uint8_t kMpuIntEnableRawRdyEn = 0x01;

static constexpr uint8_t kQmcRegChipId   = 0x00;
static constexpr uint8_t kQmcRegCtrl1    = 0x0A;
static constexpr uint8_t kQmcRegCtrl2    = 0x0B;
static constexpr uint8_t kQmcRegSetPeriod= 0x0C;
static constexpr uint8_t kQmcRegXyzSign  = 0x29;
static constexpr uint8_t kQmcRegDataXl   = 0x01;
static constexpr uint8_t kQmcRegStatus   = 0x09;
static constexpr uint8_t kQmcChipId      = 0x80;
static constexpr uint8_t kQmcCtrl1_200Hz   = 0x3D;
static constexpr uint8_t kQmcCtrl2_8Gauss  = 0x08;
static constexpr uint8_t kQmcXyzSignVal  = 0x06;

static constexpr uint8_t kBmpRegId           = 0xD0;
static constexpr uint8_t kBmpRegStatus       = 0xF3;
static constexpr uint8_t kBmpRegReset        = 0xE0;
static constexpr uint8_t kBmpRegCtrlMeas     = 0xF4;
static constexpr uint8_t kBmpRegConfig       = 0xF5;
static constexpr uint8_t kBmpRegPressureMsb  = 0xF7;
static constexpr uint8_t kBmpChipId          = 0x58;

static constexpr uint8_t kMotorPins[4] = {12, 11, 14, 13};

/* ============================================================================
   GLOBAL STATE
   ============================================================================ */

static int httpSock = -1;
static int wsSock   = -1;
static int wsClient = -1;

static uint32_t lastTelemetryMs = 0;
static char ipString[16] = "0.0.0.0";

/* Calibration validity */
static bool calAccelValid = false;
static bool calGyroValid  = false;
static bool calMagValid   = false;
static bool calBaroValid  = false;
static bool calibrationLoaded = false;

/* Sensor presence */
static uint8_t mpuAddress = 0, bmpAddress = 0, magAddress = kMagAddress;
static bool mpuPresent = false, bmpPresent = false, magConfigured = false;

/* Sensor samples */
struct ImuSample {
  bool valid = false, fresh = false; uint32_t timestamp_us = 0;
  int16_t accel_raw[3] = {}, gyro_raw[3] = {}, temp_raw = 0;
  float accel[3] = {}, gyro[3] = {}, temperature_c = 0.0f;
  float vertical_accel_world_mps2 = 0.0f;
  uint32_t fifo_count = 0, overflows = 0;
};
struct MagSample {
  bool valid = false, fresh = false; uint32_t timestamp_us = 0;
  int16_t raw[3] = {}; float calibrated[3] = {}; uint8_t status = 0;
};
struct BaroSample {
  bool valid = false, fresh = false; uint32_t timestamp_us = 0;
  uint32_t raw_pressure = 0, raw_temperature = 0;
  float temperature_c = 0.0f, pressure_pa = 0.0f;
  float height_m = 0.0f, height_sigma_m = 0.0f;
};
struct LidarSample {
  bool valid = false, fresh = false; uint32_t timestamp_us = 0;
  float distance_m = 0.0f, quality = 0.0f;
};

ImuSample imu;
MagSample mag;
BaroSample baro;
LidarSample lidar;

/* Control state */
static bool armed = false, armRequested = false, disarmed = true, failsafe = false;
static bool controllerConnected = false;
static uint32_t lastControllerMs = 0, armTimeMs = 0, disarmTimeMs = 0;
static float hoverCollective = 0.0f;
static bool hoverCollectiveValid = false;
static float ltReduceState = 0.0f, boostState = 0.0f;
static float rollAngleRef = 0.0f, pitchAngleRef = 0.0f, yawHeadingRef = 0.0f;
static float altitudeRefM = 0.0f;
static bool altitudeRefInitialized = false;

/* Motor outputs */
static float motorCmd[4] = {0,0,0,0};
static bool motorArmed = false, motorSensorFault = false, motorFailsafe = false;

/* Slew-rate state */
static float motorLastCmd[4] = {0,0,0,0};

/* ============================================================================
   PID CONTROLLERS (simulation-only values, not wired to live mixer yet)
   ============================================================================ */

struct Pid {
  float kp=0, ki=0, kd=0, integrator=0, prevMeas=0;
  float integratorLimit=0, outputLimit=0, derivFilt=0, derivativeCutoffHz=20.0f;
  bool initialized=false;

  void reset(){
    integrator=0; prevMeas=0; initialized=false; derivFilt=0;
  }
  float update(float sp, float meas, float dt){
    if(!isfinite(sp)||!isfinite(meas)||!isfinite(dt)||dt<=0.0f||dt>0.01f
       ||!isfinite(kp)||!isfinite(ki)||!isfinite(kd)
       ||!isfinite(integratorLimit)||integratorLimit<0.0f
       ||!isfinite(outputLimit)||outputLimit<=0.0f
       ||!isfinite(derivativeCutoffHz)||derivativeCutoffHz<=0.0f){
      reset(); return 0.0f;
    }
    float err = sp - meas;
    float der = 0;
    if(initialized){
      der = (meas - prevMeas) / dt;
      const float alpha = expf(-6.28318530718f * derivativeCutoffHz * dt);
      derivFilt = alpha*derivFilt + (1.0f-alpha)*der;
      der = derivFilt;
    }
    float ci = integrator + err*dt;
    ci = fmaxf(-integratorLimit, fminf(integratorLimit, ci));
    float raw = kp*err + ki*ci - kd*der;
    if(!isfinite(raw)||!isfinite(ci)){ reset(); return 0.0f; }
    float out = fmaxf(-outputLimit, fminf(outputLimit, raw));
    if(raw==out || (raw>out&&err<0) || (raw<out&&err>0)) integrator = ci;
    prevMeas = meas; initialized = true; return out;
  }
};

static Pid pidRollAngle, pidPitchAngle, pidRollRate, pidPitchRate;
static Pid pidYawRate, pidAltitude, pidVertSpeed;

/* ============================================================================
   QUATERNION ATTITUDE ESTIMATOR
   ============================================================================ */

struct Quaternion {
  float w=1, x=0, y=0, z=0;
  void normalize(){
    float n = sqrtf(w*w+x*x+y*y+z*z);
    if(n>0.001f){ w/=n; x/=n; y/=n; z/=n; }
    else { w=1; x=0; y=0; z=0; }
  }
  void toEuler(float& roll, float& pitch, float& yaw) const {
    float sr = 2.0f*(w*x + y*y), cr = 1.0f - 2.0f*(x*x + y*y);
    roll = atan2f(sr, cr) * 57.2957795f;
    float sp = 2.0f*(w*y - z*x);
    pitch = fabsf(sp) >= 1.0f ? copysignf(90.0f, sp) : asinf(sp)*57.2957795f;
    float sy = 2.0f*(w*z + x*y), cy = 1.0f - 2.0f*(y*y + z*z);
    yaw = atan2f(sy, cy) * 57.2957795f;
  }
};

static Quaternion attitudeQ;
static float attitudeGyroBias[3] = {0,0,0};
static float attitudeRollDeg = 0, attitudePitchDeg = 0, attitudeYawDeg = 0;
static float attitudeRollRateDps = 0, attitudePitchRateDps = 0, attitudeYawRateDps = 0;
static bool attitudeInitialized = false;
static uint32_t attitudeLastUpdateUs = 0;
static const float attitudeKpRoll = 0.5f, attitudeKpPitch = 0.5f, attitudeKpYaw = 0.1f;

static void attitudeInit(const float accel[3]) {
  float roll = atan2f(accel[1], accel[2]) * 57.2957795f;
  float pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 57.2957795f;
  float cr = cosf(roll*0.0174532925f), sr = sinf(roll*0.0174532925f);
  float cp = cosf(pitch*0.0174532925f), sp = sinf(pitch*0.0174532925f);
  attitudeQ.w = cr*cp + sr*sp*0;
  attitudeQ.x = sr*cp - cr*sp*0;
  attitudeQ.y = cr*sp + sr*cp*0;
  attitudeQ.z = cr*cp - sr*sp*0;
  attitudeQ.normalize();
  attitudeRollDeg = roll; attitudePitchDeg = pitch; attitudeYawDeg = 0;
  attitudeInitialized = true;
  attitudeLastUpdateUs = esp_timer_get_time();
}

static void attitudeUpdate(const float gyro[3], const float accel[3],
                            const float mag[3], bool magValid, float dt) {
  if(!attitudeInitialized || !isfinite(dt) || dt<=0.0f || dt>0.02f) return;
  float gx = gyro[0] - attitudeGyroBias[0];
  float gy = gyro[1] - attitudeGyroBias[1];
  float gz = gyro[2] - attitudeGyroBias[2];
  if(accel){
    float n = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if(n > 0.5f*cal::gravity_mps2 && n < 2.0f*cal::gravity_mps2){
      float ax = accel[0]/n, ay = accel[1]/n, az = accel[2]/n;
      float vx = 2.0f*(attitudeQ.x*attitudeQ.z - attitudeQ.w*attitudeQ.y);
      float vy = 2.0f*(attitudeQ.y*attitudeQ.z + attitudeQ.w*attitudeQ.x);
      float vz = attitudeQ.w*attitudeQ.w - attitudeQ.x*attitudeQ.x - attitudeQ.y*attitudeQ.y + attitudeQ.z*attitudeQ.z;
      float ex = ay*vz - az*vy;
      float ey = az*vx - ax*vz;
      float ez = ax*vy - ay*vx;
      const float rad2deg = 57.2957795f;
      gx += attitudeKpRoll * ex * rad2deg;
      gy += attitudeKpPitch * ey * rad2deg;
      gz += attitudeKpRoll * ez * rad2deg;
    }
  }
  if(magValid && mag){
    float mx = mag[0], my = mag[1], mz = mag[2];
    float emx = 2.0f*(attitudeQ.w*attitudeQ.x + attitudeQ.y*attitudeQ.z)*mx
              + 2.0f*(attitudeQ.w*attitudeQ.y - attitudeQ.x*attitudeQ.z)*my
              + (attitudeQ.w*attitudeQ.w - attitudeQ.x*attitudeQ.x - attitudeQ.y*attitudeQ.y + attitudeQ.z*attitudeQ.z)*mz;
    float emy = 2.0f*(attitudeQ.x*attitudeQ.y - attitudeQ.w*attitudeQ.z)*mx
              + (attitudeQ.w*attitudeQ.w + attitudeQ.x*attitudeQ.x - attitudeQ.y*attitudeQ.y - attitudeQ.z*attitudeQ.z)*my
              + 2.0f*(attitudeQ.w*attitudeQ.x + attitudeQ.y*attitudeQ.z)*mz;
    float heading = atan2f(-emy, emx) * 57.2957795f;
    float err = heading - attitudeYawDeg;
    while(err > 180) err -= 360;
    while(err < -180) err += 360;
    gz += attitudeKpYaw * err;
  }
  const float halfDt = 0.5f * dt * 0.0174532925f;
  float qx = gx*halfDt, qy = gy*halfDt, qz = gz*halfDt;
  float angle = sqrtf(qx*qx + qy*qy + qz*qz);
  Quaternion dq; dq.w = cosf(angle);
  if(angle > 1e-7f){ float scale = sinf(angle)/angle; dq.x=qx*scale; dq.y=qy*scale; dq.z=qz*scale; }
  Quaternion next;
  next.w = attitudeQ.w*dq.w - attitudeQ.x*dq.x - attitudeQ.y*dq.y - attitudeQ.z*dq.z;
  next.x = attitudeQ.w*dq.x + attitudeQ.x*dq.w + attitudeQ.y*dq.z - attitudeQ.z*dq.y;
  next.y = attitudeQ.w*dq.y - attitudeQ.x*dq.z + attitudeQ.y*dq.w + attitudeQ.z*dq.x;
  next.z = attitudeQ.w*dq.z + attitudeQ.x*dq.y - attitudeQ.y*dq.x + attitudeQ.z*dq.w;
  attitudeQ = next; attitudeQ.normalize();
  attitudeQ.toEuler(attitudeRollDeg, attitudePitchDeg, attitudeYawDeg);
  attitudeRollRateDps = gx; attitudePitchRateDps = gy; attitudeYawRateDps = gz;
  attitudeLastUpdateUs = esp_timer_get_time();
}

static float attitudeWorldVerticalSpecificForce(const float accelBody[3]){
  float r20 = 2.0f*(attitudeQ.x*attitudeQ.z - attitudeQ.w*attitudeQ.y);
  float r21 = 2.0f*(attitudeQ.y*attitudeQ.z + attitudeQ.w*attitudeQ.x);
  float r22 = 1.0f - 2.0f*(attitudeQ.x*attitudeQ.x + attitudeQ.y*attitudeQ.y);
  return r20*accelBody[0] + r21*accelBody[1] + r22*accelBody[2];
}

/* ============================================================================
   VERTICAL KALMAN FILTER
   ============================================================================ */

struct VertKf {
  float height_m=0, velocityMps=0, accelBiasMps2=0;
  float p00=0.04f, p01=0, p10=0, p11=0.1f;
  float accelNoise=1.5f, baroNoise=0.0025f, lidarNoise=0.01f;
  bool initialized=false; uint32_t lastUpdateUs=0;
  void init(float h, float v){
    height_m=h; velocityMps=v; accelBiasMps2=0;
    p00=0.04f; p01=0; p10=0; p11=0.1f;
    initialized=true; lastUpdateUs=esp_timer_get_time();
  }
  void predict(float accelUp, float dt){
    if(!initialized||!isfinite(accelUp)||!isfinite(dt)||dt<=0.0f||dt>0.02f) return;
    float a = accelUp - accelBiasMps2;
    height_m += velocityMps*dt + 0.5f*a*dt*dt;
    velocityMps += a*dt;
    float dt2=dt*dt, dt3=dt2*dt, dt4=dt3*dt, q=accelNoise*accelNoise;
    p00 += dt*(p01+p10) + dt2*p11 + 0.25f*dt4*q;
    p01 += dt*p11 + 0.5f*dt3*q;
    p10 += dt*p11 + 0.5f*dt3*q;
    p11 += dt2*q;
    lastUpdateUs = esp_timer_get_time();
  }
  void updateMeasurement(float measuredH, float sigmaM){
    if(!initialized||!isfinite(measuredH)||!isfinite(sigmaM)||sigmaM<=0.0f) return;
    float r = fmaxf(sigmaM*sigmaM, 1e-6f);
    float innovation = measuredH - height_m;
    float innovationVar = p00 + r;
    if(!isfinite(innovationVar)||innovationVar<=0.0f) return;
    if(fabsf(innovation) > 5.0f*sqrtf(innovationVar)) return;
    float k0 = p00/innovationVar;
    float k1 = p10/innovationVar;
    height_m += k0*innovation;
    velocityMps += k1*innovation;
    float oldP00=p00, oldP01=p01, oldP10=p10, oldP11=p11;
    float a = 1.0f - k0;
    float newP00 = a*a*oldP00 + k0*k0*r;
    float newP01 = a*(oldP01 - k1*oldP00) + k0*k1*r;
    float newP10 = a*(oldP10 - k1*oldP00) + k1*k0*r;
    float newP11 = oldP11 - k1*oldP01 - k1*oldP10 + k1*k1*oldP00 + k1*k1*r;
    p00 = fmaxf(newP00, 1e-8f);
    p01 = p10 = 0.5f*(newP01 + newP10);
    p11 = fmaxf(newP11, 1e-8f);
    lastUpdateUs = esp_timer_get_time();
  }
  void updateBaro(float h, float sigma){ updateMeasurement(h, sigma); }
  void updateLidar(float h, float sigma){ updateMeasurement(h, sigma); }
};

static VertKf vertKf;

/* ============================================================================
   PILOT COMMAND STRUCT
   ============================================================================ */

struct PilotCommand {
  bool valid = false;
  uint32_t timestampMs = 0;
  uint8_t buttons = 0;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  float throttle = 0.0f, rt = 0.0f, lt = 0.0f;
};

/* ============================================================================
   CALIBRATION MATH
   ============================================================================ */

static void applyAccelCal(const float raw[3], float out[3]){
  if(!calAccelValid) return;
  out[0] = cal::accel_correction[0][0]*(raw[0]-cal::accel_bias_mps2[0])
         + cal::accel_correction[0][1]*(raw[1]-cal::accel_bias_mps2[1])
         + cal::accel_correction[0][2]*(raw[2]-cal::accel_bias_mps2[2]);
  out[1] = cal::accel_correction[1][0]*(raw[0]-cal::accel_bias_mps2[0])
         + cal::accel_correction[1][1]*(raw[1]-cal::accel_bias_mps2[1])
         + cal::accel_correction[1][2]*(raw[2]-cal::accel_bias_mps2[2]);
  out[2] = cal::accel_correction[2][0]*(raw[0]-cal::accel_bias_mps2[0])
         + cal::accel_correction[2][1]*(raw[1]-cal::accel_bias_mps2[1])
         + cal::accel_correction[2][2]*(raw[2]-cal::accel_bias_mps2[2]);
}
static void applyGyroCal(const float raw[3], float out[3]){
  if(!calGyroValid) return;
  out[0] = raw[0] - cal::gyro_bias_dps[0];
  out[1] = raw[1] - cal::gyro_bias_dps[1];
  out[2] = raw[2] - cal::gyro_bias_dps[2];
}
static void applyMagCal(const float rawUt[3], float out[3]){
  if(!calMagValid) return;
  float c[3] = {
    rawUt[0] - cal::mag_hard_iron_ut[0],
    rawUt[1] - cal::mag_hard_iron_ut[1],
    rawUt[2] - cal::mag_hard_iron_ut[2]
  };
  for(int i=0; i<3; i++){
    out[i] = cal::mag_soft_iron[i][0]*c[0]
           + cal::mag_soft_iron[i][1]*c[1]
           + cal::mag_soft_iron[i][2]*c[2];
  }
}

/* ============================================================================
   WI-FI PID TUNER (WebSocket server)
   ============================================================================ */

static int createServerSocket(uint16_t port){
  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if(sock < 0) return -1;
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0){ close(sock); return -1; }
  if(listen(sock, 5) < 0){ close(sock); return -1; }
  return sock;
}

static bool sendAll(int sock, const uint8_t* data, size_t len){
  size_t sent = 0;
  while(sent < len){
    ssize_t n = send(sock, data + sent, len - sent, 0);
    if(n <= 0) return false;
    sent += n;
  }
  return true;
}

static bool wsSendText(int sock, const char* payload){
  size_t len = strlen(payload);
  uint8_t header[10]; int hlen = 0;
  header[hlen++] = 0x81;
  if(len <= 125){ header[hlen++] = (uint8_t)len; }
  else if(len <= 65535){
    header[hlen++] = 126;
    header[hlen++] = (uint8_t)(len >> 8);
    header[hlen++] = (uint8_t)(len & 0xFF);
  } else {
    header[hlen++] = 127;
    header[hlen++] = 0; header[hlen++] = 0;
    header[hlen++] = 0; header[hlen++] = 0;
    header[hlen++] = (uint8_t)(len >> 24);
    header[hlen++] = (uint8_t)(len >> 16);
    header[hlen++] = (uint8_t)(len >> 8);
    header[hlen++] = (uint8_t)len;
  }
  if(!sendAll(sock, header, hlen)) return false;
  if(!sendAll(sock, (const uint8_t*)payload, len)) return false;
  return true;
}

static int wsRecvText(int sock, char* buf, size_t bufsz){
  struct timeval tv; tv.tv_sec=0; tv.tv_usec=1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  uint8_t header[2];
  if(recv(sock, header, 2, MSG_WAITALL) != 2) return -1;
  uint8_t opcode = header[0] & 0x0F;
  if(opcode != 0x01 && opcode != 0x08) return -1;
  if(opcode == 0x08) return -2;
  uint8_t lenByte = header[1];
  size_t payloadLen = lenByte & 0x7F;
  bool masked = (lenByte & 0x80) != 0;
  if(payloadLen == 126){
    uint8_t ext[2];
    if(recv(sock, ext, 2, MSG_WAITALL) != 2) return -1;
    payloadLen = ((size_t)ext[0] << 8) | ext[1];
  } else if(payloadLen == 127){ return -1; }
  uint8_t mask[4] = {0,0,0,0};
  if(masked){ if(recv(sock, mask, 4, MSG_WAITALL) != 4) return -1; }
  if(payloadLen >= bufsz) payloadLen = bufsz - 1;
  if(payloadLen > 0){
    if(recv(sock, (uint8_t*)buf, payloadLen, MSG_WAITALL) != (int)payloadLen) return -1;
    if(masked){ for(size_t i=0; i<payloadLen; i++) buf[i] ^= mask[i%4]; }
  }
  buf[payloadLen] = 0;
  return (int)payloadLen;
}

static void handleHttpClient(int clientSock){
  char req[256];
  int len = recv(clientSock, req, sizeof(req)-1, 0);
  if(len <= 0){ close(clientSock); return; }
  req[len] = 0;
  const char* response =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n"
    "\r\n";
  send(clientSock, response, strlen(response), 0);
  extern const char WEB_GUI_HTML[];
  send(clientSock, WEB_GUI_HTML, strlen(WEB_GUI_HTML), 0);
  close(clientSock);
}

static bool startWifiAp(){
  esp_netif_t* netif = esp_netif_create_default_wifi_ap();
  if(!netif) return false;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t ret = esp_wifi_init(&cfg);
  if(ret != ESP_OK){
    Serial.printf("WiFi init failed: %s\n", esp_err_to_name(ret));
    return false;
  }
  wifi_config_t wifiConfig = {};
  memcpy(wifiConfig.ap.ssid, cal::wifi_ssid, strlen(cal::wifi_ssid));
  memcpy(wifiConfig.ap.password, cal::wifi_password, strlen(cal::wifi_password));
  wifiConfig.ap.ssid_len = strlen(cal::wifi_ssid);
  wifiConfig.ap.channel = cal::wifi_channel;
  wifiConfig.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  wifiConfig.ap.max_connection = cal::wifi_max_conn;
  ret = esp_wifi_set_mode(WIFI_MODE_AP);
  if(ret != ESP_OK){
    Serial.printf("WiFi set mode failed: %s\n", esp_err_to_name(ret));
    return false;
  }
  ret = esp_wifi_set_config(WIFI_IF_AP, &wifiConfig);
  if(ret != ESP_OK){
    Serial.printf("WiFi set config failed: %s\n", esp_err_to_name(ret));
    return false;
  }
  ret = esp_wifi_start();
  if(ret != ESP_OK){
    Serial.printf("WiFi start failed: %s\n", esp_err_to_name(ret));
    return false;
  }
  esp_netif_ip_info_t ipInfo;
  if(esp_netif_get_ip_info(netif, &ipInfo) == ESP_OK){
    snprintf(ipString, sizeof(ipString), IPSTR, IP2STR(&ipInfo.ip));
  }
  Serial.printf("WiFi AP started: SSID=%s IP=%s\n", cal::wifi_ssid, ipString);
  return true;
}

/* ============================================================================
   I²C TRANSACTION MANAGER (minimal, inline)
   ============================================================================ */

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value){
  Wire.beginTransmission(addr);
  if(Wire.write(reg) != 1){ Wire.endTransmission(); return false; }
  if(Wire.write(value) != 1){ Wire.endTransmission(); return false; }
  return Wire.endTransmission() == 0;
}
static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* out, size_t count){
  Wire.beginTransmission(addr);
  if(Wire.write(reg) != 1){ Wire.endTransmission(); return false; }
  if(Wire.endTransmission(false) != 0) return false;
  int received = Wire.requestFrom((int)addr, (int)count);
  if(received != (int)count){
    while(Wire.available()) (void)Wire.read();
    return false;
  }
  for(size_t i=0; i<count; i++){
    if(Wire.available() <= 0) return false;
    out[i] = (uint8_t)Wire.read();
  }
  return true;
}
static bool i2cReadQmcRegs(uint8_t reg, uint8_t* out, size_t count){
  Wire.beginTransmission(kMagAddress);
  if(Wire.write(reg) != 1){ (void)Wire.endTransmission(true); return false; }
  if(Wire.endTransmission(true) != 0) return false;
  int received = Wire.requestFrom((int)kMagAddress, (int)count);
  if(received != (int)count){
    while(Wire.available() > 0) (void)Wire.read();
    return false;
  }
  for(size_t i=0; i<count; ++i){
    if(Wire.available() <= 0) return false;
    out[i] = (uint8_t)Wire.read();
  }
  return true;
}
static bool i2cReadQmcByte(uint8_t reg, uint8_t& value){
  return i2cReadQmcRegs(reg, &value, 1);
}
static bool i2cIsPresent(uint8_t addr){
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

/* ============================================================================
   SENSOR DRIVERS
   ============================================================================ */

static bool initMpu(){
  Wire.setClock(400000);
  mpuAddress = 0;
  if(i2cIsPresent(kMpuPrimaryAddr)) mpuAddress = kMpuPrimaryAddr;
  else if(i2cIsPresent(kMpuAlternateAddr)) mpuAddress = kMpuAlternateAddr;
  if(!mpuAddress) return false;
  uint8_t id = 0;
  if(!i2cReadRegs(mpuAddress, kMpuRegWhoAmI, &id, 1) || id != 0x68) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegPwrMgmt1, kMpuPwrMgmt1Reset)) return false;
  delay(100);
  uint8_t gyroOffsets[6] = {};
  if(!i2cReadRegs(mpuAddress, kMpuRegXgOffsUsrh, gyroOffsets, sizeof(gyroOffsets))) return false;
  for(uint8_t v : gyroOffsets) if(v != 0u) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegPwrMgmt1, kMpuPwrMgmt1Clksel)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegPwrMgmt2, 0x00)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegConfig, kMpuDlpf188Hz)) return false;
  if(!i2cWriteReg(mpuAddress, 0x19, 0x01)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegGyroConfig, kMpuGyroFs500)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegAccelConfig, kMpuAccelFs8G)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegFifoEn,
      kMpuFifoTempEn | kMpuFifoXgEn | kMpuFifoYgEn | kMpuFifoZgEn | kMpuFifoAccelEn)) return false;
  uint8_t ic = 0;
  if(!i2cReadRegs(mpuAddress, kMpuRegIntPinCfg, &ic, 1)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegIntPinCfg, ic | kMpuIntPinCfgBypassEn)) return false;
  uint8_t uc = 0;
  if(!i2cReadRegs(mpuAddress, kMpuRegUserCtrl, &uc, 1)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegUserCtrl, uc | kMpuUserCtrlFifoEn)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegUserCtrl, uc | kMpuUserCtrlFifoEn | kMpuUserCtrlFifoReset)) return false;
  if(!i2cWriteReg(mpuAddress, kMpuRegIntEnable, kMpuIntEnableRawRdyEn)) return false;
  uint8_t gyroCfg = 0, accelCfg = 0;
  if(!i2cReadRegs(mpuAddress, kMpuRegGyroConfig, &gyroCfg, 1)
     || !i2cReadRegs(mpuAddress, kMpuRegAccelConfig, &accelCfg, 1)) return false;
  if((gyroCfg & 0x18u) != (kMpuGyroFs500 & 0x18u)
     || (accelCfg & 0x18u) != (kMpuAccelFs8G & 0x18u)) return false;
  mpuPresent = true;
  return true;
}

static bool readMpuFifo(ImuSample& s){
  s.fresh = false;
  Wire.setClock(400000);
  if(!mpuPresent) return false;
  uint8_t fc[2] = {};
  if(!i2cReadRegs(mpuAddress, kMpuRegFifoCountH, fc, 2)){
    return false;
  }
  uint16_t fcnt = (uint16_t(fc[0]) << 8) | fc[1];
  uint16_t completeRecords = fcnt / 14u;
  s.fifo_count = completeRecords;
  uint32_t nowUs = micros();
  if(fcnt >= 1024u){
    s.overflows++;
    i2cWriteReg(mpuAddress, kMpuRegUserCtrl, kMpuUserCtrlFifoEn | kMpuUserCtrlFifoReset);
    s.valid = false;
    return false;
  }
  if(completeRecords == 0u){
    if(s.timestamp_us == 0u || uint32_t(nowUs - s.timestamp_us) > 4000u)
      s.valid = false;
    return s.valid;
  }
  uint8_t d[14] = {};
  if(!i2cReadRegs(mpuAddress, kMpuRegFifoRW, d, sizeof(d))){
    return false;
  }
  for(int i=0; i<3; i++){
    s.accel_raw[i] = (int16_t)((uint16_t(d[2*i]) << 8) | d[2*i+1]);
    s.gyro_raw[i]  = (int16_t)((uint16_t(d[8+2*i]) << 8) | d[9+2*i]);
  }
  s.temp_raw = (int16_t)((uint16_t(d[6]) << 8) | d[7]);
  uint32_t backlogAgeUs = (completeRecords - 1u) * 2000u;
  s.timestamp_us = nowUs - backlogAgeUs;
  s.valid = true; s.fresh = true;
  return true;
}

static bool initMag(){
  Wire.setClock(400000);
  magAddress = kMagAddress;
  if(!i2cIsPresent(magAddress)) return false;
  uint8_t id = 0;
  if(!i2cReadQmcByte(kQmcRegChipId, id) || id != kQmcChipId) return false;
  if(!i2cWriteReg(magAddress, kQmcRegCtrl1, 0x00)) return false;
  delay(20);
  if(!i2cWriteReg(magAddress, kQmcRegXyzSign, kQmcXyzSignVal)) return false;
  if(!i2cWriteReg(magAddress, kQmcRegCtrl2, kQmcCtrl2_8Gauss)) return false;
  if(!i2cWriteReg(magAddress, kQmcRegCtrl1, kQmcCtrl1_200Hz)) return false;
  uint8_t ctrl1=0, ctrl2=0, xyzSign=0;
  if(!i2cReadQmcByte(kQmcRegCtrl1, ctrl1)
     || !i2cReadQmcByte(kQmcRegCtrl2, ctrl2)
     || !i2cReadQmcByte(kQmcRegXyzSign, xyzSign)) return false;
  if(ctrl1 != kQmcCtrl1_200Hz || ctrl2 != kQmcCtrl2_8Gauss || xyzSign != kQmcXyzSignVal) return false;
  magConfigured = true;
  Wire.setClock(400000);
  return true;
}

static bool readMag(MagSample& s){
  s.fresh = false;
  uint32_t nowUs = micros();
  if(s.timestamp_us == 0 || uint32_t(nowUs - s.timestamp_us) >= 25000u)
    s.valid = false;
  if(!magConfigured) return false;
  Wire.setClock(400000);
  uint8_t st = 0;
  if(!i2cReadQmcByte(kQmcRegStatus, st)) return false;
  s.status = st;
  if(st & 0x02) return false;
  if(!(st & 0x01)) return false;
  uint8_t d[6] = {};
  if(!i2cReadQmcRegs(kQmcRegDataXl, d, 6)) return false;
  for(int i=0; i<3; i++)
    s.raw[i] = (int16_t)((uint16_t(d[2*i+1]) << 8) | d[2*i]);
  if((s.raw[0]==0&&s.raw[1]==0&&s.raw[2]==0)
     || (s.raw[0]==-1&&s.raw[1]==-1&&s.raw[2]==-1)
     || s.raw[0]==INT16_MIN || s.raw[1]==INT16_MIN || s.raw[2]==INT16_MIN)
    return false;
  s.timestamp_us = esp_timer_get_time();
  s.valid = true; s.fresh = true;
  return true;
}

static bool initBmp(){
  Wire.setClock(400000);
  bmpAddress = 0;
  if(i2cIsPresent(kBmpPrimaryAddr)) bmpAddress = kBmpPrimaryAddr;
  else if(i2cIsPresent(kBmpAlternateAddr)) bmpAddress = kBmpAlternateAddr;
  if(!bmpAddress) return false;
  uint8_t id = 0;
  if(!i2cReadRegs(bmpAddress, kBmpRegId, &id, 1) || id != kBmpChipId) return false;
  bmpPresent = true;
  return true;
}

static bool readBmp(BaroSample& s){
  s.fresh = false;
  Wire.setClock(400000);
  if(!bmpPresent) return false;
  uint32_t nowUs = micros();
  uint32_t ageUs = s.timestamp_us == 0u ? UINT32_MAX : uint32_t(nowUs - s.timestamp_us);
  if(ageUs > 20000u) s.valid = false;
  if(s.timestamp_us != 0u && ageUs < 6000u) return s.valid;
  uint8_t status = 0;
  if(!i2cReadRegs(bmpAddress, kBmpRegStatus, &status, 1)) return s.valid;
  if((status & 0x08u) != 0u) return s.valid;
  uint8_t r[6] = {};
  if(!i2cReadRegs(bmpAddress, kBmpRegPressureMsb, r, 6)) return false;
  s.raw_pressure = ((uint32_t(r[0]) << 12) | ((uint32_t(r[1]) << 4) | (r[2] >> 4)));
  s.raw_temperature = ((uint32_t(r[3]) << 12) | ((uint32_t(r[4]) << 4) | (r[5] >> 4)));
  s.timestamp_us = micros();
  s.valid = true; s.fresh = true;
  return true;
}

/* ============================================================================
   CALIBRATION LOAD (single explicit call at startup)
   ============================================================================ */

static void loadCalibration(){
  using namespace cal;
  /* Gyro bias: subtract from raw/65.5 after confirming MPU offset registers read zero */
  calGyroValid = true;
  /* Accelerometer: apply C * (raw_mps2 - bias) once */
  calAccelValid = true;
  /* Magnetometer: apply S * (raw_uT - hardIron) once */
  calMagValid = true;
  /* Barometer: no ground reference; altitude control unavailable */
  calBaroValid = false;
  calibrationLoaded = calAccelValid && calGyroValid && calMagValid;
  Serial.printf("Calibration loaded: accel=%d gyro=%d mag=%d baro=%d\n",
                calAccelValid, calGyroValid, calMagValid, calBaroValid);
}

/* ============================================================================
   MOTOR MIXER (X-frame) with slew-rate cap
   ============================================================================ */

static void motorMixerInit(){
  for(int i=0; i<4; i++) motorLastCmd[i] = 0.0f;
}
static void motorMixer(float collective, float rollTorque, float pitchTorque,
                       float yawTorque, float ceiling, float idleFraction,
                       float dt, float out[4]){
  float cor[4] = {
    rollTorque - pitchTorque - yawTorque,
   -rollTorque - pitchTorque + yawTorque,
   -rollTorque + pitchTorque - yawTorque,
    rollTorque + pitchTorque + yawTorque
  };
  float lo = cor[0], hi = cor[0];
  for(int i=1; i<4; i++){ lo = fminf(lo, cor[i]); hi = fmaxf(hi, cor[i]); }
  float span = ceiling - idleFraction;
  float scale = (hi - lo > span) ? span / (hi - lo) : 1.0f;
  float alloc = fmaxf(idleFraction, fminf(ceiling - hi*scale, collective));
  alloc = fminf(alloc, ceiling - lo*scale);
  float maxDelta = cal::motor_max_slew_per_second * dt;
  for(int i=0; i<4; i++){
    float r = alloc + scale*cor[i];
    r = fmaxf(idleFraction, fminf(ceiling, r));
    float d = r - motorLastCmd[i];
    d = fmaxf(-maxDelta, fminf(maxDelta, d));
    out[i] = fmaxf(idleFraction, fminf(ceiling, motorLastCmd[i] + d));
    motorLastCmd[i] = out[i];
  }
}

/* ============================================================================
   WEB GUI HTML (embedded)
   ============================================================================ */

const char WEB_GUI_HTML[] =
R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>IntelQCopter - Wireless PID Tuner</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
    color: #e2e8f0; min-height: 100vh; padding: 20px;
  }
  .container { max-width: 1200px; margin: 0 auto; }
  .header { text-align: center; margin-bottom: 30px;
    padding: 20px; background: rgba(30, 41, 59, 0.8);
    border-radius: 12px; border: 1px solid #334155; }
  .header h1 { color: #38bdf8; font-size: 2.5rem; margin-bottom: 8px; }
  .header p { color: #94a3b8; font-size: 1rem; }
  .status-bar {
    display: flex; justify-content: space-between; align-items: center;
    margin-bottom: 20px; padding: 12px 20px;
    background: rgba(30, 41, 59, 0.8);
    border-radius: 12px; border: 1px solid #334155; }
  .status-indicator { display: flex; align-items: center; gap: 10px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  .dot.connected { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
  .dot.disconnected { background: #ef4444; box-shadow: 0 0 10px #ef4444; }
  .grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
    gap: 20px; margin-bottom: 30px; }
  .card {
    background: rgba(30, 41, 59, 0.6); border: 1px solid #334155;
    border-radius: 12px; padding: 20px; backdrop-filter: blur(10px); }
  .card h2 { color: #38bdf8; margin-bottom: 16px; font-size: 1.25rem;
    border-bottom: 1px solid #334155; padding-bottom: 8px; }
  .telemetry-row { display: flex; justify-content: space-between;
    margin: 8px 0; font-size: 0.95rem; }
  .telemetry-label { color: #94a3b8; }
  .telemetry-value { color: #e2e8f0; font-weight: 600;
    font-family: 'Courier New', monospace; }
  .pid-group { margin-bottom: 16px; }
  .pid-group label {
    display: block; color: #94a3b8; margin-bottom: 6px;
    font-size: 0.85rem; text-transform: uppercase; letter-spacing: 0.5px; }
  .pid-row { display: flex; gap: 10px; margin-bottom: 10px; }
  .pid-input {
    flex: 1; padding: 8px 12px;
    background: rgba(15, 23, 42, 0.8); border: 1px solid #475569;
    border-radius: 6px; color: #e2e8f0; font-size: 0.95rem;
    font-family: 'Courier New', monospace; }
  .pid-input:focus { outline: none; border-color: #38bdf8; }
  .pid-input.invalid { border-color: #ef4444; }
  .pid-axis-name {
    font-weight: 600; color: #38bdf8; font-size: 0.9rem;
    margin-bottom: 8px; display: block; }
  .btn-group { display: flex; gap: 10px; margin-top: 16px; }
  .btn {
    padding: 10px 20px; border: none; border-radius: 6px;
    color: #fff; font-size: 0.95rem; font-weight: 600;
    cursor: pointer; transition: all 0.2s; flex: 1; }
  .btn:hover { opacity: 0.9; transform: translateY(-1px); }
  .btn:active { transform: translateY(0); }
  .btn-primary { background: linear-gradient(135deg, #38bdf8, #0ea5e9); }
  .btn-danger { background: linear-gradient(135deg, #ef4444, #dc2626); }
  .btn-success { background: linear-gradient(135deg, #22c55e, #16a34a); }
  .btn:disabled { opacity: 0.5; cursor: not-allowed; }
  .armed-indicator {
    display: inline-block; padding: 6px 16px; border-radius: 20px;
    font-size: 0.85rem; font-weight: 700; }
  .armed { background: rgba(34, 197, 94, 0.2); color: #22c55e; border: 1px solid #22c55e; }
  .disarmed { background: rgba(239, 68, 68, 0.2); color: #ef4444; border: 1px solid #ef4444; }
  .footer { text-align: center; margin-top: 30px; padding: 20px;
    color: #64748b; font-size: 0.85rem; }
  .alert {
    padding: 12px 16px; border-radius: 6px; margin-bottom: 20px;
    font-size: 0.9rem; }
  .alert.info { background: rgba(56, 189, 248, 0.1); border: 1px solid #38bdf8; color: #38bdf8; }
  .alert.warn { background: rgba(245, 158, 11, 0.1); border: 1px solid #f59e0b; color: #f59e0b; }
  .alert.error { background: rgba(239, 68, 68, 0.1); border: 1px solid #ef4444; color: #ef4444; }
  .hidden { display: none; }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <h1>IntelQCopter</h1>
    <p>Wireless PID Tuner &amp; Telemetry</p>
  </div>
  <div id="alert-area"></div>
  <div class="status-bar">
    <div class="status-indicator">
      <div class="dot disconnected" id="ws-dot"></div>
      <span id="ws-status">Disconnected</span>
    </div>
    <div>
      <strong>IP:</strong> <span id="ip-addr">192.168.4.1</span> |
      <strong>Port:</strong> 81 |
      <strong>Loop:</strong> <span id="loop-rate">--</span> Hz
    </div>
  </div>
  <div class="grid">
    <div class="card">
      <h2>Attitude</h2>
      <div class="telemetry-row">
        <span class="telemetry-label">Roll</span>
        <span class="telemetry-value" id="tel-roll">0.00°</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Pitch</span>
        <span class="telemetry-value" id="tel-pitch">0.00°</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Yaw</span>
        <span class="telemetry-value" id="tel-yaw">0.00°</span>
      </div>
    </div>
    <div class="card">
      <h2>Angular Rates</h2>
      <div class="telemetry-row">
        <span class="telemetry-label">Roll Rate</span>
        <span class="telemetry-value" id="tel-roll-rate">0.00°/s</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Pitch Rate</span>
        <span class="telemetry-value" id="tel-pitch-rate">0.00°/s</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Yaw Rate</span>
        <span class="telemetry-value" id="tel-yaw-rate">0.00°/s</span>
      </div>
    </div>
    <div class="card">
      <h2>Motor Outputs</h2>
      <div class="telemetry-row">
        <span class="telemetry-label">FL</span>
        <span class="telemetry-value" id="tel-motor0">0.00</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">FR</span>
        <span class="telemetry-value" id="tel-motor1">0.00</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">RR</span>
        <span class="telemetry-value" id="tel-motor2">0.00</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">RL</span>
        <span class="telemetry-value" id="tel-motor3">0.00</span>
      </div>
    </div>
    <div class="card">
      <h2>Flight State</h2>
      <div class="telemetry-row">
        <span class="telemetry-label">Armed</span>
        <span class="armed-indicator disarmed" id="state-armed">DISARMED</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Altitude</span>
        <span class="telemetry-value" id="tel-altitude">0.00 m</span>
      </div>
      <div class="telemetry-row">
        <span class="telemetry-label">Vert Speed</span>
        <span class="telemetry-value" id="tel-vert-speed">0.00 m/s</span>
      </div>
    </div>
  </div>
  <div class="grid">
    <div class="card">
      <h2>Roll Angle PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Roll Angle</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-roll-angle-kp" step="0.01" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-roll-angle-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-roll-angle-kd" step="0.001" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('roll_angle')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Pitch Angle PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Pitch Angle</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-pitch-angle-kp" step="0.01" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-pitch-angle-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-pitch-angle-kd" step="0.001" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('pitch_angle')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Roll Rate PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Roll Rate</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-roll-rate-kp" step="0.001" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-roll-rate-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-roll-rate-kd" step="0.0001" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('roll_rate')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Pitch Rate PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Pitch Rate</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-pitch-rate-kp" step="0.001" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-pitch-rate-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-pitch-rate-kd" step="0.0001" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('pitch_rate')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Yaw Rate PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Yaw Rate</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-yaw-rate-kp" step="0.001" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-yaw-rate-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-yaw-rate-kd" step="0.0001" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('yaw_rate')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Altitude PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Altitude</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-altitude-kp" step="0.01" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-altitude-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-altitude-kd" step="0.01" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('altitude')">Apply</button>
      </div>
    </div>
    <div class="card">
      <h2>Vertical Speed PID</h2>
      <div class="pid-group">
        <span class="pid-axis-name">Vertical Speed</span>
        <div class="pid-row">
          <input type="number" class="pid-input" id="pid-vert-speed-kp" step="0.01" placeholder="Kp">
          <input type="number" class="pid-input" id="pid-vert-speed-ki" step="0.001" placeholder="Ki">
          <input type="number" class="pid-input" id="pid-vert-speed-kd" step="0.01" placeholder="Kd">
        </div>
        <button class="btn btn-primary" onclick="sendPid('vert_speed')">Apply</button>
      </div>
    </div>
  </div>
  <div class="btn-group" style="justify-content: center; max-width: 600px; margin: 0 auto;">
    <button class="btn btn-success" onclick="requestAllPids()">Request All PIDs</button>
    <button class="btn btn-danger" onclick="emergencyDisarm()">EMERGENCY DISARM</button>
  </div>
  <div class="footer">
    IntelQCopter Wireless PID Tuner &bull; ESP32-S3 WiFi AP &bull; WebSocket: ws://192.168.4.1:81
  </div>
</div>
<script>
let ws = null; let reconnectTimer = null;
function connect(){
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = protocol + '//' + window.location.hostname + ':81';
  ws = new WebSocket(wsUrl);
  ws.onopen = () => {
    document.getElementById('ws-dot').className = 'dot connected';
    document.getElementById('ws-status').textContent = 'Connected';
    requestAllPids();
  };
  ws.onclose = () => {
    document.getElementById('ws-dot').className = 'dot disconnected';
    document.getElementById('ws-status').textContent = 'Disconnected';
    ws = null; reconnectTimer = setTimeout(connect, 2000);
  };
  ws.onerror = (err) => { console.error('WebSocket error:', err); };
  ws.onmessage = (event) => {
    try { const msg = JSON.parse(event.data); handleMessage(msg); }
    catch(e) { console.error('Parse error:', e); }
  };
}
function handleMessage(msg){
  switch(msg.type){
    case 'telemetry': updateTelemetry(msg); break;
    case 'pid_values': updatePidInputs(msg); break;
    case 'pid_ack': break;
    case 'error': break;
  }
}
function updateTelemetry(t){
  document.getElementById('tel-roll').textContent = (t.roll||0).toFixed(2) + '°';
  document.getElementById('tel-pitch').textContent = (t.pitch||0).toFixed(2) + '°';
  document.getElementById('tel-yaw').textContent = (t.yaw||0).toFixed(2) + '°';
  document.getElementById('tel-roll-rate').textContent = (t.roll_rate||0).toFixed(2) + '°/s';
  document.getElementById('tel-pitch-rate').textContent = (t.pitch_rate||0).toFixed(2) + '°/s';
  document.getElementById('tel-yaw-rate').textContent = (t.yaw_rate||0).toFixed(2) + '°/s';
  const armedEl = document.getElementById('state-armed');
  if(t.armed){ armedEl.className = 'armed-indicator armed'; armedEl.textContent = 'ARMED'; }
  else { armedEl.className = 'armed-indicator disarmed'; armedEl.textContent = 'DISARMED'; }
  document.getElementById('tel-altitude').textContent = (t.altitude||0).toFixed(2) + ' m';
  document.getElementById('tel-vert-speed').textContent = (t.vert_speed||0).toFixed(2) + ' m/s';
  if(t.motor && t.motor.length === 4){
    document.getElementById('tel-motor0').textContent = (t.motor[0]||0).toFixed(3);
    document.getElementById('tel-motor1').textContent = (t.motor[1]||0).toFixed(3);
    document.getElementById('tel-motor2').textContent = (t.motor[2]||0).toFixed(3);
    document.getElementById('tel-motor3').textContent = (t.motor[3]||0).toFixed(3);
  }
  if(t.loop_rate) document.getElementById('loop-rate').textContent = t.loop_rate;
}
function updatePidInputs(pids){
  const map = {
    'roll_angle': ['roll-angle',['kp','ki','kd']],
    'pitch_angle': ['pitch-angle',['kp','ki','kd']],
    'roll_rate': ['roll-rate',['kp','ki','kd']],
    'pitch_rate': ['pitch-rate',['kp','ki','kd']],
    'yaw_rate': ['yaw-rate',['kp','ki','kd']],
    'altitude': ['altitude',['kp','ki','kd']],
    'vert_speed': ['vert-speed',['kp','ki','kd']]
  };
  for(const [axis, info] of Object.entries(map)){
    if(pids[axis]){
      const [idPrefix, gains] = info;
      gains.forEach(g => {
        const el = document.getElementById('pid-' + idPrefix + '-' + g);
        if(el) el.value = pids[axis][g];
      });
    }
  }
}
function sendPid(axis){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  const gains = {}; let valid = true;
  ['kp','ki','kd'].forEach(g => {
    const el = document.getElementById('pid-' + axis + '-' + g);
    const val = parseFloat(el.value);
    if(isNaN(val)){ el.classList.add('invalid'); valid = false; }
    else { el.classList.remove('invalid'); gains[g] = val; }
  });
  if(!valid) return;
  ws.send(JSON.stringify({ type: 'pid_set', axis: axis, kp: gains.kp, ki: gains.ki, kd: gains.kd }));
}
function requestAllPids(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: 'pid_get' }));
}
function emergencyDisarm(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  if(confirm('EMERGENCY DISARM - All motors will stop immediately. Continue?')){
    ws.send(JSON.stringify({ type: 'disarm' }));
  }
}
connect();
setInterval(() => { if(!ws && !reconnectTimer) connect(); }, 3000);
</script>
</body>
</html>
)rawliteral";

/* ============================================================================
   WEB GUI HTML (embedded) - forward declaration
   ============================================================================ */
extern const char WEB_GUI_HTML[];

/* ============================================================================
   PID getters/setters for WebSocket tuner
   ============================================================================ */

static const char* pidAxisToName(int idx){
  switch(idx){
    case 0: return "roll_angle";
    case 1: return "pitch_angle";
    case 2: return "roll_rate";
    case 3: return "pitch_rate";
    case 4: return "yaw_rate";
    case 5: return "altitude";
    case 6: return "vert_speed";
    default: return "";
  }
}
static int pidNameToAxis(const char* axis){
  if(strcmp(axis, "roll_angle") == 0) return 0;
  if(strcmp(axis, "pitch_angle") == 0) return 1;
  if(strcmp(axis, "roll_rate") == 0) return 2;
  if(strcmp(axis, "pitch_rate") == 0) return 3;
  if(strcmp(axis, "yaw_rate") == 0) return 4;
  if(strcmp(axis, "altitude") == 0) return 5;
  if(strcmp(axis, "vert_speed") == 0) return 6;
  return -1;
}

/* ============================================================================
   MAIN CONTROL STEP
   ============================================================================ */

static void computeControl(const PilotCommand& cmd, float dt, float out[4]){
  const bool imuOk = calibrationLoaded && calAccelValid && calGyroValid
      && imu.valid && attitudeInitialized;
  const bool baroOk = calBaroValid && baro.valid && vertKf.initialized;
  const bool pilotOk = cmd.valid
      && uint32_t(millis() - cmd.timestampMs) < 250u;
  if(!imuOk || !baroOk || !pilotOk || !isfinite(dt) || dt <= 0.0f || dt > 0.01f){
    armed = false; failsafe = true;
    for(int i=0; i<4; i++) out[i] = 0.0f;
    return;
  }
  failsafe = false;
  /* Attitude */
  float attRoll = attitudeRollDeg, attPitch = attitudePitchDeg, attYaw = attitudeYawDeg;
  float attRollRate = attitudeRollRateDps, attPitchRate = attitudePitchRateDps, attYawRate = attitudeYawRateDps;
  /* Angle P controllers -> rate setpoints */
  const float deg2rad = 0.0174532925f;
  float rollSp = cmd.roll * cal::max_roll_deg * deg2rad;
  float pitchSp = cmd.pitch * cal::max_pitch_deg * deg2rad;
  /* Inner rate PIDs -> torque (N·m, model-only) */
  float rollTorque = 0.0f, pitchTorque = 0.0f, yawTorque = 0.0f;
  /* NOTE: simulation-only PIDs are NOT wired to live mixer output.
     Torque-to-command conversion is not yet reconciled. */
  /* Vertical speed */
  float vs = cmd.throttle * cal::max_vertical_speed_mps;
  float boost = fmaxf(0.0f, fminf(1.0f, boostState));
  float tt = (boost - ltReduceState) * 1.0f;
  vs += tt;
  vs = fmaxf(-cal::max_descent_mps, fminf(cal::max_climb_mps, vs));
  /* Collective */
  float cr = cosf(attRoll * deg2rad), cp = cosf(attPitch * deg2rad);
  float tf = fmaxf(0.5f, cr * cp);
  float coll = (hoverCollective + 0.0f) / tf;
  coll = fmaxf(cal::motor_idle_fraction, fminf(cal::motor_boost_ceiling, coll));
  float ceiling = cal::motor_normal_ceiling
                + (cal::motor_boost_ceiling - cal::motor_normal_ceiling) * boost;
  motorMixer(coll, rollTorque, pitchTorque, yawTorque, ceiling,
             cal::motor_idle_fraction, dt, out);
}

/* ============================================================================
   Arduino entry points
   ============================================================================ */

void setup(){
  Serial.begin(921600);
  while(!Serial && millis() < 2000){}
  Serial.println("=== IntelQCopter Flight Controller ===");
  Serial.println("Target: ESP32-S3 DevKitC-1 N8");
  Serial.println("WiFi PID Tuner: Enabled");
  Serial.println("Motor Outputs: DISABLED (INPUT ONLY)");
  Serial.println();

  /* Load calibration once at startup */
  loadCalibration();

  /* Motor pins INPUT ONLY */
  for(uint8_t p : kMotorPins) pinMode(p, INPUT);

  /* Start WiFi AP + WebSocket */
  if(startWifiAp()){
    httpSock = createServerSocket(cal::http_port);
    wsSock   = createServerSocket(cal::ws_port);
  }

  /* Initialize sensors */
  mpuPresent = initMpu();
  if(mpuPresent) delay(30);
  bmpPresent = initBmp();
  magConfigured = initMag();

  Serial.println("Flight controller initialized.");
  Serial.printf("WiFi AP: %s / %s\n", cal::wifi_ssid, cal::wifi_password);
  Serial.printf("Web GUI: http://%s\n", ipString);
  Serial.printf("WebSocket: ws://%s:%d\n", ipString, cal::ws_port);
  Serial.println("========================================");
}

void loop(){
  static uint32_t seq = 0, previousLoopUs = 0;
  static uint32_t previousImuUs = 0;
  int64_t now = esp_timer_get_time();
  static int64_t nextDeadline = 0;
  if(nextDeadline == 0) nextDeadline = now + 2000;
  int64_t ud = nextDeadline - now;
  if(ud > 300){
    delayMicroseconds((uint32_t)(ud - 200));
    return;
  }
  if(ud > 0) return;
  int64_t sa = esp_timer_get_time();
  int64_t late = sa - nextDeadline;
  if(late >= (int64_t)2000){
    uint64_t sk = (uint64_t)late / 2000u;
    nextDeadline += (int64_t)sk * 2000;
  }
  const uint32_t loopStartUs = (uint32_t)sa;
  const float dt = previousLoopUs == 0u
      ? 2000.0f / 1000000.0f
      : uint32_t(loopStartUs - previousLoopUs) / 1000000.0f;
  previousLoopUs = loopStartUs;
  nextDeadline += 2000;

  /* Read sensors */
  bool imuOk = readMpuFifo(imu);
  bool magOk = readMag(mag);
  bool baroOk = readBmp(baro);

  /* Magnetometer: apply calibration once */
  if(magOk && mag.fresh){
    float mru[3] = {
      mag.raw[0] / cal::mag_counts_per_ut,
      mag.raw[1] / cal::mag_counts_per_ut,
      mag.raw[2] / cal::mag_counts_per_ut
    };
    mag.calibrated[0] = mag.calibrated[1] = mag.calibrated[2] = 0.0f;
    if(calMagValid) applyMagCal(mru, mag.calibrated);
  }

  /* IMU: apply calibration once, update attitude */
  if(imuOk && imu.fresh){
    float ar[3] = {
      imu.accel_raw[0] / 4096.0f * cal::gravity_mps2,
      imu.accel_raw[1] / 4096.0f * cal::gravity_mps2,
      imu.accel_raw[2] / 4096.0f * cal::gravity_mps2
    };
    float gr[3] = {
      imu.gyro_raw[0] / 65.5f,
      imu.gyro_raw[1] / 65.5f,
      imu.gyro_raw[2] / 65.5f
    };
    imu.temperature_c = imu.temp_raw / 340.0f + 36.53f;
    bool accelCalOk = calAccelValid;
    bool gyroCalOk = calGyroValid;
    if(accelCalOk) applyAccelCal(ar, imu.accel);
    if(gyroCalOk) applyGyroCal(gr, imu.gyro);
    if(accelCalOk && gyroCalOk){
      float imuDt = previousImuUs == 0u
          ? 2000.0f / 1000000.0f
          : uint32_t(imu.timestamp_us - previousImuUs) / 1000000.0f;
      previousImuUs = imu.timestamp_us;
      if(!attitudeInitialized) attitudeInit(imu.accel);
      const bool magUsable = false; /* kEnableMagHeadingFusion = false */
      attitudeUpdate(imu.gyro, imu.accel,
          magUsable ? mag.calibrated : nullptr, false, imuDt);
      if(vertKf.initialized){
        float worldSf = attitudeWorldVerticalSpecificForce(imu.accel);
        imu.vertical_accel_world_mps2 = worldSf - cal::gravity_mps2;
        vertKf.predict(imu.vertical_accel_world_mps2, imuDt);
      }
    } else {
      attitudeInitialized = false;
      vertKf.initialized = false;
    }
  }

  /* Barometer: process only fresh samples */
  if(baroOk && baro.fresh){
    /* Note: pressure compensation uses Adafruit_BMP280 driver or raw conversion */
    baro.timestamp_us = micros();
    if(calBaroValid && isfinite(cal::baro_ground_pressure_pa)
       && cal::baro_ground_pressure_pa > 30000.0f){
      float ratio = baro.pressure_pa / cal::baro_ground_pressure_pa;
      if(ratio > 0.5f && ratio < 1.5f){
        baro.height_m = 44330.0f * (1.0f - powf(ratio, 0.190294957f));
        float pressureVariance = fmaxf(0.0f, cal::baro_reference_variance_pa2)
                               + fmaxf(0.0f, cal::baro_measurement_variance_pa2);
        float dhDp = 44330.0f * 0.190294957f
                   * powf(ratio, 0.190294957f - 1.0f) / cal::baro_ground_pressure_pa;
        float calculatedSigma = fabsf(dhDp) * sqrtf(pressureVariance);
        baro.height_sigma_m = isfinite(calculatedSigma)
            ? fmaxf(0.15f, calculatedSigma) : 1.0f;
        if(!vertKf.initialized) vertKf.init(baro.height_m, 0.0f);
        else vertKf.updateBaro(baro.height_m, baro.height_sigma_m);
      } else {
        baro.valid = false;
      }
    }
  }

  /* Pilot command: disabled (no UART input in this build) */
  PilotCommand pilotCmd;
  pilotCmd.valid = false;
  pilotCmd.timestampMs = millis();
  pilotCmd.roll = 0; pilotCmd.pitch = 0; pilotCmd.yaw = 0;
  pilotCmd.throttle = 0; pilotCmd.rt = 0; pilotCmd.lt = 0; pilotCmd.buttons = 0;

  /* Arm/disarm: disabled (no command input) */

  /* Run control */
  float motorOut[4] = {0,0,0,0};
  computeControl(pilotCmd, dt, motorOut);

  /* Telemetry */
  static uint32_t lastTelemetryMs = 0;
  uint32_t nowMs = millis();
  if(nowMs - lastTelemetryMs >= cal::telemetry_interval_ms){
    lastTelemetryMs = nowMs;
    /* Build JSON telemetry */
    StaticJsonDocument<512> doc;
    doc["type"] = "telemetry";
    doc["roll"] = attitudeRollDeg;
    doc["pitch"] = attitudePitchDeg;
    doc["yaw"] = attitudeYawDeg;
    doc["roll_rate"] = attitudeRollRateDps;
    doc["pitch_rate"] = attitudePitchRateDps;
    doc["yaw_rate"] = attitudeYawRateDps;
    doc["altitude"] = 0.0f;
    doc["vert_speed"] = 0.0f;
    doc["armed"] = armed;
    doc["loop_rate"] = 500.0f;
    JsonArray motors = doc.createNestedArray("motor");
    motors.add(motorOut[0]); motors.add(motorOut[1]);
    motors.add(motorOut[2]); motors.add(motorOut[3]);
    String msg;
    serializeJson(doc, msg);
    if(wsClient >= 0) wsSendText(wsClient, msg.c_str());
  }

  /* Handle HTTP connections */
  if(httpSock >= 0){
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
    setsockopt(httpSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int clientSock = accept(httpSock, NULL, NULL);
    if(clientSock >= 0){ handleHttpClient(clientSock); }
  }

  /* Handle WebSocket connections */
  if(wsSock >= 0){
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
    setsockopt(wsSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if(wsClient < 0){
      int newClient = accept(wsSock, NULL, NULL);
      if(newClient >= 0){
        setsockopt(newClient, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        wsClient = newClient;
      }
    }
  }

  /* Handle WebSocket client data */
  if(wsClient >= 0){
    char buf[512];
    int len = wsRecvText(wsClient, buf, sizeof(buf));
    if(len > 0){
      buf[len] = 0;
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, buf);
      if(!err){
        const char* typeStr = doc["type"] | "";
        if(strcmp(typeStr, "pid_set") == 0){
          const char* axis = doc["axis"] | "";
          float kp = doc["kp"] | 0.0f;
          float ki = doc["ki"] | 0.0f;
          float kd = doc["kd"] | 0.0f;
          int idx = pidNameToAxis(axis);
          /* PID setters not yet wired to live control path (research-only) */
          StaticJsonDocument<128> ack;
          ack["type"] = "pid_ack";
          ack["axis"] = axis;
          String ackMsg;
          serializeJson(ack, ackMsg);
          wsSendText(wsClient, ackMsg.c_str());
        } else if(strcmp(typeStr, "pid_get") == 0){
          StaticJsonDocument<512> docOut;
          docOut["type"] = "pid_values";
          JsonObject pids = docOut.createNestedObject("pids");
          for(int i=0; i<7; i++){
            JsonObject a = pids.createNestedObject(pidAxisToName(i));
            a["kp"] = 0.0f; a["ki"] = 0.0f; a["kd"] = 0.0f;
          }
          String msg;
          serializeJson(docOut, msg);
          wsSendText(wsClient, msg.c_str());
        }
      }
    } else if(len == -2){
      close(wsClient); wsClient = -1;
    } else if(len < 0){
      close(wsClient); wsClient = -1;
    }
  }

  yield();
}