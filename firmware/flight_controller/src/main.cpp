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
 * ESC outputs: GPIO 12/11/14/13, 50 Hz standard PWM, 1000-2000 us.
 *
 * Architecture:
 *   500 Hz control loop (IMU, MAG, Baro, estimation, PID, mixing)
 *   Wireless PID tuning via WebSocket web server
 *   No auto-arm on startup/reconnect
 *   Xbox Series X|S model 1914 connects directly using BLE.
 *   Fresh controller data is required continuously; loss disarms.
 *   Arm: hold D-pad Up + LB + RB with throttle low for 1 second.
 *   Disarm: D-pad Down or B.
 *
 * Calibration (Nov 7 completed-session fit, software candidate only):
 *   MPU6050: ±8 g / ±500 deg/s, software bias + matrix applied once
 *   QMC5883P: ±8 G = 37.5 counts/µT, hard/soft-iron applied once
 *   BMP280: no ground pressure reference; altitude control unavailable
 *   LiDAR: excluded (not wired)
 *   Magnetometer heading fusion: DISABLED until axes/handedness verified
 *
 * PID values: carried-forward simulation values only.
 *   Rate loops output N·m; mixer authority is still an unvalidated assumption.
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
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <NimBLEDevice.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include "driver/ledc.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha1.h"

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

/* ---- Gyroscope (deg/s) ---- */
/* Raw conversion: counts / 65.5 */
/* Apply once: calibrated = raw - bias */
inline const float gyro_bias_dps[3] = {
    -11.04462f,  6.99774f, -1.10236f
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

/* ---- Barometer (Pa) ---- */
/* No ground pressure reference recorded. Altitude control unavailable. */
inline const float baro_measurement_variance_pa2 = 10.81f;

/* ---- Gravity ---- */
inline const float gravity_mps2 = 9.80665f;

/* ---- PID gains (carried-forward simulation values, research-only) ---- */
/* Rate loops output N·m; converted to normalized motor commands. */
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
inline const uint8_t btn_a    = 1u << 6;
inline const uint8_t btn_b    = 1u << 7;
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

/* BMP280 calibration registers (24 bytes starting at 0x88) */
static constexpr uint8_t kBmpRegCalibStart   = 0x88;

/* BMP280 calibration coefficients */
static uint16_t bmpDigT1;
static int16_t bmpDigT2, bmpDigT3;
static uint16_t bmpDigP1;
static int16_t bmpDigP2, bmpDigP3, bmpDigP4, bmpDigP5;
static int16_t bmpDigP6, bmpDigP7, bmpDigP8, bmpDigP9;
static bool bmpCalLoaded = false;
static int32_t bmpTFine = 0;

/* Compensate BMP280 raw readings to true temperature (°C) and pressure (Pa) */
static bool bmpCompensate(uint32_t rawTemp, uint32_t rawPress, float& tC, float& pPa){
  if(!bmpCalLoaded) return false;
  /* Temperature compensation */
  const int32_t tempVar1 = ((((int32_t)(rawTemp >> 3)
      - ((int32_t)bmpDigT1 << 1)) * (int32_t)bmpDigT2) >> 11);
  const int32_t tempDelta = (int32_t)(rawTemp >> 4) - (int32_t)bmpDigT1;
  const int32_t tempVar2 = (((tempDelta * tempDelta) >> 12)
      * (int32_t)bmpDigT3) >> 14;
  bmpTFine = tempVar1 + tempVar2;
  tC = (bmpTFine * 5 + 128) >> 8;
  tC /= 100.0f;
  /* Pressure compensation */
  int64_t pressureVar1 = (int64_t)bmpTFine - 128000;
  int64_t pressureVar2 = pressureVar1 * pressureVar1 * (int64_t)bmpDigP6;
  pressureVar2 += (pressureVar1 * (int64_t)bmpDigP5) * 131072LL;
  pressureVar2 += (int64_t)bmpDigP4 * 34359738368LL;
  pressureVar1 = ((pressureVar1 * pressureVar1 * (int64_t)bmpDigP3) >> 8)
      + (pressureVar1 * (int64_t)bmpDigP2) * 4096LL;
  pressureVar1 = (((((int64_t)1) << 47) + pressureVar1)
      * (int64_t)bmpDigP1) >> 33;
  if(pressureVar1 == 0) return false;
  int64_t p = (int64_t)1048576 - (int64_t)rawPress;
  p = (((p << 31) - pressureVar2) * 3125) / pressureVar1;
  pressureVar1 = ((int64_t)bmpDigP9 * (p >> 13) * (p >> 13)) >> 25;
  pressureVar2 = ((int64_t)bmpDigP8 * p) >> 19;
  p = ((p + pressureVar1 + pressureVar2) >> 8)
      + (int64_t)bmpDigP7 * 16LL;
  pPa = p / 256.0f;
  return isfinite(tC) && isfinite(pPa) && pPa > 0.0f;
}

static constexpr uint8_t kMotorPins[4] = {12, 11, 14, 13};
static constexpr uint32_t kEscPwmFrequencyHz = 50;
static constexpr uint8_t kEscPwmResolutionBits = 14;
static constexpr uint16_t kEscMinPulseUs = 1000;
static constexpr uint16_t kEscMaxPulseUs = 2000;
static constexpr uint16_t kEscBenchPulseUs = 1100;
static constexpr uint32_t kEscBenchPulseDurationMs = 1000;
static constexpr uint32_t kMpuMaxSampleAgeUs = 4000;

/* ============================================================================
   GLOBAL STATE
   ============================================================================ */

static int httpSock = -1;
static int wsSock   = -1;
static int wsClient = -1;
static bool wsHandshakeComplete = false;
static char wsHandshakeBuffer[512] = {};
static size_t wsHandshakeLength = 0;
static uint8_t wsRxBuffer[1024] = {};
static size_t wsRxLength = 0;
static bool motorOutputReady = false;
static bool motorOutputFaulted = false;
static int8_t benchTestMotor = -1;
static uint32_t benchTestUntilMs = 0;
static char serialCommandBuffer[32] = {};
static size_t serialCommandLength = 0;
static TaskHandle_t xboxTaskHandle = nullptr;
static portMUX_TYPE pilotCommandMux = portMUX_INITIALIZER_UNLOCKED;

static void clearWebSocketClient(){
  if(wsClient >= 0) close(wsClient);
  wsClient = -1;
  wsHandshakeComplete = false;
  wsHandshakeLength = 0;
  wsRxLength = 0;
}

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

ImuSample imu;
MagSample mag;
BaroSample baro;

static bool imuSampleIsRecent(){
  return imu.valid && imu.timestamp_us != 0u
      && uint32_t(micros() - imu.timestamp_us) <= kMpuMaxSampleAgeUs;
}

/* Control state */
static bool armed = false, failsafe = false;
static uint32_t armTimeMs = 0, disarmTimeMs = 0;
static float hoverCollective = 0.0f;
static float ltReduceState = 0.0f, boostState = 0.0f;
static bool verticalCollectiveSaturatedHigh = false;
static bool verticalCollectiveSaturatedLow = false;
static float altitudeRefM = 0.0f;
static bool altitudeRefInitialized = false;

/* Motor outputs */
static float motorLastCmd[4] = {0,0,0,0};

/* ============================================================================
   PID CONTROLLERS (simulation-only values, research-only)
   ============================================================================ */

struct Pid {
  float kp=0, ki=0, kd=0, integrator=0, prevMeas=0;
  float integratorLimit=0, outputLimit=0, derivFilt=0, derivativeCutoffHz=20.0f;
  bool initialized=false;

  void reset(){
    integrator=0; prevMeas=0; initialized=false; derivFilt=0;
  }
  float update(float sp, float meas, float dt, bool allowIntegration=true){
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
    // A caller can freeze integration when a downstream actuator clamp is
    // active, preventing windup outside this PID's own output limit.
    float ci = integrator + (allowIntegration ? err*dt : 0.0f);
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
static float attitudeRollDeg = 0, attitudePitchDeg = 0, attitudeYawDeg = 0;
static float attitudeRollRateDps = 0, attitudePitchRateDps = 0, attitudeYawRateDps = 0;
static bool attitudeInitialized = false;
static const float attitudeKpRoll = 0.5f, attitudeKpPitch = 0.5f, attitudeKpYaw = 0.1f;

static void attitudeInit(const float accel[3]) {
  float roll = atan2f(accel[1], accel[2]) * 57.2957795f;
  float pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 57.2957795f;
  float cr = cosf(roll*0.0174532925f), sr = sinf(roll*0.0174532925f);
  float cp = cosf(pitch*0.0174532925f), sp = sinf(pitch*0.0174532925f);
  attitudeQ.w = cr*cp;
  attitudeQ.x = sr*cp;
  attitudeQ.y = cr*sp;
  attitudeQ.z = -sr*sp;
  attitudeQ.normalize();
  attitudeRollDeg = roll; attitudePitchDeg = pitch; attitudeYawDeg = 0;
  attitudeInitialized = true;
}

static void attitudeUpdate(const float gyro[3], const float accel[3],
                            const float mag[3], bool magValid, float dt) {
  if(!attitudeInitialized || !isfinite(dt) || dt<=0.0f || dt>0.02f) return;
  float gx = gyro[0];
  float gy = gyro[1];
  float gz = gyro[2];
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
  float accelNoise=1.5f, baroNoise=0.0025f;
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

static XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
static PilotCommand latestPilotCommand;
static uint32_t armGestureStartedMs = 0;
static bool armGestureActive = false;
static bool armGestureLatched = false;

static float normalizeXboxAxis(uint16_t raw, bool invert){
  float value = ((float)raw / (float)XboxControllerNotificationParser::maxJoy)
              * 2.0f - 1.0f;
  if(invert) value = -value;
  const float deadzone = 0.06f;
  const float magnitude = fabsf(value);
  if(magnitude <= deadzone) return 0.0f;
  value = copysignf((magnitude - deadzone) / (1.0f - deadzone), value);
  return fmaxf(-1.0f, fminf(1.0f, value));
}

static void xboxControllerTask(void*){
  xboxController.begin();
  // The library's Core scanner does not enable active scanning by default.
  // Xbox discovery also matches the HID service UUID, which may be returned
  // in the scan response rather than the initial advertisement.
  NimBLEDevice::getScan()->setActiveScan(true);
  Serial.println("Xbox BLE active scan started; hold the controller pair button.");
  bool wasFresh = false;
  uint32_t lastStatusPrintMs = 0;
  for(;;){
    xboxController.onLoop();

    PilotCommand sample;
    const uint32_t receivedAt = (uint32_t)xboxController.getReceiveNotificationAt();
    const bool connected = xboxController.isConnected()
        && !xboxController.isWaitingForFirstNotification();
    const bool fresh = connected && receivedAt != 0u
        && uint32_t(millis() - receivedAt) <= 150u;
    if(fresh){
      const auto& pad = xboxController.xboxNotif;
      sample.valid = true;
      sample.timestampMs = receivedAt;
      sample.roll = normalizeXboxAxis(pad.joyLHori, false);
      sample.pitch = normalizeXboxAxis(pad.joyLVert, true);
      sample.yaw = normalizeXboxAxis(pad.joyRHori, false);
      sample.throttle = normalizeXboxAxis(pad.joyRVert, true);
      sample.lt = (float)pad.trigLT / XboxControllerNotificationParser::maxTrig;
      sample.rt = (float)pad.trigRT / XboxControllerNotificationParser::maxTrig;
      if(pad.btnLB) sample.buttons |= cal::btn_lb;
      if(pad.btnRB) sample.buttons |= cal::btn_rb;
      if(pad.btnDirUp) sample.buttons |= cal::btn_north;
      if(pad.btnDirRight) sample.buttons |= cal::btn_east;
      if(pad.btnDirDown) sample.buttons |= cal::btn_south;
      if(pad.btnDirLeft) sample.buttons |= cal::btn_west;
      if(pad.btnA) sample.buttons |= cal::btn_a;
      if(pad.btnB) sample.buttons |= cal::btn_b;
    }

    portENTER_CRITICAL(&pilotCommandMux);
    latestPilotCommand = sample;
    portEXIT_CRITICAL(&pilotCommandMux);

    if(fresh && !wasFresh){
      Serial.printf("Xbox connected: %s, battery %u%%\n",
                    xboxController.buildDeviceAddressStr().c_str(),
                    xboxController.battery);
    } else if(!fresh && wasFresh){
      Serial.println("Xbox input lost/stale; flight control will disarm.");
    }
    if(!fresh && uint32_t(millis() - lastStatusPrintMs) >= 2000u){
      lastStatusPrintMs = millis();
      Serial.printf("Xbox BLE status: link=%s, waiting_first_report=%s, failed_connections=%u\n",
          xboxController.isConnected() ? "connected" : "searching",
          xboxController.isWaitingForFirstNotification() ? "yes" : "no",
          xboxController.getCountFailedConnection());
    }
    wasFresh = fresh;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static PilotCommand readPilotCommand(){
  PilotCommand sample;
  portENTER_CRITICAL(&pilotCommandMux);
  sample = latestPilotCommand;
  portEXIT_CRITICAL(&pilotCommandMux);
  if(sample.valid && uint32_t(millis() - sample.timestampMs) > 150u){
    sample.valid = false;
  }
  return sample;
}

static void resetControlState(){
  pidRollAngle.reset(); pidPitchAngle.reset(); pidRollRate.reset();
  pidPitchRate.reset(); pidYawRate.reset();
  pidAltitude.reset(); pidVertSpeed.reset();
  altitudeRefInitialized = false;
  ltReduceState = 0.0f;
  boostState = 0.0f;
  for(float& motorCommand : motorLastCmd){
    motorCommand = cal::motor_idle_fraction;
  }
}

static void disarmFlight(bool controlFault){
  const bool stateChanged = armed || failsafe != controlFault;
  armed = false;
  failsafe = controlFault;
  if(stateChanged) resetControlState();
}

static void updateArmingFromXbox(const PilotCommand& cmd){
  const uint8_t armChord = cal::arm_buttons | cal::btn_north;
  const bool chordHeld = (cmd.buttons & armChord) == armChord;
  if(!cmd.valid){
    if(armed){
      disarmFlight(true);
      disarmTimeMs = millis();
      Serial.println("DISARMED: Xbox link/input timeout.");
    }
    armGestureActive = false;
    return;
  }
  if((cmd.buttons & (cal::btn_south | cal::btn_b)) != 0u){
    if(armed){
      disarmFlight(false);
      disarmTimeMs = millis();
      Serial.println("DISARMED by Xbox D-pad Down / B.");
    }
    armGestureActive = false;
    armGestureLatched = true;
    return;
  }
  if(!chordHeld) armGestureLatched = false;

  const bool safeThrottle = cmd.throttle <= -0.95f
      && cmd.rt <= 0.05f && cmd.lt <= 0.05f;
  const bool armReady = motorOutputReady && !motorOutputFaulted
      && imuSampleIsRecent()
      && attitudeInitialized && calibrationLoaded && safeThrottle;
  if(!armed && !armGestureLatched && chordHeld && armReady){
    if(!armGestureActive){
      armGestureActive = true;
      armGestureStartedMs = millis();
    } else if(uint32_t(millis() - armGestureStartedMs) >= 1000u){
      armed = true;
      failsafe = false;
      armTimeMs = millis();
      resetControlState();
      armGestureLatched = true;
      Serial.println("ARMED by Xbox hold gesture.");
    }
  } else if(!chordHeld || !armReady){
    armGestureActive = false;
  }
}

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

static bool setSocketNonBlocking(int sock){
  const int flags = fcntl(sock, F_GETFL, 0);
  return flags >= 0 && fcntl(sock, F_SETFL, flags | O_NONBLOCK) == 0;
}

static bool setSocketBlocking(int sock){
  const int flags = fcntl(sock, F_GETFL, 0);
  return flags >= 0 && fcntl(sock, F_SETFL, flags & ~O_NONBLOCK) == 0;
}

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
  if(!setSocketNonBlocking(sock)){ close(sock); return -1; }
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

static int wsPerformHandshake(int sock){
  if(wsHandshakeLength >= sizeof(wsHandshakeBuffer) - 1u) return -1;
  const int received = recv(sock, wsHandshakeBuffer + wsHandshakeLength,
      sizeof(wsHandshakeBuffer) - wsHandshakeLength - 1u, 0);
  if(received == 0) return -1;
  if(received < 0){
    if(errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  wsHandshakeLength += (size_t)received;
  wsHandshakeBuffer[wsHandshakeLength] = '\0';
  if(strstr(wsHandshakeBuffer, "\r\n\r\n") == nullptr) return 0;

  char clientKey[128] = {};
  const char* line = wsHandshakeBuffer;
  while(*line != '\0'){
    const char* lineEnd = strstr(line, "\r\n");
    if(lineEnd == nullptr) break;
    const size_t lineLength = (size_t)(lineEnd - line);
    static constexpr char keyHeader[] = "Sec-WebSocket-Key:";
    if(lineLength >= sizeof(keyHeader) - 1u
        && strncasecmp(line, keyHeader, sizeof(keyHeader) - 1u) == 0){
      const char* value = line + sizeof(keyHeader) - 1u;
      const char* valueEnd = lineEnd;
      while(value < valueEnd && (*value == ' ' || *value == '\t')) ++value;
      while(valueEnd > value && (valueEnd[-1] == ' ' || valueEnd[-1] == '\t')) --valueEnd;
      const size_t keyLength = (size_t)(valueEnd - value);
      if(keyLength == 0u || keyLength >= sizeof(clientKey)) return -1;
      memcpy(clientKey, value, keyLength);
      break;
    }
    line = lineEnd + 2;
  }
  if(clientKey[0] == '\0') return -1;

  static constexpr char websocketGuid[] =
      "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char hashInput[sizeof(clientKey) + sizeof(websocketGuid)] = {};
  const int hashInputLength = snprintf(hashInput, sizeof(hashInput), "%s%s",
                                       clientKey, websocketGuid);
  if(hashInputLength <= 0 || (size_t)hashInputLength >= sizeof(hashInput)) return -1;
  unsigned char digest[20] = {};
  if(mbedtls_sha1((const unsigned char*)hashInput,
                  (size_t)hashInputLength, digest) != 0){
    return -1;
  }
  unsigned char acceptKey[32] = {};
  size_t acceptLength = 0;
  if(mbedtls_base64_encode(acceptKey, sizeof(acceptKey), &acceptLength,
                           digest, sizeof(digest)) != 0
      || acceptLength >= sizeof(acceptKey)){
    return -1;
  }
  acceptKey[acceptLength] = '\0';

  char response[256] = {};
  const int responseLength = snprintf(response, sizeof(response),
      "HTTP/1.1 101 Switching Protocols\r\n"
      "Upgrade: websocket\r\n"
      "Connection: Upgrade\r\n"
      "Sec-WebSocket-Accept: %s\r\n\r\n", acceptKey);
  if(responseLength <= 0 || (size_t)responseLength >= sizeof(response)
      || !sendAll(sock, (const uint8_t*)response, (size_t)responseLength)){
    return -1;
  }
  return 1;
}

static void consumeWsFrame(size_t frameLength){
  wsRxLength -= frameLength;
  if(wsRxLength > 0u){
    memmove(wsRxBuffer, wsRxBuffer + frameLength, wsRxLength);
  }
}

static int wsExtractTextFrame(int sock, char* buf, size_t bufsz){
  if(wsRxLength < 2u) return -3;
  const uint8_t first = wsRxBuffer[0];
  const uint8_t second = wsRxBuffer[1];
  const uint8_t opcode = first & 0x0Fu;
  if((first & 0x80u) == 0u || (first & 0x70u) != 0u
      || (second & 0x80u) == 0u){
    return -1;
  }

  size_t payloadLength = second & 0x7Fu;
  size_t headerLength = 2u;
  if(payloadLength == 126u){
    if(wsRxLength < 4u) return -3;
    payloadLength = ((size_t)wsRxBuffer[2] << 8) | wsRxBuffer[3];
    headerLength = 4u;
  } else if(payloadLength == 127u){
    return -1;
  }
  if(payloadLength >= bufsz || payloadLength > 512u
      || payloadLength + headerLength + 4u > sizeof(wsRxBuffer)){
    return -1;
  }
  const size_t frameLength = headerLength + 4u + payloadLength;
  if(wsRxLength < frameLength) return -3;
  if(opcode != 0x01u && opcode != 0x08u && opcode != 0x09u){
    return -1;
  }
  if((opcode == 0x08u || opcode == 0x09u) && payloadLength > 125u){
    return -1;
  }

  const uint8_t* mask = wsRxBuffer + headerLength;
  uint8_t payload[512] = {};
  for(size_t i = 0; i < payloadLength; ++i){
    payload[i] = wsRxBuffer[headerLength + 4u + i] ^ mask[i & 0x03u];
  }
  consumeWsFrame(frameLength);
  if(opcode == 0x08u) return -2;
  if(opcode == 0x09u){
    const uint8_t pongHeader[2] = {0x8Au, (uint8_t)payloadLength};
    if(!sendAll(sock, pongHeader, sizeof(pongHeader))
        || !sendAll(sock, payload, payloadLength)) return -1;
    return 0;
  }
  memcpy(buf, payload, payloadLength);
  buf[payloadLength] = '\0';
  return (int)payloadLength;
}

static int wsRecvText(int sock, char* buf, size_t bufsz){
  int parsed = wsExtractTextFrame(sock, buf, bufsz);
  if(parsed != -3) return parsed;
  if(wsRxLength >= sizeof(wsRxBuffer)) return -1;
  const int received = recv(sock, wsRxBuffer + wsRxLength,
      sizeof(wsRxBuffer) - wsRxLength, 0);
  if(received == 0) return -2;
  if(received < 0){
    if(errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  wsRxLength += (size_t)received;
  parsed = wsExtractTextFrame(sock, buf, bufsz);
  return parsed == -3 ? 0 : parsed;
}

extern const char WEB_GUI_HTML[];

static void handleHttpClient(int clientSock){
  char req[256];
  int len = recv(clientSock, req, sizeof(req)-1, 0);
  if(len <= 0){ close(clientSock); return; }
  req[len] = 0;
  const size_t htmlLength = strlen(WEB_GUI_HTML);
  char response[160];
  const int responseLength = snprintf(response, sizeof(response),
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html; charset=utf-8\r\n"
      "Content-Length: %lu\r\n"
      "Connection: close\r\n\r\n",
      (unsigned long)htmlLength);
  if(responseLength > 0 && (size_t)responseLength < sizeof(response)){
    sendAll(clientSock, (const uint8_t*)response, (size_t)responseLength);
    sendAll(clientSock, (const uint8_t*)WEB_GUI_HTML, htmlLength);
  }
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
  const uint32_t nowUs = micros();
  if(s.timestamp_us == 0u
      || uint32_t(nowUs - s.timestamp_us) > kMpuMaxSampleAgeUs){
    s.valid = false;
  }
  Wire.setClock(400000);
  if(!mpuPresent) return false;
  uint8_t fc[2] = {};
  if(!i2cReadRegs(mpuAddress, kMpuRegFifoCountH, fc, 2)){
    return false;
  }
  uint16_t fcnt = (uint16_t(fc[0]) << 8) | fc[1];
  uint16_t completeRecords = fcnt / 14u;
  s.fifo_count = completeRecords;
  if(fcnt >= 1024u){
    s.overflows++;
    i2cWriteReg(mpuAddress, kMpuRegUserCtrl, kMpuUserCtrlFifoEn | kMpuUserCtrlFifoReset);
    s.valid = false;
    return false;
  }
  if(completeRecords == 0u){
    if(s.timestamp_us == 0u
        || uint32_t(nowUs - s.timestamp_us) > kMpuMaxSampleAgeUs)
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
  s.valid = uint32_t(micros() - s.timestamp_us) <= kMpuMaxSampleAgeUs;
  s.fresh = s.valid;
  return s.fresh;
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

  /* Read BMP280 calibration coefficients (24 bytes from 0x88) */
  uint8_t cal[24] = {};
  if(!i2cReadRegs(bmpAddress, kBmpRegCalibStart, cal, 24)) return false;
  bmpDigT1 = (uint16_t)(cal[1] << 8 | cal[0]);
  bmpDigT2 = (int16_t)(cal[3] << 8 | cal[2]);
  bmpDigT3 = (int16_t)(cal[5] << 8 | cal[4]);
  bmpDigP1 = (uint16_t)(cal[7] << 8 | cal[6]);
  bmpDigP2 = (int16_t)(cal[9] << 8 | cal[8]);
  bmpDigP3 = (int16_t)(cal[11] << 8 | cal[10]);
  bmpDigP4 = (int16_t)(cal[13] << 8 | cal[12]);
  bmpDigP5 = (int16_t)(cal[15] << 8 | cal[14]);
  bmpDigP6 = (int16_t)(cal[17] << 8 | cal[16]);
  bmpDigP7 = (int16_t)(cal[19] << 8 | cal[18]);
  bmpDigP8 = (int16_t)(cal[21] << 8 | cal[20]);
  bmpDigP9 = (int16_t)(cal[23] << 8 | cal[22]);

  /* Configure BMP280: normal mode, temp x1, pressure x1, filter off */
  if(!i2cWriteReg(bmpAddress, kBmpRegCtrlMeas, 0x27)) return false;
  if(!i2cWriteReg(bmpAddress, kBmpRegConfig, 0x00)) return false;

  bmpCalLoaded = true;
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
  /* Compensate to true temperature and pressure */
  float tC = 0.0f, pPa = 0.0f;
  if(bmpCompensate(s.raw_temperature, s.raw_pressure, tC, pPa)){
    s.temperature_c = tC;
    s.pressure_pa = pPa;
  } else {
    s.temperature_c = 0.0f;
    s.pressure_pa = 0.0f;
    s.valid = false;
    return false;
  }
  s.timestamp_us = micros();
  s.valid = true; s.fresh = true;
  return true;
}

/* ============================================================================
   CALIBRATION LOAD (single explicit call at startup)
   ============================================================================ */

static void loadCalibration(){
  using namespace cal;
  calGyroValid = true;
  calAccelValid = true;
  calMagValid = true;
  calBaroValid = bmpCalLoaded;
  calibrationLoaded = calAccelValid && calGyroValid && calMagValid;
  Serial.printf("Calibration loaded: accel=%d gyro=%d mag=%d baro=%d\n",
                calAccelValid, calGyroValid, calMagValid, calBaroValid);
}

/* ============================================================================
   ESP32-S3 ESC OUTPUTS
   ============================================================================ */

static uint32_t escPulseToDuty(uint16_t pulseUs){
  const uint32_t periodTicks = 1UL << kEscPwmResolutionBits;
  return ((uint32_t)pulseUs * kEscPwmFrequencyHz * periodTicks) / 1000000UL;
}

static bool configureMotorOutputs(){
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = (ledc_timer_bit_t)kEscPwmResolutionBits;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = kEscPwmFrequencyHz;
  timer.clk_cfg = LEDC_AUTO_CLK;
  if(ledc_timer_config(&timer) != ESP_OK) return false;

  for(uint8_t i = 0; i < 4; ++i){
    ledc_channel_config_t channel = {};
    channel.gpio_num = kMotorPins[i];
    channel.speed_mode = LEDC_LOW_SPEED_MODE;
    channel.channel = (ledc_channel_t)i;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = LEDC_TIMER_0;
    channel.duty = escPulseToDuty(kEscMinPulseUs);
    channel.hpoint = 0;
    if(ledc_channel_config(&channel) != ESP_OK) return false;
  }
  return true;
}

static bool writeEscPulse(uint8_t motor, uint16_t pulseUs){
  if(motor >= 4) return false;
  pulseUs = (uint16_t)constrain(pulseUs, kEscMinPulseUs, kEscMaxPulseUs);
  if(ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor,
                   escPulseToDuty(pulseUs)) != ESP_OK){
    return false;
  }
  return ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor)
      == ESP_OK;
}

static void pollBenchMotorTest(){
  while(Serial.available() > 0){
    char ch = (char)Serial.read();
    if(ch == '\r') continue;
    if(ch == '\n'){
      serialCommandBuffer[serialCommandLength] = '\0';
      if(strcmp(serialCommandBuffer, "STOP") == 0){
        benchTestMotor = -1;
        Serial.println("Bench motor test stopped; all ESCs at 1000 us.");
      } else if(strncmp(serialCommandBuffer, "MTEST ", 6) == 0){
        char* end = nullptr;
        long motorNumber = strtol(serialCommandBuffer + 6, &end, 10);
        if(!motorOutputReady || motorOutputFaulted){
          Serial.println("MTEST rejected: ESC PWM setup failed.");
        } else if(armed){
          Serial.println("MTEST rejected: flight controller is armed.");
        } else if(end == serialCommandBuffer + 6 || *end != '\0'
                  || motorNumber < 1 || motorNumber > 4){
          Serial.println("Use MTEST 1, MTEST 2, MTEST 3, MTEST 4, or STOP.");
        } else {
          benchTestMotor = (int8_t)(motorNumber - 1);
          benchTestUntilMs = millis() + kEscBenchPulseDurationMs;
          Serial.printf("Motor %ld: %u us for %lu ms. REMOVE ALL PROPELLERS.\n",
                        motorNumber, kEscBenchPulseUs,
                        (unsigned long)kEscBenchPulseDurationMs);
        }
      } else if(serialCommandLength != 0){
        Serial.println("Bench command: MTEST 1..4 or STOP.");
      }
      serialCommandLength = 0;
      serialCommandBuffer[0] = '\0';
    } else if(serialCommandLength < sizeof(serialCommandBuffer) - 1){
      serialCommandBuffer[serialCommandLength++] = ch;
    } else {
      serialCommandLength = 0;
      serialCommandBuffer[0] = '\0';
    }
  }

  if(benchTestMotor >= 0
      && (int32_t)(millis() - benchTestUntilMs) >= 0){
    benchTestMotor = -1;
    Serial.println("Bench pulse complete; all ESCs at 1000 us.");
  }
}

static void writeMotorOutputs(const float command[4]){
  if(!motorOutputReady) return;
  if(motorOutputFaulted){
    for(uint8_t i = 0; i < 4; ++i){
      writeEscPulse(i, kEscMinPulseUs);
    }
    return;
  }
  const bool benchActive = !armed && benchTestMotor >= 0
      && (int32_t)(millis() - benchTestUntilMs) < 0;
  for(uint8_t i = 0; i < 4; ++i){
    uint16_t pulseUs = kEscMinPulseUs;
    if(benchActive && i == (uint8_t)benchTestMotor){
      pulseUs = kEscBenchPulseUs;
    } else if(armed && !failsafe && isfinite(command[i])){
      const float fraction = fmaxf(0.0f, fminf(1.0f, command[i]));
      pulseUs = (uint16_t)(kEscMinPulseUs
          + fraction * (kEscMaxPulseUs - kEscMinPulseUs) + 0.5f);
    }
    if(!writeEscPulse(i, pulseUs)){
      motorOutputFaulted = true;
      disarmFlight(true);
      benchTestMotor = -1;
      Serial.printf("ESC PWM write failed on motor %u; latched disarmed, sending minimum pulses.\n",
                    (unsigned)(i + 1));
      for(uint8_t safeMotor = 0; safeMotor < 4; ++safeMotor){
        writeEscPulse(safeMotor, kEscMinPulseUs);
      }
      return;
    }
  }
}

/* ============================================================================
   MOTOR MIXER (X-frame) with slew-rate cap
   ============================================================================ */

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
    case 'arm_ack': break;
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
function requestArm(arm){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: 'arm', arm: arm }));
}
function emergencyDisarm(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  if(confirm('EMERGENCY DISARM - All motors will stop immediately. Continue?')){
    ws.send(JSON.stringify({ type: 'arm', arm: false }));
  }
}
connect();
setInterval(() => { if(!ws && !reconnectTimer) connect(); }, 3000);
</script>
</body>
</html>
)rawliteral";

/* ============================================================================
   PID getters/setters for WebSocket tuner
   ============================================================================ */

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
  if(!armed){
    for(int i = 0; i < 4; i++) out[i] = 0.0f;
    return;
  }
  const bool imuOk = calibrationLoaded && calAccelValid && calGyroValid
      && imuSampleIsRecent() && attitudeInitialized;
  const bool pilotOk = cmd.valid
      && uint32_t(millis() - cmd.timestampMs) < 250u;
  if(!imuOk || !pilotOk || !motorOutputReady || motorOutputFaulted
      || !isfinite(dt) || dt <= 0.0f || dt > 0.01f){
    disarmFlight(true);
    for(int i=0; i<4; i++) out[i] = 0.0f;
    return;
  }
  failsafe = false;

  const float triggerAlpha = fminf(1.0f, dt / 0.10f);
  boostState += triggerAlpha * (cmd.rt - boostState);
  ltReduceState += triggerAlpha * (cmd.lt - ltReduceState);

  /* Initialize vertKf from first baro reading if not yet done */
  if(baro.valid && baro.fresh && !vertKf.initialized){
    vertKf.init(0.0f, 0.0f);
  }

  /* Attitude */
  float attRoll = attitudeRollDeg, attPitch = attitudePitchDeg;
  float attRollRate = attitudeRollRateDps, attPitchRate = attitudePitchRateDps, attYawRate = attitudeYawRateDps;

  /* Angle P controllers -> rate setpoints (deg) */
  const float deg2rad = 0.0174532925f;
  float rollSpDeg = cmd.roll * cal::max_roll_deg;
  float pitchSpDeg = cmd.pitch * cal::max_pitch_deg;
  float rollRateSp = pidRollAngle.update(rollSpDeg, attRoll, dt);
  float pitchRateSp = pidPitchAngle.update(pitchSpDeg, attPitch, dt);

  /* Inner rate PIDs -> torque (N·m, model-only) */
  float rollTorque = pidRollRate.update(rollRateSp, attRollRate, dt);
  float pitchTorque = pidPitchRate.update(pitchRateSp, attPitchRate, dt);
  float yawTorque = pidYawRate.update(cmd.yaw * cal::max_yaw_rate_dps, attYawRate, dt);

  /* Vertical speed PID -> vertical acceleration */
  float vs = cmd.throttle * cal::max_vertical_speed_mps;
  float boost = fmaxf(0.0f, fminf(1.0f, boostState));
  float tt = (boost - ltReduceState) * 1.0f;
  vs += tt;
  vs = fmaxf(-cal::max_descent_mps, fminf(cal::max_climb_mps, vs));
  const float verticalSpeedError = vs - vertKf.velocityMps;
  const bool allowVerticalIntegration = !(
      (verticalCollectiveSaturatedHigh && verticalSpeedError > 0.0f)
      || (verticalCollectiveSaturatedLow && verticalSpeedError < 0.0f));
  float accelCmd = pidVertSpeed.update(
      vs, vertKf.velocityMps, dt, allowVerticalIntegration);

  /* Altitude PID -> altitude hold correction (armed + throttle near center) */
  bool altitudeHold = fabsf(cmd.throttle) < 0.1f && armed;
  if(altitudeHold && altitudeRefInitialized && vertKf.initialized){
    float altCorr = pidAltitude.update(altitudeRefM, vertKf.height_m, dt);
    accelCmd += altCorr;
  } else if(!altitudeRefInitialized && vertKf.initialized){
    altitudeRefM = vertKf.height_m;
    altitudeRefInitialized = true;
  } else if(altitudeRefInitialized && vertKf.initialized){
    altitudeRefM += vs * dt;
  }

  /* Collective thrust from vertical acceleration */
  float cr = cosf(attRoll * deg2rad), cp = cosf(attPitch * deg2rad);
  float tf = fmaxf(0.5f, cr * cp);
  float ceiling = cal::motor_normal_ceiling
                + (cal::motor_boost_ceiling - cal::motor_normal_ceiling) * boost;
  const float requestedCollective = (hoverCollective + accelCmd * 0.05f) / tf;
  verticalCollectiveSaturatedHigh = requestedCollective > ceiling;
  verticalCollectiveSaturatedLow = requestedCollective < cal::motor_idle_fraction;
  const float coll = fmaxf(cal::motor_idle_fraction,
                           fminf(ceiling, requestedCollective));

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
  Serial.println("ESC PWM: GPIO 12/11/14/13, 50 Hz, 1000-2000 us");
  Serial.println("Flight arm: Xbox BLE gesture only; WiFi arming disabled");
  Serial.println();

  /* Load calibration once at startup */
  loadCalibration();

  /* Initialize PIDs with Nov 7 calibration gains (research/simulation values) */
  pidRollAngle.kp = cal::pid_roll_angle[0];  pidRollAngle.ki = cal::pid_roll_angle[1];  pidRollAngle.kd = cal::pid_roll_angle[2];
  pidRollAngle.integratorLimit = 0.5f; pidRollAngle.outputLimit = cal::max_roll_deg; pidRollAngle.reset();
  pidPitchAngle.kp = cal::pid_pitch_angle[0]; pidPitchAngle.ki = cal::pid_pitch_angle[1]; pidPitchAngle.kd = cal::pid_pitch_angle[2];
  pidPitchAngle.integratorLimit = 0.5f; pidPitchAngle.outputLimit = cal::max_pitch_deg; pidPitchAngle.reset();
  pidRollRate.kp = cal::pid_roll_rate[0];    pidRollRate.ki = cal::pid_roll_rate[1];    pidRollRate.kd = cal::pid_roll_rate[2];
  pidRollRate.integratorLimit = 0.5f; pidRollRate.outputLimit = 0.75f; pidRollRate.reset();
  pidPitchRate.kp = cal::pid_pitch_rate[0];  pidPitchRate.ki = cal::pid_pitch_rate[1];  pidPitchRate.kd = cal::pid_pitch_rate[2];
  pidPitchRate.integratorLimit = 0.5f; pidPitchRate.outputLimit = 0.75f; pidPitchRate.reset();
  pidYawRate.kp = cal::pid_yaw_rate[0];      pidYawRate.ki = cal::pid_yaw_rate[1];      pidYawRate.kd = cal::pid_yaw_rate[2];
  pidYawRate.integratorLimit = 0.5f; pidYawRate.outputLimit = 0.07f; pidYawRate.reset();
  pidAltitude.kp = cal::pid_altitude[0];     pidAltitude.ki = cal::pid_altitude[1];     pidAltitude.kd = cal::pid_altitude[2];
  pidAltitude.integratorLimit = 0.5f; pidAltitude.outputLimit = cal::max_vertical_speed_mps; pidAltitude.reset();
  pidVertSpeed.kp = cal::pid_vert_speed[0];  pidVertSpeed.ki = cal::pid_vert_speed[1];  pidVertSpeed.kd = cal::pid_vert_speed[2];
  pidVertSpeed.integratorLimit = 0.5f; pidVertSpeed.outputLimit = 5.0f; pidVertSpeed.reset();

  /* Model median only (about 45%); actual hover command has not been measured.
     The selected 35% normal ceiling therefore requires RT boost for takeoff. */
  hoverCollective = 0.44955984f;

  /* Keep ESC signals low during setup, then send the standard minimum pulse. */
  for(uint8_t p : kMotorPins){
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  motorOutputReady = configureMotorOutputs();
  Serial.printf("ESC PWM initialization: %s\n",
                motorOutputReady ? "READY" : "FAILED - outputs remain unavailable");
  Serial.println("Props-off bench commands: MTEST 1..4 (1100 us, 1 second), STOP");

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

  BaseType_t xboxTaskResult = xTaskCreatePinnedToCore(
      xboxControllerTask, "xbox_ble", 8192, nullptr, 1,
      &xboxTaskHandle, 0);
  Serial.printf("Xbox BLE task: %s\n",
                xboxTaskResult == pdPASS ? "STARTED" : "FAILED");

  Serial.println("Flight controller initialized.");
  Serial.printf("WiFi AP: %s / %s\n", cal::wifi_ssid, cal::wifi_password);
  Serial.printf("Web GUI: http://%s\n", ipString);
  Serial.printf("WebSocket: ws://%s:%d\n", ipString, cal::ws_port);
  Serial.println("========================================");
}

void loop(){
  static uint32_t previousLoopUs = 0;
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
  bool imuOk = readMpuFifo(imu) && imuSampleIsRecent();
  if(!imuSampleIsRecent()) attitudeInitialized = false;
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
    baro.timestamp_us = micros();
    if(!vertKf.initialized){
      vertKf.init(0.0f, 0.0f);
    }
    vertKf.updateBaro(baro.height_m, baro.height_sigma_m);
  }

  /* Read the latest fresh Xbox BLE command; stale input invalidates arming. */
  PilotCommand pilotCmd = readPilotCommand();
  updateArmingFromXbox(pilotCmd);
  static uint32_t lastXboxDebugMs = 0;
  if(!armed && pilotCmd.valid
      && uint32_t(millis() - lastXboxDebugMs) >= 200u){
    lastXboxDebugMs = millis();
    Serial.printf("XBOX roll=%+.2f pitch=%+.2f yaw=%+.2f vertical=%+.2f LT=%.2f RT=%.2f\n",
                  pilotCmd.roll, pilotCmd.pitch, pilotCmd.yaw,
                  pilotCmd.throttle, pilotCmd.lt, pilotCmd.rt);
  }
  pollBenchMotorTest();

  /* Run control */
  float motorOut[4] = {0,0,0,0};
  computeControl(pilotCmd, dt, motorOut);
  writeMotorOutputs(motorOut);

  /* Telemetry */
  static uint32_t lastTelemetryMs = 0;
  uint32_t nowMs = millis();
  if(nowMs - lastTelemetryMs >= cal::telemetry_interval_ms){
    lastTelemetryMs = nowMs;
    StaticJsonDocument<512> doc;
    doc["type"] = "telemetry";
    doc["roll"] = attitudeRollDeg;
    doc["pitch"] = attitudePitchDeg;
    doc["yaw"] = attitudeYawDeg;
    doc["roll_rate"] = attitudeRollRateDps;
    doc["pitch_rate"] = attitudePitchRateDps;
    doc["yaw_rate"] = attitudeYawRateDps;
    doc["altitude"] = vertKf.initialized ? vertKf.height_m : 0.0f;
    doc["vert_speed"] = vertKf.initialized ? vertKf.velocityMps : 0.0f;
    doc["armed"] = armed;
    doc["loop_rate"] = 500.0f;
    JsonArray motors = doc.createNestedArray("motor");
    motors.add(motorOut[0]); motors.add(motorOut[1]);
    motors.add(motorOut[2]); motors.add(motorOut[3]);
    String msg;
    serializeJson(doc, msg);
    if(!armed && benchTestMotor < 0 && wsClient >= 0 && wsHandshakeComplete){
      if(!wsSendText(wsClient, msg.c_str())) clearWebSocketClient();
    }
  }

  /* HTTP clients can block, so accept and serve them only when idle. */
  if(!armed && benchTestMotor < 0 && httpSock >= 0){
    int clientSock = accept(httpSock, NULL, NULL);
    if(clientSock >= 0){
      if(setSocketBlocking(clientSock)){
        struct timeval clientTimeout = {0, 100000};
        setsockopt(clientSock, SOL_SOCKET, SO_RCVTIMEO,
                   &clientTimeout, sizeof(clientTimeout));
        setsockopt(clientSock, SOL_SOCKET, SO_SNDTIMEO,
                   &clientTimeout, sizeof(clientTimeout));
        handleHttpClient(clientSock);
      } else {
        close(clientSock);
      }
    }
  }

  if(!armed && benchTestMotor < 0 && wsSock >= 0){
    if(wsClient < 0){
      int newClient = accept(wsSock, NULL, NULL);
      if(newClient >= 0 && setSocketNonBlocking(newClient)){
        wsClient = newClient;
        wsHandshakeComplete = false;
        wsHandshakeLength = 0;
        wsRxLength = 0;
      } else if(newClient >= 0){
        close(newClient);
      }
    }
  }

  if(benchTestMotor < 0 && wsClient >= 0 && !wsHandshakeComplete){
    const int handshakeResult = wsPerformHandshake(wsClient);
    if(handshakeResult > 0){
      wsHandshakeComplete = true;
    } else if(handshakeResult < 0){
      clearWebSocketClient();
    }
  }

  /* Established WebSocket reads are nonblocking, preserving emergency disarm. */
  if(benchTestMotor < 0 && wsClient >= 0 && wsHandshakeComplete){
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
          bool applied = false;
          const bool gainsValid = !armed && isfinite(kp) && isfinite(ki)
              && isfinite(kd);
          if(gainsValid && idx == 0){ pidRollAngle.kp=kp; pidRollAngle.ki=ki; pidRollAngle.kd=kd; pidRollAngle.reset(); applied=true; }
          else if(gainsValid && idx == 1){ pidPitchAngle.kp=kp; pidPitchAngle.ki=ki; pidPitchAngle.kd=kd; pidPitchAngle.reset(); applied=true; }
          else if(gainsValid && idx == 2){ pidRollRate.kp=kp; pidRollRate.ki=ki; pidRollRate.kd=kd; pidRollRate.reset(); applied=true; }
          else if(gainsValid && idx == 3){ pidPitchRate.kp=kp; pidPitchRate.ki=ki; pidPitchRate.kd=kd; pidPitchRate.reset(); applied=true; }
          else if(gainsValid && idx == 4){ pidYawRate.kp=kp; pidYawRate.ki=ki; pidYawRate.kd=kd; pidYawRate.reset(); applied=true; }
          else if(gainsValid && idx == 5){ pidAltitude.kp=kp; pidAltitude.ki=ki; pidAltitude.kd=kd; pidAltitude.reset(); applied=true; }
          else if(gainsValid && idx == 6){ pidVertSpeed.kp=kp; pidVertSpeed.ki=ki; pidVertSpeed.kd=kd; pidVertSpeed.reset(); applied=true; }
          StaticJsonDocument<128> ack;
          ack["type"] = "pid_ack";
          ack["axis"] = axis;
          ack["applied"] = applied;
          String ackMsg;
          serializeJson(ack, ackMsg);
          if(!wsSendText(wsClient, ackMsg.c_str())) clearWebSocketClient();
        } else if(strcmp(typeStr, "pid_get") == 0){
          StaticJsonDocument<512> docOut;
          docOut["type"] = "pid_values";
          JsonObject pids = docOut.createNestedObject("pids");
          JsonObject a0 = pids.createNestedObject("roll_angle");
          a0["kp"]=pidRollAngle.kp; a0["ki"]=pidRollAngle.ki; a0["kd"]=pidRollAngle.kd;
          JsonObject a1 = pids.createNestedObject("pitch_angle");
          a1["kp"]=pidPitchAngle.kp; a1["ki"]=pidPitchAngle.ki; a1["kd"]=pidPitchAngle.kd;
          JsonObject a2 = pids.createNestedObject("roll_rate");
          a2["kp"]=pidRollRate.kp; a2["ki"]=pidRollRate.ki; a2["kd"]=pidRollRate.kd;
          JsonObject a3 = pids.createNestedObject("pitch_rate");
          a3["kp"]=pidPitchRate.kp; a3["ki"]=pidPitchRate.ki; a3["kd"]=pidPitchRate.kd;
          JsonObject a4 = pids.createNestedObject("yaw_rate");
          a4["kp"]=pidYawRate.kp; a4["ki"]=pidYawRate.ki; a4["kd"]=pidYawRate.kd;
          JsonObject a5 = pids.createNestedObject("altitude");
          a5["kp"]=pidAltitude.kp; a5["ki"]=pidAltitude.ki; a5["kd"]=pidAltitude.kd;
          JsonObject a6 = pids.createNestedObject("vert_speed");
          a6["kp"]=pidVertSpeed.kp; a6["ki"]=pidVertSpeed.ki; a6["kd"]=pidVertSpeed.kd;
          String msg;
          serializeJson(docOut, msg);
          if(!wsSendText(wsClient, msg.c_str())) clearWebSocketClient();
        } else if(strcmp(typeStr, "arm") == 0){
          bool arm = doc["arm"] | false;
          if(arm){
            Serial.println("WiFi ARM rejected; use Xbox LB+RB+D-pad Up gesture.");
          } else if(armed){
            disarmFlight(false);
            disarmTimeMs = millis();
            Serial.println("DISARMED via WiFi");
          }
          StaticJsonDocument<128> ack;
          ack["type"] = "arm_ack";
          ack["armed"] = armed;
          String ackMsg;
          serializeJson(ack, ackMsg);
          if(!wsSendText(wsClient, ackMsg.c_str())) clearWebSocketClient();
        }
      }
    } else if(len < 0){
      clearWebSocketClient();
    }
  }

  yield();
}
