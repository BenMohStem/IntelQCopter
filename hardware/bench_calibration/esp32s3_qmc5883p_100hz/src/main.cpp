/*
 * IntelQCopter — ESP32-S3 Competition Flight Controller
 * Sensor acquisition: fixed 400 kHz shared I2C bus, MPU6050 FIFO,
 * nonblocking 500 Hz loop, BMP/QMC cached-with-timestamp.
 *
 * I²C policy: single 400 kHz bus for MPU6050, BMP280, QMC5883P.
 * No clock switching, no fallback to 100 kHz. Wire.setTimeOut(30)
 * bounds every transaction.
 *
 * Loop policy: 500 Hz nonblocking. No waiting for DRDY/INT,
 * BMP conversion, or LiDAR. Every cycle uses the latest available
 * sample per sensor, retaining its real timestamp and marking
 * whether it is new (fresh) or reused. Reused samples are not
 * counted as fresh.
 *
 * MPU6050: FIFO enabled, ±500°/s, ±8g, DLPF_CFG=1, SMPLRT_DIV=1
 * (188/184 Hz), FIFO_EN=0xF8. 14-byte FIFO packets read each cycle.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_timer.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <climits>
#include <initializer_list>

#ifndef IQ_WIFI_ENABLED
#define IQ_WIFI_ENABLED 0
#endif
#if IQ_WIFI_ENABLED != 0
// Wi-Fi calibration enabled — UDP telemetry on IntelQCopter-Cal AP
#endif
#ifndef IQ_I2C_HZ
#define IQ_I2C_HZ 400000
#endif
#ifndef IQ_MAG_CONTROL1
#define IQ_MAG_CONTROL1 0x3D
#endif

#if IQ_WIFI_ENABLED
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

// ============================================================================
// CONFIGURATION (global scope)
// ============================================================================

constexpr uint8_t SDA_PIN = 9;
constexpr uint8_t SCL_PIN = 8;
constexpr uint8_t MOTOR_PINS[4] = {12, 11, 14, 13}; // FL, FR, RR, RL: INPUT ONLY
constexpr uint8_t MPU_PRIMARY_ADDRESS = 0x68;
constexpr uint8_t MPU_ALTERNATE_ADDRESS = 0x69;
constexpr uint8_t BMP_PRIMARY_ADDRESS = 0x76;
constexpr uint8_t BMP_ALTERNATE_ADDRESS = 0x77;
constexpr uint8_t MAG_ADDRESS = 0x2C; // QMC5883P

constexpr uint32_t CONTROL_PERIOD_US = 2000;   // 500 Hz
constexpr uint32_t BMP_PERIOD_US = 6000;       // ~166.7 Hz practical
constexpr uint32_t MAG_PERIOD_US = 5000;       // 200 Hz QMC ODR target
constexpr uint32_t MAG_STALE_MS = 1000;
constexpr uint32_t MAG_CONFIG_RETRY_MS = 1000;
constexpr uint8_t MAG_MAX_CONFIG_ATTEMPTS = 5;
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t PACKET_SIZE = 52;
constexpr uint32_t I2C_HZ = 400000;            // Single shared bus speed

constexpr uint8_t MAG_CONTROL1 = static_cast<uint8_t>(IQ_MAG_CONTROL1);
constexpr uint8_t MAG_CONTROL2 = 0x08;
constexpr uint8_t MAG_XYZ_SIGN_VAL = 0x06;

// MPU6050 register map
constexpr uint8_t MPU_REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t MPU_REG_PWR_MGMT_2    = 0x6C;
constexpr uint8_t MPU_REG_CONFIG        = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t MPU_REG_FIFO_EN       = 0x23;
constexpr uint8_t MPU_REG_INT_PIN_CFG   = 0x37;
constexpr uint8_t MPU_REG_USER_CTRL     = 0x6A;
constexpr uint8_t MPU_REG_FIFO_COUNT_H  = 0x72;
constexpr uint8_t MPU_REG_FIFO_R_W      = 0x74;
constexpr uint8_t MPU_REG_WHO_AM_I      = 0x75;

constexpr uint8_t MPU_PWR_MGMT_1_RESET  = 0x80;
constexpr uint8_t MPU_PWR_MGMT_1_CLKSEL = 0x01;
constexpr uint8_t MPU_DLPF_CFG_1        = 0x01;
constexpr uint8_t MPU_SMPLRT_DIV_1      = 0x01;
constexpr uint8_t MPU_GYRO_FS_500       = 0x08;
constexpr uint8_t MPU_ACCEL_FS_8G       = 0x10;
constexpr uint8_t MPU_FIFO_EN_ALL       = 0xF8;
constexpr uint8_t MPU_BYPASS_EN         = 0x02;
constexpr uint8_t MPU_FIFO_EN_BIT       = 0x40;
constexpr uint8_t MPU_FIFO_RESET_BIT    = 0x04;

constexpr size_t MPU_FIFO_PACKET_BYTES = 14;

// QMC5883P register map
constexpr uint8_t QMC_REG_CHIP_ID   = 0x00;
constexpr uint8_t QMC_REG_CTRL1     = 0x0A;
constexpr uint8_t QMC_REG_CTRL2     = 0x0B;
constexpr uint8_t QMC_REG_XYZ_SIGN  = 0x29;
constexpr uint8_t QMC_REG_DATA_XL   = 0x01;
constexpr uint8_t QMC_REG_STATUS    = 0x09;

constexpr uint8_t QMC_CHIP_ID       = 0x80;

// BMP280 register map
constexpr uint8_t BMP_REG_ID           = 0xD0;
constexpr uint8_t BMP_REG_RESET        = 0xE0;
constexpr uint8_t BMP_REG_CTRL_MEAS    = 0xF4;
constexpr uint8_t BMP_REG_CONFIG       = 0xF5;
constexpr uint8_t BMP_REG_PRESSURE_MSB = 0xF7;

constexpr uint8_t BMP_CHIP_ID          = 0x58;

// ============================================================================
// I²C BUS MANAGER — single 400 kHz bus
// ============================================================================

class I2cBus {
public:
  struct Stats {
    uint32_t total = 0;
    uint32_t ok = 0;
    uint32_t fail = 0;
    uint32_t mpu = 0;
    uint32_t bmp = 0;
    uint32_t mag = 0;
  } st;

  bool begin() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_HZ);
    Wire.setTimeOut(30);
    return true;
  }

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t value, uint32_t &sc) {
    ++st.total; ++sc;
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) { Wire.endTransmission(); ++st.fail; return false; }
    if (Wire.write(value) != 1) { Wire.endTransmission(); ++st.fail; return false; }
    if (Wire.endTransmission() != 0) { ++st.fail; return false; }
    ++st.ok; return true;
  }

  bool readRegs(uint8_t addr, uint8_t reg, uint8_t *out, size_t count,
                uint32_t &sc) {
    ++st.total; ++sc;
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) { Wire.endTransmission(); ++st.fail; return false; }
    if (Wire.endTransmission(false) != 0) { ++st.fail; return false; }
    int got = Wire.requestFrom((int)addr, (int)count);
    if (got != (int)count) {
      while (Wire.available()) (void)Wire.read();
      ++st.fail; return false;
    }
    for (size_t i = 0; i < count; ++i) out[i] = static_cast<uint8_t>(Wire.read());
    ++st.ok; return true;
  }

  bool isPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }
};

I2cBus i2cBus;

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

struct ImuSample {
  bool valid = false;
  bool fresh = false;
  uint32_t timestamp_us = 0;
  int16_t accel_raw[3] = {};
  int16_t gyro_raw[3] = {};
  int16_t temp_raw = 0;
  uint32_t fifo_packets = 0;
  uint32_t fifo_overflows = 0;
};

struct PressureSample {
  bool valid = false;
  bool fresh = false;
  uint32_t timestamp_us = 0;
  uint32_t raw_pressure = 0;
  uint32_t raw_temperature = 0;
  float temperature_c = 0.0f;
  float pressure_pa = 0.0f;
};

struct MagneticSample {
  bool valid = false;
  bool fresh = false;
  uint32_t timestamp_us = 0;
  int16_t raw[3] = {};
  float calibrated[3] = {};
  uint8_t status = 0;
};

ImuSample imu;
PressureSample baro;
MagneticSample mag;

Adafruit_BMP280 *bmp_driver = nullptr;

struct Health {
  uint32_t loop_count = 0;
  uint32_t missed_deadlines = 0;
  uint32_t imu_errors = 0;
  uint32_t baro_errors = 0;
  uint32_t mag_errors = 0;
  uint32_t serial_drops = 0;
  uint32_t uptime_us = 0;
};
Health health;

struct CalVector3 { float x, y, z; };
struct CalMatrix3x3 { float m[3][3]; };

CalVector3 cal_accel_bias = {0, 0, 0};
CalMatrix3x3 cal_accel_scale = {{{1,0,0},{0,1,0},{0,0,1}}};
CalVector3 cal_gyro_bias = {0, 0, 0};
CalVector3 cal_mag_hard_iron = {0, 0, 0};
CalMatrix3x3 cal_mag_soft_iron = {{{1,0,0},{0,1,0},{0,0,1}}};
bool calibration_loaded = false;

uint8_t mpu_address = 0;
uint8_t bmp_address = 0;
uint8_t mag_address = MAG_ADDRESS;
bool mpu_present = false;
bool bmp_present = false;
bool mag_configured = false;

// ============================================================================
// MPU6050 FIFO DRIVER
// ============================================================================

bool initMpu6050() {
  mpu_address = i2cBus.isPresent(MPU_PRIMARY_ADDRESS)
      ? MPU_PRIMARY_ADDRESS
      : (i2cBus.isPresent(MPU_ALTERNATE_ADDRESS)
          ? MPU_ALTERNATE_ADDRESS : 0);
  if (!mpu_address) return false;

  uint8_t id = 0;
  if (!i2cBus.readRegs(mpu_address, MPU_REG_WHO_AM_I, &id, 1, health.imu_errors) || id != 0x68)
    return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_PWR_MGMT_1, MPU_PWR_MGMT_1_RESET, health.imu_errors))
    return false;
  delay(100);

  if (!i2cBus.writeReg(mpu_address, MPU_REG_PWR_MGMT_1, MPU_PWR_MGMT_1_CLKSEL, health.imu_errors))
    return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_PWR_MGMT_2, 0x00, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_CONFIG, MPU_DLPF_CFG_1, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, 0x19, MPU_SMPLRT_DIV_1, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_500, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_ACCEL_CONFIG, MPU_ACCEL_FS_8G, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_FIFO_EN, MPU_FIFO_EN_ALL, health.imu_errors)) return false;

  uint8_t ic = 0;
  if (!i2cBus.readRegs(mpu_address, MPU_REG_INT_PIN_CFG, &ic, 1, health.imu_errors)) return false;
  if (!i2cBus.writeReg(mpu_address, MPU_REG_INT_PIN_CFG, ic | MPU_BYPASS_EN, health.imu_errors)) return false;

  uint8_t uc = 0;
  if (!i2cBus.readRegs(mpu_address, MPU_REG_USER_CTRL, &uc, 1, health.imu_errors)) return false;
  if (!i2cBus.writeReg(mpu_address, MPU_REG_USER_CTRL, uc | MPU_FIFO_EN_BIT, health.imu_errors)) return false;

  if (!i2cBus.writeReg(mpu_address, MPU_REG_USER_CTRL,
      uc | MPU_FIFO_EN_BIT | MPU_FIFO_RESET_BIT, health.imu_errors)) return false;

  mpu_present = true;
  return true;
}

bool readMpuFifo() {
  if (!mpu_present) { imu.valid = false; imu.fresh = false; return false; }

  uint8_t fc[2] = {};
  if (!i2cBus.readRegs(mpu_address, MPU_REG_FIFO_COUNT_H, fc, 2, health.imu_errors)) {
    health.imu_errors++;
    imu.valid = false; imu.fresh = false;
    return false;
  }
  uint16_t fifo_count = (uint16_t(fc[0]) << 8) | fc[1];

  if (fifo_count >= 1024) {
    uint8_t uc = 0;
    i2cBus.readRegs(mpu_address, MPU_REG_USER_CTRL, &uc, 1, health.imu_errors);
    i2cBus.writeReg(mpu_address, MPU_REG_USER_CTRL,
        uc | MPU_FIFO_EN_BIT | MPU_FIFO_RESET_BIT, health.imu_errors);
    health.imu_errors++;
    imu.valid = false; imu.fresh = false;
    return false;
  }

  if (fifo_count < MPU_FIFO_PACKET_BYTES) {
    imu.valid = false; imu.fresh = false;
    return false;
  }

  uint8_t data[MPU_FIFO_PACKET_BYTES] = {};
  if (!i2cBus.readRegs(mpu_address, MPU_REG_FIFO_R_W, data, MPU_FIFO_PACKET_BYTES, health.imu_errors)) {
    health.imu_errors++;
    imu.valid = false; imu.fresh = false;
    return false;
  }

  imu.timestamp_us = (uint32_t)esp_timer_get_time();
  imu.fresh = true;
  imu.valid = true;
  imu.fifo_packets = fifo_count / MPU_FIFO_PACKET_BYTES;

  imu.accel_raw[0] = (int16_t)((uint16_t(data[0]) << 8) | data[1]);
  imu.accel_raw[1] = (int16_t)((uint16_t(data[2]) << 8) | data[3]);
  imu.accel_raw[2] = (int16_t)((uint16_t(data[4]) << 8) | data[5]);
  imu.temp_raw     = (int16_t)((uint16_t(data[6]) << 8) | data[7]);
  imu.gyro_raw[0]  = (int16_t)((uint16_t(data[8]) << 8) | data[9]);
  imu.gyro_raw[1]  = (int16_t)((uint16_t(data[10]) << 8) | data[11]);
  imu.gyro_raw[2]  = (int16_t)((uint16_t(data[12]) << 8) | data[13]);

  return true;
}

// ============================================================================
// QMC5883P DRIVER — 400 kHz only
// ============================================================================

bool initQmc5883p() {
  mag_address = MAG_ADDRESS;
  if (!i2cBus.isPresent(mag_address)) return false;

  uint8_t id = 0;
  if (!i2cBus.readRegs(mag_address, QMC_REG_CHIP_ID, &id, 1, health.mag_errors) || id != QMC_CHIP_ID)
    return false;

  if (!i2cBus.writeReg(mag_address, QMC_REG_CTRL1, 0x00, health.mag_errors)) return false;
  delay(20);

  if (!i2cBus.writeReg(mag_address, QMC_REG_XYZ_SIGN, MAG_XYZ_SIGN_VAL, health.mag_errors)) return false;

  if (!i2cBus.writeReg(mag_address, QMC_REG_CTRL2, MAG_CONTROL2, health.mag_errors)) return false;

  if (!i2cBus.writeReg(mag_address, QMC_REG_CTRL1, MAG_CONTROL1, health.mag_errors)) return false;

  mag_configured = true;
  return true;
}

bool readQmc5883p() {
  if (!mag_configured) { mag.valid = false; mag.fresh = false; return false; }

  uint8_t status = 0;
  if (!i2cBus.readRegs(mag_address, QMC_REG_STATUS, &status, 1, health.mag_errors)) {
    health.mag_errors++;
    mag.valid = false; mag.fresh = false;
    return false;
  }
  mag.status = status;

  if (status & 0x02) { health.mag_errors++; mag.valid = false; mag.fresh = false; return false; }
  if (!(status & 0x01)) { mag.valid = false; mag.fresh = false; return false; }

  uint8_t data[6] = {};
  if (!i2cBus.readRegs(mag_address, QMC_REG_DATA_XL, data, 6, health.mag_errors)) {
    health.mag_errors++;
    mag.valid = false; mag.fresh = false;
    return false;
  }

  for (int i = 0; i < 3; i++)
    mag.raw[i] = (int16_t)((uint16_t(data[2*i+1]) << 8) | data[2*i]);

  if ((mag.raw[0] == 0 && mag.raw[1] == 0 && mag.raw[2] == 0) ||
      (mag.raw[0] == -1 && mag.raw[1] == -1 && mag.raw[2] == -1) ||
      mag.raw[0] == INT16_MIN || mag.raw[1] == INT16_MIN || mag.raw[2] == INT16_MIN) {
    mag.valid = false; mag.fresh = false;
    return false;
  }

  mag.timestamp_us = (uint32_t)esp_timer_get_time();
  mag.fresh = true;
  mag.valid = true;

  float raw_uT[3] = {mag.raw[0] / 37.5f, mag.raw[1] / 37.5f, mag.raw[2] / 37.5f};
  float centered[3] = {
    raw_uT[0] - cal_mag_hard_iron.x,
    raw_uT[1] - cal_mag_hard_iron.y,
    raw_uT[2] - cal_mag_hard_iron.z
  };
  for (int i = 0; i < 3; i++)
    mag.calibrated[i] = cal_mag_soft_iron.m[i][0] * centered[0] +
                        cal_mag_soft_iron.m[i][1] * centered[1] +
                        cal_mag_soft_iron.m[i][2] * centered[2];

  return true;
}

// ============================================================================
// BMP280 DRIVER — 400 kHz only
// ============================================================================

bool initBmp280() {
  bmp_address = i2cBus.isPresent(BMP_PRIMARY_ADDRESS)
      ? BMP_PRIMARY_ADDRESS
      : (i2cBus.isPresent(BMP_ALTERNATE_ADDRESS)
          ? BMP_ALTERNATE_ADDRESS : 0);
  if (!bmp_address) return false;

  uint8_t id = 0;
  if (!i2cBus.readRegs(bmp_address, BMP_REG_ID, &id, 1, health.baro_errors) || id != BMP_CHIP_ID)
    return false;

  if (!i2cBus.writeReg(bmp_address, BMP_REG_RESET, 0xB6, health.baro_errors)) return false;
  delay(10);

  if (!i2cBus.writeReg(bmp_address, BMP_REG_CTRL_MEAS, 0x27, health.baro_errors)) return false;
  if (!i2cBus.writeReg(bmp_address, BMP_REG_CONFIG, 0x04, health.baro_errors)) return false;

  bmp_present = true;
  return true;
}

bool readBmp280() {
  if (!bmp_present) { baro.valid = false; baro.fresh = false; return false; }

  uint8_t regs[6] = {};
  if (!i2cBus.readRegs(bmp_address, BMP_REG_PRESSURE_MSB, regs, 6, health.baro_errors)) {
    health.baro_errors++;
    baro.valid = false; baro.fresh = false;
    return false;
  }

  baro.raw_pressure  = ((uint32_t(regs[0]) << 12) | (uint32_t(regs[1]) << 4) | (regs[2] >> 4));
  baro.raw_temperature = ((uint32_t(regs[3]) << 12) | (uint32_t(regs[4]) << 4) | (regs[5] >> 4));
  baro.timestamp_us = (uint32_t)esp_timer_get_time();
  baro.fresh = true;
  baro.valid = true;

  if (bmp_driver) {
    float t = bmp_driver->readTemperature();
    float p = bmp_driver->readPressure();
    if (std::isfinite(t) && std::isfinite(p) && t > -100.f && t < 100.f && p > 10000.f && p < 120000.f) {
      baro.temperature_c = t;
      baro.pressure_pa = p;
    }
  }

  return true;
}

bool loadCalDefaults() {
  cal_accel_bias = {0, 0, 0};
  cal_accel_scale = {{{1,0,0},{0,1,0},{0,0,1}}};
  cal_gyro_bias = {0, 0, 0};
  cal_mag_hard_iron = {0, 0, 0};
  cal_mag_soft_iron = {{{1,0,0},{0,1,0},{0,0,1}}};
  calibration_loaded = true;
  return true;
}

bool applyAccelCal(const int16_t raw[3], float out[3]) {
  for (int i = 0; i < 3; i++) {
    float r = raw[i] / 4096.0f * 9.80665f;
    out[i] = cal_accel_scale.m[i][0] * (r - cal_accel_bias.x) +
             cal_accel_scale.m[i][1] * (r - cal_accel_bias.y) +
             cal_accel_scale.m[i][2] * (r - cal_accel_bias.z);
  }
  return true;
}

bool applyGyroCal(const int16_t raw[3], float out[3]) {
  for (int i = 0; i < 3; i++) {
    float r = raw[i] / 65.5f;
    out[i] = r - (i == 0 ? cal_gyro_bias.x : (i == 1 ? cal_gyro_bias.y : cal_gyro_bias.z));
  }
  return true;
}

// ============================================================================
// BINARY TELEMETRY v3 (52 bytes)
// ============================================================================

struct Packet {
  uint8_t bytes[PACKET_SIZE] = {};
  size_t len = 0;
  void u8(uint8_t v) { if (len < PACKET_SIZE) bytes[len++] = v; }
  void u16(uint16_t v) { u8(v & 0xFF); u8((v >> 8) & 0xFF); }
  void i16(int16_t v) { u16((uint16_t)v); }
  void u32(uint32_t v) { u8(v & 0xFF); u8((v >> 8) & 0xFF); u8((v >> 16) & 0xFF); u8((v >> 24) & 0xFF); }
};

uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
  }
  return c;
}

void buildPacket(Packet &p, const ImuSample &imu, const MagneticSample &mag,
                 const PressureSample &baro, uint32_t seq,
                 uint32_t timestamp_us, uint16_t late_us) {
  uint8_t flags = 0;
  flags |= imu.valid  ? 0x01 : 0;
  flags |= baro.valid ? 0x02 : 0;
  flags |= mag.valid  ? 0x04 : 0;
  flags |= baro.fresh ? 0x08 : 0;
  flags |= mag.fresh  ? 0x10 : 0;
  flags |= late_us > 250 ? 0x20 : 0;
  flags |= 0x40;

  p.len = 0;
  p.u8(0xA5); p.u8(0x5A); p.u8(3); p.u8(flags);
  p.u32(seq);
  p.u32(timestamp_us);
  p.u16(late_us);

  for (int i = 0; i < 3; i++) p.i16(imu.accel_raw[i]);
  for (int i = 0; i < 3; i++) p.i16(imu.gyro_raw[i]);
  p.i16(imu.temp_raw);
  for (int i = 0; i < 3; i++) p.i16(mag.raw[i]);

  p.u32(baro.raw_pressure);
  p.u32(baro.raw_temperature);
  p.i16(int16_t(baro.temperature_c * 100.0f));
  p.u32(uint32_t(baro.pressure_pa));

  p.u16(uint16_t(health.missed_deadlines));

  while (p.len < 50) p.u8(0);
  p.u16(crc16(p.bytes, 50));
}

// ============================================================================
// MAIN LOOP — 500 Hz nonblocking
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2000) { /* wait for USB */ }

  for (uint8_t pin : MOTOR_PINS) pinMode(pin, INPUT);

  i2cBus.begin();
  loadCalDefaults();

  mpu_present = initMpu6050();
  delay(30);
  mag_configured = initQmc5883p();
  if (mpu_present) {
    delay(30);
    bmp_present = initBmp280();
    if (bmp_present) bmp_driver = new Adafruit_BMP280(&Wire);
  }

  health.uptime_us = esp_timer_get_time();

  Serial.printf("IQ FC 400kHz: MPU=%d BMP=%d MAG=%d I2C=400kHz\n",
    mpu_present, bmp_present, mag_configured);
}

void loop() {
  static uint32_t next_deadline = 0;
  static uint32_t seq = 0;
  static uint32_t last_baro_us = 0;
  static uint32_t last_mag_us = 0;
  Packet pkt;

  uint32_t now = (uint32_t)esp_timer_get_time();
  if (next_deadline == 0) next_deadline = now + CONTROL_PERIOD_US;

  int64_t until = (int64_t)next_deadline - (int64_t)now;
  if (until > 240) {
    delayMicroseconds((uint32_t)(until - 140));
    return;
  }
  if (until > 0) return;

  uint32_t started = (uint32_t)esp_timer_get_time();
  int64_t late = (int64_t)started - (int64_t)next_deadline;
  if (late >= (int64_t)CONTROL_PERIOD_US) {
    uint64_t skipped = (uint64_t)late / CONTROL_PERIOD_US;
    health.missed_deadlines += (uint32_t)(skipped > UINT32_MAX ? UINT32_MAX : skipped);
    next_deadline += (uint64_t)skipped * CONTROL_PERIOD_US;
  }
  next_deadline += CONTROL_PERIOD_US;
  health.loop_count++;
  health.uptime_us = started;

  bool imu_ok = readMpuFifo();
  if (!imu_ok) health.imu_errors++;

  bool baro_ok = false;
  if ((int64_t)(started - last_baro_us) >= (int64_t)BMP_PERIOD_US) {
    last_baro_us = started;
    baro_ok = readBmp280();
    if (!baro_ok) health.baro_errors++;
  } else {
    baro.fresh = false;
  }

  bool mag_ok = false;
  if ((int64_t)(started - last_mag_us) >= (int64_t)MAG_PERIOD_US) {
    last_mag_us = started;
    mag_ok = readQmc5883p();
    if (!mag_ok) health.mag_errors++;
  } else {
    mag.fresh = false;
  }

  uint16_t late_us = (uint16_t)(late > 65535 ? 65535 : (late > 0 ? late : 0));
  buildPacket(pkt, imu, mag, baro, seq++, started, late_us);

  if (Serial.availableForWrite() >= (int)PACKET_SIZE)
    Serial.write(pkt.bytes, PACKET_SIZE);
  else
    health.serial_drops++;
}