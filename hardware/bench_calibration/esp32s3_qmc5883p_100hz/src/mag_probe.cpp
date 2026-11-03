/*
 * IntelQCopter v13 - QMC5883P fixed-boot-profile investigation.
 *
 * MAGNETOMETER ONLY.  No Wi-Fi, 500 Hz IMU logging, BMP readings,
 * motor PWM, runtime frequency commands, or automatic register reconfiguration.
 *
 * Tested hardware identity: QMC5883P at I2C address 0x2C, chip ID 0x80.
 * MPU6050 at 0x68 is accessed solely for AUX I2C bypass.
 *
 * QST QMC5883P datasheet Rev. E:
 *   0x00 chip ID; 0x01..0x06 XYZ (little-endian); 0x09 DRDY/OVFL;
 *   0x0A OSR2/OSR1/ODR/MODE; 0x0B range and set/reset; 0x29 axis sign.
 *   0x0B=0x08 => 8 gauss (3750 counts/G = 37.5 counts/uT).
 *
 * CHANGE IQ_MAG_BOOT_PROFILE IN platformio.ini *BEFORE FLASHING*.
 * FULLY REMOVE 3.3 V POWER between profile experiments.
 */
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>

#ifndef IQ_MAG_TEXT_DIAGNOSTIC
#define IQ_MAG_TEXT_DIAGNOSTIC 1
#endif
#ifndef IQ_WIFI_ENABLED
#define IQ_WIFI_ENABLED 0
#endif
#ifndef IQ_MAG_BOOT_PROFILE
#define IQ_MAG_BOOT_PROFILE 50
#endif
#ifndef IQ_MAG_CSV_DIVIDER
#define IQ_MAG_CSV_DIVIDER 1
#endif
#ifndef IQ_I2C_HZ
#define IQ_I2C_HZ 100000
#endif
#ifndef IQ_MAG_CONTINUOUS_FAST
#define IQ_MAG_CONTINUOUS_FAST 0
#endif
#ifndef IQ_MAG_POLL_INTERVAL_US
#define IQ_MAG_POLL_INTERVAL_US (IQ_MAG_BOOT_PROFILE == 0 ? 500 : 2000)
#endif
#ifndef IQ_MAG_STOP_START_READ
#define IQ_MAG_STOP_START_READ 1
#endif
#ifndef IQ_MAG_200HZ_LOW_OSR
#define IQ_MAG_200HZ_LOW_OSR 0
#endif
#if IQ_MAG_TEXT_DIAGNOSTIC != 1 || IQ_WIFI_ENABLED != 0
#error "v13 is isolated: IQ_MAG_TEXT_DIAGNOSTIC=1 and IQ_WIFI_ENABLED=0"
#endif
#if IQ_MAG_BOOT_PROFILE != 10 && IQ_MAG_BOOT_PROFILE != 50 && IQ_MAG_BOOT_PROFILE != 100 && IQ_MAG_BOOT_PROFILE != 200 && IQ_MAG_BOOT_PROFILE != 0
#error "IQ_MAG_BOOT_PROFILE must be 10, 50, 100, 200 or 0 (continuous)"
#endif
#if IQ_MAG_CSV_DIVIDER < 1
#error "IQ_MAG_CSV_DIVIDER must be >= 1"
#endif

namespace iq {

namespace hw {
constexpr uint8_t sda = 9, scl = 8;
constexpr uint8_t mpu = 0x68, mag = 0x2C;
constexpr uint8_t magId = 0x80;
constexpr uint8_t idReg = 0x00, xyzReg = 0x01, statusReg = 0x09;
constexpr uint8_t ctrl1 = 0x0A, ctrl2 = 0x0B, signReg = 0x29;
constexpr uint8_t motorPins[] = {12, 11, 14, 13};
constexpr uint32_t pollIntervalUs = IQ_MAG_POLL_INTERVAL_US;
constexpr uint32_t healthIntervalMs = 1000;
constexpr uint32_t verifyIntervalMs = 5000;
constexpr uint8_t range8g = 0x08;
constexpr uint8_t sign = 0x06;
// Serial bandwidth: do not let high-rate profiles flood the 115200 baud link.
constexpr uint32_t csvDivider =
  IQ_MAG_BOOT_PROFILE == 0 && IQ_MAG_CSV_DIVIDER < 16 ? 16 :
  IQ_MAG_BOOT_PROFILE == 200 && IQ_MAG_CSV_DIVIDER < 4 ? 4 :
  IQ_MAG_CSV_DIVIDER;
constexpr uint8_t ctrl1Value =
  IQ_MAG_BOOT_PROFILE == 200 && IQ_MAG_200HZ_LOW_OSR ? 0x3D :
  IQ_MAG_BOOT_PROFILE == 0 && IQ_MAG_CONTINUOUS_FAST ? 0x33 :
  IQ_MAG_BOOT_PROFILE == 10  ? 0xC1 :
  IQ_MAG_BOOT_PROFILE == 50  ? 0xC5 :
  IQ_MAG_BOOT_PROFILE == 100 ? 0xC9 :
  IQ_MAG_BOOT_PROFILE == 200 ? 0xCD : 0xC3;
} // namespace hw

struct ReadByteResult {
  bool ok = false;
  uint8_t value = 0;
  uint8_t phaseError = 255;
  uint8_t received = 0;
};

uint8_t addressProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return static_cast<uint8_t>(Wire.endTransmission(true));
}

ReadByteResult readRegister(uint8_t address, uint8_t reg) {
  ReadByteResult r;
  Wire.beginTransmission(address);
  if (Wire.write(reg) != 1) { (void)Wire.endTransmission(true); return r; }
  // Match the integrated calibration firmware: STOP after setting the
  // register pointer, then issue a separate read request. On this ESP32-S3
  // the QMC repeated-START path was observed to fail intermittently.
  r.phaseError = static_cast<uint8_t>(Wire.endTransmission(IQ_MAG_STOP_START_READ));
  if (r.phaseError != 0) return r;
  int n = Wire.requestFrom(static_cast<int>(address), 1);
  r.received = n >= 0 && n < 256 ? static_cast<uint8_t>(n) : 0;
  if (n == 1 && Wire.available() > 0) {
    r.value = static_cast<uint8_t>(Wire.read());
    r.ok = true;
  }
  while (Wire.available() > 0) (void)Wire.read();
  return r;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value, const char *name) {
  Wire.beginTransmission(address);
  const size_t nr = Wire.write(reg);
  const size_t nv = Wire.write(value);
  const uint8_t error = static_cast<uint8_t>(Wire.endTransmission(true));
  const bool ok = nr == 1 && nv == 1 && error == 0;
  Serial.printf("WRITE name=%s reg=0x%02X value=0x%02X endTx=%u ack=%d\n",
                name, reg, value, error, ok);
  return ok;
}

bool readXYZ(int16_t xyz[3]) {
  uint8_t bytes[6]{};
  Wire.beginTransmission(hw::mag);
  if (Wire.write(hw::xyzReg) != 1) { (void)Wire.endTransmission(true); return false; }
  if (Wire.endTransmission(IQ_MAG_STOP_START_READ) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(hw::mag), 6) != 6) {
    while (Wire.available() > 0) (void)Wire.read();
    return false;
  }
  for (uint8_t &b : bytes) {
    if (Wire.available() <= 0) return false;
    b = static_cast<uint8_t>(Wire.read());
  }
  for (int i = 0; i < 3; ++i)
    xyz[i] = static_cast<int16_t>((uint16_t(bytes[2*i+1]) << 8) | bytes[2*i]);
  return true;
}

bool configureMpuBypass() {
  const ReadByteResult who = readRegister(hw::mpu, 0x75);
  const ReadByteResult user = readRegister(hw::mpu, 0x6A);
  const ReadByteResult cfg = readRegister(hw::mpu, 0x37);
  if (!who.ok || who.value != 0x68 || !user.ok || !cfg.ok) {
    Serial.println("MPU_BYPASS unavailable; QMC may still be on main I2C bus");
    return false;
  }
  // No MPU internal master; release its auxiliary pins to the ESP32 I2C bus.
  const bool a = writeRegister(hw::mpu, 0x6A, uint8_t(user.value & ~0x20), "MPU_DISABLE_MASTER");
  const bool b = writeRegister(hw::mpu, 0x37, uint8_t(cfg.value | 0x02), "MPU_AUX_BYPASS");
  delay(20);
  const ReadByteResult readUser = readRegister(hw::mpu, 0x6A);
  const ReadByteResult readCfg  = readRegister(hw::mpu, 0x37);
  Serial.printf("MPU_BYPASS after USER_CTRL=%s0x%02X PIN_CFG=%s0x%02X\n",
    readUser.ok?"":"ERR/", readUser.value, readCfg.ok?"":"ERR/", readCfg.value);
  return a && b && readUser.ok && !(readUser.value & 0x20)
         && readCfg.ok && (readCfg.value & 0x02);
}

void scanBus() {
  Serial.print("I2C_SCAN");
  for (uint8_t address = 0x08; address <= 0x77; ++address)
    if (addressProbe(address) == 0) Serial.printf(" 0x%02X", address);
  Serial.println();
}

struct Health {
  bool firstIdentityOK = false;
  bool allWritesAck = false;
  bool readbackConfirmed = false; // Latches only after exact control-register readback.
  bool sawFresh = false;
  bool firstFailurePrinted = false;
  bool missingFreshAlert = false;
  uint32_t fresh = 0, notReady = 0, overflows = 0;
  uint32_t statusFailures = 0, xyzFailures = 0, verifyFailures = 0;
  uint32_t identicalReads = 0;
  uint64_t firstFreshUs = 0, lastFreshUs = 0;
  uint64_t totalFreshIntervalsUs = 0;
  uint32_t intervalCount = 0;
  int16_t lastXYZ[3] = {};
  int16_t previousXYZ[3] = {};
  bool havePreviousXYZ = false;
  uint8_t lastStatus = 0;
  uint8_t latestVerify1 = 0, latestVerify2 = 0;
  uint64_t firstFailureUs = 0;
} health;

void verifyRegisters(const char *reason) {
  const ReadByteResult id = readRegister(hw::mag, hw::idReg);
  const ReadByteResult c1 = readRegister(hw::mag, hw::ctrl1);
  const ReadByteResult c2 = readRegister(hw::mag, hw::ctrl2);
  const bool exact = id.ok && id.value == hw::magId
                  && c1.ok && c1.value == hw::ctrl1Value
                  && c2.ok && (c2.value & 0x3F) == hw::range8g;
  // A transient read error must not erase the previously observed readback.
  // A *successful* conflicting read is meaningful and revokes confirmation.
  if (exact) health.readbackConfirmed = true;
  else {
    ++health.verifyFailures;
    if (c1.ok && c1.value != hw::ctrl1Value) health.readbackConfirmed = false;
    if (c2.ok && (c2.value & 0x3F) != hw::range8g) health.readbackConfirmed = false;
  }
  health.latestVerify1 = c1.value;
  health.latestVerify2 = c2.value;
  Serial.printf("VERIFY reason=%s id=%s0x%02X C1=%s0x%02X C2=%s0x%02X "
                "this_read=%d confirmed_ever_and_not_contradicted=%d\n",
                reason, id.ok?"":"ERR/", id.value,
                c1.ok?"":"ERR/", c1.value,
                c2.ok?"":"ERR/", c2.value, exact, health.readbackConfirmed);
}

void configureOnceAtBoot() {
  Serial.printf("BOOT_PROFILE requested=%d ctrl1=0x%02X ctrl2=0x08 I2C=%dHz poll_us=%lu stop_start=%d csv_every=%d\n",
    IQ_MAG_BOOT_PROFILE, hw::ctrl1Value, IQ_I2C_HZ,
    static_cast<unsigned long>(hw::pollIntervalUs), IQ_MAG_STOP_START_READ, int(hw::csvDivider));
  const uint8_t ack = addressProbe(hw::mag);
  const ReadByteResult id = readRegister(hw::mag, hw::idReg);
  health.firstIdentityOK = ack == 0 && id.ok && id.value == hw::magId;
  Serial.printf("MAG_ID ack=%u read=%s0x%02X expected=0x80 valid=%d\n",
    ack, id.ok?"":"ERR/", id.value, health.firstIdentityOK);
  if (!health.firstIdentityOK) {
    Serial.println("BOOT_CONFIG_ABORT: QMC not identified. No writes sent to QMC.");
    return;
  }
  // No software mode changes after these four writes. No automatic retries.
  const bool a = writeRegister(hw::mag, hw::ctrl1, 0x00, "SUSPEND");
  delay(20);
  const bool b = writeRegister(hw::mag, hw::signReg, hw::sign, "XYZ_SIGN");
  const bool c = writeRegister(hw::mag, hw::ctrl2, hw::range8g, "RANGE_8G_SETRESET");
  const bool d = writeRegister(hw::mag, hw::ctrl1, hw::ctrl1Value, "START_SELECTED_PROFILE");
  health.allWritesAck = a && b && c && d;
  // One immediate readback is valuable: v12 showed it can succeed while a
  // subsequent delayed read fails transiently. Leave the device alone after.
  verifyRegisters("IMMEDIATE");
  Serial.printf("BOOT_CONFIG_DONE writes_ack=%d confirmed=%d; NO RUNTIME RECONFIGURATION\n",
    health.allWritesAck, health.readbackConfirmed);
}

void noteReadFailure(const char *where, ReadByteResult r) {
  if (!health.firstFailurePrinted) {
    health.firstFailurePrinted = true;
    health.firstFailureUs = static_cast<uint64_t>(esp_timer_get_time());
    Serial.printf("FIRST_BUS_READ_FAILURE stage=%s t_us=%llu reg_phase=%u returned_bytes=%u "
                  "SDA=%d SCL=%d\n", where,
                  static_cast<unsigned long long>(health.firstFailureUs),
                  r.phaseError, r.received, digitalRead(hw::sda), digitalRead(hw::scl));
  }
}

void poll() {
  const ReadByteResult s = readRegister(hw::mag, hw::statusReg);
  if (!s.ok) { ++health.statusFailures; noteReadFailure("STATUS_09", s); return; }
  health.lastStatus = s.value;
  if (s.value & 0x02) { ++health.overflows; return; }
  if (!(s.value & 0x01)) { ++health.notReady; return; }
  int16_t xyz[3];
  if (!readXYZ(xyz)) {
    ++health.xyzFailures;
    if (!health.firstFailurePrinted) {
      health.firstFailurePrinted = true;
      health.firstFailureUs = static_cast<uint64_t>(esp_timer_get_time());
      Serial.printf("FIRST_BUS_READ_FAILURE stage=XYZ_01 t_us=%llu SDA=%d SCL=%d\n",
        static_cast<unsigned long long>(health.firstFailureUs), digitalRead(hw::sda), digitalRead(hw::scl));
    }
    return;
  }
  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());
  if (health.sawFresh) {
    health.totalFreshIntervalsUs += t - health.lastFreshUs;
    ++health.intervalCount;
  } else health.firstFreshUs = t;
  health.sawFresh = true;
  health.lastFreshUs = t;
  if (health.havePreviousXYZ && xyz[0] == health.previousXYZ[0]
      && xyz[1] == health.previousXYZ[1] && xyz[2] == health.previousXYZ[2])
    ++health.identicalReads;
  for (int i=0; i<3; ++i) {
    health.lastXYZ[i] = xyz[i];
    health.previousXYZ[i] = xyz[i];
  }
  health.havePreviousXYZ = true;
  ++health.fresh;
  if (health.missingFreshAlert) {
    Serial.printf("FRESH_RECOVERED t_us=%llu after missing sample interval\n",
      static_cast<unsigned long long>(t));
    health.missingFreshAlert = false;
  }
  // CSV timestamps use actual sensor-read time, not a fabricated nominal dt.
  // 'verified' means registers were actually seen matching at least once;
  // it is NOT a guarantee of accuracy, noise floor or PCB power quality.
  if (health.fresh % hw::csvDivider == 0) {
    Serial.printf("CSV,%llu,%d,%d,%d,0x%02X,%d\n",
      static_cast<unsigned long long>(t), xyz[0],xyz[1],xyz[2],s.value,
      health.readbackConfirmed && health.allWritesAck);
  }
}

void report() {
  const double rate = health.intervalCount
    ? double(health.intervalCount) * 1e6 / double(health.totalFreshIntervalsUs)
    : 0.0;
  const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
  const double age = health.sawFresh ? (nowUs - health.lastFreshUs) * 1e-6 : -1.0;
  Serial.printf("HEALTH profile=%d measured_fresh_hz=%.2f fresh=%lu "
                "not_ready=%lu overflow=%lu status_fail=%lu xyz_fail=%lu "
                "verify_fail=%lu identical=%lu regs_ok=%d writes_ack=%d csv_every=%d last_status=0x%02X "
                "xyz=%d,%d,%d age_sec=%.3f\n",
                IQ_MAG_BOOT_PROFILE, rate,
                static_cast<unsigned long>(health.fresh),
                static_cast<unsigned long>(health.notReady),
                static_cast<unsigned long>(health.overflows),
                static_cast<unsigned long>(health.statusFailures),
                static_cast<unsigned long>(health.xyzFailures),
                static_cast<unsigned long>(health.verifyFailures),
                static_cast<unsigned long>(health.identicalReads),
                health.readbackConfirmed, health.allWritesAck, int(hw::csvDivider),
                health.lastStatus, health.lastXYZ[0], health.lastXYZ[1],
                health.lastXYZ[2], age);
  if (health.allWritesAck && nowUs > 2000000ULL &&
      (!health.sawFresh || nowUs - health.lastFreshUs > 1500000ULL) &&
      !health.missingFreshAlert) {
    health.missingFreshAlert = true;
    Serial.println("FRESH_MISSING: no new magnetic conversion for >=1.5s; "
                   "no automatic register rewrite; check supply/C1 and I2C");
  }
}

void setup() {
  for (uint8_t pin : hw::motorPins) pinMode(pin, INPUT);
  Serial.begin(115200);
  delay(900);
  Wire.begin(hw::sda, hw::scl);
  Wire.setClock(IQ_I2C_HZ);
  Wire.setTimeOut(30);
  Serial.println("\n=== IQ v13 QMC5883P FIXED-BOOT MAG-ONLY ===");
  Serial.println("NO runtime commands / NO Wifi / NO motor output / NO mode switching");
  scanBus();
  (void)configureMpuBypass();
  scanBus();
  configureOnceAtBoot();
  Serial.println("CSV_HEADER CSV,t_us,x_counts,y_counts,z_counts,status,verified");
  Serial.println("EXPERIMENT: do not send serial commands; change profile in platformio.ini, flash, FULL POWER CYCLE");
}

void loop() {
  // Intentionally ignore all serial commands, including v12's '1', '2', 'c', 'z'.
  while (Serial.available() > 0) (void)Serial.read();
  static uint64_t previousPollUs = 0;
  static uint32_t previousHealthMs = 0;
  static uint32_t previousVerifyMs = 0;
  const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
  if (nowUs - previousPollUs >= hw::pollIntervalUs) {
    previousPollUs = nowUs;
    if (health.firstIdentityOK && health.allWritesAck) poll();
  }
  const uint32_t nowMs = millis();
  if (nowMs - previousHealthMs >= hw::healthIntervalMs) {
    previousHealthMs = nowMs;
    report();
  }
  if (nowMs - previousVerifyMs >= hw::verifyIntervalMs) {
    previousVerifyMs = nowMs;
    if (health.firstIdentityOK) verifyRegisters("PERIODIC_READ_ONLY");
  }
  if (IQ_MAG_BOOT_PROFILE == 0) delayMicroseconds(50);
  else delay(1);
}

} // namespace iq
void setup() { iq::setup(); }
void loop() { iq::loop(); }
