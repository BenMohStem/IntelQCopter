/*
 * IntelQCopter v18 - QMC5883P system-aware startup / Wi-Fi differential diagnostics
 * Derived from the user's v13 fixed-boot MAG-only firmware.
 *
 * SAFE BENCH DIAGNOSTIC ONLY.  Motor pins are INPUT, never PWM or arming.
 * Wi-Fi is OPTIONAL for controlled A/B testing; no ESC, flight, BMP, or LiDAR code.
 *
 * Datasheet: QST QMC5883P Rev E, 200 Hz normal mode 0x0A=0xCD,
 * 0x29=0x06 and 0x0B=0x08. DO NOT substitute QMC5883L register layout.
 *
 * v13 result: at 200 Hz the control write ACKed, later C1 and C2 were
 * both 0x00 and all measurements stopped.  That is not proof of the cause.
 * This program reports the failure and tries bounded recovery (configurable).
 */
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stdint.h>
#include <esp_system.h>

#ifndef IQ_WIFI_STRESS_MODE
#define IQ_WIFI_STRESS_MODE 0
#endif
#ifndef IQ_BUS_LOAD_ENABLE
#define IQ_BUS_LOAD_ENABLE 0
#endif
#ifndef IQ_WIFI_START_DELAY_MS
#define IQ_WIFI_START_DELAY_MS 5000
#endif
#ifndef IQ_WIFI_SCAN_INTERVAL_MS
#define IQ_WIFI_SCAN_INTERVAL_MS 10000
#endif
#if IQ_WIFI_STRESS_MODE != 0
#include <WiFi.h>
#endif

#ifndef IQ_RUN_NONCE
#define IQ_RUN_NONCE 0
#endif

#ifndef IQ_MAG_BOOT_PROFILE
#define IQ_MAG_BOOT_PROFILE 200
#endif
#ifndef IQ_MAG_OSR1
#define IQ_MAG_OSR1 8
#endif
#ifndef IQ_MAG_CSV_DIVIDER
#define IQ_MAG_CSV_DIVIDER 4
#endif
#ifndef IQ_I2C_HZ
#define IQ_I2C_HZ 100000
#endif
#ifndef IQ_SERIAL_BAUD
#define IQ_SERIAL_BAUD 115200
#endif
#ifndef IQ_MAG_READ_STOP
#define IQ_MAG_READ_STOP 0
#endif
#ifndef IQ_MAG_SOFT_RESET
#define IQ_MAG_SOFT_RESET 0
#endif
#ifndef IQ_MAG_START_ENABLE
#define IQ_MAG_START_ENABLE 1
#endif
#ifndef IQ_MAG_CTRL1_OVERRIDE
#define IQ_MAG_CTRL1_OVERRIDE 0
#endif
#ifndef IQ_RECOVERY_ENABLE
#define IQ_RECOVERY_ENABLE 1
#endif
#ifndef IQ_RECOVERY_MAX_ATTEMPTS
#define IQ_RECOVERY_MAX_ATTEMPTS 2
#endif
#ifndef IQ_ALLOW_100HZ_FALLBACK
#define IQ_ALLOW_100HZ_FALLBACK 0
#endif

#if IQ_BUS_LOAD_ENABLE != 0 && IQ_BUS_LOAD_ENABLE != 1
#error "IQ_BUS_LOAD_ENABLE must be 0 or 1"
#endif
#if IQ_WIFI_STRESS_MODE < 0 || IQ_WIFI_STRESS_MODE > 2
#error "IQ_WIFI_STRESS_MODE must be 0 (off), 1 (AP) or 2 (STA scanning)"
#endif
#if IQ_WIFI_START_DELAY_MS > 120000
#error "IQ_WIFI_START_DELAY_MS should not exceed 120000"
#endif
#if IQ_MAG_BOOT_PROFILE != 10 && IQ_MAG_BOOT_PROFILE != 50 && IQ_MAG_BOOT_PROFILE != 100 && IQ_MAG_BOOT_PROFILE != 200
#error "IQ_MAG_BOOT_PROFILE should be 10, 50, 100 or 200 Hz"
#endif
#if IQ_MAG_OSR1 != 1 && IQ_MAG_OSR1 != 2 && IQ_MAG_OSR1 != 4 && IQ_MAG_OSR1 != 8
#error "IQ_MAG_OSR1 must be 1,2,4,8"
#endif
#if IQ_MAG_CSV_DIVIDER < 1 || IQ_MAG_CSV_DIVIDER > 1000
#error "IQ_MAG_CSV_DIVIDER must be 1..1000"
#endif
#if IQ_I2C_HZ != 50000 && IQ_I2C_HZ != 100000 && IQ_I2C_HZ != 400000
#error "Set IQ_I2C_HZ=50000, 100000 or 400000"
#endif
#if IQ_MAG_READ_STOP != 0 && IQ_MAG_READ_STOP != 1
#error "IQ_MAG_READ_STOP must be 0 (repeated START) or 1 (STOP+START experiment)"
#endif
#if IQ_MAG_SOFT_RESET != 0 && IQ_MAG_SOFT_RESET != 1
#error "IQ_MAG_SOFT_RESET must be 0 (off) or 1 (explicit QMC reset before config)"
#endif
#if IQ_MAG_START_ENABLE != 0 && IQ_MAG_START_ENABLE != 1
#error "IQ_MAG_START_ENABLE must be 0 (prestart only) or 1"
#endif
#if IQ_MAG_CTRL1_OVERRIDE < 0 || IQ_MAG_CTRL1_OVERRIDE > 255
#error "IQ_MAG_CTRL1_OVERRIDE must be 0 or a byte value"
#endif
#if IQ_RECOVERY_MAX_ATTEMPTS < 0 || IQ_RECOVERY_MAX_ATTEMPTS > 5
#error "Bound recovery attempts to 0..5"
#endif

namespace iq {
constexpr uint8_t SDA_PIN=9, SCL_PIN=8, MPU=0x68, MAG=0x2C;
constexpr uint8_t ID=0x00, XYZ=0x01, STATUS=0x09, CTRL1=0x0A, CTRL2=0x0B, SIGN=0x29;
constexpr uint8_t MAG_ID=0x80, SIGN_VALUE=0x06, RANGE8G=0x08;
constexpr uint8_t MOTOR_PINS[] = {12, 11, 14, 13};
constexpr uint32_t POLL_US=1000, VERIFY_MS=5000, REPORT_MS=1000, STARTUP_SETTLE_MS=40;
constexpr uint32_t WARMUP_MS=200, MISSING_MS=1500, REPAIR_INTERVAL_MS=1500;

constexpr uint8_t osr1Bits() { return IQ_MAG_OSR1==8?0 : IQ_MAG_OSR1==4?1 : IQ_MAG_OSR1==2?2:3; }
constexpr uint8_t odrBits(uint16_t hz) { return hz==10?0 : hz==50?1 : hz==100?2:3; }
uint8_t expectedC1(uint16_t hz) {
  return IQ_MAG_CTRL1_OVERRIDE ? static_cast<uint8_t>(IQ_MAG_CTRL1_OVERRIDE)
                               : static_cast<uint8_t>(0xC0 | (osr1Bits()<<4) | (odrBits(hz)<<2) | 0x01);
}

struct ByteRead { bool ok=false; uint8_t value=0, phase=255, received=0; };
struct Counters {
  uint32_t fresh=0, notReady=0, overflow=0, statusFail=0, xyzFail=0, verifyFail=0;
  uint32_t failedRegisterPhases=0, zeroByteReads=0, recoveries=0, resetSignatures=0;
  uint32_t windowFresh=0;
  uint64_t lastFreshUs=0, prevFreshUs=0, totalDeltaUs=0;
  uint32_t deltaCount=0;
  uint8_t lastStatus=0;
  int16_t lastXYZ[3]={};
} ctr;

bool identityOK=false, configured=false, prestartOK=false, firstFailurePrinted=false, fallback=false;
bool softResetAttempted=false, softResetOK=false;
bool wifiStarted=false, wifiStartTried=false, wifiScanPending=false;
uint32_t wifiStartedAtMs=0, wifiLastScanAtMs=0, wifiScanCount=0, wifiScanFail=0;
uint32_t imuLoadOk=0, imuLoadFail=0, bmpLoadOk=0, bmpLoadFail=0;
uint32_t imuLoadMaxUs=0, bmpLoadMaxUs=0;
uint16_t activeHz=IQ_MAG_BOOT_PROFILE;
uint32_t configuredAtMs=0, lastAttemptMs=0, lastHealthMs=0, lastVerifyMs=0;
uint32_t windowStartedMs=0;

void beginWifiTest(const char* reason) {
#if IQ_WIFI_STRESS_MODE == 0
  (void)reason;
#else
  if (wifiStartTried) return;
  wifiStartTried = true;
  Serial.printf("WIFI_EVENT stage=START reason=%s mode=%d t_ms=%lu\n",
    reason, IQ_WIFI_STRESS_MODE, static_cast<unsigned long>(millis()));
  bool okay = false;
#if IQ_WIFI_STRESS_MODE == 1
  okay = WiFi.mode(WIFI_AP) && WiFi.softAP("IntelQCopter-DIAG", "iqmagtest42");
#elif IQ_WIFI_STRESS_MODE == 2
  okay = WiFi.mode(WIFI_STA);
  if (okay) WiFi.disconnect();
#endif
  wifiStarted = okay;
  wifiStartedAtMs = millis();
  wifiLastScanAtMs = wifiStartedAtMs;
  Serial.printf("WIFI_EVENT stage=READY mode=%d active=%d t_ms=%lu\n",
     IQ_WIFI_STRESS_MODE, wifiStarted?1:0, static_cast<unsigned long>(wifiStartedAtMs));
  if (!okay) Serial.println("WIFI_EVENT stage=ERROR check WiFi initialization");
#endif
}

void pollWifiTest() {
#if IQ_WIFI_STRESS_MODE == 0
  return;
#else
  const uint32_t now = millis();
  if (!wifiStartTried && now >= IQ_WIFI_START_DELAY_MS) beginWifiTest("DELAYED");
#if IQ_WIFI_STRESS_MODE == 2
  if (wifiStarted) {
    if (wifiScanPending) {
      const int n = WiFi.scanComplete();
      if (n != WIFI_SCAN_RUNNING) {
        wifiScanPending = false;
        if (n >= 0) ++wifiScanCount;
        else ++wifiScanFail;
        Serial.printf("WIFI_EVENT stage=SCAN_DONE count=%d ok_total=%lu fail_total=%lu t_ms=%lu\n",
          n, static_cast<unsigned long>(wifiScanCount),
          static_cast<unsigned long>(wifiScanFail), static_cast<unsigned long>(now));
        WiFi.scanDelete();
      }
    }
    if (!wifiScanPending && now - wifiLastScanAtMs >= IQ_WIFI_SCAN_INTERVAL_MS) {
      wifiLastScanAtMs = now;
      const int result = WiFi.scanNetworks(true);
      wifiScanPending = (result == WIFI_SCAN_RUNNING);
      Serial.printf("WIFI_EVENT stage=SCAN_START result=%d t_ms=%lu\n",result,
        static_cast<unsigned long>(now));
      if (result == WIFI_SCAN_FAILED) ++wifiScanFail;
      else if (result >= 0) { ++wifiScanCount; WiFi.scanDelete(); }
    }
  }
#endif
#endif
}

// Optional bus-traffic load. This is not calibrated MPU/BMP sensor fusion.
// All I2C calls remain in this single loop to eliminate self-induced bus races.
bool loadRead(uint8_t addr, uint8_t reg, uint8_t amount, uint32_t &maxDuration) {
  const uint64_t started=static_cast<uint64_t>(esp_timer_get_time());
  Wire.beginTransmission(addr);
  if (Wire.write(reg)!=1) { (void)Wire.endTransmission(true); return false; }
  if (Wire.endTransmission(false)!=0) return false;
  const int n=Wire.requestFrom(static_cast<int>(addr), static_cast<int>(amount));
  if (n!=amount) { while(Wire.available()>0) (void)Wire.read(); return false; }
  for (int i=0;i<n;++i) { if(Wire.available()>0) (void)Wire.read(); else return false; }
  const uint32_t duration=static_cast<uint32_t>(static_cast<uint64_t>(esp_timer_get_time())-started);
  if(duration>maxDuration) maxDuration=duration;
  return true;
}
void sensorBusLoad() {
#if IQ_BUS_LOAD_ENABLE
  static uint64_t prevImuUs=0, prevBmpUs=0;
  const uint64_t now=static_cast<uint64_t>(esp_timer_get_time());
  if(now-prevImuUs>=2000) { // MPU 500 Hz, burst 14 bytes, no calibration.
    prevImuUs=now;
    if(loadRead(MPU,0x3B,14,imuLoadMaxUs)) ++imuLoadOk;
    else ++imuLoadFail;
  }
  if(now-prevBmpUs>=20000) { // BMP 50 Hz, raw 6-byte register read.
    prevBmpUs=now;
    if(loadRead(0x76,0xF7,6,bmpLoadMaxUs)) ++bmpLoadOk;
    else ++bmpLoadFail;
  }
#endif
}

void beginWire() {
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(IQ_I2C_HZ);
  Wire.setTimeOut(25);
}
uint8_t probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return static_cast<uint8_t>(Wire.endTransmission(true));
}
ByteRead readByteMode(uint8_t addr, uint8_t reg, bool useStop) {
  ByteRead r;
  Wire.beginTransmission(addr);
  if (Wire.write(reg)!=1) { (void)Wire.endTransmission(true); return r; }
  r.phase=static_cast<uint8_t>(Wire.endTransmission(useStop));
  if (r.phase!=0) { ++ctr.failedRegisterPhases; return r; }
  const int n=Wire.requestFrom(static_cast<int>(addr),1);
  r.received = (n>=0 && n<=255) ? static_cast<uint8_t>(n) : 0;
  if (n==1 && Wire.available()>0) {
    r.value=static_cast<uint8_t>(Wire.read());
    r.ok=true;
  } else ++ctr.zeroByteReads;
  while (Wire.available()>0) (void)Wire.read();
  return r;
}
ByteRead readByte(uint8_t addr, uint8_t reg) {
  return readByteMode(addr, reg, IQ_MAG_READ_STOP != 0);
}
bool readXYZ(int16_t out[3]) {
  Wire.beginTransmission(MAG);
  if (Wire.write(XYZ)!=1) { (void)Wire.endTransmission(true); return false; }
  const uint8_t e=static_cast<uint8_t>(Wire.endTransmission(IQ_MAG_READ_STOP ? true : false));
  if (e!=0) { ++ctr.failedRegisterPhases; return false; }
  const int n=Wire.requestFrom(static_cast<int>(MAG),6);
  if (n!=6) {
    ++ctr.zeroByteReads;
    while(Wire.available()>0) (void)Wire.read();
    return false;
  }
  uint8_t buf[6];
  for (uint8_t &x:buf) {
    if (Wire.available()==0) return false;
    x=static_cast<uint8_t>(Wire.read());
  }
  for(int i=0;i<3;++i) out[i]=static_cast<int16_t>((uint16_t(buf[2*i+1])<<8)|buf[2*i]);
  return true;
}
bool writeByte(uint8_t addr,uint8_t reg,uint8_t val,const char *name) {
  Wire.beginTransmission(addr);
  const size_t a=Wire.write(reg),b=Wire.write(val);
  const uint8_t e=static_cast<uint8_t>(Wire.endTransmission(true));
  const bool ok=a==1 && b==1 && e==0;
  Serial.printf("WRITE name=%s reg=0x%02X value=0x%02X tx_error=%u ack=%d t_ms=%lu\n",
      name,reg,val,e,ok,static_cast<unsigned long>(millis()));
  return ok;
}

void scanBus() {
  Serial.print("I2C_SCAN");
  for(uint8_t a=0x08;a<=0x77;++a) if(probe(a)==0) Serial.printf(" 0x%02X",a);
  Serial.println();
}
bool enableMpuBypass() {
  const ByteRead who=readByte(MPU,0x75);
  const ByteRead user=readByte(MPU,0x6A);
  const ByteRead cfg=readByte(MPU,0x37);
  if(!who.ok || who.value!=0x68 || !user.ok || !cfg.ok) {
    Serial.printf("MPU_BYPASS_SKIPPED who_ok=%d user_ok=%d cfg_ok=%d\n",who.ok,user.ok,cfg.ok);
    return false;
  }
  const bool a=writeByte(MPU,0x6A,static_cast<uint8_t>(user.value & ~uint8_t(0x20)),"MPU_DISABLE_MASTER");
  const bool b=writeByte(MPU,0x37,static_cast<uint8_t>(cfg.value | 0x02),"MPU_AUX_BYPASS");
  delay(20);
  const ByteRead u=readByte(MPU,0x6A),c=readByte(MPU,0x37);
  const bool ok=a&&b&&u.ok&&c.ok&&!(u.value&0x20)&&(c.value&0x02);
  Serial.printf("MPU_BYPASS result=%d ctrl=%s0x%02X cfg=%s0x%02X\n",
      ok,u.ok?"":"ERR/",u.value,c.ok?"":"ERR/",c.value);
  return ok;
}

bool verify(const char *stage) {
  // Deliberate spacing avoids back-to-back register traffic while measuring.
  const ByteRead id=readByte(MAG,ID);
  delay(2);
  const ByteRead c1=readByte(MAG,CTRL1);
  delay(2);
  const ByteRead c2=readByte(MAG,CTRL2);
  const uint8_t target=expectedC1(activeHz);
  const bool exact=id.ok && id.value==MAG_ID && c1.ok && c1.value==target
                && c2.ok && (c2.value&0x3F)==RANGE8G;
  if(!exact) ++ctr.verifyFail;
  if(id.ok && id.value==MAG_ID && c1.ok && c2.ok && c1.value==0x00 && c2.value==0x00) {
    ++ctr.resetSignatures;
    Serial.println("RESET_SIGNATURE: ID valid but C1=C2=0; suspect POR/soft reset (NOT confirmed)");
  }
  // Never use a stale previous verification as if it certified the current mode.
  configured=exact;
  Serial.printf("VERIFY stage=%s id=%s0x%02X C1=%s0x%02X target=0x%02X C2=%s0x%02X ok=%d "
                "phase=%u,%u,%u bytes=%u,%u,%u\n",
    stage,id.ok?"":"ERR/",id.value,c1.ok?"":"ERR/",c1.value,target,
    c2.ok?"":"ERR/",c2.value,exact,id.phase,c1.phase,c2.phase,
    id.received,c1.received,c2.received);
  return exact;
}

// Non-destructive evidence: if QMC reads fail, do other devices also fail?
// Execute immediately after first failed verify (no mode changes / reboots).
void postStartBusDiagnosis() {
  const uint8_t magAck=probe(MAG), mpuAck=probe(MPU), bmpAck=probe(0x76);
  const ByteRead magRep=readByteMode(MAG,ID,false);
  const ByteRead magStop=readByteMode(MAG,ID,true);
  const ByteRead mpuRep=readByteMode(MPU,0x75,false);
  const ByteRead mpuStop=readByteMode(MPU,0x75,true);
  const ByteRead bmpStop=readByteMode(0x76,0xD0,true);
  Serial.printf("POST_START_BUS mag_ack=%u mpu_ack=%u bmp_ack=%u "
                "mag_rs=%d,0x%02X,%u,%u mag_stop=%d,0x%02X,%u,%u "
                "mpu_rs=%d,0x%02X mpu_stop=%d,0x%02X bmp_stop=%d,0x%02X "
                "SDA=%d SCL=%d\n",
    magAck,mpuAck,bmpAck,
    magRep.ok,magRep.value,magRep.phase,magRep.received,
    magStop.ok,magStop.value,magStop.phase,magStop.received,
    mpuRep.ok,mpuRep.value,mpuStop.ok,mpuStop.value,
    bmpStop.ok,bmpStop.value,digitalRead(SDA_PIN),digitalRead(SCL_PIN));
}

// QST QMC5883P Rev E section 7.6: write 0x80 to control register 0x0B.
// This resets internal registers, but does NOT remove VDD or reset the MPU.
// Do this only before configuration, NEVER as an undocumented mode transition.
bool resetMagRegisters() {
#if IQ_MAG_SOFT_RESET
  softResetAttempted=true;
  Serial.printf("MAG_SOFT_RESET stage=REQUEST reg=0x%02X value=0x80 t_ms=%lu\n",
     CTRL2,static_cast<unsigned long>(millis()));
  // Only reset a confirmed QMC5883P; never write to unknown I2C devices.
  const ByteRead before=readByte(MAG,ID);
  if (!before.ok || before.value!=MAG_ID) {
    Serial.println("MAG_SOFT_RESET stage=FAIL reason=IDENTITY_NOT_CONFIRMED");
    return false;
  }
  const bool ack=writeByte(MAG,CTRL2,0x80,"QMC_SOFT_RESET");
  if (!ack) {
    Serial.println("MAG_SOFT_RESET stage=FAIL reason=WRITE_NOT_ACKNOWLEDGED");
    return false;
  }
  const uint32_t began=millis();
  // A reset takes finite time.  The datasheet doesn't prescribe a fixed 10-s
  // power-off here; poll for reset-bit clearance, with bounded safety timeout.
  delay(50);
  while (millis()-began<1000) {
    const ByteRead id=readByte(MAG,ID);
    const ByteRead c1=readByte(MAG,CTRL1);
    const ByteRead c2=readByte(MAG,CTRL2);
    if(id.ok && id.value==MAG_ID && c1.ok && c2.ok && !(c2.value&0x80)) {
      softResetOK=(c1.value==0x00);
      Serial.printf("MAG_SOFT_RESET stage=%s ack=1 id=0x%02X C1=0x%02X C2=0x%02X t_ms=%lu\n",
        softResetOK?"CONFIRMED":"REGS_NOT_DEFAULT",id.value,c1.value,c2.value,
        static_cast<unsigned long>(millis()));
      return softResetOK;
    }
    delay(20);
  }
  Serial.println("MAG_SOFT_RESET stage=FAIL reason=RESET_BIT_OR_ID_READBACK_TIMEOUT");
  return false;
#else
  Serial.println("MAG_SOFT_RESET stage=SKIPPED requested=0");
  return true;
#endif
}

bool configure(uint16_t hz,const char* why) {
  activeHz=hz;
  configured=false;
  prestartOK=false;
  Serial.printf("CONFIG_BEGIN reason=%s requested=%u ctrl1=0x%02X OSR1=%d I2C=%d serial=%d read_stop=%d\n",
    why,static_cast<unsigned>(hz),expectedC1(hz),IQ_MAG_OSR1,IQ_I2C_HZ,IQ_SERIAL_BAUD,IQ_MAG_READ_STOP);
  const uint8_t ack=probe(MAG);
  const ByteRead id=readByte(MAG,ID);
  identityOK=ack==0 && id.ok && id.value==MAG_ID;
  Serial.printf("MAG_ID ack=%u read=%s0x%02X valid=%d\n",ack,id.ok?"":"ERR/",id.value,identityOK);
  if(!identityOK) return false;
  // Datasheet mode transition via suspend. Control register configuration may
  // reset on poor power or bus errors. Diagnose EACH stage separately.
  const bool suspend=writeByte(MAG,CTRL1,0x00,"SUSPEND");
  delay(20);
  const bool sign=writeByte(MAG,SIGN,SIGN_VALUE,"XYZ_SIGN");
  delay(10);
  const bool range=writeByte(MAG,CTRL2,RANGE8G,"SET_RESET_8G");
  delay(10);
  const ByteRead pre1=readByte(MAG,CTRL1);
  const ByteRead pre2=readByte(MAG,CTRL2);
  const bool preOk=pre1.ok&&pre2.ok&&pre1.value==0x00&&(pre2.value&0x3F)==RANGE8G;
  prestartOK=preOk;
  Serial.printf("PRE_START_VERIFY c1=%s0x%02X c2=%s0x%02X ok=%d\n",
    pre1.ok?"":"ERR/",pre1.value,pre2.ok?"":"ERR/",pre2.value,preOk);
  // Guard against mistaken 'ACK == configuration stored' interpretation.
  if(!(suspend&&sign&&range&&preOk)) {
    Serial.println("CONFIG_ABORT prestart_failed; no 200Hz transition attempted");
    return false;
  }
  if (!IQ_MAG_START_ENABLE) {
    Serial.printf("CONFIG_PARKED prestart_ok=1 chip_id=0x%02X c1=0x%02X c2=0x%02X\n",
       id.value,pre1.value,pre2.value);
    return true;
  }
  const bool started=writeByte(MAG,CTRL1,expectedC1(hz),"START_PROFILE");
  configuredAtMs=millis();
  delay(STARTUP_SETTLE_MS);
  const bool ok=started&&verify("AFTER_START_40MS");
  if (!ok) postStartBusDiagnosis();
  Serial.printf("CONFIG_DONE requested=%u writes_ack=%d readback=%d\n",hz,started,ok);
  return ok;
}

void recover(const char* reason) {
#if IQ_RECOVERY_ENABLE
  const uint32_t now=millis();
  if(now-lastAttemptMs<REPAIR_INTERVAL_MS) return;
  lastAttemptMs=now;
  if(ctr.recoveries<IQ_RECOVERY_MAX_ATTEMPTS) {
    ++ctr.recoveries;
    Serial.printf("RECOVERY_ATTEMPT n=%lu reason=%s\n",static_cast<unsigned long>(ctr.recoveries),reason);
    // Reinitialize ESP32 I2C controller; this cannot repair a bad power rail.
    Wire.end();
    delay(20);
    beginWire();
    (void)enableMpuBypass();
    (void)configure(activeHz,"BOUNDED_RECOVERY");
    return;
  }
#if IQ_ALLOW_100HZ_FALLBACK
  if(!fallback && activeHz==200) {
    fallback=true;
    Serial.println("FALLBACK_100HZ: 200Hz NOT WORKING, trying safe 100Hz; do not report 200Hz success");
    Wire.end();
    delay(20);
    beginWire();
    (void)enableMpuBypass();
    (void)configure(100,"100HZ_FALLBACK");
    return;
  }
#endif
  static bool exhausted=false;
  if(!exhausted) {
    exhausted=true;
    Serial.println("RECOVERY_EXHAUSTED: 200Hz remains unverified; inspect 3.3V, bypass wiring and I2C");
  }
#else
  (void)reason;
#endif
}

void poll() {
  const ByteRead s=readByte(MAG,STATUS);
  if(!s.ok) {
    ++ctr.statusFail;
    if(!firstFailurePrinted) {
      firstFailurePrinted=true;
      Serial.printf("FIRST_STATUS_FAILURE t_us=%llu phase=%u received=%u SDA=%d SCL=%d\n",
         static_cast<unsigned long long>(esp_timer_get_time()),s.phase,s.received,
         digitalRead(SDA_PIN),digitalRead(SCL_PIN));
    }
    return;
  }
  ctr.lastStatus=s.value;
  if(s.value&0x02) { ++ctr.overflow; return; }
  if(!(s.value&0x01)) { ++ctr.notReady; return; }
  int16_t xyz[3];
  if(!readXYZ(xyz)) { ++ctr.xyzFail; return; }
  const uint64_t t=static_cast<uint64_t>(esp_timer_get_time());
  if(millis()-configuredAtMs<WARMUP_MS) return; // do not claim startup transients as calibrated measurements
  if(ctr.prevFreshUs) { ctr.totalDeltaUs+=t-ctr.prevFreshUs; ++ctr.deltaCount; }
  ctr.prevFreshUs=t;
  ctr.lastFreshUs=t;
  for(int i=0;i<3;++i) ctr.lastXYZ[i]=xyz[i];
  ++ctr.fresh;
  ++ctr.windowFresh;
  if(ctr.fresh%IQ_MAG_CSV_DIVIDER==0) {
    Serial.printf("CSV,%llu,%d,%d,%d,0x%02X,%d,%u\n",
      static_cast<unsigned long long>(t),xyz[0],xyz[1],xyz[2],s.value,
      configured?1:0,activeHz);
  }
}

void health() {
  // Repeat the exact compiled settings every second, so missing an early boot
  // banner cannot make the host silently accept a stale firmware upload.
  Serial.printf("STATE fw=18 run_nonce=%lu rate=%d osr1=%d serial=%d I2C=%d "
                "read_stop=%d start_enable=%d ctrl1_override=%d wifi_mode=%d bus_load=%d recover=%d "
                "max_attempts=%d csv_divider=%d fallback_allowed=%d soft_reset=%d prestart_ok=%d\n",
      static_cast<unsigned long>(IQ_RUN_NONCE),IQ_MAG_BOOT_PROFILE,IQ_MAG_OSR1,
      IQ_SERIAL_BAUD,IQ_I2C_HZ,IQ_MAG_READ_STOP,IQ_MAG_START_ENABLE,IQ_MAG_CTRL1_OVERRIDE,IQ_WIFI_STRESS_MODE,
      IQ_BUS_LOAD_ENABLE,IQ_RECOVERY_ENABLE,IQ_RECOVERY_MAX_ATTEMPTS,
      IQ_MAG_CSV_DIVIDER,IQ_ALLOW_100HZ_FALLBACK,IQ_MAG_SOFT_RESET,prestartOK?1:0);
  const uint32_t now=millis();
  const uint32_t elapsed=now-windowStartedMs;
  const double rate=(elapsed>0) ? double(ctr.windowFresh)*1000.0/double(elapsed) : 0.0;
  const double avg=(ctr.deltaCount>0 && ctr.totalDeltaUs>0)
                   ? double(ctr.deltaCount)*1e6/double(ctr.totalDeltaUs) : 0.0;
  const double age=ctr.lastFreshUs ? double(static_cast<uint64_t>(esp_timer_get_time())-ctr.lastFreshUs)/1e6 : -1.0;
  Serial.printf("HEALTH selected=%u measured_hz_window=%.2f measured_hz_global=%.2f "
    "fresh=%lu not_ready=%lu overflow=%lu status_fail=%lu xyz_fail=%lu "
    "verify_fail=%lu reset_signature=%lu read_phase_fail=%lu read_zero_bytes=%lu "
    "recoveries=%lu regs_ok=%d fallback=%d soft_reset_attempted=%d soft_reset_ok=%d age_s=%.3f "
    "wifi_mode=%d wifi_active=%d wifi_scan_ok=%lu wifi_scan_fail=%lu uptime_ms=%lu "
    "imu_ok=%lu imu_fail=%lu bmp_ok=%lu bmp_fail=%lu imu_read_max_us=%lu bmp_read_max_us=%lu\n",
    activeHz,rate,avg,static_cast<unsigned long>(ctr.fresh),static_cast<unsigned long>(ctr.notReady),
    static_cast<unsigned long>(ctr.overflow),static_cast<unsigned long>(ctr.statusFail),
    static_cast<unsigned long>(ctr.xyzFail),static_cast<unsigned long>(ctr.verifyFail),
    static_cast<unsigned long>(ctr.resetSignatures),static_cast<unsigned long>(ctr.failedRegisterPhases),
    static_cast<unsigned long>(ctr.zeroByteReads),static_cast<unsigned long>(ctr.recoveries),
    configured?1:0,fallback?1:0,softResetAttempted?1:0,softResetOK?1:0,age,IQ_WIFI_STRESS_MODE,wifiStarted?1:0,
    static_cast<unsigned long>(wifiScanCount),static_cast<unsigned long>(wifiScanFail),
    static_cast<unsigned long>(now),
    static_cast<unsigned long>(imuLoadOk),static_cast<unsigned long>(imuLoadFail),
    static_cast<unsigned long>(bmpLoadOk),static_cast<unsigned long>(bmpLoadFail),
    static_cast<unsigned long>(imuLoadMaxUs),static_cast<unsigned long>(bmpLoadMaxUs));
  windowStartedMs=now;
  ctr.windowFresh=0;
  if(now-configuredAtMs>1500 && (ctr.lastFreshUs==0 || age>1.5)) {
    Serial.println("FRESH_MISSING: no recent valid XYZ conversion");
    recover("NO_FRESH");
  }
}

void setup() {
  for(const uint8_t pin: MOTOR_PINS) pinMode(pin,INPUT);
  Serial.begin(IQ_SERIAL_BAUD);
  delay(1800);
  Serial.println("\n=== IQ v18 QMC5883P AUTOMATED SYSTEM DIAGNOSTIC ===");
  Serial.println("SAFETY motor pins INPUT; no flight outputs");
  Serial.printf("BOOT reset_reason=%d t_ms=%lu\n",int(esp_reset_reason()),static_cast<unsigned long>(millis()));
  Serial.printf("BUILD rate=%d osr1=%d serial=%d I2C=%d csv_divider=%d recover=%d max_attempts=%d fallback=%d soft_reset=%d\n",
    IQ_MAG_BOOT_PROFILE,IQ_MAG_OSR1,IQ_SERIAL_BAUD,IQ_I2C_HZ,
    IQ_MAG_CSV_DIVIDER,IQ_RECOVERY_ENABLE,IQ_RECOVERY_MAX_ATTEMPTS,IQ_ALLOW_100HZ_FALLBACK,IQ_MAG_SOFT_RESET);
  Serial.printf("BUILD wifi_mode=%d wifi_start_delay_ms=%d wifi_scan_interval_ms=%d\n",
    IQ_WIFI_STRESS_MODE,IQ_WIFI_START_DELAY_MS,IQ_WIFI_SCAN_INTERVAL_MS);
  Serial.printf("BUILD bus_load=%d (MPU 500Hz, BMP 50Hz)\n",IQ_BUS_LOAD_ENABLE);
  Serial.printf("BUILD start_enable=%d ctrl1_override=0x%02X\n",IQ_MAG_START_ENABLE,IQ_MAG_CTRL1_OVERRIDE);
  beginWire();
  scanBus();
  (void)enableMpuBypass();
  scanBus();
  if (IQ_WIFI_STRESS_MODE != 0 && IQ_WIFI_START_DELAY_MS == 0) beginWifiTest("BEFORE_MAG");
  (void)resetMagRegisters();
  (void)configure(IQ_MAG_BOOT_PROFILE,"AUTOMATED_WARM_START");
  lastAttemptMs=millis();
  windowStartedMs=millis();
  lastVerifyMs=millis();
  lastHealthMs=millis();
  Serial.println("CSV_HEADER CSV,t_us,x_counts,y_counts,z_counts,status,verified,selected_hz");
  Serial.println("POWER CONTROL is host-managed: label WARM vs COLD; do not send runtime commands");
  Serial.println("WIFI_TEST modes: 0=OFF, 1=AP, 2=periodic STA scans (not production WiFi stack)");
}

void loop() {
  while(Serial.available()>0) (void)Serial.read();
  pollWifiTest();
  static uint64_t lastPollUs=0;
  const uint64_t nowUs=static_cast<uint64_t>(esp_timer_get_time());
  // Failed I2C reads can emit thousands of Arduino Wire errors per run.
  // Once a persistent boot failure is evident, sample the fault at 10 Hz.
  const uint32_t pollPeriod=(ctr.statusFail>=25 && ctr.fresh==0) ? 100000 : POLL_US;
  if(nowUs-lastPollUs>=pollPeriod) {
    lastPollUs=nowUs;
    if(identityOK && IQ_MAG_START_ENABLE) poll();
  }
  sensorBusLoad();
  const uint32_t nowMs=millis();
  if(nowMs-lastVerifyMs>=VERIFY_MS) {
    lastVerifyMs=nowMs;
    if(identityOK && IQ_MAG_START_ENABLE && !verify("PERIODIC")) recover("VERIFY_FAILED");
  }
  if(nowMs-lastHealthMs>=REPORT_MS) {
    lastHealthMs=nowMs;
    health();
  }
  delay(1);
}
} // namespace iq
void setup() { iq::setup(); }
void loop() { iq::loop(); }
