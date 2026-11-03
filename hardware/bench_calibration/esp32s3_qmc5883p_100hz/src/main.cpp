/*
 * IntelQCopter — competition calibration, full MPU/BMP/QMC binary UART.
 * No Wi-Fi, no external INT/DRDY wiring. MAG freshness uses internal 0x09 status.
 *
 * Hardware validated previously by the user's measurements:
 *   MPU6050  0x68: ESP32 SDA=GPIO9, SCL=GPIO8.
 *   BMP280   0x76: reachable through MPU6050 XDA/XCL I2C bypass.
 *   QMC5883P 0x2C: chip ID 0x80 on the main I2C bus.
 *
 * This program never generates motor/ESC output.
 *
 * Architecture:
 *   Arduino loop():  500 Hz target, absolute 2,000 us deadlines.
 *   Sensor reads:    MPU every cycle, QMC 200 Hz low-OSR, BMP ~10 Hz.
 *   Transport:       identical 52-byte CRC-protected v3 packets over
 *                    USB serial and optionally Wi-Fi UDP.
 *   Wi-Fi task:      runs on core 0, owns all UDP calls, NEVER touches I2C.
 *   UART: CH343 921600 baud, 52-byte CRC-framed packets.
 *
 * Note: an attempted host read is NOT proof of a fresh physical ADC sample.
 * Record actual timestamps and validate acquisition jitter offline.
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
#error "Competition calibration must have IQ_WIFI_ENABLED=0"
#endif
#ifndef IQ_I2C_HZ
#define IQ_I2C_HZ 100000
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

namespace iq {
namespace config {
constexpr uint8_t SDA_PIN = 9;
constexpr uint8_t SCL_PIN = 8;
constexpr uint8_t MOTOR_PINS[4] = {12, 11, 14, 13}; // FL, FR, RR, RL: INPUT ONLY
constexpr uint8_t MPU_PRIMARY_ADDRESS = 0x68;
constexpr uint8_t MPU_ALTERNATE_ADDRESS = 0x69;
constexpr uint8_t BMP_PRIMARY_ADDRESS = 0x76;
constexpr uint8_t BMP_ALTERNATE_ADDRESS = 0x77;
constexpr uint8_t MAG_ADDRESS = 0x2C; // QMC5883P, not HMC5883L or QMC5883L
constexpr uint32_t IMU_PERIOD_US = 2000; // 500 Hz scheduling target
constexpr uint32_t BMP_PERIOD_US = 100000; // 10 Hz
constexpr uint32_t MAG_PERIOD_US = 5000; // Poll internal DRDY at the selected 200 Hz ODR
constexpr uint32_t MAG_STALE_MS = 1000;
constexpr uint32_t SERIAL_BAUD = 921600; // CH343 UART, binary v3 packets
constexpr size_t PACKET_SIZE = 52;

// QMC5883P normal-mode 200 Hz low-OSR profile (ID 0x80, C1 0x3D, C2 0x08).
// A 30 s MAG-only run measured 188.84 fresh samples/s; full-system rate is
// not yet measured. C1 0x3D selects ODR 200 Hz / OSR1=1 / OSR2=1.
// C2 0x08: +/-8 gauss + set/reset. 0x29 = 0x06 XYZ sign.
// This is now the FULL sensor calibration application (not MAG-only).
constexpr uint8_t MAG_CONTROL1 = static_cast<uint8_t>(IQ_MAG_CONTROL1);
constexpr uint8_t MAG_CONTROL2 = 0x08;
constexpr uint8_t MAG_XYZ_SIGN = 0x06;

#if IQ_WIFI_ENABLED
constexpr char WIFI_AP_NAME[] = "IntelQCopter-Cal";
// Replace in your private copy before wireless use; >= 8 characters.
constexpr char WIFI_PASSWORD[] = "SET_PRIVATE_VALUE_LOCALLY";
constexpr uint16_t WIFI_CONTROL_PORT = 19006;
constexpr size_t UDP_BATCH_MAX = 16;
#endif
} // namespace config

// ---------------------------- I2C access ----------------------------------
// Wire is owned by this acquisition task; the Wi-Fi task never calls it.
bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool i2cWrite(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t address, uint8_t reg, uint8_t *output, size_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(address), static_cast<int>(count)) !=
      static_cast<int>(count)) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i)
    output[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool i2cReadByte(uint8_t address, uint8_t reg, uint8_t &value) {
  return i2cRead(address, reg, &value, 1);
}

// QMC-specific access uses STOP+START between the register-pointer write and
// read. The integrated captures reported ESP32 Wire repeated-START failures
// after QMC startup (endTransmission(false) / i2cWriteReadNonStop error -1).
// Keep the normal repeated-START helper for MPU/BMP; this changes only the QMC
// transaction method. A successful bus read is still not a fresh conversion:
// updateMagnetometer() requires the internal status register's DRDY bit.
bool i2cReadQmc(uint8_t reg, uint8_t *output, size_t count) {
  Wire.beginTransmission(config::MAG_ADDRESS);
  if (Wire.write(reg) != 1) {
    (void)Wire.endTransmission(true);
    return false;
  }
  if (Wire.endTransmission(true) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(config::MAG_ADDRESS),
                       static_cast<int>(count)) != static_cast<int>(count)) {
    while (Wire.available() > 0) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    if (Wire.available() <= 0) return false;
    output[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}
bool i2cReadQmcByte(uint8_t reg, uint8_t &value) {
  return i2cReadQmc(reg, &value, 1);
}

int16_t signedBigEndian(const uint8_t *b) {
  return static_cast<int16_t>((uint16_t(b[0]) << 8) | b[1]);
}

int16_t signedLittleEndian(const uint8_t *b) {
  return static_cast<int16_t>((uint16_t(b[1]) << 8) | b[0]);
}

// ------------------------------ Measurements -------------------------------
struct ImuReading {
  bool valid = false;
  int16_t accel[3] = {};
  int16_t gyro[3] = {};
  int16_t temperature = 0;
};

struct PressureReading {
  bool valid = false;
  bool fresh = false;
  uint32_t rawPressure = 0;
  uint32_t rawTemperature = 0;
  int16_t temperatureCentiC = 0;
  uint32_t pressureCentiPa = 0;
};

struct MagneticReading {
  bool valid = false;
  bool fresh = false;
  int16_t axes[3] = {};
};

ImuReading currentImu;
PressureReading currentPressure;
MagneticReading currentMagnetic;
Adafruit_BMP280 bmp(&Wire);
uint8_t mpuAddress = 0;
uint8_t bmpAddress = 0;
bool mpuPresent = false;
bool bmpPresent = false;
bool magneticConfigured = false;

// Diagnostic counters are never interpreted as successful measurements.
struct MagnetometerHealth {
  uint32_t attempts = 0;
  uint32_t failures = 0;
  uint32_t successfulConfigurations = 0;
  uint32_t i2cFailures = 0;
  uint32_t noFreshData = 0;
  uint32_t statusReadFails = 0;
  uint32_t notReady = 0;
  uint32_t overflows = 0;
  uint32_t xyzReadFails = 0;
  uint32_t freshConversions = 0;
  uint8_t lastControl1 = 0;
  uint8_t lastControl2 = 0;
  uint32_t lastGoodMs = 0;
  uint32_t lastAttemptMs = 0;
  uint32_t identicalReads = 0;
  uint32_t rawReadSuccess = 0;
  uint16_t consecutiveStatusFailures = 0;
};
MagnetometerHealth magHealth;

uint32_t lastPressurePollUs = 0;
uint32_t lastMagneticPollUs = 0;

// ----------------------------- MPU6050 -------------------------------------
bool initializeMpu6050() {
  mpuAddress = i2cPresent(config::MPU_PRIMARY_ADDRESS)
      ? config::MPU_PRIMARY_ADDRESS
      : (i2cPresent(config::MPU_ALTERNATE_ADDRESS)
            ? config::MPU_ALTERNATE_ADDRESS : 0);
  if (!mpuAddress) return false;
  uint8_t id = 0;
  if (!i2cReadByte(mpuAddress, 0x75, id) || id != 0x68) return false;
  if (!i2cWrite(mpuAddress, 0x6B, 0x01)) return false; // Wake, gyro PLL
  delay(100);
  bool ok = i2cWrite(mpuAddress, 0x6C, 0x00); // Enable all 6 axes
  ok &= i2cWrite(mpuAddress, 0x1A, 0x03);      // DLPF 42/44 Hz
  ok &= i2cWrite(mpuAddress, 0x19, 0x01);      // 1 kHz / 2 = 500 Hz
  ok &= i2cWrite(mpuAddress, 0x1B, 0x00);      // Gyro ±250 deg/s
  ok &= i2cWrite(mpuAddress, 0x1C, 0x00);      // Accel ±2g
  ok &= i2cWrite(mpuAddress, 0x23, 0x00);      // FIFO disabled
  ok &= i2cWrite(mpuAddress, 0x38, 0x00);      // Polled data

  // INT_PIN_CFG (0x37) is an MPU register that enables I2C bypass.
  // It does NOT require wiring an external INT or DRDY pin.
  // Allow ESP32 to access BMP280 through MPU's auxiliary XDA/XCL pins.
  uint8_t userControl = 0, interruptPins = 0;
  if (!i2cReadByte(mpuAddress, 0x6A, userControl) ||
      !i2cReadByte(mpuAddress, 0x37, interruptPins)) return false;
  ok &= i2cWrite(mpuAddress, 0x6A, userControl & uint8_t(~0x60));
  ok &= i2cWrite(mpuAddress, 0x37, interruptPins | 0x02);
  uint8_t divider = 0, filter = 0;
  ok &= i2cReadByte(mpuAddress, 0x6A, userControl);
  ok &= i2cReadByte(mpuAddress, 0x37, interruptPins);
  ok &= i2cReadByte(mpuAddress, 0x19, divider);
  ok &= i2cReadByte(mpuAddress, 0x1A, filter);
  return ok && !(userControl & 0x20) && (interruptPins & 0x02) &&
      divider == 1 && (filter & 0x07) == 3;
}

ImuReading sampleImu6050() {
  ImuReading result;
  if (!mpuPresent) return result;
  uint8_t data[14];
  if (!i2cRead(mpuAddress, 0x3B, data, sizeof(data))) return result;
  for (int i = 0; i < 3; ++i) {
    result.accel[i] = signedBigEndian(data + 2*i);
    result.gyro[i] = signedBigEndian(data + 8 + 2*i);
  }
  result.temperature = signedBigEndian(data + 6);
  result.valid = true;
  return result;
}

// ------------------------------ BMP280 -------------------------------------
bool initializeBmp280() {
  bmpAddress = i2cPresent(config::BMP_PRIMARY_ADDRESS)
      ? config::BMP_PRIMARY_ADDRESS
      : (i2cPresent(config::BMP_ALTERNATE_ADDRESS)
            ? config::BMP_ALTERNATE_ADDRESS : 0);
  if (!bmpAddress) return false;
  uint8_t id = 0;
  if (!i2cReadByte(bmpAddress, 0xD0, id) || id != 0x58) return false;
  if (!bmp.begin(bmpAddress)) return false;
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_63);
  return true;
}

void updatePressure(uint32_t nowUs) {
  currentPressure.fresh = false;
  if (!bmpPresent || uint32_t(nowUs-lastPressurePollUs) < config::BMP_PERIOD_US)
    return;
  lastPressurePollUs = nowUs;
  uint8_t registers[6];
  if (!i2cRead(bmpAddress, 0xF7, registers, sizeof(registers))) {
    currentPressure.valid = false;
    return;
  }
  const float temperature = bmp.readTemperature();
  const float pressure = bmp.readPressure();
  if (!std::isfinite(temperature) || !std::isfinite(pressure) ||
      temperature < -100.f || temperature > 100.f ||
      pressure < 10000.f || pressure > 120000.f) {
    currentPressure.valid = false;
    return;
  }
  currentPressure.rawPressure = (uint32_t(registers[0]) << 12) |
      (uint32_t(registers[1]) << 4) | (registers[2] >> 4);
  currentPressure.rawTemperature = (uint32_t(registers[3]) << 12) |
      (uint32_t(registers[4]) << 4) | (registers[5] >> 4);
  currentPressure.temperatureCentiC = int16_t(lroundf(temperature*100.f));
  currentPressure.pressureCentiPa = uint32_t(lroundf(pressure*100.f));
  currentPressure.valid = true;
  currentPressure.fresh = true;
}

// ----------------------------- QMC5883P ------------------------------------
// Register map verified against QST's datasheet and the Betaflight driver.
// ------------------------------ QMC5883P ------------------------------------
// NO external INT/DRDY pin is used. Register 0x09 is the sensor's internal
// conversion status; QST recommends read 0x09 -> if DRDY -> read XYZ 0x01..06.
// Reproduce the successful v13 fixed-boot sequence, not v14/v15 recovery.
bool configureQmc5883p() {
  ++magHealth.attempts;
  magHealth.lastAttemptMs = millis();
  magneticConfigured = false;
  currentMagnetic.valid = false;
  currentMagnetic.fresh = false;

  uint8_t id = 0;
  if (!i2cReadQmcByte(0x00, id) || id != 0x80) {
    ++magHealth.failures;
    return false;
  }
  // v13 verified order/timing: suspend -> wait 20 ms -> sign -> ctrl2 -> ctrl1.
  if (!i2cWrite(config::MAG_ADDRESS, 0x0A, 0x00)) {
    ++magHealth.failures;
    return false;
  }
  delay(20);
  const bool signOk = i2cWrite(config::MAG_ADDRESS, 0x29, config::MAG_XYZ_SIGN);
  const bool ctrl2Ok = i2cWrite(config::MAG_ADDRESS, 0x0B, config::MAG_CONTROL2);
  const bool ctrl1Ok = i2cWrite(config::MAG_ADDRESS, 0x0A, config::MAG_CONTROL1);
  if (!(signOk && ctrl2Ok && ctrl1Ok)) {
    ++magHealth.failures;
    return false;
  }
  // The v13 boot sequence read back chip ID, C1 and C2 immediately.
  // A missing readback just after startup was transient in v13; it is NOT
  // grounds for cycling the sensor into suspend during acquisition.
  uint8_t id2 = 0, c1 = 0, c2 = 0;
  const bool idOK = i2cReadQmcByte(0x00, id2);
  const bool c1OK = i2cReadQmcByte(0x0A, c1);
  const bool c2OK = i2cReadQmcByte(0x0B, c2);
  magHealth.lastControl1 = c1;
  magHealth.lastControl2 = c2;
  if ((idOK && id2 != 0x80) ||
      (c1OK && c1 != config::MAG_CONTROL1) ||
      (c2OK && (c2 & 0x3F) != config::MAG_CONTROL2)) {
    ++magHealth.failures;
    return false;
  }
  if (!(idOK && c1OK && c2OK)) ++magHealth.i2cFailures;
  magneticConfigured = true;
  magHealth.lastGoodMs = millis();
  ++magHealth.successfulConfigurations;
  lastMagneticPollUs = micros();
  return true;
}

void updateMagnetometer(uint32_t nowUs) {
  currentMagnetic.fresh = false;
  if (!magneticConfigured) { currentMagnetic.valid = false; return; }
// On complete MAG I2C loss, stop saturating the shared sensor bus.
  // Normal operation still polls 0x09 at 200Hz for a 100Hz conversion rate.
  const uint32_t pollUs = magHealth.consecutiveStatusFailures >= 8 ? 100000U :
                          config::MAG_PERIOD_US;
  if (uint32_t(nowUs - lastMagneticPollUs) < pollUs) return;
  lastMagneticPollUs = nowUs;

  // IMPORTANT: status register DRDY is not a physical interrupt pin.
  // Read-to-clear DRDY signals a newly completed measurement, regardless
  // of whether its 16-bit XYZ values happen to equal the previous sample.
  uint8_t status = 0;
  if (!i2cReadQmcByte(0x09, status)) {
    ++magHealth.i2cFailures;
    ++magHealth.statusReadFails;
    if (magHealth.consecutiveStatusFailures < 65535)
      ++magHealth.consecutiveStatusFailures;
  } else if (status & 0x02) {
    magHealth.consecutiveStatusFailures = 0;
    ++magHealth.overflows;
  } else if (!(status & 0x01)) {
    magHealth.consecutiveStatusFailures = 0;
    ++magHealth.notReady;
  } else {
    magHealth.consecutiveStatusFailures = 0;
    uint8_t data[6] = {};
    if (!i2cReadQmc(0x01, data, sizeof(data))) {
      ++magHealth.i2cFailures;
      ++magHealth.xyzReadFails;
    } else {
      int16_t xyz[3];
      for (int i = 0; i < 3; ++i) xyz[i] = signedLittleEndian(data + i*2);
      if (!((xyz[0] == 0 && xyz[1] == 0 && xyz[2] == 0) ||
            (xyz[0] == -1 && xyz[1] == -1 && xyz[2] == -1) ||
            xyz[0] == INT16_MIN || xyz[1] == INT16_MIN || xyz[2] == INT16_MIN)) {
        const bool identical = currentMagnetic.valid &&
            xyz[0] == currentMagnetic.axes[0] &&
            xyz[1] == currentMagnetic.axes[1] &&
            xyz[2] == currentMagnetic.axes[2];
        for (int i=0; i<3; ++i) currentMagnetic.axes[i] = xyz[i];
        currentMagnetic.valid = true;
        currentMagnetic.fresh = true;
        ++magHealth.rawReadSuccess;
        ++magHealth.freshConversions;
        if (identical) ++magHealth.identicalReads; // Still a fresh ADC result.
        magHealth.lastGoodMs = millis();
      } else ++magHealth.noFreshData;
    }
  }
  // Staleness invalidates cached samples but does not repeatedly rewrite
  // registers. Fixed-boot v13 worked without automatic runtime reconfiguration.
  if (uint32_t(millis() - magHealth.lastGoodMs) >= config::MAG_STALE_MS)
    currentMagnetic.valid = false;
}

// ------------------------ Binary telemetry v3 ------------------------------
// Do not change layout: the existing Python GUI expects exactly 52 bytes.
struct PacketWriter {
  uint8_t bytes[config::PACKET_SIZE] = {};
  size_t length = 0;
  void u8(uint8_t value) { bytes[length++] = value; }
  void u16(uint16_t value) { u8(uint8_t(value)); u8(uint8_t(value >> 8)); }
  void i16(int16_t value) { u16(uint16_t(value)); }
  void u32(uint32_t value) { u16(uint16_t(value)); u16(uint16_t(value>>16)); }
};

uint16_t crcCcittFalse(const uint8_t *bytes, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= uint16_t(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021)
                           : uint16_t(crc << 1);
  }
  return crc;
}

struct TimingState {
  int64_t nextDeadlineUs = 0;
  int64_t previousSampleUs = 0;
  uint32_t missedDeadlines = 0;
  uint32_t consecutiveSamples = 0;
  uint32_t observedDtUs = config::IMU_PERIOD_US;
  uint32_t serialDrops = 0;
};
TimingState timing;
uint32_t packetSequence = 0;

#if IQ_WIFI_ENABLED
struct WifiFrame { uint8_t data[config::PACKET_SIZE]; };
QueueHandle_t wifiQueue = nullptr;
WiFiUDP wifiSocket;
uint32_t wifiQueueDrops = 0;
uint32_t wifiSendFailures = 0;

// Networking belongs to core 0. Any delay/drop stays out of I2C acquisition.
void wifiSenderTask(void *) {
  IPAddress receiver;
  uint16_t receiverPort = 0;
  uint32_t lastHandshakeMs = 0;
  uint8_t batch[config::UDP_BATCH_MAX * config::PACKET_SIZE];
  constexpr char HELLO[] = "IQ_HELLO_V1";

  for (;;) {
    const int received = wifiSocket.parsePacket();
    if (received > 0) {
      char text[48] = {};
      const int length = wifiSocket.read(
          reinterpret_cast<uint8_t*>(text), (received < 47 ? received : 47));
      if (length == int(sizeof(HELLO) - 1) &&
          memcmp(text, HELLO, sizeof(HELLO) - 1) == 0) {
        receiver = wifiSocket.remoteIP();
        receiverPort = wifiSocket.remotePort();
        lastHandshakeMs = millis();
        wifiSocket.beginPacket(receiver, receiverPort);
        wifiSocket.write(reinterpret_cast<const uint8_t*>("IQ_READY_V1"), 11);
        wifiSocket.endPacket();
      }
      while (wifiSocket.available()) (void)wifiSocket.read();
    }
    if (receiverPort && uint32_t(millis()-lastHandshakeMs) > 5000)
      receiverPort = 0;

    WifiFrame frame;
    if (xQueueReceive(wifiQueue, &frame, pdMS_TO_TICKS(3)) != pdTRUE)
      continue;
    if (!receiverPort) continue;

    memcpy(batch, frame.data, config::PACKET_SIZE);
    size_t frames = 1;
    // Small bounded batching reduces Wi-Fi interrupts; timestamps unchanged.
    vTaskDelay(pdMS_TO_TICKS(5));
    while (frames < config::UDP_BATCH_MAX &&
           xQueueReceive(wifiQueue, &frame, 0) == pdTRUE) {
      memcpy(batch + frames * config::PACKET_SIZE,
             frame.data, config::PACKET_SIZE);
      ++frames;
    }
    const size_t bytes = frames * config::PACKET_SIZE;
    if (!wifiSocket.beginPacket(receiver, receiverPort) ||
        wifiSocket.write(batch, bytes) != bytes ||
        !wifiSocket.endPacket()) ++wifiSendFailures;
  }
}

bool initializeWifiTelemetry() {
  // Wi-Fi starts BEFORE QMC setup: QMC is reconfigured after the radio is active.
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(config::WIFI_AP_NAME,
                   config::WIFI_PASSWORD, 6, false, 1)) return false;
  wifiQueue = xQueueCreate(256, sizeof(WifiFrame));
  if (!wifiQueue || !wifiSocket.begin(config::WIFI_CONTROL_PORT)) return false;
  return xTaskCreatePinnedToCore(wifiSenderTask, "iq_wifi_telemetry", 6144,
                                  nullptr, 1, nullptr, 0) == pdPASS;
}
#endif

void transmitCurrentMeasurements(uint32_t acquisitionTimestampUs,
                                 uint16_t deadlineLateUs) {
  // Binary v3 flags: 0 MPU valid, 1 BMP valid, 2 MAG valid,
  // 3 BMP fresh, 4 MAG fresh, 5 late, 6 QMC5883P, 7 QMC5883L.
  const uint8_t flags =
      (currentImu.valid ? 0x01 : 0) |
      (currentPressure.valid ? 0x02 : 0) |
      (currentMagnetic.valid ? 0x04 : 0) |
      (currentPressure.fresh ? 0x08 : 0) |
      (currentMagnetic.fresh ? 0x10 : 0) |
      (deadlineLateUs > 250 ? 0x20 : 0) |
      0x40; // Known QMC5883P at 0x2C, packet version 3.

  PacketWriter p;
  p.u8(0xA5); p.u8(0x5A); p.u8(3); p.u8(flags);
  p.u32(packetSequence++);
  p.u32(acquisitionTimestampUs);
  p.u16(deadlineLateUs);
  for (int i=0;i<3;++i) p.i16(currentImu.accel[i]);
  for (int i=0;i<3;++i) p.i16(currentImu.gyro[i]);
  p.i16(currentImu.temperature);
  for (int i=0;i<3;++i) p.i16(currentMagnetic.axes[i]);
  p.u32(currentPressure.rawPressure);
  p.u32(currentPressure.rawTemperature);
  p.i16(currentPressure.temperatureCentiC);
  p.u32(currentPressure.pressureCentiPa);
  p.u16(uint16_t(timing.missedDeadlines));
  if (p.length != config::PACKET_SIZE-2) return;
  p.u16(crcCcittFalse(p.bytes, p.length));

#if IQ_WIFI_ENABLED
  if (wifiQueue) {
    WifiFrame frame;
    memcpy(frame.data, p.bytes, config::PACKET_SIZE);
    if (xQueueSend(wifiQueue, &frame, 0) != pdTRUE) ++wifiQueueDrops;
  }
#endif
  if (Serial.availableForWrite() >= int(config::PACKET_SIZE))
    Serial.write(p.bytes, config::PACKET_SIZE);
  else
    ++timing.serialDrops;
}

// ------------------------- Boot and task loop ------------------------------
void initializeSensorsAndRadio() {
  for (uint8_t pin : config::MOTOR_PINS) pinMode(pin, INPUT);
  delay(1200); // USB and 3.3V rails settle before touching the I2C bus.
  Wire.begin(config::SDA_PIN, config::SCL_PIN);
  Wire.setClock(IQ_I2C_HZ);
  Wire.setTimeOut(30);

  for (int retry=0;retry<5 && !mpuPresent;++retry) {
    mpuPresent = initializeMpu6050();
    if (!mpuPresent) delay(200);
  }
  // Match the successful isolated v13 test as closely as possible:
  // configure the QMC after MPU bypass and before BMP280 traffic.
  (void)configureQmc5883p();

  if (mpuPresent) {
    delay(30); // Bypass needs to settle before asking the BMP for its ID.
    for (int retry=0;retry<3 && !bmpPresent;++retry) {
      bmpPresent = initializeBmp280();
      if (!bmpPresent) delay(100);
    }
  }

#if IQ_WIFI_ENABLED
  const bool wifiReady = initializeWifiTelemetry();
  (void)wifiReady; // USB still works if the Wi-Fi access point cannot start.
  delay(350); // Allow radio startup load to finish BEFORE QMC configuration.
#endif

  // One fixed boot configuration, just like the v13 100Hz success.
  // Never repeatedly suspend/start the sensor during calibration.
  lastMagneticPollUs = micros();
  lastPressurePollUs = micros();
  timing.nextDeadlineUs = esp_timer_get_time() + config::IMU_PERIOD_US;
}

void sampleAtAbsoluteDeadline() {
  const int64_t nowUs = esp_timer_get_time();
  const int64_t untilDeadline = timing.nextDeadlineUs - nowUs;
  if (untilDeadline > 240) {
    delayMicroseconds(uint32_t(untilDeadline - 140));
    return;
  }
  if (untilDeadline > 0) return;

  const int64_t startedAtUs = esp_timer_get_time();
  const int64_t lateness = startedAtUs-timing.nextDeadlineUs;
  if (lateness >= int64_t(config::IMU_PERIOD_US)) {
    const uint64_t skipped = uint64_t(lateness)/config::IMU_PERIOD_US;
    timing.missedDeadlines += uint32_t(skipped > UINT32_MAX ? UINT32_MAX : skipped);
    timing.nextDeadlineUs += int64_t(skipped)*config::IMU_PERIOD_US;
  }
  timing.nextDeadlineUs += config::IMU_PERIOD_US;
  if (timing.previousSampleUs != 0 && startedAtUs > timing.previousSampleUs)
    timing.observedDtUs = uint32_t(startedAtUs - timing.previousSampleUs);
  timing.previousSampleUs = startedAtUs;
  ++timing.consecutiveSamples;

  // MPU first, timestamped at cycle start. Slower sensors only when due.
  currentImu = sampleImu6050();
  updateMagnetometer(uint32_t(startedAtUs));
  updatePressure(uint32_t(startedAtUs));
  const uint16_t late = uint16_t(lateness > 65535 ? 65535 :
                                   (lateness > 0 ? lateness : 0));
  transmitCurrentMeasurements(uint32_t(startedAtUs), late);
}

} // namespace iq

void setup() {
  Serial.begin(iq::config::SERIAL_BAUD);
  iq::initializeSensorsAndRadio();
}

void loop() {
  iq::sampleAtAbsoluteDeadline();
}
