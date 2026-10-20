#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#define I2C_SDA 9
#define I2C_SCL 8
#define MOTOR_FR 1
#define MOTOR_FL 12
#define MOTOR_RL 14
#define MOTOR_RR 2
#define STATUS_LED 42

#define LOOP_US 2000UL
#define LOOP_DT 0.002f

#define MAX_ANGLE 30.0f
#define MAX_YAW_RATE 150.0f
#define RAMP_MAX 120.0f
#define MIN_US 1000.0f
#define MAX_US 2000.0f
#define FLOOR_US 1050.0f
#define STICK_DZ 0.12f

static volatile bool tickFlag = false;
static hw_timer_t* loopTimer = NULL;
static void IRAM_ATTR onLoopTimer()
{
  tickFlag = true;
}

static Preferences nvs;
static MPU6050 mpu;
static Adafruit_BMP280 bmp;
static bool bmpOK = false;
static Servo esc[4];

static int16_t rax, ray, raz, rgx, rgy, rgz;
static volatile float roll, pitch, yawRate, rollRate, pitchRate, mpuTemp;
static volatile float altitude;
static volatile float levelRoll, levelPitch;
static float gyroBias[3] = {0, 0, 0};
static float accBias[3] = {0, 0, 0};
static bool calValid = false;
static unsigned long lastBaroMs, lastTempMs;

struct PIDC
{
  double kp, ki, kd, outMin, outMax, iMax;
  double integ, prevErr, out;
};

static PIDC pRA = {4.5, 0, 0, -250, 250, 0, 0, 0, 0};
static PIDC pPA = {4.5, 0, 0, -250, 250, 0, 0, 0, 0};
static PIDC pRR = {0.6, 0.9, 0.0002, -400, 400, 120, 0, 0, 0};
static PIDC pPR = {0.6, 0.9, 0.0002, -400, 400, 120, 0, 0, 0};
static PIDC pYR = {4.0, 0.1, 0, -300, 300, 80, 0, 0, 0};

static void pidReset(PIDC& p)
{
  p.integ = p.prevErr = p.out = 0;
}

static void pidRun(PIDC& p, double err, float dt)
{
  p.integ += err * dt;
  if (p.integ > p.iMax) p.integ = p.iMax;
  if (p.integ < -p.iMax) p.integ = -p.iMax;
  double der = (err - p.prevErr) / dt;
  p.prevErr = err;
  double o = p.kp * err + p.ki * p.integ + p.kd * der;
  if (o > p.outMax) o = p.outMax;
  if (o < p.outMin) o = p.outMin;
  p.out = o;
}

static float motorCur[4] = {1000.f, 1000.f, 1000.f, 1000.f};

static void motorsInit()
{
  const int pins[4] = {MOTOR_FR, MOTOR_FL, MOTOR_RL, MOTOR_RR};
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  for (int i = 0; i < 4; i++)
  {
    esc[i].setPeriodHertz(50);
    esc[i].attach(pins[i], (int)MIN_US, (int)MAX_US);
    esc[i].writeMicroseconds((int)MIN_US);
    motorCur[i] = MIN_US;
  }
}

static void motorsMix(float thr, float ro, float pi, float ya)
{
  float t[4];
  t[0] = thr - ro - pi + ya;
  t[1] = thr + ro - pi - ya;
  t[2] = thr + ro + pi + ya;
  t[3] = thr - ro + pi - ya;
  for (int i = 0; i < 4; i++)
  {
    if (t[i] < MIN_US) t[i] = MIN_US;
    if (t[i] > MAX_US) t[i] = MAX_US;
    float d = t[i] - motorCur[i];
    if (d > RAMP_MAX)  d =  RAMP_MAX;
    if (d < -RAMP_MAX) d = -RAMP_MAX;
    motorCur[i] += d;
    esc[i].writeMicroseconds((int)motorCur[i]);
  }
}

static void motorsStop()
{
  for (int i = 0; i < 4; i++)
  {
    motorCur[i] = MIN_US;
    esc[i].writeMicroseconds((int)MIN_US);
  }
}

static void imuRaw()
{
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
}

static void sensorsInit()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println(F("MPU6050 NOT FOUND"));
    while (1) delay(100);
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);
  mpu.setRate(0);

  bmpOK = bmp.begin(0x76);
  if (!bmpOK) bmpOK = bmp.begin(0x77);
  if (!bmpOK) Serial.println(F("BMP280 NOT FOUND"));

  nvs.begin("fc", true);
  gyroBias[0] = nvs.getFloat("gb0", 0);
  gyroBias[1] = nvs.getFloat("gb1", 0);
  gyroBias[2] = nvs.getFloat("gb2", 0);
  accBias[0] = nvs.getFloat("ab0", 0);
  accBias[1] = nvs.getFloat("ab1", 0);
  accBias[2] = nvs.getFloat("ab2", 0);
  levelRoll = nvs.getFloat("lr", 0);
  levelPitch = nvs.getFloat("lp", 0);
  calValid = nvs.getBool("ok", false);
  nvs.end();

  if (!calValid)
  {
    Serial.println(F("No stored calibration, gathering gyro bias (2 s)..."));
    long s[3] = {0, 0, 0};
    int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000)
    {
      imuRaw();
      s[0] += rgx; s[1] += rgy; s[2] += rgz; n++;
      delay(1);
    }
    gyroBias[0] = s[0] / (float)n;
    gyroBias[1] = s[1] / (float)n;
    gyroBias[2] = s[2] / (float)n;
    calValid = true;
    nvs.begin("fc", false);
    nvs.putFloat("gb0", gyroBias[0]); nvs.putFloat("gb1", gyroBias[1]); nvs.putFloat("gb2", gyroBias[2]);
    nvs.putFloat("ab0", accBias[0]);  nvs.putFloat("ab1", accBias[1]);  nvs.putFloat("ab2", accBias[2]);
    nvs.putFloat("lr", levelRoll);    nvs.putFloat("lp", levelPitch);
    nvs.putBool("ok", true);
    nvs.end();
    Serial.println(F("Gyro bias saved"));
  }
  Serial.printf("Bias: %.1f %.1f %.1f  cal=%d\n", gyroBias[0], gyroBias[1], gyroBias[2], calValid);
}

static void attitudeUpdate(float dt)
{
  imuRaw();
  float gx = rgx - gyroBias[0];
  float gy = rgy - gyroBias[1];
  float gz = rgz - gyroBias[2];
  rollRate = gx / 65.5f;
  pitchRate = -gy / 65.5f;
  yawRate = -gz / 65.5f;

  float axf = rax - accBias[0];
  float ayf = ray - accBias[1];
  float azf = raz - accBias[2];
  float accRoll = atan2f(ayf, azf) * RAD_TO_DEG;
  float accPitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * RAD_TO_DEG;

  const float tauG = 0.50f, tauA = 0.05f;
  float kG = tauG / (tauG + dt);
  float kA = dt / (tauA + dt);
  roll = kG * (roll + rollRate * dt) + kA * (accRoll - roll);
  pitch = kG * (pitch + pitchRate * dt) + kA * (accPitch - pitch);

  unsigned long now = millis();
  if (bmpOK && now - lastBaroMs >= 50)
  {
    lastBaroMs = now;
    static float p0 = 0;
    float p = bmp.readPressure();
    if (p0 == 0) p0 = p;
    altitude = 44330.0f * (1.0f - powf(p / p0, 0.1903f));
  }
  if (now - lastTempMs >= 500)
  {
    lastTempMs = now;
    mpuTemp = mpu.getTemperature() / 340.0f + 36.53;
  }
}

static XboxSeriesXControllerESP32_asukiaaa::Core xBoxController;
static volatile float inRoll, inPitch, inYaw, inThr;
static volatile bool ctrlConnected = false;

static inline float normStick(uint16_t v)
{
  float f = ((int32_t)v - 32768) / 32767.0f;
  if (fabsf(f) < STICK_DZ) return 0.0f;
  float m = (fabsf(f) - STICK_DZ) / (1.0f - STICK_DZ);
  return (f < 0 ? -m : m);
}

static void ctrlInit()
{
  xBoxController.begin();
}

static void ctrlBackground()
{
  xBoxController.onLoop();
}

static bool ctrlIsConnected()
{
  return xBoxController.isConnected() && !xBoxController.isWaitingForFirstNotification();
}

static void ctrlUpdateInputs()
{
  const auto& n = xBoxController.xboxNotif;
  inRoll = normStick(n.joyRHori);
  inPitch = -normStick(n.joyRVert);
  inYaw = normStick(n.joyLHori);
  inThr = n.trigRT / 1023.0f;
  if (inThr < 0.03f) inThr = 0;
}

static void slowOpsTask(void* param)
{
  (void)param;
  static unsigned long lastBg = 0;
  static unsigned long lastTelemetryMs = 0;

  while (true)
  {
    ctrlBackground();

    unsigned long now = millis();

    if (ctrlConnected)
    {
      ctrlUpdateInputs();
    }
    ctrlConnected = ctrlIsConnected();

    if (now - lastTelemetryMs >= 250 && Serial.availableForWrite() > 100)
    {
      lastTelemetryMs = now;
      Serial.printf("CTRL,%d,%.2f,%.2f,%.2f,%.2f | IMU,%.1f,%.1f,%.1f,%.1f | ALT,%.1f\n",
        ctrlConnected, (float)inRoll, (float)inPitch, (float)inYaw, (float)inThr,
        (float)roll, (float)pitch, (float)rollRate, (float)pitchRate,
        (float)altitude);
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  loopTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(loopTimer, &onLoopTimer, true);
  timerAlarmWrite(loopTimer, LOOP_US, true);
  timerAlarmEnable(loopTimer);

  motorsInit();
  sensorsInit();
  ctrlInit();

  pidReset(pRA); pidReset(pPA); pidReset(pRR); pidReset(pPR); pidReset(pYR);

  xTaskCreatePinnedToCore(slowOpsTask, "slowOps", 4096, NULL, 1, NULL, 1);

  Serial.println(F("READY"));
}

void loop()
{
  if (tickFlag)
  {
    tickFlag = false;
    attitudeUpdate(LOOP_DT);

    if (ctrlConnected)
    {
      float rollSp = (float)inRoll * MAX_ANGLE + levelRoll;
      float pitchSp = (float)inPitch * MAX_ANGLE + levelPitch;
      float yawSp = (float)inYaw * MAX_YAW_RATE;

      pidRun(pRA, rollSp - roll, LOOP_DT);
      pidRun(pPA, pitchSp - pitch, LOOP_DT);
      pidRun(pRR, pRA.out - rollRate, LOOP_DT);
      pidRun(pPR, pPA.out - pitchRate, LOOP_DT);
      pidRun(pYR, yawSp - yawRate, LOOP_DT);

      float thr = MIN_US + (float)inThr * (MAX_US - MIN_US);
      motorsMix(thr, (float)pRR.out, (float)pPR.out, (float)pYR.out);
    }
    else
    {
      motorsStop();
      pidReset(pRA); pidReset(pPA); pidReset(pRR); pidReset(pPR); pidReset(pYR);
    }
  }
}
