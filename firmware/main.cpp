#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Preferences.h>
#include <string.h>

Adafruit_MPU6050 mpu;
bool mpu_started = false;
bool calibration_ready = false;

struct AccelCalibration
{
    uint32_t version;
    uint32_t measurement_id;
    float bias[3];
    float sensor_to_drone[3][3];
};

// Provisional six-pose fit: output is in DRONE axes, not MPU axes.
// Original measurements and derivation are in measurements/mpu6050/.
static const AccelCalibration measured_calibration =
{
    1,
    0x62bce267u,
    {1.293084598f, -0.079166657f, -0.639507624f},
    {
        {0.046659658f, 0.992539802f, -0.045743511f},
        {-0.955388252f, 0.053316466f, 0.289431328f},
        {0.272504170f, 0.023832247f, 0.951963666f}
    }
};

AccelCalibration calibration = {};

static bool load_or_save_calibration(void)
{
    Preferences storage;
    if (!storage.begin("mpu6050", false))
    {
        Serial.printf("CALIBRATION,flash_open_failed\n");
        return false;
    }

    bool loaded = storage.getBytesLength("accel-v1") == sizeof(calibration);
    if (loaded)
    {
        loaded = storage.getBytes("accel-v1", &calibration, sizeof(calibration)) == sizeof(calibration);
    }

    if (!loaded || memcmp(&calibration, &measured_calibration, sizeof(calibration)) != 0)
    {
        size_t written = storage.putBytes("accel-v1", &measured_calibration, sizeof(calibration));
        size_t restored = storage.getBytes("accel-v1", &calibration, sizeof(calibration));
        bool verified = written == sizeof(calibration) && restored == sizeof(calibration);
        verified = verified && memcmp(&calibration, &measured_calibration, sizeof(calibration)) == 0;
        storage.end();
        Serial.printf("CALIBRATION,%s\n", verified ? "SAVED_AND_READ_BACK" : "SAVE_OR_READ_BACK_FAILED");
        return verified;
    }

    storage.end();
    Serial.printf("CALIBRATION,LOADED_FROM_FLASH\n");
    return true;
}

static void correct_acceleration(const sensors_event_t *event, float drone[3])
{
    float centred[3];
    centred[0] = event->acceleration.x - calibration.bias[0];
    centred[1] = event->acceleration.y - calibration.bias[1];
    centred[2] = event->acceleration.z - calibration.bias[2];

    for (int axis = 0; axis < 3; axis++)
    {
        const float *row = calibration.sensor_to_drone[axis];
        drone[axis] = row[0] * centred[0] + row[1] * centred[1] + row[2] * centred[2];
    }
}

void setup(void)
{
    Serial.begin(115200);

    // Motor signal pins stay inactive. No ESC commands are generated.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    bool i2c_started = Wire.begin(9, 8, 100000);
    Serial.printf("I2C controller started : %s.\n", i2c_started ? "Yes" : "No");
    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.printf("MPU6050 initialized : %s.\n", mpu_started ? "Yes" : "No");
    if (mpu_started)
    {
        calibration_ready = load_or_save_calibration();
        Serial.printf("CALIBRATION,id=%08lx,status=PROVISIONAL,frame=drone,ready=%s\n",
            (unsigned long)measured_calibration.measurement_id, calibration_ready ? "Yes" : "No");
    }
}

void loop(void)
{
    Serial.printf("ESP32 is running, uptime = %lu ms\n", (unsigned long)millis());
    if (mpu_started)
    {
        sensors_event_t acceleration;
        mpu.getAccelerometerSensor()->getEvent(&acceleration);
        Serial.printf("RAW_SENSOR,X=%.4f,Y=%.4f,Z=%.4f,m/s^2\n",
            acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z);
        if (calibration_ready)
        {
            float drone[3];
            correct_acceleration(&acceleration, drone);
            Serial.printf("CORRECTED_DRONE,X=%.4f,Y=%.4f,Z=%.4f,m/s^2\n", drone[0], drone[1], drone[2]);
        }
    }
    delay(1000);
}
