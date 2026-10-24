#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
sensors_event_t acceleration;
// sensors_event_t rotation;
// double avrg_xacc =  3.350011f;
// double avrg_yacc = -0.411099f;
// double avrg_zacc = -0.716789f;
/*
+Z:
Acceleration x = 3.98873996734619140625 m/s^2; avrg = 4.07511502630710609196
Acceleration y = 0.21547815203666687012 m/s^2; avrg = 0.20177829112187029015
Acceleration z = 8.83699893951416015625 m/s^2; avrg = 8.85224291868209789413
-Z:
Acceleration x = -1.71185421943664550781 m/s^2; avrg = -1.69509672316312798301
Acceleration y = -0.40940847992897033691 m/s^2; avrg = -0.39968013050556183696
Acceleration z = -10.05564689636230468750 m/s^2; avrg = -10.08394106149673419281
+X:
Acceleration x = 1.83635270595550537109 m/s^2; avrg = 1.86468975150585181133
Acceleration y = 9.74918937683105468750 m/s^2; avrg = 9.81029463844299343123
Acceleration z = -1.02711248397827148438 m/s^2; avrg = -1.01952218657135973423
-X:
Acceleration x = 0.94570964574813842773 m/s^2; avrg = 0.99117339883595700911
Acceleration y = -9.85214042663574218750 m/s^2; avrg = -9.87512305355071973167
Acceleration z = -0.28969839215278625488 m/s^2; avrg = -0.27665287937144750208
+Y:
Acceleration x = -8.16901588439941406250 m/s^2; avrg = -8.15901172018051212831
Acceleration y = 0.48602294921875000000 m/s^2; avrg = 0.46034417325668036680
Acceleration z = 2.14041638374328613281 m/s^2; avrg = 2.02784522942900657938
-Y:
Acceleration x = 10.72602367401123046875 m/s^2; avrg = 10.68163785209655713970
Acceleration y = -0.67995327711105346680 m/s^2; avrg = -0.67261385777629911775
Acceleration z = -3.35427665710449218750 m/s^2; avrg = -3.33701776551008233440

*/
double avrg_xacc = 0.f;
double avrg_yacc = 0.f;
double avrg_zacc = 0.f;
bool mpu_started = false;
bool i2c_started = false;

void setup(void)
{
    Serial.begin(115200);

    // Keep motor signal pins inactive. No motor commands are generated.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    i2c_started = Wire.begin(9, 8, 100000);
    Serial.printf("I2C controller started : %s.\n", (i2c_started) ? "Yes" : "No");

    if (i2c_started)
    {
        mpu_started = mpu.begin(0x68, &Wire);
    }
    Serial.printf("MPU6050 initialized : %s.\n", (mpu_started) ? "Yes" : "No");
    delay(500);

    for (int i = 0; i < 10000; i++)
    {
        mpu.getAccelerometerSensor()->getEvent(&acceleration);
        avrg_xacc += acceleration.acceleration.x;
        avrg_yacc += acceleration.acceleration.y;
        avrg_zacc += acceleration.acceleration.z;
        delayMicroseconds(50);
    }

    avrg_xacc /= 10000.f;
    avrg_yacc /= 10000.f;
    avrg_zacc /= 10000.f;
}

void loop(void)
{
    Serial.printf("ESP32 is running, uptime = %lu ms\n", millis());
    if (i2c_started && mpu_started)
    {
        mpu.getAccelerometerSensor()->getEvent(&acceleration);
        Serial.printf("Acceleration x = %.20f m/s^2; avrg = %.20f\n", acceleration.acceleration.x, avrg_xacc);
        Serial.printf("Acceleration y = %.20f m/s^2; avrg = %.20f\n", acceleration.acceleration.y, avrg_yacc);
        Serial.printf("Acceleration z = %.20f m/s^2; avrg = %.20f\n", acceleration.acceleration.z, avrg_zacc);

        // sensors_event_t rotation;
        // mpu.getGyroSensor()->getEvent(&rotation);
        // Serial.printf("Gyro Z = %.3f rad/s\n", rotation.gyro.z);
    }
    delay(1000);
}