#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Input-only lesson: no sensor changes, PWM, Servo, PID or motor commands.
XboxSeriesXControllerESP32_asukiaaa::Core xbox;
static uint32_t last_print_ms = 0;

void setup(void)
{
    // Keep signal pins from both previous carrier layouts as inputs.
    const uint8_t motor_pins[] = {12, 1, 2, 14, 4, 21};
    for (uint8_t i = 0; i < sizeof(motor_pins) / sizeof(motor_pins[0]); i++)
    {
        pinMode(motor_pins[i], INPUT_PULLDOWN);
    }
    Serial.begin(115200);
    delay(1000);
    Serial.printf("XBOX_INPUT_LESSON_V1,MOTOR_OUTPUTS_DISABLED\n");
    xbox.begin();
}

void loop(void)
{
    xbox.onLoop();
    uint32_t now = millis();
    if ((uint32_t)(now - last_print_ms) < 200)
    {
        delay(1);
        return;
    }
    last_print_ms = now;

    if (!xbox.isConnected())
    {
        Serial.printf("XBOX,CONNECTED=0,INPUT_READY=0\n");
        return;
    }
    if (xbox.isWaitingForFirstNotification())
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,WAITING_FOR_DATA\n");
        return;
    }

    uint32_t age_ms = now - (uint32_t)xbox.getReceiveNotificationAt();
    if (age_ms > 300)
    {
        Serial.printf("XBOX,CONNECTED=1,INPUT_READY=0,STALE,AGE_MS=%lu\n",
            (unsigned long)age_ms);
        return;
    }

    Serial.printf("XBOX,CONNECTED=1,INPUT_READY=1,AGE_MS=%lu\n%s\n",
        (unsigned long)age_ms, xbox.xboxNotif.toString().c_str());
}
