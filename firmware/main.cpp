#include <Arduino.h>

void setup(void)
{
    // Keep motor signal pins inactive. No motor commands are generated.
    pinMode(12, INPUT_PULLDOWN);
    pinMode(1, INPUT_PULLDOWN);
    pinMode(2, INPUT_PULLDOWN);
    pinMode(14, INPUT_PULLDOWN);

    Serial.begin(115200);
}

void loop(void)
{
    Serial.println("ESP32 is running");
    delay(1000);
}
