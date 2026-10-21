#include <Arduino.h>
extern "C" void run_scenario(void);
void setup() {
    Serial.begin(115200);
    delay(100);
    run_scenario();
}
void loop() { delay(1000); }
