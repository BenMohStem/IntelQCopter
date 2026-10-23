# Step01 — see that the programme is running

## Goal

Print `ESP32 is running` once per second. That is the entire first lesson.

## Read main.cpp from top to bottom

- `#include <Arduino.h>` provides Arduino functions such as Serial, pinMode and delay.
- `setup()` runs once after boot/reset.
- The four `pinMode(..., INPUT_PULLDOWN)` lines keep the known motor-signal GPIOs as inputs pulled low. They do not generate ESC commands. We keep these four lines explicit instead of introducing arrays and loops yet. This is a software configuration, not an electrical bench measurement.
- `Serial.begin(115200)` starts serial communication at115200baud.
- `loop()` repeats after setup finishes.
- `Serial.println(...)` sends one line of text.
- `delay(1000)` waits1000milliseconds, which is one second.

## Later, on the real board

With propellers removed, select the actual ESP32 serial port, upload deliberately, and open Serial Monitor at115200baud. Expected output:

```text
ESP32 is running
ESP32 is running
ESP32 is running
```

Each line should arrive approximately one second apart. Keep the actual log and say whether it matches. No fabricated sample output is hardware evidence: the text above is only the expected result.

The one-second delay is useful for this lesson. It will not be used as the timing method for the eventual500Hz flight controller.

**Stop here until this small programme is clear.** Step02 adds only an uptime value. No sensor code is hidden in step01.
