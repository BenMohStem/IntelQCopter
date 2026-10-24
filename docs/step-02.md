# Step 02 - print time since boot

## One new idea: millis()

`millis()` returns how many milliseconds have elapsed since the programme started. One second is 1000 milliseconds. Resetting the board starts this count again. It is not the clock time or date.

## The only firmware change

Replace the old `Serial.println("ESP32 is running");` with:

```cpp
    Serial.print("ESP32 is running, uptime = ");
    Serial.print(millis());
    Serial.println(" ms");
```

Read these three instructions in order:

1. `Serial.print(...)` writes the sentence without starting a new line.
2. `Serial.print(millis())` writes the current uptime after that sentence.
3. `Serial.println(" ms")` writes the unit and finishes the line.

The existing `delay(1000)` still waits approximately one second between messages. Nothing else in the firmware changes. Motor pins remain inputs pulled low, with no motor commands.

## Upload and check it yourself

First confirm step01 printed its original sentence approximately once per second. If it did not, fix that step before testing this one; its original implementation remains in Git commit `2144a52`.

Close any Serial Monitor using the board port, click PlatformIO **Upload**, then open **Monitor** at 115200 baud after upload succeeds. Keep propellers removed. The assistant has not uploaded or run this lesson on hardware.

Illustrative expected output (your first number can differ):

```text
ESP32 is running, uptime = 25 ms
ESP32 is running, uptime = 1025 ms
ESP32 is running, uptime = 2025 ms
```

Subtract one number from the next: it should be approximately 1000 ms. Small differences are normal because printing and other execution take time. This is not yet a precise periodic scheduler.

Press RESET/EN once: the uptime should return to a small value. Preserve the actual monitor output and report the result; the example above is not test evidence.

`millis()` wraps to zero after about 49.7 days on this board. Later timing lessons will handle that using unsigned subtraction. No timing scheduler is added here.

## Status

Prepared only. No build, flash or hardware test was performed for this lesson. `implemented: true` in the stage catalogue means code is present, not that a physical test passed. No later sensor lesson is included.
