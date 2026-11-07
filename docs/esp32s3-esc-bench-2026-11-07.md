# ESP32-S3 ESC bench setup — 2026-11-07

## Verified wiring supplied by the user

- ESC signal order: GPIO 12, 11, 14, 13.
- Signal protocol: standard PWM at 50 Hz, 1000–2000 µs.
- Motor order is assumed to match the firmware's existing FL, FR, RR, RL order; verify each motor physically.

## Firmware behavior

`firmware/flight_controller/src/main.cpp` configures four ESP32-S3 LEDC channels at 50 Hz and 14-bit resolution. At boot and while disarmed, it sends 1000 µs to every ESC. A serial `MTEST 1` through `MTEST 4` command sends 1100 µs to only that motor for one second, then returns all outputs to 1000 µs. `STOP` ends the pulse early. If an LEDC duty write or update reports an error, the firmware latches the output fault, disarms, cancels a bench pulse, resets controller state and repeatedly attempts minimum pulses; re-arming and bench commands remain inhibited until reboot.

The flight-control candidate now uses `XboxSeriesXControllerESP32_asukiaaa` 1.1.3 over BLE. Stick mapping is left X→roll, left Y→pitch, right X→yaw, right Y→vertical-speed command; RT requests boost and LT reduces the vertical-speed command. The library runs in a separate FreeRTOS task. Commands older than 150 ms become invalid and disarm. Arming requires holding D-pad Up + LB + RB for one second while the right stick is held fully down and both triggers are released. D-pad Down or B disarms. Wi-Fi cannot arm the motors.

While disarmed, fresh stick and trigger values are printed at 5 Hz to the 921600-baud serial monitor for props-off direction checks. The PlatformIO monitor default is set to the same baud rate.

Disarming now bypasses the mixer, resets PID and throttle state, and initializes the stored motor slew state to the 10% armed-idle fraction. This prevents commands calculated during a disarmed interval from becoming the first motor command after arming.

Build from the repository root:

```powershell
pio run -d firmware/flight_controller
```

Upload only with all propellers removed:

```powershell
pio run -d firmware/flight_controller -t upload
```

Open the serial monitor at 921600 baud and send one command at a time:

```powershell
pio device monitor -p COM3 -b 921600
```

Use `MTEST 1`, `MTEST 2`, `MTEST 3`, or `MTEST 4`; `STOP` cancels the active pulse. The bench pulse is deliberately fixed at 1100 µs and one second.

## Flight readiness

Xbox arming is implemented, but has not been validated with the user's controller or aircraft. The quaternion initialization error was fixed. The carried-forward PID gains and 45% hover collective are model candidates, not flight measurements; the normal 35% command cap may be below the actual hover point. The BMP280 has no captured ground-pressure reference, magnetometer heading fusion is disabled, and motor torque-to-command authority has not been measured. A successful build or motor spin does not establish flight readiness.

## Verification and limits

PlatformIO successfully compiled the ESP32-S3 DevKitC-1 N8 target with Espressif32 8.0.0 / Arduino-ESP32 3.3.12, XboxSeriesXControllerESP32_asukiaaa 1.1.3, and NimBLE-Arduino 2.5.2. No firmware was flashed and no physical ESC, controller, or motor test was performed. GPIO wiring and 50 Hz / 1000–2000 µs were supplied by the user; signal timing, motor order, spin direction, ESC endpoints, controller pairing, stick signs, failsafe timing, and flight behavior remain unvalidated.
