# ESP32-S3 ESC bench setup — 2026-11-07

## Verified wiring supplied by the user

- ESC signal order: GPIO 12, 11, 14, 13.
- Signal protocol: standard PWM at 50 Hz, 1000–2000 µs.
- Motor order is assumed to match the firmware's existing FL, FR, RR, RL order; verify each motor physically.

## Firmware behavior

`firmware/flight_controller/src/main.cpp` configures four ESP32-S3 LEDC channels at 50 Hz and 14-bit resolution. At boot and while disarmed, it sends 1000 µs to every ESC. A serial `MTEST 1` through `MTEST 4` command sends 1100 µs to only that motor for one second, then returns all outputs to 1000 µs. `STOP` ends the pulse early.

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

Flight arming is blocked because this firmware has no live pilot-input parser or receiver connection. The WebSocket arm request cannot arm it, and a stale or invalid pilot command forces disarm. The carried-forward PID gains are simulation-only, and the barometer has no saved ground reference. This code is therefore for a props-off ESC signal check only; successful compilation or motor spin does not establish flight readiness.

## Verification and limits

PlatformIO successfully compiled the ESP32-S3 DevKitC-1 N8 target with Espressif32 8.0.0 / Arduino-ESP32 3.3.12. No firmware was flashed and no physical ESC or motor test was performed. GPIO wiring and 50 Hz / 1000–2000 µs were supplied by the user; signal timing, motor order, spin direction, ESC endpoints, and fail-safe behavior still require props-off hardware verification.
