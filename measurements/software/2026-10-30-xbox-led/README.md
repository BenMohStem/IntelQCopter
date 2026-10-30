# Xbox connection indicator and simpler test selection

Current installed motor-disabled test-menu firmware, 30 October 2026.

- Green fast blink: searching, 80 ms on / 80 ms off (about 6 flashes/s).
- Green slow blink: connected, waiting for the first input report.
- Green steady: connected and at least one report received.
- DevKitC-1 RGB GPIO48 is assumed; the separate fixed power LED cannot be controlled. Some board revisions use another RGB GPIO: override XBOX_LED_GPIO only after checking the actual board.
- The slow-core library connection call may temporarily pause LED updates during a connection attempt. This is not a hardware heartbeat or flight-readiness signal.

RECENT_INPUT means a report arrived within 300 ms. It can become zero while the controller is idle; it no longer resets the menu or changes the connected LED. Connected alone is not a future flight failsafe: command freshness/loss policy still requires separate validation before motors.

Discovery now requests active scan responses using the existing library. No claim that pairing is fixed until observed on this controller. SCANNING is printed for diagnosis. Reconnection remains managed by the Xbox library; no bonds are erased by our added code.

## Run a test

Close Serial Monitor, then:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-xbox-menu --port COM3
```

Pair the Xbox to the ESP32, not the PC. D-pad LEFT/RIGHT selects the test shown in the terminal. Initial selection is DIRECT_NORMAL. Press A to start it. Keep the drone stationary for four seconds and wait for Saved test. B cancels; Y shows selection/information. Menu also advances selection. Release buttons between presses. Records save automatically; Ctrl+C exits.

Choices: DIRECT_NORMAL, FIFO_NORMAL, DIRECT_PAUSES, FIFO_PAUSES. The last two deliberately inject acquisition pauses; they are not normal control modes. Selection does not start a test until A is pressed. A held button does not repeat.

## Evidence and limits

See build.log and boot.json for the exact new source and actual checks. Two initial compile attempts failed (API/header mismatch and private API usage); originals retained and corrections made using this pinned Arduino core's neopixelWrite and public NimBLE scan API. Prior four capture results belong to the earlier source and do not automatically validate this changed source. Physical LED color/blink, button use, and connected-load timing require new observation. No PID, PWM, motor output or calibration change. Production firmware/main.cpp user edit remains untouched and stale in its old evidence snapshot.

Upload retry succeeded after the operator closed the recorder. New source fingerprint matched on the device; real serial status reported CONNECTED=1, RECENT_INPUT=0, SCANNING=0 twice while idle. About 148 kB free heap observed. Physical LED appearance and button selection still await operator observation. Failed busy-port upload retained privately.

