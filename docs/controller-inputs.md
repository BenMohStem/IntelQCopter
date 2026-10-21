# Hand-controller input and radio frame

The RP2040 hand-controller firmware samples controls at 100 Hz and creates one fixed 22-byte command frame per sample. At the moment it prints each tenth frame over USB for bench inspection; it does **not** transmit over LoRa. The encoder/parser is embedded in both [`flight_controller.c`](../firmware/flight_controller/flight_controller.c) and [`handler_controller.c`](../firmware/handler_controller/handler_controller.c). A host check requires the two wire-protocol sections to remain identical.

## Proposed Pico pin map

This pin map is a software bring-up proposal, not a verified PCB schematic. It assumes a standard Pico plus one 74HC4051 analog multiplexer so all six analog controls can share the Pico's external ADC inputs.

| Signal | Pico GPIO | Function |
|---|---:|---|
| 74HC4051 analog output | GP26 / ADC0 | Selected analog channel |
| 74HC4051 select S0/S1/S2 | GP6 / GP7 / GP8 | Select mux input |
| Buttons LB, RB, north, east, south, west | GP9–GP14 | Active-low, internal pull-ups |
| Mux input 0–5 | — | Left X, left Y, right X, right Y, LT, RT |
| Future LoRa SPI0 | GP16–GP19 | Candidate only; module pins not verified |
| Future LoRa DIO/reset | GP20–GP21 | Candidate only; module pins not verified |

Check the selected RP2040 board's pinout before wiring; do not connect the mux output and another analog source to GP26 at the same time. The six analog controls are read twice per channel to discard the ADC conversion immediately after mux switching.

## Default stick interpretation

This uses the common Mode 2 layout: left X commands yaw rate, left Y commands vertical speed, right X commands roll angle, and right Y commands pitch angle. Stick centers are sampled once at startup, so power up with both sticks centered and both triggers released. A 4% dead band suppresses center noise. Move left Y upward for climb and downward for descent, subject to the analog wiring sign; confirm this on the printed USB frames before connecting any radio.

Input processing also applies cubic expo 0.25, a 35 ms low-pass filter and a
3/s slew limit, with rejected invalid ADC values/time steps. Trigger endpoints
support either wiring direction; full-range capture and persistent calibration
remain unfinished (startup currently assumes ADC rails).

RT progressively raises the permitted vertical speed from 2 to 3 m/s and the
collective ceiling from 0.65 to 0.80. Boost ramps up over 250 ms and releases over
100 ms. A centered vertical stick still holds height: RT alone is not a thrust
kick or acrobatic mode. The allocator preserves motor torque ratios and adjusts
collective to leave correction authority; finite motor limits still apply.

RB + north held 1 s requests takeoff; LB + south held 1 s requests landing. Both
require centered sticks and released RT, and fire once per hold. For now these
requests appear in USB debug logs; the physical radio/companion request bridge is
not connected. LT is reserved. LT/RT use 0–1000 analog values. LB/RB and the four additional buttons are encoded as six separate bits. Arming means explicitly allowing the flight controller to drive motors. It is
separate from requesting takeoff: an armed motor can spin while still on the
ground. Physical arming remains disabled as requested. None of these buttons
currently enables physical motors or changes the flight mode. The flight firmware hardcodes `arm_request=false` and has no arming state machine.

## Packet and failsafe

The little-endian command frame contains magic bytes, protocol version, sequence number, normalized roll/pitch/yaw-rate, centered throttle position, LT/RT values, six button bits, and CRC-16/CCITT-FALSE. The RP2350 parser rejects wrong length/version/CRC, out-of-range values, duplicate or old sequence numbers, and commands older than 150 ms. Its core-1 receive function is still a stub, so this validation is host-tested but not yet connected to a physical LoRa receiver.

The LoRa radio model, frequency/band, module voltage, antenna, legal transmit power, and link budget must be confirmed before enabling RF. Two CC1101 modules remain unused. ROS 2 belongs on the Linux companion computer (Raspberry Pi or Jetson); it will send bounded autonomy setpoints to the RP2350 through a separately defined wired interface. It does not run on either Pico.
