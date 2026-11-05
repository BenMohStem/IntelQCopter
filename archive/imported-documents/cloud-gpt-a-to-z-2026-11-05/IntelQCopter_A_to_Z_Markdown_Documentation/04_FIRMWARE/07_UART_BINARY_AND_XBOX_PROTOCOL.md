# UART communications and command framing

## Historical calibration stream

The full calibration recorder used **52-byte binary frames** near 500 Hz with raw MPU, QMC and BMP values, flags, timestamps/sequence information, and CRC validation. At 52×500 = 26,000 payload bytes/s, 8N1 line use is roughly 260 kbit/s, below a 921600-baud CH343 link's nominal bit rate. This does not prove Windows capture lacks buffering or resets; verify packet loss and CRC.

## v2 FC telemetry

The FC bench project emits predominantly **ASCII diagnostics at low rate** (`FC`, `FC_TIMING`, etc.), while the serial RX path parses Xbox messages. It is **not a drop-in replacement** for the full calibration GUI binary feed. Use the correct tool for whichever firmware is on the board.

## Xbox `$IQ2` command

`$IQ2,<seq>,<throttle>,<roll>,<pitch>,<yaw>,<vertical>,<buttons>*<4hexCRC16>` plus newline. CRC-16/CCITT computed on the payload after `$` up to `*`. The parser checks count, bounds, finite numbers, monotonic sequence and age. Historical `$IQ` accepted with vertical=0 compatibility. Command setpoint fields are normalized and need documented physical scaling in the FC.

## Serial sharing hazard

The v2 Xbox bridge and v2 terminal capture cannot both open the same COM port at once. The real Wi-Fi stack remains disabled, so there is no secondary network link for simultaneous operation. A serious next iteration may require separate control UART and diagnostics UART or an explicitly multiplexed protocol with priority and rate budget.

## Reset hazard

DTR/RTS behavior on CH343 can reset ESP32; a programming baud speed change is not a field control setting. Cold-power cycles should be logged and distinguished from reset-only events.
