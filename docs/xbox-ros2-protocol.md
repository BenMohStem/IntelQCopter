# Xbox modes and ROS2 companion link

## Controls

“Bottom of the + buttons” is interpreted as **D-pad Down**. Diagonal Down
positions also count. Neutral means both sticks centred and triggers released.

| Input | Effect |
|---|---|
| Left X | Yaw angular rate, up to ±180°/s |
| Left Y | Vertical speed; centre holds the current height reference |
| Right X / Y | Roll / pitch angle, up to ±35° |
| RT | Smooth additional climb and collective allowance |
| LT | Smooth descent and reduced collective allowance |
| Down + A for 5 seconds | Stable FC mode |
| Down + B for 5 seconds | ROS2 navigation mode |
| Down + LT ≥90% for 2 seconds | Immediate **latched disarm** at the end of the hold |

A hold fires once until released. A different combination, conflicting A+B, or
a report gap greater than 100 ms restarts its timer. Disarm takes priority.
Neither a mode change nor calibration reset clears disarm; a session restart is
required. Selecting a mode is **not physical motor arming**.

Let \(b\) be the smoothed \(RT-LT\), \(s_z\in[-1,1]\) the vertical stick:

\[
v_z^*=s_z(2+|b|)+0.6b\quad[\mathrm{m/s}],\qquad
u_{\max}=0.65+0.15b.
\]

These are provisional response limits. The vertical loop still computes
collective, and the mixer reserves correction headroom. Triggers do not bypass
the controller and write individual motors. In navigation mode, the companion
supplies angles/rates/vertical speed; the Xbox must remain fresh, and trigger
bias/reduction remains available to the pilot. LT is not a hardware power switch.

## Roles and authority

```mermaid
flowchart LR
  X[Xbox / simulated raw HID] --> F[ESP32 C input arbiter]
  PC[Ground PC: start and finish] -->|ROS2 mission/request| N[Drone C++ mission + A*]
  RGB[RGB camera] --> Y[YOLO panel detector]
  Y --> P[Known-size PnP + local pose]
  P --> N
  N -->|28-byte UART command| F
  F --> K[500 Hz estimators and six loops]
  K --> M[Simulated motor plant / disabled physical ESC output]
  F -->|16-byte telemetry| N
```

Stable mode ignores companion setpoints. Navigation accepts fresh companion
setpoints only after the five-second Xbox gesture. Companion age above 150 ms
requests level angles, zero yaw rate and centred vertical command. Xbox age above
250 ms invalidates pilot input and zeroes mathematical demands. This preserves
the project's requested disarm policy; no automatic emergency landing is added.

## Binary interface, version 1

All multibyte values are little endian. Floats are IEEE-754 binary32. CRC is
reflected CRC32/ISO-HDLC (polynomial 0xEDB88320, initial/final XOR 0xFFFFFFFF).

### Companion → FC: 28 bytes

| Offset | Bytes | Value |
|---|---|---|
| 0 | 2 | ASCII QN |
| 2 | 1 | Version 1 |
| 3 | 1 | Flags 1: valid command |
| 4 | 4 | Increasing uint32 sequence |
| 8 / 12 | 4 each | Roll / pitch, degrees, ±35 |
| 16 | 4 | Yaw rate, degrees/s, ±180 |
| 20 | 4 | Vertical speed, m/s, ±3 |
| 24 | 4 | CRC32 of bytes 0–23 |

Malformed length/header/flags/CRC, nonfinite values, out-of-range values and
duplicate/older sequences within a one-second live session are rejected.
After one second without frames a restarted companion sequence can resynchronise.
This CRC detects corruption; it is not cryptographic authentication.

### FC → companion: 16 bytes

| Offset | Bytes | Value |
|---|---|---|
| 0 | 2 | ASCII QS |
| 2 / 3 | 1 each | Version 1 / mode (0 stable, 1 navigation) |
| 4 / 5 / 6 | 1 each | User disarm / pilot fresh / companion fresh |
| 7 | 1 | Bit 0: companion enabled; other bits reserved 0 |
| 8 | 4 | Accepted Xbox report count |
| 12 | 4 | CRC32 of bytes 0–11 |

Firmware Serial2: 115200 baud, RX18/TX17, provisional routing. The portable C++
`uart_bridge_node` accepts a `port` parameter (`COM…` or `/dev/tty…`). Its empty
default opens no device. Electrical wiring, flow control, sustained transfer,
latency and physical computer integration remain untested.

## Raw Xbox report used in tests

16 bytes: six uint16 values at offsets 0–10 (LX, LY, RX, RY, LT, RT), hat at
12, A/B bits 0/1 at 13, remaining bytes reserved. Stick centre is 32768 with
12% deadband; triggers range 0–1023. This matches the pinned library's parser.
The simulator emits this format over ROS2; it does not emulate Bluetooth radio.

## Live sessions and selectable routes

See [live Xbox and companion guide](live-xbox-and-companion.md). The additional
`mission/route` service accepts 2 to 64 geometry points and a cruise height. Middle
points are visited at cruise height before the final landing. The legacy
`mission/request` service remains available for two endpoints.

`FC_ENABLE_COMPANION=0` rejects Down+B and QN frames and disables Serial2 activity.
QS byte 7 advertises this capability; its previous reserved-zero use is superseded.
Status telemetry does not provide the full pose or velocity needed for navigation.


## Navigation completion/disarm extension

QN byte 3 accepts `1` (active command) or `3` (active command plus latched disarm request). Other bits are rejected. CRC, finite-value/range and sequence validation apply before either is accepted. Flag 3 latches disarm only in navigation mode; stable mode ignores its disarm bit. Standalone rejects all companion frames. The simulated ROS2 mission emits flag 3 after its landed/contact dwell completes; it does not rely on a ground-PC test pilot to disarm. The ground-contact topic is synthetic in Gazebo, not a validated real landing sensor. Physical ESC outputs remain disabled. QEMU checks cover valid completion, unknown flags, stable isolation and navigation disarm.
