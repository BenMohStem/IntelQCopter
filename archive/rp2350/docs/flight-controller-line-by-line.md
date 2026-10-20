# flight_controller.c — complete line-by-line teaching guide

**Historical source snapshot at commit 68619b5, before flash persistence and the
35% warning / 20% disarm policy.** For current behavior, see
[the model/integration guide](current-model-and-fc-integration.md) and
[calibration flash guide](calibration-flash.md). Old LAND and volatile-calibration
descriptions below apply only to this snapshot.

This guide explains that snapshot in order. It does not change the firmware
or rerun implementation tests. Every nonblank physical source line appears below;
blank lines are spacing only and are omitted. For a C statement spanning several
lines, the first line explains the complete operation and subsequent lines identify
the continuation. Comments and braces are included. Line numbers refer to this
exact source snapshot, not every future revision.

## Read this first: what actually runs

- **On a Pico:** `main()` creates a core-1 receive queue and schedules a 500 Hz
  target loop. Radio reception is a stub. It passes invalid zero sensor data into
  the math core, which returns zero mathematical demands and a fault. There is no
  ESC driver, physical sensor acquisition, hardware arming, or actual calibrated
  flight pipeline.
- **In simulation:** the host entry points `qc_sitl_reset`, `qc_sitl_step` and
  `qc_sitl_height_observation` let the workstation call the same C control math.
  The simulation supplies measurements and separately uses the resulting normalized
  motor demands to drive its virtual plant. `out.armed` remains false.
- Battery and calibration functions are available utilities in this file. Their
  presence does not mean they are called or wired to hardware in `main()`.
- HMC5883L-style magnetic calibration is present, but neither magnetometer samples
  nor a heading estimate enter `flight_control_step`. There is no absolute-yaw hold.
- The opening Isaac comment is historical. The current active simulation is Gazebo.

## C syntax used repeatedly

| Syntax | Meaning here |
|---|---|
| `float`, `double` | Floating-point numbers; `double` is used for more accurate calibration accumulation. |
| `uint16_t`, `int16_t` | Unsigned/signed integers with exactly 16 bits. |
| `0.5f`, `500u` | Float literal and unsigned integer literal. |
| `typedef struct { ... } name;` | Define a named record type with multiple fields. |
| `const sensor_sample_t *s` | Read through a pointer to a sample; do not modify it through that pointer. |
| `s->field` | Access a member through a pointer; equivalent to `(*s).field`. |
| `&value`, `*pointer` | Take an address; read/write the object at an address. |
| `static` at file scope | Private to this C translation unit; a static object retains its state between calls. |
| `{0}` | Zero-initialize every member, including false Boolean flags. |
| `.kp = ...` | Designated structure-member initialization. |
| `(type){...}` | A compound literal: construct a whole typed object for assignment. |
| `&&`, `||`, `!` | Boolean AND, OR and NOT; AND/OR short-circuit left to right. |
| `&`, `|`, `^`, `~`, `<<` | Bitwise AND, OR, XOR, complement and left shift. |
| `condition ? a : b` | Choose a when condition is true, otherwise b. |
| `return false` | Reject this operation; the calling function decides what happens next. |
| `#if`, `#ifdef`, `#ifndef` | Select which code is compiled; these are not runtime branches. |
| `(void)unused` | Suppress an unused-value warning; it does not transmit, execute or validate anything. |

## Follow the control path before memorizing helpers

```text
sensor sample -> validity -> roll/pitch angle observations -> angle/bias Kalman filters
                         -> height/velocity prediction + fresh height correction
pilot/companion command -> validity + limits -> outer angle P -> inner rate PI
                                           -> yaw rate PI
                                           -> integrate vertical-speed command into height target
                                           -> one altitude PID -> tilt compensation
                                           -> common collective + 4-motor torque allocation
                                           -> mathematical demands, physical armed=false
```

The battery monitor and calibration collectors are separate branches, not steps
automatically executed inside this path. The current file is not re-entrant: its
global filter/PID state represents one aircraft. The simulation/firmware calling
context must serialize calls.

## The six controllers and their units

There are six `pid_t` objects, but not all have nonzero P, I and D gains:

| Controller | Error/input | Output | Source default Kp, Ki, Kd |
|---|---|---|---|
| Roll angle | degrees | desired degrees/s | 3, 0, 0 |
| Roll rate | degrees/s | normalized roll correction | .000829338, .003870245, 0 |
| Pitch angle | degrees | desired degrees/s | 3, 0, 0 |
| Pitch rate | degrees/s | normalized pitch correction | .000829338, .003870245, 0 |
| Yaw rate | degrees/s | normalized yaw correction | .005375614, .008362066, 0 |
| Altitude | metres, with vertical-speed error | normalized collective correction | .108045, .03087, .1225 |

These are the C initialization values. A simulation caller can overwrite all 18
gains through `qc_sitl_reset`; a report's gain scale is not encoded by changing
these initializer lines. They are not identified real-airframe tuning values.

For roll, a target of 10 degrees with estimated angle 7 degrees gives an outer
P output `3*(10-7)=9 degrees/s`. If measured gyro x is 4 degrees/s, the inner
error is `9-4=5 degrees/s`. Its PI calculates a differential motor correction;
9 degrees/s is not a motor duty and 10 degrees is not a gyro-rate command.

For height control, define `e_h = h_ref-h`, then:

```text
v_cmd = (throttle - .5)*2*(2 + boost)
h_ref_new = h_ref_old + v_cmd*dt           [then limited near estimated height]
u_h = Kp*e_h + Ki*integral(e_h dt) + Kd*(v_cmd-v_est)
```

At throttle .5, `v_cmd=0`: the stored reference stops moving, so the controller
holds that reference height. It does not merely hold a fixed motor percentage.
At throttle .75 with no boost, `v_cmd=1 m/s`; at dt=.002 the height target advances
by .002 m per step. There is no seventh vertical-speed PID.

## Why the angle filter is not literally one state

Each axis uses `x=[angle, residual_gyro_bias]^T`. Its prediction is:

```text
angle_minus = angle + dt*(gyro - bias)
bias_minus = bias
F = [[1,-dt],[0,1]]
```

Writing the old covariance as `P=[[a,b],[c,d]]`, direct multiplication gives:

```text
F*P = [[a-dt*c, b-dt*d],[c,d]]
F*P*F^T = [[a-dt*(b+c)+dt²*d, b-dt*d],[c-dt*d,d]]
```

This explains the signs in lines 935–938. Gyro measurement variance contributes
`gyro_noise*dt²` to angle uncertainty. A continuous random-walk bias with intensity
`qb` contributes `[[qb*dt³/3,-qb*dt²/2],[-qb*dt²/2,qb*dt]]`.
The observation is the accelerometer-derived angle with `H=[1,0]`:

```text
S = P00_minus + R
K = [P00_minus, P10_minus]^T / S
innovation = measured_angle - predicted_angle
x_plus = x_minus + K*innovation
```

Thus the usual phrase “1D roll/pitch Kalman” means independent angular axes here;
the implementation is a two-state angle/bias filter for each axis. Kalman gains
are recalculated each update, not manually fixed like PID gains. Their uncertainty
meaning depends on the input/noise assumptions actually being reasonable.

## Altitude filter and covariance correction

Altitude uses `x=[h,v]^T`, `F=[[1,dt],[0,1]]`, external input acceleration `a_up`
and `G=[dt²/2,dt]^T`. Therefore:

```text
x_minus = F*x + G*a_up
P_minus = F*P*F^T + G*q*G^T
q = accel_noise²
G*q*G^T = q*[[dt^4/4,dt³/2],[dt³/2,dt²]]
```

This is exactly the component form at lines 952–965. Acceleration is treated as
an input, not a third state; no vertical acceleration-bias state exists. A fresh
barometer or supplied lidar-derived height then observes `H=[1,0]` and corrects
both height and velocity through the cross covariance.

`covariance_correct` uses Joseph form:

```text
A = I-K*H = [[1-K0,0],[-K1,1]]
P_plus = A*P_minus*A^T + K*R*K^T
```

Keeping the additional `K*R*K^T` term and explicitly restoring off-diagonal
symmetry improves the numerical form of the covariance calculation. It does not
compensate for an incorrect sensor model or prove statistically calibrated errors.

## Scope and unfinished connections to keep in mind

1. No physical MPU6050, BMP280, HMC5883L, LoRa, CC1101, ADC/current sensor,
   TOFSense or ESC driver is called from this file's `main()`.
2. A voltage divider provides voltage, not measured discharge current; current
   input is required by the present coulomb-count SOC function.
3. The battery action enum is not connected to an automatic landing controller.
   It requests LAND in air rather than cutting motors at a low-SOC threshold.
4. Calibration capture results are not automatically loaded into the live filter
   noise parameters, stored in flash, or passed to real sensor acquisition.
5. Gyro rate is used as Euler-axis rate approximately; accelerometer tilt assumes
   gravity-dominated motion. There is no full quaternion/heading estimator here.
6. `sample_is_healthy` checks flags/finiteness, not timestamps or vibration quality.
   There is no complete physical sensor-loss/radio-loss recovery state machine.
7. Individual PID anti-windup does not account for later collective/mixer limiting.
   The mixer preserves relative torque ratios by shifting collective, so the .65/.80
   pre-mixer cap is not an unconditional final throttle/current/headroom guarantee.
8. `main()` aims for 500 Hz, but no physical worst-case execution-time measurement
   or hard deadline guarantee is established. Replaying a simulated .002 s step
   is not evidence that hardware always finishes its work in .002 s.

## Source lines

In each table, source is copied verbatim with Markdown escaping. A continuation
entry refers to the complete multi-line statement, not an additional operation.

Source: [flight_controller.c at 68619b5](https://github.com/BenMohStem/IntelQCopter/blob/68619b5/flight_controller/flight_controller.c) · 1238 physical lines.

Git blob content SHA256: `194580b7235aa4bdccd4c1ccfcbd4dd25500d4d065eb87e21fe294be8fe15314`.


### L1: Purpose, C includes and the 22-byte command protocol

| Line | Exact C source | Explanation |
|---:|---|---|
| 1 | <code>/*</code> | Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 2 | <code> * QCopter flight-controller foundation.</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 3 | <code> *</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 4 | <code> * This file contains hardware-independent estimation and control logic so the</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 5 | <code> * same C implementation can be connected to a future Isaac Sim HIL backend.</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 6 | <code> * Hardware sensor and ESC drivers are intentionally not enabled: board pins,</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 7 | <code> * ESC protocol, frame orientation, and motor order have not been confirmed.</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 8 | <code> * Therefore this firmware remains disarmed and never produces motor outputs.</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 9 | <code> */</code> | Continuation of the statement at L1: Opening documentation: this is a hardware-independent control foundation, not a connected sensor/ESC firmware. The Isaac reference is historical and stale: current simulation uses Gazebo. The executable still deliberately denies physical arming. |
| 10 | <code>#include &lt;math.h&gt;</code> | Include math declarations for sin/cos, atan2, square roots, finite-value checks and min/max operations. |
| 11 | <code>#include &lt;stdbool.h&gt;</code> | Include C Boolean types and the true/false constants. |
| 12 | <code>#include &lt;stdio.h&gt;</code> | Include puts and printf for the Pico USB/serial status messages. |
| 13 | <code>#include &lt;stdint.h&gt;</code> | Include fixed-width integer types used in packets, timestamps and counters. |
| 14 | <code>/* BEGIN WIRE PROTOCOL v1: keep identical in both firmware files. */</code> | Document the command protocol duplicated in the handheld's one-file firmware. Serialization explicitly chooses byte order instead of transmitting an in-memory C struct with possible padding. |
| 15 | <code>/* Fixed-size, endian-stable command frame shared by the two Pico firmwares. */</code> | Continuation of the statement at L14: Document the command protocol duplicated in the handheld's one-file firmware. Serialization explicitly chooses byte order instead of transmitting an in-memory C struct with possible padding. |
| 16 | <code>#include &lt;stdbool.h&gt;</code> | Include stdbool again. Repeating a standard header is harmless but redundant here. |
| 17 | <code>#include &lt;stddef.h&gt;</code> | Include size_t for buffer lengths and NULL for pointer checks. |
| 18 | <code>#include &lt;stdint.h&gt;</code> | Include stdint again; this is redundant but harmless. |
| 20 | <code>#define QCOPTER_FRAME_SIZE 22u</code> | A complete command packet occupies exactly 22 bytes; the unsigned suffix u makes this an unsigned integer constant. |
| 21 | <code>#define QCOPTER_PROTOCOL_VERSION 1u</code> | The packet format version is 1. Both sender and receiver must agree. |
| 22 | <code>#define QCOPTER_AXIS_SCALE 10000</code> | Signed roll, pitch and yaw commands use integer scale 10000: -10000..10000 corresponds to -1..1 before conversion to physical command units. |
| 23 | <code>#define QCOPTER_TRIGGER_SCALE 1000u</code> | Each trigger uses integer scale 1000: 0..1000 means released..fully pressed. |
| 24 | <code>#define QCOPTER_THROTTLE_SCALE 10000u</code> | Throttle uses 0..10000. The midpoint 5000 will become 0.5, meaning zero requested vertical speed. |
| 26 | <code>enum {</code> | Begin an anonymous enumeration that names the six supported button bits. |
| 27 | <code>    QCOPTER_BUTTON_LB = 1u &lt;&lt; 0,</code> | LB occupies bit 0: 1u shifted left zero positions gives mask 0x01. |
| 28 | <code>    QCOPTER_BUTTON_RB = 1u &lt;&lt; 1,</code> | RB occupies bit 1, mask 0x02. |
| 29 | <code>    QCOPTER_BUTTON_NORTH = 1u &lt;&lt; 2,</code> | North occupies bit 2, mask 0x04. |
| 30 | <code>    QCOPTER_BUTTON_EAST = 1u &lt;&lt; 3,</code> | East occupies bit 3, mask 0x08. |
| 31 | <code>    QCOPTER_BUTTON_SOUTH = 1u &lt;&lt; 4,</code> | South occupies bit 4, mask 0x10. |
| 32 | <code>    QCOPTER_BUTTON_WEST = 1u &lt;&lt; 5</code> | West occupies bit 5, mask 0x20. |
| 33 | <code>};</code> | End the button-mask enumeration. |
| 35 | <code>typedef struct {</code> | Begin the C structure describing decoded packet fields, not the on-air bytes. |
| 36 | <code>    uint16_t sequence;</code> | Store a 16-bit packet sequence number, wrapping from 65535 back to 0. |
| 37 | <code>    int16_t roll;</code> | Store the signed roll-stick command in scaled integer units. |
| 38 | <code>    int16_t pitch;</code> | Store the signed pitch-stick command in scaled integer units. |
| 39 | <code>    int16_t yaw_rate;</code> | Store the signed yaw-rate-stick command in scaled integer units. |
| 40 | <code>    uint16_t throttle;</code> | Store the unsigned centered throttle command; its meaning is vertical speed, not direct motor power. |
| 41 | <code>    uint16_t lt;</code> | Store the left trigger value. |
| 42 | <code>    uint16_t rt;</code> | Store the right trigger value; later it supplies the boost command. |
| 43 | <code>    uint16_t buttons;</code> | Store the six button states as a bit mask. |
| 44 | <code>} qcopter_command_frame_t;</code> | Name this structure type qcopter_command_frame_t. |
| 46 | <code>static inline uint16_t qcopter_crc16(const uint8_t *data, size_t length) {</code> | Define the internal CRC function. data points to the bytes; length says how many bytes to check. static keeps the function private to this translation unit; inline permits inlining but does not require it. |
| 47 | <code>    uint16_t crc = 0xFFFFu;</code> | Initialize the 16-bit CRC accumulator to hexadecimal FFFF. |
| 48 | <code>    for (size_t i = 0; i &lt; length; ++i) {</code> | Visit every input byte, from index 0 through length-1. |
| 49 | <code>        crc ^= (uint16_t)data[i] &lt;&lt; 8;</code> | Move the next byte into the CRC's upper eight bits and XOR it into the accumulator. |
| 50 | <code>        for (unsigned bit = 0; bit &lt; 8; ++bit)</code> | Process the eight bits contributed by that byte. |
| 51 | <code>            crc = (crc &amp; 0x8000u) ? (uint16_t)((crc &lt;&lt; 1) ^ 0x1021u)</code> | If the outgoing top bit is 1, shift and XOR polynomial 0x1021; otherwise only shift. Casting back to uint16_t keeps the low 16 bits. This is a bitwise integrity checksum, not encryption or authentication. |
| 52 | <code>                                  : (uint16_t)(crc &lt;&lt; 1);</code> | Continuation of the statement at L51: If the outgoing top bit is 1, shift and XOR polynomial 0x1021; otherwise only shift. Casting back to uint16_t keeps the low 16 bits. This is a bitwise integrity checksum, not encryption or authentication. |
| 53 | <code>    }</code> | Close the per-byte loop. |
| 54 | <code>    return crc;</code> | Return the final checksum. |
| 55 | <code>}</code> | End qcopter_crc16. |
| 57 | <code>static inline void qcopter_put_u16(uint8_t *p, uint16_t value) {</code> | Define a helper that writes a 16-bit unsigned integer to a two-byte destination. |
| 58 | <code>    p[0] = (uint8_t)(value &amp; 0xFFu);</code> | Write its low eight bits first. |
| 59 | <code>    p[1] = (uint8_t)(value &gt;&gt; 8);</code> | Write its high eight bits second: the wire format is little-endian. |
| 60 | <code>}</code> | End the write helper. |
| 62 | <code>static inline uint16_t qcopter_get_u16(const uint8_t *p) {</code> | Define the inverse helper that reads two little-endian bytes. |
| 63 | <code>    return (uint16_t)p[0] &#124; ((uint16_t)p[1] &lt;&lt; 8);</code> | Combine byte 0 with byte 1 shifted eight positions left; return the unsigned 16-bit value. |
| 64 | <code>}</code> | End the read helper. |
| 66 | <code>static inline bool qcopter_frame_values_valid(const qcopter_command_frame_t *f) {</code> | Define semantic validation for a decoded frame, separate from CRC/header validation. |
| 67 | <code>    return f != NULL</code> | Require a non-null frame before reading any field. The following && operations short-circuit. |
| 68 | <code>        &amp;&amp; f-&gt;roll &gt;= -QCOPTER_AXIS_SCALE &amp;&amp; f-&gt;roll &lt;= QCOPTER_AXIS_SCALE</code> | Require roll in the signed integer range -10000..10000. |
| 69 | <code>        &amp;&amp; f-&gt;pitch &gt;= -QCOPTER_AXIS_SCALE &amp;&amp; f-&gt;pitch &lt;= QCOPTER_AXIS_SCALE</code> | Require pitch in the same range. |
| 70 | <code>        &amp;&amp; f-&gt;yaw_rate &gt;= -QCOPTER_AXIS_SCALE &amp;&amp; f-&gt;yaw_rate &lt;= QCOPTER_AXIS_SCALE</code> | Require yaw-rate command in the same range. |
| 71 | <code>        &amp;&amp; f-&gt;throttle &lt;= QCOPTER_THROTTLE_SCALE</code> | Require throttle no greater than 10000; its unsigned type already excludes negative values. |
| 72 | <code>        &amp;&amp; f-&gt;lt &lt;= QCOPTER_TRIGGER_SCALE &amp;&amp; f-&gt;rt &lt;= QCOPTER_TRIGGER_SCALE</code> | Require both triggers no greater than 1000. |
| 73 | <code>        &amp;&amp; (f-&gt;buttons &amp; (uint16_t)~0x003Fu) == 0u;</code> | Reject any button bits outside the supported low six bits. ~ inverts the mask; & isolates unsupported bits. |
| 74 | <code>}</code> | End frame-value validation. |
| 76 | <code>static inline bool qcopter_encode_frame(const qcopter_command_frame_t *f,</code> | Declare the packet encoder: read a const decoded frame and write a 22-byte output buffer. The array parameter does not independently prove the caller actually allocated 22 bytes. |
| 77 | <code>                                        uint8_t out[QCOPTER_FRAME_SIZE]) {</code> | Continuation of the statement at L76: Declare the packet encoder: read a const decoded frame and write a 22-byte output buffer. The array parameter does not independently prove the caller actually allocated 22 bytes. |
| 78 | <code>    if (out == NULL &#124;&#124; !qcopter_frame_values_valid(f)) return false;</code> | Return false if the output pointer is null or the values are invalid. |
| 79 | <code>    out[0] = 0x51u;</code> | Write the first header marker, 0x51, the ASCII letter Q. |
| 80 | <code>    out[1] = 0x43u;</code> | Write the second marker, 0x43, the ASCII letter C. |
| 81 | <code>    out[2] = QCOPTER_PROTOCOL_VERSION;</code> | Write protocol version 1. |
| 82 | <code>    out[3] = 0u;</code> | Write a reserved byte as zero; decoding requires it to stay zero. |
| 83 | <code>    qcopter_put_u16(&amp;out[4], f-&gt;sequence);</code> | Encode the sequence into bytes 4 and 5. |
| 84 | <code>    qcopter_put_u16(&amp;out[6], (uint16_t)f-&gt;roll);</code> | Encode roll into bytes 6 and 7. Converting a negative int16_t to uint16_t produces its modulo-65536 unsigned representation. |
| 85 | <code>    qcopter_put_u16(&amp;out[8], (uint16_t)f-&gt;pitch);</code> | Encode pitch into bytes 8 and 9. |
| 86 | <code>    qcopter_put_u16(&amp;out[10], (uint16_t)f-&gt;yaw_rate);</code> | Encode yaw rate into bytes 10 and 11. |
| 87 | <code>    qcopter_put_u16(&amp;out[12], f-&gt;throttle);</code> | Encode throttle into bytes 12 and 13. |
| 88 | <code>    qcopter_put_u16(&amp;out[14], f-&gt;lt);</code> | Encode LT into bytes 14 and 15. |
| 89 | <code>    qcopter_put_u16(&amp;out[16], f-&gt;rt);</code> | Encode RT into bytes 16 and 17. |
| 90 | <code>    qcopter_put_u16(&amp;out[18], f-&gt;buttons);</code> | Encode the button mask into bytes 18 and 19. |
| 91 | <code>    qcopter_put_u16(&amp;out[20], qcopter_crc16(out, 20u));</code> | Compute the checksum over bytes 0..19 and place its two bytes at offsets 20 and 21. |
| 92 | <code>    return true;</code> | Report successful encoding. |
| 93 | <code>}</code> | End the encoder. |
| 95 | <code>static inline bool qcopter_decode_frame(const uint8_t *data, size_t length,</code> | Declare the decoder: inspect a byte buffer and length, and write a decoded structure only after successful validation. |
| 96 | <code>                                        qcopter_command_frame_t *out) {</code> | Continuation of the statement at L95: Declare the decoder: inspect a byte buffer and length, and write a decoded structure only after successful validation. |
| 97 | <code>    if (data == NULL &#124;&#124; out == NULL &#124;&#124; length != QCOPTER_FRAME_SIZE</code> | Reject null pointers or any length other than 22 before reading packet fields. |
| 98 | <code>        &#124;&#124; data[0] != 0x51u &#124;&#124; data[1] != 0x43u</code> | Reject a packet whose two header markers are not Q and C. |
| 99 | <code>        &#124;&#124; data[2] != QCOPTER_PROTOCOL_VERSION &#124;&#124; data[3] != 0u</code> | Reject an unsupported version or nonzero reserved byte. |
| 100 | <code>        &#124;&#124; qcopter_get_u16(&amp;data[20]) != qcopter_crc16(data, 20u))</code> | Reject the packet if its supplied CRC does not equal the CRC recomputed over the first 20 bytes. |
| 101 | <code>        return false;</code> | Return false on any preceding failed packet check. |
| 102 | <code>    qcopter_command_frame_t f = {</code> | Create a temporary decoded structure, so a rejected packet does not partially overwrite the caller's result. |
| 103 | <code>        .sequence = qcopter_get_u16(&amp;data[4]),</code> | Decode the sequence number. |
| 104 | <code>        .roll = (int16_t)qcopter_get_u16(&amp;data[6]),</code> | Decode roll, converting its unsigned wire representation back to int16_t. This relies on the target compiler's signed conversion behavior for values above INT16_MAX; it works on the project's usual two's-complement targets but is not a portable-C serialization proof. |
| 105 | <code>        .pitch = (int16_t)qcopter_get_u16(&amp;data[8]),</code> | Decode signed pitch using the same conversion. |
| 106 | <code>        .yaw_rate = (int16_t)qcopter_get_u16(&amp;data[10]),</code> | Decode signed yaw-rate command using the same conversion. |
| 107 | <code>        .throttle = qcopter_get_u16(&amp;data[12]),</code> | Decode unsigned throttle. |
| 108 | <code>        .lt = qcopter_get_u16(&amp;data[14]),</code> | Decode LT. |
| 109 | <code>        .rt = qcopter_get_u16(&amp;data[16]),</code> | Decode RT. |
| 110 | <code>        .buttons = qcopter_get_u16(&amp;data[18])</code> | Decode the button mask. |
| 111 | <code>    };</code> | Finish the temporary decoded structure initializer. |
| 112 | <code>    if (!qcopter_frame_values_valid(&amp;f)) return false;</code> | Reject semantically invalid commands even if header and CRC passed. |
| 113 | <code>    *out = f;</code> | Copy the fully validated temporary structure to the caller's output object. |
| 114 | <code>    return true;</code> | Report successful decoding. |
| 115 | <code>}</code> | End the decoder. |
| 117 | <code>/* Valid for sequence gaps shorter than half the 16-bit sequence space. */</code> | Document the half-sequence-space assumption needed to compare wrapping sequence numbers. |
| 118 | <code>static inline bool qcopter_sequence_is_newer(uint16_t value, uint16_t previous) {</code> | Declare the sequence comparator. |
| 119 | <code>    return (int16_t)(value - previous) &gt; 0;</code> | Treat a positive signed 16-bit difference as newer. For example 0 follows 65535 under the target's signed conversion behavior. Exactly half a cycle and longer ambiguities are not resolved; a sender restart is not explicitly negotiated. |
| 120 | <code>}</code> | End the comparator. |
| 121 | <code>/* END WIRE PROTOCOL v1 */</code> | Mark the end of the intentionally duplicated wire protocol. |

### L124: Conditional Pico includes and battery-monitor data

| Line | Exact C source | Explanation |
|---:|---|---|
| 124 | <code>#ifndef QCOPTER_HOST_TEST</code> | Compile Pico-specific includes only when building firmware, not a host simulation/test library. |
| 125 | <code>#include "pico/stdlib.h"</code> | Include Pico timing, sleep and stdio APIs. |
| 126 | <code>#include "pico/multicore.h"</code> | Include the API used to launch core 1. |
| 127 | <code>#include "pico/util/queue.h"</code> | Include Pico's inter-core queue API. |
| 128 | <code>#endif</code> | End that conditional include block. |
| 130 | <code>/* Battery monitoring: types and implementation stay in this subproject file. */</code> | Introduce the pack-monitoring types, kept in the same C file as requested. |
| 131 | <code>#include &lt;stdbool.h&gt;</code> | Include stdbool once more; again redundant. |
| 133 | <code>typedef enum {</code> | Begin the enumeration of requested battery actions. |
| 134 | <code>    BATTERY_ACTION_NORMAL = 0,</code> | Normal battery operation has numeric value zero. |
| 135 | <code>    BATTERY_ACTION_LAND,</code> | LAND is a request for controlled landing, not an immediate motor cut. |
| 136 | <code>    BATTERY_ACTION_LANDED_DISARM,</code> | LANDED_DISARM requests disarming only after the supplied landed flag is true. |
| 137 | <code>    BATTERY_ACTION_CONFIGURATION_FAULT,</code> | CONFIGURATION_FAULT reports invalid monitor settings. |
| 138 | <code>    BATTERY_ACTION_SENSOR_FAULT</code> | SENSOR_FAULT reports invalid measurements or sampling time. |
| 139 | <code>} battery_action_t;</code> | Name the enumeration battery_action_t. |
| 141 | <code>typedef struct {</code> | Begin the structure supplied to a battery monitor update. |
| 142 | <code>    float pack_voltage_v;</code> | Pack voltage in volts, from a future calibrated voltage measurement path. |
| 143 | <code>    float pack_current_a;       /* positive means battery discharging */</code> | Pack current in amperes; positive current means discharge. |
| 144 | <code>    float minimum_cell_voltage_v;</code> | Optional minimum individual-cell voltage. No cell monitor is implemented here. |
| 145 | <code>    float rested_soc_observation_percent; /* from pack-specific rested OCV curve */</code> | Optional SOC observation from a calibrated rested open-circuit voltage curve, expressed in percentage points. |
| 146 | <code>    bool pack_voltage_valid;</code> | Flag that pack-voltage data is valid. |
| 147 | <code>    bool pack_current_valid;</code> | Flag that current data is valid; the current implementation requires this for SOC updates. |
| 148 | <code>    bool cell_voltage_valid;</code> | Flag that the optional individual-cell voltage is available. |
| 149 | <code>    bool rested_soc_observation_valid;</code> | Flag that a rested-pack SOC observation is available. |
| 150 | <code>    bool bms_alert;</code> | Optional external BMS warning input; declaring this flag does not provide a hardware BMS. |
| 151 | <code>    bool aircraft_landed;</code> | Caller-supplied landed status; the battery function does not detect landing itself. |
| 152 | <code>} battery_sample_t;</code> | Name the battery sample type. |
| 154 | <code>typedef struct {</code> | Begin the battery configuration structure. |
| 155 | <code>    float capacity_ah;</code> | Capacity in ampere-hours; the intended 4200 mAh pack corresponds to 4.2 Ah. |
| 156 | <code>    float initial_soc_percent;  /* initialize from verified fully charged pack */</code> | Initial SOC percentage, which must be supplied from a credible starting condition. |
| 157 | <code>    float low_soc_percent;      /* request landing at/below this estimate */</code> | SOC threshold at which the monitor requests landing. The function has no hard-coded 20% default; the caller must configure it. |
| 158 | <code>    float low_cell_voltage_v;</code> | Optional low individual-cell voltage threshold. |
| 159 | <code>    float critical_cell_voltage_v;</code> | Optional critical individual-cell voltage threshold, no greater than the low-cell threshold. |
| 160 | <code>    float low_pack_voltage_v;</code> | Low total-pack voltage threshold. |
| 161 | <code>    float maximum_discharge_current_a;</code> | Maximum permitted measured discharge current. A battery's printed C rating is not used here. |
| 162 | <code>    float voltage_measurement_variance_v2;</code> | Pack-voltage measurement variance R in V squared. |
| 163 | <code>    float voltage_process_variance_v2;</code> | Pack-voltage process variance Q, added once per accepted monitor update, not multiplied by dt. |
| 164 | <code>    float initial_soc_variance_percent2;</code> | Initial SOC uncertainty variance in percentage-points squared. |
| 165 | <code>    float soc_process_variance_percent2_per_s;</code> | SOC process-uncertainty growth per second. |
| 166 | <code>    float soc_measurement_variance_percent2;</code> | Variance R for the optional rested SOC measurement. |
| 167 | <code>    float maximum_sample_dt_s;</code> | Largest accepted interval between battery updates. |
| 168 | <code>} battery_config_t;</code> | Name the battery configuration type. |
| 170 | <code>typedef struct {</code> | Begin the battery status/estimator state structure. |
| 171 | <code>    float filtered_pack_voltage_v;</code> | Estimated filtered total-pack voltage. |
| 172 | <code>    float pack_voltage_sigma_v;</code> | Standard deviation sigma of that voltage estimate, in volts; its square is the covariance. |
| 173 | <code>    float estimated_soc_percent; /* coulomb count, initialized by caller */</code> | Estimated SOC percentage maintained by coulomb counting and optional correction. |
| 174 | <code>    float estimated_soc_sigma_percent;</code> | SOC standard deviation, in percentage points. |
| 175 | <code>    battery_action_t action;</code> | Current requested battery action. |
| 176 | <code>    bool voltage_filter_initialized;</code> | Whether the voltage estimate has been initialized from a measurement. |
| 177 | <code>    bool soc_initialized;</code> | Whether an initial SOC estimate has been supplied. |
| 178 | <code>    bool soc_filter_initialized;</code> | Whether SOC uncertainty has been initialized. |
| 179 | <code>    bool sensor_fault;</code> | Whether the last attempted update detected a sensor fault. |
| 180 | <code>} battery_status_t;</code> | Name the status type. |
| 182 | <code>void battery_monitor_init(battery_status_t *status, float initial_soc_percent);</code> | Declare initialization before its later definition; parameters are a writable status pointer and starting SOC. |
| 183 | <code>battery_action_t battery_monitor_update(battery_status_t *status,</code> | Declare the monitor-update function. It modifies status, reads sample/config through const pointers, takes elapsed seconds, and returns the selected action. |
| 184 | <code>                                        const battery_sample_t *sample,</code> | Continuation of the statement at L183: Declare the monitor-update function. It modifies status, reads sample/config through const pointers, takes elapsed seconds, and returns the selected action. |
| 185 | <code>                                        const battery_config_t *config,</code> | Continuation of the statement at L183: Declare the monitor-update function. It modifies status, reads sample/config through const pointers, takes elapsed seconds, and returns the selected action. |
| 186 | <code>                                        float dt_s);</code> | Continuation of the statement at L183: Declare the monitor-update function. It modifies status, reads sample/config through const pointers, takes elapsed seconds, and returns the selected action. |
| 187 | <code>/* Configurable battery telemetry / warning logic. No hardware cut-off path. */</code> | Clarify that this is monitoring/warning logic with no connected hardware power switch. |
| 190 | <code>#include &lt;math.h&gt;</code> | Repeat the math include; no behavioral change. |
| 191 | <code>#include &lt;stddef.h&gt;</code> | Repeat the size_t/NULL include; no behavioral change. |

### L193: Battery configuration, voltage Kalman filter and SOC

| Line | Exact C source | Explanation |
|---:|---|---|
| 193 | <code>static bool positive_finite(float value) {</code> | Declare a small helper for checking physically positive numeric configuration values. |
| 194 | <code>    return isfinite(value) &amp;&amp; value &gt; 0.0f;</code> | Return true only for finite numbers strictly greater than zero; NaN, infinity, zero and negative values fail. |
| 195 | <code>}</code> | End positive_finite. |
| 197 | <code>static bool config_is_valid(const battery_config_t *c) {</code> | Declare validation of a battery configuration. |
| 198 | <code>    return c != NULL &amp;&amp; positive_finite(c-&gt;capacity_ah)</code> | Require a configuration pointer and positive finite capacity. |
| 199 | <code>        &amp;&amp; isfinite(c-&gt;initial_soc_percent)</code> | Require finite initial SOC. |
| 200 | <code>        &amp;&amp; c-&gt;initial_soc_percent &gt;= 0.0f &amp;&amp; c-&gt;initial_soc_percent &lt;= 100.0f</code> | Bound initial SOC to 0..100 percent. |
| 201 | <code>        &amp;&amp; isfinite(c-&gt;low_soc_percent)</code> | Require finite low-SOC threshold. |
| 202 | <code>        &amp;&amp; c-&gt;low_soc_percent &gt;= 0.0f &amp;&amp; c-&gt;low_soc_percent &lt;= 100.0f</code> | Bound the low-SOC threshold to 0..100 percent. |
| 203 | <code>        &amp;&amp; positive_finite(c-&gt;critical_cell_voltage_v)</code> | Require positive finite critical-cell threshold. |
| 204 | <code>        &amp;&amp; c-&gt;low_cell_voltage_v &gt;= c-&gt;critical_cell_voltage_v</code> | Require the low-cell threshold to be at least the critical-cell threshold. |
| 205 | <code>        &amp;&amp; positive_finite(c-&gt;low_cell_voltage_v)</code> | Require positive finite low-cell threshold. |
| 206 | <code>        &amp;&amp; positive_finite(c-&gt;low_pack_voltage_v)</code> | Require positive finite total-pack low-voltage threshold. |
| 207 | <code>        &amp;&amp; positive_finite(c-&gt;maximum_discharge_current_a)</code> | Require positive finite current limit. |
| 208 | <code>        &amp;&amp; positive_finite(c-&gt;voltage_measurement_variance_v2)</code> | Require positive finite voltage measurement variance, avoiding a zero observation-noise denominator. |
| 209 | <code>        &amp;&amp; c-&gt;voltage_process_variance_v2 &gt;= 0.0f</code> | Allow voltage process variance to be zero, but not negative. |
| 210 | <code>        &amp;&amp; isfinite(c-&gt;voltage_process_variance_v2)</code> | Also reject NaN/infinite voltage process variance. |
| 211 | <code>        &amp;&amp; positive_finite(c-&gt;initial_soc_variance_percent2)</code> | Require positive finite initial SOC variance. |
| 212 | <code>        &amp;&amp; c-&gt;soc_process_variance_percent2_per_s &gt;= 0.0f</code> | Allow nonnegative SOC process-variance growth. |
| 213 | <code>        &amp;&amp; isfinite(c-&gt;soc_process_variance_percent2_per_s)</code> | Require that SOC process-variance growth be finite. |
| 214 | <code>        &amp;&amp; positive_finite(c-&gt;soc_measurement_variance_percent2)</code> | Require positive finite SOC observation variance. |
| 215 | <code>        &amp;&amp; positive_finite(c-&gt;maximum_sample_dt_s);</code> | Require a positive finite maximum update interval, finishing the long Boolean expression. |
| 216 | <code>}</code> | End configuration validation. |
| 218 | <code>void battery_monitor_init(battery_status_t *status, float initial_soc_percent) {</code> | Define battery initialization. |
| 219 | <code>    if (status == NULL) return;</code> | If the status pointer is null, return without dereferencing it. |
| 220 | <code>    *status = (battery_status_t){0};</code> | Overwrite every status member with its zero value using a compound literal; enum action zero means NORMAL. |
| 221 | <code>    if (isfinite(initial_soc_percent)</code> | Accept the requested initial SOC only if finite and within 0..100. |
| 222 | <code>        &amp;&amp; initial_soc_percent &gt;= 0.0f &amp;&amp; initial_soc_percent &lt;= 100.0f) {</code> | Continuation of the statement at L221: Accept the requested initial SOC only if finite and within 0..100. |
| 223 | <code>        status-&gt;estimated_soc_percent = initial_soc_percent;</code> | Store the accepted initial SOC. |
| 224 | <code>        status-&gt;soc_initialized = true;</code> | Mark SOC initialized, but leave covariance initialization for the first valid update. |
| 225 | <code>    }</code> | Close the optional valid-SOC initialization block. |
| 226 | <code>}</code> | End initialization. |
| 228 | <code>battery_action_t battery_monitor_update(battery_status_t *status,</code> | Define one battery update. dt_s is time elapsed since the previous sample, in seconds. |
| 229 | <code>                                        const battery_sample_t *sample,</code> | Continuation of the statement at L228: Define one battery update. dt_s is time elapsed since the previous sample, in seconds. |
| 230 | <code>                                        const battery_config_t *config,</code> | Continuation of the statement at L228: Define one battery update. dt_s is time elapsed since the previous sample, in seconds. |
| 231 | <code>                                        float dt_s) {</code> | Continuation of the statement at L228: Define one battery update. dt_s is time elapsed since the previous sample, in seconds. |
| 232 | <code>    if (status == NULL) return BATTERY_ACTION_CONFIGURATION_FAULT;</code> | A null status pointer returns a configuration-fault enum because no status object can be written. |
| 233 | <code>    status-&gt;sensor_fault = false;</code> | Clear the last sensor-fault flag before assessing the new update. |
| 234 | <code>    if (!config_is_valid(config)) {</code> | Branch if configuration validation fails. |
| 235 | <code>        status-&gt;action = BATTERY_ACTION_CONFIGURATION_FAULT;</code> | Store the configuration-fault action. |
| 236 | <code>        return status-&gt;action;</code> | Return immediately without applying any measurements. |
| 237 | <code>    }</code> | Close the configuration-fault branch. |
| 238 | <code>    if (sample == NULL &#124;&#124; !isfinite(dt_s) &#124;&#124; dt_s &lt;= 0.0f</code> | Start rejection of null samples or nonfinite/nonpositive elapsed time. |
| 239 | <code>        &#124;&#124; dt_s &gt; config-&gt;maximum_sample_dt_s</code> | Also reject elapsed time beyond the configured maximum. |
| 240 | <code>        &#124;&#124; !sample-&gt;pack_voltage_valid &#124;&#124; !sample-&gt;pack_current_valid</code> | Require both pack-voltage and current validity flags. A voltage divider alone does not satisfy this SOC monitor's interface. |
| 241 | <code>        &#124;&#124; !isfinite(sample-&gt;pack_voltage_v) &#124;&#124; sample-&gt;pack_voltage_v &lt;= 0.0f</code> | Require positive finite pack voltage. |
| 242 | <code>        &#124;&#124; !isfinite(sample-&gt;pack_current_a)</code> | Require finite current; negative current is allowed mathematically and represents charging. |
| 243 | <code>        &#124;&#124; (sample-&gt;cell_voltage_valid</code> | If individual-cell data is flagged valid, require positive finite minimum-cell voltage. If not flagged valid, this optional value is ignored. |
| 244 | <code>            &amp;&amp; (!isfinite(sample-&gt;minimum_cell_voltage_v)</code> | Continuation of the statement at L243: If individual-cell data is flagged valid, require positive finite minimum-cell voltage. If not flagged valid, this optional value is ignored. |
| 245 | <code>                &#124;&#124; sample-&gt;minimum_cell_voltage_v &lt;= 0.0f))</code> | Continuation of the statement at L243: If individual-cell data is flagged valid, require positive finite minimum-cell voltage. If not flagged valid, this optional value is ignored. |
| 246 | <code>        &#124;&#124; (sample-&gt;rested_soc_observation_valid</code> | If rested SOC is flagged valid, require a finite percentage in 0..100. The function trusts the caller that the pack really was rested and calibrated. |
| 247 | <code>            &amp;&amp; (!isfinite(sample-&gt;rested_soc_observation_percent)</code> | Continuation of the statement at L246: If rested SOC is flagged valid, require a finite percentage in 0..100. The function trusts the caller that the pack really was rested and calibrated. |
| 248 | <code>                &#124;&#124; sample-&gt;rested_soc_observation_percent &lt; 0.0f</code> | Continuation of the statement at L246: If rested SOC is flagged valid, require a finite percentage in 0..100. The function trusts the caller that the pack really was rested and calibrated. |
| 249 | <code>                &#124;&#124; sample-&gt;rested_soc_observation_percent &gt; 100.0f))) {</code> | Continuation of the statement at L246: If rested SOC is flagged valid, require a finite percentage in 0..100. The function trusts the caller that the pack really was rested and calibrated. |
| 250 | <code>        status-&gt;sensor_fault = true;</code> | Latch a sensor fault for this failed update. |
| 251 | <code>        status-&gt;action = BATTERY_ACTION_SENSOR_FAULT;</code> | Select SENSOR_FAULT. |
| 252 | <code>        return status-&gt;action;</code> | Return before modifying the voltage/SOC estimate. |
| 253 | <code>    }</code> | Close sample validation. |
| 255 | <code>    /* Scalar Kalman estimate of noisy total pack voltage; covariance in V^2. */</code> | Introduce the scalar pack-voltage Kalman filter. It estimates voltage, not capacity by itself. |
| 256 | <code>    if (!status-&gt;voltage_filter_initialized) {</code> | Branch on the first valid voltage measurement. |
| 257 | <code>        status-&gt;filtered_pack_voltage_v = sample-&gt;pack_voltage_v;</code> | Initialize estimated voltage directly to the sample. |
| 258 | <code>        status-&gt;pack_voltage_sigma_v =</code> | Initialize its standard deviation to sqrt(R), where R is measurement variance. |
| 259 | <code>            sqrtf(config-&gt;voltage_measurement_variance_v2);</code> | Continuation of the statement at L258: Initialize its standard deviation to sqrt(R), where R is measurement variance. |
| 260 | <code>        status-&gt;voltage_filter_initialized = true;</code> | Mark voltage filter initialized. |
| 261 | <code>    } else {</code> | For subsequent samples, perform predict/correct instead of reinitializing. |
| 262 | <code>        float p = status-&gt;pack_voltage_sigma_v * status-&gt;pack_voltage_sigma_v;</code> | Recover covariance P by squaring stored sigma. |
| 263 | <code>        p += config-&gt;voltage_process_variance_v2;</code> | Predict uncertainty: P_minus = P + Q. Here Q is per accepted update. |
| 264 | <code>        const float gain = p / (p + config-&gt;voltage_measurement_variance_v2);</code> | Compute scalar gain K = P_minus/(P_minus + R). |
| 265 | <code>        status-&gt;filtered_pack_voltage_v += gain</code> | Correct the voltage estimate: V_hat += K*(V_measured - V_hat). |
| 266 | <code>            * (sample-&gt;pack_voltage_v - status-&gt;filtered_pack_voltage_v);</code> | Continuation of the statement at L265: Correct the voltage estimate: V_hat += K*(V_measured - V_hat). |
| 267 | <code>        p *= 1.0f - gain;</code> | Update scalar covariance P_plus = (1-K)*P_minus. |
| 268 | <code>        status-&gt;pack_voltage_sigma_v = sqrtf(fmaxf(p, 0.0f));</code> | Store sigma = sqrt(max(P_plus,0)); max protects against a small negative roundoff result. |
| 269 | <code>    }</code> | End the voltage-filter branch. |
| 271 | <code>    /* SOC Kalman predict: coulomb count is the process model. */</code> | Introduce SOC prediction: current integration supplies the process model. |
| 272 | <code>    if (!status-&gt;soc_initialized) {</code> | If SOC was not initialized earlier, use configuration's initial value. |
| 273 | <code>        status-&gt;estimated_soc_percent = config-&gt;initial_soc_percent;</code> | Copy initial SOC from configuration. |
| 274 | <code>        status-&gt;soc_initialized = true;</code> | Mark its state initialized. |
| 275 | <code>    }</code> | Close SOC state initialization. |
| 276 | <code>    if (!status-&gt;soc_filter_initialized) {</code> | Initialize SOC uncertainty only once. |
| 277 | <code>        status-&gt;estimated_soc_sigma_percent =</code> | Set initial SOC sigma to the square root of the configured initial variance. |
| 278 | <code>            sqrtf(config-&gt;initial_soc_variance_percent2);</code> | Continuation of the statement at L277: Set initial SOC sigma to the square root of the configured initial variance. |
| 279 | <code>        status-&gt;soc_filter_initialized = true;</code> | Mark SOC covariance initialized. |
| 280 | <code>    }</code> | Close SOC covariance initialization. |
| 281 | <code>    status-&gt;estimated_soc_percent -= sample-&gt;pack_current_a * dt_s</code> | Subtract discharged capacity: delta_SOC = 100*I*dt/(capacity_Ah*3600). I*dt is ampere-seconds; 3600 converts Ah to ampere-seconds. At 10 A, 1 s, 4.2 Ah, the decrease is approximately 0.0661 percentage points. |
| 282 | <code>        / (config-&gt;capacity_ah * 3600.0f) * 100.0f;</code> | Continuation of the statement at L281: Subtract discharged capacity: delta_SOC = 100*I*dt/(capacity_Ah*3600). I*dt is ampere-seconds; 3600 converts Ah to ampere-seconds. At 10 A, 1 s, 4.2 Ah, the decrease is approximately 0.0661 percentage points. |
| 283 | <code>    float soc_variance = status-&gt;estimated_soc_sigma_percent</code> | Predict SOC variance by squaring current sigma and adding configured variance growth times elapsed seconds. This is a simplified uncertainty model, not a complete identified current-bias/capacity model. |
| 284 | <code>                       * status-&gt;estimated_soc_sigma_percent</code> | Continuation of the statement at L283: Predict SOC variance by squaring current sigma and adding configured variance growth times elapsed seconds. This is a simplified uncertainty model, not a complete identified current-bias/capacity model. |
| 285 | <code>                       + config-&gt;soc_process_variance_percent2_per_s * dt_s;</code> | Continuation of the statement at L283: Predict SOC variance by squaring current sigma and adding configured variance growth times elapsed seconds. This is a simplified uncertainty model, not a complete identified current-bias/capacity model. |
| 286 | <code>    /* Optional correction comes only from a separately calibrated, rested-pack</code> | Explain why voltage under motor load is not directly converted to SOC: load sag is not the same thing as rested open-circuit voltage. |
| 287 | <code>       OCV-to-SOC table. Loaded pack voltage is not a valid SOC measurement. */</code> | Continuation of the statement at L286: Explain why voltage under motor load is not directly converted to SOC: load sag is not the same thing as rested open-circuit voltage. |
| 288 | <code>    if (sample-&gt;rested_soc_observation_valid) {</code> | Apply an SOC measurement correction only when the optional rested observation is declared valid. |
| 289 | <code>        const float gain = soc_variance</code> | Calculate SOC gain K = P_minus/(P_minus + R_soc). |
| 290 | <code>            / (soc_variance + config-&gt;soc_measurement_variance_percent2);</code> | Continuation of the statement at L289: Calculate SOC gain K = P_minus/(P_minus + R_soc). |
| 291 | <code>        status-&gt;estimated_soc_percent += gain</code> | Move the predicted SOC toward the independent rested SOC observation by K times their difference. |
| 292 | <code>            * (sample-&gt;rested_soc_observation_percent</code> | Continuation of the statement at L291: Move the predicted SOC toward the independent rested SOC observation by K times their difference. |
| 293 | <code>               - status-&gt;estimated_soc_percent);</code> | Continuation of the statement at L291: Move the predicted SOC toward the independent rested SOC observation by K times their difference. |
| 294 | <code>        soc_variance *= 1.0f - gain;</code> | Reduce its covariance by multiplying by 1-K. |
| 295 | <code>    }</code> | Close optional SOC correction. |
| 296 | <code>    status-&gt;estimated_soc_percent = fminf(100.0f,</code> | Clamp the reported estimate into the physically meaningful interval 0..100 percent. |
| 297 | <code>        fmaxf(0.0f, status-&gt;estimated_soc_percent));</code> | Continuation of the statement at L296: Clamp the reported estimate into the physically meaningful interval 0..100 percent. |
| 298 | <code>    status-&gt;estimated_soc_sigma_percent = sqrtf(fmaxf(soc_variance, 0.0f));</code> | Convert the nonnegative updated variance back to sigma. |
| 300 | <code>    /* BMS alerts and cell-level undervoltage outrank pack-level warnings. */</code> | Document warning priority: optional BMS/cell-critical conditions are evaluated before lower-priority warnings. |
| 301 | <code>    if (sample-&gt;bms_alert</code> | If an external BMS alert is active, or available minimum-cell voltage is critical, select the critical action branch. |
| 302 | <code>        &#124;&#124; (sample-&gt;cell_voltage_valid</code> | Continuation of the statement at L301: If an external BMS alert is active, or available minimum-cell voltage is critical, select the critical action branch. |
| 303 | <code>            &amp;&amp; sample-&gt;minimum_cell_voltage_v &lt;= config-&gt;critical_cell_voltage_v)) {</code> | Continuation of the statement at L301: If an external BMS alert is active, or available minimum-cell voltage is critical, select the critical action branch. |
| 304 | <code>        status-&gt;action = sample-&gt;aircraft_landed</code> | If already landed, request LANDED_DISARM; otherwise request LAND. The ternary condition chooses one enum; it does not stop motors here. |
| 305 | <code>            ? BATTERY_ACTION_LANDED_DISARM : BATTERY_ACTION_LAND;</code> | Continuation of the statement at L304: If already landed, request LANDED_DISARM; otherwise request LAND. The ternary condition chooses one enum; it does not stop motors here. |
| 306 | <code>    } else if (status-&gt;estimated_soc_percent &lt;= config-&gt;low_soc_percent</code> | Otherwise check low SOC against the configured threshold. |
| 307 | <code>        &#124;&#124; status-&gt;filtered_pack_voltage_v &lt;= config-&gt;low_pack_voltage_v</code> | Also check low filtered total-pack voltage. |
| 308 | <code>        &#124;&#124; (sample-&gt;cell_voltage_valid</code> | Also check optional low-cell voltage. |
| 309 | <code>            &amp;&amp; sample-&gt;minimum_cell_voltage_v &lt;= config-&gt;low_cell_voltage_v)</code> | Continuation of the statement at L308: Also check optional low-cell voltage. |
| 310 | <code>        &#124;&#124; sample-&gt;pack_current_a &gt; config-&gt;maximum_discharge_current_a) {</code> | Also check measured current above the configured discharge limit. |
| 311 | <code>        status-&gt;action = sample-&gt;aircraft_landed</code> | For any of these warnings, request landing in air or disarming when already landed. |
| 312 | <code>            ? BATTERY_ACTION_LANDED_DISARM : BATTERY_ACTION_LAND;</code> | Continuation of the statement at L311: For any of these warnings, request landing in air or disarming when already landed. |
| 313 | <code>    } else {</code> | If none of the thresholds fired, take the normal branch. |
| 314 | <code>        status-&gt;action = BATTERY_ACTION_NORMAL;</code> | Set the normal action. |
| 315 | <code>    }</code> | Close action selection. There is no latching or hysteresis here; actions may clear on a later normal sample. |
| 316 | <code>    return status-&gt;action;</code> | Return the current action enum. |
| 317 | <code>}</code> | End the battery monitor. It is not called from the firmware main loop below; monitoring math is present, but integration is unfinished. |

### L319: Control timing and calibration data

| Line | Exact C source | Explanation |
|---:|---|---|
| 319 | <code>#define FC_LOOP_HZ 500.0f</code> | Set the intended control-loop frequency to 500 Hz. |
| 320 | <code>#define FC_LOOP_PERIOD_US 2000u</code> | Set the corresponding scheduling period to 2000 microseconds. |
| 321 | <code>#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)</code> | Define nominal dt as 1/500 = 0.002 seconds; this macro is not the actual measured dt used in main. |
| 322 | <code>#define FC_MAX_CONTROL_DT 0.010f</code> | Reject control intervals over 10 ms. At 500 Hz, that is five nominal periods. |
| 323 | <code>#define FC_RAD_TO_DEG 57.2957795f</code> | Approximate 180/pi for converting radians to degrees. |
| 324 | <code>#define FC_DEG_TO_RAD 0.0174532925f</code> | Approximate pi/180 for converting degrees to radians. |
| 325 | <code>#define FC_HOVER_THROTTLE_PLACEHOLDER 0.50f</code> | Assume normalized hover collective 0.50 until a caller supplies a better value. This is not measured thrust or necessarily physical ESC duty. |
| 326 | <code>#define FC_MAX_VERTICAL_SPEED_MPS 2.0f</code> | Set full-stick baseline vertical-speed magnitude to 2 m/s; RT can extend it to 3 m/s. |
| 327 | <code>#define FC_RADIO_TIMEOUT_US 150000u</code> | Commands become stale after 150000 microseconds = 150 ms. |
| 328 | <code>#define FC_SENSOR_LOOP_HZ 1000u</code> | Declare a sensor-rate target of 1000 Hz. This constant does not create a sensor task and is not used to acquire real IMU samples. |
| 330 | <code>/* SENSOR CALIBRATION -------------------------------------------------------</code> | Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 331 | <code> * Physical units after sensor factory conversion: g, deg/s, microtesla, Pa.</code> | Continuation of the statement at L330: Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 332 | <code> * These collectors accept fresh samples only while disarmed. Results live in</code> | Continuation of the statement at L330: Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 333 | <code> * RAM; sensor drivers, guided UI, temperature modelling and flash storage are</code> | Continuation of the statement at L330: Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 334 | <code> * not implemented. No test fixture is ever installed as a flight calibration.</code> | Continuation of the statement at L330: Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 335 | <code> */</code> | Continuation of the statement at L330: Document calibration units and limitations: fresh disarmed captures; RAM storage only; no drivers, guided calibration UI, temperature model or flash persistence. |
| 336 | <code>typedef struct {</code> | Begin online three-axis calibration statistics. |
| 337 | <code>    uint32_t n;</code> | Number of samples accumulated so far. |
| 338 | <code>    double mean[3], m2[3];</code> | Per-axis double-precision mean and centered sum of squared deviations M2. M2 is not yet divided by sample count. |
| 339 | <code>} cal_stats3_t;</code> | Name the statistics type. |
| 341 | <code>typedef struct {</code> | Begin the stored IMU calibration parameters. |
| 342 | <code>    float accel_bias_g[3], accel_gain[3];</code> | Accelerometer bias offsets in g and multiplicative axis gains. |
| 343 | <code>    float accel_bias_variance_g2[3], accel_gain_variance[3];</code> | Uncertainty of accelerometer bias and gain calibration parameters. |
| 344 | <code>    float gyro_bias_dps[3], gyro_variance_dps2[3];</code> | Gyroscope zero-rate offsets in degrees/s and sample noise variances in (degrees/s) squared. |
| 345 | <code>    float gyro_bias_variance_dps2[3];</code> | Uncertainty variance of the estimated gyro mean bias, distinct from individual-sample noise. |
| 346 | <code>    uint32_t gyro_samples;</code> | Number of samples used for gyro calibration. |
| 347 | <code>    bool accel_valid, gyro_valid;</code> | Validity flags for accelerometer and gyro calibration fits. |
| 348 | <code>} imu_calibration_t;</code> | Name the IMU calibration type. |
| 350 | <code>typedef struct { cal_stats3_t accel, gyro; } imu_stationary_cal_t;</code> | Group stationary accelerometer and gyro statistics so capture stability can be checked jointly. |

### L352: Online statistics, stationary gyro and six-face accelerometer calibration

| Line | Exact C source | Explanation |
|---:|---|---|
| 352 | <code>static bool cal_vector_finite(const float v[3]) {</code> | Declare three-component finite-value validation. |
| 353 | <code>    return v != NULL &amp;&amp; isfinite(v[0]) &amp;&amp; isfinite(v[1]) &amp;&amp; isfinite(v[2]);</code> | Require an array pointer and finite x, y and z components. |
| 354 | <code>}</code> | End the validation helper. |
| 356 | <code>static void cal_stats3_add(cal_stats3_t *s, const float v[3]) {</code> | Declare Welford's online statistics update for a three-axis vector. The caller must already have validated the pointers and overflow. |
| 357 | <code>    s-&gt;n++;</code> | Increment the sample count. |
| 358 | <code>    for (unsigned i = 0; i &lt; 3; ++i) {</code> | Repeat the statistics update independently for axes 0, 1 and 2. |
| 359 | <code>        const double delta = v[i] - s-&gt;mean[i];</code> | Calculate delta = new sample - old mean. |
| 360 | <code>        s-&gt;mean[i] += delta / s-&gt;n;</code> | Update mean: mean_new = mean_old + delta/n. |
| 361 | <code>        s-&gt;m2[i] += delta * (v[i] - s-&gt;mean[i]);</code> | Update M2 with delta*(sample - mean_new), using the new mean. This avoids subtracting two very large raw sums. |
| 362 | <code>    }</code> | End the axis loop. |
| 363 | <code>}</code> | End the online update. |
| 365 | <code>static double cal_variance(const cal_stats3_t *s, unsigned axis) {</code> | Declare conversion from accumulated M2 to sample variance. |
| 366 | <code>    return s-&gt;n &gt; 1u ? s-&gt;m2[axis] / (s-&gt;n - 1u) : INFINITY;</code> | Use unbiased sample variance M2/(n-1) if at least two samples exist; otherwise report infinity so later validation rejects insufficient data. |
| 367 | <code>}</code> | End variance calculation. |
| 369 | <code>bool imu_stationary_cal_add(imu_stationary_cal_t *s, const float accel_g[3],</code> | Accept one stationary IMU calibration pair. disarmed must be true; this API expects samples already converted from raw register counts to g and degrees/s. |
| 370 | <code>                            const float gyro_dps[3], bool disarmed) {</code> | Continuation of the statement at L369: Accept one stationary IMU calibration pair. disarmed must be true; this API expects samples already converted from raw register counts to g and degrees/s. |
| 371 | <code>    if (s == NULL &#124;&#124; !disarmed &#124;&#124; !cal_vector_finite(accel_g)</code> | Reject null statistics, an armed capture, nonfinite input vectors or counters at UINT32_MAX. |
| 372 | <code>        &#124;&#124; !cal_vector_finite(gyro_dps) &#124;&#124; s-&gt;accel.n == UINT32_MAX</code> | Continuation of the statement at L371: Reject null statistics, an armed capture, nonfinite input vectors or counters at UINT32_MAX. |
| 373 | <code>        &#124;&#124; s-&gt;gyro.n == UINT32_MAX) return false;</code> | Continuation of the statement at L371: Reject null statistics, an armed capture, nonfinite input vectors or counters at UINT32_MAX. |
| 374 | <code>    const float norm2 = accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1]</code> | Calculate squared accelerometer magnitude ax²+ay²+az², in g squared. |
| 375 | <code>                      + accel_g[2]*accel_g[2];</code> | Continuation of the statement at L374: Calculate squared accelerometer magnitude ax²+ay²+az², in g squared. |
| 376 | <code>    if (norm2 &lt; 0.64f &#124;&#124; norm2 &gt; 1.44f) return false;</code> | Require squared magnitude between 0.64 and 1.44: magnitude 0.8..1.2 g, an initial loose stationary plausibility check. |
| 377 | <code>    for (unsigned i = 0; i &lt; 3; ++i)</code> | Inspect all gyro axes. |
| 378 | <code>        if (fabsf(gyro_dps[i]) &gt; 10.0f) return false;</code> | Reject any instantaneous rotation over 10 degrees/s in magnitude. |
| 379 | <code>    cal_stats3_add(&amp;s-&gt;accel, accel_g);</code> | Add accelerometer data to stationary statistics. |
| 380 | <code>    cal_stats3_add(&amp;s-&gt;gyro, gyro_dps);</code> | Add gyro data to stationary statistics. |
| 381 | <code>    return true;</code> | Report acceptance of this sample pair; this alone does not complete calibration. |
| 382 | <code>}</code> | End stationary capture. |
| 384 | <code>bool imu_gyro_cal_finish(const imu_stationary_cal_t *s, imu_calibration_t *out,</code> | Finalize the gyro bias/noise fit from a stationary capture, writing output only after aggregate checks. |
| 385 | <code>                         bool disarmed) {</code> | Continuation of the statement at L384: Finalize the gyro bias/noise fit from a stationary capture, writing output only after aggregate checks. |
| 386 | <code>    if (s == NULL &#124;&#124; out == NULL &#124;&#124; !disarmed &#124;&#124; s-&gt;gyro.n &lt; 500u</code> | Require pointers, disarmed capture, at least 500 samples and equal accelerometer/gyro sample counts. |
| 387 | <code>        &#124;&#124; s-&gt;accel.n != s-&gt;gyro.n) return false;</code> | Continuation of the statement at L386: Require pointers, disarmed capture, at least 500 samples and equal accelerometer/gyro sample counts. |
| 388 | <code>    double norm2 = 0.0;</code> | Start the squared norm of mean acceleration at zero. |
| 389 | <code>    for (unsigned i = 0; i &lt; 3; ++i) {</code> | Validate each axis's aggregate measurements. |
| 390 | <code>        norm2 += s-&gt;accel.mean[i] * s-&gt;accel.mean[i];</code> | Accumulate squared mean acceleration components. |
| 391 | <code>        if (!isfinite(s-&gt;gyro.mean[i]) &#124;&#124; fabs(s-&gt;gyro.mean[i]) &gt; 5.0</code> | Reject a nonfinite mean gyro reading or stationary mean over 5 degrees/s. |
| 392 | <code>            &#124;&#124; !isfinite(cal_variance(&amp;s-&gt;accel, i))</code> | Require finite accelerometer and gyro sample variances. |
| 393 | <code>            &#124;&#124; !isfinite(cal_variance(&amp;s-&gt;gyro, i))</code> | Continuation of the statement at L392: Require finite accelerometer and gyro sample variances. |
| 394 | <code>            &#124;&#124; cal_variance(&amp;s-&gt;accel, i) &lt; 0.0</code> | Require both variances nonnegative. |
| 395 | <code>            &#124;&#124; cal_variance(&amp;s-&gt;gyro, i) &lt; 0.0</code> | Continuation of the statement at L394: Require both variances nonnegative. |
| 396 | <code>            &#124;&#124; cal_variance(&amp;s-&gt;accel, i) &gt; 0.0004</code> | Require accelerometer sample variance no larger than 0.0004 g², corresponding to sigma <=0.02 g per axis. |
| 397 | <code>            &#124;&#124; cal_variance(&amp;s-&gt;gyro, i) &gt; 0.01) return false;</code> | Require gyro variance no larger than 0.01 (degrees/s)², corresponding to sigma <=0.1 degrees/s per axis. |
| 398 | <code>    }</code> | Finish per-axis stability checks. |
| 399 | <code>    if (!isfinite(norm2) &#124;&#124; norm2 &lt; 0.81 &#124;&#124; norm2 &gt; 1.21) return false;</code> | Require finite mean acceleration magnitude 0.9..1.1 g; the code compares its square to 0.81..1.21. |
| 400 | <code>    for (unsigned i = 0; i &lt; 3; ++i) {</code> | Now write each accepted gyro calibration parameter. |
| 401 | <code>        out-&gt;gyro_bias_dps[i] = (float)s-&gt;gyro.mean[i];</code> | Estimate zero-rate bias as that axis's sample mean. |
| 402 | <code>        out-&gt;gyro_variance_dps2[i] = (float)cal_variance(&amp;s-&gt;gyro, i);</code> | Store individual-sample gyro variance. |
| 403 | <code>        out-&gt;gyro_bias_variance_dps2[i] = out-&gt;gyro_variance_dps2[i] / s-&gt;gyro.n;</code> | Estimate variance of the mean bias as sample variance/n, assuming effectively independent stationary samples. |
| 404 | <code>    }</code> | Finish writing all axes. |
| 405 | <code>    out-&gt;gyro_samples = s-&gt;gyro.n;</code> | Record sample count for provenance. |
| 406 | <code>    out-&gt;gyro_valid = true;</code> | Mark gyro calibration valid. |
| 407 | <code>    return true;</code> | Report success. |
| 408 | <code>}</code> | End gyro finalization. |
| 410 | <code>/* Six poses in sensor axes: +X, -X, +Y, -Y, +Z, -Z (specific force). */</code> | Document six accelerometer poses: +X,-X,+Y,-Y,+Z,-Z in sensor axes, referring to specific force rather than world position. |
| 411 | <code>bool imu_accel_face_add(cal_stats3_t faces[6], unsigned face,</code> | Accept one accelerometer sample into one selected face's statistics while disarmed. |
| 412 | <code>                        const float accel_g[3], bool disarmed) {</code> | Continuation of the statement at L411: Accept one accelerometer sample into one selected face's statistics while disarmed. |
| 413 | <code>    if (faces == NULL &#124;&#124; face &gt;= 6u &#124;&#124; !disarmed</code> | Require a valid faces array, face index below six, disarmed capture, finite sample and no count overflow. |
| 414 | <code>        &#124;&#124; !cal_vector_finite(accel_g) &#124;&#124; faces[face].n == UINT32_MAX)</code> | Continuation of the statement at L413: Require a valid faces array, face index below six, disarmed capture, finite sample and no count overflow. |
| 415 | <code>        return false;</code> | Continuation of the statement at L413: Require a valid faces array, face index below six, disarmed capture, finite sample and no count overflow. |
| 416 | <code>    cal_stats3_add(&amp;faces[face], accel_g);</code> | Accumulate this sample in that face's online statistics. |
| 417 | <code>    return true;</code> | Report successful sample accumulation. |
| 418 | <code>}</code> | End face capture. Orientation/stability checks happen on finalization; this function trusts the supplied face label initially. |
| 420 | <code>bool imu_accel_cal_finish(const cal_stats3_t faces[6], imu_calibration_t *out,</code> | Finalize the six-face accelerometer calibration. |
| 421 | <code>                          bool disarmed) {</code> | Continuation of the statement at L420: Finalize the six-face accelerometer calibration. |
| 422 | <code>    if (faces == NULL &#124;&#124; out == NULL &#124;&#124; !disarmed) return false;</code> | Require valid input/output pointers and disarmed state. |
| 423 | <code>    float bias[3], gain[3];</code> | Prepare temporary bias and gain arrays before committing any fit. |
| 424 | <code>    for (unsigned face = 0; face &lt; 6u; ++face) {</code> | Inspect each of the six faces. |
| 425 | <code>        if (faces[face].n &lt; 100u) return false;</code> | Require at least 100 samples per face. |
| 426 | <code>        for (unsigned axis = 0; axis &lt; 3u; ++axis) {</code> | Inspect all three axes for that face. |
| 427 | <code>            if (!isfinite(faces[face].mean[axis])</code> | Require finite per-axis mean. |
| 428 | <code>                &#124;&#124; !isfinite(cal_variance(&amp;faces[face], axis))</code> | Require finite, nonnegative sample variance <=0.0004 g². |
| 429 | <code>                &#124;&#124; cal_variance(&amp;faces[face], axis) &lt; 0.0</code> | Continuation of the statement at L428: Require finite, nonnegative sample variance <=0.0004 g². |
| 430 | <code>                &#124;&#124; cal_variance(&amp;faces[face], axis) &gt; 0.0004) return false;</code> | Continuation of the statement at L428: Require finite, nonnegative sample variance <=0.0004 g². |
| 431 | <code>            if (axis != face / 2u &amp;&amp; fabs(faces[face].mean[axis]) &gt; 0.25)</code> | Axes other than the intended face axis must have mean magnitude <=0.25 g, rejecting strongly tilted or mislabeled faces. Integer face/2 selects the intended axis. |
| 432 | <code>                return false;</code> | Continuation of the statement at L431: Axes other than the intended face axis must have mean magnitude <=0.25 g, rejecting strongly tilted or mislabeled faces. Integer face/2 selects the intended axis. |
| 433 | <code>        }</code> | End per-axis checks for this face. |
| 434 | <code>    }</code> | End all-face validation. |
| 435 | <code>    for (unsigned axis = 0; axis &lt; 3u; ++axis) {</code> | Fit one bias/gain pair per axis. |
| 436 | <code>        const double pos = faces[axis * 2u].mean[axis];</code> | Read the positive-face mean for this axis. |
| 437 | <code>        const double neg = faces[axis * 2u + 1u].mean[axis];</code> | Read the negative-face mean for this axis. |
| 438 | <code>        const double span = pos - neg;</code> | Calculate measured span = positive mean - negative mean; ideal span is 2 g. |
| 439 | <code>        if (pos &lt; 0.7 &#124;&#124; neg &gt; -0.7 &#124;&#124; span &lt; 1.5 &#124;&#124; span &gt; 2.5)</code> | Require a sufficiently positive positive-face reading, negative negative-face reading and a span in 1.5..2.5 g. |
| 440 | <code>            return false;</code> | Continuation of the statement at L439: Require a sufficiently positive positive-face reading, negative negative-face reading and a span in 1.5..2.5 g. |
| 441 | <code>        bias[axis] = (float)((pos + neg) * 0.5);</code> | Bias is the midpoint (positive+negative)/2. |
| 442 | <code>        gain[axis] = (float)(2.0 / span);</code> | Gain is ideal span/measured span = 2/span, so corrected positive and negative readings approach +1 and -1 g. |
| 443 | <code>        if (fabsf(bias[axis]) &gt; 0.25f) return false;</code> | Reject absolute bias exceeding 0.25 g. |
| 444 | <code>    }</code> | End preliminary fit for all axes. |
| 445 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Commit the accepted calibration and calculate parameter uncertainties per axis. |
| 446 | <code>        out-&gt;accel_bias_g[i] = bias[i];</code> | Store the estimated bias. |
| 447 | <code>        out-&gt;accel_gain[i] = gain[i];</code> | Store the estimated scale gain. |
| 448 | <code>        const cal_stats3_t *pos = &amp;faces[2u*i], *neg = &amp;faces[2u*i+1u];</code> | Take pointers to this axis's positive and negative statistics. |
| 449 | <code>        const double mean_variance = cal_variance(pos, i)/pos-&gt;n</code> | Sum variances of the two face means: sampleVariance_pos/n_pos + sampleVariance_neg/n_neg. |
| 450 | <code>                                   + cal_variance(neg, i)/neg-&gt;n;</code> | Continuation of the statement at L449: Sum variances of the two face means: sampleVariance_pos/n_pos + sampleVariance_neg/n_neg. |
| 451 | <code>        const double span = pos-&gt;mean[i] - neg-&gt;mean[i];</code> | Recalculate the positive-minus-negative mean span. |
| 452 | <code>        out-&gt;accel_bias_variance_g2[i] = (float)(0.25 * mean_variance);</code> | Bias variance is one quarter of that sum, from differentiating (pos+neg)/2. |
| 453 | <code>        out-&gt;accel_gain_variance[i] = (float)(4.0 * mean_variance / pow(span, 4.0));</code> | Gain variance is 4*meanVariance/span^4, from differentiating 2/(pos-neg), assuming independent face means. |
| 454 | <code>    }</code> | Finish writing axis fits. |
| 455 | <code>    out-&gt;accel_valid = true;</code> | Mark accelerometer calibration valid. |
| 456 | <code>    return true;</code> | Report success. |
| 457 | <code>}</code> | End six-face finalization. This is a diagonal gain/offset model, not a full cross-axis alignment fit. |
| 459 | <code>bool imu_cal_apply(const imu_calibration_t *c, const float accel_g[3],</code> | Apply stored IMU calibration to input acceleration and gyro vectors, returning corrected vectors through output pointers. |
| 460 | <code>                    const float gyro_dps[3], float corrected_accel_g[3],</code> | Continuation of the statement at L459: Apply stored IMU calibration to input acceleration and gyro vectors, returning corrected vectors through output pointers. |
| 461 | <code>                    float corrected_gyro_dps[3]) {</code> | Continuation of the statement at L459: Apply stored IMU calibration to input acceleration and gyro vectors, returning corrected vectors through output pointers. |
| 462 | <code>    if (c == NULL &#124;&#124; !c-&gt;accel_valid &#124;&#124; !c-&gt;gyro_valid</code> | Require a calibration object with both validity flags, finite input vectors and non-null output arrays. |
| 463 | <code>        &#124;&#124; !cal_vector_finite(accel_g) &#124;&#124; !cal_vector_finite(gyro_dps)</code> | Continuation of the statement at L462: Require a calibration object with both validity flags, finite input vectors and non-null output arrays. |
| 464 | <code>        &#124;&#124; corrected_accel_g == NULL &#124;&#124; corrected_gyro_dps == NULL)</code> | Continuation of the statement at L462: Require a calibration object with both validity flags, finite input vectors and non-null output arrays. |
| 465 | <code>        return false;</code> | Continuation of the statement at L462: Require a calibration object with both validity flags, finite input vectors and non-null output arrays. |
| 466 | <code>    float a[3], g[3];</code> | Use temporary corrected arrays so a failure does not partially overwrite caller outputs. |
| 467 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Correct each axis independently. |
| 468 | <code>        if (!isfinite(c-&gt;accel_gain[i]) &#124;&#124; c-&gt;accel_gain[i] &lt;= 0.0f</code> | Require a finite positive accelerometer gain and finite accelerometer/gyro biases. |
| 469 | <code>            &#124;&#124; !isfinite(c-&gt;accel_bias_g[i]) &#124;&#124; !isfinite(c-&gt;gyro_bias_dps[i]))</code> | Continuation of the statement at L468: Require a finite positive accelerometer gain and finite accelerometer/gyro biases. |
| 470 | <code>            return false;</code> | Continuation of the statement at L468: Require a finite positive accelerometer gain and finite accelerometer/gyro biases. |
| 471 | <code>        a[i] = (accel_g[i] - c-&gt;accel_bias_g[i]) * c-&gt;accel_gain[i];</code> | Correct acceleration as (raw_g - bias_g)*gain. |
| 472 | <code>        g[i] = gyro_dps[i] - c-&gt;gyro_bias_dps[i];</code> | Correct gyro rate as raw_dps - stationary_bias_dps. |
| 473 | <code>        if (!isfinite(a[i]) &#124;&#124; !isfinite(g[i])) return false;</code> | Reject nonfinite corrected results. |
| 474 | <code>    }</code> | End calculation loop. |
| 475 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | After all calculations succeeded, copy the temporary outputs to the caller. |
| 476 | <code>        corrected_accel_g[i] = a[i];</code> | Write corrected acceleration for this axis. |
| 477 | <code>        corrected_gyro_dps[i] = g[i];</code> | Write corrected gyro rate for this axis. |
| 478 | <code>    }</code> | End copy loop. |
| 479 | <code>    return true;</code> | Report success. |
| 480 | <code>}</code> | End IMU correction. |

### L482: Magnetometer calibration — not yet flight fusion

| Line | Exact C source | Explanation |
|---:|---|---|
| 482 | <code>typedef struct {</code> | Begin state accumulated during a magnetometer sweep. |
| 483 | <code>    cal_stats3_t stats;</code> | Hold online three-axis statistics. |
| 484 | <code>    float minimum_ut[3], maximum_ut[3];</code> | Track minimum and maximum magnetic readings per axis, in microtesla. |
| 485 | <code>    double cross_m2[3]; /* xy, xz, yz: reject a rotation confined to a plane */</code> | Track centered xy,xz,yz cross-products to reject a sweep confined approximately to a plane. |
| 486 | <code>} mag_sweep_cal_t;</code> | Name sweep-statistics type. |
| 488 | <code>typedef struct { float offset_ut[3], gain[3]; bool valid; } mag_calibration_t;</code> | Define a diagonal magnetic calibration: per-axis hard-iron offset, scale gain and a validity flag. |
| 490 | <code>bool mag_cal_add(mag_sweep_cal_t *s, const float field_ut[3], bool disarmed) {</code> | Accept one magnetic field vector during a disarmed sweep. |
| 491 | <code>    if (s == NULL &#124;&#124; !disarmed &#124;&#124; !cal_vector_finite(field_ut)</code> | Require a statistics pointer, disarmed state, finite field vector and no sample-count overflow. |
| 492 | <code>        &#124;&#124; s-&gt;stats.n == UINT32_MAX) return false;</code> | Continuation of the statement at L491: Require a statistics pointer, disarmed state, finite field vector and no sample-count overflow. |
| 493 | <code>    double delta[3];</code> | Prepare double-precision differences from the old mean. |
| 494 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Inspect each axis. |
| 495 | <code>        if (fabsf(field_ut[i]) &gt; 1000.0f) return false;</code> | Reject individual components exceeding 1000 microtesla in magnitude. |
| 496 | <code>        delta[i] = field_ut[i] - s-&gt;stats.mean[i];</code> | Save sample-minus-old-mean for covariance accumulation. |
| 497 | <code>    }</code> | End difference collection. |
| 498 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Update minimum and maximum values for each axis. |
| 499 | <code>        if (s-&gt;stats.n == 0u &#124;&#124; field_ut[i] &lt; s-&gt;minimum_ut[i])</code> | Initialize the minimum on the first sample, or replace it if this reading is smaller. |
| 500 | <code>            s-&gt;minimum_ut[i] = field_ut[i];</code> | Continuation of the statement at L499: Initialize the minimum on the first sample, or replace it if this reading is smaller. |
| 501 | <code>        if (s-&gt;stats.n == 0u &#124;&#124; field_ut[i] &gt; s-&gt;maximum_ut[i])</code> | Initialize the maximum on the first sample, or replace it if this reading is larger. |
| 502 | <code>            s-&gt;maximum_ut[i] = field_ut[i];</code> | Continuation of the statement at L501: Initialize the maximum on the first sample, or replace it if this reading is larger. |
| 503 | <code>    }</code> | End extrema collection. |
| 504 | <code>    cal_stats3_add(&amp;s-&gt;stats, field_ut);</code> | Update means and diagonal centered sums with the shared Welford helper. |
| 505 | <code>    s-&gt;cross_m2[0] += delta[0] * (field_ut[1] - s-&gt;stats.mean[1]);</code> | Update centered xy cross-product using old x difference and new y mean. |
| 506 | <code>    s-&gt;cross_m2[1] += delta[0] * (field_ut[2] - s-&gt;stats.mean[2]);</code> | Update xz cross-product. |
| 507 | <code>    s-&gt;cross_m2[2] += delta[1] * (field_ut[2] - s-&gt;stats.mean[2]);</code> | Update yz cross-product. |
| 508 | <code>    return true;</code> | Report accepted sweep sample. |
| 509 | <code>}</code> | End magnetic sample collection. |
| 511 | <code>/* Min/max hard-iron + diagonal scale fit, NOT a full soft-iron matrix fit. */</code> | Explicitly state model limits: min/max hard-iron plus diagonal scaling, not a full soft-iron ellipsoid/matrix fit. |
| 512 | <code>bool mag_cal_finish(const mag_sweep_cal_t *s, mag_calibration_t *out,</code> | Finalize magnetic calibration, requiring both caller-confirmed full rotation and disarmed state. |
| 513 | <code>                     bool full_rotation_confirmed, bool disarmed) {</code> | Continuation of the statement at L512: Finalize magnetic calibration, requiring both caller-confirmed full rotation and disarmed state. |
| 514 | <code>    if (s == NULL &#124;&#124; out == NULL &#124;&#124; !disarmed &#124;&#124; !full_rotation_confirmed</code> | Reject missing pointers, armed/incomplete rotation captures or fewer than 300 samples. |
| 515 | <code>        &#124;&#124; s-&gt;stats.n &lt; 300u) return false;</code> | Continuation of the statement at L514: Reject missing pointers, armed/incomplete rotation captures or fewer than 300 samples. |
| 516 | <code>    double radius[3], mean_radius = 0.0;</code> | Prepare three measured half-ranges and their average. |
| 517 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Evaluate each magnetic axis. |
| 518 | <code>        radius[i] = (s-&gt;maximum_ut[i] - s-&gt;minimum_ut[i]) * 0.5;</code> | Radius is half of maximum-minus-minimum span. |
| 519 | <code>        if (!isfinite(radius[i]) &#124;&#124; radius[i] &lt; 5.0 &#124;&#124; radius[i] &gt; 100.0</code> | Require a finite radius 5..100 microtesla and a finite positive centered sum of squares. |
| 520 | <code>            &#124;&#124; !isfinite(s-&gt;stats.m2[i]) &#124;&#124; s-&gt;stats.m2[i] &lt;= 0.0)</code> | Continuation of the statement at L519: Require a finite radius 5..100 microtesla and a finite positive centered sum of squares. |
| 521 | <code>            return false;</code> | Continuation of the statement at L519: Require a finite radius 5..100 microtesla and a finite positive centered sum of squares. |
| 522 | <code>        mean_radius += radius[i] / 3.0;</code> | Accumulate one third of each radius to obtain the average radius. |
| 523 | <code>    }</code> | Finish per-axis radius validation. |
| 524 | <code>    const double xy = s-&gt;cross_m2[0] / sqrt(s-&gt;stats.m2[0]*s-&gt;stats.m2[1]);</code> | Compute normalized xy correlation from the cross-product and diagonal sums. |
| 525 | <code>    const double xz = s-&gt;cross_m2[1] / sqrt(s-&gt;stats.m2[0]*s-&gt;stats.m2[2]);</code> | Compute xz correlation. |
| 526 | <code>    const double yz = s-&gt;cross_m2[2] / sqrt(s-&gt;stats.m2[1]*s-&gt;stats.m2[2]);</code> | Compute yz correlation. |
| 527 | <code>    const double coverage = 1.0 + 2.0*xy*xz*yz - xy*xy - xz*xz - yz*yz;</code> | Compute determinant of the 3x3 correlation matrix: 1+2*rxy*rxz*ryz-rxy²-rxz²-ryz². Near-zero determinant indicates inadequate three-dimensional variation. |
| 528 | <code>    if (!isfinite(coverage) &#124;&#124; coverage &lt; 0.05) return false;</code> | Require finite determinant at least 0.05. This heuristic is not a complete proof of uniform sphere coverage or magnetic cleanliness. |
| 529 | <code>    for (unsigned i = 0; i &lt; 3u; ++i)</code> | Require every proposed diagonal gain mean_radius/radius_i within 0.5..2.0. |
| 530 | <code>        if (mean_radius / radius[i] &lt; 0.5 &#124;&#124; mean_radius / radius[i] &gt; 2.0)</code> | Continuation of the statement at L529: Require every proposed diagonal gain mean_radius/radius_i within 0.5..2.0. |
| 531 | <code>            return false;</code> | Continuation of the statement at L529: Require every proposed diagonal gain mean_radius/radius_i within 0.5..2.0. |
| 532 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Commit the three accepted axis fits. |
| 533 | <code>        out-&gt;offset_ut[i] = (s-&gt;maximum_ut[i] + s-&gt;minimum_ut[i]) * 0.5f;</code> | Offset is midpoint of the observed min and max on this axis. |
| 534 | <code>        out-&gt;gain[i] = (float)(mean_radius / radius[i]);</code> | Scale the axis radius toward the mean radius. |
| 535 | <code>    }</code> | Finish axis fit storage. |
| 536 | <code>    out-&gt;valid = true;</code> | Mark magnetic calibration valid. |
| 537 | <code>    return true;</code> | Report success. |
| 538 | <code>}</code> | End magnetic finalization. These calibration parameters are not used by the current attitude/control step. |
| 540 | <code>bool mag_cal_apply(const mag_calibration_t *c, const float raw_ut[3],</code> | Apply a previously accepted magnetic offset/scale calibration. |
| 541 | <code>                    float corrected_ut[3]) {</code> | Continuation of the statement at L540: Apply a previously accepted magnetic offset/scale calibration. |
| 542 | <code>    if (c == NULL &#124;&#124; !c-&gt;valid &#124;&#124; !cal_vector_finite(raw_ut)</code> | Require valid calibration, finite input vector and output storage. |
| 543 | <code>        &#124;&#124; corrected_ut == NULL) return false;</code> | Continuation of the statement at L542: Require valid calibration, finite input vector and output storage. |
| 544 | <code>    float corrected[3];</code> | Prepare temporary corrected vector. |
| 545 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) {</code> | Correct each axis. |
| 546 | <code>        if (!isfinite(c-&gt;offset_ut[i]) &#124;&#124; !isfinite(c-&gt;gain[i])</code> | Require finite offset and finite positive gain. |
| 547 | <code>            &#124;&#124; c-&gt;gain[i] &lt;= 0.0f) return false;</code> | Continuation of the statement at L546: Require finite offset and finite positive gain. |
| 548 | <code>        corrected[i] = (raw_ut[i] - c-&gt;offset_ut[i]) * c-&gt;gain[i];</code> | Compute corrected magnetic field (raw-offset)*gain. |
| 549 | <code>        if (!isfinite(corrected[i])) return false;</code> | Reject nonfinite results. |
| 550 | <code>    }</code> | Finish calculation loop. |
| 551 | <code>    for (unsigned i = 0; i &lt; 3u; ++i) corrected_ut[i] = corrected[i];</code> | Copy corrected vector only after all axes succeeded. |
| 552 | <code>    return true;</code> | Report success. |
| 553 | <code>}</code> | End magnetic correction. No heading, tilt compensation, declination or yaw fusion is performed here. |

### L555: BMP280 factory compensation and ground-reference altitude

| Line | Exact C source | Explanation |
|---:|---|---|
| 555 | <code>/* BMP280 OTP coefficients are factory calibration, separate from ground zero.</code> | Introduce BMP280 factory-trim compensation. This is separate from learning local ground pressure. No I2C register reading is implemented by these arithmetic functions. |
| 556 | <code> * Compensation equations: Bosch BMP280 datasheet, section 3.11.3.</code> | Continuation of the statement at L555: Introduce BMP280 factory-trim compensation. This is separate from learning local ground pressure. No I2C register reading is implemented by these arithmetic functions. |
| 557 | <code> */</code> | Continuation of the statement at L555: Introduce BMP280 factory-trim compensation. This is separate from learning local ground pressure. No I2C register reading is implemented by these arithmetic functions. |
| 558 | <code>typedef struct {</code> | Begin storage for the BMP280 factory coefficients. |
| 559 | <code>    uint16_t t1, p1;</code> | T1 and P1 are unsigned 16-bit coefficients. |
| 560 | <code>    int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9;</code> | T2,T3 and P2..P9 are signed 16-bit coefficients; signedness matters for decoding. |
| 561 | <code>} bmp280_trim_t;</code> | Name the trim structure. |
| 563 | <code>bool bmp280_trim_decode(const uint8_t *bytes, size_t length, bmp280_trim_t *out) {</code> | Decode the 24 factory-calibration bytes into trim coefficients. |
| 564 | <code>    if (bytes == NULL &#124;&#124; out == NULL &#124;&#124; length != 24u) return false;</code> | Require pointers and exactly 24 input bytes. |
| 565 | <code>    const bmp280_trim_t t = {</code> | Build a temporary trim object. |
| 566 | <code>        .t1 = qcopter_get_u16(bytes), .t2 = (int16_t)qcopter_get_u16(bytes + 2),</code> | Read T1 at offsets 0..1 and signed T2 at 2..3. |
| 567 | <code>        .t3 = (int16_t)qcopter_get_u16(bytes + 4), .p1 = qcopter_get_u16(bytes + 6),</code> | Read signed T3 at 4..5 and unsigned P1 at 6..7. |
| 568 | <code>        .p2 = (int16_t)qcopter_get_u16(bytes + 8), .p3 = (int16_t)qcopter_get_u16(bytes + 10),</code> | Read signed P2 at 8..9 and P3 at 10..11. |
| 569 | <code>        .p4 = (int16_t)qcopter_get_u16(bytes + 12), .p5 = (int16_t)qcopter_get_u16(bytes + 14),</code> | Read signed P4 at 12..13 and P5 at 14..15. |
| 570 | <code>        .p6 = (int16_t)qcopter_get_u16(bytes + 16), .p7 = (int16_t)qcopter_get_u16(bytes + 18),</code> | Read signed P6 at 16..17 and P7 at 18..19. |
| 571 | <code>        .p8 = (int16_t)qcopter_get_u16(bytes + 20), .p9 = (int16_t)qcopter_get_u16(bytes + 22)</code> | Read signed P8 at 20..21 and P9 at 22..23. |
| 572 | <code>    };</code> | Finish temporary coefficient initialization. |
| 573 | <code>    if (t.t1 == 0u &#124;&#124; t.t1 == 0xFFFFu &#124;&#124; t.p1 == 0u &#124;&#124; t.p1 == 0xFFFFu)</code> | Reject all-zero/all-one sentinel values for essential coefficients T1 and P1. |
| 574 | <code>        return false;</code> | Continuation of the statement at L573: Reject all-zero/all-one sentinel values for essential coefficients T1 and P1. |
| 575 | <code>    *out = t;</code> | Commit the validated coefficients. |
| 576 | <code>    return true;</code> | Report successful decoding. |
| 577 | <code>}</code> | End trim decoder. |
| 579 | <code>bool bmp280_compensate(const bmp280_trim_t *t, uint32_t raw_temperature,</code> | Convert raw BMP280 temperature/pressure ADC values to degrees Celsius and pascals through output pointers, using factory trim. |
| 580 | <code>                        uint32_t raw_pressure, double *temperature_c,</code> | Continuation of the statement at L579: Convert raw BMP280 temperature/pressure ADC values to degrees Celsius and pascals through output pointers, using factory trim. |
| 581 | <code>                        double *pressure_pa) {</code> | Continuation of the statement at L579: Convert raw BMP280 temperature/pressure ADC values to degrees Celsius and pascals through output pointers, using factory trim. |
| 582 | <code>    if (t == NULL &#124;&#124; temperature_c == NULL &#124;&#124; pressure_pa == NULL</code> | Require valid pointers, essential nonzero trim values, 20-bit raw values, and reject the 0x80000 skipped-measurement sentinel. |
| 583 | <code>        &#124;&#124; t-&gt;t1 == 0u &#124;&#124; t-&gt;p1 == 0u &#124;&#124; raw_temperature &gt; 0xFFFFFu</code> | Continuation of the statement at L582: Require valid pointers, essential nonzero trim values, 20-bit raw values, and reject the 0x80000 skipped-measurement sentinel. |
| 584 | <code>        &#124;&#124; raw_pressure &gt; 0xFFFFFu &#124;&#124; raw_temperature == 0x80000u</code> | Continuation of the statement at L582: Require valid pointers, essential nonzero trim values, 20-bit raw values, and reject the 0x80000 skipped-measurement sentinel. |
| 585 | <code>        &#124;&#124; raw_pressure == 0x80000u) return false;</code> | Continuation of the statement at L582: Require valid pointers, essential nonzero trim values, 20-bit raw values, and reject the 0x80000 skipped-measurement sentinel. |
| 586 | <code>    const double dt = raw_temperature / 131072.0 - t-&gt;t1 / 8192.0;</code> | Form a scaled raw-temperature difference used in the quadratic temperature correction. This local variable dt is not a time step. |
| 587 | <code>    const double fine = (raw_temperature / 16384.0 - t-&gt;t1 / 1024.0) * t-&gt;t2</code> | Compute the temperature fine term: the linear temperature difference times T2 plus the squared difference term times T3. |
| 588 | <code>                      + dt * dt * t-&gt;t3;</code> | Continuation of the statement at L587: Compute the temperature fine term: the linear temperature difference times T2 plus the squared difference term times T3. |
| 589 | <code>    const double temp = fine / 5120.0;</code> | Convert fine temperature to degrees Celsius by dividing by 5120. |
| 590 | <code>    double v1 = fine / 2.0 - 64000.0;</code> | Initialize the first pressure intermediate from temperature fine compensation. |
| 591 | <code>    double v2 = v1*v1*t-&gt;p6/32768.0 + v1*t-&gt;p5*2.0;</code> | Calculate the quadratic/linear temperature contribution using P6 and P5. |
| 592 | <code>    v2 = v2/4.0 + t-&gt;p4*65536.0;</code> | Scale that contribution and add P4's offset term. |
| 593 | <code>    v1 = (t-&gt;p3*v1*v1/524288.0 + t-&gt;p2*v1)/524288.0;</code> | Calculate a second temperature-dependent pressure-scale intermediate using P3 and P2. |
| 594 | <code>    v1 = (1.0 + v1/32768.0) * t-&gt;p1;</code> | Finish the pressure denominator using P1. |
| 595 | <code>    if (!isfinite(v1) &#124;&#124; fabs(v1) &lt; 1e-12) return false;</code> | Reject a nonfinite or nearly zero denominator before dividing. |
| 596 | <code>    double pressure = (1048576.0 - raw_pressure - v2/4096.0) * 6250.0 / v1;</code> | Calculate the first compensated pressure estimate from raw pressure and the temperature correction terms. |
| 597 | <code>    v1 = t-&gt;p9*pressure*pressure/2147483648.0;</code> | Calculate pressure-squared correction using P9. |
| 598 | <code>    v2 = pressure*t-&gt;p8/32768.0;</code> | Calculate linear pressure correction using P8. |
| 599 | <code>    pressure += (v1 + v2 + t-&gt;p7)/16.0;</code> | Add those terms and P7's offset to pressure. |
| 600 | <code>    if (!isfinite(temp) &#124;&#124; !isfinite(pressure) &#124;&#124; temp &lt; -40.0 &#124;&#124; temp &gt; 85.0</code> | Reject nonfinite output, temperatures outside -40..85 Celsius and pressures outside 30000..110000 Pa. These are broad plausibility bounds, not a guarantee of a good sensor measurement. |
| 601 | <code>        &#124;&#124; pressure &lt; 30000.0 &#124;&#124; pressure &gt; 110000.0) return false;</code> | Continuation of the statement at L600: Reject nonfinite output, temperatures outside -40..85 Celsius and pressures outside 30000..110000 Pa. These are broad plausibility bounds, not a guarantee of a good sensor measurement. |
| 602 | <code>    *temperature_c = temp;</code> | Write compensated temperature. |
| 603 | <code>    *pressure_pa = pressure;</code> | Write compensated pressure. |
| 604 | <code>    return true;</code> | Report success. |
| 605 | <code>}</code> | End compensation. Calling this does not automatically feed the altitude filter; integration must supply its result. |
| 607 | <code>typedef struct { uint32_t n; double mean_pa, m2_pa2; } baro_reference_cal_t;</code> | Define ground-reference running statistics: count, mean pressure and M2 in pressure-squared units. |
| 608 | <code>typedef struct {</code> | Begin stored ground-reference calibration. |
| 609 | <code>    double ground_pressure_pa, reference_variance_pa2, measurement_variance_pa2;</code> | Store ground mean pressure, variance of that mean reference, and individual pressure-measurement variance. |
| 610 | <code>    bool valid;</code> | Flag whether a valid ground reference exists. |
| 611 | <code>} baro_calibration_t;</code> | Name the ground-reference calibration type. |
| 613 | <code>bool baro_reference_add(baro_reference_cal_t *s, double compensated_pa,</code> | Add one compensated ground-pressure sample while disarmed. |
| 614 | <code>                         bool disarmed) {</code> | Continuation of the statement at L613: Add one compensated ground-pressure sample while disarmed. |
| 615 | <code>    if (s == NULL &#124;&#124; !disarmed &#124;&#124; !isfinite(compensated_pa)</code> | Require pointer, disarmed status, finite pressure in broad allowed range and no count overflow. |
| 616 | <code>        &#124;&#124; compensated_pa &lt; 30000.0 &#124;&#124; compensated_pa &gt; 110000.0</code> | Continuation of the statement at L615: Require pointer, disarmed status, finite pressure in broad allowed range and no count overflow. |
| 617 | <code>        &#124;&#124; s-&gt;n == UINT32_MAX) return false;</code> | Continuation of the statement at L615: Require pointer, disarmed status, finite pressure in broad allowed range and no count overflow. |
| 618 | <code>    const double delta = compensated_pa - s-&gt;mean_pa;</code> | Calculate pressure-minus-old-mean. |
| 619 | <code>    s-&gt;n++;</code> | Increment count. |
| 620 | <code>    s-&gt;mean_pa += delta / s-&gt;n;</code> | Update pressure mean by delta/n. |
| 621 | <code>    s-&gt;m2_pa2 += delta * (compensated_pa - s-&gt;mean_pa);</code> | Update centered squared pressure deviations using the new mean. |
| 622 | <code>    return true;</code> | Report sample acceptance. |
| 623 | <code>}</code> | End ground-reference capture. |
| 625 | <code>bool baro_reference_finish(const baro_reference_cal_t *s, baro_calibration_t *out,</code> | Finalize a pressure reference, with a caller-specified maximum allowed pressure standard deviation. |
| 626 | <code>                            double maximum_sigma_pa, bool disarmed) {</code> | Continuation of the statement at L625: Finalize a pressure reference, with a caller-specified maximum allowed pressure standard deviation. |
| 627 | <code>    if (s == NULL &#124;&#124; out == NULL &#124;&#124; !disarmed &#124;&#124; s-&gt;n &lt; 100u</code> | Require pointers, disarmed status, at least 100 samples, positive finite sigma limit and a plausible finite mean pressure. |
| 628 | <code>        &#124;&#124; !isfinite(maximum_sigma_pa) &#124;&#124; maximum_sigma_pa &lt;= 0.0</code> | Continuation of the statement at L627: Require pointers, disarmed status, at least 100 samples, positive finite sigma limit and a plausible finite mean pressure. |
| 629 | <code>        &#124;&#124; !isfinite(s-&gt;mean_pa) &#124;&#124; s-&gt;mean_pa &lt; 30000.0</code> | Continuation of the statement at L627: Require pointers, disarmed status, at least 100 samples, positive finite sigma limit and a plausible finite mean pressure. |
| 630 | <code>        &#124;&#124; s-&gt;mean_pa &gt; 110000.0) return false;</code> | Continuation of the statement at L627: Require pointers, disarmed status, at least 100 samples, positive finite sigma limit and a plausible finite mean pressure. |
| 631 | <code>    const double variance = s-&gt;m2_pa2 / (s-&gt;n - 1u);</code> | Estimate individual-sample pressure variance with M2/(n-1). |
| 632 | <code>    if (!isfinite(variance) &#124;&#124; variance &lt; 0.0</code> | Reject invalid/negative variance or variance above maximum_sigma_pa squared. |
| 633 | <code>        &#124;&#124; variance &gt; maximum_sigma_pa * maximum_sigma_pa) return false;</code> | Continuation of the statement at L632: Reject invalid/negative variance or variance above maximum_sigma_pa squared. |
| 634 | <code>    *out = (baro_calibration_t){s-&gt;mean_pa, variance / s-&gt;n, variance, true};</code> | Store mean ground pressure, reference-mean variance variance/n, individual-measurement variance and valid=true. |
| 635 | <code>    return true;</code> | Report success. |
| 636 | <code>}</code> | End ground-reference fitting. |
| 638 | <code>bool baro_relative_height(const baro_calibration_t *c, double compensated_pa,</code> | Convert compensated pressure into height relative to calibrated ground pressure and propagate pressure uncertainty to a height sigma. |
| 639 | <code>                           double *height_m, double *sigma_m) {</code> | Continuation of the statement at L638: Convert compensated pressure into height relative to calibrated ground pressure and propagate pressure uncertainty to a height sigma. |
| 640 | <code>    if (c == NULL &#124;&#124; !c-&gt;valid &#124;&#124; height_m == NULL &#124;&#124; sigma_m == NULL</code> | Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 641 | <code>        &#124;&#124; !isfinite(compensated_pa) &#124;&#124; compensated_pa &lt; 30000.0</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 642 | <code>        &#124;&#124; compensated_pa &gt; 110000.0 &#124;&#124; !isfinite(c-&gt;ground_pressure_pa)</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 643 | <code>        &#124;&#124; c-&gt;ground_pressure_pa &lt; 30000.0 &#124;&#124; c-&gt;ground_pressure_pa &gt; 110000.0</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 644 | <code>        &#124;&#124; !isfinite(c-&gt;reference_variance_pa2) &#124;&#124; c-&gt;reference_variance_pa2 &lt; 0.0</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 645 | <code>        &#124;&#124; !isfinite(c-&gt;measurement_variance_pa2) &#124;&#124; c-&gt;measurement_variance_pa2 &lt; 0.0)</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 646 | <code>        return false;</code> | Continuation of the statement at L640: Require valid pointers/calibration, plausible finite current/reference pressures and finite nonnegative pressure variances. |
| 647 | <code>    const double exponent = 0.190294957;</code> | Set exponent for the pressure-ratio standard-atmosphere approximation. |
| 648 | <code>    const double ratio = compensated_pa / c-&gt;ground_pressure_pa;</code> | Compute pressure ratio p/p0, where p0 is the local calibrated ground reference. |
| 649 | <code>    const double derivative = 44330.0 * exponent * pow(ratio, exponent - 1.0);</code> | Calculate a shared magnitude factor 44330*a*(p/p0)^(a-1) used in the height derivatives. |
| 650 | <code>    const double dp = -derivative / c-&gt;ground_pressure_pa;</code> | Derivative dh/dp = -factor/p0: higher pressure means lower inferred altitude. |
| 651 | <code>    const double dp0 = derivative * compensated_pa</code> | Derivative dh/dp0 = factor*p/p0², accounting for uncertain ground reference. |
| 652 | <code>                      / (c-&gt;ground_pressure_pa * c-&gt;ground_pressure_pa);</code> | Continuation of the statement at L651: Derivative dh/dp0 = factor*p/p0², accounting for uncertain ground reference. |
| 653 | <code>    *height_m = 44330.0 * (1.0 - pow(ratio, exponent));</code> | Calculate relative height h = 44330*(1-(p/p0)^a). It assumes an atmospheric model; it is not a measured weather correction. |
| 654 | <code>    *sigma_m = sqrt(dp*dp*c-&gt;measurement_variance_pa2</code> | Propagate independent pressure/reference variances: sigma_h = sqrt((dh/dp)²*Var(p)+(dh/dp0)²*Var(p0)). Correlation and atmospheric-model uncertainty are not included. |
| 655 | <code>                    + dp0*dp0*c-&gt;reference_variance_pa2);</code> | Continuation of the statement at L654: Propagate independent pressure/reference variances: sigma_h = sqrt((dh/dp)²*Var(p)+(dh/dp0)²*Var(p0)). Correlation and atmospheric-model uncertainty are not included. |
| 656 | <code>    return isfinite(*height_m) &amp;&amp; isfinite(*sigma_m);</code> | Return true only for finite output height and sigma. |
| 657 | <code>}</code> | End relative-height conversion. |

### L659: Generic linear reference calibration

| Line | Exact C source | Explanation |
|---:|---|---|
| 659 | <code>/* Reference-pair regression for battery volts, current zero/gain, or lidar.</code> | Introduce a reusable linear reference calibration y=gain*x+offset for voltage/current/lidar. Reference data must come from independently known physical values. |
| 660 | <code> * Example: input ADC volts, reference trusted multimeter volts / measured amps.</code> | Continuation of the statement at L659: Introduce a reusable linear reference calibration y=gain*x+offset for voltage/current/lidar. Reference data must come from independently known physical values. |
| 661 | <code> * For lidar, reference a measured target distance; terrain is not a reference.</code> | Continuation of the statement at L659: Introduce a reusable linear reference calibration y=gain*x+offset for voltage/current/lidar. Reference data must come from independently known physical values. |
| 662 | <code> */</code> | Continuation of the statement at L659: Introduce a reusable linear reference calibration y=gain*x+offset for voltage/current/lidar. Reference data must come from independently known physical values. |
| 663 | <code>typedef struct { uint32_t n; double x, y, xx, yy, xy; } linear_cal_fit_t;</code> | Store online paired statistics: n, means x/y, centered sums xx/yy and cross-product xy. |
| 664 | <code>typedef struct { double gain, offset, residual_sigma; bool valid; } linear_cal_t;</code> | Store fitted gain, offset, residual standard deviation and validity. |
| 666 | <code>bool linear_cal_add(linear_cal_fit_t *s, double input, double reference,</code> | Accumulate one calibration input/reference pair while disarmed. |
| 667 | <code>                      bool disarmed) {</code> | Continuation of the statement at L666: Accumulate one calibration input/reference pair while disarmed. |
| 668 | <code>    if (s == NULL &#124;&#124; !disarmed &#124;&#124; !isfinite(input) &#124;&#124; !isfinite(reference)</code> | Require pointer, disarmed state, finite values bounded in magnitude by 1e9 and no count overflow. |
| 669 | <code>        &#124;&#124; fabs(input) &gt; 1e9 &#124;&#124; fabs(reference) &gt; 1e9 &#124;&#124; s-&gt;n == UINT32_MAX)</code> | Continuation of the statement at L668: Require pointer, disarmed state, finite values bounded in magnitude by 1e9 and no count overflow. |
| 670 | <code>        return false;</code> | Continuation of the statement at L668: Require pointer, disarmed state, finite values bounded in magnitude by 1e9 and no count overflow. |
| 671 | <code>    const double dx = input - s-&gt;x, dy = reference - s-&gt;y;</code> | Compute input and reference differences from their old means. |
| 672 | <code>    s-&gt;n++;</code> | Increment count. |
| 673 | <code>    s-&gt;x += dx / s-&gt;n;</code> | Update input mean. |
| 674 | <code>    s-&gt;y += dy / s-&gt;n;</code> | Update reference mean. |
| 675 | <code>    s-&gt;xx += dx * (input - s-&gt;x);</code> | Update centered input sum of squares. |
| 676 | <code>    s-&gt;yy += dy * (reference - s-&gt;y);</code> | Update centered reference sum of squares. |
| 677 | <code>    s-&gt;xy += dx * (reference - s-&gt;y);</code> | Update centered input/reference cross-product using the old input difference and new reference mean. |
| 678 | <code>    return true;</code> | Report successful accumulation. |
| 679 | <code>}</code> | End paired capture. |
| 681 | <code>bool linear_cal_finish(const linear_cal_fit_t *s, linear_cal_t *out,</code> | Fit a straight line and reject excessive residual scatter, taking the maximum acceptable sigma from the caller. |
| 682 | <code>                         double maximum_residual_sigma, bool disarmed) {</code> | Continuation of the statement at L681: Fit a straight line and reject excessive residual scatter, taking the maximum acceptable sigma from the caller. |
| 683 | <code>    if (s == NULL &#124;&#124; out == NULL &#124;&#124; !disarmed &#124;&#124; s-&gt;n &lt; 3u</code> | Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit. |
| 684 | <code>        &#124;&#124; !isfinite(s-&gt;xx) &#124;&#124; !isfinite(s-&gt;yy) &#124;&#124; !isfinite(s-&gt;xy)</code> | Continuation of the statement at L683: Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit. |
| 685 | <code>        &#124;&#124; !isfinite(s-&gt;x) &#124;&#124; !isfinite(s-&gt;y) &#124;&#124; s-&gt;yy &lt; 0.0 &#124;&#124; s-&gt;xx &lt;= 1e-8</code> | Continuation of the statement at L683: Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit. |
| 686 | <code>        &#124;&#124; !isfinite(maximum_residual_sigma) &#124;&#124; maximum_residual_sigma &lt;= 0.0)</code> | Continuation of the statement at L683: Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit. |
| 687 | <code>        return false;</code> | Continuation of the statement at L683: Require valid pointers/disarmed state, at least three pairs, finite statistics, nonnegative reference variation, sufficient input variation and a positive finite residual limit. |
| 688 | <code>    const double gain = s-&gt;xy / s-&gt;xx, offset = s-&gt;y - gain * s-&gt;x;</code> | Least-squares gain is Sxy/Sxx; offset is mean_y - gain*mean_x. |
| 689 | <code>    const double residual = sqrt(fmax(0.0, s-&gt;yy - gain*s-&gt;xy) / (s-&gt;n - 2u));</code> | Estimate residual sigma = sqrt(max(0,Syy-gain*Sxy)/(n-2)). Two fitted parameters use two degrees of freedom. |
| 690 | <code>    if (!isfinite(gain) &#124;&#124; fabs(gain) &lt; 1e-12 &#124;&#124; !isfinite(offset)</code> | Reject a near-zero/nonfinite gain, nonfinite offset or residual, or residual sigma above the caller's bound. Negative gains are permitted for inverted electrical relationships. |
| 691 | <code>        &#124;&#124; !isfinite(residual) &#124;&#124; residual &gt; maximum_residual_sigma)</code> | Continuation of the statement at L690: Reject a near-zero/nonfinite gain, nonfinite offset or residual, or residual sigma above the caller's bound. Negative gains are permitted for inverted electrical relationships. |
| 692 | <code>        return false;</code> | Continuation of the statement at L690: Reject a near-zero/nonfinite gain, nonfinite offset or residual, or residual sigma above the caller's bound. Negative gains are permitted for inverted electrical relationships. |
| 693 | <code>    *out = (linear_cal_t){gain, offset, residual, true};</code> | Commit the accepted gain, offset, residual and validity. |
| 694 | <code>    return true;</code> | Report success. |
| 695 | <code>}</code> | End line fitting. |
| 697 | <code>bool linear_cal_apply(const linear_cal_t *c, double raw, double *corrected) {</code> | Apply a fitted linear sensor conversion. |
| 698 | <code>    if (c == NULL &#124;&#124; !c-&gt;valid &#124;&#124; corrected == NULL &#124;&#124; !isfinite(raw)</code> | Require valid calibration/input/output and finite gain/offset. |
| 699 | <code>        &#124;&#124; !isfinite(c-&gt;gain) &#124;&#124; !isfinite(c-&gt;offset)) return false;</code> | Continuation of the statement at L698: Require valid calibration/input/output and finite gain/offset. |
| 700 | <code>    const double value = c-&gt;gain * raw + c-&gt;offset;</code> | Calculate corrected = gain*raw + offset. |
| 701 | <code>    if (!isfinite(value)) return false;</code> | Reject nonfinite converted output. |
| 702 | <code>    *corrected = value;</code> | Write the result only after successful conversion. |
| 703 | <code>    return true;</code> | Report success. |
| 704 | <code>}</code> | End the generic reference conversion. |

### L706: PID, estimator, command and sensor structures

| Line | Exact C source | Explanation |
|---:|---|---|
| 706 | <code>typedef struct {</code> | Begin generic PID state/configuration type. |
| 707 | <code>    float kp;</code> | Proportional gain Kp. |
| 708 | <code>    float ki;</code> | Integral gain Ki, multiplying accumulated error-times-seconds. |
| 709 | <code>    float kd;</code> | Derivative gain Kd; the generic PID differentiates measurement, not setpoint. |
| 710 | <code>    float integrator;</code> | Stored integral of error, before multiplying by Ki. |
| 711 | <code>    float previous_measurement;</code> | Previous measured value, needed to estimate its time derivative. |
| 712 | <code>    float integrator_limit;</code> | Magnitude bound on stored error integral, not directly on the I-term output. |
| 713 | <code>    float output_limit;</code> | Symmetric magnitude bound on the complete PID output. |
| 714 | <code>    bool initialized;</code> | Whether a previous measurement exists, suppressing the first derivative spike. |
| 715 | <code>} pid_t;</code> | Name the PID type. Six instances follow; some use zero I/D gains. |
| 717 | <code>typedef struct {</code> | Begin the per-axis angle/bias Kalman state. |
| 718 | <code>    float angle_deg;</code> | Estimated roll or pitch angle in degrees. |
| 719 | <code>    float gyro_bias_dps;</code> | Estimated residual gyro bias in degrees/s. This is additional to optional fixed calibration bias removal. |
| 720 | <code>    float p00;</code> | P00: variance of estimated angle, in degrees squared. |
| 721 | <code>    float p01;</code> | P01: covariance between angle and gyro bias. |
| 722 | <code>    float p10;</code> | P10: transpose covariance; ideally equals P01. |
| 723 | <code>    float p11;</code> | P11: gyro-bias variance in (degrees/s) squared. |
| 724 | <code>    float gyro_noise;</code> | Per-sample gyro-rate measurement variance; its contribution to angle variance is multiplied by dt squared. |
| 725 | <code>    float bias_walk;</code> | Continuous bias random-walk intensity, in (degrees/s) squared per second. |
| 726 | <code>    float accel_noise;</code> | Accelerometer-derived angle observation variance R in degrees squared; despite the name it is not acceleration variance in g squared. |
| 727 | <code>    float gain_angle, gain_bias, innovation, innovation_variance;</code> | Last angle/bias gains, innovation and innovation variance, retained for diagnostics/graphs. |
| 728 | <code>} angle_kf_t;</code> | Name the angle filter type. Although one axis is estimated, it contains TWO states: angle and bias. |
| 730 | <code>typedef struct {</code> | Begin the altitude/vertical-velocity Kalman state. |
| 731 | <code>    float height_m;</code> | Estimated upward-positive height in metres. |
| 732 | <code>    float velocity_mps;</code> | Estimated upward-positive vertical velocity in metres/s. |
| 733 | <code>    float p00;</code> | P00: height variance in m squared. |
| 734 | <code>    float p01;</code> | P01: height/vertical-speed covariance. |
| 735 | <code>    float p10;</code> | P10: transpose covariance. |
| 736 | <code>    float p11;</code> | P11: vertical-speed variance in (m/s) squared. |
| 737 | <code>    float accel_noise;</code> | Acceleration noise standard deviation in m/s squared; prediction squares it to obtain variance. This differs from angle_kf_t's gyro_noise convention. |
| 738 | <code>    float baro_noise;</code> | Height-measurement variance R in m squared, despite the name baro_noise. |
| 739 | <code>    float gain_height, gain_velocity, innovation, innovation_variance;</code> | Last height/velocity gains, height innovation and innovation variance for diagnostics. |
| 740 | <code>} altitude_kf_t;</code> | Name the two-state altitude filter type. |
| 742 | <code>typedef struct {</code> | Begin the internal pilot/companion command type after packet scaling. |
| 743 | <code>    float roll_deg;</code> | Requested roll angle in degrees. |
| 744 | <code>    float pitch_deg;</code> | Requested pitch angle in degrees. |
| 745 | <code>    float yaw_rate_dps;</code> | Requested yaw angular rate in degrees/s; no absolute-yaw heading target exists. |
| 746 | <code>    float throttle; /* [0,1], center requests zero vertical speed */</code> | Centered throttle in 0..1; 0.5 requests zero vertical speed. |
| 747 | <code>    float boost;    /* RT [0,1]: extra climb/collective allowance, same flight mode */</code> | RT boost in 0..1, which expands climb/collective limits without selecting a new flight mode. |
| 748 | <code>    bool valid;</code> | Whether the command is valid/fresh according to its source. |
| 749 | <code>    bool arm_request;</code> | Reserved arm-request field. Current math/runtime never uses it to arm hardware. |
| 750 | <code>} pilot_command_t;</code> | Name the internal command type. |
| 752 | <code>typedef struct {</code> | Begin one sensor sample as consumed by the control core. |
| 753 | <code>    float accel_x_g;</code> | Body/sensor-frame x acceleration in g, after the agreed frame mapping and calibration. |
| 754 | <code>    float accel_y_g;</code> | y acceleration in g. |
| 755 | <code>    float accel_z_g;</code> | z acceleration in g. |
| 756 | <code>    float gyro_x_dps;</code> | x-axis angular rate in degrees/s. |
| 757 | <code>    float gyro_y_dps;</code> | y-axis angular rate in degrees/s. |
| 758 | <code>    float gyro_z_dps;</code> | z-axis angular rate in degrees/s; used as the yaw-rate measurement. |
| 759 | <code>    float vertical_acceleration_mps2; /* gravity compensated, world-up positive */</code> | Separately supplied upward world acceleration in m/s squared AFTER rotation and gravity compensation; the core does not calculate this rotation itself. |
| 760 | <code>    float barometer_height_m;</code> | Relative barometer height in metres, already pressure-compensated and referenced by its producer. |
| 761 | <code>    bool imu_valid;</code> | Flag that IMU data is valid; there is no sample timestamp in this structure. |
| 762 | <code>    bool imu_calibrated; /* scaled + offset/gain corrected before control */</code> | Flag that IMU axis scale/bias calibration has been applied. |
| 763 | <code>    bool barometer_valid;</code> | Flag that height aiding is valid. |
| 764 | <code>    bool barometer_updated; /* apply each independent pressure sample once */</code> | Flag that this is a new independent pressure sample. It prevents correcting the filter repeatedly with one stale repeated measurement. |
| 765 | <code>} sensor_sample_t;</code> | Name the sensor sample type. |
| 767 | <code>bool sensor_sample_apply_imu_calibration(sensor_sample_t *s,</code> | Apply IMU calibration in-place to one sample before control. |
| 768 | <code>                                          const imu_calibration_t *cal) {</code> | Continuation of the statement at L767: Apply IMU calibration in-place to one sample before control. |
| 769 | <code>    if (s == NULL &#124;&#124; !s-&gt;imu_valid &#124;&#124; s-&gt;imu_calibrated) return false;</code> | Require a sample with valid IMU, and reject already-calibrated samples to avoid correcting the same data twice. |
| 770 | <code>    const float accel[3] = {s-&gt;accel_x_g, s-&gt;accel_y_g, s-&gt;accel_z_g};</code> | Copy acceleration components into an input vector. |
| 771 | <code>    const float gyro[3] = {s-&gt;gyro_x_dps, s-&gt;gyro_y_dps, s-&gt;gyro_z_dps};</code> | Copy angular rates into another input vector. |
| 772 | <code>    float a[3], g[3];</code> | Prepare temporary corrected outputs. |
| 773 | <code>    if (!imu_cal_apply(cal, accel, gyro, a, g)) return false;</code> | Use the earlier imu_cal_apply helper; return false if calibration/inputs are invalid. |
| 774 | <code>    s-&gt;accel_x_g = a[0]; s-&gt;accel_y_g = a[1]; s-&gt;accel_z_g = a[2];</code> | Write corrected accelerometer components back to the sample. |
| 775 | <code>    s-&gt;gyro_x_dps = g[0]; s-&gt;gyro_y_dps = g[1]; s-&gt;gyro_z_dps = g[2];</code> | Write corrected gyro components back to the sample. |
| 776 | <code>    s-&gt;imu_calibrated = true;</code> | Mark this sample calibrated. |
| 777 | <code>    /* Derive world-up acceleration AFTER this correction and frame rotation. */</code> | Document that world-up acceleration must be derived after correction AND frame rotation. This function does not perform that extra step. |
| 778 | <code>    return true;</code> | Report success. |
| 779 | <code>}</code> | End in-place calibration. |
| 781 | <code>typedef struct {</code> | Begin control result structure. |
| 782 | <code>    float motor[4]; /* normalized math result only; no ESC output is connected */</code> | Four normalized motor demands, mathematical results only. These are not connected ESC pulses and need not be linear in physical thrust on real hardware. |
| 783 | <code>    bool armed;</code> | Hardware-arming flag; it is always false in this scaffold. |
| 784 | <code>    bool sensor_fault;</code> | Fault flag, also used for invalid pilot command/height aiding rather than exclusively IMU faults. |
| 785 | <code>} control_output_t;</code> | Name the control result type. |
| 787 | <code>typedef struct {</code> | Begin persistent pilot-link tracking state. |
| 788 | <code>    pilot_command_t latest;</code> | Store the most recently accepted physical-unit command. |
| 789 | <code>    uint16_t last_sequence;</code> | Store its sequence number for stale/duplicate ordering checks. |
| 790 | <code>    uint64_t last_received_us;</code> | Store local reception time in microseconds. |
| 791 | <code>    bool received;</code> | Record whether any frame has ever been accepted. |
| 792 | <code>} pilot_link_state_t;</code> | Name the link state. |

### L794: Pilot packet scaling, freshness and initial control settings

| Line | Exact C source | Explanation |
|---:|---|---|
| 794 | <code>static bool pilot_link_accept(pilot_link_state_t *link, const uint8_t *bytes,</code> | Accept one received radio packet and its local timestamp into link state; this function does not itself read the radio. |
| 795 | <code>                              size_t length, uint64_t received_us) {</code> | Continuation of the statement at L794: Accept one received radio packet and its local timestamp into link state; this function does not itself read the radio. |
| 796 | <code>    qcopter_command_frame_t frame;</code> | Prepare a temporary decoded frame. |
| 797 | <code>    if (link == NULL &#124;&#124; !qcopter_decode_frame(bytes, length, &amp;frame)) return false;</code> | Require a link pointer and successfully decoded, semantically valid packet. |
| 798 | <code>    if (link-&gt;received</code> | After the first frame, reject duplicates/older sequence numbers using the wrap comparator. |
| 799 | <code>        &amp;&amp; !qcopter_sequence_is_newer(frame.sequence, link-&gt;last_sequence))</code> | Continuation of the statement at L798: After the first frame, reject duplicates/older sequence numbers using the wrap comparator. |
| 800 | <code>        return false;</code> | Continuation of the statement at L798: After the first frame, reject duplicates/older sequence numbers using the wrap comparator. |
| 801 | <code>    link-&gt;latest = (pilot_command_t){</code> | Construct the new internal pilot command. |
| 802 | <code>        .roll_deg = (float)frame.roll * 35.0f / QCOPTER_AXIS_SCALE,</code> | Map full signed roll stick to -35..+35 degrees. |
| 803 | <code>        .pitch_deg = (float)frame.pitch * 35.0f / QCOPTER_AXIS_SCALE,</code> | Map full signed pitch stick to -35..+35 degrees. |
| 804 | <code>        .yaw_rate_dps = (float)frame.yaw_rate * 180.0f / QCOPTER_AXIS_SCALE,</code> | Map full signed yaw stick to -180..+180 degrees/s. |
| 805 | <code>        .throttle = (float)frame.throttle / QCOPTER_THROTTLE_SCALE,</code> | Convert integer throttle to 0..1; midpoint 0.5 means hold current height reference. |
| 806 | <code>        .boost = (float)frame.rt / QCOPTER_TRIGGER_SCALE,</code> | Map RT to boost 0..1. LT/buttons are validated in the frame but not used here for flight control. |
| 807 | <code>        .valid = true,</code> | Mark the just-decoded command valid. |
| 808 | <code>        .arm_request = false /* no arming gesture is defined or enabled yet */</code> | Force arm_request false; no physical arming gesture is enabled. |
| 809 | <code>    };</code> | Complete the command initializer. |
| 810 | <code>    link-&gt;last_sequence = frame.sequence;</code> | Save the sequence of the accepted frame. |
| 811 | <code>    link-&gt;last_received_us = received_us;</code> | Save reception timestamp. |
| 812 | <code>    link-&gt;received = true;</code> | Mark that a frame was received. |
| 813 | <code>    return true;</code> | Report acceptance. |
| 814 | <code>}</code> | End frame acceptance. |
| 816 | <code>static pilot_command_t pilot_link_current(const pilot_link_state_t *link,</code> | Return the current pilot command if still fresh at the supplied local time. |
| 817 | <code>                                          uint64_t now_us) {</code> | Continuation of the statement at L816: Return the current pilot command if still fresh at the supplied local time. |
| 818 | <code>    pilot_command_t command = {0};</code> | Initialize a zero-filled command, including valid=false and throttle=0. |
| 819 | <code>    if (link != NULL &amp;&amp; link-&gt;received &amp;&amp; now_us &gt;= link-&gt;last_received_us</code> | Require an existing link, at least one frame, non-backward local time and packet age <=150 ms. |
| 820 | <code>        &amp;&amp; now_us - link-&gt;last_received_us &lt;= FC_RADIO_TIMEOUT_US)</code> | Continuation of the statement at L819: Require an existing link, at least one frame, non-backward local time and packet age <=150 ms. |
| 821 | <code>        command = link-&gt;latest;</code> | For a fresh packet, return its saved command. |
| 822 | <code>    else</code> | Otherwise take the stale/no-link branch. |
| 823 | <code>        command.valid = false;</code> | Explicitly mark the zero command invalid. Its zero throttle is not treated as a valid descent request because valid=false is checked downstream. |
| 824 | <code>    return command;</code> | Return the chosen command. |
| 825 | <code>}</code> | End link freshness handling. The current physical scaffold has no airborne recovery/landing strategy for a stale link. |
| 827 | <code>static angle_kf_t roll_estimator = {</code> | Create one persistent roll angle/bias estimator, private to this file. |
| 828 | <code>    .p00 = 1.0f, .p11 = 0.01f,</code> | Initialize angle variance to 1 degree squared and bias variance to 0.01 (degrees/s) squared; unspecified members begin at zero. |
| 829 | <code>    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f</code> | Set gyro-rate variance 0.04, bias random-walk intensity 0.0001 and accelerometer-angle variance 4 degrees squared. These are provisional noise assumptions. |
| 830 | <code>};</code> | End roll estimator initializer. |
| 831 | <code>static angle_kf_t pitch_estimator = {</code> | Create a separate persistent pitch angle/bias estimator. |
| 832 | <code>    .p00 = 1.0f, .p11 = 0.01f,</code> | Give pitch the same initial diagonal covariance. |
| 833 | <code>    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f</code> | Give pitch the same provisional noise parameters. |
| 834 | <code>};</code> | End pitch initialization. |
| 835 | <code>static altitude_kf_t altitude_estimator = {</code> | Create the persistent height/velocity estimator. |
| 836 | <code>    .p00 = 1.0f, .p11 = 1.0f,</code> | Initialize height and velocity variances to 1 in their respective squared units. |
| 837 | <code>    .accel_noise = 1.5f, .baro_noise = 2.0f</code> | Set acceleration sigma to 1.5 m/s² and height observation variance to 2 m². Host simulation reset later uses a different barometer variance; do not confuse these defaults. |
| 838 | <code>};</code> | End altitude-estimator initializer. |
| 840 | <code>/* Gains are placeholders for simulation/bench tuning, not flight values. */</code> | Label the PID gains as simulation/bench candidates rather than identified real-aircraft gains. |
| 841 | <code>static pid_t roll_angle_pid = {</code> | Create the outer roll-angle controller. |
| 842 | <code>    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,</code> | Set Kp=3, Ki=Kd=0: this instance is presently a P controller despite using pid_t. |
| 843 | <code>    .integrator_limit = 0.0f, .output_limit = 180.0f</code> | Disable its integrator and cap its angular-rate output at +/-180 degrees/s. |
| 844 | <code>};</code> | End outer-roll initialization. |
| 845 | <code>static pid_t pitch_angle_pid = {</code> | Create the outer pitch-angle controller. |
| 846 | <code>    .kp = 3.0f, .ki = 0.0f, .kd = 0.0f,</code> | Use the same P-only gain 3. |
| 847 | <code>    .integrator_limit = 0.0f, .output_limit = 180.0f</code> | Disable integral accumulation and limit pitch-rate command to +/-180 degrees/s. |
| 848 | <code>};</code> | End outer-pitch initialization. |
| 849 | <code>static pid_t roll_rate_pid = {</code> | Create the inner roll-rate controller. |
| 850 | <code>    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,</code> | Set provisional Kp=0.000829338, Ki=0.003870245 and Kd=0: currently PI. |
| 851 | <code>    .integrator_limit = 15.0f, .output_limit = 0.35f</code> | Bound its error integral to +/-15 and normalized roll correction to +/-0.35. |
| 852 | <code>};</code> | End inner-roll initialization. |
| 853 | <code>static pid_t pitch_rate_pid = {</code> | Create the inner pitch-rate controller. |
| 854 | <code>    .kp = 0.000829338f, .ki = 0.003870245f, .kd = 0.0f,</code> | Use the same provisional PI gains as roll rate. |
| 855 | <code>    .integrator_limit = 15.0f, .output_limit = 0.35f</code> | Use the same integral and output limits. |
| 856 | <code>};</code> | End inner-pitch initialization. |
| 857 | <code>static pid_t yaw_rate_pid = {</code> | Create the yaw-rate controller, without an outer yaw-angle controller. |
| 858 | <code>    .kp = 0.005375614f, .ki = 0.008362066f, .kd = 0.0f,</code> | Set provisional yaw PI gains Kp=0.005375614, Ki=0.008362066, Kd=0. |
| 859 | <code>    .integrator_limit = 8.0f, .output_limit = 0.35f</code> | Limit yaw error integral to +/-8 and normalized yaw correction to +/-0.35. |
| 860 | <code>};</code> | End yaw initialization. |
| 861 | <code>static pid_t altitude_pid = {</code> | Create the sole altitude PID. Vertical speed feedback is its D/error-derivative term, not a seventh PID instance. |
| 862 | <code>    .kp = 0.108045f, .ki = 0.03087f, .kd = 0.1225f,</code> | Set provisional height gains Kp=0.108045, Ki=0.03087, Kd=0.1225. |
| 863 | <code>    .integrator_limit = 2.0f, .output_limit = 0.35f</code> | Bound height-error integral to +/-2 metre-seconds and normalized collective correction to +/-0.35. |
| 864 | <code>};</code> | End altitude-PID initialization. |
| 865 | <code>static float altitude_hold_reference_m;</code> | Persistent target height in metres; initially zero by static initialization, then captured from the estimator when control first becomes valid. |
| 866 | <code>static float hover_collective = FC_HOVER_THROTTLE_PLACEHOLDER;</code> | Persistent hover feed-forward, initially the unmeasured 0.50 normalized value. |
| 867 | <code>static float boost_state;</code> | Persistent smoothed boost value, initially zero. |
| 868 | <code>static bool altitude_reference_initialized;</code> | Flag that height reference has been initialized. |

### L870: Clamping, resets and generic PID

| Line | Exact C source | Explanation |
|---:|---|---|
| 870 | <code>static float clampf(float value, float low, float high) {</code> | Declare a floating-point clamp helper. |
| 871 | <code>    if (value &lt; low) return low;</code> | Return the lower bound if value is too small. |
| 872 | <code>    if (value &gt; high) return high;</code> | Return the upper bound if value is too large. |
| 873 | <code>    return value;</code> | Otherwise return value unchanged. This function alone does not reject NaN; callers must validate their data. |
| 874 | <code>}</code> | End clamp helper. |
| 876 | <code>static void pid_reset(pid_t *pid) {</code> | Reset a PID's memory without changing gains or limits. |
| 877 | <code>    pid-&gt;integrator = 0.0f;</code> | Clear the accumulated error integral. |
| 878 | <code>    pid-&gt;previous_measurement = 0.0f;</code> | Clear the previous measurement. |
| 879 | <code>    pid-&gt;initialized = false;</code> | Mark derivative history uninitialized. |
| 880 | <code>}</code> | End PID reset. |
| 882 | <code>static void reset_control_state(void) {</code> | Reset all control-memory state, but not the three Kalman estimators or hover feed-forward. |
| 883 | <code>    boost_state=0;</code> | Immediately reset the boost ramp to zero. |
| 884 | <code>    pid_reset(&amp;roll_angle_pid);</code> | Reset the outer roll PID memory. |
| 885 | <code>    pid_reset(&amp;pitch_angle_pid);</code> | Reset the outer pitch PID memory. |
| 886 | <code>    pid_reset(&amp;roll_rate_pid);</code> | Reset inner roll-rate memory. |
| 887 | <code>    pid_reset(&amp;pitch_rate_pid);</code> | Reset inner pitch-rate memory. |
| 888 | <code>    pid_reset(&amp;yaw_rate_pid);</code> | Reset yaw-rate memory. |
| 889 | <code>    pid_reset(&amp;altitude_pid);</code> | Reset altitude-PID memory. |
| 890 | <code>    altitude_reference_initialized = false;</code> | Require a fresh capture of the height reference when valid control resumes. |
| 891 | <code>}</code> | End control reset. |
| 893 | <code>/* Derivative is taken on the measurement to avoid setpoint kick. */</code> | Explain derivative-on-measurement: changing a target does not directly create a derivative spike. |
| 894 | <code>static float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {</code> | Define a generic PID update, taking state/config, target, measurement and elapsed seconds. |
| 895 | <code>    if (!(dt &gt; 0.0f) &#124;&#124; dt &gt; FC_MAX_CONTROL_DT) {</code> | Reject dt that is not positive (including NaN in this comparison) or exceeds 10 ms. The outer control step also checks finiteness explicitly. |
| 896 | <code>        pid_reset(pid);</code> | On invalid timing reset this PID's memory. |
| 897 | <code>        return 0.0f;</code> | Return zero correction on invalid timing. |
| 898 | <code>    }</code> | End invalid-time branch. |
| 899 | <code>    const float error = setpoint - measurement;</code> | Compute control error e = setpoint - measurement. |
| 900 | <code>    const float derivative = pid-&gt;initialized</code> | If derivative history exists, compute measurement slope (y_k-y_previous)/dt; otherwise use zero for the first sample. |
| 901 | <code>        ? (measurement - pid-&gt;previous_measurement) / dt : 0.0f;</code> | Continuation of the statement at L900: If derivative history exists, compute measurement slope (y_k-y_previous)/dt; otherwise use zero for the first sample. |
| 902 | <code>    const float candidate_i = clampf(pid-&gt;integrator + error * dt,</code> | Form a bounded candidate integral I_candidate = clamp(I_old+e*dt, +/-integrator_limit). |
| 903 | <code>                                     -pid-&gt;integrator_limit,</code> | Continuation of the statement at L902: Form a bounded candidate integral I_candidate = clamp(I_old+e*dt, +/-integrator_limit). |
| 904 | <code>                                     pid-&gt;integrator_limit);</code> | Continuation of the statement at L902: Form a bounded candidate integral I_candidate = clamp(I_old+e*dt, +/-integrator_limit). |
| 905 | <code>    const float raw = pid-&gt;kp * error + pid-&gt;ki * candidate_i</code> | Calculate raw output u_raw=Kp*e+Ki*I_candidate-Kd*measurement_slope. The derivative has a minus sign because increasing measurement should oppose positive error. |
| 906 | <code>                    - pid-&gt;kd * derivative;</code> | Continuation of the statement at L905: Calculate raw output u_raw=Kp*e+Ki*I_candidate-Kd*measurement_slope. The derivative has a minus sign because increasing measurement should oppose positive error. |
| 907 | <code>    const float output = clampf(raw, -pid-&gt;output_limit, pid-&gt;output_limit);</code> | Clamp raw correction to +/-output_limit. |
| 908 | <code>    /* Conditional integration prevents windup when saturation is driven farther. */</code> | Introduce conditional-integration anti-windup. |
| 909 | <code>    if (raw == output &#124;&#124; (raw &gt; output &amp;&amp; error &lt; 0.0f)</code> | Accept integral accumulation when unsaturated, or when error drives a saturated command back toward the allowed range. This checks this PID's own limiter, not later collective/mixer saturation. |
| 910 | <code>                      &#124;&#124; (raw &lt; output &amp;&amp; error &gt; 0.0f)) {</code> | Continuation of the statement at L909: Accept integral accumulation when unsaturated, or when error drives a saturated command back toward the allowed range. This checks this PID's own limiter, not later collective/mixer saturation. |
| 911 | <code>        pid-&gt;integrator = candidate_i;</code> | Commit the candidate integral only under that condition. |
| 912 | <code>    }</code> | End integral-acceptance branch. |
| 913 | <code>    pid-&gt;previous_measurement = measurement;</code> | Save the current measurement for next derivative calculation. |
| 914 | <code>    pid-&gt;initialized = true;</code> | Mark derivative history initialized. |
| 915 | <code>    return output;</code> | Return the bounded correction. |
| 916 | <code>}</code> | End the generic PID. There is no derivative low-pass filter in this implementation. |

### L918: Kalman prediction and correction mathematics

| Line | Exact C source | Explanation |
|---:|---|---|
| 918 | <code>/* Joseph scalar-observation covariance update, H=[1,0]. */</code> | Introduce Joseph-form covariance correction for a two-state filter observed through its first component: H=[1,0]. |
| 919 | <code>static void covariance_correct(float *p00, float *p01, float *p10, float *p11,</code> | Receive writable covariance entries, two Kalman gains and scalar observation variance R. Pointer arguments allow changing the caller's filter in-place. |
| 920 | <code>                                float k0, float k1, float r) {</code> | Continuation of the statement at L919: Receive writable covariance entries, two Kalman gains and scalar observation variance R. Pointer arguments allow changing the caller's filter in-place. |
| 921 | <code>    const float a = *p00, b = (*p01 + *p10)*0.5f, d = *p11;</code> | Copy old diagonal entries and average the two off-diagonals, removing small symmetry discrepancies before correction. |
| 922 | <code>    const float z = 1.0f-k0;</code> | Set z=1-K0, the first diagonal term of I-KH. |
| 923 | <code>    *p00 = z*z*a + k0*k0*r;</code> | Compute P00_plus=z²*P00_old+K0²*R. |
| 924 | <code>    *p01 = z*(b-k1*a) + k0*k1*r;</code> | Compute P01_plus=z*(P01_old-K1*P00_old)+K0*K1*R. |
| 925 | <code>    *p10 = *p01;</code> | Set P10_plus exactly equal to P01_plus to preserve covariance symmetry. |
| 926 | <code>    *p11 = d-2.0f*k1*b+k1*k1*a+k1*k1*r;</code> | Compute P11_plus=P11_old-2*K1*P01_old+K1²*P00_old+K1²*R. |
| 927 | <code>}</code> | End covariance correction. Algebraically this is (I-KH)P(I-KH)^T + K R K^T. |
| 929 | <code>static void angle_kf_update(angle_kf_t *kf, float gyro_dps,</code> | Define one angle/bias filter update for an individual axis, using gyro rate, accelerometer-derived angle and elapsed seconds. |
| 930 | <code>                            float accel_angle_deg, float dt) {</code> | Continuation of the statement at L929: Define one angle/bias filter update for an individual axis, using gyro rate, accelerometer-derived angle and elapsed seconds. |
| 931 | <code>    kf-&gt;angle_deg += dt*(gyro_dps-kf-&gt;gyro_bias_dps);</code> | Predict angle by integrating bias-corrected rate: angle_minus=angle_old+dt*(gyro-bias_old). Bias prediction is constant and therefore needs no assignment. |
| 932 | <code>    const float a=kf-&gt;p00, b=kf-&gt;p01, c=kf-&gt;p10, d=kf-&gt;p11;</code> | Save all old covariance entries before modifying any of them. |
| 933 | <code>    const float qb=kf-&gt;bias_walk;</code> | Read bias random-walk intensity qb. |
| 934 | <code>    /* gyro_noise is per-sample (deg/s)^2; bias_walk is (deg/s)^2/s. */</code> | Document noise units: gyro_noise is rate variance per sample; bias_walk is variance growth per second. |
| 935 | <code>    kf-&gt;p00=a-dt*(b+c)+dt*dt*d+kf-&gt;gyro_noise*dt*dt+qb*dt*dt*dt/3.0f;</code> | Predict angle variance: a-dt*(b+c)+dt²*d + gyroVariance*dt² + qb*dt³/3. The negative cross terms come from subtracting gyro bias in angle propagation. |
| 936 | <code>    kf-&gt;p01=b-dt*d-qb*dt*dt/2.0f;</code> | Predict angle/bias cross covariance b-dt*d-qb*dt²/2. |
| 937 | <code>    kf-&gt;p10=c-dt*d-qb*dt*dt/2.0f;</code> | Predict its transpose using old c; the process covariance cross term has the same negative sign. |
| 938 | <code>    kf-&gt;p11=d+qb*dt;</code> | Predict bias variance d+qb*dt. |
| 939 | <code>    kf-&gt;innovation_variance=kf-&gt;p00+kf-&gt;accel_noise;</code> | Calculate innovation variance S=P00_minus+R_angle. |
| 940 | <code>    kf-&gt;gain_angle=kf-&gt;p00/kf-&gt;innovation_variance;</code> | Calculate angle gain K_angle=P00_minus/S. |
| 941 | <code>    kf-&gt;gain_bias=kf-&gt;p10/kf-&gt;innovation_variance;</code> | Calculate bias gain K_bias=P10_minus/S. This gain is generally not constrained to 0..1; its units differ from the angle gain. |
| 942 | <code>    kf-&gt;innovation=accel_angle_deg-kf-&gt;angle_deg;</code> | Innovation nu=accelerometer_angle-predicted_angle. |
| 943 | <code>    kf-&gt;angle_deg+=kf-&gt;gain_angle*kf-&gt;innovation;</code> | Correct angle by K_angle*nu. |
| 944 | <code>    kf-&gt;gyro_bias_dps+=kf-&gt;gain_bias*kf-&gt;innovation;</code> | Correct residual gyro bias by K_bias*nu. |
| 945 | <code>    covariance_correct(&amp;kf-&gt;p00,&amp;kf-&gt;p01,&amp;kf-&gt;p10,&amp;kf-&gt;p11,</code> | Use Joseph correction with these gains and angle observation variance. |
| 946 | <code>                       kf-&gt;gain_angle,kf-&gt;gain_bias,kf-&gt;accel_noise);</code> | Continuation of the statement at L945: Use Joseph correction with these gains and angle observation variance. |
| 947 | <code>}</code> | End angle update. There is no acceleration-magnitude rejection/gain adaptation here; translational acceleration can corrupt the gravity-angle measurement. |
| 949 | <code>/* 2-state constant-acceleration filter: state = [height, vertical speed]. */</code> | Introduce altitude state [height,vertical_speed]. Acceleration is an external input, not a third estimated state. |
| 950 | <code>static void altitude_kf_predict(altitude_kf_t *kf, float accel_up_mps2,</code> | Predict height and velocity from supplied upward gravity-compensated acceleration. |
| 951 | <code>                                float dt) {</code> | Continuation of the statement at L950: Predict height and velocity from supplied upward gravity-compensated acceleration. |
| 952 | <code>    kf-&gt;height_m += kf-&gt;velocity_mps * dt</code> | Update height with h_minus=h_old+v_old*dt+0.5*a*dt²; old velocity is used before it is modified. |
| 953 | <code>                  + 0.5f * accel_up_mps2 * dt * dt;</code> | Continuation of the statement at L952: Update height with h_minus=h_old+v_old*dt+0.5*a*dt²; old velocity is used before it is modified. |
| 954 | <code>    kf-&gt;velocity_mps += accel_up_mps2 * dt;</code> | Update vertical velocity with v_minus=v_old+a*dt. |
| 956 | <code>    const float p00 = kf-&gt;p00;</code> | Copy old height variance. |
| 957 | <code>    const float p01 = kf-&gt;p01;</code> | Copy old h/v covariance. |
| 958 | <code>    const float p10 = kf-&gt;p10;</code> | Copy old v/h covariance. |
| 959 | <code>    const float p11 = kf-&gt;p11;</code> | Copy old velocity variance. |
| 960 | <code>    const float q = kf-&gt;accel_noise * kf-&gt;accel_noise;</code> | Square acceleration-noise sigma to obtain acceleration variance q. |
| 961 | <code>    kf-&gt;p00 = p00 + dt * (p01 + p10) + dt * dt * p11</code> | Predict height variance P00_plus=P00+dt*(P01+P10)+dt²*P11+0.25*dt^4*q. |
| 962 | <code>            + 0.25f * dt * dt * dt * dt * q;</code> | Continuation of the statement at L961: Predict height variance P00_plus=P00+dt*(P01+P10)+dt²*P11+0.25*dt^4*q. |
| 963 | <code>    kf-&gt;p01 = p01 + dt * p11 + 0.5f * dt * dt * dt * q;</code> | Predict h/v covariance P01_plus=P01+dt*P11+0.5*dt³*q. |
| 964 | <code>    kf-&gt;p10 = p10 + dt * p11 + 0.5f * dt * dt * dt * q;</code> | Predict transpose covariance with the corresponding old entry. |
| 965 | <code>    kf-&gt;p11 = p11 + dt * dt * q;</code> | Predict velocity variance P11_plus=P11+dt²*q. This is a per-step acceleration-error model with Q=G*q*G^T, G=[dt²/2,dt]^T. |
| 966 | <code>}</code> | End altitude prediction. |
| 968 | <code>static void altitude_kf_baro_update(altitude_kf_t *kf, float height_m) {</code> | Correct height/velocity from one independent scalar height measurement. Despite its name, this helper can also fuse a caller-supplied lidar-derived height. |
| 969 | <code>    kf-&gt;innovation_variance=kf-&gt;p00+kf-&gt;baro_noise;</code> | Calculate height innovation variance S=P00_minus+R_height. |
| 970 | <code>    kf-&gt;gain_height=kf-&gt;p00/kf-&gt;innovation_variance;</code> | Height gain K_h=P00_minus/S. |
| 971 | <code>    kf-&gt;gain_velocity=kf-&gt;p10/kf-&gt;innovation_variance;</code> | Velocity gain K_v=P10_minus/S: a height measurement can also correct velocity through cross covariance. |
| 972 | <code>    kf-&gt;innovation=height_m-kf-&gt;height_m;</code> | Innovation nu=measured_height-predicted_height. |
| 973 | <code>    kf-&gt;height_m+=kf-&gt;gain_height*kf-&gt;innovation;</code> | Correct height by K_h*nu. |
| 974 | <code>    kf-&gt;velocity_mps+=kf-&gt;gain_velocity*kf-&gt;innovation;</code> | Correct vertical speed by K_v*nu. |
| 975 | <code>    covariance_correct(&amp;kf-&gt;p00,&amp;kf-&gt;p01,&amp;kf-&gt;p10,&amp;kf-&gt;p11,</code> | Apply Joseph covariance update using height observation variance. |
| 976 | <code>                       kf-&gt;gain_height,kf-&gt;gain_velocity,kf-&gt;baro_noise);</code> | Continuation of the statement at L975: Apply Joseph covariance update using height observation variance. |
| 977 | <code>}</code> | End height correction. Gains vary with covariance; they are not fixed tuning constants like PID gains. |

### L979: Altitude PID and complete flight-control step

| Line | Exact C source | Explanation |
|---:|---|---|
| 979 | <code>/* Exactly one altitude PID: D uses commanded/measured vertical velocity. */</code> | Document that altitude uses a single PID whose D term is desired-minus-measured vertical velocity. |
| 980 | <code>static float altitude_pid_update(float reference, float height, float v_cmd,</code> | Declare the specialized altitude update: reference height, measured height, desired vertical speed, estimated vertical speed and dt. |
| 981 | <code>                                  float velocity, float dt) {</code> | Continuation of the statement at L980: Declare the specialized altitude update: reference height, measured height, desired vertical speed, estimated vertical speed and dt. |
| 982 | <code>    const float error=reference-height;</code> | Calculate height error reference-height. |
| 983 | <code>    const float candidate=clampf(altitude_pid.integrator+error*dt,</code> | Form a bounded candidate integral of height error. |
| 984 | <code>        -altitude_pid.integrator_limit,altitude_pid.integrator_limit);</code> | Continuation of the statement at L983: Form a bounded candidate integral of height error. |
| 985 | <code>    const float raw=altitude_pid.kp*error+altitude_pid.ki*candidate</code> | Compute collective correction Kp*(h_ref-h)+Ki*I+Kd*(v_cmd-v_est). Since h_ref is generated by integrating v_cmd, its derivative is approximately v_cmd when the reference limiter is inactive. |
| 986 | <code>                    +altitude_pid.kd*(v_cmd-velocity);</code> | Continuation of the statement at L985: Compute collective correction Kp*(h_ref-h)+Ki*I+Kd*(v_cmd-v_est). Since h_ref is generated by integrating v_cmd, its derivative is approximately v_cmd when the reference limiter is inactive. |
| 987 | <code>    const float output=clampf(raw,-altitude_pid.output_limit,altitude_pid.output_limit);</code> | Clamp correction to the altitude PID's output magnitude limit. |
| 988 | <code>    if (raw==output &#124;&#124; (raw&gt;output &amp;&amp; error&lt;0.0f) &#124;&#124; (raw&lt;output &amp;&amp; error&gt;0.0f))</code> | Conditionally accept integration only if this PID is not saturating farther in the error direction. Later tilt/collective/mixer saturation is not fed back here. |
| 989 | <code>        altitude_pid.integrator=candidate;</code> | Continuation of the statement at L988: Conditionally accept integration only if this PID is not saturating farther in the error direction. Later tilt/collective/mixer saturation is not fed back here. |
| 990 | <code>    return output;</code> | Return normalized collective correction. |
| 991 | <code>}</code> | End altitude PID. This helper relies on its caller to validate dt. |
| 993 | <code>static bool sample_is_healthy(const sensor_sample_t *s) {</code> | Declare the IMU health predicate used at the start of the control step. |
| 994 | <code>    return s != NULL &amp;&amp; s-&gt;imu_valid &amp;&amp; s-&gt;imu_calibrated</code> | Require a non-null sample and both valid/calibrated IMU flags. |
| 995 | <code>        &amp;&amp; isfinite(s-&gt;accel_x_g) &amp;&amp; isfinite(s-&gt;accel_y_g)</code> | Require every accelerometer and gyro component finite. This does not check sensor age, plausible magnitude or zero-vector acceleration. |
| 996 | <code>        &amp;&amp; isfinite(s-&gt;accel_z_g) &amp;&amp; isfinite(s-&gt;gyro_x_dps)</code> | Continuation of the statement at L995: Require every accelerometer and gyro component finite. This does not check sensor age, plausible magnitude or zero-vector acceleration. |
| 997 | <code>        &amp;&amp; isfinite(s-&gt;gyro_y_dps) &amp;&amp; isfinite(s-&gt;gyro_z_dps);</code> | Continuation of the statement at L995: Require every accelerometer and gyro component finite. This does not check sensor age, plausible magnitude or zero-vector acceleration. |
| 998 | <code>}</code> | End the health predicate. |
| 1000 | <code>/*</code> | Document that the following step is mathematical control only and that motor ordering/signs are provisional until physically validated. |
| 1001 | <code> * This function is currently a math/control core only. Mixer motor order and</code> | Continuation of the statement at L1000: Document that the following step is mathematical control only and that motor ordering/signs are provisional until physically validated. |
| 1002 | <code> * signs must be validated against the chosen frame before its outputs are used.</code> | Continuation of the statement at L1000: Document that the following step is mathematical control only and that motor ordering/signs are provisional until physically validated. |
| 1003 | <code> */</code> | Continuation of the statement at L1000: Document that the following step is mathematical control only and that motor ordering/signs are provisional until physically validated. |
| 1004 | <code>static control_output_t flight_control_step(const sensor_sample_t *s,</code> | Define one control update from sensor sample, command and measured dt. |
| 1005 | <code>                                             const pilot_command_t *cmd,</code> | Continuation of the statement at L1004: Define one control update from sensor sample, command and measured dt. |
| 1006 | <code>                                             float dt) {</code> | Continuation of the statement at L1004: Define one control update from sensor sample, command and measured dt. |
| 1007 | <code>    control_output_t out = { .motor = {0.0f, 0.0f, 0.0f, 0.0f},</code> | Start from four zero motor demands, armed=false, sensor_fault=false. Early returns therefore have zero mathematical demands. |
| 1008 | <code>                             .armed = false, .sensor_fault = false };</code> | Continuation of the statement at L1007: Start from four zero motor demands, armed=false, sensor_fault=false. Early returns therefore have zero mathematical demands. |
| 1009 | <code>    if (!sample_is_healthy(s) &#124;&#124; !isfinite(dt) &#124;&#124; dt &lt;= 0.0f</code> | Reject unhealthy IMU or dt that is nonfinite, nonpositive or above 10 ms. |
| 1010 | <code>        &#124;&#124; dt &gt; FC_MAX_CONTROL_DT) {</code> | Continuation of the statement at L1009: Reject unhealthy IMU or dt that is nonfinite, nonpositive or above 10 ms. |
| 1011 | <code>        reset_control_state();</code> | Clear PID/reference/boost state on rejection; Kalman estimates remain as they were. |
| 1012 | <code>        out.sensor_fault = true;</code> | Set the fault flag. |
| 1013 | <code>        return out;</code> | Return zero demands while remaining disarmed. This scaffold behavior is not a completed airborne failsafe design. |
| 1014 | <code>    }</code> | End early validation. |
| 1016 | <code>    const float accel_roll_deg = atan2f(s-&gt;accel_y_g, s-&gt;accel_z_g)</code> | Derive roll observation atan2(ay,az) in radians, then convert to degrees. This interprets measured specific force as gravity-dominated and uses the source's axis/sign convention. |
| 1017 | <code>                               * FC_RAD_TO_DEG;</code> | Continuation of the statement at L1016: Derive roll observation atan2(ay,az) in radians, then convert to degrees. This interprets measured specific force as gravity-dominated and uses the source's axis/sign convention. |
| 1018 | <code>    const float accel_pitch_deg = atan2f(-s-&gt;accel_x_g,</code> | Derive pitch observation atan2(-ax,sqrt(ay²+az²)), then convert to degrees. It has the usual limited Euler-angle interpretation; this is not quaternion attitude estimation. |
| 1019 | <code>        sqrtf(s-&gt;accel_y_g * s-&gt;accel_y_g + s-&gt;accel_z_g * s-&gt;accel_z_g))</code> | Continuation of the statement at L1018: Derive pitch observation atan2(-ax,sqrt(ay²+az²)), then convert to degrees. It has the usual limited Euler-angle interpretation; this is not quaternion attitude estimation. |
| 1020 | <code>        * FC_RAD_TO_DEG;</code> | Continuation of the statement at L1018: Derive pitch observation atan2(-ax,sqrt(ay²+az²)), then convert to degrees. It has the usual limited Euler-angle interpretation; this is not quaternion attitude estimation. |
| 1021 | <code>    angle_kf_update(&amp;roll_estimator, s-&gt;gyro_x_dps, accel_roll_deg, dt);</code> | Fuse x gyro rate with accelerometer-derived roll observation in the roll angle/bias filter. |
| 1022 | <code>    angle_kf_update(&amp;pitch_estimator, s-&gt;gyro_y_dps, accel_pitch_deg, dt);</code> | Fuse y gyro rate with accelerometer-derived pitch observation in the pitch filter. Body gyro axes only approximate Euler roll/pitch derivatives at modest coupled attitudes. |
| 1024 | <code>    if (s-&gt;barometer_valid &amp;&amp; isfinite(s-&gt;barometer_height_m)</code> | Allow altitude prediction/correction only if barometer-valid and finite height/world-up acceleration are provided. |
| 1025 | <code>        &amp;&amp; isfinite(s-&gt;vertical_acceleration_mps2)) {</code> | Continuation of the statement at L1024: Allow altitude prediction/correction only if barometer-valid and finite height/world-up acceleration are provided. |
| 1026 | <code>        altitude_kf_predict(&amp;altitude_estimator,</code> | Predict height/velocity on every accepted control step from external upward acceleration. |
| 1027 | <code>                            s-&gt;vertical_acceleration_mps2, dt);</code> | Continuation of the statement at L1026: Predict height/velocity on every accepted control step from external upward acceleration. |
| 1028 | <code>        if (s-&gt;barometer_updated)</code> | Correct only when the producer flags a new independent barometer sample. |
| 1029 | <code>            altitude_kf_baro_update(&amp;altitude_estimator, s-&gt;barometer_height_m);</code> | Fuse that new height measurement. A repeated held sample with updated=false only participates in validity, not another correction. |
| 1030 | <code>    }</code> | End altitude-estimation branch. |
| 1032 | <code>    if (cmd == NULL &#124;&#124; !cmd-&gt;valid &#124;&#124; !isfinite(cmd-&gt;roll_deg) &#124;&#124; !isfinite(cmd-&gt;pitch_deg)</code> | Reject null/invalid command or nonfinite roll, pitch, yaw rate, throttle or boost. Estimators above may already have advanced before this command rejection. |
| 1033 | <code>        &#124;&#124; !isfinite(cmd-&gt;yaw_rate_dps) &#124;&#124; !isfinite(cmd-&gt;throttle) &#124;&#124; !isfinite(cmd-&gt;boost)) {</code> | Continuation of the statement at L1032: Reject null/invalid command or nonfinite roll, pitch, yaw rate, throttle or boost. Estimators above may already have advanced before this command rejection. |
| 1034 | <code>        reset_control_state();</code> | Reset PID/reference/boost state. |
| 1035 | <code>        out.sensor_fault = true;</code> | Set fault flag, even though the cause can be command validity rather than a physical sensor failure. |
| 1036 | <code>        return out;</code> | Return zero demands and armed=false. |
| 1037 | <code>    }</code> | End command validation. |
| 1038 | <code>    /* The only implemented mode requires valid height/rate aiding. */</code> | State that the only current mode requires height aiding; there is no alternative manual-thrust mode. |
| 1039 | <code>    if (!s-&gt;barometer_valid &#124;&#124; !isfinite(s-&gt;barometer_height_m)</code> | Reject invalid barometer aiding or nonfinite height/vertical acceleration. |
| 1040 | <code>        &#124;&#124; !isfinite(s-&gt;vertical_acceleration_mps2)) {</code> | Continuation of the statement at L1039: Reject invalid barometer aiding or nonfinite height/vertical acceleration. |
| 1041 | <code>        reset_control_state();</code> | Reset control memory. |
| 1042 | <code>        out.sensor_fault = true;</code> | Set the fault flag. |
| 1043 | <code>        return out;</code> | Return zero mathematical demands. Sensor-loss landing/fallback authority remains future integration work. |
| 1044 | <code>    }</code> | End height-aiding requirement. |
| 1046 | <code>    /* Bound pilot inputs before they enter the control math. */</code> | Introduce command bounds, separate from packet validation, because host/companion callers can bypass radio decoding. |
| 1047 | <code>    const float roll_setpoint = clampf(cmd-&gt;roll_deg, -35.0f, 35.0f);</code> | Clamp requested roll angle to +/-35 degrees. |
| 1048 | <code>    const float pitch_setpoint = clampf(cmd-&gt;pitch_deg, -35.0f, 35.0f);</code> | Clamp pitch angle to +/-35 degrees. |
| 1049 | <code>    const float yaw_rate_setpoint = clampf(cmd-&gt;yaw_rate_dps, -180.0f, 180.0f);</code> | Clamp requested yaw rate to +/-180 degrees/s. |
| 1050 | <code>    const float throttle = clampf(cmd-&gt;throttle, 0.0f, 1.0f);</code> | Clamp throttle to 0..1. |
| 1051 | <code>    const float boost_target=clampf(cmd-&gt;boost,0,1);</code> | Clamp RT boost target to 0..1. |
| 1052 | <code>    boost_state+=clampf(boost_target-boost_state,-10*dt,4*dt);</code> | Slew boost: increase by at most 4*dt, decrease by at most 10*dt. A full rise takes at least 0.25 s and full fall at least 0.10 s. At dt=.002, per-step changes are +.008 or -.020. |
| 1053 | <code>    const float boost=boost_state;</code> | Use the smoothed boost value for this iteration. |
| 1054 | <code>    const float roll_rate_setpoint = pid_update(&amp;roll_angle_pid, roll_setpoint,</code> | Outer roll PID maps angle error to desired roll rate using filtered roll angle as measurement. |
| 1055 | <code>                                                 roll_estimator.angle_deg, dt);</code> | Continuation of the statement at L1054: Outer roll PID maps angle error to desired roll rate using filtered roll angle as measurement. |
| 1056 | <code>    const float pitch_rate_setpoint = pid_update(&amp;pitch_angle_pid, pitch_setpoint,</code> | Outer pitch PID maps angle error to desired pitch rate. |
| 1057 | <code>                                                  pitch_estimator.angle_deg, dt);</code> | Continuation of the statement at L1056: Outer pitch PID maps angle error to desired pitch rate. |
| 1058 | <code>    const float roll = pid_update(&amp;roll_rate_pid, roll_rate_setpoint,</code> | Inner roll-rate PI maps desired-minus-measured x gyro rate to normalized roll mixer correction. It uses calibrated gyro_x directly, not gyro_x minus the Kalman filter's additionally estimated residual bias. |
| 1059 | <code>                                  s-&gt;gyro_x_dps, dt);</code> | Continuation of the statement at L1058: Inner roll-rate PI maps desired-minus-measured x gyro rate to normalized roll mixer correction. It uses calibrated gyro_x directly, not gyro_x minus the Kalman filter's additionally estimated residual bias. |
| 1060 | <code>    const float pitch = pid_update(&amp;pitch_rate_pid, pitch_rate_setpoint,</code> | Inner pitch-rate PI similarly uses calibrated gyro_y directly. |
| 1061 | <code>                                   s-&gt;gyro_y_dps, dt);</code> | Continuation of the statement at L1060: Inner pitch-rate PI similarly uses calibrated gyro_y directly. |
| 1062 | <code>    const float yaw = pid_update(&amp;yaw_rate_pid, yaw_rate_setpoint,</code> | Yaw-rate PI compares commanded rate with calibrated gyro_z; no magnetometer or absolute-heading state participates. |
| 1063 | <code>                                 s-&gt;gyro_z_dps, dt);</code> | Continuation of the statement at L1062: Yaw-rate PI compares commanded rate with calibrated gyro_z; no magnetometer or absolute-heading state participates. |
| 1065 | <code>    if (!altitude_reference_initialized) {</code> | If this is the first valid control step after startup/reset, initialize height hold. |
| 1066 | <code>        altitude_hold_reference_m = altitude_estimator.height_m;</code> | Capture current estimated height as the initial target, preventing an automatic jump to zero metres. |
| 1067 | <code>        pid_reset(&amp;altitude_pid);</code> | Clear altitude PID memory at target capture. |
| 1068 | <code>        altitude_reference_initialized = true;</code> | Mark height reference initialized. |
| 1069 | <code>    }</code> | End target initialization. |
| 1070 | <code>    const float pilot_vertical_speed = (throttle - 0.5f)</code> | Map throttle to upward velocity: v_cmd=(throttle-.5)*2*(2+boost). With boost=0, endpoints give -2,+2 m/s; boost=1 gives -3,+3 m/s. Center remains zero, so boost alone does not force a climb. |
| 1071 | <code>                                     * 2.0f * (FC_MAX_VERTICAL_SPEED_MPS+boost);</code> | Continuation of the statement at L1070: Map throttle to upward velocity: v_cmd=(throttle-.5)*2*(2+boost). With boost=0, endpoints give -2,+2 m/s; boost=1 gives -3,+3 m/s. Center remains zero, so boost alone does not force a climb. |
| 1072 | <code>    altitude_hold_reference_m += pilot_vertical_speed * dt;</code> | Integrate desired velocity to move the height target: h_ref += v_cmd*dt. With centered throttle, the target remains fixed. |
| 1073 | <code>    altitude_hold_reference_m=clampf(altitude_hold_reference_m,</code> | Keep reference within +/-2 m of current estimated height to bound target run-away. While limited, derivative of the stored reference is not necessarily exactly v_cmd. |
| 1074 | <code>        altitude_estimator.height_m-2.0f,altitude_estimator.height_m+2.0f);</code> | Continuation of the statement at L1073: Keep reference within +/-2 m of current estimated height to bound target run-away. While limited, derivative of the stored reference is not necessarily exactly v_cmd. |
| 1075 | <code>    const float collective_correction=altitude_pid_update(altitude_hold_reference_m,</code> | Run the single altitude PID using height error plus desired/estimated vertical velocity; obtain a normalized collective correction. |
| 1076 | <code>        altitude_estimator.height_m,pilot_vertical_speed,altitude_estimator.velocity_mps,dt);</code> | Continuation of the statement at L1075: Run the single altitude PID using height error plus desired/estimated vertical velocity; obtain a normalized collective correction. |
| 1077 | <code>    const float tilt_factor=fmaxf(0.5f,cosf(roll_estimator.angle_deg*FC_DEG_TO_RAD)</code> | Estimate vertical thrust projection cos(roll)*cos(pitch), with a floor of .5 to bound tilt compensation to at most a factor of two. It is an approximate geometric correction. |
| 1078 | <code>                                     *cosf(pitch_estimator.angle_deg*FC_DEG_TO_RAD));</code> | Continuation of the statement at L1077: Estimate vertical thrust projection cos(roll)*cos(pitch), with a floor of .5 to bound tilt compensation to at most a factor of two. It is an approximate geometric correction. |
| 1079 | <code>    const float collective=clampf((hover_collective+collective_correction)/tilt_factor,0.0f,.65f+.15f*boost);</code> | Add hover feed-forward and altitude correction, divide by tilt projection, then clamp pre-mixer collective to 0..(.65+.15*boost): .65 normally, .80 at full boost. This reserves nominal room for attitude corrections but is not a universal post-mixer headroom guarantee. |
| 1081 | <code>    /* Provisional X-frame math in logical order FL, FR, RR, RL. The result is</code> | Document the provisional X-frame order FL,FR,RR,RL and the lack of a connected ESC driver. |
| 1082 | <code>       for simulation only until motor order, spin directions, and frame axes</code> | Continuation of the statement at L1081: Document the provisional X-frame order FL,FR,RR,RL and the lack of a connected ESC driver. |
| 1083 | <code>       are verified. No PWM/DShot/ESC driver consumes these values. */</code> | Continuation of the statement at L1081: Document the provisional X-frame order FL,FR,RR,RL and the lack of a connected ESC driver. |
| 1084 | <code>    float correction[4]={roll-pitch-yaw,-roll-pitch+yaw,-roll+pitch-yaw,roll+pitch+yaw};</code> | Create four differential corrections: FL=roll-pitch-yaw; FR=-roll-pitch+yaw; RR=-roll+pitch-yaw; RL=roll+pitch+yaw. Each axis's four signs sum to zero before limits. |
| 1085 | <code>    float low=correction[0],high=correction[0];</code> | Start the minimum and maximum correction at the first motor's value. |
| 1086 | <code>    for(unsigned i=1;i&lt;4;i++){low=fminf(low,correction[i]);high=fmaxf(high,correction[i]);}</code> | Scan the other three motors to get the full differential-correction range. |
| 1087 | <code>    // Uniformly scale only infeasible torque combinations, then shift collective</code> | Explain allocation strategy: preserve correction ratios through uniform scaling, then move collective into a feasible interval; separate per-motor clipping would change requested torque ratios. |
| 1088 | <code>    // into the feasible interval. Independent clipping would distort torques.</code> | Continuation of the statement at L1087: Explain allocation strategy: preserve correction ratios through uniform scaling, then move collective into a feasible interval; separate per-motor clipping would change requested torque ratios. |
| 1089 | <code>    const float scale=high-low&gt;1 ? 1/(high-low) : 1;</code> | If maxCorrection-minCorrection exceeds normalized motor range 1, scale all corrections by its reciprocal; otherwise scale=1. |
| 1090 | <code>    const float allocated=clampf(collective,-low*scale,1-high*scale);</code> | Choose common allocated collective in [-low*scale, 1-high*scale], ensuring every shifted correction fits 0..1. This common shift may change the pre-mixer collective, including pushing it above its earlier cap; the cap is not a hard final total-demand/current limit. |
| 1091 | <code>    for(unsigned i=0;i&lt;4;i++)out.motor[i]=clampf(allocated+scale*correction[i],0,1);</code> | Add each scaled correction to the common collective and clamp to 0..1 for roundoff protection. Final motor demands are ordered FL,FR,RR,RL. |
| 1093 | <code>    /* There is deliberately no hardware arming/output path in this scaffold. */</code> | Document that no hardware arm/output path follows the mathematical mixer. |
| 1094 | <code>    out.armed = false;</code> | Explicitly force armed=false even when mathematical motor demands are nonzero. |
| 1095 | <code>    return out;</code> | Return the calculated result. |
| 1096 | <code>}</code> | End the control step. |

### L1098: Host simulation ABI and diagnostics

| Line | Exact C source | Explanation |
|---:|---|---|
| 1098 | <code>#ifdef QCOPTER_HOST_TEST</code> | Compile the following simulation ABI only when QCOPTER_HOST_TEST is defined. |
| 1099 | <code>/* Host-only ABI: same C control equations, no physical ESC/arming path. */</code> | Document host-only C entry points: same control equations, no physical arming/ESC path. |
| 1100 | <code>#ifdef QCOPTER_SITL_DLL</code> | If additionally building the Windows SITL DLL, export the following symbols. |
| 1101 | <code>#define QC_SIM_EXPORT __declspec(dllexport)</code> | Map QC_SIM_EXPORT to Windows __declspec(dllexport), making functions visible for ctypes/library loading. |
| 1102 | <code>#else</code> | Otherwise select the non-DLL host path. |
| 1103 | <code>#define QC_SIM_EXPORT</code> | Define an empty export macro for that path. |
| 1104 | <code>#endif</code> | End export-macro selection. |
| 1105 | <code>QC_SIM_EXPORT void qc_sitl_reset(float hover, float height, const float *gains) {</code> | Export simulation reset: caller supplies hover feed-forward, initial height and optionally 18 gain values. Pointers/array lengths are a trusted ABI contract, not fully checked here. |
| 1106 | <code>    boost_state=0;</code> | Clear boost state. |
| 1107 | <code>    roll_estimator=(angle_kf_t){.p00=1.0f,.p11=0.01f,</code> | Reset roll estimator angle/bias/covariance/noise using a compound literal; omitted members become zero. |
| 1108 | <code>        .gyro_noise=0.04f,.bias_walk=0.0001f,.accel_noise=4.0f};</code> | Continuation of the statement at L1107: Reset roll estimator angle/bias/covariance/noise using a compound literal; omitted members become zero. |
| 1109 | <code>    pitch_estimator=roll_estimator;</code> | Copy that initial angle-filter state to pitch. |
| 1110 | <code>    altitude_estimator=(altitude_kf_t){.height_m=height,.p00=0.04f,.p11=0.1f,</code> | Reset altitude estimate to caller-supplied height, P00=.04 m², P11=.1 (m/s)², acceleration sigma=1.5 and barometer variance=.0025 m². That height R corresponds to .05 m measurement sigma, unlike the firmware-global default R=2. |
| 1111 | <code>        .accel_noise=1.5f,.baro_noise=0.0025f};</code> | Continuation of the statement at L1110: Reset altitude estimate to caller-supplied height, P00=.04 m², P11=.1 (m/s)², acceleration sigma=1.5 and barometer variance=.0025 m². That height R corresponds to .05 m measurement sigma, unlike the firmware-global default R=2. |
| 1112 | <code>    pid_t *loops[6]={&amp;roll_angle_pid,&amp;roll_rate_pid,&amp;pitch_angle_pid,</code> | Build the six-PID pointer order expected by the gain array: roll angle, roll rate, pitch angle, pitch rate, yaw rate, altitude. |
| 1113 | <code>                     &amp;pitch_rate_pid,&amp;yaw_rate_pid,&amp;altitude_pid};</code> | Continuation of the statement at L1112: Build the six-PID pointer order expected by the gain array: roll angle, roll rate, pitch angle, pitch rate, yaw rate, altitude. |
| 1114 | <code>    for(unsigned i=0;i&lt;6u;++i) {</code> | Visit all six PID instances. |
| 1115 | <code>        if(gains!=NULL) {</code> | If a gains pointer was supplied, replace gains; otherwise retain current values, not necessarily the original source defaults after an earlier reset call. |
| 1116 | <code>            loops[i]-&gt;kp=gains[3u*i]; loops[i]-&gt;ki=gains[3u*i+1u];</code> | For loop i, read Kp at gains[3*i] and Ki at gains[3*i+1]. |
| 1117 | <code>            loops[i]-&gt;kd=gains[3u*i+2u];</code> | Read Kd at gains[3*i+2]. Gain finiteness and array length are not validated in this host helper. |
| 1118 | <code>        }</code> | End optional gain replacement. |
| 1119 | <code>        pid_reset(loops[i]);</code> | Reset the selected PID's integral/measurement history. |
| 1120 | <code>    }</code> | End loop over PID instances. |
| 1121 | <code>    hover_collective=clampf(hover,0.05f,0.85f);</code> | Clamp supplied normalized hover feed-forward to .05.. .85. The host caller must supply a finite value. |
| 1122 | <code>    altitude_reference_initialized=false;</code> | Require height-reference capture on next valid control step. |
| 1123 | <code>}</code> | End host reset. |
| 1125 | <code>QC_SIM_EXPORT void qc_sitl_step(const float *input, const float *command,</code> | Export one host control step. Input/command arrays enter; four motors and sixteen diagnostics leave through caller-allocated arrays. No I2C, SPI, radio or ESC transport occurs inside it. |
| 1126 | <code>                                float dt, unsigned flags, float *motors,</code> | Continuation of the statement at L1125: Export one host control step. Input/command arrays enter; four motors and sixteen diagnostics leave through caller-allocated arrays. No I2C, SPI, radio or ESC transport occurs inside it. |
| 1127 | <code>                                float *diagnostics) {</code> | Continuation of the statement at L1125: Export one host control step. Input/command arrays enter; four motors and sixteen diagnostics leave through caller-allocated arrays. No I2C, SPI, radio or ESC transport occurs inside it. |
| 1128 | <code>    const sensor_sample_t s={</code> | Create the internal sensor sample from the host's flat arrays and flags. |
| 1129 | <code>        .accel_x_g=input[0],.accel_y_g=input[1],.accel_z_g=input[2],</code> | input[0..2] supply accelerometer x,y,z in g. |
| 1130 | <code>        .gyro_x_dps=input[3],.gyro_y_dps=input[4],.gyro_z_dps=input[5],</code> | input[3..5] supply gyro x,y,z in degrees/s. |
| 1131 | <code>        .vertical_acceleration_mps2=input[6],.barometer_height_m=input[7],</code> | input[6] supplies upward gravity-compensated acceleration and input[7] supplies barometer height. |
| 1132 | <code>        .barometer_updated=input[8]&gt;0.5f,.imu_valid=(flags&amp;1u)!=0u,</code> | input[8]>.5 flags a new barometer sample; flags bit 0 supplies IMU validity. |
| 1133 | <code>        .imu_calibrated=(flags&amp;1u)!=0u,.barometer_valid=(flags&amp;2u)!=0u};</code> | The same flags bit 0 also marks host IMU data already calibrated; flags bit 1 supplies barometer validity. The simulation assumes its producer did the unit/frame/calibration work. |
| 1134 | <code>    const pilot_command_t c={.roll_deg=command[0],.pitch_deg=command[1],</code> | command[0] is requested roll angle and command[1] requested pitch angle. |
| 1135 | <code>        .yaw_rate_dps=command[2],.throttle=command[3],.boost=(flags&amp;8u)!=0u?command[4]:0,.valid=(flags&amp;4u)!=0u};</code> | command[2] is yaw rate, command[3] centered throttle; command[4] boost is read only when flags bit 3 is set. Flags bit 2 controls command validity. |
| 1136 | <code>    const control_output_t out=flight_control_step(&amp;s,&amp;c,dt);</code> | Call the same flight_control_step used by firmware. |
| 1137 | <code>    for(unsigned i=0;i&lt;4u;++i) motors[i]=out.motor[i];</code> | Copy four normalized motor demands to the host's motor array. |
| 1138 | <code>    const float d[16]={roll_estimator.angle_deg,pitch_estimator.angle_deg,</code> | Begin sixteen diagnostic values: indices 0 and 1 are estimated roll and pitch angles. |
| 1139 | <code>        altitude_estimator.height_m,altitude_estimator.velocity_mps,</code> | Diagnostic 2 is estimated height; 3 is estimated vertical velocity. |
| 1140 | <code>        roll_estimator.gain_angle,roll_estimator.gain_bias,</code> | Diagnostics 4 and 5 are roll angle and residual-bias Kalman gains. |
| 1141 | <code>        altitude_estimator.gain_height,altitude_estimator.gain_velocity,</code> | Diagnostics 6 and 7 are altitude height and velocity gains. |
| 1142 | <code>        sqrtf(fmaxf(0.0f,roll_estimator.p00)),</code> | Diagnostic 8 is roll-angle sigma = sqrt(max(P00_roll,0)). |
| 1143 | <code>        sqrtf(fmaxf(0.0f,altitude_estimator.p00)),</code> | Diagnostic 9 is height sigma = sqrt(max(P00_height,0)). |
| 1144 | <code>        roll_estimator.innovation,altitude_estimator.innovation,</code> | Diagnostics 10 and 11 are roll and height innovations. |
| 1145 | <code>        altitude_hold_reference_m,(float)out.armed,(float)out.sensor_fault,</code> | Diagnostic 12 is stored height target; 13 is armed (always 0); 14 is the fault flag converted to float. |
| 1146 | <code>        roll_estimator.innovation_variance};</code> | Diagnostic 15 is roll innovation variance S; complete pitch covariance, yaw uncertainty and battery state are not exposed in this array. |
| 1147 | <code>    for(unsigned i=0;i&lt;16u;++i) diagnostics[i]=d[i];</code> | Copy all sixteen diagnostic values into the host output buffer. |
| 1148 | <code>}</code> | End the exported simulation control step. |
| 1149 | <code>QC_SIM_EXPORT void qc_sitl_height_observation(float height,float variance) {</code> | Export an extra scalar height-observation update, used by the simulation to provide lidar-derived height with caller-specified variance. |
| 1150 | <code>    if(isfinite(height)&amp;&amp;isfinite(variance)&amp;&amp;variance&gt;0) {</code> | Only accept finite height and finite strictly positive variance. Range, timestamp, alignment and outlier checks are not implemented by this helper. |
| 1151 | <code>        const float previous=altitude_estimator.baro_noise;</code> | Save the normal barometer observation variance. |
| 1152 | <code>        altitude_estimator.baro_noise=variance;</code> | Temporarily use the caller's observation variance for this measurement. |
| 1153 | <code>        altitude_kf_baro_update(&amp;altitude_estimator,height);</code> | Fuse this height with the shared height/velocity Kalman correction. |
| 1154 | <code>        altitude_estimator.baro_noise=previous;</code> | Restore the original barometer variance afterward, so lidar does not permanently replace its noise setting. |
| 1155 | <code>    }</code> | End accepted observation branch. |
| 1156 | <code>}</code> | End host-only extra-height helper. |
| 1157 | <code>#endif</code> | End all QCOPTER_HOST_TEST-only entry points. |

### L1159: Pico core 1, receive queue and core-0 main loop

| Line | Exact C source | Explanation |
|---:|---|---|
| 1159 | <code>#ifndef QCOPTER_HOST_TEST</code> | Compile the following radio/core-1 scaffold only for Pico firmware. |
| 1160 | <code>typedef struct {</code> | Begin the object passed through the inter-core receive queue. |
| 1161 | <code>    uint8_t bytes[QCOPTER_FRAME_SIZE];</code> | Store a complete 22-byte packet buffer. |
| 1162 | <code>    size_t length;</code> | Store actual received length as size_t. |
| 1163 | <code>    uint64_t received_us;</code> | Store local reception time in microseconds. |
| 1164 | <code>} radio_rx_item_t;</code> | Name the queued receive-item type. |
| 1166 | <code>static queue_t radio_rx_queue;</code> | Allocate one persistent Pico queue object; its storage is initialized in main. |
| 1168 | <code>/* Nonblocking placeholder: connect this to the selected LoRa module driver. */</code> | Explicitly label radio_receive_frame as a placeholder, not an operational LoRa driver. |
| 1169 | <code>static bool radio_receive_frame(uint8_t *bytes, size_t capacity, size_t *length) {</code> | Declare a nonblocking radio read interface: destination, capacity and returned length. |
| 1170 | <code>    (void)bytes;</code> | Cast the unused destination parameter to void to suppress compiler warnings. |
| 1171 | <code>    (void)capacity;</code> | Likewise suppress the unused capacity warning. |
| 1172 | <code>    (void)length;</code> | Likewise suppress the unused length-pointer warning; no length is written. |
| 1173 | <code>    return false;</code> | Always report no packet received. No LoRa or CC1101 peripheral is initialized or used. |
| 1174 | <code>}</code> | End the radio stub. |
| 1176 | <code>static void slow_core(void) {</code> | Define the function that core 1 will execute. |
| 1177 | <code>    for (;;) {</code> | Run core 1 forever. |
| 1178 | <code>        /*</code> | Document intended ownership of slow/nonblocking radio and sensor work. Only the radio placeholder/polling loop exists; slower sensor acquisition is not implemented. |
| 1179 | <code>         * Core 1 owns nonblocking radio and slower sensor acquisition.</code> | Continuation of the statement at L1178: Document intended ownership of slow/nonblocking radio and sensor work. Only the radio placeholder/polling loop exists; slower sensor acquisition is not implemented. |
| 1180 | <code>         * The radio stub returns no data until a module driver is selected.</code> | Continuation of the statement at L1178: Document intended ownership of slow/nonblocking radio and sensor work. Only the radio placeholder/polling loop exists; slower sensor acquisition is not implemented. |
| 1181 | <code>         */</code> | Continuation of the statement at L1178: Document intended ownership of slow/nonblocking radio and sensor work. Only the radio placeholder/polling loop exists; slower sensor acquisition is not implemented. |
| 1182 | <code>        radio_rx_item_t item = {0};</code> | Create a fresh zero-initialized queue item on the core-1 stack. |
| 1183 | <code>        if (radio_receive_frame(item.bytes, sizeof(item.bytes), &amp;item.length)</code> | If a future radio driver returns data and its reported length fits the buffer, process that received item. |
| 1184 | <code>            &amp;&amp; item.length &lt;= sizeof(item.bytes)) {</code> | Continuation of the statement at L1183: If a future radio driver returns data and its reported length fits the buffer, process that received item. |
| 1185 | <code>            item.received_us = time_us_64();</code> | Timestamp accepted received data using the local 64-bit microsecond clock. |
| 1186 | <code>            (void)queue_try_add(&amp;radio_rx_queue, &amp;item);</code> | Try to enqueue without blocking; ignore the Boolean result. If the four-item queue is full, this item is dropped with no drop counter. |
| 1187 | <code>        }</code> | End received-item branch. |
| 1188 | <code>        sleep_ms(1u);</code> | Sleep roughly 1 ms before polling again. This is not a precisely scheduled 1000 Hz sensor task. |
| 1189 | <code>    }</code> | Close the endless polling loop. |
| 1190 | <code>}</code> | End the core-1 function. |
| 1191 | <code>#endif</code> | End Pico-only radio/core-1 definitions. |
| 1193 | <code>#ifndef QCOPTER_HOST_TEST</code> | Compile main only for Pico firmware, excluding it from the host DLL. |
| 1194 | <code>int main(void) {</code> | Define firmware entry point. |
| 1195 | <code>    stdio_init_all();</code> | Initialize Pico's configured stdio interfaces; actual USB/UART enablement comes from the build configuration. |
| 1196 | <code>    queue_init(&amp;radio_rx_queue, sizeof(radio_rx_item_t), 4u);</code> | Initialize a queue with capacity for four radio_rx_item_t objects. Queue use is the explicit communication channel between cores here. |
| 1197 | <code>    multicore_launch_core1(slow_core);</code> | Launch slow_core on core 1; the current core continues as core 0. |
| 1198 | <code>    sleep_ms(1500);</code> | Wait 1500 ms before startup messages/loop setup, commonly useful for a console connection; this does not calibrate sensors. |
| 1199 | <code>    puts("QCopter RP2350 FC: 500 Hz loop, radio/sensors/ESC outputs not connected.");</code> | Print that 500 Hz is the target and hardware radio/sensors/ESC outputs are not connected. |
| 1200 | <code>    puts("Core 1 radio receive is a stub; outputs stay disarmed.");</code> | Print the radio-stub and disarmed status. |
| 1202 | <code>    pilot_link_state_t pilot_link = {0};</code> | Start with no accepted pilot-link state and invalid latest command. |
| 1203 | <code>    uint64_t previous_us = time_us_64();</code> | Read the timestamp used to calculate the first measured control interval. |
| 1204 | <code>    absolute_time_t next_cycle = delayed_by_us(get_absolute_time(),</code> | Schedule the first control deadline 2000 us after current absolute time. |
| 1205 | <code>                                                FC_LOOP_PERIOD_US);</code> | Continuation of the statement at L1204: Schedule the first control deadline 2000 us after current absolute time. |
| 1206 | <code>    uint32_t loop_count = 0u;</code> | Initialize iteration count for periodic status reporting. |
| 1207 | <code>    uint32_t missed_deadlines = 0u;</code> | Initialize late-deadline count for the next reporting window. |
| 1208 | <code>    for (;;) {</code> | Enter the endless core-0 control loop. |
| 1209 | <code>        sleep_until(next_cycle);</code> | Sleep until the scheduled absolute deadline; if already late, sleep_until does not restore time already lost. |
| 1210 | <code>        next_cycle = delayed_by_us(next_cycle, FC_LOOP_PERIOD_US);</code> | Advance the planned deadline by exactly one 2000 us period, rather than scheduling from current completion time. This limits accumulating phase drift but has no explicit missed-period skip/resynchronization policy. |
| 1211 | <code>        const uint64_t now_us = time_us_64();</code> | Read actual wake-up time in microseconds. |
| 1212 | <code>        const uint64_t scheduled_us = to_us_since_boot(next_cycle)</code> | Recover the deadline for the iteration just started by subtracting one period from the newly advanced deadline. |
| 1213 | <code>                                    - FC_LOOP_PERIOD_US;</code> | Continuation of the statement at L1212: Recover the deadline for the iteration just started by subtracting one period from the newly advanced deadline. |
| 1214 | <code>        if (now_us &gt; scheduled_us + 100u) missed_deadlines++;</code> | Count the iteration late if wake-up is more than 100 us past that deadline. This measures start lateness, not complete execution time or a certified hard real-time guarantee. |
| 1215 | <code>        const float dt = (float)(now_us - previous_us) * 1e-6f;</code> | Calculate actual dt=(now_us-previous_us)*1e-6 seconds, rather than assuming exactly .002. |
| 1216 | <code>        previous_us = now_us;</code> | Store this wake-up timestamp for next interval calculation. |
| 1218 | <code>        radio_rx_item_t item;</code> | Prepare a receive item for draining core 1's queue. |
| 1219 | <code>        while (queue_try_remove(&amp;radio_rx_queue, &amp;item))</code> | Remove queued items until the queue is empty, without blocking. |
| 1220 | <code>            (void)pilot_link_accept(&amp;pilot_link, item.bytes, item.length,</code> | Attempt packet validation/sequence acceptance for each item with its original receive timestamp; ignore the Boolean return. Accepted later queue entries replace earlier ones. |
| 1221 | <code>                                    item.received_us);</code> | Continuation of the statement at L1220: Attempt packet validation/sequence acceptance for each item with its original receive timestamp; ignore the Boolean return. Accepted later queue entries replace earlier ones. |
| 1223 | <code>        /* Sensor acquisition is not implemented: safe zero outputs by design. */</code> | Document that no sensor acquisition exists and therefore the following input intentionally fails sensor-health checks. |
| 1224 | <code>        const sensor_sample_t no_sensors = {0};</code> | Construct an all-zero sample, including imu_valid=false, imu_calibrated=false and barometer_valid=false. |
| 1225 | <code>        const pilot_command_t command = pilot_link_current(&amp;pilot_link, now_us);</code> | Retrieve the last pilot command only if the link timestamp is fresh. |
| 1226 | <code>        const control_output_t status = flight_control_step(&amp;no_sensors,</code> | Call flight_control_step with the invalid no_sensors sample and measured dt. It therefore follows the early fault/zero-demand path on every firmware iteration. |
| 1227 | <code>                                                             &amp;command, dt);</code> | Continuation of the statement at L1226: Call flight_control_step with the invalid no_sensors sample and measured dt. It therefore follows the early fault/zero-demand path on every firmware iteration. |
| 1228 | <code>        (void)status;</code> | Discard the returned status to suppress an unused-variable warning. No function writes PWM/DShot/ESC output. |
| 1229 | <code>        if (++loop_count == FC_LOOP_HZ) {</code> | Increment iteration count and report after 500 iterations. This is about once per second at the target rate, not necessarily exactly once per wall-clock second. |
| 1230 | <code>            printf("fc_loop=500Hz target; missed=%lu; command=%s; motors=DISABLED\n",</code> | Print the target-frequency label, late-start count and command freshness; the string does not prove measured loop frequency. |
| 1231 | <code>                   (unsigned long)missed_deadlines,</code> | Cast the late-start count for printf's unsigned-long format. |
| 1232 | <code>                   command.valid ? "fresh" : "failsafe");</code> | Print fresh if the command is valid; otherwise failsafe. This is a status word, not proof of an implemented airborne failsafe maneuver. |
| 1233 | <code>            loop_count = 0u;</code> | Reset iteration counter for the next reporting window. |
| 1234 | <code>            missed_deadlines = 0u;</code> | Reset late-start counter for that window. |
| 1235 | <code>        }</code> | Close periodic printing branch. Printing can itself consume time; no worst-case logging budget is enforced here. |
| 1236 | <code>    }</code> | Close the endless control loop. |
| 1237 | <code>}</code> | End firmware main; the endless loop normally never returns. |
| 1238 | <code>#endif</code> | End the firmware-only conditional compilation block. |
