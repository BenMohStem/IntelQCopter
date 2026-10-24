# Understand and configure the MPU6050

This is our learning guide and a route through the manufacturer documents, not a replacement datasheet. Start with sections 1–5; change one setting, read it back, and observe the effect before changing another. The code remains a bench sensor programme, not a 500 Hz flight controller.

## 1. Which document answers which question?

The **product specification** explains the chip's electrical/mechanical limits, sensor performance, interfaces, axes and application circuit. The **register map** explains the software controls and data layout. Read both: a writable field is not a guarantee that every value is appropriate for your circuit or application.

- [Manufacturer product specification, PS-MPU-6000A-00](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).
- [Manufacturer register map, RM-MPU-6000A-00, revision 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
- [Alternate manufacturer register-map URL](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).
- [Adafruit's actual setter/getter implementations](https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp).
- [Adafruit parameter enum names](https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.h).

TDK's older PDF URLs can redirect to document search. Search their document numbers above if a link redirects. The DLPF table was checked against the manufacturer's indexed revision-4.2 register map; APIs/defaults were also inspected in the installed Adafruit 2.2.9 source. Unexposed features below are reading topics, not newly implemented drivers.

## 2. Follow one measurement through the system

```text
physical acceleration/angular motion
    -> sensor and internal sampling/filtering
    -> output registers and optional FIFO
    -> I2C read
    -> Adafruit conversion to physical units
    -> our mounting/scale correction
    -> printed result (later: estimator and controller)
```

The MPU accelerometer offset words now apply our bias trim before the software receives its event. The matrix then maps those sensor coordinates into drone coordinates. Neither stage removes gravity. A stationary accelerometer should retain a vector of magnitude approximately g; a stationary gyro should approach zero after its separate bias calibration.

“Sensor sample rate”, “I2C bus speed”, “how often we read”, “how often we print” and “PID loop rate” are five different rates. Raising one does not automatically raise the others.

## 3. Your editable block in main.cpp

```cpp
static const uint8_t mpu_address = 0x68;
static const int sda_pin = 9;
static const int scl_pin = 8;
static const uint32_t i2c_clock_hz = 100000;
static const mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G;
static const mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_500_DEG;
static const mpu6050_bandwidth_t filter_bandwidth = MPU6050_BAND_260_HZ;
static const uint8_t sample_divider = 0;
static const mpu6050_clock_select_t clock_source = MPU6050_PLL_GYROX;
```

These retain the previous bench configuration rather than silently selecting flight tuning. Edit these named values, upload yourself, and compare the MPU_CONFIG readback status and measured output. I2C address follows actual AD0 wiring; changing the code alone does not change the device address. Pin numbers describe ESP32 GPIOs, not MPU package-pin numbers or header-hole positions.

The Arduino enum names are readable labels for bit-field values. For example an acceleration-range code of 0 selects the 2g setting; logging `accel=0` does not mean the sensor has zero range. The code explicitly calls setters AFTER Adafruit begin, because begin installs its own defaults. A later reset/begin would reinstall defaults and require configuration/calibration again.

## 4. Range, sensitivity and units

Range is the largest representable input before clipping. Sensitivity describes counts per physical unit. For a signed 16-bit output, increasing range spreads the same number of codes over more physical input:

$$
a_g=\frac{n_a}{S_a},\qquad a_{\mathrm{m/s^2}}=g\frac{n_a}{S_a}.
$$

$$
\omega_{\mathrm{deg/s}}=\frac{n_g}{S_g},\qquad
\omega_{\mathrm{rad/s}}=\frac{\pi}{180}\frac{n_g}{S_g}.
$$

Here n is the signed output count, and S is the selected sensitivity. Adafruit's event API performs that conversion; do not divide its already-converted values by a raw-count sensitivity again.

| Acceleration setting | Counts/g | Gyro setting | Counts/(degree/s) |
|---|---:|---|---:|
| `MPU6050_RANGE_2_G` | 16384 | `MPU6050_RANGE_250_DEG` | 131 |
| `MPU6050_RANGE_4_G` | 8192 | `MPU6050_RANGE_500_DEG` | 65.5 |
| `MPU6050_RANGE_8_G` | 4096 | `MPU6050_RANGE_1000_DEG` | 32.8 |
| `MPU6050_RANGE_16_G` | 2048 | `MPU6050_RANGE_2000_DEG` | 16.4 |

These nominal conversion values are also encoded in Adafruit's implementation. They are not calibration uncertainty or effective noise-free resolution.

Example: with S_a=16384, a count of 8192 corresponds to 0.5g, about 4.9033 m/s². Changing range while keeping a hard-coded raw conversion would give a wrong answer. Our current MPU bias conversion concerns offset-register units, not the selectable output-register sensitivity; the two must not be confused. After a range change, independently repeat the pose checks because range-dependent gain errors or saturation may invalidate the previous fit.

More range gives more headroom for motion/vibration but a coarser nominal step per count. Choose it from observed clipping and required motion, not from the belief that the largest number is automatically most accurate. We have not yet characterized flight vibration or maximum angular speed.

## 5. Filter bandwidth, sample rate and latency

The manufacturer's CONFIG table couples accelerometer and gyro filtering. Adafruit's enum label follows the acceleration bandwidth; the gyro bandwidth is slightly different. In DLPF mode 0 the acceleration/gyro bandwidths are 260/256 Hz and the gyro base rate is 8 kHz. For modes 1–6 the gyro base rate is 1 kHz; the accelerometer internal rate is 1 kHz. Mode 7 is reserved: do not select it through a cast. Consult the manufacturer table for the exact delays and bandwidths for each mode.

Library choices, from higher to lower bandwidth:

```text
MPU6050_BAND_260_HZ
MPU6050_BAND_184_HZ
MPU6050_BAND_94_HZ
MPU6050_BAND_44_HZ
MPU6050_BAND_21_HZ
MPU6050_BAND_10_HZ
MPU6050_BAND_5_HZ
```

The sample divider is an unsigned 8-bit value:

$$
f_{sample}=\frac{f_{gyro\ base}}{1+d},\qquad d\in\{0,1,\ldots,255\}.
$$

Consequently, divider=0 does NOT mean zero Hz or “sampling disabled”; it means divide by one. With a 1 kHz base, divider=1 gives 500 Hz. With an 8 kHz base, divider=15 gives 500 Hz. Those are rate calculations, not recommendations or proof of a timed 500 Hz implementation. When output sampling exceeds the accelerometer's internal rate, repeated acceleration values can occur.

The existing `delay(100)` makes this a roughly 10 Hz read/print programme, regardless of the internal rate. It will not collect every internal sample. A later flight loop will need explicit timing and fresh-sample handling rather than this delay.

Filtering reduces some noise but adds delay and reduces useful response to fast changes. For illustration, a delay tau contributes an approximate phase lag:

$$
\phi(f)=-2\pi f\tau\quad\text{radians}.
$$

That relation explains why we must consider delay when tuning the future controller. A smoother-looking serial plot alone does not establish a better flight configuration. Likewise a 100 ms print interval hides fast vibration instead of filtering it properly; a later characterization lesson must record at a suitable rate and consider aliasing.

## 6. How a register field works

A register is an addressed byte. A field is a group of bits within that byte. To understand one, read: address -> reset value -> R/W access -> bit positions -> allowed meanings -> side effects -> timing.

For example, ACCEL_CONFIG at 0x1C has a range field AFS_SEL in bits 4:3. To choose a field value q without overwriting unrelated bits:

$$
M=((1\ll w)-1)\ll s,
\qquad r_{new}=(r_{old}\mathbin{\&}\sim M)\mathbin{|}((q\ll s)\mathbin{\&}M).
$$

With width w=2 and shift s=3, M is 0x18. This is the general read-modify-write idea used by library bit-field helpers. Reading an R/W field back checks the stored code, but does not prove the analog sensor meets its specification. Some status fields clear on read and some reset controls clear themselves: do not expect every register to behave like ordinary RAM.

Use library calls for covered settings. For unsupported fields, study the register description before adding one small BusIO helper. Do not overwrite reserved bits with guessed values.

## 7. Read the rest of the datasheet in this order

| Lesson | Read in the manufacturer documents | What you should be able to answer |
|---|---|---|
| A | Overview/block diagram and MPU6000 versus MPU6050 | Which features actually belong to our part? |
| B | Electrical specifications, pinout and application circuit | Which limits apply to the chip rather than a breakout's regulator? |
| C | Axes/orientation, mounting and mechanical sections | Which sensor axis points in each drone direction? |
| D | Gyro and acceleration characteristics | What do range, noise, offset, drift and self-test mean? |
| E | Serial interface and output-data layout | How are signed words transferred and assembled? |
| F | Power, clock and reset controls | Which settings disappear on reset and which clock runs sampling? |
| G | CONFIG, range fields and sample divider | What bandwidth, delay and output rate are actually selected? |
| H | Data-ready interrupt and interrupt pin behavior | Can software react to a new sample instead of guessing readiness? |
| I | FIFO, count, overflow and reset | How do we preserve multiple samples and detect losses? |
| J | Auxiliary I2C master and bypass | Who owns communication with an external sensor? |
| K | Motion detection, high-pass and low-power cycle | Which features detect events rather than supply normal flight data? |
| L | Self-test and temperature output | How do we check sensor response and examine thermal drift? |
| M | FSYNC and synchronization | What happens to a data bit when external sync is enabled? |
| N | DMP description | Which processing is supported by documented APIs versus vendor firmware? |

This route covers the document's functional areas. We will implement them only when a lesson needs them; the current code does not secretly enable FIFO, DMP, motion detection or an auxiliary sensor master.

### Important distinctions when you reach those chapters

- **Clock versus bus:** selecting a sensor clock source is not selecting SDA/SCL speed. External clock options require the corresponding real hardware; an enum alone does not create a signal.
- **Data-ready versus polling:** a new event from the library is a software read. It does not by itself prove a new internal sample arrived. The current code has no data-ready freshness guarantee.
- **FIFO versus output registers:** reading a register gives the available sample, not every earlier sample that occurred while software waited. FIFO adds capacity and overflow handling, not infinite storage or better intrinsic accuracy.
- **Interrupt wiring:** pin polarity, latching and clear behavior must match both the register settings and the ESP32 input/wiring. INT is not currently used by our bench lesson.
- **Auxiliary bus:** a magnetometer still needs its own correct driver. The MPU is a six-axis sensor; bypass/master settings are routing choices, not magnetometer calibration.
- **High-pass versus gravity:** a motion-detection high-pass function is not a replacement for an attitude-based gravity model in altitude estimation.
- **Self-test versus calibration:** self-test deliberately changes sensor response to check function. Passing it is not proof that bias, scale or mounting has been calibrated accurately.
- **Temperature:** the chip's die temperature can help investigate drift. It is not automatically ambient-air temperature or an already fitted temperature-compensation model.
- **FSYNC versus extra precision:** synchronization may repurpose a low data bit; understand that effect rather than assuming it creates a higher-resolution sample.
- **DMP versus Kalman:** do not infer a documented Kalman implementation or open configuration interface simply because a digital motion processor is mentioned.

## 8. A repeatable configuration experiment

1. Save the current working commit and raw log.
2. Write down the question: for example, “Does this bandwidth reduce stationary noise at an acceptable delay?”
3. Change one named setting and note the expected register code and physical meaning.
4. Upload yourself and record the startup readback. A failure means stop and resolve the configuration, not reinterpret the log as success.
5. Use the same fixture, sensor mounting, temperature conditions and read timing. Record original samples and timestamps rather than screenshots alone.
6. Compare statistics and a deliberately repeatable motion. For noise, preserve sample count and spacing; for delay, use a reference or a controlled timing experiment. Printed decimal places do not create measurement accuracy.
7. Keep the failed trial too. Recheck calibration whenever the setup or response changes.

For N stationary measurements, our later statistics lesson will compute:

$$
\bar a=\frac1N\sum_{i=1}^{N}a_i,
\qquad s^2=\frac1{N-1}\sum_{i=1}^{N}(a_i-\bar a)^2.
$$

Closely spaced filtered samples can be correlated; N readings do not automatically give N independent observations. That is why we will not derive trustworthy Kalman noise parameters from the current six means or invent standard deviations from their many displayed decimal digits.

## 9. Current limits

This is still one accelerometer calibration lesson. The calibration includes the current sensor-to-drone mounting. Gyro offsets, continuous disconnection detection, sensor timing profiling and filter selection for flight are not validated. Only bias fits in the MPU offset words; the complete scale/mounting matrix stays in software. The registers are volatile and restored by our boot code from ESP32 firmware constants. See [calibration and power-cycle checks](accelerometer-calibration.md) before leaving this phase.
