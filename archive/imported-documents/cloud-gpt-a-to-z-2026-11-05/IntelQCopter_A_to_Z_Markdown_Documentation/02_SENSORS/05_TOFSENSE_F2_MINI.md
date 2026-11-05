# TOFSense F2 Mini — range sensor integration and calibration

Device: Nooploop TOFSense F2 Mini mounted approximately under the lower center PCB and pointed toward the ground. The November 4 successful calibration session contains **no validated LiDAR samples**; the earlier GUI allows LiDAR stages to be skipped without losing IMU/MAG/BMP data.

## Known software interfaces

- Older calibration GUI expects LiDAR on **a second USB-to-UART COM port**, not embedded in the ESP32 52-byte calibration packet.
- FC v2 includes an optional 16-byte Nooploop Frame0 parser, checksum/status checks and tilt-compensated relative height input.
- `IQ_LIDAR_ENABLE=0` and `IQ_LIDAR_RX_PIN=-1` by default. No actual ESP32 GPIO routing has been verified.
- Typical maximum fresh range output is around 50 Hz for the referenced F2 Mini mode; verify exact model firmware and mode.

## Geometry

If the downward sensor measures slant distance r to a locally flat ground plane and body tilt angle is α, the vertical component is approximately `r cos(α)`, after accounting for the sensor's body-axis orientation and optical-center offset. Incorrect sign of +Z or incorrect offset can shift all altitude readings.

## Calibration plan

Calibrate at actual known distances 0.25, 0.50, 1.00 and 2.00 m, with 300 samples per distance in the historical GUI defaults. Measure uncertainty, dropouts, confidence/signal-strength flags and tilt behavior. Validate the ranging surface and minimum distance. Store measured offset/scale/variance per distance, and use innovations to reject bad measurement updates rather than forcing the Kalman filter to follow them.

## Flight readiness

A LiDAR can improve near-ground height estimation but cannot be assumed to work over every surface, in bright light, at large tilt or through propwash/dust. Do not arm on an unverified UART RX pin or sensor status.

Reference: Nooploop TOFSense F2 Mini datasheet and `nlink_tofsense_frame0` parser cited in FC v2 sources.
