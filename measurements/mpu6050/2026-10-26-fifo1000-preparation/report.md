# Prepared 1 kHz stage: verification status

The ESP32 firmware builds with its source fingerprint embedded. Forty-one unique offline checks and fourteen native actual-source/fake-peripheral cases pass. All calibration constants and target register words are retained. Fast math uses float constants; observable latest state is written every packet. Slow register verification is spread out instead of reading six words in one service slot.

**Physical 1 kHz validation is pending.** The connected board remains on older V2 firmware; it has not been flashed by the assistant. The separate upload-choice question is pending. Source execution under a host model is not ESP32 emulation, silicon behavior, electrical validation or measured timing.

Host cases cover normal capture, timer wrap inside capture, retained offsets, invalid accel/gyro boot words, failed trim readback, failed FIFO enable, FIFO overflow, failed status/packet reads, changed offsets, slow-bus backlog, timeout and abort. Three successful source runs each generate 20,000 model packets and 200 MCU diagnostic lines verified by the real transport parser. Their clocks/devices are modeled; no physical samples are claimed.

Original compiler, test and host-model logs remain in the local artifact directories identified by [verification.json](verification.json). Earlier warning-bearing builds and trial logs are retained. [Current stage guide](../../../docs/fifo1000-stage.md) defines the exact real-data acceptance evidence. No PID or motor operation is claimed.
