# Outstanding unknowns — answer with measurements, not guesses

1. What is the actual weighed ready-to-fly mass, battery model/mass and complete propulsion inventory?
2. What propeller diameter, pitch and handedness are physically installed?
3. What are the exact motor GPIO -> ESC -> arm positions and rotation directions?
4. What are the measured per-motor thrust curves, yaw torque leverage and lag with the 3S pack?
5. Which physical MPU and QMC XYZ directions correspond to body FLU, verified via hand rotations?
6. Does MPU hardware-offset readback match its predicted scaling and preserve factory compensation bits on this exact chip?
7. Can FC v2 sustain a real 500 Hz loop at 400 kHz I²C with near-zero overruns over extended operation?
8. How often does QMC fall back to 100 kHz and is it stable after warm/cold starts and representative electrical load?
9. How does BMP static/dynamic pressure noise change under enclosed mounting, airflow, temperature and actual propwash?
10. Which ESP32 RX pin, logic voltage and UART port are connected to the TOFSense F2 Mini?
11. Is the Xbox controller's BLE firmware compatible with direct ESP32-S3, and what independent kill/failsafe exists?
12. Can the proposed N150 hardware actually run the planned ROS2/vision software within its RAM/power budget?
13. Are the mass/inertia model coordinate origin, LiDAR offset and URDF base_link definitions consistent?
14. What flight-test acceptance criteria and safety review sign-off process will be required?

Document dates and evidence when answering. A guess from a product listing should remain marked as an assumption.
