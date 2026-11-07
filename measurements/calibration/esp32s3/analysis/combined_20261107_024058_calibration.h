#pragma once
// Auto-generated sensor calibration constants for ESP32-S3.
// Sensor frame only. Mounting orientation/frame transforms are separate.
// v_corrected = MATRIX * (v_raw_in_physical_units - BIAS)
namespace intelqcopter_cal {
inline constexpr float GYRO_BIAS_DPS[3] = {-5.51816794f, 3.50552035f, -0.538596664f};

inline constexpr float GYRO_SCALE_MATRIX[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

inline constexpr float MAG_BIAS_UT[3] = {-6.68549549f, -13.911805f, -23.3829756f};

inline constexpr float MAG_MATRIX[3][3] = {
    {0.964395659f, -0.0102022241f, 0.0458702652f},
    {-0.0102022241f, 0.949233316f, -0.0211798552f},
    {0.0458702652f, -0.0211798552f, 1.08143617f}
};

inline constexpr float BMP_REFERENCE_PRESSURE_PA = 99126.0f;
} // namespace intelqcopter_cal
