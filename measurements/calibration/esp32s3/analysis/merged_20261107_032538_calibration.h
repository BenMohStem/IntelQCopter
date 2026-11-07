#pragma once
// Auto-generated sensor calibration constants for ESP32-S3.
// Sensor frame only. Mounting orientation/frame transforms are separate.
// v_corrected = MATRIX * (v_raw_in_physical_units - BIAS)
namespace intelqcopter_cal {
inline constexpr float GYRO_BIAS_DPS[3] = {-5.52104771f, 3.49252927f, -0.559375925f};

inline constexpr float GYRO_SCALE_MATRIX[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

inline constexpr float MAG_BIAS_UT[3] = {-7.29598505f, -14.1749726f, -23.0468091f};

inline constexpr float MAG_MATRIX[3][3] = {
    {0.964456918f, -0.0258025275f, 0.0358141011f},
    {-0.0258025275f, 0.951675463f, -0.0225649054f},
    {0.0358141011f, -0.0225649054f, 1.07649431f}
};

inline constexpr float BMP_REFERENCE_PRESSURE_PA = 99115.5f;
} // namespace intelqcopter_cal
