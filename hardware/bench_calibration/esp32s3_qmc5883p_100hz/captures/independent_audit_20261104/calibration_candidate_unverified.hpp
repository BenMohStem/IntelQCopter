// IntelQCopter raw calibration candidate, 2026-11-04.
// GENERATED FROM REAL DATA. NOT AUTOMATICALLY VALIDATED FOR FLIGHT.
// Raw sensor-frame axes. DO NOT apply the GUI XY-swap (determinant -1)
// as a physical rotation; mounting/sign/yaw checks remain required.
// Apply subtract-bias THEN matrix multiplication.
#pragma once
namespace iq_cal_candidate {
static constexpr float kGyroBiasDps[3] = {0.161968974f, -0.304950151f, -0.877433074f};
static constexpr float kAccelBiasMps2[3] = {1.264655572f, -0.059103429f, -0.678744839f};
static constexpr float kAccelCorrection[3][3] = {
 {0.999284313f, -0.038750762f, -0.032967877f},
 {0.041351387f, 0.992793975f, 0.040449121f},
 {0.042481183f, -0.040866969f, 0.987204883f}
};
static constexpr float kMagHardIronUt[3] = {-29.482301006f, -18.397983738f, -35.474272846f};
static constexpr float kMagSoftIronRelative[3][3] = {
 {0.973366985f, 0.005036971f, 0.038277934f},
 {0.005036971f, 0.964534610f, -0.017911741f},
 {0.038277934f, -0.017911741f, 1.067011358f}
};
// The mag field magnitude after correction is only inferred: 40.7434 uT mean.
// MAG capture firmware used 0x3D (ODR 200 Hz, OSR1=1, OSR2=1), not 0xC9 (100 Hz).
}
