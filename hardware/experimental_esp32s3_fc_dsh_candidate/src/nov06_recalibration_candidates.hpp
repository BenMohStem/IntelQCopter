#pragma once

// Independent Nov 6, 2026 three-session recalibration and PID audit.
// Values are loaded into the isolated ESP32-S3 software estimator/controller
// candidate for review. This does not validate hardware or authorize outputs.

namespace iq::nov06_recalibration {

struct Vector3f {
  float x;
  float y;
  float z;
};

struct Matrix3f {
  float m[3][3];
};

struct PidGains {
  float kp;
  float ki;
  float kd;
};

// Software candidate values are active in the isolated S3 source. Physical
// command input/output remain compile-time disabled in main.cpp.
inline constexpr bool kResearchOnly = true;
inline constexpr bool kHardwareValidated = false;
inline constexpr bool kApplyCalibrationToRuntime = true;
inline constexpr bool kApplySimulationPidToRuntime = true;
inline constexpr bool kEnableMagHeadingFusion = false;
inline constexpr bool kEnableMotorAccelerationPid = false;
inline constexpr bool kHasMotorRpmTelemetry = false;

// Source sessions: iq_500hz_20261106_202225, _205042, and _205914.
// Fits use the last accepted attempt per phase and raw integer counts.
inline constexpr char kSourcePackage[] =
    "IntelQCopter_Nov06_Recalibration_and_PID_Model_v1";
inline constexpr char kFitSessions[] =
    "20261106_202225, 20261106_205042, 20261106_205914";

// Assumed source-sensor scales. The captures do not contain authoritative
// register readback tying these scales to the exact flashed image.
inline constexpr float kAccelCountsPerG = 4096.0f;       // MPU6050 +/-8 g
inline constexpr float kGyroCountsPerDps = 65.5f;        // MPU6050 +/-500 deg/s
inline constexpr float kMagCountsPerMicrotesla = 37.5f;  // QMC5883P +/-8 G
inline constexpr float kPressureCsvToPa = 100.0f;        // inferred hPa -> Pa
inline constexpr bool kPressureUnitConversionVerified = false;

// Candidate accelerometer fit in chip frame; bias units are m/s^2.
// It was trained on sessions 20:22 and 20:50. Cross-axis terms may include
// hand-placement and chip-mounting error; session 20:59 max vector residual
// was 0.3480 m/s^2. Verify axis signs with a fixture before use.
inline constexpr Vector3f kAccelBiasMps2 = {
    0.24092418f, -0.07899307f, -0.88462110f};
inline constexpr float kAccelCrossSessionMaxVectorErrorMps2 = 0.34802136f;
inline constexpr Matrix3f kAccelCorrection = {{
    { 0.99455302f, -0.04276151f,  0.05176808f},
    { 0.04260636f,  0.99510499f,  0.02178648f},
    {-0.04984395f, -0.03960407f,  0.98668384f}
}};

// Cross-session gyro-rest bias candidate in deg/s. Magnitude is unusually
// large; MPU6050 hardware-offset register contents were not captured. Do not
// apply in software until register readback rules out double subtraction.
inline constexpr Vector3f kGyroBiasDps = {
    -11.01505496f, 7.00116285f, -1.09212824f};
inline constexpr Vector3f kGyroBiasBetweenSessionSdDps = {
    0.00671808f, 0.00262948f, 0.00838520f};
inline constexpr Vector3f kGyroRestNoiseSdDps = {
    0.07033f, 0.08091f, 0.07713f};

// QMC5883P sphere/ellipsoid magnitude-fit candidate in native chip XYZ.
// This is not a heading or sensor-to-body calibration. The body transform,
// heading sign, and powered-motor magnetic interference remain unverified.
inline constexpr Vector3f kMagHardIronMicrotesla = {
    -5.88572601f, -14.17272173f, -22.98138635f};
inline constexpr Matrix3f kMagSoftIronDet1 = {{
    {0.95989367f,  0.01097993f,  0.04246714f},
    {0.01097993f,  0.97021800f, -0.02282975f},
    {0.04246714f, -0.02282975f,  1.07633899f}
}};
inline constexpr float kMagFieldRadiusMicrotesla = 41.54973544f;
inline constexpr float kMagWithinSessionHoldoutPct = 4.0063f;
inline constexpr float kMagCrossSessionHoldoutPct20_22 = 4.3468f;
inline constexpr float kMagCrossSessionHoldoutPct20_50 = 3.8814f;

// Pressure data are strongly autocorrelated and the CSV unit interpretation
// remains unconfirmed. No portable ground-pressure reference is supplied.
inline constexpr float kBaroBestCaseStationarySdPa = 2.86239293f;
inline constexpr float kBaroConservativeStageSdPa = 5.78360071f;
inline constexpr bool kHasValidatedGroundPressureReference = false;

// Inherited model priors; none were measured in the three recalibration runs.
inline constexpr float kAssumedMassMeanKg = 1.10392102f;
inline constexpr float kAssumedMassSdKg = 0.05035945f;
inline constexpr Matrix3f kAssumedInertiaMeanKgM2 = {{
    {0.00978100f, 0.0f,        0.0f},
    {0.0f,        0.01038993f, 0.0f},
    {0.0f,        0.0f,        0.01917019f}
}};
inline constexpr float kAssumedMotorLagMeanSeconds = 0.04018372f;
inline constexpr float kAssumedMotorLagSdSeconds = 0.01479226f;
inline constexpr float kAssumedMaxThrustPerMotorMeanN = 6.00658665f;
inline constexpr float kAssumedMaxThrustPerMotorSdN = 1.48957696f;
inline constexpr float kAssumedLinearHoverFractionMedian = 0.44955984f;
inline constexpr float kAssumedLinearHoverFractionQ025 = 0.29834097f;
inline constexpr float kAssumedLinearHoverFractionQ975 = 0.88856974f;
inline constexpr float kDrawFractionAboveHistorical45PctCap = 0.4986f;

// Simulation-only PID candidates. Rate-loop gains output N*m in the model,
// while this firmware's mixer expects normalized motor-command differentials;
// no validated torque-to-command effectiveness matrix exists. Outer-loop gains
// are design choices, not values identified from stationary sensor data.
inline constexpr PidGains kRollRateTorquePid = {
    0.0723794083f, 0.0342335039f, 0.0000440145f};
inline constexpr PidGains kPitchRateTorquePid = {
    0.0768854934f, 0.0363647604f, 0.0000467547f};
inline constexpr PidGains kYawRateTorquePid = {
    0.0709297215f, 0.0191701950f, 0.0000575106f};
inline constexpr PidGains kRollAnglePid = {0.85f, 0.012f, 0.008f};
inline constexpr PidGains kPitchAnglePid = {0.85f, 0.012f, 0.008f};
inline constexpr PidGains kAltitudePositionPid = {0.35f, 0.003f, 0.010f};
inline constexpr PidGains kVerticalSpeedPid = {0.75f, 0.040f, 0.018f};
inline constexpr PidGains kOptionalYawHeadingPid = {0.70f, 0.010f, 0.006f};

// Research-only motor RPM/rotational-acceleration PID. It requires measured
// per-motor RPM, an identified ESC/motor plant, and a verified propeller limit.
// IMU translational acceleration is not rotor angular acceleration.
inline constexpr PidGains kMotorAccelerationPidResearchOnly = {
    0.0000100348765f, 0.0000100348765f, 0.0000000200698f};

// Assumed maximum incremental torque authority for normalizing inner-loop
// torque demands into the mixer's unitless differential-command inputs.
inline constexpr float kRollPitchTorqueAuthorityNm = 0.74993305f;
inline constexpr float kYawTorqueAuthorityNm = 0.07080835f;
inline constexpr float kDerivativeFilterCutoffHz = 20.0f;

} // namespace iq::nov06_recalibration
