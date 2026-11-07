#pragma once

// Nov 7, 2026 completed-session calibration fit for the isolated ESP32-S3
// software candidate. Numerical sensor fits are derived from raw integer
// counts. The simulation PID set is carried forward from the Nov 6 model audit;
// the Nov 7 stationary calibration data cannot identify flight PID gains.

namespace iq::nov07_recalibration {

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

inline constexpr bool kResearchOnly = true;
inline constexpr bool kHardwareValidated = false;
inline constexpr bool kApplyCalibrationToRuntime = true;
inline constexpr bool kApplySimulationPidToRuntime = true;
inline constexpr bool kPidValuesAreFlightValidated = false;
inline constexpr bool kSensorAxesVerified = false;
inline constexpr bool kMpuRangeAndOffsetReadbackRequired = true;
inline constexpr bool kEnableMagHeadingFusion = false;
inline constexpr bool kEnableMotorAccelerationPid = false;
inline constexpr bool kHasMotorRpmTelemetry = false;
inline constexpr bool kHasLidarCalibrationData = false;

// Three complete Nov 7 sessions. Filename timestamps are Africa/Lagos local;
// session_started_utc in the CSV is one hour earlier.
inline constexpr char kSourcePackage[] =
    "IntelQCopter_Nov07_Calibration_Model_Gains_500dps_8g";
inline constexpr char kFitSessions[] =
    "20261107_021542_146786, 20261107_024540_453344, "
    "20261107_032549_896941";

// MPU6050 raw-count scales from the configured full-scale ranges.
inline constexpr float kAccelCountsPerG = 4096.0f;
inline constexpr float kGyroCountsPerDps = 65.5f;
inline constexpr float kGravityMps2 = 9.80665f;

// The Nov 7 CSV pressure_pa column already contains Pa.
inline constexpr float kPressureCsvToPa = 1.0f;
inline constexpr bool kPressureUnitConversionVerified = true;

// Accelerometer affine fit in the captured sensor-chip frame.
// Input: raw_counts / 4096 * 9.80665, in m/s^2.
// Output: fitted specific-force vector in the report's reference frame.
// Board-to-body rotation and pose signs still require physical confirmation.
inline constexpr Vector3f kAccelBiasMps2 = {
    0.25928109f, -0.08780301f, -0.89685228f};
inline constexpr Matrix3f kAccelCorrection = {{
    { 0.99463969f, -0.04280700f,  0.04798329f},
    { 0.04255186f,  0.99493002f,  0.02387155f},
    {-0.05126050f, -0.03758379f,  0.98640514f}
}};
inline constexpr float kAccelFitResidualRmsG = 0.0061374f;
inline constexpr float kAccelFitResidualRmsMps2 = 0.06018733f;

// Robust mean from 89,409 accepted REST gyro records after rejecting 591
// gross anomalies from the 90,000 raw REST records. Subtract from raw/65.5.
inline constexpr Vector3f kGyroBiasDps = {
    -11.04461742f, 6.99774234f, -1.10236200f};

// Sample SD across the three session REST means, not a temperature model.
inline constexpr Vector3f kGyroBiasBetweenSessionSdDps = {
    0.00700642f, 0.03277836f, 0.04753799f};

// Rounded conservative per-axis stationary noise estimate; the report gives
// approximately 0.07-0.08 deg/s RMS for the two stable-temperature sessions.
inline constexpr Vector3f kGyroRestNoiseSdDps = {
    0.08f, 0.08f, 0.08f};

// QMC5883P sphere-fit candidate in chip-frame microtesla.
// Scaling assumes ±8 G: raw / 37.5 = microtesla. Heading/body rotation and
// powered-motor magnetic distortion have not been validated.
inline constexpr float kMagCountsPerMicrotesla = 37.5f;
inline constexpr Vector3f kMagHardIronMicrotesla = {
    -7.34180573f, -13.99272901f, -22.84747418f};
inline constexpr Matrix3f kMagSoftIronDet1 = {{
    { 0.97174713f, -0.01740207f,  0.03634599f},
    {-0.01740207f,  0.96045521f, -0.02183163f},
    { 0.03634599f, -0.02183163f,  1.07361879f}
}};
inline constexpr float kMagFieldRadiusMicrotesla = 40.8129f;
inline constexpr float kMagSessionCorrectedRadiusSdMicrotesla[3] = {
    1.8706f, 1.8785f, 1.3534f};

// BMP stationary noise evidence. No session mean is a portable ground
// reference; calibration/altitude control remains invalid until ground zero.
inline constexpr float kBaroBestCaseStationarySdPa = 3.0144f;
inline constexpr float kBaroConservativeStageSdPa = 3.2875f;
inline constexpr float kBaroConservativePressureVariancePa2 = 10.81f;
inline constexpr float kBaroHeightMeasurementVarianceM2 = 0.08f;
inline constexpr float kBaroSessionMeanPressurePa[3] = {
    99125.624f, 99106.733f, 99101.015f};
inline constexpr bool kHasValidatedGroundPressureReference = false;

// Inherited Nov 6 model priors; the Nov 7 calibration captures do not measure
// mass, inertia, thrust, torque authority, ESC dynamics, or motor lag.
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

// Carried-forward simulation-only PID candidates. Rate-loop gains output N*m
// in the model, while the actuator effectiveness and torque-to-command
// conversion remain assumed. These are not Nov 7 fitted or flight-validated.
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

// Requires measured motor RPM and plant identification; remains disabled.
inline constexpr PidGains kMotorAccelerationPidResearchOnly = {
    0.0000100348765f, 0.0000100348765f, 0.0000000200698f};

inline constexpr float kRollPitchTorqueAuthorityNm = 0.74993305f;
inline constexpr float kYawTorqueAuthorityNm = 0.07080835f;
inline constexpr float kDerivativeFilterCutoffHz = 20.0f;

} // namespace iq::nov07_recalibration