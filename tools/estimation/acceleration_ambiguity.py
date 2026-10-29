"""Known-truth static IMU ambiguity model; stdlib only, no hardware access."""
import json
import math


def experiment():
    angle = math.radians(10.0)
    gravity = 9.80665
    # Sign convention: level, stationary accelerometer reads +g on body Z.
    tilted = (0.0, gravity * math.sin(angle), gravity * math.cos(angle))
    acceleration = (0.0, tilted[1], tilted[2] - gravity)
    level_accelerating = (0.0, acceleration[1], gravity + acceleration[2])
    assert all(abs(a - b) < 1e-12 for a, b in zip(tilted, level_accelerating))
    inferred = math.degrees(math.atan2(tilted[1], tilted[2]))
    assert abs(inferred - 10.0) < 1e-12
    assert abs(math.sqrt(sum(v * v for v in tilted)) - gravity) < 1e-12
    return {
        'engine': 'Python standard-library analytic model; no sensor or simulator',
        'assumptions': 'Ideal noiseless IMU, constant orientations, zero angular rate; initial attitude unknown; transitions excluded.',
        'gravity_m_s2': gravity,
        'tilted_stationary_truth_roll_deg': 10.0,
        'level_accelerating_truth_roll_deg': 0.0,
        'level_case_body_acceleration_m_s2': acceleration,
        'identical_accelerometer_m_s2': tilted,
        'identical_gyro_rad_s': [0.0, 0.0, 0.0],
        'accelerometer_norm_g': 1.0,
        'accelerometer_roll_deg_both_cases': inferred,
        'level_case_accelerometer_angle_error_deg': inferred,
        'conclusion': 'A norm-only acceleration gate accepts both. Identical IMU inputs cannot uniquely determine both true attitudes without additional information.',
        'limits': 'Does not prove every maneuver ambiguous. Known initial attitude, motion history or independent aiding can distinguish cases. No physical frame, MCU timing, dynamic filter or flight acceptance.'
    }


if __name__ == '__main__':
    print(json.dumps(experiment(), indent=2))
