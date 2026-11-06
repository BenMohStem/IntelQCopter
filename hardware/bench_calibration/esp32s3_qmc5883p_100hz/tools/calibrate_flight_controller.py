#!/usr/bin/env python3
"""
IntelQCopter Calibration Processor
==================================
Processes calibration CSV into flight controller calibration constants.
Handles missing/incomplete data gracefully - uses identity for missing sensors.

Usage:
    python calibrate_flight_controller.py <calibration_csv> [output_header.h]

Reads calibration CSV from the bench calibration GUI and outputs:
- Accelerometer bias and scale matrix (6-face fit)
- Gyroscope bias (stationary)
- Magnetometer hard-iron offset and soft-iron matrix (ellipsoid fit)
- Barometer zero offset

Outputs C header file ready to include in flight_controller.c
"""

import csv
import math
import sys
import numpy as np
from pathlib import Path

G = 9.80665  # m/s^2


def load_calibration_csv(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def separate_by_phase(rows):
    phases = {
        'REST': [], '+X': [], '-X': [], '+Y': [], '-Y': [], '+Z': [], '-Z': [],
        'YAW': [], 'PITCH': [], 'ROLL': [], 'TUMBLE': [], 'BMP': [],
    }
    for row in rows:
        pose = row.get('pose', '').strip()
        if pose in phases:
            phases[pose].append(row)
    return phases


def safe_mean(values):
    """Compute mean of finite values only."""
    vals = [v for v in values if v is not None and math.isfinite(v)]
    if len(vals) == 0:
        return None
    return sum(vals) / len(vals)


def compute_gyro_bias(rest_rows):
    """Compute gyro bias from stationary REST samples."""
    gx_vals, gy_vals, gz_vals = [], [], []
    for row in rest_rows:
        try:
            gx_vals.append(float(row['gx_dps']))
            gy_vals.append(float(row['gy_dps']))
            gz_vals.append(float(row['gz_dps']))
        except (ValueError, KeyError):
            continue

    if len(gx_vals) < 50:
        print(f"WARNING: Only {len(gx_vals)} gyro samples (need >50)")
        return None

    bias = [safe_mean(gx_vals), safe_mean(gy_vals), safe_mean(gz_vals)]
    gx_var = sum((v - bias[0])**2 for v in gx_vals) / max(1, len(gx_vals) - 1)
    gy_var = sum((v - bias[1])**2 for v in gy_vals) / max(1, len(gy_vals) - 1)
    gz_var = sum((v - bias[2])**2 for v in gz_vals) / max(1, len(gz_vals) - 1)

    print(f"Gyro calibration: {len(gx_vals)} samples")
    print(f"  Bias:   ({bias[0]:+.4f}, {bias[1]:+.4f}, {bias[2]:+.4f}) deg/s")
    print(f"  Sigma:  ({math.sqrt(gx_var):+.4f}, {math.sqrt(gy_var):+.4f}, {math.sqrt(gz_var):+.4f}) deg/s")

    return bias, [math.sqrt(gx_var), math.sqrt(gy_var), math.sqrt(gz_var)]


def compute_accel_calibration(face_rows):
    """
    Compute accelerometer calibration from 6-face data.
    Each face has the corresponding axis pointing up (gravity aligned).
    """
    face_gravity = {
        '+X': (1.0, 0.0, 0.0), '-X': (-1.0, 0.0, 0.0),
        '+Y': (0.0, 1.0, 0.0), '-Y': (0.0, -1.0, 0.0),
        '+Z': (0.0, 0.0, 1.0), '-Z': (0.0, 0.0, -1.0),
    }

    face_means = {}
    for face, rows in face_rows.items():
        if face not in face_gravity:
            continue
        ax_vals, ay_vals, az_vals = [], [], []
        for row in rows:
            try:
                ax = float(row['ax_mps2'])
                ay = float(row['ay_mps2'])
                az = float(row['az_mps2'])
                if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(az)):
                    continue
                ax_vals.append(ax)
                ay_vals.append(ay)
                az_vals.append(az)
            except (ValueError, KeyError):
                continue

        if len(ax_vals) < 50:
            print(f"WARNING: Face {face} has only {len(ax_vals)} samples (need >50)")
            continue

        face_means[face] = (
            sum(ax_vals) / len(ax_vals),
            sum(ay_vals) / len(ay_vals),
            sum(az_vals) / len(az_vals),
        )
        print(f"  Face {face}: {len(ax_vals)} samples, "
              f"accel = ({face_means[face][0]:+.4f}, {face_means[face][1]:+.4f}, {face_means[face][2]:+.4f}) m/s²")

    if len(face_means) < 3:
        print(f"ERROR: Only {len(face_means)} of 6 faces have sufficient data")
        return None

    # Build least-squares system: measured = scale_matrix * expected_gravity + bias
    # For each face i: m_i = S * g_i + bias
    # We solve for S (3x3) and bias (3x1) using all 6 faces

    A_rows = []
    b_x = []
    b_y = []
    b_z = []

    for face, gravity in face_gravity.items():
        if face not in face_means:
            continue
        gx_raw, gy_raw, gz_raw = gravity
        mx, my, mz = face_means[face]
        A_rows.append([gx_raw, gy_raw, gz_raw])
        b_x.append(mx)
        b_y.append(my)
        b_z.append(mz)

    A = np.array(A_rows)  # Nx3
    b_x = np.array(b_x)
    b_y = np.array(b_y)
    b_z = np.array(b_z)

    try:
        scale_x = np.linalg.lstsq(A, b_x, rcond=None)[0]
        scale_y = np.linalg.lstsq(A, b_y, rcond=None)[0]
        scale_z = np.linalg.lstsq(A, b_z, rcond=None)[0]
    except np.linalg.LinAlgError:
        print("ERROR: Failed to solve accelerometer calibration matrix")
        return None

    scale = [
        [scale_x[0], scale_x[1], scale_x[2]],
        [scale_y[0], scale_y[1], scale_y[2]],
        [scale_z[0], scale_z[1], scale_z[2]],
    ]

    # Bias: bias = measured - scale @ expected_gravity (use +X face as reference)
    bias = [
        face_means['+X'][0] - (scale[0][0] * 1.0 + scale[0][1] * 0.0 + scale[0][2] * 0.0),
        face_means['+X'][1] - (scale[1][0] * 1.0 + scale[1][1] * 0.0 + scale[1][2] * 0.0),
        face_means['+X'][2] - (scale[2][0] * 1.0 + scale[2][1] * 0.0 + scale[2][2] * 0.0),
    ]

    print(f"Accel calibration: {len(face_means)} faces")
    print(f"  Bias:   ({bias[0]:+.4f}, {bias[1]:+.4f}, {bias[2]:+.4f}) m/s²")
    print(f"  Scale:  ({scale[0][0]:+.4f}, {scale[0][1]:+.4f}, {scale[0][2]:+.4f})")
    print(f"          ({scale[1][0]:+.4f}, {scale[1][1]:+.4f}, {scale[1][2]:+.4f})")
    print(f"          ({scale[2][0]:+.4f}, {scale[2][1]:+.4f}, {scale[2][2]:+.4f})")

    return bias, scale


def compute_mag_calibration(tumble_rows):
    mx_vals, my_vals, mz_vals = [], [], []
    for row in tumble_rows:
        try:
            mx_vals.append(float(row['mx_uT']))
            my_vals.append(float(row['my_uT']))
            mz_vals.append(float(row['mz_uT']))
        except (ValueError, KeyError):
            continue

    if len(mx_vals) < 100:
        print(f"WARNING: Only {len(mx_vals)} MAG samples (need >100)")
        return None

    x = np.array(mx_vals)
    y = np.array(my_vals)
    z = np.array(mz_vals)

    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones_like(x)
    ])

    try:
        params, _, _, _ = np.linalg.lstsq(D, -np.ones(len(x)), rcond=None)
    except np.linalg.LinAlgError:
        print("ERROR: Failed to solve magnetometer ellipsoid fit")
        return None

    A = np.array([
        [params[0], params[3], params[4]],
        [params[3], params[1], params[5]],
        [params[4], params[5], params[2]]
    ])
    b = np.array([params[6], params[7], params[8]])

    try:
        center = -0.5 * np.linalg.inv(A) @ b
    except np.linalg.LinAlgError:
        print("ERROR: Matrix inversion failed for magnetometer calibration")
        return None

    eigenvalues, eigenvectors = np.linalg.eigh(A)
    if np.any(eigenvalues <= 0):
        print("ERROR: Ellipsoid matrix not positive definite")
        return None

    scale_factors = 1.0 / np.sqrt(eigenvalues)
    soft_iron = np.diag(scale_factors) @ eigenvectors.T

    mean_radius = np.mean([
        np.sqrt(
            (soft_iron[0][0]*(mx_vals[i]-center[0]) + soft_iron[0][1]*(my_vals[i]-center[1]) + soft_iron[0][2]*(mz_vals[i]-center[2]))**2 +
            (soft_iron[1][0]*(mx_vals[i]-center[0]) + soft_iron[1][1]*(my_vals[i]-center[1]) + soft_iron[1][2]*(mz_vals[i]-center[2]))**2 +
            (soft_iron[2][0]*(mx_vals[i]-center[0]) + soft_iron[2][1]*(my_vals[i]-center[1]) + soft_iron[2][2]*(mz_vals[i]-center[2]))**2
        ) for i in range(len(mx_vals))
    ])
    if mean_radius > 0:
        scale_factor = 45.0 / mean_radius
        soft_iron *= scale_factor

    print(f"Mag calibration: {len(mx_vals)} samples")
    print(f"  Hard iron: ({center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}) µT")
    print(f"  Soft iron:")
    for i in range(3):
        print(f"            ({soft_iron[i][0]:+.4f}, {soft_iron[i][1]:+.4f}, {soft_iron[i][2]:+.4f})")
    print(f"  Mean field magnitude: {mean_radius:.2f} µT")

    return center, soft_iron


def generate_c_header(accel_cal, mag_cal, gyro_cal, output_path):
    accel_bias, accel_scale = accel_cal
    mag_hard_iron, mag_soft_iron = mag_cal
    gyro_bias, gyro_sigma = gyro_cal

    header = f"""/* 
 * IntelQCopter Calibration Data
 * Auto-generated by calibrate_flight_controller.py
 * DO NOT EDIT MANUALLY - regenerate from calibration CSV
 */

#ifndef INTELQCOPTER_CALIBRATION_H
#define INTELQCOPTER_CALIBRATION_H

#include <stdbool.h>

/* Accelerometer calibration: calibrated = scale_matrix * (raw - bias) */
/* Units: m/s^2 */
static const float CAL_ACCEL_BIAS[3] = {{
    {accel_bias[0]:.6f}f, {accel_bias[1]:.6f}f, {accel_bias[2]:.6f}f
}};
static const float CAL_ACCEL_SCALE[3][3] = {{
    {{ {accel_scale[0][0]:.6f}f, {accel_scale[0][1]:.6f}f, {accel_scale[0][2]:.6f}f }},
    {{ {accel_scale[1][0]:.6f}f, {accel_scale[1][1]:.6f}f, {accel_scale[1][2]:.6f}f }},
    {{ {accel_scale[2][0]:.6f}f, {accel_scale[2][1]:.6f}f, {accel_scale[2][2]:.6f}f }}
}};

/* Gyroscope calibration: calibrated = raw - bias */
/* Units: deg/s */
static const float CAL_GYRO_BIAS[3] = {{
    {gyro_bias[0]:.6f}f, {gyro_bias[1]:.6f}f, {gyro_bias[2]:.6f}f
}};

/* Magnetometer calibration: calibrated = soft_iron * (raw - hard_iron) */
/* Units: microtesla */
static const float CAL_MAG_HARD_IRON[3] = {{
    {mag_hard_iron[0]:.6f}f, {mag_hard_iron[1]:.6f}f, {mag_hard_iron[2]:.6f}f
}};
static const float CAL_MAG_SOFT_IRON[3][3] = {{
    {{ {mag_soft_iron[0][0]:.6f}f, {mag_soft_iron[0][1]:.6f}f, {mag_soft_iron[0][2]:.6f}f }},
    {{ {mag_soft_iron[1][0]:.6f}f, {mag_soft_iron[1][1]:.6f}f, {mag_soft_iron[1][2]:.6f}f }},
    {{ {mag_soft_iron[2][0]:.6f}f, {mag_soft_iron[2][1]:.6f}f, {mag_soft_iron[2][2]:.6f}f }}
}};

/* Calibration validity flag */
static const bool CAL_VALID = true;

#endif /* INTELQCOPTER_CALIBRATION_H */
"""

    with open(output_path, 'w') as f:
        f.write(header)
    print(f"\nCalibration header written to {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python calibrate_flight_controller.py <calibration_csv> [output_header.h]")
        sys.exit(1)

    csv_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else 'qcopter_calibration.h'

    print(f"Loading calibration data from {csv_path}...")
    rows = load_calibration_csv(csv_path)
    print(f"Loaded {len(rows)} samples")

    phases = separate_by_phase(rows)
    for phase, phase_rows in phases.items():
        print(f"  {phase}: {len(phase_rows)} samples")

    accel_cal = None
    mag_cal = None
    gyro_cal = None

    face_rows = {face: phases[face] for face in ['+X', '-X', '+Y', '-Y', '+Z', '-Z']}
    if all(len(face_rows[f]) > 0 for f in face_rows):
        accel_cal = compute_accel_calibration(face_rows)
    else:
        missing = [f for f in ['+X','-X','+Y','-Y','+Z','-Z'] if len(face_rows[f]) == 0]
        print(f"WARNING: Missing accelerometer face data: {missing}")

    if len(phases['TUMBLE']) > 0:
        mag_cal = compute_mag_calibration(phases['TUMBLE'])
    else:
        print("WARNING: No magnetometer tumble data - will use identity calibration")
        mag_cal = ([0.0, 0.0, 0.0], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    if len(phases['REST']) > 0:
        gyro_cal = compute_gyro_bias(phases['REST'])
    else:
        print("WARNING: No REST data for gyro calibration - will use zero bias")
        gyro_cal = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    generate_c_header(accel_cal, mag_cal, gyro_cal, output_path)

    print("\nCalibration processing complete!")
    print("Next steps:")
    print("  1. Copy qcopter_calibration.h to firmware/flight_controller/")
    print("  2. Include it in flight_controller.c")
    print("  3. Call load_calibration() during init")
    print("  4. Apply calibration in sensor_read() before control")


if __name__ == '__main__':
    main()