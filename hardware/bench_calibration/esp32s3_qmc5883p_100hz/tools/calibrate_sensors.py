#!/usr/bin/env python3
"""
IntelQCopter Sensor Calibration Analysis
========================================

Reads calibration CSV captures and computes:
  - Accelerometer bias & scale matrix (6-face least-squares)
  - Magnetometer hard-iron offset & soft-iron matrix (min/max box + ellipsoid)
  - Standard deviations / residuals
  - 3D plots: raw cloud, corrected sphere

Usage:
    python calibrate_sensors.py <capture.csv> [--output cal_output.json]

Requires: numpy, scipy, matplotlib, pandas
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.linalg import eigh

G = 9.80665  # standard gravity, m/s^2


# ===========================================================================
# ACCELEROMETER CALIBRATION (6-face least-squares)
# ===========================================================================

def accel_calibration(df: pd.DataFrame) -> dict:
    """
    Six-face accelerometer calibration.

    Model:  m = S @ t + b
    where m = measured (m/s^2), t = true gravity vector, S = 3x3 scale matrix,
    b = 3x1 bias vector.
    """
    face_map = {
        "+X": np.array([1.0, 0.0, 0.0]) * G,
        "-X": np.array([-1.0, 0.0, 0.0]) * G,
        "+Y": np.array([0.0, 1.0, 0.0]) * G,
        "-Y": np.array([0.0, -1.0, 0.0]) * G,
        "+Z": np.array([0.0, 0.0, 1.0]) * G,
        "-Z": np.array([0.0, 0.0, -1.0]) * G,
    }

    A_rows, b_rows = [], []
    face_stats = {}

    for face_name, true_vec in face_map.items():
        sub = df[(df["pose"] == face_name) & (df["mpu_ok"] == 1)]
        if sub.empty:
            print(f"WARNING: no accelerometer data for face {face_name}")
            continue
        m = sub[["ax_mps2", "ay_mps2", "az_mps2"]].to_numpy(dtype=float)
        face_stats[face_name] = {
            "mean": m.mean(axis=0).tolist(),
            "std": m.std(axis=0).tolist(),
            "n": int(len(m)),
        }
        # For each row m_i: m_i = S @ t + b
        # Unknowns: S[0,:], S[1,:], S[2,:] (9) + b (3) = 12
        for row in m:
            A_rows.append([true_vec[0], true_vec[1], true_vec[2], 1.0,
                           0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0])
            b_rows.append(row[0])
            A_rows.append([0.0, 0.0, 0.0, 0.0,
                           true_vec[0], true_vec[1], true_vec[2], 1.0,
                           0.0, 0.0, 0.0, 0.0])
            b_rows.append(row[1])
            A_rows.append([0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0,
                           true_vec[0], true_vec[1], true_vec[2], 1.0])
            b_rows.append(row[2])

    A = np.array(A_rows, dtype=float)
    b = np.array(b_rows, dtype=float)
    x, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)

    S = np.array([[x[0], x[1], x[2]],
                  [x[4], x[5], x[6]],
                  [x[8], x[9], x[10]]])
    b_vec = np.array([x[3], x[7], x[11]])

    # Residuals
    m_all = df[df["pose"].isin(face_map) & (df["mpu_ok"] == 1)][
        ["ax_mps2", "ay_mps2", "az_mps2"]].to_numpy(dtype=float)
    t_all = np.array([face_map[row["pose"]]
                      for _, row in df[df["pose"].isin(face_map) & (df["mpu_ok"] == 1)].iterrows()],
                     dtype=float)
    m_pred = (S @ t_all.T).T + b_vec
    resid = m_all - m_pred
    rms = float(np.sqrt(np.mean(resid ** 2)))
    max_err = float(np.max(np.abs(resid)))

    return {
        "accel_scale": S.tolist(),
        "accel_bias": b_vec.tolist(),
        "accel_rms_mps2": rms,
        "accel_max_err_mps2": max_err,
        "n_samples": int(len(m_all)),
        "face_stats": face_stats,
    }


# ===========================================================================
# MAGNETOMETER CALIBRATION (ellipsoid fit with outlier rejection)
# ===========================================================================

def mag_calibration(df: pd.DataFrame) -> dict:
    """
    Magnetometer ellipsoid calibration.

    Model:  (m - h)^T @ M @ (m - h) = 1
    h = hard-iron offset (uT), M = soft-iron matrix (3x3 symmetric PD).

    Rejects outliers where field magnitude is outside [20, 100] uT.
    Falls back to diagonal (axis-aligned) calibration if full ellipsoid fit
    is degenerate.
    """
    sub = df[(df["mag_stage"].isin(["YAW", "PITCH", "ROLL", "TUMBLE"])) &
             (df["mag_ok"] == 1) & (df["mag_new"] == 1)]
    if sub.empty:
        raise ValueError("No magnetometer data found in capture")

    m = sub[["mx_uT", "my_uT", "mz_uT"]].to_numpy(dtype=float)

    # Reject outliers: Earth's field is ~20-100 uT
    radii = np.linalg.norm(m, axis=1)
    mask = (radii > 20) & (radii < 100)
    m = m[mask]
    print(f"  MAG samples after outlier rejection: {len(m)} (removed {len(sub) - len(m)})")

    if len(m) < 100:
        raise ValueError(f"Not enough magnetometer samples: {len(m)}")

    n = len(m)
    x, y, z = m[:, 0], m[:, 1], m[:, 2]

    # ---- Try full ellipsoid fit ----
    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        x, y, z,
        np.ones(n),
    ])
    ones = np.ones(n)

    theta, _, rank, _ = np.linalg.lstsq(D, ones, rcond=None)

    A_xx, A_yy, A_zz, A_xy, A_xz, A_yz, b_x, b_y, b_z, c = theta

    M = np.array([[A_xx, A_xy, A_xz],
                  [A_xy, A_yy, A_yz],
                  [A_xz, A_yz, A_zz]])
    b_vec = np.array([b_x, b_y, b_z])

    eigvals, eigvecs = eigh(M)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    # Check if M is positive definite (non-degenerate ellipsoid)
    if np.all(eigvals > 1e-10):
        M_inv = np.linalg.inv(M)
        h = -0.5 * M_inv @ b_vec

        eigvals_safe = np.maximum(eigvals, 1e-12)
        soft_gains = 1.0 / np.sqrt(eigvals_safe)
        soft_matrix = (eigvecs * soft_gains) @ eigvecs.T

        m_corr = (m - h) @ soft_matrix.T
        radii_corr = np.linalg.norm(m_corr, axis=1)

        result = {
            "mag_hard_iron_uT": h.tolist(),
            "mag_soft_iron_matrix": M.tolist(),
            "mag_soft_iron_gains": soft_gains.tolist(),
            "mag_soft_iron_axes": eigvecs.tolist(),
            "mag_radius_mean_uT": float(np.mean(radii_corr)),
            "mag_radius_std_uT": float(np.std(radii_corr)),
            "mag_radius_min_uT": float(np.min(radii_corr)),
            "mag_radius_max_uT": float(np.max(radii_corr)),
            "mag_raw_radius_mean_uT": float(np.mean(radii)),
            "mag_raw_radius_std_uT": float(np.std(radii)),
            "mag_c_constraint": float(h @ M @ h - 1.0),
            "n_samples": int(n),
            "fit_method": "ellipsoid",
        }
    else:
        # ---- Fallback: diagonal (axis-aligned) calibration ----
        print("  WARNING: Full ellipsoid fit degenerate - using diagonal calibration")
        h = np.array([(m[:, 0].max() + m[:, 0].min()) / 2,
                      (m[:, 1].max() + m[:, 1].min()) / 2,
                      (m[:, 2].max() + m[:, 2].min()) / 2])

        # Diagonal soft-iron: scale each axis to unit range
        ranges = np.array([m[:, i].max() - m[:, i].min() for i in range(3)])
        mean_range = ranges.mean()
        soft_gains = mean_range / ranges
        soft_matrix = np.diag(soft_gains)

        m_corr = (m - h) @ soft_matrix.T
        radii_corr = np.linalg.norm(m_corr, axis=1)

        M = np.diag(1.0 / (ranges / 2) ** 2)  # Approximate

        result = {
            "mag_hard_iron_uT": h.tolist(),
            "mag_soft_iron_matrix": M.tolist(),
            "mag_soft_iron_gains": soft_gains.tolist(),
            "mag_soft_iron_axes": np.eye(3).tolist(),
            "mag_radius_mean_uT": float(np.mean(radii_corr)),
            "mag_radius_std_uT": float(np.std(radii_corr)),
            "mag_radius_min_uT": float(np.min(radii_corr)),
            "mag_radius_max_uT": float(np.max(radii_corr)),
            "mag_raw_radius_mean_uT": float(np.mean(radii)),
            "mag_raw_radius_std_uT": float(np.std(radii)),
            "mag_c_constraint": float(h @ M @ h - 1.0),
            "n_samples": int(n),
            "fit_method": "diagonal",
        }

    return result


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_magnetometer(m_raw: np.ndarray, m_corr: np.ndarray,
                      out_png: Path):
    """Create 3D ellipsoid/sphere plots."""
    fig = plt.figure(figsize=(14, 6))

    ax1 = fig.add_subplot(121, projection="3d")
    ax1.scatter(m_raw[:, 0], m_raw[:, 1], m_raw[:, 2],
                c="red", s=1, alpha=0.4, label="raw")
    ax1.set_xlabel("X (µT)")
    ax1.set_ylabel("Y (µT)")
    ax1.set_zlabel("Z (µT)")
    ax1.set_title("Raw Magnetometer")
    ax1.legend()

    # Reference ellipsoid
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    r = max(np.abs(m_raw).max(), 50)
    xs = r * np.outer(np.cos(u), np.sin(v))
    ys = r * np.outer(np.sin(u), np.sin(v))
    zs = r * np.outer(np.ones_like(u), np.cos(v))
    ax1.plot_wireframe(xs, ys, zs, color="gray", alpha=0.2, linewidth=0.5)

    ax2 = fig.add_subplot(122, projection="3d")
    ax2.scatter(m_corr[:, 0], m_corr[:, 1], m_corr[:, 2],
                c="green", s=1, alpha=0.4, label="corrected")
    ax2.set_xlabel("X (µT)")
    ax2.set_ylabel("Y (µT)")
    ax2.set_zlabel("Z (µT)")
    ax2.set_title("Corrected Magnetometer")
    ax2.legend()

    us = np.outer(np.cos(u), np.sin(v))
    vs = np.outer(np.sin(u), np.sin(v))
    ws = np.outer(np.ones_like(u), np.cos(v))
    ax2.plot_wireframe(us, vs, ws, color="gray", alpha=0.2, linewidth=0.5)

    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close()
    print(f"Saved magnetometer plot: {out_png}")


def plot_accelerometer(df: pd.DataFrame, out_png: Path):
    """Plot accelerometer data per face."""
    face_map = {"+X": [1, 0, 0], "-X": [-1, 0, 0],
                "+Y": [0, 1, 0], "-Y": [0, -1, 0],
                "+Z": [0, 0, 1], "-Z": [0, 0, -1]}

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    axes = axes.flatten()

    for ax, (face, true_vec) in zip(axes, face_map.items()):
        sub = df[(df["pose"] == face) & (df["mpu_ok"] == 1)]
        if sub.empty:
            ax.set_title(f"{face} (no data)")
            continue
        m = sub[["ax_mps2", "ay_mps2", "az_mps2"]].to_numpy(dtype=float)
        ax.scatter(m[:, 0], m[:, 1], m[:, 2], c="blue", s=2, alpha=0.5)
        ax.set_xlabel("ax")
        ax.set_ylabel("ay")
        ax.set_zlabel("az")
        ax.set_title(f"{face} (true={[round(v,2) for v in true_vec]})")
        ax.set_box_aspect([1,1,1])

    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close()
    print(f"Saved accelerometer plot: {out_png}")


def main():
    parser = argparse.ArgumentParser(description="IntelQCopter sensor calibration analysis")
    parser.add_argument("csv", help="Path to calibration CSV file")
    parser.add_argument("--output", "-o", default=None,
                        help="Output calibration JSON path")
    parser.add_argument("--plot", "-p", default=None,
                        help="Output plot PNG path prefix")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: CSV file not found: {csv_path}")
        sys.exit(1)

    print(f"Loading capture: {csv_path}")
    df = load_capture(csv_path)
    print(f"  Total rows: {len(df)}")
    print(f"  Poses: {df['pose'].value_counts().to_dict()}")
    print(f"  Mag stages: {df['mag_stage'].value_counts().to_dict()}")

    print("\n=== Accelerometer Calibration ===")
    accel_res = accel_calibration(df)
    print(json.dumps(accel_res, indent=2))

    print("\n=== Magnetometer Calibration ===")
    mag_res = mag_calibration(df)
    print(json.dumps(mag_res, indent=2))

    output = {"accelerometer": accel_res, "magnetometer": mag_res}

    if args.output:
        Path(args.output).write_text(json.dumps(output, indent=2))
        print(f"\nSaved calibration JSON: {args.output}")

    if args.plot:
        plot_prefix = Path(args.plot)
        mag_sub = df[(df["mag_stage"].isin(["YAW", "PITCH", "ROLL", "TUMBLE"])) &
                     (df["mag_ok"] == 1) & (df["mag_new"] == 1)]
        m_raw = mag_sub[["mx_uT", "my_uT", "mz_uT"]].to_numpy(dtype=float)
        radii = np.linalg.norm(m_raw, axis=1)
        m_raw = m_raw[(radii > 20) & (radii < 100)]

        h = np.array(mag_res["mag_hard_iron_uT"])
        soft = np.array(mag_res["mag_soft_iron_matrix"])
        m_corr = (m_raw - h) @ soft.T

        plot_magnetometer(m_raw, m_corr, plot_prefix.with_suffix(".mag.png"))
        plot_accelerometer(df, plot_prefix.with_suffix(".accel.png"))
        print(f"Saved plots: {plot_prefix}*.png")


def load_capture(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path, comment="#")
    df.columns = [c.strip() for c in df.columns]
    return df


if __name__ == "__main__":
    main()
