"""Verify an actual ACCEL_XYZ_V1 recording and report observed behavior without refitting."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import record_accel as recorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--condition", choices=("stationary_unlevel", "stationary_level", "moving", "unknown"), default="unknown",
                        help="Operator-reported condition, not inferred from the sensor")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a fresh output directory to preserve previous reports")
    summary = json.loads((args.session / "summary.json").read_text(encoding="utf-8"))
    if summary["status"] != "CAPTURE_COMPLETE_REVIEW_REQUIRED":
        raise ValueError("Recording is incomplete; inspect original failure evidence")
    if args.condition == "unknown":
        args.condition = summary.get("operator_condition", "unknown")
    reference_description = {
        "stationary_unlevel": "Motionless on a support; not independently checked level. The indicated tilt cannot be identified as calibration error from this pose.",
        "stationary_level": "Operator reports independently checked level. Reference uncertainty must be reviewed before judging directional accuracy.",
        "moving": "Operator reports movement. Specific force contains real acceleration; a static tilt interpretation is not generally valid.",
        "unknown": "Physical reference condition is not documented; no angular-accuracy judgment is possible."
    }[args.condition]
    raw = (args.session / "serial.log").read_bytes()
    if hashlib.sha256(raw).hexdigest() != summary["serial_sha256"]:
        raise ValueError("Original serial log SHA mismatch")
    lines = raw.splitlines()
    start = next(i for i, line in enumerate(lines) if line.endswith(b"QUERY_REPLY"))
    lines = [line.decode("ascii") for line in lines[start + 1:]]
    ready = lines.index("READY,ACCEL_XYZ_V1")
    header = lines[:ready + 1]
    if header != summary["header"]:
        raise ValueError("Original header differs from the saved summary")
    fit = json.loads((recorder.ROOT / "measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json").read_text(encoding="utf-8"))
    profile = recorder.parse_profile(header, summary["source_sha256"], fit)
    samples = [recorder.validate_sample(line, profile) for line in lines[ready + 1:]]
    saved = json.loads((args.session / "samples.json").read_text(encoding="utf-8"))["samples"]
    if samples != saved or len(samples) != summary["samples"]:
        raise ValueError("Saved samples/count differ from the original stream")
    ms = np.array([row["ms"] for row in samples], dtype=np.int64)
    intervals = np.diff(ms) & 0xFFFFFFFF
    if np.any(intervals <= 0) or np.any(intervals >= 2000):
        raise ValueError("Reset, duplicate timestamp or long reporting gap")
    t = np.r_[0, np.cumsum(intervals)] / 1000
    body = np.array([row["drone_m_s2"] for row in samples])
    counts = np.array([row["counts_after_mpu_bias"] for row in samples])
    centred = counts * 9.80665 / 4096 - np.array(profile["remaining_bias"])
    independently_computed = centred @ np.array(profile["matrix"]).T
    maximum_error = float(np.max(np.abs(body - independently_computed)))
    mean = body.mean(axis=0)
    norm = np.linalg.norm(body, axis=1)
    rate = (len(body) - 1) / t[-1]
    if not np.allclose(mean, summary["mean_drone_m_s2"], atol=1e-12, rtol=0):
        raise ValueError("Original mean differs from the saved summary")
    tilt = math.degrees(math.atan2(math.hypot(mean[0], mean[1]), mean[2]))
    report = {"engine": "Offline NumPy/Matplotlib verification of actual MCU-emitted XYZ serial data; no simulator",
        "operator_condition": args.condition, "operator_note": summary.get("operator_note", ""), "original_serial_sha256": summary["serial_sha256"],
        "recorded_source_sha256": summary["source_sha256"],
        "source_matches_current_file": hashlib.sha256((recorder.ROOT / "firmware/main.cpp").read_bytes()).hexdigest() == summary["source_sha256"],
        "samples_verified": len(body), "duration_device_s": float(t[-1]), "observed_report_rate_hz": float(rate),
        "report_interval_min_max_ms": [int(intervals.min()), int(intervals.max())],
        "boot_offsets": profile["boot_offsets"], "applied_register_targets": [row[1] for row in profile["offsets"]],
        "matrix_consistency_max_error_m_s2": maximum_error,
        "mean_drone_m_s2": mean.tolist(), "std_drone_m_s2": body.std(axis=0, ddof=1).tolist(),
        "covariance_drone": np.cov(body, rowvar=False).tolist(),
        "mean_norm_m_s2": float(norm.mean()), "norm_minus_standard_g_m_s2": float(norm.mean() - 9.80665),
        "norm_difference_percent": float(100 * (norm.mean() / 9.80665 - 1)),
        "mean_vector_tilt_deg": tilt,
        "status": "One real full-XYZ implementation recording verified; absolute accuracy and repeated restart/power-cycle behavior not established",
        "firmware_changed": False, "refitted": False, "uploaded_by_assistant": False,
        "physical_angle_error_claimed": False, "reset_power_cycle_series": "PENDING_OPERATOR_RUNS"}
    args.output.mkdir(parents=True)
    (args.output / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), layout="constrained")
    colors = ("#2583a4", "#b36c31", "#398d65")
    for i, axis in enumerate(axes.flat):
        values = body[:, i] if i < 3 else norm
        axis.plot(t, values, linewidth=0.65, color=colors[i] if i < 3 else "#765a9e")
        axis.set(xlabel="Device elapsed time (s)", ylabel="m/s²", title=("Drone X", "Drone Y", "Drone Z", "Measured norm, not forced to g")[i])
        axis.grid(alpha=0.2)
        if i == 3:
            axis.axhline(9.80665, color="grey", linestyle="--", label="Standard g; not fitted")
            axis.legend(fontsize=8)
    fig.suptitle(f"Actual MCU-emitted XYZ | {len(body)} samples | operator: {args.condition} | no refit")
    fig.savefig(args.output / "actual-xyz.png", dpi=160)
    plt.close(fig)
    text = ["# First real full-XYZ recording review", "",
        "Engine: offline Python/NumPy/Matplotlib checks of real ESP32-S3/MPU6050 serial output. No simulator or synthetic samples are used in the figures. The user uploaded and ran the firmware; the assistant only reads the saved files and did not flash or open the serial port.", "",
        f"Verified **{len(body)} actual samples** spanning {t[-1]:.3f} device seconds at **{rate:.3f} reports/s**. Original-file SHA, source/configuration identity, coefficients, known boot states, sample timestamps, saved-sample equality and every reported XYZ computation were checked.", "",
        "## Observed register restoration and arithmetic", "",
        "| Axis | Observed word at initialization | Applied/verified target |", "|---|---:|---:|"]
    for i, axis in enumerate("XYZ"):
        text.append(f"| {axis} | {profile['boot_offsets'][i]} | {profile['offsets'][i][1]} |")
    text += ["", f"Maximum difference between actual MCU XYZ and independent double-precision recomputation from its ADC counts is **{maximum_error:.9f} m/s²**. This checks implementation/float rounding, not absolute sensor accuracy. The source verifies the register targets before reporting; this single recording is evidence of restoration in this run, not a repeated reset/power-cycle series. These writable register targets are absolute values, not cumulative corrections.", "",
        "## Operator reference and measured vector", "",
        f"Operator-reported condition: **{args.condition}**. {reference_description}", "",
        f"Mean drone-frame XYZ: **({mean[0]:+.6f}, {mean[1]:+.6f}, {mean[2]:+.6f}) m/s²**. Its direction is {tilt:.3f} degrees from body Z. This is an acceleration-derived static indication, not a validated attitude estimate or independent reference measurement.", "",
        f"Mean measured magnitude: **{norm.mean():.6f} m/s²**, about **{100*(norm.mean()/9.80665-1):+.3f}%** relative to assumed standard gravity 9.80665 m/s². Non-level orientation can explain the nonzero X/Y components, but does not by itself explain magnitude error. Scale/bias/reference/temperature/model effects remain possible; this one pose cannot identify their causes. No normalization to g, new level trim, bias refit or coefficient change was made.", "",
        "![All actual XYZ samples and measured norm](actual-xyz.png)", "",
        "Per-axis sample spread and covariance are in analysis.json. Samples may be time-correlated; this near-91 Hz register stream is not the original 1000 Hz FIFO noise characterization or a 500 Hz flight loop. No Gaussian, independence, Kalman-gain or flight-accuracy claim follows from this log.", "",
        "## Next: verify restoration across separate boots", "",
        "1. Reset the ESP32, then run record-accel again with the same firmware. Record the run as after reset.",
        "2. Power-cycle both ESP32 and MPU, then record another separate run. Record the run as after full power cycle.",
        "No reflash is needed. Preserve the fixed mounting. The reference can be any resting orientation; do not compare mean direction across different poses as if it were sensor drift. Known baseline or retained-target startup words are allowed; the final targets must stay -2348, 2993, 1212. Any unexpected startup word must stop/report rather than trigger another subtraction. Actual repeated-boot evidence is still pending.", "",
        "Motor outputs remain disabled. Physical correction response and absolute multi-orientation accuracy remain limited by the earlier reference quality; do not mark all calibration or flight readiness accepted from this one implementation check.", ""]
    (args.output / "report.md").write_text("\n".join(text), encoding="utf-8")
    print(f"Verified {len(body)} actual XYZ samples; max arithmetic difference {maximum_error:.9f} m/s^2.")
    print(f"Report: {args.output / 'report.md'}")


if __name__ == "__main__":
    main()
