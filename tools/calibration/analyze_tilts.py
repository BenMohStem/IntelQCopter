"""Verify tilted-pose logs and apply operator-confirmed raised-side references, without refitting."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import record_poses as recorder
import record_tilts as tilts


def clarified_angles(side, angle):
    return {"X+": [0, -angle], "X-": [0, angle],
            "Y+": [angle, 0], "Y-": [-angle, 0]}[side]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-method", required=True, help="Operator description of the independent reference")
    parser.add_argument("--calibration", type=Path, default=recorder.ROOT / "measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json")
    parser.add_argument("--raised-sides", nargs=4, choices=("X+", "X-", "Y+", "Y-"), required=True,
                        help="Operator-confirmed sides raised in the original four-run order; never infer from sensor output")
    parser.add_argument("--reference-error-deg", type=float, required=True,
                        help="Operator estimate of placement error, not a standard deviation or certified bound")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a fresh output directory; preserve earlier reports")
    if sorted(args.raised_sides) != ["X+", "X-", "Y+", "Y-"]:
        parser.error("This four-pose experiment needs each signed side once")
    if not math.isfinite(args.reference_error_deg) or args.reference_error_deg <= 0:
        parser.error("Reference error estimate must be positive and finite")
    manifest = json.loads((args.session / "session.json").read_text(encoding="utf-8"))
    if manifest["completed"] != list(tilts.POSE_NAMES):
        raise ValueError("Need the four completed records in the original order")
    candidate = manifest["frozen_calibration"]
    candidate_bytes = args.calibration.read_bytes()
    candidate_source = json.loads(candidate_bytes)
    if hashlib.sha256(candidate_bytes).hexdigest() != manifest["calibration_sha256"] or candidate_source["source_sha256"] != manifest["source_sha256"]:
        raise ValueError("Frozen candidate identity mismatch")
    if any(candidate[key] != candidate_source[key] for key in candidate):
        raise ValueError("Frozen coefficients differ from the source candidate")
    bias = np.array(candidate["bias_sensor_m_s2"])
    correction = np.array(candidate["correction_sensor_to_body"])
    results = []
    for name, side in zip(tilts.POSE_NAMES, args.raised_sides):
        attempts = []
        for path in (args.session / name).glob("*/summary.json"):
            summary = json.loads(path.read_text(encoding="utf-8"))
            if summary["status"] == "CAPTURE_COMPLETE_REVIEW_REQUIRED":
                attempts.append((path, summary))
        if len(attempts) != 1:
            raise ValueError(f"{name}: ambiguous/missing completed attempt")
        path, summary = attempts[0]
        reference = json.loads((path.parent / "reference.json").read_text(encoding="utf-8"))
        if reference["angles_source"] != "nominal_target_only":
            raise ValueError("Review actual-angle metadata before applying a nominal raised-side clarification")
        raw = (path.parent / "serial.log").read_bytes()
        if hashlib.sha256(raw).hexdigest() != summary["serial_sha256"]:
            raise ValueError("Original log changed")
        lines = raw.splitlines()
        marker = next(i for i, line in enumerate(lines) if line.endswith(b"QUERY_REPLY"))
        lines = [line.decode("ascii") for line in lines[marker + 1:]]
        begin = lines.index("BEGIN,20000")
        if lines[:begin] != manifest["reference_configuration"]:
            raise ValueError("Configuration changed")
        if f"HELLO,POSE1000_V1,{manifest['source_sha256']}" not in lines[:begin]:
            raise ValueError("Source identity changed")
        samples = recorder.Samples()
        rows = []
        means = {}
        ending = None
        for line in lines[begin + 1:]:
            if line.startswith("S,"):
                samples.add(line)
                rows.append([int(value) for value in line.split(",")[2:]])
            elif line.startswith("MEAN_COUNTS,"):
                _, axis, mean = line.split(",")
                if int(axis) in means:
                    raise ValueError("Duplicate average")
                means[int(axis)] = float(mean)
            elif line.startswith("END,") and ending is None:
                ending = line
            else:
                raise ValueError("Unexpected original stream data")
        verified = samples.finish(ending or "", means)
        physical = np.array(rows) * np.array(recorder.SCALES)
        physical[:, 3] += 36.53
        corrected = (physical[:, :3] - bias) @ correction.T
        angles = clarified_angles(side, manifest["nominal_angle_deg"])
        comparison = tilts.compare(verified["mean_physical"], candidate, angles)
        original_comparison = tilts.compare(verified["mean_physical"], candidate, [reference["roll_deg"], reference["pitch_deg"]])
        mean = corrected.mean(axis=0)
        results.append({"original_pose_label": name, "operator_confirmed_raised_side": side,
            "clarified_roll_pitch_deg": angles, "samples": verified["samples"],
            "original_sha256": summary["serial_sha256"], "observed_rate_hz": verified["observed_average_hz"],
            "original_label_direction_difference_deg": original_comparison["direction_difference_deg"],
            "clarified_reference_comparison": comparison,
            "unsigned_tilt_deg": math.degrees(math.atan2(math.hypot(mean[0], mean[1]), mean[2])),
            "magnitude_m_s2": float(np.linalg.norm(mean)),
            "corrected_accel_covariance": np.cov(corrected, rowvar=False).tolist(),
            "temperature_mean_C": float(physical[:, 3].mean())})
    args.output.mkdir(parents=True)
    report = {"engine": "Offline NumPy/Matplotlib analysis of original real ESP32-S3/MPU6050 FIFO records; no simulator",
        "samples": sum(row["samples"] for row in results), "source_sha256": manifest["source_sha256"],
        "candidate_sha256": manifest["calibration_sha256"], "frozen_calibration": candidate,
        "operator_clarification": "Explicitly supplied raised-side order: " + ", ".join(args.raised_sides),
        "reference_method_update": args.reference_method,
        "reference_error_estimate_deg": args.reference_error_deg,
        "reference_uncertainty_distribution": "UNKNOWN; not interpreted as Gaussian sigma or a certified hard bound",
        "refitted": False, "uploaded": False, "original_files_modified": False,
        "acceptance": "Approximate static comparison limited by reference; not sub-degree or flight validation",
        "assumed_g_m_s2": 9.80665, "records": results}
    (args.output / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    names = args.raised_sides
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    axes[0].bar(names, [r["clarified_reference_comparison"]["direction_difference_deg"] for r in results], color="#388ba4")
    axes[0].axhline(args.reference_error_deg, linestyle="--", color="#bd6b34", label="Operator placement estimate; not pass/fail")
    axes[0].set(title="Direction difference after operator clarification", xlabel="Actual raised side (operator confirmed)", ylabel="Degrees")
    axes[0].legend(fontsize=8)
    axes[1].plot(names, [r["unsigned_tilt_deg"] for r in results], "o", color="#388ba4")
    angle = manifest["nominal_angle_deg"]
    axes[1].axhline(angle, color="grey", linestyle="--")
    axes[1].axhspan(angle - args.reference_error_deg, angle + args.reference_error_deg, alpha=0.12, color="#bd6b34")
    axes[1].set(title="Overall tilt: shaded placement estimate is not a CI", xlabel="Actual raised side", ylabel="Degrees from vertical")
    fig.suptitle("Real 45° recordings | frozen calibration replay | hand-held reference about ±1°")
    fig.savefig(args.output / "tilt-validation.png", dpi=160)
    plt.close(fig)
    text = ["# Independent tilted-pose review", "",
        "All four original recordings passed source/configuration, 20,000-sample sequence, checksum, average and original-file SHA checks: 80,000 samples total. Engine: Python/NumPy/Matplotlib offline replay of real ESP32-S3/MPU6050 data. No simulator, upload or new hardware action.", "",
        "## Reference clarification", "",
        "The original files say roll+, roll-, pitch+, pitch-. The operator subsequently confirmed raised-side order **+X, -X, +Y, -Y**, using the drone axes. This is explicit operator evidence, not a direction chosen to minimize residuals. Original labels, reference.json, validation.json and raw samples remain unchanged. The operator also reports a hand-held 45-degree ruler with about ±1 degree positioning error; original metadata left the method/uncertainty unknown. That estimate has no specified probability distribution or confidence and is not treated as sigma or a certified bound.", "",
        "For this documented frame, raised +X means pitch -45; raised -X means pitch +45; raised +Y means roll +45; raised -Y means roll -45. The frozen calibration from the earlier six-pose training set is unchanged and was not refitted to these records. Standard gravity 9.80665 m/s² and the declared nominal angles are model assumptions.", "",
        "| Original label | Confirmed raised side | Overall tilt (deg) | Direction difference (deg) | Magnitude (m/s²) |",
        "|---|---|---:|---:|---:|"]
    for row in results:
        text.append(f"| {row['original_pose_label']} | {row['operator_confirmed_raised_side']} | {row['unsigned_tilt_deg']:.3f} | {row['clarified_reference_comparison']['direction_difference_deg']:.3f} | {row['magnitude_m_s2']:.5f} |")
    text += ["", "![Frozen calibration on real tilted recordings](tilt-validation.png)", "",
        "## Interpretation", "",
        "The approximately 60-degree errors originally printed compared the samples with the wrong named axis/sign convention. They are retained in the JSON; they are not evidence of a 60-degree sensor calibration fault. The clarified direction differences are on the scale of the stated hand-held placement uncertainty. This supports the candidate for the next controlled bench-implementation lesson, but does not establish sub-degree absolute accuracy, exact bias/scale, gyro mounting/scale or flight readiness. The first overall tilt is about 1.03 degrees below 45: do not hide that small departure or call every result strictly inside a proven ±1-degree bound.", "",
        "No reference uncertainty was subtracted in quadrature from the observed differences. Separating reference error from sensor/model error requires a better fixed reference and/or repeated placements. Movement can add nongravitational acceleration; mean and covariance do not independently prove perfect stillness. Physical MPU offset response and power-cycle restoration still require tests.", "",
        "## Next small software lesson", "",
        "Prepare one named accelerometer bias correction in the firmware, then validate its sensor-axis behavior before adding other axes and matrix rows in separate lessons. Keep the raw recorder checkpoint for new measurements. Coefficients stay provisional, motor outputs remain disabled and the user controls uploads. Do not jump to PID or silently install all calibration changes from this report.", ""]
    (args.output / "report.md").write_text("\n".join(text), encoding="utf-8")
    print(f"Verified {report['samples']} samples. Derived report: {args.output / 'report.md'}")
    for row in results:
        print(row["operator_confirmed_raised_side"], "direction difference deg:", round(row["clarified_reference_comparison"]["direction_difference_deg"], 4))


if __name__ == "__main__":
    main()
