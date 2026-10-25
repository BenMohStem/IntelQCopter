"""Read-only analysis of a complete POSE1000_V1 session. Never connect or upload."""
import argparse
import hashlib
import json
from pathlib import Path
from statistics import NormalDist
import sys

import record_poses as recorder

G = 9.80665
POSES = ("Zplus", "Zminus", "Xplus", "Xminus", "Yplus", "Yminus")


def label(repeat, pose):
    return f"repeat-{repeat}_{pose}"


def target(pose):
    vector = np.zeros(3)
    vector["XYZ".index(pose[0])] = G if pose.endswith("plus") else -G
    return vector


def read_session(folder):
    manifest = json.loads((folder / "session.json").read_text(encoding="utf-8"))
    expected = [label(repeat, pose) for repeat in (1, 2, 3) for pose in POSES]
    if sorted(manifest["completed"]) != sorted(expected):
        raise ValueError("Need exactly the 18 completed poses; never infer missing measurements")
    result = {}
    for name in expected:
        attempts = []
        for path in (folder / name).glob("*/summary.json"):
            summary = json.loads(path.read_text(encoding="utf-8"))
            if summary["status"] == "CAPTURE_COMPLETE_REVIEW_REQUIRED":
                attempts.append((path, summary))
        if len(attempts) != 1:
            raise ValueError(f"{name}: expected one completed attempt; review ambiguous repeats")
        path, summary = attempts[0]
        raw = (path.parent / "serial.log").read_bytes()
        if hashlib.sha256(raw).hexdigest() != summary["serial_sha256"]:
            raise ValueError(f"{name}: original log SHA mismatch")
        # Ignore pre-query noise only, exactly as the recorder's synchronization does.
        lines = raw.splitlines()
        start = next(i for i, line in enumerate(lines) if line.endswith(b"QUERY_REPLY"))
        lines = [line.decode("ascii") for line in lines[start + 1:]]
        begin = lines.index("BEGIN,20000")
        header = lines[:begin]
        if header != manifest["reference_configuration"]:
            raise ValueError(f"{name}: board/configuration/baseline mismatch")
        if f"HELLO,POSE1000_V1,{manifest['source_sha256']}" not in header or recorder.CONFIG not in header:
            raise ValueError(f"{name}: source/configuration identity mismatch")
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
                raise ValueError(f"Unexpected post-BEGIN data in {name}")
        check = samples.finish(ending or "", means)
        physical = np.asarray(rows, dtype=float) * np.asarray(recorder.SCALES)
        physical[:, 3] += 36.53
        result[name] = {"data": physical, "rate": check["observed_average_hz"],
                        "sha256": summary["serial_sha256"], "maximum_fifo": check["maximum_fifo_bytes"]}
    return manifest, result


def fit_calibration(measurements):
    # Forward model: measured sensor vector = bias + A * true body acceleration.
    # Fit repeats 1-2 only. Equal pose weights avoid pretending placement error
    # disappears when the number of high-rate samples is large.
    names = [label(repeat, pose) for repeat in (1, 2) for pose in POSES]
    design = np.array([np.r_[1.0, target(name.split("_")[1])] for name in names])
    observed = np.array([measurements[name]["data"][:, :3].mean(axis=0) for name in names])
    parameters, _, rank, _ = np.linalg.lstsq(design, observed, rcond=None)
    response = parameters[1:].T
    if rank != 4 or np.linalg.cond(response) > 100:
        raise ValueError("Degenerate calibration geometry")
    return parameters[0], np.linalg.inv(response), response


def noise_statistics(data):
    centred = data - data.mean(axis=0)
    covariance = np.cov(data, rowvar=False, ddof=1)
    deviation = np.sqrt(np.diag(covariance))
    if np.any(deviation <= 0):
        raise ValueError("Zero-variance channel; investigate data")
    standardized = centred / deviation
    fft = np.fft.rfft(centred, n=2 * len(data), axis=0)
    acf = np.fft.irfft(fft * fft.conj(), n=2 * len(data), axis=0)[:101]
    acf /= acf[0].copy()
    directions = np.array([[1, 1, 1], [1, -1, 1], [-1, 1, 1], [1, 1, -1]], dtype=float)
    projection_diagnostics = []
    for direction in directions:
        projection = centred[:, :3] @ direction
        projection /= projection.std(ddof=1)
        projection_diagnostics.append({"direction": direction.tolist(),
            "skewness": float(np.mean(projection ** 3)),
            "excess_kurtosis": float(np.mean(projection ** 4) - 3)})
    return {"covariance": covariance.tolist(),
            "correlation": (covariance / np.outer(deviation, deviation)).tolist(),
            "std": deviation.tolist(),
            "skewness": np.mean(standardized ** 3, axis=0).tolist(),
            "excess_kurtosis": (np.mean(standardized ** 4, axis=0) - 3).tolist(),
            "acf_lag1": acf[1].tolist(), "acf_lag10": acf[10].tolist(),
            "acf_lag100": acf[100].tolist(), "accel_projection_checks": projection_diagnostics}, acf


def evaluate(measurements, bias, correction):
    records = []
    curves = {}
    for name, item in measurements.items():
        data = item["data"]
        corrected = (data[:, :3] - bias) @ correction.T
        desired = target(name.split("_")[1])
        mean = corrected.mean(axis=0)
        angle = np.degrees(np.arccos(np.clip(mean @ desired / (np.linalg.norm(mean) * G), -1, 1)))
        sensors = data[:, [0, 1, 2, 4, 5, 6]]
        noise, acf = noise_statistics(sensors)
        curves[name] = acf
        t = np.arange(len(data)) / item["rate"]
        tc = t - t.mean()
        slopes = tc @ (sensors - sensors.mean(axis=0)) / (tc @ tc)
        records.append({"pose": name, "split": "validation" if name.startswith("repeat-3_") else "training",
            "samples": len(data), "original_sha256": item["sha256"], "rate_hz": item["rate"],
            "maximum_fifo_bytes": item["maximum_fifo"],
            "sensor_accel_mean_m_s2": data[:, :3].mean(axis=0).tolist(),
            "sensor_gyro_mean_rad_s": data[:, 4:].mean(axis=0).tolist(),
            "temperature_mean_C": float(data[:, 3].mean()),
            "temperature_range_C": [float(data[:, 3].min()), float(data[:, 3].max())],
            "replay_corrected_mean_m_s2": mean.tolist(),
            "vector_error_m_s2": float(np.linalg.norm(mean - desired)),
            "direction_error_deg": float(angle), "magnitude_error_m_s2": float(np.linalg.norm(mean) - G),
            "linear_drift_per_second": slopes.tolist(), "sensor_noise": noise,
            "replay_corrected_accel_covariance": (correction @ np.array(noise["covariance"])[:3, :3] @ correction.T).tolist()})
    return records, curves


def figures(folder, measurements, records, curves):
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.2})
    validation = [row for row in records if row["split"] == "validation"]
    names = [row["pose"].split("_")[1] for row in validation]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    axes[0, 0].bar(names, [r["direction_error_deg"] for r in validation], color="#df8344")
    axes[0, 0].set(title="Reserved repetition 3: direction error", ylabel="Degrees vs labelled reference")
    axes[0, 1].bar(names, [r["vector_error_m_s2"] for r in validation], color="#398aa4")
    axes[0, 1].set(title="Reserved repetition 3: vector error", ylabel="m/s²")
    axes[1, 0].plot(range(1, 19), [r["temperature_mean_C"] for r in records], "o-")
    axes[1, 0].set(title="Temperature differs between recordings", xlabel="Recording number", ylabel="°C")
    for axis in range(3):
        axes[1, 1].plot(range(1, 19), [r["sensor_gyro_mean_rad_s"][axis] for r in records], "o-", label="XYZ"[axis])
    axes[1, 1].set(title="Stationary sensor gyro means: not a scale calibration", xlabel="Recording number", ylabel="rad/s")
    axes[1, 1].legend()
    fig.suptitle("Real 18-pose logs | correction is OFFLINE REPLAY, not post-upload hardware")
    fig.savefig(folder / "calibration.png", dpi=160)
    plt.close(fig)

    representative = label(2, "Zplus")
    data = measurements[representative]["data"][:, [0, 1, 2, 4, 5, 6]]
    standardized = (data - data.mean(axis=0)) / data.std(axis=0, ddof=1)
    probabilities = np.linspace(0.0005, 0.9995, 400)
    normal = np.array([NormalDist().inv_cdf(p) for p in probabilities])
    fig, axes = plt.subplots(2, 3, figsize=(12, 8), layout="constrained")
    channels = ("accel X", "accel Y", "accel Z", "gyro X", "gyro Y", "gyro Z")
    for index, axis in enumerate(axes.flat):
        axis.plot(normal, np.quantile(standardized[:, index], probabilities), ".", markersize=3)
        axis.plot(normal, normal, "--", color="grey")
        axis.set(title=channels[index], xlabel="Theoretical normal quantile", ylabel="Recorded standardized quantile")
    fig.suptitle(f"Real {representative}: marginal Q-Q checks | not proof of joint Gaussianity")
    fig.savefig(folder / "noise-qq.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    for index, channel in enumerate(channels):
        axes[0].plot(np.arange(101) * 1000 / measurements[representative]["rate"], curves[representative][:, index], label=channel)
    axes[0].set(title="Temporal correlation within a pose", xlabel="Lag (ms, inferred FIFO time)", ylabel="Normalized autocorrelation")
    axes[0].legend(fontsize=8)
    row = next(r for r in records if r["pose"] == representative)
    im = axes[1].imshow(row["sensor_noise"]["correlation"], vmin=-1, vmax=1, cmap="coolwarm")
    axes[1].set_xticks(range(6), channels, rotation=60, ha="right")
    axes[1].set_yticks(range(6), channels)
    axes[1].set_title("Cross-channel correlation at the same sample")
    axes[1].grid(False)
    fig.colorbar(im, ax=axes[1], label="Correlation")
    fig.suptitle(f"Real {representative}: spatial and temporal dependence are different")
    fig.savefig(folder / "correlation.png", dpi=160)
    plt.close(fig)

    worst = max(records, key=lambda r: max(r["sensor_noise"]["excess_kurtosis"]))
    channel = int(np.argmax(worst["sensor_noise"]["excess_kurtosis"]))
    original = measurements[worst["pose"]]["data"][:, [0, 1, 2, 4, 5, 6]][:, channel]
    residual = original - original.mean()
    standardized = residual / residual.std(ddof=1)
    probabilities = (np.arange(len(original)) + 0.5) / len(original)
    normal = np.array([NormalDist().inv_cdf(p) for p in probabilities])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    axes[0].plot(np.arange(len(original)) / measurements[worst["pose"]]["rate"], residual, linewidth=0.5)
    axes[0].set(title="Every recorded sample retained", xlabel="Seconds (inferred FIFO time)", ylabel="Deviation from pose mean (m/s² or rad/s)")
    axes[1].plot(normal, np.sort(standardized), ".", markersize=1.5)
    axes[1].plot(normal, normal, "--", color="grey")
    axes[1].set(title="Tails depart from a Gaussian model", xlabel="Theoretical normal quantile", ylabel="Recorded standardized quantile")
    fig.suptitle(f"Transient example: {worst['pose']} / {channels[channel]} | no samples removed")
    fig.savefig(folder / "transients.png", dpi=160)
    plt.close(fig)


def write_report(folder, records, bias, correction):
    validation = [r for r in records if r["split"] == "validation"]
    text = ["# Eighteen-pose calibration and noise analysis", "",
        "Engine: Python/NumPy/Matplotlib offline analysis of original real MPU6050 FIFO logs from ESP32-S3. No simulator, serial connection or upload. Raw console logs remain local; these are derived results.", "",
        "All 18 log hashes, sample sequences, firmware/configuration identities, checksums and final averages were rechecked. A complete capture is not physical calibration acceptance. Actual reference angles, fixture accuracy, mounting rigidity, gyro scaling/axis alignment and physical offset-register correction are not independently validated.", "",
        "## Candidate calibration", "",
        "Fit repeats 1-2 only, with equal weights for the 12 pose means. Model: r = b + A a; correction: a = inverse(A) (r - b). Assumed gravity magnitude is 9.80665 m/s² (standard gravity, not an independently measured local value). The pose labels are treated as exact gravity directions for this provisional fit. Equal weights do not imply equal or zero reference uncertainty. No extra level-only rotation was fitted.", "",
        "```text", f"bias (sensor m/s²): {bias}", f"correction matrix:\n{correction}", "```", "",
        "Repetition 3 was reserved before this fit. These offline replay errors include reference/placement error; they are not certified sensor accuracy.", "",
        "| Reserved pose | Direction error (deg) | Vector error (m/s²) |", "|---|---:|---:|"]
    for row in validation:
        text.append(f"| {row['pose']} | {row['direction_error_deg']:.4f} | {row['vector_error_m_s2']:.5f} |")
    text += ["", "![Calibration replay and temperature](calibration.png)", "",
        "## Noise and Gaussian approximation", "",
        "Covariance is calculated separately within each pose, on SENSOR accel XYZ (m/s²) and gyro XYZ (rad/s), with N-1. Off-diagonal entries are retained. Mixed accel/gyro covariance entries have mixed units; correlation is dimensionless. No pooled distribution of different pose means is called noise.", "",
        "The JSON contains per-pose covariance/correlation, skewness, excess kurtosis, selected autocorrelations, linear drift slopes and several acceleration projection diagnostics. Gaussian reference skewness/excess kurtosis are zero, but finite-sample values and temporal correlation matter. No IID normality p-value or independence claim is made. Projection and marginal checks are diagnostics, not a proof of joint Gaussianity.", "",
        "![Marginal Q-Q checks](noise-qq.png)", "", "![Temporal and cross-channel correlations](correlation.png)", "",
        "The curves use a representative second-repetition Z+ record; all 18 per-pose statistics remain in analysis.json. No detrending, outlier deletion or temperature compensation is applied. Drift, vibration and positioning effects therefore remain visible. Gyro means are stationary bias observations, not a validated universal bias or rate-scale calibration.", "",
        "## Transients prevent a blanket Gaussian assumption", "",
        f"The largest observed channel excess kurtosis is {max(max(r['sensor_noise']['excess_kurtosis']) for r in records):.3f}, whereas a Gaussian population has zero excess kurtosis. This is a descriptive diagnostic, not an IID hypothesis test. A quiet-looking marginal Q-Q plot cannot justify the same distribution for every recording or establish joint Gaussianity.", "",
        "![All samples and tail behavior in a transient record](transients.png)", "",
        "## Next physical step", "",
        "Validate the frozen candidate on independently measured static tilted orientations. Start with +30/-30 degree roll and pitch, then combined tilts. Record actual reference angles, their uncertainty and the unchanged mounting. Keep these records out of the fit initially. A separate 2-3 minute stationary log can then improve short-term correlation/drift diagnostics; longer records are needed for long-term Allan/bias characterization.", "",
        "Do not install the candidate as accepted calibration, enable motors or progress to PID yet. Current firmware remains the raw recorder. After tilt review, implement bias restoration and matrix correction as separate small lessons, followed by reset/power-cycle verification and remeasurement. A full calibration matrix cannot be represented by bias registers alone.", "",
        "Sources: [NXP AN4399](https://www.nxp.com/docs/en/application-note/AN4399.pdf), [NIST normal probability plots](https://www.itl.nist.gov/div898/handbook/eda/section3/normprpl.htm), [NIST measurement assumptions](https://www.itl.nist.gov/div898/handbook/mpc/section1/mpc121.htm). Sources support the method, not the accuracy of this hardware.", ""]
    (folder / "report.md").write_text("\n".join(text), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--deps", type=Path, help="Optional existing directory containing NumPy and Matplotlib")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("Choose a new output folder; preserve earlier analysis attempts")
    if args.deps:
        sys.path.insert(0, str(args.deps))
    global np, plt
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    manifest, measurements = read_session(args.session)
    bias, correction, response = fit_calibration(measurements)
    records, curves = evaluate(measurements, bias, correction)
    args.output.mkdir(parents=True)
    report = {"engine": "Offline analysis of real ESP32-S3/MPU6050 FIFO recordings; no simulator",
        "source_sha256": manifest["source_sha256"], "samples": sum(r["samples"] for r in records),
        "uploaded": False, "new_hardware_test": False, "calibration_acceptance": "PENDING_INDEPENDENT_TILT_VALIDATION",
        "training_repetitions": [1, 2], "validation_repetitions": [3],
        "bias_sensor_m_s2": bias.tolist(), "correction_sensor_to_body": correction.tolist(),
        "forward_response": response.tolist(), "response_condition_number": float(np.linalg.cond(response)),
        "noise_channel_order": ["ax", "ay", "az", "gx", "gy", "gz"], "records": records}
    (args.output / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    figures(args.output, measurements, records, curves)
    write_report(args.output, records, bias, correction)
    print(f"Verified {len(records)} recordings and {report['samples']} samples. Report: {args.output / 'report.md'}")


if __name__ == "__main__":
    main()
