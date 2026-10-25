"""Record four independent tilted poses using the existing 20,000-sample firmware."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import serial
import record_poses as recorder

ROOT = Path(__file__).resolve().parents[2]
POSE_NAMES = ("roll-plus", "roll-minus", "pitch-plus", "pitch-minus")


def reference_vector(roll_deg, pitch_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    return [-9.80665 * math.sin(pitch),
            9.80665 * math.sin(roll) * math.cos(pitch),
            9.80665 * math.cos(roll) * math.cos(pitch)]


def read_angles(text, nominal):
    if not text.strip():
        return nominal, "nominal_target_only"
    fields = text.split(",")
    if len(fields) != 2:
        raise ValueError("Enter roll,pitch in degrees, for example 45,0")
    angles = [float(value) for value in fields]
    if not all(math.isfinite(value) and abs(value) <= 90 for value in angles):
        raise ValueError("Angles must be finite and between -90 and +90 degrees")
    return angles, "operator_entered_reference"


def compare(mean, calibration, angles):
    bias = calibration["bias_sensor_m_s2"]
    matrix = calibration["correction_sensor_to_body"]
    centred = [mean[key] - bias[i] for i, key in enumerate(("ax", "ay", "az"))]
    corrected = [sum(row[j] * centred[j] for j in range(3)) for row in matrix]
    expected = reference_vector(*angles)
    magnitude = math.sqrt(sum(value * value for value in corrected))
    if magnitude <= 0 or not math.isfinite(magnitude):
        raise ValueError("Invalid corrected acceleration magnitude")
    cosine = sum(corrected[i] * expected[i] for i in range(3)) / (magnitude * 9.80665)
    return {"replay_corrected_mean_m_s2": corrected, "expected_reference_m_s2": expected,
            "direction_difference_deg": math.degrees(math.acos(max(-1, min(1, cosine)))),
            "vector_difference_m_s2": math.sqrt(sum((corrected[i] - expected[i]) ** 2 for i in range(3))),
            "note": "Offline application of frozen candidate; includes reference/mounting uncertainty. Not a post-upload calibration test."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--angle", type=float, default=45, help="Nominal roll/pitch angle in degrees (default 45)")
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--baseline-session", type=Path,
                        default=ROOT / "artifacts/pose-recordings/20261025T132707615977Z")
    parser.add_argument("--calibration", type=Path,
                        default=ROOT / "measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json")
    args = parser.parse_args(argv)
    if not math.isfinite(args.angle) or not 0 < args.angle < 90:
        parser.error("Use a nominal angle strictly between 0 and 90 degrees")
    baseline = json.loads((args.baseline_session / "session.json").read_text(encoding="utf-8"))
    calibration_bytes = args.calibration.read_bytes()
    calibration = json.loads(calibration_bytes)
    calibration_sha = hashlib.sha256(calibration_bytes).hexdigest()
    source_sha = hashlib.sha256((ROOT / "firmware/main.cpp").read_bytes()).hexdigest()
    if source_sha != baseline["source_sha256"] or source_sha != calibration["source_sha256"]:
        raise ValueError("Firmware differs from the calibration session. Do not mix datasets.")
    for row in calibration["records"]:
        if row["pose"] not in baseline["completed"]:
            raise ValueError("Calibration/baseline session mismatch")
        summaries = list((args.baseline_session / row["pose"]).glob("*/summary.json"))
        if not any(json.loads(p.read_text(encoding="utf-8")).get("serial_sha256") == row["original_sha256"] for p in summaries):
            raise ValueError("Calibration does not reference this baseline recording")
    if args.resume:
        folder = args.resume.resolve(strict=True)
        manifest = json.loads((folder / "session.json").read_text(encoding="utf-8"))
        if (manifest.get("protocol") != "TILT_VALIDATION_V1" or manifest["source_sha256"] != source_sha
                or manifest["calibration_sha256"] != calibration_sha or manifest["nominal_angle_deg"] != args.angle
                or manifest["reference_configuration"] != baseline["reference_configuration"]):
            raise ValueError("Session/candidate/configuration changed; start a new tilt session")
    else:
        folder = ROOT / "artifacts/tilt-recordings" / recorder.stamp()
        folder.mkdir(parents=True)
        method = input("Independent angle reference (e.g. 45-degree jig; Enter = unknown): ").strip()
        uncertainty_text = input("Reference angle uncertainty in degrees, if known (Enter = unknown): ").strip()
        uncertainty = float(uncertainty_text) if uncertainty_text else None
        if uncertainty is not None and (not math.isfinite(uncertainty) or uncertainty < 0):
            raise ValueError("Reference uncertainty must be finite and nonnegative")
        manifest = {"protocol": "TILT_VALIDATION_V1", "engine": "Real MPU6050 FIFO via ESP32-S3; no simulator",
            "source_sha256": source_sha, "calibration_sha256": calibration_sha,
            "frozen_calibration": {key: calibration[key] for key in ("bias_sensor_m_s2", "correction_sensor_to_body")},
            "nominal_angle_deg": args.angle, "reference_method": method or "UNKNOWN",
            "reference_uncertainty_deg": uncertainty, "reference_configuration": baseline["reference_configuration"],
            "completed": [], "hardware_acceptance": "UNREVIEWED"}
        recorder.save_json(folder / "session.json", manifest)
    print(f"Automatic recording folder: {folder}")
    print("20 seconds / 20000 samples per pose; progress every 1000 samples.")
    print("Keep FC mounting unchanged. Close Serial Monitor. Propellers removed; no motor outputs.")
    print("Signs from level: +roll lifts +Y side; -roll lifts -Y side.")
    print("+pitch lifts -X side; -pitch lifts +X side. Keep the other angle near zero.")
    device = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    device.port = args.port
    device.dtr = False
    device.rts = False
    try:
        input("Press Enter only if FC mounting is unchanged since the six-pose session: ")
        manifest["operator_confirmed_unchanged_mounting"] = True
        recorder.save_json(folder / "session.json", manifest)
        device.open()
        nominal_angles = ([args.angle, 0], [-args.angle, 0], [0, args.angle], [0, -args.angle])
        for name, nominal in zip(POSE_NAMES, nominal_angles):
            if name in manifest["completed"]:
                continue
            print(f"\n{name}: target roll={nominal[0]:g}, pitch={nominal[1]:g} degrees.")
            while True:
                try:
                    angles, source = read_angles(input("Position and check reference. Enter actual roll,pitch, or Enter for nominal: "), nominal)
                    break
                except ValueError as error:
                    print(error)
            attempt = folder / name / recorder.stamp()
            attempt.mkdir(parents=True)
            recorder.save_json(attempt / "reference.json", {"roll_deg": angles[0], "pitch_deg": angles[1],
                "angles_source": source, "reference_method": manifest["reference_method"],
                "reference_uncertainty_deg": manifest["reference_uncertainty_deg"],
                "independently_verified_by_software": False})
            print(f"Saving this attempt: {attempt}")
            result = recorder.capture(device, attempt, source_sha, manifest["reference_configuration"])
            comparison = compare(result["mean_physical"], manifest["frozen_calibration"], angles)
            comparison["angles_source"] = source
            comparison["acceptance"] = "UNREVIEWED"
            recorder.save_json(attempt / "validation.json", comparison)
            manifest["completed"].append(name)
            recorder.save_json(folder / "session.json", manifest)
            print(f"Saved: {result['samples']} samples at {result['observed_average_hz']:.2f} Hz.")
            print(f"Difference from declared reference: {comparison['direction_difference_deg']:.3f} deg (provisional).")
            if source == "nominal_target_only" or manifest["reference_method"] == "UNKNOWN":
                print("Reference not fully documented: this difference is indicative, not verified angle accuracy.")
        print("All four tilt recordings saved. Candidate unchanged; keep the folder for analysis.")
    finally:
        device.close()
        print(f'Resume: .\\qcopter.ps1 record-tilts --port {args.port} --angle {args.angle:g} --resume "{folder}" --baseline-session "{args.baseline_session}" --calibration "{args.calibration}"')


if __name__ == "__main__":
    main()
