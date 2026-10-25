"""Save live fully corrected XYZ acceleration in arbitrary orientations. Sends '?' only; never uploads."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import serial
import record_poses as recorder

ROOT = Path(__file__).resolve().parents[2]
CONFIG = "CONFIG,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=xyz_mpu_plus_matrix"
BASELINES = [-2074, 2979, 1074]


def register_profile(bias):
    decrements = []
    for value in bias:
        q = value * 1024 / 9.80665
        decrements.append(2 * (math.floor(q + 0.5) if q >= 0 else math.ceil(q - 0.5)))
    targets = [BASELINES[i] - decrements[i] for i in range(3)]
    residual = [bias[i] - decrements[i] * 9.80665 / 2048 for i in range(3)]
    return targets, residual


def parse_profile(header, expected_sha, fit):
    if f"HELLO,ACCEL_XYZ_V1,{expected_sha}" not in header or CONFIG not in header:
        raise ValueError("Current XYZ firmware/source/configuration does not match; upload bench first")
    if len([line for line in header if line.startswith("BOARD,")]) != 1:
        raise ValueError("Missing board identity")
    bias = [None] * 3
    matrix = [None] * 3
    offsets = [None] * 3
    boot = [None] * 3
    for line in header:
        fields = line.split(",")
        if fields[0] not in ("BIAS", "ROW", "OFFSET", "BOOT_OFFSET"):
            continue
        axis = int(fields[1])
        if not 0 <= axis < 3:
            raise ValueError("Invalid calibration axis")
        if fields[0] == "BOOT_OFFSET" and len(fields) == 3 and boot[axis] is None:
            boot[axis] = int(fields[2])
        elif fields[0] == "BIAS" and len(fields) == 3 and bias[axis] is None:
            bias[axis] = float(fields[2])
        elif fields[0] == "ROW" and len(fields) == 5 and matrix[axis] is None:
            matrix[axis] = [float(value) for value in fields[2:]]
        elif fields[0] == "OFFSET" and len(fields) == 5 and offsets[axis] is None:
            offsets[axis] = [int(fields[2]), int(fields[3]), float(fields[4])]
        else:
            raise ValueError("Malformed or duplicate calibration metadata")
    if any(value is None for value in bias + matrix + offsets + boot):
        raise ValueError("Incomplete calibration metadata")
    numbers = bias + [v for row in matrix for v in row] + [v for row in offsets for v in row]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("Nonfinite calibration")
    targets, remaining = register_profile(bias)
    for axis in range(3):
        if abs(bias[axis] - fit["bias_sensor_m_s2"][axis]) > 1e-7:
            raise ValueError("Firmware bias differs from the frozen candidate")
        if any(abs(matrix[axis][j] - fit["correction_sensor_to_body"][axis][j]) > 1e-7 for j in range(3)):
            raise ValueError("Firmware matrix differs from the frozen candidate")
        if boot[axis] not in (BASELINES[axis], targets[axis]):
            raise ValueError("Unexpected retained boot offset")
        if offsets[axis][:2] != [BASELINES[axis], targets[axis]] or abs(offsets[axis][2] - remaining[axis]) > 1e-6:
            raise ValueError("Wrong register target or double/missing bias compensation")
    return {"bias": bias, "matrix": matrix, "offsets": offsets, "boot_offsets": boot,
            "remaining_bias": [row[2] for row in offsets]}


def validate_sample(line, profile):
    fields = line.split(",")
    if len(fields) != 8 or fields[0] != "ACCEL":
        raise ValueError("Malformed XYZ sample")
    ms = int(fields[1])
    counts = [int(value) for value in fields[2:5]]
    actual = [float(value) for value in fields[5:]]
    if not 0 <= ms <= 0xFFFFFFFF or any(not -32768 < value < 32767 for value in counts):
        raise ValueError("Clipped or invalid sample")
    centred = [counts[i] * 9.80665 / 4096 - profile["remaining_bias"][i] for i in range(3)]
    expected = [sum(row[i] * centred[i] for i in range(3)) for row in profile["matrix"]]
    if any(not math.isfinite(actual[i]) or abs(actual[i] - expected[i]) > 0.00005 for i in range(3)):
        raise ValueError("XYZ output does not match the full matrix computation")
    return {"ms": ms, "counts_after_mpu_bias": counts, "drone_m_s2": actual,
            "norm_m_s2": math.sqrt(sum(value * value for value in actual))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=20)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 600:
        parser.error("Choose 5-600 seconds")
    source_sha = hashlib.sha256((ROOT / "firmware/main.cpp").read_bytes()).hexdigest()
    fit = json.loads((ROOT / "measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json").read_text(encoding="utf-8"))
    folder = ROOT / "artifacts/accel-recordings" / recorder.stamp()
    folder.mkdir(parents=True)
    result = {"engine": "Real ESP32-S3/MPU6050 XYZ-calibrated bench stream; no simulator",
              "status": "FAILED", "source_sha256": source_sha, "uploaded": False,
              "acceptance": "UNREVIEWED; software consistency is not absolute sensor accuracy"}
    device = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    device.port = args.port
    device.dtr = False
    device.rts = False
    samples = []
    try:
        print(f"Automatic recording folder: {folder}")
        print("All XYZ axes are corrected continuously. No prescribed pose or switching of axes.")
        input("Keep FC mounting unchanged, close Serial Monitor, then Enter to connect: ")
        device.open()
        with (folder / "serial.log").open("xb") as log:
            device.write(b"?")
            synchronized = False
            header = []
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                line = recorder.receive(device, log, synchronizing=not synchronized)
                if line == "QUERY_REPLY":
                    synchronized = True
                    continue
                if not synchronized or not line:
                    continue
                if line.startswith("ERROR,"):
                    raise ValueError(line)
                header.append(line)
                if line == "READY,ACCEL_XYZ_V1":
                    break
            else:
                raise ValueError("No XYZ ready response")
            profile = parse_profile(header, source_sha, fit)
            result["header"] = header
            result["profile"] = profile
            started = time.monotonic()
            next_progress = 0
            while time.monotonic() - started < args.seconds:
                line = recorder.receive(device, log)
                if not line:
                    continue
                if line.startswith("ERROR,"):
                    raise ValueError(line)
                sample = validate_sample(line, profile)
                if samples and not 0 < ((sample["ms"] - samples[-1]["ms"]) & 0xFFFFFFFF) < 2000:
                    raise ValueError("Reset, duplicate timestamp or long reporting gap")
                samples.append(sample)
                elapsed = time.monotonic() - started
                if elapsed >= next_progress:
                    x, y, z = sample["drone_m_s2"]
                    print(f"{elapsed:5.1f}/{args.seconds:g}s  X={x:+.4f} Y={y:+.4f} Z={z:+.4f}  norm={sample['norm_m_s2']:.4f} m/s^2")
                    log.flush()
                    next_progress = int(elapsed) + 1
            if len(samples) < args.seconds * 20:
                raise ValueError("Unexpectedly few XYZ reports")
            mean = [statistics.mean(row["drone_m_s2"][axis] for row in samples) for axis in range(3)]
            deviation = [statistics.stdev(row["drone_m_s2"][axis] for row in samples) for axis in range(3)]
            elapsed_device = ((samples[-1]["ms"] - samples[0]["ms"]) & 0xFFFFFFFF) / 1000
            result.update({"status": "CAPTURE_COMPLETE_REVIEW_REQUIRED", "samples": len(samples),
                "observed_report_rate_hz": (len(samples) - 1) / elapsed_device,
                "mean_drone_m_s2": mean, "sample_std_drone_m_s2": deviation,
                "mean_norm_m_s2": statistics.mean(row["norm_m_s2"] for row in samples),
                "note": "Arbitrary orientations/movement are allowed. Spread includes real movement; norm need not equal g during acceleration. These are bench reports, not 1000 Hz FIFO data or a 500 Hz controller."})
            print("Saved complete XYZ recording; no automatic calibration pass is declared.")
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or "User interrupted"
        raise
    finally:
        device.close()
        if samples:
            recorder.save_json(folder / "samples.json", {"samples": samples})
        if (folder / "serial.log").exists():
            result["serial_sha256"] = hashlib.sha256((folder / "serial.log").read_bytes()).hexdigest()
        recorder.save_json(folder / "summary.json", result)
        print(f"Saved evidence: {folder}")


if __name__ == "__main__":
    main()
