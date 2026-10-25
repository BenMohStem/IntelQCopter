"""Record stationary gyro XYZ, corrected acceleration and temperature; never writes gyro offsets or flashes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import serial
import record_poses as transport
import record_accel as acceleration

ROOT = Path(__file__).resolve().parents[2]
GYRO_SCALE = "GYRO_SCALE,65.5,counts_per_degree_per_second,frame=sensor,correction=none"


def read_header(device, log, source_sha, fit):
    device.write(b"?")
    synchronized = False
    header = []
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        line = transport.receive(device, log, synchronizing=not synchronized)
        if line == "QUERY_REPLY":
            synchronized = True
            continue
        if not synchronized or not line:
            continue
        if line.startswith("ERROR,"):
            raise ValueError(line)
        header.append(line)
        if line == "READY,IMU_XYZ_V1":
            break
    else:
        raise ValueError("No IMU_XYZ_V1 response; upload the current bench firmware first")
    profile = acceleration.parse_profile(header, source_sha, fit, protocol="IMU_XYZ_V1")
    if GYRO_SCALE not in header:
        raise ValueError("Unexpected gyro units/frame or correction mode")
    offsets = [None] * 3
    for line in header:
        if line.startswith("GYRO_OFFSET,"):
            fields = line.split(",")
            if len(fields) != 3:
                raise ValueError("Malformed gyro offset metadata")
            axis, value = map(int, fields[1:])
            if not 0 <= axis < 3 or offsets[axis] is not None or not -32768 <= value <= 32767:
                raise ValueError("Invalid/duplicate gyro offset")
            offsets[axis] = value
    if any(value is None for value in offsets):
        raise ValueError("Missing gyro offset metadata")
    profile["gyro_offsets"] = offsets
    return header, profile


def parse_sample(line, profile):
    fields = line.split(",")
    if len(fields) != 16 or fields[0] != "IMU":
        raise ValueError("Malformed IMU packet")
    accel_line = ",".join(["ACCEL", fields[1], *fields[2:5], *fields[9:12]])
    sample = acceleration.validate_sample(accel_line, profile)
    temperature_count = int(fields[5])
    counts = [int(value) for value in fields[6:9]]
    rates = [float(value) for value in fields[12:15]]
    temperature = float(fields[15])
    if not -32768 <= temperature_count <= 32767 or any(not -32768 < value < 32767 for value in counts):
        raise ValueError("Clipped/invalid gyro or temperature counts")
    for count, value in zip(counts, rates):
        if not math.isfinite(value) or abs(value - count * math.pi / (180 * 65.5)) > 0.000003:
            raise ValueError("Gyro ADC/rad-per-second conversion mismatch")
    if not math.isfinite(temperature) or abs(temperature - (temperature_count / 340 + 36.53)) > 0.001:
        raise ValueError("Temperature conversion mismatch")
    sample.update({"gyro_counts_at_recorded_offsets": counts, "gyro_sensor_rad_s": rates,
                   "temperature_count": temperature_count, "temperature_C": temperature})
    return sample


def mean_covariance(vectors):
    if len(vectors) < 2:
        raise ValueError("Need at least two samples")
    mean = [statistics.mean(row[i] for row in vectors) for i in range(3)]
    covariance = [[sum((row[i]-mean[i])*(row[j]-mean[j]) for row in vectors)/(len(vectors)-1)
                   for j in range(3)] for i in range(3)]
    return mean, covariance


def validate_timing(samples, requested_seconds):
    if len(samples) < requested_seconds * 20:
        raise ValueError("Too few gyro reports")
    elapsed_ms = 0
    max_gap_ms = 0
    for previous, current in zip(samples, samples[1:]):
        gap = (current["ms"] - previous["ms"]) & 0xFFFFFFFF
        if not 0 < gap <= 100:
            raise ValueError("Duplicate/reset timestamp or gyro report gap exceeds 100 ms")
        elapsed_ms += gap
        max_gap_ms = max(max_gap_ms, gap)
    duration = elapsed_ms / 1000
    if not 0.90 * requested_seconds <= duration <= 1.10 * requested_seconds:
        raise ValueError("Device timestamps do not cover the requested recording duration")
    return {"device_duration_s": duration, "max_report_gap_ms": max_gap_ms,
            "observed_report_rate_hz": (len(samples)-1)/duration}


def summarize(samples):
    if len(samples) < 100:
        raise ValueError("Too few gyro samples")
    split = len(samples) * 2 // 3
    training = [row["gyro_sensor_rad_s"] for row in samples[:split]]
    validation = [row["gyro_sensor_rad_s"] for row in samples[split:]]
    bias, covariance = mean_covariance(training)
    held_mean, held_covariance = mean_covariance(validation)
    origin = samples[0]["ms"]
    times = [((row["ms"]-origin) & 0xFFFFFFFF)/1000 for row in samples]
    mean_t = statistics.mean(times)
    denominator = sum((t-mean_t)**2 for t in times)
    if denominator <= 0:
        raise ValueError("Cannot estimate drift from identical timestamps")
    all_mean = [statistics.mean(row["gyro_sensor_rad_s"][i] for row in samples) for i in range(3)]
    slopes = [sum((t-mean_t)*(row["gyro_sensor_rad_s"][i]-all_mean[i]) for t,row in zip(times,samples))/denominator
              for i in range(3)]
    return {"training_samples": split, "held_out_samples": len(samples)-split,
            "candidate_bias_sensor_rad_s": bias,
            "candidate_bias_sensor_deg_s": [v*180/math.pi for v in bias],
            "sample_std_training_rad_s": [math.sqrt(covariance[i][i]) for i in range(3)],
            "sample_covariance_training_rad2_s2": covariance,
            "held_out_mean_raw_rad_s": held_mean,
            "offline_predicted_held_out_mean_after_bias_rad_s": [held_mean[i]-bias[i] for i in range(3)],
            "held_out_covariance_rad2_s2": held_covariance,
            "linear_rate_drift_rad_s2": slopes,
            "temperature_range_C": [min(row["temperature_C"] for row in samples),max(row["temperature_C"] for row in samples)],
            "temperature_mean_C": statistics.mean(row["temperature_C"] for row in samples),
            "gyro_peak_magnitude_rad_s": max(math.sqrt(sum(v*v for v in row["gyro_sensor_rad_s"])) for row in samples),
            "status": "CANDIDATE_ONLY_REVIEW_STATIONARITY_TEMPERATURE_AND_DRIFT",
            "note": "Gyro offsets were only observed, not written. Bias removal on the held-out segment is offline prediction. No Gaussian/IID, rate-scale, mounting or final Kalman-noise-density claim."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=60)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 20 <= args.seconds <= 600:
        parser.error("Choose 20-600 seconds")
    source_sha = hashlib.sha256((ROOT/"firmware/main.cpp").read_bytes()).hexdigest()
    fit = json.loads((ROOT/"measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json").read_text(encoding="utf-8"))
    folder = ROOT/"artifacts/gyro-recordings"/transport.stamp()
    folder.mkdir(parents=True)
    report = {"engine": "Real ESP32-S3/MPU6050 gyro-observation stream; no simulator",
              "source_sha256": source_sha, "status": "FAILED", "uploaded": False,
              "commands_sent": [], "gyro_offsets_written": False,
              "operator_confirmed_motionless": False}
    device = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    device.port = args.port
    device.dtr = False
    device.rts = False
    samples = []
    try:
        print(f"Automatic recording folder: {folder}")
        print("Put the drone on a fixed support. Any orientation; do not hold or rotate it. Close Serial Monitor.")
        input("Press Enter only when completely motionless; keep still until Saved: ")
        report["operator_confirmed_motionless"] = True
        device.open()
        with (folder/"serial.log").open("xb") as log:
            report["commands_sent"].append("?")
            header, profile = read_header(device, log, source_sha, fit)
            report["header"] = header
            report["profile"] = profile
            print("Five seconds settling, then stationary measurement...")
            started = time.monotonic()
            last_ms = None
            last_progress = -1
            all_packets = 0
            while time.monotonic()-started < args.seconds+5:
                line = transport.receive(device, log)
                if not line:
                    continue
                if line.startswith("ERROR,"):
                    raise ValueError(line)
                sample = parse_sample(line, profile)
                if last_ms is not None and not 0 < ((sample["ms"]-last_ms) & 0xFFFFFFFF) < 2000:
                    raise ValueError("Reset, duplicate timestamp or long gap")
                last_ms = sample["ms"]
                all_packets += 1
                elapsed = time.monotonic()-started-5
                if elapsed < 0:
                    continue
                samples.append(sample)
                if int(elapsed) > last_progress:
                    last_progress = int(elapsed)
                    x,y,z=sample["gyro_sensor_rad_s"]
                    print(f"{min(elapsed,args.seconds):5.1f}/{args.seconds:g}s  gyro X={x:+.6f} Y={y:+.6f} Z={z:+.6f} rad/s  T={sample['temperature_C']:.2f} C")
                    log.flush()
            timing = validate_timing(samples, args.seconds)
            report["commands_sent"].append("?")
            final_header, _ = read_header(device, log, source_sha, fit)
            if final_header != header:
                raise ValueError("Calibration/offset/source state changed during capture")
            report.update(timing)
            report.update(summarize(samples))
            report.update({"status":"CAPTURE_COMPLETE_REVIEW_REQUIRED", "samples":len(samples),
                "all_packets_verified_including_settling":all_packets,
                "final_header_unchanged":True})
            print("Saved. Candidate gyro bias (sensor rad/s):",report["candidate_bias_sensor_rad_s"])
            print("Candidate only: no gyro correction was applied or stored in the MPU.")
    except (Exception,KeyboardInterrupt) as error:
        report["error"] = str(error) or "User interrupted"
        raise
    finally:
        device.close()
        if samples:
            transport.save_json(folder/"samples.json",{"selected_after_settling":samples})
        if (folder/"serial.log").exists():
            report["serial_sha256"] = hashlib.sha256((folder/"serial.log").read_bytes()).hexdigest()
        transport.save_json(folder/"summary.json",report)
        print(f"Saved evidence: {folder}")


if __name__=="__main__":
    main()
