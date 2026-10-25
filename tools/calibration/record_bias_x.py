"""Save an X-bias raw/corrected/raw bench check. Sends r/c/? only; never flashes."""
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
BASELINE = -2074
CORRECTED = -2348
EXPECTED_SHIFT = (BASELINE - CORRECTED) * 9.80665 / 2048
CONFIG = "CONFIG,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=accel_x_mpu_only"


def summarize(phases):
    """Interpolate before/after raw means to the corrected phase's midpoint."""
    result = {}
    origin = phases["raw_before"][0]["ms"]
    for name, rows in phases.items():
        result[name] = {"samples": len(rows),
            "mean_time_ms": statistics.mean((row["ms"] - origin) & 0xFFFFFFFF for row in rows),
            "mean_x_m_s2": statistics.mean(row["x"] for row in rows),
            "sample_std_m_s2": statistics.stdev(row["x"] for row in rows)}
    before, corrected, after = [result[name] for name in ("raw_before", "corrected", "raw_after")]
    span = after["mean_time_ms"] - before["mean_time_ms"]
    if span <= 0:
        raise ValueError("Invalid phase timing")
    weight = (corrected["mean_time_ms"] - before["mean_time_ms"]) / span
    if not 0 < weight < 1:
        raise ValueError("Corrected interval must lie between raw intervals")
    interpolated_raw = (1-weight)*before["mean_x_m_s2"] + weight*after["mean_x_m_s2"]
    shift = interpolated_raw - corrected["mean_x_m_s2"]
    return {"phases": result, "expected_x_decrease_m_s2": EXPECTED_SHIFT,
            "interpolated_raw_mean_m_s2": interpolated_raw,
            "observed_x_decrease_m_s2": shift, "observed_minus_expected_m_s2": shift - EXPECTED_SHIFT,
            "raw_before_after_difference_m_s2": after["mean_x_m_s2"] - before["mean_x_m_s2"],
            "acceptance": "UNREVIEWED: register readback and physical shift need operator review",
            "limitations": "Interpolation cancels only linear drift; movement, nonlinear drift and timing matter. Standard deviations are spread, not uncertainty of mean. About 10 Hz bench reporting, not a noise-density/FC timing test."}


def wait_line(device, log):
    line = recorder.receive(device, log)
    if line.startswith("ERROR,"):
        raise ValueError(line)
    return line


def select_mode(device, log, command, offset):
    device.write(command)
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        if wait_line(device, log) == f"MODE,{offset}":
            return
    raise ValueError("No verified register-mode acknowledgment")


def collect(device, log, offset, seconds):
    # Receive and preserve the settling interval but exclude it from the means.
    start = time.monotonic()
    deadline = start + seconds + 2
    rows = []
    last_ms = None
    last_progress = 0
    while time.monotonic() < deadline:
        line = wait_line(device, log)
        if not line:
            continue
        fields = line.split(",")
        if len(fields) != 5 or fields[0] != "AX":
            raise ValueError(f"Unexpected bench output: {line[:100]}")
        ms, actual_offset, counts = map(int, fields[1:4])
        x = float(fields[4])
        if actual_offset != offset or not -32768 < counts < 32767 or not math.isfinite(x):
            raise ValueError("Invalid mode or ADC reading")
        if abs(x - counts*9.80665/4096) > 0.0001:
            raise ValueError("ADC/physical-unit conversion mismatch")
        if last_ms is not None and not 0 < ((ms-last_ms) & 0xFFFFFFFF) < 2000:
            raise ValueError("Duplicate timestamp/reset or large reporting gap")
        last_ms = ms
        elapsed = time.monotonic() - start
        if elapsed >= 2:
            rows.append({"ms": ms, "counts": counts, "x": x})
            if int(elapsed-2) > last_progress:
                last_progress = int(elapsed-2)
                log.flush()
                print(f"  {min(last_progress,seconds):g}/{seconds:g} seconds saved")
    if len(rows) < max(2, seconds*5):
        raise ValueError("Too few samples for this bench reporting rate")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=20, help="Seconds per phase (default 20)")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 60:
        parser.error("Choose 5-60 seconds per phase")
    source_sha = hashlib.sha256((ROOT / "firmware/main.cpp").read_bytes()).hexdigest()
    folder = ROOT / "artifacts/bias-x-recordings" / recorder.stamp()
    folder.mkdir(parents=True)
    report = {"source_sha256": source_sha, "engine": "Real ESP32-S3/MPU6050 register-toggle bench capture; no simulator",
              "status": "FAILED", "uploaded": False, "transmitted_commands": ["?", "r", "c"],
              "note": "Only X bias changes; no Y/Z, mounting, scale or motor output. Physical test not yet accepted."}
    device = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    device.port = args.port
    device.dtr = False
    device.rts = False
    matched = False
    try:
        print(f"Automatic recording folder: {folder}")
        input("Keep the drone on a fixed support throughout all three phases; close Serial Monitor, then Enter: ")
        device.open()
        with (folder / "serial.log").open("xb") as log:
            device.write(b"?")
            deadline = time.monotonic() + 8
            synchronized = False
            header = []
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
                if line == "READY,BIAS_X_V1":
                    break
            if f"HELLO,BIAS_X_V1,{source_sha}" not in header or CONFIG not in header:
                raise ValueError("Upload the current bench lesson first; source/configuration mismatch")
            offset_lines = [line for line in header if line.startswith("OFFSET_X,")]
            if len(offset_lines) != 1 or offset_lines[0] not in (f"OFFSET_X,{BASELINE},{CORRECTED},{BASELINE}", f"OFFSET_X,{BASELINE},{CORRECTED},{CORRECTED}"):
                raise ValueError("Unexpected register baseline/target")
            matched = True
            report["header"] = header
            phases = {}
            for name, command, offset in (("raw_before",b"r",BASELINE), ("corrected",b"c",CORRECTED), ("raw_after",b"r",BASELINE)):
                print(f"\n{name}: keep still; settling 2 seconds, then recording {args.seconds:g} seconds")
                select_mode(device, log, command, offset)
                phases[name] = collect(device, log, offset, args.seconds)
                recorder.save_json(folder / f"{name}.json", {"offset": offset, "selected_samples": phases[name]})
            select_mode(device, log, b"c", CORRECTED)
            report.update(summarize(phases))
            report["status"] = "CAPTURE_COMPLETE_REVIEW_REQUIRED"
            print(f"Expected X decrease: {EXPECTED_SHIFT:.6f} m/s^2")
            print(f"Observed X decrease: {report['observed_x_decrease_m_s2']:.6f} m/s^2")
            print("Saved; final register mode is X-corrected. Review results before the next step.")
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = str(error) or "User interrupted"
        raise
    finally:
        if matched and report["status"] == "FAILED":
            try:
                device.write(b"c")
                report["failure_recovery"] = "Requested X-corrected mode; not acknowledged/verified"
            except serial.SerialException:
                report["failure_recovery"] = "Could not request X-corrected mode"
        device.close()
        log_path = folder / "serial.log"
        if log_path.exists():
            report["serial_sha256"] = hashlib.sha256(log_path.read_bytes()).hexdigest()
        recorder.save_json(folder / "summary.json", report)
        print(f"Saved evidence: {folder}")


if __name__ == "__main__":
    main()
