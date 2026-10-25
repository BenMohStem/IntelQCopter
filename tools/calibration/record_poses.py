"""Prompt for 18 stationary poses; save original serial data automatically. Never flash."""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import time

import serial

ROOT = Path(__file__).resolve().parents[2]
POSES = ("Z+", "Z-", "X+", "X-", "Y+", "Y-")
CHANNELS = ("ax", "ay", "az", "temperature", "gx", "gy", "gz")
CONFIG = "CONFIG,hz=1000,accel_g=8,gyro_dps=500,dlpf=0,divider=7,i2c_hz=400000,user_cal=none"
SCALES = (9.80665 / 4096,) * 3 + (1 / 340,) + (math.pi / (180 * 65.5),) * 3


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def save_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class Samples:
    """Online means/standard deviations; individual original samples remain in serial.log."""
    def __init__(self):
        self.count = 0
        self.mean = [0.0] * 7
        self.m2 = [0.0] * 7
        self.hash = 2166136261
        self.clipped = 0

    def add(self, line):
        fields = line.split(",")
        if len(fields) != 9 or int(fields[1]) != self.count:
            raise ValueError("Missing, duplicate or malformed sample")
        values = [int(value) for value in fields[2:]]
        payload = struct.pack(">7h", *values)
        if self.count >= 20000:
            raise ValueError("More than 20000 samples")
        self.count += 1
        for index, value in enumerate(values):
            delta = value - self.mean[index]
            self.mean[index] += delta / self.count
            self.m2[index] += delta * (value - self.mean[index])
            self.clipped += int(value in (-32768, 32767))
        for byte in payload:
            self.hash = ((self.hash ^ byte) * 16777619) & 0xFFFFFFFF

    def finish(self, line, firmware_means):
        fields = line.split(",")
        if len(fields) != 5 or int(fields[1]) != 20000 or self.count != 20000:
            raise ValueError("Incomplete recording")
        elapsed_us = int(fields[2])
        maximum_fifo = int(fields[3])
        if elapsed_us <= 0:
            raise ValueError("Invalid device timing")
        rate = self.count * 1e6 / elapsed_us
        if not 970 <= rate <= 1030 or not 14 <= maximum_fifo < 980:
            raise ValueError("Timing/backlog check failed; preserve and inspect this attempt")
        if int(fields[4], 16) != self.hash:
            raise ValueError("Sample payload checksum mismatch")
        if set(firmware_means) != set(range(7)):
            raise ValueError("Missing firmware averages")
        if any(abs(firmware_means[i] - self.mean[i]) > 1e-6 for i in range(7)):
            raise ValueError("PC and firmware averages disagree")
        if self.clipped:
            raise ValueError("Clipped sensor data")
        deviations = [math.sqrt(value / (self.count - 1)) for value in self.m2]
        physical = [self.mean[i] * SCALES[i] for i in range(7)]
        physical[3] += 36.53
        return {
            "samples": self.count,
            "elapsed_device_us": elapsed_us,
            "observed_average_hz": rate,
            "maximum_fifo_bytes": maximum_fifo,
            "checksum_fnv1a32": f"{self.hash:08X}",
            "mean_counts": dict(zip(CHANNELS, self.mean)),
            "sample_std_counts": dict(zip(CHANNELS, deviations)),
            "mean_physical": dict(zip(CHANNELS, physical)),
            "sample_std_physical": dict(zip(CHANNELS, [deviations[i] * SCALES[i] for i in range(7)])),
            "units": "accel m/s^2; temperature degC; gyro rad/s; SENSOR axes",
            "note": "Std describes sample spread, not accuracy or standard error. Samples may be correlated.",
        }


def receive(device, log, synchronizing=False):
    raw = device.readline()
    if not raw:
        return ""
    log.write(raw)
    if synchronizing:
        # Preserve all bytes, but wait for the fresh query marker before decoding.
        # An idle UART byte may prefix the marker on the same line.
        if raw.endswith((b"QUERY_REPLY\n", b"QUERY_REPLY\r\n")):
            return "QUERY_REPLY"
        return ""
    if not raw.endswith(b"\n"):
        raise ValueError("Truncated serial line; keep failed attempt")
    try:
        return raw.decode("ascii").strip()
    except UnicodeDecodeError as error:
        raise ValueError("Non-text serial data after synchronization; recording rejected and original bytes saved") from error


def capture(device, attempt, expected_sha, reference_configuration=None):
    result = {"status": "FAILED", "hardware_acceptance": "UNREVIEWED"}
    try:
        with (attempt / "serial.log").open("xb") as log:
            # A request for identity/configuration only; no reset or offset changes.
            device.write(b"?")
            deadline = time.monotonic() + 8
            header = []
            reply_started = False
            while time.monotonic() < deadline:
                line = receive(device, log, synchronizing=not reply_started)
                if line == "QUERY_REPLY":
                    reply_started = True
                    continue
                if not reply_started:
                    continue
                if line.startswith("ERROR,"):
                    raise ValueError(line)
                if line:
                    header.append(line)
                if line.startswith("READY,"):
                    break
            else:
                raise ValueError("No READY response. Upload this firmware and close other serial monitors.")
            if f"HELLO,POSE1000_V1,{expected_sha}" not in header or CONFIG not in header:
                raise ValueError("Uploaded firmware/configuration does not match this source")
            if sum(line.startswith("OFFSET,") for line in header) != 6:
                raise ValueError("Missing offset register baseline")
            if reference_configuration is not None and header != reference_configuration:
                raise ValueError("Board/configuration/offset baseline changed; start a separate session")
            result["configuration"] = header
            print("Keep still: 5 seconds to settle...")
            time.sleep(5)
            device.write(b"s")
            samples = Samples()
            firmware_means = {}
            started = False
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                line = receive(device, log)
                if not line:
                    continue
                if line == "BEGIN,20000" and not started:
                    started = True
                elif line.startswith("S,") and started:
                    samples.add(line)
                    if samples.count % 1000 == 0:
                        log.flush()
                        print(f"  {samples.count}/20000 samples saved")
                elif line.startswith("MEAN_COUNTS,") and samples.count == 20000:
                    _, axis, mean = line.split(",")
                    if int(axis) in firmware_means or not math.isfinite(float(mean)):
                        raise ValueError("Invalid firmware mean")
                    firmware_means[int(axis)] = float(mean)
                elif line.startswith("END,") and started:
                    result.update(samples.finish(line, firmware_means))
                    result["status"] = "CAPTURE_COMPLETE_REVIEW_REQUIRED"
                    break
                else:
                    raise ValueError(f"Unexpected firmware output: {line[:160]}")
            else:
                raise ValueError("Capture timeout")
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or "User interrupted recording"
        raise
    finally:
        if result["status"] == "FAILED":
            try:
                device.write(b"x")
            except serial.SerialException:
                pass
        raw_path = attempt / "serial.log"
        if raw_path.exists():
            result["serial_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        save_json(attempt / "summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--resume", type=Path, help="Existing session directory printed by this tool")
    args = parser.parse_args()
    source_sha = hashlib.sha256((ROOT / "firmware/main.cpp").read_bytes()).hexdigest()
    if args.resume:
        folder = args.resume.resolve(strict=True)
        manifest = json.loads((folder / "session.json").read_text())
        if manifest["source_sha256"] != source_sha:
            raise ValueError("Source changed: start a new session, do not mix firmware versions")
    else:
        folder = ROOT / "artifacts/pose-recordings" / stamp()
        folder.mkdir(parents=True)
        manifest = {"source_sha256": source_sha, "port": args.port, "baud": 921600,
                    "completed": [], "engine": "Real MPU6050 FIFO via ESP32-S3; no simulator",
                    "poses": list(POSES), "repeats": 3, "hardware_acceptance": "UNREVIEWED"}
        save_json(folder / "session.json", manifest)
    print(f"Automatic recording folder: {folder}")
    print("MPU stays mounted. Move the whole assembly; named DRONE axis points upward.")
    print("Propellers removed, motors disabled. Close PlatformIO Serial Monitor.")
    device = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    device.port = args.port
    device.dtr = False
    device.rts = False
    try:
        # Open only after human confirmation; does not upload or change motor outputs.
        input("Check the setup, then press Enter to connect (Ctrl+C quits): ")
        device.open()
        for repeat in range(1, 4):
            for pose in POSES:
                label = f"repeat-{repeat}_{pose.replace('+', 'plus').replace('-', 'minus')}"
                if label in manifest["completed"]:
                    continue
                input(f"\n{label}: position {pose} UP, check reference, then Enter: ")
                attempt = folder / label / stamp()
                attempt.mkdir(parents=True)
                print(f"Saving this attempt: {attempt}")
                result = capture(device, attempt, source_sha, manifest.get("reference_configuration"))
                manifest["reference_configuration"] = result["configuration"]
                manifest["completed"].append(label)
                save_json(folder / "session.json", manifest)
                means = result["mean_physical"]
                print(f"Saved. Mean accel sensor XYZ = {means['ax']:.6f}, {means['ay']:.6f}, {means['az']:.6f} m/s^2")
                print(f"Measured average rate: {result['observed_average_hz']:.2f} Hz. Calibration not yet assessed.")
        print("All 18 recordings saved. Keep the folder intact for analysis; no calibration was fitted.")
    finally:
        device.close()
        print(f'Resume if needed: .\\qcopter.ps1 record-poses --port {args.port} --resume "{folder}"')


if __name__ == "__main__":
    main()
