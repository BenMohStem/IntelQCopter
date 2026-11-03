#!/usr/bin/env python3
"""IntelQCopter v18 — unattended differential QMC5883P/ESP32-S3 diagnostics.

No GUI, interactive selection, arbitrary 60 s port waiting, or serial commands.
A genuine cold power cycle requires an externally switchable *power* path.
USB/ESP reset is labelled WARM and is NEVER claimed to power-cycle the sensor.

Standard library + pyserial; firmware from IntelQCopter v15/v16 with repeated
versioned STATE frames, immutable build-specific run nonce per experiment.
"""
from __future__ import annotations
import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import random
import re
import shutil
import statistics
import subprocess
import sys
import time
import traceback
import zipfile

ROOT = Path(__file__).resolve().parent.parent
ENV = "esp32-s3-calibration-500hz"
INI = ROOT / "platformio.ini"
RESULTS = ROOT / "results"
BUILD_KEYS = (
    "IQ_RUN_NONCE", "IQ_MAG_BOOT_PROFILE", "IQ_MAG_OSR1", "IQ_MAG_CSV_DIVIDER",
    "IQ_I2C_HZ", "IQ_SERIAL_BAUD", "IQ_MAG_READ_STOP", "IQ_MAG_SOFT_RESET", "IQ_MAG_START_ENABLE", "IQ_MAG_CTRL1_OVERRIDE", "IQ_RECOVERY_ENABLE",
    "IQ_RECOVERY_MAX_ATTEMPTS", "IQ_ALLOW_100HZ_FALLBACK", "IQ_BUS_LOAD_ENABLE",
    "IQ_WIFI_STRESS_MODE", "IQ_WIFI_START_DELAY_MS", "IQ_WIFI_SCAN_INTERVAL_MS",
)
BASE = {
    "IQ_RUN_NONCE": 0, "IQ_MAG_BOOT_PROFILE": 50, "IQ_MAG_OSR1": 8,
    "IQ_MAG_CSV_DIVIDER": 1, "IQ_I2C_HZ": 100000, "IQ_SERIAL_BAUD": 115200,
    "IQ_MAG_READ_STOP": 0, "IQ_MAG_SOFT_RESET": 0, "IQ_MAG_START_ENABLE": 1, "IQ_MAG_CTRL1_OVERRIDE": 0, "IQ_RECOVERY_ENABLE": 0,
    "IQ_RECOVERY_MAX_ATTEMPTS": 2, "IQ_ALLOW_100HZ_FALLBACK": 0,
    "IQ_BUS_LOAD_ENABLE": 0, "IQ_WIFI_STRESS_MODE": 0,
    "IQ_WIFI_START_DELAY_MS": 15000, "IQ_WIFI_SCAN_INTERVAL_MS": 10000,
}
# Each profile varies only the explicitly listed factors versus BASE.  All
# tests are independently rebuilt/flashed: no runtime ODR change is attempted.
PRESETS = {
    "T0": ("QMC prestart parked - NO measurement-mode write", {"IQ_MAG_START_ENABLE": 0}),
    "T1": ("QST 0xC3 continuous example / STOP+START (ODR not applicable)", {"IQ_MAG_BOOT_PROFILE": 10, "IQ_MAG_CTRL1_OVERRIDE": 195, "IQ_MAG_READ_STOP": 1}),
    "H0": ("Historical 0x87 continuous mode / STOP+START (ODR not applicable)", {"IQ_MAG_CTRL1_OVERRIDE": 135, "IQ_MAG_READ_STOP": 1}),
    "A0": ("50Hz clean baseline", {}),
    "A1": ("100Hz clean baseline", {"IQ_MAG_BOOT_PROFILE": 100}),
    "A2": ("200Hz clean baseline", {"IQ_MAG_BOOT_PROFILE": 200}),
    "A3": ("200Hz low OSR", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_MAG_OSR1": 1}),
    "A4": ("200Hz bounded recovery", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_RECOVERY_ENABLE": 1}),
    "B1": ("50Hz delayed WiFi AP", {"IQ_WIFI_STRESS_MODE": 1}),
    "B2": ("50Hz WiFi scan stress", {"IQ_WIFI_STRESS_MODE": 2}),
    "C1": ("50Hz MPU/BMP I2C load", {"IQ_BUS_LOAD_ENABLE": 1}),
    "C2": ("50Hz WiFi AP + I2C load", {"IQ_BUS_LOAD_ENABLE": 1, "IQ_WIFI_STRESS_MODE": 1}),
    "C3": ("100Hz WiFi AP + I2C load", {"IQ_MAG_BOOT_PROFILE": 100, "IQ_BUS_LOAD_ENABLE": 1, "IQ_WIFI_STRESS_MODE": 1}),
    "C4": ("200Hz WiFi AP + I2C load", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_BUS_LOAD_ENABLE": 1, "IQ_WIFI_STRESS_MODE": 1}),
    "D1": ("50Hz AP + load / I2C 400k", {"IQ_BUS_LOAD_ENABLE": 1, "IQ_WIFI_STRESS_MODE": 1, "IQ_I2C_HZ": 400000}),
    "D2": ("200Hz AP + load / I2C 400k", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_BUS_LOAD_ENABLE": 1, "IQ_WIFI_STRESS_MODE": 1, "IQ_I2C_HZ": 400000}),
    "E1": ("50Hz baud 230400", {"IQ_SERIAL_BAUD": 230400}),
    "E2": ("50Hz baud 460800", {"IQ_SERIAL_BAUD": 460800}),
    "E3": ("50Hz AP / baud 460800", {"IQ_SERIAL_BAUD": 460800, "IQ_WIFI_STRESS_MODE": 1}),
    "E4": ("100Hz AP / baud 460800", {"IQ_MAG_BOOT_PROFILE": 100, "IQ_SERIAL_BAUD": 460800, "IQ_WIFI_STRESS_MODE": 1}),
    "E5": ("200Hz AP / baud 460800", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_SERIAL_BAUD": 460800, "IQ_WIFI_STRESS_MODE": 1}),
    "F1": ("50Hz STOP+START instead of repeated START", {"IQ_MAG_READ_STOP": 1}),
    "F2": ("50Hz AP + STOP+START", {"IQ_MAG_READ_STOP": 1, "IQ_WIFI_STRESS_MODE": 1}),
    "F3": ("200Hz STOP+START", {"IQ_MAG_READ_STOP": 1, "IQ_MAG_BOOT_PROFILE": 200}),
    "G0": ("50Hz explicit QMC register soft reset", {"IQ_MAG_SOFT_RESET": 1}),
    "G1": ("100Hz explicit QMC register soft reset", {"IQ_MAG_BOOT_PROFILE": 100, "IQ_MAG_SOFT_RESET": 1}),
    "G2": ("200Hz explicit QMC register soft reset", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_MAG_SOFT_RESET": 1}),
    "G3": ("200Hz low OSR + QMC register soft reset", {"IQ_MAG_BOOT_PROFILE": 200, "IQ_MAG_OSR1": 1, "IQ_MAG_SOFT_RESET": 1}),
    "G4": ("50Hz WiFi AP + QMC register soft reset", {"IQ_WIFI_STRESS_MODE": 1, "IQ_MAG_SOFT_RESET": 1}),
    "G5": ("50Hz STOP+START + QMC register soft reset", {"IQ_MAG_READ_STOP": 1, "IQ_MAG_SOFT_RESET": 1}),
}
SUITES = {
    "triage": "T0 F1 H0 A0 G5 T1 A1", # no WiFi, baud stress or 200Hz until basic read path works
    "core": "A0 G0 A1 G1 A2 G2 B1 G4 C1 C2 E1 E2 F1 G5",
    "rates": "A0 G0 A1 G1 A2 G2 A3 G3 A4 F3",
    "wifi": "A0 G0 B1 G4 B2 C1 C2 C3 C4",
    "uart": "A0 G0 E1 E2 B1 E3 E4 E5",
    "i2c": "A0 G0 F1 G5 B1 F2 C1 C2 D1 A2 F3 D2",
    "failure": "A0 G0 F1 G5 B1 G4 F2 A1 G1 A2 G2 A3 G3 A4 F3",
    "softreset": "A0 G0 A1 G1 A2 G2 B1 G4 F1 G5 A3 G3",
    "full": " ".join(PRESETS),
}
STATE_KEYS = {
    "run_nonce": "IQ_RUN_NONCE", "rate": "IQ_MAG_BOOT_PROFILE",
    "osr1": "IQ_MAG_OSR1", "serial": "IQ_SERIAL_BAUD", "I2C": "IQ_I2C_HZ",
    "read_stop": "IQ_MAG_READ_STOP", "start_enable": "IQ_MAG_START_ENABLE", "ctrl1_override": "IQ_MAG_CTRL1_OVERRIDE", "soft_reset": "IQ_MAG_SOFT_RESET", "wifi_mode": "IQ_WIFI_STRESS_MODE",
    "bus_load": "IQ_BUS_LOAD_ENABLE", "recover": "IQ_RECOVERY_ENABLE",
    "max_attempts": "IQ_RECOVERY_MAX_ATTEMPTS",
    "csv_divider": "IQ_MAG_CSV_DIVIDER", "fallback_allowed": "IQ_ALLOW_100HZ_FALLBACK",
}
KV = re.compile(r"\b([a-zA-Z_][a-zA-Z_0-9]*)=([^\s]+)")


def flags_for(pid: str, nonce: int = 0) -> dict[str, int]:
    if pid not in PRESETS:
        raise ValueError(f"Unknown preset {pid}")
    d = dict(BASE)
    d.update(PRESETS[pid][1])
    d["IQ_MAG_CSV_DIVIDER"] = max(1, int(round(d["IQ_MAG_BOOT_PROFILE"] / 50)))
    d["IQ_RUN_NONCE"] = nonce
    return d


def render_ini(original: str, flags: dict[str, int]) -> str:
    data = original
    for name in BUILD_KEYS:
        if name not in flags:
            raise ValueError("Missing build flag: " + name)
        pattern = re.compile(r"(?m)^([ \t]*-D" + re.escape(name) + r"=)[^\r\n]*(\r?)$")
        if len(pattern.findall(data)) != 1:
            raise ValueError(f"Expected exactly one build flag {name} in ini")
        data = pattern.sub(lambda m, val=flags[name]: m.group(1)+str(val)+m.group(2), data)
    data = re.sub(r"(?m)^(monitor_speed\s*=\s*)\d+", "\\g<1>" + str(flags["IQ_SERIAL_BAUD"]), data)
    return data


def platformio_cmd(user_path: str) -> str:
    if user_path:
        p = Path(user_path).expanduser()
        if p.exists():
            return str(p)
        v = shutil.which(user_path)
        if v: return v
        raise RuntimeError("PlatformIO executable not found: " + user_path)
    for p in (Path.home()/".platformio/penv/Scripts/platformio.exe", Path.home()/".platformio/penv/bin/platformio"):
        if p.exists(): return str(p)
    for x in ("platformio", "pio"):
        p = shutil.which(x)
        if p: return p
    raise RuntimeError("PlatformIO not found; specify --pio PATH")


def ports():
    from serial.tools import list_ports
    return list(list_ports.comports())


def describe_ports() -> list[dict]:
    rows = [{"device": p.device, "description": p.description, "vid": p.vid,
             "pid": p.pid, "serial_number": p.serial_number, "location": p.location} for p in ports()]
    for p in rows:
        print(f"{p['device']}: {p['description']} VID={p['vid']} PID={p['pid']} serial={p['serial_number']}")
    return rows


def choose_device(given: str):
    available = ports()
    if given:
        for p in available:
            if p.device.lower() == given.lower(): return p
        raise RuntimeError(f"{given} not present at startup; connect board USB and check Device Manager")
    # Avoid selecting Bluetooth ports when the CH343 USB UART is present.
    matching = [p for p in available if p.vid == 0x1A86 and p.pid == 0x55D3]
    if len(matching) == 1: return matching[0]
    if len(available) == 1: return available[0]
    raise RuntimeError("Cannot safely determine ESP32 port. Supply --port COM3; see --list-ports")


def locate_device(info, original: str):
    found = ports()
    # Strongest identity first: CH343 serial number / VID / PID, then USB topology.
    if info.serial_number and info.vid is not None:
        same = [p for p in found if (p.vid, p.pid, p.serial_number) == (info.vid, info.pid, info.serial_number)]
        if len(same) == 1: return same[0].device
    if info.location and info.vid is not None:
        same = [p for p in found if (p.vid, p.pid, p.location) == (info.vid, info.pid, info.location)]
        if len(same) == 1: return same[0].device
    same = [p for p in found if p.device.lower() == original.lower()]
    if len(same) == 1: return same[0].device
    return None


def wait_for_device(info, preferred: str, timeout: float, event, baud: int):
    started = time.monotonic()
    last = -100.0
    while True:
        found = locate_device(info, preferred)
        if found:
            try:
                import serial
                # Avoid intentionally driving DTR/RTS; auto-reset depends on the board.
                conn = serial.Serial(port=None, baudrate=baud, timeout=0.20, write_timeout=1,
                                     dsrdtr=False, rtscts=False)
                conn.dtr = False
                conn.rts = False
                conn.port = found
                conn.open()
                return conn, found
            except (serial.SerialException, OSError, ValueError):
                pass
        if timeout > 0 and time.monotonic()-started >= timeout:
            raise TimeoutError(f"Port did not become usable within {timeout:g}s; connection failed, NOT sensor data")
        if time.monotonic()-last > 15:
            last = time.monotonic()
            event("WAIT_PORT", target=preferred, seconds=round(time.monotonic()-started, 1))
        time.sleep(0.25)


def kv(line: str) -> dict[str, str]:
    return dict(KV.findall(line))


@dataclass
class Capture:
    observed_port: str
    elapsed_s: float
    stop_reason: str
    disconnected: bool
    byte_count: int
    lines: list[str]


def capture_stream(conn, port: str, path: Path, args, expected_wifi: int, event) -> Capture:
    start = time.monotonic()
    last_byte = start
    pending = bytearray()
    parsed: list[str] = []
    health: list[dict[str,str]] = []
    csv_rows = 0
    size = 0
    disconnected = False
    stop_reason = "MAX_MEASUREMENT_SAFETY_LIMIT"
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("wb") as fp:
            while True:
                now = time.monotonic()
                age = now - start
                if age >= args.max_sec:
                    break
                if age >= 5 and len(health)>=5 and (any(l.startswith("CONFIG_PARKED prestart_ok=1") for l in parsed) or any(l.startswith("STATE ") and kv(l).get("prestart_ok")=="1" for l in parsed)):
                    stop_reason = "VERIFIED_PRESTART_ONLY"
                    break
                if age >= args.min_sec and len(health) >= args.min_health:
                    pre = sum(h.get("wifi_active") == "0" for h in health)
                    post = sum(h.get("wifi_active") == "1" for h in health)
                    if expected_wifi == 0 or (pre >= args.pre_wifi_health and post >= args.post_wifi_health):
                        stop_reason = "ADAPTIVE_EVIDENCE_COLLECTED"
                        break
                # Persistent zero-byte QMC reads do not require 25 seconds of repeated errors.
                # Eight HEALTH reports and >40 failed polls are sufficient fault evidence.
                if age >= 8 and len(health)>=8 and expected_wifi == 0:
                    if all(num(h, "fresh", 0)==0 for h in health) and num(health[-1],"status_fail",0)>=40:
                        stop_reason = "EARLY_PERSISTENT_I2C_FAILURE"
                        break
                if (now-last_byte) > args.silence_sec and age > args.silence_sec:
                    stop_reason = "SERIAL_SILENCE_WATCHDOG"
                    break
                try:
                    chunk = conn.read(min(max(conn.in_waiting,1), 4096))
                except (OSError, ValueError, Exception) as ex:
                    # pyserial exceptions are expected on USB disconnect, but
                    # preserve the diagnostic message rather than masking them.
                    event("SERIAL_READ_ERROR", detail=str(ex))
                    disconnected = True
                    stop_reason = "USB_DISCONNECTED"
                    break
                if not chunk:
                    continue
                last_byte = time.monotonic()
                size += len(chunk)
                fp.write(chunk)
                pending.extend(chunk)
                if len(pending) > 8192 and b"\n" not in pending:
                    event("UNTERMINATED_SERIAL_LINE", bytes=len(pending))
                    pending.clear()
                while b"\n" in pending:
                    block, _, rest = pending.partition(b"\n")
                    pending = bytearray(rest)
                    line = block.decode("utf-8", "replace").rstrip("\r")
                    if len(parsed) >= 125000:
                        # Keep full bytes on disk and bounded RAM usage.
                        parsed = parsed[-10000:]
                    parsed.append(line)
                    if line.startswith("HEALTH "):
                        h = kv(line)
                        health.append(h)
                        if len(health) % 5 == 0:
                            event("HEALTH", windows=len(health), hz=h.get("measured_hz_window"),
                                  fresh=h.get("fresh"), regs_ok=h.get("regs_ok"),
                                  status_fail=h.get("status_fail"), wifi=h.get("wifi_active"))
                    elif line.startswith("CSV,"):
                        csv_rows += 1
                    elif line.startswith(("STATE ","BUILD ","VERIFY ","WIFI_EVENT", "FIRST_STATUS_FAILURE", "CONFIG_", "RESET_SIGNATURE", "RECOVERY_", "MAG_SOFT_RESET", "POST_START_BUS")):
                        event("FW", text=line[:300])
        if pending:
            parsed.append(pending.decode("utf-8", "replace"))
    finally:
        conn.close()
    event("CAPTURE_END", seconds=round(time.monotonic()-start,2), reason=stop_reason, bytes=size, csv=csv_rows)
    return Capture(port, time.monotonic()-start, stop_reason, disconnected, size, parsed)


def num(d, key, default=None):
    try: return float(d[key])
    except (KeyError, ValueError, TypeError): return default



def xyz_diagnostics(lines: list[str]) -> dict:
    """Descriptive stationary bench readings; NOT a 3-D ellipsoid calibration.

    CSV timestamps are sensor poll times, and decimation is intentional.  Count
    fresh/ODR from HEALTH instead.  Split Wi-Fi before/after using firmware
    WIFI_EVENT t_ms (board uptime), not host capture time.
    """
    rows=[]
    for line in lines:
        if not line.startswith("CSV,"):
            continue
        v=line.split(",")
        if len(v)<8: continue
        try:
            rows.append((int(v[1]),int(v[2]),int(v[3]),int(v[4]),int(v[6]),int(v[7])))
        except ValueError:
            continue
    if not rows:
        return {"count":0,"warning":"No valid CSV sample rows"}
    def describe(data):
        if not data: return {"count":0}
        a=[[float(row[j]) for row in data] for j in (1,2,3)]
        means=[statistics.mean(x) for x in a]
        stds=[statistics.stdev(x) if len(x)>1 else 0.0 for x in a]
        norms=[(r[1]**2+r[2]**2+r[3]**2)**0.5 for r in data]
        meanmag=statistics.mean(norms)
        if len(data)>1:
            dts=[(b[0]-a[0])/1000 for a,b in zip(data,data[1:]) if b[0]>a[0]]
            dupes=sum(a[1:4]==b[1:4] for a,b in zip(data,data[1:]))
        else:
            dts=[]; dupes=0
        cov=[]
        for i in range(3):
            cov.append([round(sum((a[i][k]-means[i])*(a[j][k]-means[j]) for k in range(len(data)))/(len(data)-1),3)
                        if len(data)>1 else 0.0 for j in range(3)])
        return {
            "count":len(data),
            "x_y_z_mean_counts":[round(x,3) for x in means],
            "x_y_z_std_counts":[round(x,3) for x in stds],
            "xyz_covariance_counts2":cov,
            "magnitude_mean_counts":round(meanmag,3),
            "magnitude_std_counts":round(statistics.stdev(norms),3) if len(norms)>1 else 0.0,
            "median_csv_interval_ms":round(statistics.median(dts),3) if dts else None,
            "largest_csv_interval_ms":round(max(dts),3) if dts else None,
            "duplicate_xyz_pairs":dupes,
            "verified_csv_rows":sum(x[4]==1 for x in data),
            "duration_s":round((data[-1][0]-data[0][0])/1e6,3),
        }
    out=describe(rows)
    n=max(1,len(rows)//4)
    early=describe(rows[:n]); late=describe(rows[-n:])
    out["first_vs_last_quartile_magnitude_delta_counts"]=(
        round(late["magnitude_mean_counts"]-early["magnitude_mean_counts"],3))
    ready_t_ms=None
    for line in lines:
        if line.startswith("WIFI_EVENT ") and "stage=READY" in line and "active=1" in line:
            k=kv(line)
            if num(k,"t_ms") is not None:
                ready_t_ms=float(k["t_ms"])
                break
    if ready_t_ms is not None:
        pre=[r for r in rows if r[0]/1000 < ready_t_ms]
        post=[r for r in rows if r[0]/1000 >= ready_t_ms]
        out["wifi_before"]=describe(pre)
        out["wifi_after"]=describe(post)
        if pre and post:
            out["wifi_magnitude_delta_counts"]=round(
                out["wifi_after"]["magnitude_mean_counts"]-out["wifi_before"]["magnitude_mean_counts"],3)
    out["caution"]="Compare noise/magnitude only with the drone motionless; hard/soft-iron calibration needs orientations."
    return out

def assess(lines: list[str], flags: dict[str,int], power_mode: str, capture: Capture) -> dict:
    states = [kv(l) for l in lines if l.startswith("STATE ")]
    health = [kv(l) for l in lines if l.startswith("HEALTH ")]
    events = [l for l in lines if l.startswith("WIFI_EVENT")]
    csv_count = sum(1 for l in lines if l.startswith("CSV,"))
    last = health[-1] if health else {}
    errors, warnings = [], []
    if not states:
        errors.append("No STATE signature received; cannot certify the running build")
    else:
        good = states[-1]
        if good.get("fw") != "18": errors.append("Firmware version is not diagnostic v18-compatible")
        for field, name in STATE_KEYS.items():
            if num(good, field) != flags[name]:
                errors.append(f"STATE {field} expected {flags[name]} got {good.get(field)}")
    if flags["IQ_MAG_START_ENABLE"] == 0:
        parked = (any(line.startswith("CONFIG_PARKED prestart_ok=1") for line in lines) or
                  any(x.get("prestart_ok")=="1" for x in states))
        if not parked: errors.append("No verified parked/prestart evidence")
        if not states: errors.append("No STATE signature in prestart-only run")
        return {"verdict": "PRESTART_ONLY PASS / NO ODR TEST" if not errors else "PRESTART_ONLY FAIL",
                "measurement_s":round(capture.elapsed_s,2),"stop_reason":capture.stop_reason,
                "power_classification":"WARM_RESET_NOT_POWER_CYCLE" if power_mode=="warm" else "POWER_SWITCHED",
                "serial_disconnected":capture.disconnected,"firmware_state":states[-1] if states else None,
                "state_messages":len(states),"health_windows":len(health),"csv_rows":csv_count,
                "median_last_8_health_hz":None,"selected_hz":flags["IQ_MAG_BOOT_PROFILE"],
                "final_health":last,"wifi_events":events,"wifi_median_delta_hz":None,
                "errors":errors,"warnings":["No magnetic measurement intentionally requested"],
                "xyz_diagnostics":xyz_diagnostics(lines)}
    if len(health) < 8: errors.append(f"Insufficient HEALTH windows ({len(health)})")
    if csv_count < 5: errors.append(f"Too few physical samples in CSV ({csv_count})")
    if capture.disconnected: errors.append("USB disconnected during measurement")
    if capture.stop_reason == "SERIAL_SILENCE_WATCHDOG": errors.append("Serial silent during measurement")
    if num(last,"regs_ok") != 1: errors.append("Last QMC register verification was not valid")
    if flags["IQ_MAG_SOFT_RESET"]:
        if num(last,"soft_reset_attempted") != 1 or num(last,"soft_reset_ok") != 1:
            errors.append("Requested QMC soft reset not confirmed in HEALTH readback")
        reset_messages = [l for l in lines if l.startswith("MAG_SOFT_RESET ")]
        if not any("stage=CONFIRMED" in l for l in reset_messages):
            warnings.append("Initial MAG_SOFT_RESET confirmation missed in UART capture; HEALTH flag must validate it")
    # 0x0A bit[1:0]=3 is continuous mode; ODR bits do not define 50/10Hz.
    continuous_mode=(flags["IQ_MAG_CTRL1_OVERRIDE"] != 0 and
                     (flags["IQ_MAG_CTRL1_OVERRIDE"] & 0x03) == 0x03)
    if continuous_mode:
        warnings.append("Continuous mode selected; do not interpret configured ODR bits as actual 10/50Hz")
    for field in ("verify_fail", "reset_signature", "status_fail", "xyz_fail"):
        val = num(last, field, 0)
        if val > 0:
            errors.append(f"{field} = {int(val)} (nonzero; inspect raw log)")
    if num(last,"fallback",0) != 0 or any("FALLBACK_100HZ" in l for l in lines):
        errors.append("Fallback invalidates selected-rate claim")
    if num(last,"recoveries",0) != 0:
        warnings.append("Recovery required; not a clean boot")
    if any("RESET_SIGNATURE:" in l for l in lines):
        errors.append("QMC registers appeared cleared; possible reset / bus issue")
    if any("FIRST_STATUS_FAILURE" in l for l in lines):
        warnings.append("First status-read failure timestamp available in raw log")
    recent = health[-min(len(health),8):]
    rates = [num(h,"measured_hz_window") for h in recent]
    rates = [r for r in rates if r is not None]
    med_hz = round(statistics.median(rates),2) if rates else None
    target = flags["IQ_MAG_BOOT_PROFILE"]
    if not continuous_mode:
        if med_hz is None or med_hz < 0.82*target:
            errors.append(f"Last-window median rate {med_hz} lower than 82% of {target}Hz")
        if med_hz is not None and med_hz > 1.30*target:
            warnings.append("Measured fresh rate >130% of target; check duplicate/stale sample handling")
    if flags["IQ_WIFI_STRESS_MODE"]:
        before = [num(h,"measured_hz_window") for h in health if h.get("wifi_active")=="0"]
        after = [num(h,"measured_hz_window") for h in health if h.get("wifi_active")=="1"]
        before = [x for x in before if x is not None]
        after = [x for x in after if x is not None]
        if len(before)<5 or len(after)<8: errors.append("Too few windows to compare WiFi-before and WiFi-after")
        if not any("stage=READY" in l and "active=1" in l for l in events):
            errors.append("No positive WiFi READY event")
        if before and after:
            b, a = statistics.median(before), statistics.median(after)
            if b >= 1 and a < b*0.85:
                errors.append(f"WiFi phase rate dropped {b:.1f} -> {a:.1f}Hz")
            wifi_delta = round(a-b, 3)
        else: wifi_delta = None
        if flags["IQ_WIFI_STRESS_MODE"]==2:
            if not any("stage=SCAN_DONE" in l for l in events):
                warnings.append("WiFi scan did not finish; no completed scan stress measurement")
    else:
        wifi_delta = None
    if flags["IQ_BUS_LOAD_ENABLE"]:
        if num(last,"imu_ok",0) < 10: warnings.append("Insufficient MPU stress-bus reads")
        if num(last,"bmp_ok",0) < 5: warnings.append("BMP stress reads did not succeed; verify address/device")
    # Heuristic triage, always labeled as an inference rather than a diagnosis.
    post_start_probes=[kv(l) for l in lines if l.startswith("POST_START_BUS ")]
    bus_hypothesis=None
    if post_start_probes:
        probe=post_start_probes[-1]
        def good(k):
            return probe.get(k,"").split(",",1)[0]=="1"
        if good("mag_stop") and not good("mag_rs"):
            bus_hypothesis="QMC responds to STOP+START but not repeated START; prioritize I2C transaction implementation"
        elif not good("mag_stop") and good("mpu_stop"):
            bus_hypothesis="Other I2C sensor can be read after QMC start; prioritize QMC state, bypass wiring and sensor VDD"
        elif not good("mag_stop") and not good("mpu_stop"):
            bus_hypothesis="Both QMC and MPU post-start reads fail; investigate shared I2C controller, lines and supply"
        elif good("mag_stop") and good("mag_rs"):
            bus_hypothesis="Both QMC register-read methods succeed in post-start probe; investigate polling/DRDY behavior"
    failure = bool(errors)
    if not states or capture.stop_reason in ("SERIAL_SILENCE_WATCHDOG","USB_DISCONNECTED") or len(health)<8:
        verdict = "INCONCLUSIVE / TRANSPORT FAILURE"
    elif failure:
        verdict = "FAIL / NEEDS INVESTIGATION"
    elif warnings:
        verdict = "PASS WITH WARNINGS (PROVISIONAL)"
    else:
        verdict = "PROVISIONAL PASS"
    if power_mode == "warm" and verdict.startswith(("PASS", "PROVISIONAL")):
        verdict += " / WARM-BOOT ONLY"
    return {
        "verdict":verdict,"measurement_s":round(capture.elapsed_s,2),"stop_reason":capture.stop_reason,
        "power_classification": "COLD_BOOT_REQUESTED_NEEDS_POWER_RAIL_VERIFICATION" if power_mode!="warm" else "WARM_RESET_NOT_POWER_CYCLE",
        "serial_disconnected":capture.disconnected,"firmware_state":states[-1] if states else None,
        "state_messages":len(states), "health_windows":len(health), "csv_rows":csv_count,
        "median_last_8_health_hz":med_hz,"selected_hz":target,
        "final_health":last,"wifi_events":events,"wifi_median_delta_hz":wifi_delta,
        "errors":errors,"warnings":warnings,
        "post_start_probes":post_start_probes,
        "post_start_inference":bus_hypothesis,
        "xyz_diagnostics":xyz_diagnostics(lines),
    }


class EventLogger:
    def __init__(self, file: Path):
        file.parent.mkdir(parents=True, exist_ok=True)
        self.f = file.open("a",encoding="utf-8",buffering=1)
    def __call__(self, label, **fields):
        timestamp=datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        evt={"time_utc":timestamp,"event":label,**fields}
        self.f.write(json.dumps(evt,ensure_ascii=True)+"\n")
        if label == "FW":
            msg = fields.get("text","")
            if msg.startswith("STATE "): return
            if msg.startswith("VERIFY stage=PERIODIC"): return
            print("   ", msg[:180],flush=True)
        elif label == "HEALTH":
            print(f"    [{fields.get('windows')}s windows] {fields.get('hz')}Hz fresh={fields.get('fresh')} reg={fields.get('regs_ok')} I2C_fail={fields.get('status_fail')} wifi={fields.get('wifi')}",flush=True)
        else:
            print("   ",label," ".join(f"{k}={v}" for k,v in fields.items()),flush=True)
    def close(self): self.f.close()


class PowerSwitchError(RuntimeError):
    """External power control failure; abort remaining tests for safety."""


class PowerController:
    def __init__(self, args, log):
        self.args=args
        self.log=log
        self.switch_off=False
        if args.power_mode == "ykush" and not (Path(args.ykush_exe).exists() or shutil.which(args.ykush_exe)):
            raise PowerSwitchError("YKUSH control utility not found: " + args.ykush_exe)
        self.commands=None
        if args.power_mode == "command":
            data=json.loads(Path(args.power_commands).read_text(encoding="utf-8"))
            for name in ("off", "on"):
                if not isinstance(data.get(name),list) or not data[name] or not all(isinstance(x,str) and x for x in data[name]):
                    raise ValueError("Power command JSON must contain nonempty string arrays: off and on")
            self.commands=data
    def switch(self, turn_on):
        if self.args.power_mode == "warm":
            self.log("POWER_MODE", classification="WARM_RESET_FROM_UPLOAD", verified_power_off=False)
            return
        if self.args.power_mode == "ykush":
            cmd=[self.args.ykush_exe]
            if self.args.ykush_serial: cmd += ["-s",self.args.ykush_serial]
            cmd += ["-u" if turn_on else "-d",str(self.args.ykush_port)]
        else:
            cmd=self.commands["on" if turn_on else "off"]
        self.log("POWER_COMMAND", action="ON" if turn_on else "OFF", argv=cmd)
        try:
            completed=subprocess.run(cmd, cwd=ROOT, capture_output=True,text=True,errors="replace", timeout=30,check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise PowerSwitchError(f"Power switch command execution failed: {exc}") from exc
        self.log("POWER_COMMAND_RESULT", action="ON" if turn_on else "OFF", rc=completed.returncode,
                 stdout=completed.stdout[-1200:], stderr=completed.stderr[-1200:])
        if completed.returncode:
            raise PowerSwitchError(f"Power command returned {completed.returncode}: {cmd}")
        self.switch_off = not turn_on
    def cold_boot(self, info, expected_port):
        if self.args.power_mode == "warm":
            self.log("POWER_CLASS", mode="WARM", description="PlatformIO/ESP reset does not switch sensor VDD")
            return
        self.switch(False)
        start=time.monotonic()
        while time.monotonic()-start < self.args.off_detection_sec:
            if locate_device(info,expected_port) is None:
                self.log("POWER_OFF_EVIDENCE", port_disappeared=True)
                break
            time.sleep(.2)
        else:
            # Prevent a false claim of physical USB removal; power restored in finally.
            raise PowerSwitchError("USB port did not disappear after OFF; cannot confirm controlled USB rail switched")
        remaining = self.args.off_seconds-(time.monotonic()-start)
        if remaining>0: time.sleep(remaining)
        self.log("POWER_OFF_HOLD", seconds=round(time.monotonic()-start,2), sensor_vdd_measured=False)
        self.switch(True)
        self.log("POWER_ON_REQUESTED", port=expected_port)
    def rescue(self):
        if self.switch_off:
            try:
                self.switch(True)
                self.log("POWER_RESCUE", status="restored switch to ON")
            except Exception as exc:
                self.log("POWER_RESCUE_FAILED", error=str(exc))


class UploadPortBusyError(RuntimeError):
    """The Windows COM port is unavailable; abort subsequent invalid tests."""


def execute_pio(pio: str, port: str, logpath: Path, event, attempts: int = 4):
    cmd=[pio,"run","-e",ENV,"-t","upload","--upload-port",port]
    for attempt in range(1,attempts+1):
        event("UPLOAD_START", argv=cmd, attempt=attempt)
        with logpath.open("a",encoding="utf-8",errors="replace") as f:
            f.write(f"\n===== UPLOAD ATTEMPT {attempt}/{attempts} =====\n")
            f.flush()
            p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=False)
        event("UPLOAD_END", rc=p.returncode, attempt=attempt, build_log=str(logpath))
        if p.returncode==0: return
        tail="\n".join(logpath.read_text(encoding="utf-8",errors="replace").splitlines()[-30:])
        busy = ("PermissionError(13" in tail or "Access is denied" in tail or
                "port is busy" in tail or "could not open port" in tail.lower())
        if not busy:
            raise RuntimeError(f"PlatformIO build/upload failed (non-port error):\n{tail}")
        if attempt==attempts:
            raise UploadPortBusyError(
                f"COM upload remained inaccessible after {attempts} attempts. "
                "Stop testing; close other serial monitors, check CH343 connection.\n"+tail)
        pause = (2,4,8)[min(attempt-1,2)]
        event("PORT_BUSY_UPLOAD_RETRY", port=port, attempt=attempt, delay_seconds=pause)
        time.sleep(pause)


def save_json(p:Path,obj):
    p.write_text(json.dumps(obj,indent=2,ensure_ascii=True)+"\n", encoding="utf-8")


def summary_md(rows:list[dict], session_dir:Path):
    lines=["IntelQCopter v18 — unattended diagnostic summary",
           f"Session: {session_dir.name}","",
           "IMPORTANT: only external switching may remove sensor power; measure 3.3V for proof.",
           "WARM-BOOT success is not a valid cold-power-up comparison.","",
           "ID   | Verdict                              | Median Hz | I2C status failures | Power", "-----|--------------------------------------|-----------|---------------------|------"]
    for x in rows:
        s=x.get("summary",{})
        h=s.get("final_health",{}) or {}
        lines.append(f"{x['test']:<4} | {s.get('verdict','ERROR'):<36} | {str(s.get('median_last_8_health_hz','-')):<9} | {h.get('status_fail','-'):<19} | {x['power_mode']}")
    lines += ["","Full UART raw logs, firmware upload logs, settings, timestamps and errors are in the runs/ subdirectory."]
    (session_dir/"SUMMARY.txt").write_text("\n".join(lines)+"\n",encoding="utf-8")
    with (session_dir/"SUMMARY.csv").open("w",encoding="utf-8",newline="") as f:
        wr=csv.writer(f); wr.writerow(["test","repeat","verdict","rate_med_hz","status_fail","verify_fail","csv_rows","power","run_dir"])
        for r in rows:
            s=r.get("summary",{}) or {};h=s.get("final_health",{}) or {}
            wr.writerow([r["test"],r["repeat"],s.get("verdict","ERROR"),s.get("median_last_8_health_hz"),
                         h.get("status_fail"),h.get("verify_fail"),s.get("csv_rows"),r["power_mode"],r["run_dir"]])


def bundle(session_dir:Path):
    archive=session_dir.parent/(session_dir.name+"_SEND_THIS.zip")
    with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(session_dir.rglob("*")):
            if p.is_file(): z.write(p,arcname=str(Path(session_dir.name)/p.relative_to(session_dir)))
    return archive


def sequence(args):
    if args.tests:
        chosen=[x.strip().upper() for x in args.tests.replace(","," ").split() if x.strip()]
    else: chosen=SUITES[args.suite].split()
    unknown=[x for x in chosen if x not in PRESETS]
    if unknown: raise ValueError("Unknown test ID(s): "+", ".join(unknown))
    if not chosen: raise ValueError("Test sequence is empty")
    return chosen


def cli(argv=None):
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--port",default="",help="COM3; omitted = auto-detect unique CH343")
    p.add_argument("--suite",choices=SUITES,default="triage")
    p.add_argument("--tests",default="",help="Override suite with IDs e.g. A0,F1,B1,F2")
    p.add_argument("--repeat",type=int,default=1,help="Repeat each preset with independent new uploads")
    p.add_argument("--power-mode",choices=("warm","ykush","command"),default="warm")
    p.add_argument("--ykush-exe",default="ykush.exe",help="Path to Windows YKUSH control CLI")
    p.add_argument("--ykush-port",type=int,choices=(1,2,3),default=1)
    p.add_argument("--ykush-serial",default="",help="Identify hub if more than one YKUSH connected")
    p.add_argument("--power-commands",default="",help="JSON path for external off/on commands")
    p.add_argument("--off-seconds",type=float,default=10.0,help="Minimum true switched-off hold")
    p.add_argument("--off-detection-sec",type=float,default=12.0,help="Timeout to verify USB COM disappears when switch is OFF")
    p.add_argument("--connect-timeout",type=float,default=0.0,help="Port wait watchdog; 0=no limit (Ctrl+C stops)")
    p.add_argument("--min-sec",type=float,default=25.0,help="Minimum measurement period")
    p.add_argument("--max-sec",type=float,default=120.0,help="Measurement watchdog maximum; separate from port wait")
    p.add_argument("--min-health",type=int,default=24,help="Minimum HEALTH windows for adaptive completion")
    p.add_argument("--pre-wifi-health",type=int,default=5)
    p.add_argument("--post-wifi-health",type=int,default=20)
    p.add_argument("--silence-sec",type=float,default=18,help="No-byte watchdog when USB is open")
    p.add_argument("--pio",default="",help="PlatformIO executable path")
    p.add_argument("--list-ports",action="store_true")
    p.add_argument("--list-tests",action="store_true")
    p.add_argument("--dry-run",action="store_true",help="No hardware or filesystem changes")
    p.add_argument("--stop-on-error",action="store_true")
    p.add_argument("--result-dir",default="",help="Custom result root (default project/results)")
    args=p.parse_args(argv)
    if args.repeat<1 or args.repeat>50: p.error("--repeat must be 1..50")
    if args.min_sec<=0 or args.max_sec<args.min_sec: p.error("0 < min-sec <= max-sec required")
    if args.off_seconds<1 or args.off_detection_sec<1: p.error("power-off delays must be >=1 second")
    if args.connect_timeout<0 or args.silence_sec<=0: p.error("invalid timeout")
    if args.power_mode=="command" and not args.power_commands: p.error("--power-commands required for command mode")
    return args


def main(argv=None) -> int:
    args=cli(argv)
    if args.list_tests:
        for k,(name,delta) in PRESETS.items(): print(f"{k}: {name}  {json.dumps(delta)}")
        return 0
    if args.list_ports:
        try: describe_ports(); return 0
        except ImportError: print("pyserial missing; install requirements.txt",file=sys.stderr); return 2
    seq=sequence(args)
    print("IntelQCopter v18 unattended QMC5883P suite — NO GUI / NO MANUAL TEST PROMPTS")
    print("Sequence:"," ".join(seq),"x",args.repeat)
    print("Power mode:",args.power_mode, "(only external switching can attempt real cold boots)")
    print("Adaptive evidence:",args.min_health,"HEALTH windows; maximum",args.max_sec,"s measurement")
    print("COM connection waiting:","unlimited" if not args.connect_timeout else f"{args.connect_timeout}s")
    print("SAFETY: REMOVE PROPELLERS and physically DISCONNECT ESC signal/power wiring.")
    if args.dry_run:
        for t in seq:
            print(f"  {t:<3} {PRESETS[t][0]}  {json.dumps(flags_for(t))}")
        return 0
    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial missing. Run: py -3 -m pip install pyserial",file=sys.stderr)
        return 2
    pio=platformio_cmd(args.pio)
    device=choose_device(args.port)
    root=Path(args.result_dir) if args.result_dir else RESULTS
    root.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir=root/(f"SESSION_{stamp}_{os.getpid()}")
    session_dir.mkdir(parents=True,exist_ok=False)
    log=EventLogger(session_dir/"events.jsonl")
    original=INI.read_text(encoding="utf-8")
    (session_dir/"platformio.original.ini").write_text(original,encoding="utf-8")
    manifest={"version":"18","started":datetime.now(timezone.utc).isoformat(),
              "selected_tests":seq,"repeat":args.repeat,"power_mode":args.power_mode,
              "hardware_verified":False, "device":{
                  "initial_port":device.device,"vid":device.vid,"pid":device.pid,
                  "serial_number":device.serial_number,"location":device.location},
              "args":{k:v for k,v in vars(args).items() if not k.startswith("_")},
              "runs":[],"complete":False}
    results=[]
    ctrl=None
    active_port=device.device
    try:
        ctrl=PowerController(args,log)
        log("SESSION_START",count=len(seq)*args.repeat,serial_port=active_port)
        for idx, pid in enumerate(seq,1):
            for rep in range(1,args.repeat+1):
                nonce=random.SystemRandom().randint(10000000,2000000000)
                flags=flags_for(pid,nonce)
                rundir=session_dir/"runs"/(f"{idx:02}_{pid}_rep{rep}")
                rundir.mkdir(parents=True)
                meta={"test":pid,"label":PRESETS[pid][0],"repeat":rep,
                      "power_mode":args.power_mode,"run_dir":str(rundir.relative_to(session_dir)),
                      "settings":flags,"nonce":nonce,"port_before":active_port,
                      "started":datetime.now(timezone.utc).isoformat()}
                (rundir/"firmware_options.ini").write_text(render_ini(original,flags),encoding="utf-8")
                save_json(rundir/"settings.json",meta)
                print(f"\n========= [{idx}/{len(seq)}] {pid} repeat {rep}/{args.repeat}: {PRESETS[pid][0]} =========",flush=True)
                log("TEST_START",id=pid,repeat=rep,baud=flags["IQ_SERIAL_BAUD"],rate=flags["IQ_MAG_BOOT_PROFILE"],nonce=nonce)
                try:
                    INI.write_text(render_ini(original,flags),encoding="utf-8")
                    execute_pio(pio,active_port,rundir/"platformio_build_upload.log",log)
                    ctrl.cold_boot(device, active_port)
                    ser, active_port=wait_for_device(device,active_port,args.connect_timeout,log,flags["IQ_SERIAL_BAUD"])
                    log("SERIAL_OPEN",port=active_port,baud=flags["IQ_SERIAL_BAUD"])
                    cap=capture_stream(ser,active_port,rundir/"uart_raw.log",args,flags["IQ_WIFI_STRESS_MODE"],log)
                    summary=assess(cap.lines,flags,args.power_mode,cap)
                    meta["summary"]=summary
                    meta["port_after"]=active_port
                    print("  >>>",summary["verdict"],"median rate",summary["median_last_8_health_hz"],"Hz")
                    for s in summary["errors"][:8]: print("      !",s)
                    for s in summary["warnings"][:5]: print("      ~",s)
                except KeyboardInterrupt:
                    raise
                except Exception as ex:
                    meta["summary"]={"verdict":"ERROR / TEST NOT COMPLETED","errors":[str(ex)],"traceback":traceback.format_exc()}
                    (rundir/"error.txt").write_text(traceback.format_exc(),encoding="utf-8")
                    log("TEST_ERROR",test=pid,repeat=rep,error=str(ex))
                    print("  >>> ERROR:",str(ex),flush=True)
                    if args.stop_on_error or isinstance(ex, (PowerSwitchError, UploadPortBusyError)):
                        results.append(meta)
                        manifest["runs"]=results
                        save_json(rundir/"summary.json",meta)
                        raise
                finally:
                    ctrl.rescue()
                    meta["finished"]=datetime.now(timezone.utc).isoformat()
                    save_json(rundir/"summary.json",meta)
                    results.append(meta) if meta not in results else None
                    manifest["runs"]=results
                    save_json(session_dir/"manifest.json",manifest)
                # Retain runs and try subsequent upload after a test failure.
        manifest["complete"]=True
        log("SESSION_COMPLETE",runs=len(results))
        return 0
    except KeyboardInterrupt:
        log("SESSION_INTERRUPTED",reason="Ctrl+C")
        return 130
    except Exception as ex:
        log("SESSION_ERROR",detail=str(ex),trace=traceback.format_exc())
        print("SESSION ERROR:",ex,file=sys.stderr)
        return 1
    finally:
        if ctrl is not None: ctrl.rescue()
        INI.write_text(original,encoding="utf-8")
        manifest["finished"]=datetime.now(timezone.utc).isoformat()
        save_json(session_dir/"manifest.json",manifest)
        summary_md(results,session_dir)
        log("RESULTS_READY",results=len(results))
        log.close()
        archive=bundle(session_dir)
        print("\nRESULTS FOLDER:",session_dir)
        print("SHARE THIS SINGLE ZIP:",archive)
        print("platformio.ini restored to its original contents.")

if __name__=="__main__":
    raise SystemExit(main())
