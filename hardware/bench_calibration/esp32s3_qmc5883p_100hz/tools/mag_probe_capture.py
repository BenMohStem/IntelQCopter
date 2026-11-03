#!/usr/bin/env python3
"""Record original v13 QMC5883P 100Hz text stream. No commands sent."""
import argparse
from datetime import datetime
from pathlib import Path
import json
import re
import time


CSV = re.compile(r'^CSV,(\d+),(-?\d+),(-?\d+),(-?\d+),0x([0-9A-Fa-f]{2}),([01])$')
RATE = re.compile(r'measured_fresh_hz=([\d.]+)')


def extract_summary(lines):
    samples = []
    health_rates = []
    verifies = []
    errors = []
    profiles = []
    for line in lines:
        match = CSV.match(line)
        if match:
            samples.append([int(match.group(i)) for i in range(1, 5)])
        if line.startswith('HEALTH '):
            m = RATE.search(line)
            if m:
                health_rates.append(float(m.group(1)))
        if line.startswith('VERIFY '):
            verifies.append(line)
        if 'FIRST_BUS_READ_FAILURE' in line or 'FRESH_MISSING' in line:
            errors.append(line)
        if line.startswith('BOOT_PROFILE '):
            profiles.append(line)
    if len(samples) > 1 and samples[-1][0] > samples[0][0]:
        hz = (len(samples) - 1) * 1e6 / (samples[-1][0] - samples[0][0])
    else:
        hz = 0.0
    return {
        'csv_samples': len(samples), 'csv_rate_hz': round(hz, 2),
        'last_health_hz': health_rates[-1] if health_rates else None,
        'verify_lines': verifies, 'errors': errors,
        'boot_profiles': profiles,
        'first_sample': samples[0] if samples else None,
        'last_sample': samples[-1] if samples else None,
    }


def run(port, seconds, baud):
    try:
        import serial
    except ImportError:
        print('Missing pyserial: py -3 -m pip install pyserial')
        return 6
    capture_dir = Path('captures')
    capture_dir.mkdir(parents=True, exist_ok=True)
    stem = 'mag_probe_v13_100hz_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    raw_path = capture_dir / (stem + '.log')
    json_path = capture_dir / (stem + '.json')
    lines = []
    print(f'MAG-ONLY v13 source at {baud} baud on {port}. No serial commands sent.')
    print(f'Capturing for {seconds:g}s. No motors, Wi-Fi or GUI.')
    try:
        with serial.Serial(port=port, baudrate=baud, timeout=0.15,
                           rtscts=False, dsrdtr=False) as uart:
            uart.reset_input_buffer()
            start = time.monotonic()
            with raw_path.open('w', encoding='utf-8', newline='\n') as output:
                last_health_print = ''
                while time.monotonic() - start < seconds:
                    data = uart.readline()
                    if not data:
                        continue
                    line = data.decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    lines.append(line)
                    output.write(line + '\n')
                    if line.startswith(('BOOT_PROFILE', 'MAG_ID', 'BOOT_CONFIG',
                                        'VERIFY', 'FIRST_BUS_READ_FAILURE', 'FRESH_MISSING')):
                        print(line)
                    elif line.startswith('HEALTH '):
                        if line != last_health_print:
                            print(line)
                        last_health_print = line
    except (serial.SerialException, OSError) as error:
        print(f'ERROR: COM port read failed: {error}')
        print('Close all other serial applications and retry. Raw output:', raw_path)
        return 5
    summary = extract_summary(lines)
    json_path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print('RESULT', f"fresh_csv={summary['csv_samples']}",
          f"measured_csv_hz={summary['csv_rate_hz']}",
          f"last_health_hz={summary['last_health_hz']}")
    print('Log:', raw_path, 'Summary:', json_path)
    if summary['csv_samples'] == 0:
        print('FAIL: Even the historically working v13 MAG-only source produced zero samples.')
        print('Check a TRUE cold power cycle, QMC 3.3V, soldering, shared I2C, and MPU bypass.')
        return 2
    if summary['csv_rate_hz'] < 60:
        print('WARN: Sensor produced samples but did not sustain a useful rate.')
        return 3
    print('PASS MAG-ONLY: This does NOT prove full calibration MAG works.')
    print('Next: reflash full firmware and compare with -Check.')
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--baud', type=int, default=115200)
    a = parser.parse_args()
    raise SystemExit(run(a.port, a.seconds, a.baud))
