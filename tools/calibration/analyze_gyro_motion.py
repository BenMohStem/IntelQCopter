"""Review a verified operator rotation; never changes calibration or firmware."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import math
import struct


ROOT = Path(__file__).resolve().parents[2]
SCALE_RAD_PER_COUNT = math.pi / (180.0 * 65.5)


def fnv1a_rows(rows):
    value = 2166136261
    for row in rows:
        for byte in struct.pack('>7h', *row):
            value = ((value ^ byte) * 16777619) & 0xffffffff
    return f'{value:08X}'


def verify_serial_capture(serial_bytes, rows, summary):
    try:
        lines = serial_bytes.decode('ascii').splitlines()
        start = lines.index('BEGIN,20000')
        end = next(i for i in range(start + 1, len(lines)) if lines[i].startswith('END,'))
        trailer = lines[end].split(',')
        samples = [line.split(',') for line in lines[start + 1:end] if line.startswith('S,')]
        if len(samples) != len(rows) or len(trailer) != 8:
            raise ValueError('Wrong sample or trailer count')
        for i, fields in enumerate(samples):
            if len(fields) != 9 or int(fields[1]) != i or [int(x) for x in fields[2:]] != rows[i]:
                raise ValueError('Saved samples differ from original serial records')
        if int(trailer[1]) != len(rows) or int(trailer[2]) != summary['elapsed_us']:
            raise ValueError('Capture duration differs from original serial trailer')
        if trailer[7].upper() != summary['fnv1a_raw_payload'].upper():
            raise ValueError('Checksum differs from original serial trailer')
        if not math.isclose(summary['observed_capture_rate_hz'], len(rows) * 1e6 / int(trailer[2]), rel_tol=1e-10):
            raise ValueError('Rate differs from original serial trailer')
        if f"HELLO,FIFO1000_V2,{summary['source_sha256']}" not in lines:
            raise ValueError('Source hash differs from original serial header')
        for axis in range(3):
            rotation = [line.split(',') for line in lines[:start] if line.startswith(f'GYRO_ROT,{axis},')]
            trim = [line.split(',') for line in lines[:start] if line.startswith(f'GYRO_TRIM,{axis},')]
            if not rotation or not trim:
                raise ValueError('Missing original gyro frame or trim')
            if any(abs(float(rotation[-1][j + 2]) - summary['profile']['gyro_to_drone'][axis][j]) > 1e-9 for j in range(3)):
                raise ValueError('Gyro frame differs from original serial header')
            if abs(float(trim[-1][4]) - summary['profile']['gyro_remaining_bias'][axis]) > 1e-9:
                raise ValueError('Gyro trim differs from original serial header')
    except (UnicodeDecodeError, StopIteration, KeyError, IndexError, TypeError, ValueError) as error:
        raise ValueError(f'Original serial evidence does not match the saved capture: {error}') from error


def calculate_rates(rows, matrix, bias, rate):
    """Return component integrals, peak rates and ten-sample means."""
    dt = 1.0 / rate
    sums = [0.0, 0.0, 0.0]
    peaks = [0.0, 0.0, 0.0]
    bucket = [0.0, 0.0, 0.0]
    plot = []
    for sequence, row in enumerate(rows):
        sensor = [row[i + 4] * SCALE_RAD_PER_COUNT - bias[i] for i in range(3)]
        body = [sum(matrix[i][j] * sensor[j] for j in range(3)) for i in range(3)]
        for i in range(3):
            sums[i] += body[i] * dt
            peaks[i] = max(peaks[i], abs(body[i]))
            bucket[i] += math.degrees(body[i])
        if (sequence + 1) % 10 == 0:
            plot.append([(sequence + 1) * dt] + [value / 10 for value in bucket])
            bucket = [0.0, 0.0, 0.0]
    return sums, peaks, plot


def analyze(recording, axis, direction, reference_deg=None):
    recording = Path(recording)
    summary = json.loads((recording / 'summary.json').read_text(encoding='utf-8'))
    rows_file = recording / 'samples.json'
    rows = json.loads(rows_file.read_text(encoding='utf-8'))['raw_post_trim_words']
    serial_file = recording / 'serial.log'

    if summary.get('operator_condition') != 'operator_rotation':
        raise ValueError('Capture must be labelled operator_rotation; do not use stationary data as a rotation.')
    if summary.get('status') != 'CAPTURE_COMPLETE_REVIEW_REQUIRED' or not summary.get('final_header_unchanged'):
        raise ValueError('Capture is incomplete, failed or has changed final metadata.')
    serial_bytes = serial_file.read_bytes()
    if hashlib.sha256(serial_bytes).hexdigest() != summary.get('serial_sha256'):
        raise ValueError('Original serial log hash does not match its capture summary.')
    if len(rows) != 20000 or any(len(row) != 7 or any(not isinstance(x, int) or not -32768 <= x <= 32767 for x in row) for row in rows):
        raise ValueError('Expected 20,000 complete signed 16-bit seven-word FIFO samples.')
    if fnv1a_rows(rows) != summary.get('fnv1a_raw_payload', '').upper():
        raise ValueError('Raw sample checksum does not match the capture trailer.')
    rate = summary.get('observed_capture_rate_hz')
    if not isinstance(rate, (int, float)) or not math.isfinite(rate) or not 980 <= rate <= 1020:
        raise ValueError('Capture average rate is invalid or outside the expected 1 kHz range.')
    verify_serial_capture(serial_bytes, rows, summary)
    profile = summary['profile']
    matrix = profile['gyro_to_drone']
    bias = profile['gyro_remaining_bias']
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix) or len(bias) != 3:
        raise ValueError('Capture lacks the complete gyro frame or residual bias.')
    if any(not math.isfinite(x) for row in matrix for x in row) or any(not math.isfinite(x) for x in bias):
        raise ValueError('Gyro frame or residual bias contains a non-finite value.')
    if axis not in ('x', 'y', 'z') or direction not in ('positive', 'negative'):
        raise ValueError('Specify drone axis x/y/z and the operator-observed direction.')
    if reference_deg is not None and (not math.isfinite(reference_deg) or reference_deg <= 0):
        raise ValueError('Reference angle must be a positive independently measured magnitude.')

    # MPU samples have no individual timestamps. Use the observed capture-average
    # interval; this cannot reconstruct per-sample timing or rotation ground truth.
    dt = 1.0 / rate
    sums, peaks, plot = calculate_rates(rows, matrix, bias, rate)

    angles = [math.degrees(value) for value in sums]
    expected_index = 'xyz'.index(axis)
    sign = 1 if direction == 'positive' else -1
    expected_component = sign * angles[expected_index]
    transverse = math.hypot(*(angles[i] for i in range(3) if i != expected_index))
    dominant_index = max(range(3), key=lambda i: abs(angles[i]))
    result = {
        'status': 'REVIEW_REQUIRED_NOT_PHYSICAL_ACCEPTANCE',
        'engine': 'PC replay of recorded FIFO-format samples',
        'capture_engine': summary.get('engine', 'unspecified'),
        'source_sha256': summary.get('source_sha256'),
        'serial_sha256': summary['serial_sha256'],
        'samples_sha256': hashlib.sha256(rows_file.read_bytes()).hexdigest(),
        'samples': len(rows),
        'observed_rate_hz': rate,
        'average_interval_seconds': dt,
        'operator_axis': axis,
        'operator_direction': direction,
        'operator_note': summary.get('operator_note', ''),
        'integrated_body_rate_degrees': dict(zip('xyz', angles)),
        'peak_abs_body_rate_deg_s': dict(zip('xyz', [math.degrees(v) for v in peaks])),
        'expected_signed_component_deg': expected_component,
        'transverse_integral_magnitude_deg': transverse,
        'dominant_integral_axis': 'xyz'[dominant_index],
        'reference_magnitude_deg': reference_deg,
        'reference_error_deg': None if reference_deg is None else expected_component - reference_deg,
        'plot_100hz_deg_s': plot,
        'gyro_calibration_changed': False,
        'physical_axis_validated': False,
        'limits': [
            'Operator labels and any independent reference must be checked by a person.',
            'Net integrated body rates are meaningful as a rotation-angle comparison only for an approximately fixed single-axis rotation.',
            'Opposite movements can cancel; review the motion and raw time series.',
            'FIFO samples have no individual timestamps; the average interval is approximate.',
            'Residual gyro bias, temperature drift, cross-axis coupling and reference uncertainty are not included in an uncertainty bound.',
            'These results do not validate gyro scale, attitude estimation, PID gains or flight safety.',
        ],
    }
    return result


def make_plot(points, title='Gyro rate in drone axes — 10-sample averages'):
    width, height = 1000, 430
    left, top, plot_width, plot_height = 80, 55, 870, 300
    duration = points[-1][0]
    maximum = max(0.1, max(abs(value) for point in points for value in point[1:]))
    maximum *= 1.1
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="80" y="26" font-family="sans-serif" font-size="18">{title}</text>']
    for i in range(5):
        y = top + i * plot_height / 4
        value = maximum * (1 - i / 2)
        parts.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#d8e0e8"/>')
        parts.append(f'<text x="{left - 8}" y="{y + 5:.2f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:.1f}</text>')
    for i in range(5):
        x = left + i * plot_width / 4
        parts.append(f'<text x="{x:.2f}" y="{top + plot_height + 22}" text-anchor="middle" font-family="sans-serif" font-size="12">{duration * i / 4:.1f}</text>')
    for axis, color in enumerate(['#cf3e3e', '#16845b', '#315fc4'], 1):
        coordinates = ' '.join(f'{left + p[0] / duration * plot_width:.2f},{top + (1 - p[axis] / maximum) * plot_height / 2:.2f}' for p in points)
        parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="1.4"/>')
        parts.append(f'<text x="{left + (axis - 1) * 110}" y="410" fill="{color}" font-family="sans-serif" font-size="14">Drone {"xyz"[axis - 1]}</text>')
    parts.append(f'<text x="{left + plot_width / 2}" y="{height - 8}" font-family="sans-serif" font-size="13">Time (seconds); vertical axis: degrees/second</text>')
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', required=True, type=Path, help='Completed record-1khz evidence folder')
    parser.add_argument('--axis', required=True, choices=['x', 'y', 'z'], help='Drone axis intentionally rotated')
    parser.add_argument('--direction', required=True, choices=['positive', 'negative'], help='Observed right-hand-rule rotation direction')
    parser.add_argument('--reference-deg', type=float, help='Optional independent reference angle magnitude in degrees')
    parser.add_argument('--output', type=Path, help='New output folder; defaults to a timestamped artifacts folder')
    args = parser.parse_args()
    result = analyze(args.recording, args.axis, args.direction, args.reference_deg)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output = args.output or ROOT / 'artifacts/gyro-motion-analysis' / stamp
    output.mkdir(parents=True, exist_ok=False)
    (output / 'analysis.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    (output / 'gyro-rates.svg').write_text(make_plot(result['plot_100hz_deg_s']), encoding='utf-8')
    print('Integrated drone-axis rates, degrees:', result['integrated_body_rate_degrees'])
    print('Expected-direction component, degrees:', round(result['expected_signed_component_deg'], 3))
    print('Other-axis integral magnitude, degrees:', round(result['transverse_integral_magnitude_deg'], 3))
    print('Review required; no physical axis or scale acceptance is inferred.')
    print('Saved:', output / 'analysis.json')
    print('Graph:', output / 'gyro-rates.svg')


if __name__ == '__main__':
    main()
