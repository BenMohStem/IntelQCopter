"""Source-expression numerical check, NOT an ESP32 or sensor execution test."""
import ast
import ctypes
import datetime
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'firmware/main.cpp'
text = SOURCE.read_text()
fit = json.loads((ROOT / 'measurements/mpu6050/2026-10-24-six-pose/calculation.json').read_text())
stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
folder = ROOT / 'artifacts/calibration-check' / stamp
folder.mkdir(parents=True)
lines = []
report = dict(engine='Python source-expression harness with IEEE float32 rounding',
              source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              hardware_test=False, uploaded=False, results=[])

def f32(value):
    return ctypes.c_float(value).value

def expression(node, values):
    if isinstance(node, ast.Constant):
        return f32(node.value)
    if isinstance(node, ast.Name):
        return f32(values[node.id])
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return f32(-expression(node.operand, values))
    if isinstance(node, ast.BinOp):
        left = expression(node.left, values)
        right = expression(node.right, values)
        if isinstance(node.op, ast.Add):
            return f32(left + right)
        if isinstance(node.op, ast.Sub):
            return f32(left - right)
        if isinstance(node.op, ast.Mult):
            return f32(left * right)
    raise ValueError('Unsupported firmware expression')

def source_expression(name):
    match = re.search(r'float\s+' + name + r'\s*=\s*([^;]+);', text)
    assert match, 'Missing firmware expression: ' + name
    value = re.sub(r'acceleration\.acceleration\.([xyz])', r'raw_\1', match.group(1))
    value = re.sub(r'(?<=\d)f\b', '', value)
    return ast.parse(value, mode='eval').body

try:
    assert '#include <Adafruit_MPU6050.h>' in text
    assert 'mpu.getAccelerometerSensor()->getEvent(&acceleration);' in text
    assert 'setXAccelOffset' not in text and 'apply_accel_offsets' not in text
    expressions = {name: source_expression(name) for name in
                   ('accel_bias_x','accel_bias_y','accel_bias_z','sensor_x','sensor_y','sensor_z','drone_x','drone_y','drone_z')}
    for pose, raw in fit['raw_means_m_s2'].items():
        values = {'raw_' + axis: f32(raw[i]) for i,axis in enumerate('xyz')}
        for name, tree in expressions.items():
            values[name] = expression(tree, values)
        actual = [values['drone_' + axis] for axis in 'xyz']
        expected = fit['training_residuals'][pose]['corrected_drone_m_s2']
        difference = max(abs(a-b) for a,b in zip(actual,expected))
        assert difference < 0.00001, (pose, actual, expected)
        row = dict(pose=pose, corrected=actual, reference=expected, max_expression_difference=difference)
        report['results'].append(row)
        lines.append(f'{pose}: corrected={actual}; max difference from double-precision fit={difference:.9g}')
    # Deliberately incorrect variants must not reproduce the reference fit.
    for bias_times in (0, 2):
        worst_difference = 0.0
        for pose, raw in fit['raw_means_m_s2'].items():
            values = {'sensor_' + axis: f32(raw[i] - bias_times * fit['bias_sensor_m_s2'][i]) for i,axis in enumerate('xyz')}
            actual = [expression(expressions['drone_' + axis], values) for axis in 'xyz']
            expected = fit['training_residuals'][pose]['corrected_drone_m_s2']
            worst_difference = max(worst_difference, max(abs(a-b) for a,b in zip(actual,expected)))
        assert worst_difference > 0.1
        lines.append(f'Negative control, bias applied {bias_times} times: reference mismatch={worst_difference:.6f} m/s^2 (detected)')
    report['passed'] = True
    lines.append('PASS: source-expression check. Not a hardware calibration pass.')
except Exception as error:
    report['passed'] = False
    report['error'] = repr(error)
    lines.append('FAIL: ' + repr(error))
finally:
    (folder / 'result.json').write_text(json.dumps(report,indent=2)+'\n')
    (folder / 'original-output.log').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print('Original numerical evidence:', folder)
sys.exit(0 if report['passed'] else 1)
