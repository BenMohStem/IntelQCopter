"""Source-expression numerical check, NOT an ESP32 or sensor execution test."""
import ast
import ctypes
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'firmware/main.cpp'
text = SOURCE.read_text()
fit = json.loads((ROOT / 'measurements/mpu6050/2026-10-24-six-pose/calculation.json').read_text())
refinement_path = ROOT / 'measurements/mpu6050/2026-10-25-level-refinement/refinement.json'
if refinement_path.exists():
    refinement = json.loads(refinement_path.read_text())
    level = refinement['level_rotation']
    fit['transform_sensor_to_drone'] = refinement['transform_sensor_to_drone']
    for result in fit['training_residuals'].values():
        old = result['corrected_drone_m_s2']
        result['corrected_drone_m_s2'] = [sum(level[i][j]*old[j] for j in range(3)) for i in range(3)]

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
        if isinstance(node.op, ast.Div):
            return f32(left / right)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'lroundf':
        value = expression(node.args[0], values)
        return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)
    raise ValueError('Unsupported firmware expression')

def source_expression(name):
    match = re.search(r'float\s+' + name + r'\s*=\s*([^;]+);', text)
    assert match, 'Missing firmware expression: ' + name
    value = re.sub(r'acceleration\.acceleration\.([xyz])', r'raw_\1', match.group(1))
    value = re.sub(r'(?<=\d)f\b', '', value)
    return ast.parse(value, mode='eval').body

try:
    assert '#include <Adafruit_MPU6050.h>' in text
    assert ('mpu.getAccelerometerSensor()->getEvent(&acceleration);' in text
            or 'mpu.getEvent(&acceleration, &rotation, &temperature);' in text)
    assert 'setXAccelOffset' not in text and 'apply_accel_offsets' not in text
    hardware_mode = ('write_accel_bias(0x06, accel_bias_x' in text
                     or 'write_sensor_bias(0x06, accel_bias_x' in text)
    report['mode'] = 'modeled MPU hardware trim + source expressions' if hardware_mode else 'software-only bias'
    trim = [0, 0, 0]
    quantization = [0.0, 0.0, 0.0]
    if hardware_mode:
        match = re.search(r'int32_t change\s*=\s*([^;]+);', text)
        assert match
        trim_tree = ast.parse(re.sub(r'(?<=\d)f\b', '', match.group(1)), mode='eval').body
        trim = [-int(expression(trim_tree, {'bias': value})) for value in fit['bias_sensor_m_s2']]
        assert trim == [-270, 16, 134]
        assert all(value % 2 == 0 for value in trim)
        for baseline in (-30000, -1, 0, 1, 30000):
            for delta in trim:
                assert ((baseline + delta) & 1) == (baseline & 1)
        quantization = [fit['bias_sensor_m_s2'][i] + trim[i] * 9.80665 / 2048 for i in range(3)]
        lines.append('MPU trim model: delta=' + str(trim) + '; low-bit preservation checked on signed baseline examples. No I2C performed.')
    expressions = {name: source_expression(name) for name in
                   ('accel_bias_x','accel_bias_y','accel_bias_z','sensor_x','sensor_y','sensor_z','drone_x','drone_y','drone_z')}
    for pose, raw in fit['raw_means_m_s2'].items():
        values = {'raw_' + axis: f32(raw[i] + (trim[i] * 9.80665 / 2048 if hardware_mode else 0)) for i,axis in enumerate('xyz')}
        for name, tree in expressions.items():
            values[name] = expression(tree, values)
        actual = [values['drone_' + axis] for axis in 'xyz']
        expected = fit['training_residuals'][pose]['corrected_drone_m_s2']
        if hardware_mode:
            expected = [expected[i] + sum(fit['transform_sensor_to_drone'][i][j] * quantization[j] for j in range(3)) for i in range(3)]
        difference = max(abs(a-b) for a,b in zip(actual,expected))
        assert difference < 0.00001, (pose, actual, expected)
        row = dict(pose=pose, corrected=actual, reference=expected, max_expression_difference=difference)
        report['results'].append(row)
        lines.append(f'{pose}: corrected={actual}; max difference from double-precision fit={difference:.9g}')
    # Deliberately incorrect total bias corrections must not reproduce the original fit.
    for bias_times in (0, 2):
        worst_difference = 0.0
        for pose, raw in fit['raw_means_m_s2'].items():
            values = {'sensor_' + axis: f32(raw[i] - bias_times * fit['bias_sensor_m_s2'][i]) for i,axis in enumerate('xyz')}
            actual = [expression(expressions['drone_' + axis], values) for axis in 'xyz']
            expected = fit['training_residuals'][pose]['corrected_drone_m_s2']
            worst_difference = max(worst_difference, max(abs(a-b) for a,b in zip(actual,expected)))
        assert worst_difference > 0.1
        lines.append(f'Negative control, bias applied {bias_times} times: reference mismatch={worst_difference:.6f} m/s^2 (detected)')
    if refinement_path.exists():
        match = re.search(r'change = (lroundf\(bias \* \(180\.0f / PI\) \* 32\.75f\));', text)
        assert match, 'Missing source gyro register conversion'
        tree = ast.parse(re.sub(r'(?<=\d)f\b', '', match.group(1)), mode='eval').body
        measured = []
        for axis in 'xyz':
            bias = expression(source_expression('gyro_bias_' + axis), {})
            measured.append(-int(expression(tree, {'bias': bias, 'PI': 3.141592653589793})))
        assert measured == refinement['gyro_register_delta']
        lines.append('Gyro source register-conversion model matches measured candidate deltas: ' + str(measured))
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
