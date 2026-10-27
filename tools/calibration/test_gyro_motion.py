"""Independent synthetic rotation and integrity rejection controls."""
from pathlib import Path
import hashlib
import io
import json
import math
import tempfile
import unittest

import analyze_gyro_motion as motion
import test_gyro_frame as frame
import test_gyro_bias as bias


class GyroMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        frame.FrameTests.setUpClass()
        cls.header = [line.replace('ACCEL_XYZ_V1', 'FIFO1000_V2') for line in bias.accel.AccelTests.header]
        cls.header += bias.GyroBiasTests.header + frame.FrameTests.rows
        cls.header += [f'GYRO_OFFSET,{i},{bias.GyroBiasTests.targets[i]}' for i in range(3)]
        cls.header += [motion.gyro.CORRECTED_GYRO_SCALE,
                       'RATE,hz=1000,packet_bytes=14,clock=mpu_fifo,diagnostic_divider=100',
                       'READY,FIFO1000_V2']
        class Device:
            def __init__(self, data):
                self.data = io.BytesIO(data)
            def write(self, value):
                assert value == b'?'
            def readline(self):
                return self.data.readline()
        wire = ('QUERY_REPLY\n' + '\n'.join(cls.header) + '\n').encode()
        _, cls.profile = motion.gyro.read_header(Device(wire), io.BytesIO(), 'synthetic', bias.accel.AccelTests.fit)

    def make_capture(self, folder, row, condition='operator_rotation'):
        rows = [row] * 20000
        checksum = motion.fnv1a_rows(rows)
        centred = [row[i] * 9.80665 / 4096 - self.profile['remaining_bias'][i] for i in range(3)]
        accel = [sum(self.profile['matrix'][i][j] * centred[j] for j in range(3)) for i in range(3)]
        rates = [row[i + 4] * motion.SCALE_RAD_PER_COUNT - self.profile['gyro_remaining_bias'][i] for i in range(3)]
        body = [sum(self.profile['gyro_to_drone'][i][j] * rates[j] for j in range(3)) for i in range(3)]
        yz = math.hypot(accel[1], accel[2])
        roll = math.degrees(math.atan2(accel[1], accel[2]))
        pitch = math.degrees(math.atan2(-accel[0], yz))
        diagnostics = [*(f'{v:.6f}' for v in accel), *(f'{v:.7f}' for v in rates), f'{roll:.3f}', f'{pitch:.3f}', *(f'{v:.7f}' for v in body)]
        lines = ['QUERY_REPLY', *self.header, 'BEGIN,20000']
        for i, values in enumerate(rows):
            lines.append('S,' + str(i) + ',' + ','.join(map(str, values)))
            if i % 100 == 0:
                lines.append('D,' + str(i) + ',' + ','.join(diagnostics))
        lines.append('PROFILE,100,10,0,0,1')
        lines += [f'END,20000,20000000,14,1000,50,500,{checksum}']
        lines += ['QUERY_REPLY', *self.header]
        serial = ('\n'.join(lines) + '\n').encode('ascii')
        (folder / 'serial.log').write_bytes(serial)
        (folder / 'samples.json').write_text(json.dumps({'raw_post_trim_words': rows}), encoding='utf-8')
        summary = {
            'operator_condition': condition,
            'status': 'CAPTURE_COMPLETE_REVIEW_REQUIRED',
            'final_header_unchanged': True,
            'serial_sha256': hashlib.sha256(serial).hexdigest(),
            'fnv1a_raw_payload': checksum,
            'observed_capture_rate_hz': 1000,
            'elapsed_us': 20000000,
            'source_sha256': 'synthetic',
            'header': self.header,
            'profile': self.profile,
        }
        (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')

    def test_constant_single_axis_angle_and_reference(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 624, -56, -190])
            result = motion.analyze(folder, 'x', 'positive', 200)
            self.assertAlmostEqual(result['integrated_body_rate_degrees']['x'], 200, delta=0.3)
            self.assertAlmostEqual(result['reference_error_deg'], 0, delta=0.3)
            self.assertLess(result['transverse_integral_magnitude_deg'], 0.3)
            self.assertFalse(result['physical_axis_validated'])
            self.assertEqual(len(result['plot_100hz_deg_s']), 2000)
            self.assertIn('<polyline', motion.make_plot(result['plot_100hz_deg_s']))

    def test_negative_rotation_and_cross_axis(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, -21, -656, 125])
            result = motion.analyze(folder, 'y', 'negative')
            self.assertAlmostEqual(result['expected_signed_component_deg'], 200, delta=0.3)
            self.assertAlmostEqual(result['transverse_integral_magnitude_deg'], 40, delta=0.3)
            self.assertEqual(result['dominant_integral_axis'], 'y')

    def test_stationary_and_changed_raw_samples_are_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0], 'stationary_assumed')
            with self.assertRaisesRegex(ValueError, 'operator_rotation'):
                motion.analyze(folder, 'z', 'positive')
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            data = json.loads((folder / 'samples.json').read_text())
            data['raw_post_trim_words'][0][4] = 1
            (folder / 'samples.json').write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'checksum'):
                motion.analyze(folder, 'z', 'positive')

    def test_bad_serial_and_invalid_rate_are_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            (folder / 'serial.log').write_bytes(b'altered')
            with self.assertRaisesRegex(ValueError, 'serial log'):
                motion.analyze(folder, 'z', 'positive')
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            summary = json.loads((folder / 'summary.json').read_text())
            summary['observed_capture_rate_hz'] = math.nan
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'rate'):
                motion.analyze(folder, 'z', 'positive')

    def test_changed_gyro_frame_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            summary = json.loads((folder / 'summary.json').read_text())
            summary['profile']['gyro_to_drone'][0][0] = 0.5
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'gyro frame differs'):
                motion.analyze(folder, 'z', 'positive')

    def test_recorded_error_and_changed_final_trim_are_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            data = (folder / 'serial.log').read_bytes().replace(b'BEGIN,20000\n', b'BEGIN,20000\nERROR,FIFO_OVERFLOW_OR_BACKLOG\n', 1)
            (folder / 'serial.log').write_bytes(data)
            summary = json.loads((folder / 'summary.json').read_text())
            summary['serial_sha256'] = hashlib.sha256(data).hexdigest()
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'acquisition error'):
                motion.analyze(folder, 'x', 'positive')
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            data = (folder / 'serial.log').read_bytes()
            old = next(line.encode() for line in self.header if line.startswith('GYRO_TRIM,0,'))
            head, middle, tail = data.rpartition(old)
            assert middle
            data = head + old.rsplit(b',', 1)[0] + b',0.1' + tail
            (folder / 'serial.log').write_bytes(data)
            summary = json.loads((folder / 'summary.json').read_text())
            summary['serial_sha256'] = hashlib.sha256(data).hexdigest()
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaises(ValueError):
                motion.analyze(folder, 'x', 'positive')

    def test_saturation_and_wrong_sensor_range_are_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 32767, 0, 0])
            with self.assertRaisesRegex(ValueError, 'signed 16-bit'):
                motion.analyze(folder, 'x', 'positive')
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            data = (folder / 'serial.log').read_bytes().replace(b'gyro_dps=500', b'gyro_dps=250')
            data = data.replace(b'GYRO_SCALE,65.5', b'GYRO_SCALE,131')
            (folder / 'serial.log').write_bytes(data)
            summary = json.loads((folder / 'summary.json').read_text())
            summary['serial_sha256'] = hashlib.sha256(data).hexdigest()
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaises(ValueError):
                motion.analyze(folder, 'x', 'positive')

    def test_missing_diagnostic_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, 0, 0])
            lines = (folder / 'serial.log').read_bytes().splitlines()
            data = b'\n'.join(line for line in lines if not line.startswith(b'D,100,')) + b'\n'
            (folder / 'serial.log').write_bytes(data)
            summary = json.loads((folder / 'summary.json').read_text())
            summary['serial_sha256'] = hashlib.sha256(data).hexdigest()
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            with self.assertRaises(ValueError):
                motion.analyze(folder, 'x', 'positive')


if __name__ == '__main__':
    unittest.main(verbosity=2)
