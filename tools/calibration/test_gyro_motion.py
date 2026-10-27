"""Independent synthetic rotation and integrity rejection controls."""
from pathlib import Path
import hashlib
import json
import math
import tempfile
import unittest

import analyze_gyro_motion as motion


class GyroMotionTests(unittest.TestCase):
    def make_capture(self, folder, row, condition='operator_rotation'):
        rows = [row] * 20000
        checksum = motion.fnv1a_rows(rows)
        lines = ['HELLO,FIFO1000_V2,synthetic']
        lines += [f'GYRO_ROT,{i},' + ','.join(str(int(i == j)) for j in range(3)) for i in range(3)]
        lines += [f'GYRO_TRIM,{i},0,0,0' for i in range(3)]
        lines += ['BEGIN,20000']
        lines += ['S,' + str(i) + ',' + ','.join(map(str, values)) for i, values in enumerate(rows)]
        lines += [f'END,20000,20000000,14,1000,50,500,{checksum}']
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
            'profile': {'gyro_to_drone': [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'gyro_remaining_bias': [0, 0, 0]},
        }
        (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')

    def test_constant_single_axis_angle_and_reference(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 655, 0, 0])
            result = motion.analyze(folder, 'x', 'positive', 200)
            self.assertAlmostEqual(result['integrated_body_rate_degrees']['x'], 200, places=8)
            self.assertAlmostEqual(result['reference_error_deg'], 0, places=8)
            self.assertAlmostEqual(result['transverse_integral_magnitude_deg'], 0, places=8)
            self.assertFalse(result['physical_axis_validated'])
            self.assertEqual(len(result['plot_100hz_deg_s']), 2000)
            self.assertIn('<polyline', motion.make_plot(result['plot_100hz_deg_s']))

    def test_negative_rotation_and_cross_axis(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.make_capture(folder, [0, 0, 4096, 0, 0, -655, 131])
            result = motion.analyze(folder, 'y', 'negative')
            self.assertAlmostEqual(result['expected_signed_component_deg'], 200, places=8)
            self.assertAlmostEqual(result['transverse_integral_magnitude_deg'], 40, places=8)
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
            with self.assertRaisesRegex(ValueError, 'Gyro frame differs'):
                motion.analyze(folder, 'z', 'positive')


if __name__ == '__main__':
    unittest.main(verbosity=2)
