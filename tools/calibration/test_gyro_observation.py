"""PC packet/statistics tests only; no hardware or MCU execution."""
import io
import math
import unittest

import record_gyro as gyro
from test_accel_xyz import AccelTests


class GyroTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        AccelTests.setUpClass()
        cls.accel = AccelTests()
        cls.profile = AccelTests.profile

    def packet(self):
        counts = [0, 0, 4096]
        body = self.accel.simulated_source_math(counts)
        rates = [131 * math.pi / (180 * 65.5), -262 * math.pi / (180 * 65.5), 0]
        return ','.join(['IMU', '100', *map(str, counts), '340', '131', '-262', '0',
                         *(f'{v:.6f}' for v in body), *(f'{v:.7f}' for v in rates), '37.530'])

    def test_coherent_packet_units_and_axis_order(self):
        result = gyro.parse_sample(self.packet(), self.profile)
        self.assertEqual(result['gyro_counts_at_recorded_offsets'], [131, -262, 0])
        self.assertAlmostEqual(result['gyro_sensor_rad_s'][0], math.radians(2), places=6)
        self.assertAlmostEqual(result['gyro_sensor_rad_s'][1], math.radians(-4), places=6)
        self.assertEqual(result['temperature_C'], 37.53)

    def test_wrong_conversion_and_clipping_rejected(self):
        for index, replacement in [(6, '32767'), (12, '2.0'), (15, 'nan'), (5, '40000')]:
            fields = self.packet().split(',')
            fields[index] = replacement
            with self.assertRaises(ValueError):
                gyro.parse_sample(','.join(fields), self.profile)

    def test_held_out_data_does_not_fit_bias(self):
        samples = []
        for i in range(300):
            x = 0.01 + (0.002 if i % 2 else -0.002) + (0.03 if i >= 200 else 0)
            samples.append({'ms': i * 11, 'gyro_sensor_rad_s': [x, 2*x, -x], 'temperature_C': 30})
        result = gyro.summarize(samples)
        self.assertAlmostEqual(result['candidate_bias_sensor_rad_s'][0], 0.01)
        self.assertAlmostEqual(result['offline_predicted_held_out_mean_after_bias_rad_s'][0], 0.03)
        cov = result['sample_covariance_training_rad2_s2']
        self.assertAlmostEqual(cov[0][1], 2 * cov[0][0])
        self.assertAlmostEqual(cov[2][0], -cov[0][0])

    def test_timing_accepts_complete_record_and_millis_wrap(self):
        start = 0xFFFFFF00
        samples = [{"ms": (start + i * 11) & 0xFFFFFFFF} for i in range(1819)]
        result = gyro.validate_timing(samples, 20)
        self.assertAlmostEqual(result["device_duration_s"], 19.998)
        self.assertEqual(result["max_report_gap_ms"], 11)

    def test_sample_count_alone_does_not_prove_complete_recording(self):
        short = [{"ms": i * 11} for i in range(500)]
        with self.assertRaises(ValueError):
            gyro.validate_timing(short, 20)

    def test_duplicate_reset_and_missing_report_gaps_rejected(self):
        good = [{"ms": i * 11} for i in range(1819)]
        for replacement in [good[499]["ms"], 0, good[499]["ms"] + 101]:
            bad = [dict(row) for row in good]
            bad[500]["ms"] = replacement
            with self.assertRaises(ValueError):
                gyro.validate_timing(bad, 20)

    def test_drift_requires_elapsed_device_time(self):
        samples = [{"ms": 10, "gyro_sensor_rad_s": [0, 0, 0], "temperature_C": 30} for _ in range(100)]
        with self.assertRaises(ValueError):
            gyro.summarize(samples)

    def test_sample_covariance_denominator(self):
        mean, cov = gyro.mean_covariance([[1, 2, 3], [3, 6, 9]])
        self.assertEqual(mean, [2, 4, 6])
        self.assertEqual(cov[0], [2, 4, 6])

    def test_header_requires_all_gyro_offset_words(self):
        header = [x.replace('ACCEL_XYZ_V1', 'IMU_XYZ_V1') for x in AccelTests.header]
        class Device:
            def __init__(self, lines):
                self.lines = iter(('QUERY_REPLY\n' + '\n'.join(lines) + '\n').encode().splitlines(keepends=True))
                self.sent = []
            def write(self, value):
                self.sent.append(value)
            def readline(self):
                return next(self.lines, b'')
        complete = header + ['GYRO_OFFSET,0,0', 'GYRO_OFFSET,1,-12', 'GYRO_OFFSET,2,33', gyro.GYRO_SCALE, 'READY,IMU_XYZ_V1']
        device = Device(complete)
        _, profile = gyro.read_header(device, io.BytesIO(), 'synthetic', AccelTests.fit)
        self.assertEqual(profile['gyro_offsets'], [0, -12, 33])
        self.assertEqual(device.sent, [b'?'])
        with self.assertRaises(ValueError):
            gyro.read_header(Device([x for x in complete if not x.startswith('GYRO_OFFSET,1,')]), io.BytesIO(), 'synthetic', AccelTests.fit)


if __name__ == '__main__':
    unittest.main()
