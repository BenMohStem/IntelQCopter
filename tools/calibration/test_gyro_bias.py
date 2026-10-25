"""Source-coefficient/nominal trim model checks; not MCU or physical sensor execution."""
import json
import io
import math
import random
import re
import unittest

import record_gyro as recording
import test_accel_xyz as accel


class GyroBiasTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        accel.AccelTests.setUpClass()
        text = (recording.ROOT/'firmware/main.cpp').read_text(encoding='utf-8')
        block = re.search(r'const float gyro_bias\[3\]\s*=\s*\{(.*?)\};', text, re.S)[1]
        cls.bias = [accel.f32(float(v)) for v in re.findall(r'[-+]?\d+\.\d+', block)]
        assert len(cls.bias) == 3
        cls.targets = [-round(v*180/math.pi*32.8) for v in cls.bias]
        cls.remaining = [cls.bias[i]+cls.targets[i]*math.pi/(180*32.8) for i in range(3)]
        cls.header = []
        for i in range(3):
            cls.header += [f'GYRO_BIAS,{i},{cls.bias[i]:.9f}', f'GYRO_BOOT_OFFSET,{i},0',
                           f'GYRO_TRIM,{i},0,{cls.targets[i]},{cls.remaining[i]:.9f}']
        cls.profile = dict(accel.AccelTests.profile)
        cls.profile.update(recording.parse_gyro_correction(cls.header, cls.targets))

    def test_source_bias_matches_frozen_candidate_and_targets(self):
        measured=json.loads((recording.ROOT/'measurements/mpu6050/2026-10-25-three-gyro-runs/analysis.json').read_text(encoding='utf-8'))['candidate_bias_sensor_rad_s']
        self.assertEqual(self.targets, [-6, 9, 27])
        for actual, expected in zip(self.bias, measured):
            self.assertLess(abs(actual-expected), 1e-9)

    def test_hardware_plus_remainder_removes_bias_once_in_nominal_model(self):
        rng=random.Random(328)
        for _ in range(1000):
            for axis in range(3):
                before=rng.uniform(-4,4)
                after=before+self.targets[axis]*math.pi/(180*32.8)
                self.assertAlmostEqual(after-self.remaining[axis],before-self.bias[axis],places=12)

    def test_known_retained_targets_accepted_without_accumulation(self):
        retained=[line if not line.startswith('GYRO_BOOT_OFFSET,') else f'GYRO_BOOT_OFFSET,{line.split(",")[1]},{self.targets[int(line.split(",")[1])]}' for line in self.header]
        profile=recording.parse_gyro_correction(retained,self.targets)
        self.assertEqual(profile['gyro_boot_offsets'], self.targets)

    def test_unknown_boot_words_rejected(self):
        bad=[line if not line.startswith('GYRO_BOOT_OFFSET,0,') else 'GYRO_BOOT_OFFSET,0,-12' for line in self.header]
        with self.assertRaises(ValueError):
            recording.parse_gyro_correction(bad,self.targets)

    def test_full_duplicate_bias_subtraction_rejected(self):
        bad=[line if not line.startswith('GYRO_TRIM,0,') else f'GYRO_TRIM,0,0,-6,{self.bias[0]:.9f}' for line in self.header]
        with self.assertRaises(ValueError):
            recording.parse_gyro_correction(bad,self.targets)

    def test_full_v2_header_is_verified(self):
        class SavedDevice:
            def __init__(self, data):
                self.data=io.BytesIO(data)
            def write(self, query):
                assert query == b'?'
            def readline(self):
                return self.data.readline()
        header=[line.replace('ACCEL_XYZ_V1','IMU_XYZ_V2') for line in accel.AccelTests.header]
        header += self.header
        header += [f'GYRO_OFFSET,{i},{self.targets[i]}' for i in range(3)]
        header += [recording.CORRECTED_GYRO_SCALE,'READY,IMU_XYZ_V2']
        stream=('QUERY_REPLY\n'+'\n'.join(header)+'\n').encode()
        _, profile=recording.read_header(SavedDevice(stream),io.BytesIO(),'synthetic',accel.AccelTests.fit)
        self.assertEqual(profile['gyro_offsets'],[-6,9,27])
        self.assertIn('gyro_remaining_bias',profile)
        with self.assertRaises(ValueError):
            recording.read_header(SavedDevice(stream),io.BytesIO(),'different-source',accel.AccelTests.fit)

    def test_corrected_packet_checks_software_remainder(self):
        helper=accel.AccelTests()
        body=helper.simulated_source_math([0,0,4096])
        corrected=[-v for v in self.profile['gyro_remaining_bias']]
        packet=','.join(['IMU','100','0','0','4096','0','0','0','0',
                         *(f'{v:.6f}' for v in body),*(f'{v:.7f}' for v in corrected),'36.530'])
        recording.parse_sample(packet,self.profile)
        with self.assertRaises(ValueError):
            recording.parse_sample(packet,accel.AccelTests.profile)

    def test_missing_and_nonfinite_metadata_rejected(self):
        for bad in [self.header[1:], [line if not line.startswith('GYRO_TRIM,2,') else 'GYRO_TRIM,2,0,27,nan' for line in self.header]]:
            with self.assertRaises(ValueError):
                recording.parse_gyro_correction(bad,self.targets)


if __name__ == '__main__':
    unittest.main()
