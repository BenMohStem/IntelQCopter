"""Float32/source-coefficient algebra and protocol checks; NOT MCU/sensor execution."""
import hashlib
import json
import math
from pathlib import Path
import random
import re
import struct
import unittest

import record_accel as recording

ROOT = Path(__file__).resolve().parents[2]


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def source_coefficients():
    text = (ROOT/'firmware/main.cpp').read_text(encoding='utf-8')
    bias_text = re.search(r'accel_bias\[3\]\s*=\s*\{(.*?)\};',text,re.S)[1]
    matrix_text = re.search(r'accel_matrix\[3\]\[3\]\s*=\s*\{(.*?)\};',text,re.S)[1]
    numbers = lambda value: [f32(float(x)) for x in re.findall(r'[-+]?\d+\.\d+',value)]
    bias = numbers(bias_text)
    values = numbers(matrix_text)
    if len(bias)!=3 or len(values)!=9:
        raise ValueError('Expected the actual full XYZ source coefficients')
    return bias,[values[i:i+3] for i in (0,3,6)]


class AccelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bias,cls.matrix = source_coefficients()
        cls.fit=json.loads((ROOT/'measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json').read_text(encoding='utf-8'))
        cls.targets,nominal_remaining=recording.register_profile(cls.bias)
        cls.decrements=[recording.BASELINES[i]-cls.targets[i] for i in range(3)]
        cls.remaining=[f32(cls.bias[i]-f32(cls.decrements[i]*f32(f32(9.80665)/2048))) for i in range(3)]
        cls.header=['HELLO,ACCEL_XYZ_V1,synthetic',recording.CONFIG,'BOARD,synthetic,0000']
        for i in range(3):
            cls.header.extend([f'BIAS,{i},{cls.bias[i]:.9f}',f'BOOT_OFFSET,{i},{recording.BASELINES[i]}',
                f'ROW,{i},'+','.join(f'{v:.9f}' for v in cls.matrix[i]),
                f'OFFSET,{i},{recording.BASELINES[i]},{cls.targets[i]},{cls.remaining[i]:.9f}'])
        cls.profile=recording.parse_profile(cls.header,'synthetic',cls.fit)

    def simulated_source_math(self,counts):
        centred=[f32(f32(value*f32(f32(9.80665)/4096))-self.remaining[i]) for i,value in enumerate(counts)]
        body=[]
        for row in self.matrix:
            value=f32(0)
            for i in range(3):
                value=f32(value+f32(row[i]*centred[i]))
            body.append(value)
        return body

    def test_all_targets_preserve_compensation_bits(self):
        self.assertEqual(self.targets,[-2348,2993,1212])
        for base,target in zip(recording.BASELINES,self.targets):
            self.assertEqual(base&1,target&1)

    def test_whole_pipeline_equals_full_software_fit_for_arbitrary_vectors(self):
        rng=random.Random(4096)
        for _ in range(1000):
            before=[rng.randint(-20000,20000) for i in range(3)]
            after=[before[i]-2*self.decrements[i] for i in range(3)]
            output=self.simulated_source_math(after)
            centred=[before[i]*9.80665/4096-self.fit['bias_sensor_m_s2'][i] for i in range(3)]
            expected=[sum(row[i]*centred[i] for i in range(3)) for row in self.fit['correction_sensor_to_body']]
            self.assertLess(max(abs(output[i]-expected[i]) for i in range(3)),0.00005)
            line='ACCEL,100,'+','.join(map(str,after))+','+','.join(f'{v:.6f}' for v in output)
            recording.validate_sample(line,self.profile)

    def test_known_orientations_preserve_direction_not_level_everything(self):
        h=9.80665/math.sqrt(2)
        vectors=[[0,0,9.80665],[9.80665,0,0],[0,-9.80665,0],[h,0,h],[0,h,h],[0,0,-9.80665]]
        for desired in vectors:
            sensor=[self.fit['bias_sensor_m_s2'][i]+sum(self.fit['forward_response'][i][j]*desired[j] for j in range(3)) for i in range(3)]
            before=[round(v*4096/9.80665) for v in sensor]
            after=[before[i]-2*self.decrements[i] for i in range(3)]
            actual=self.simulated_source_math(after)
            self.assertLess(max(abs(actual[i]-desired[i]) for i in range(3)),0.003)

    def test_missing_rounding_compensation_is_rejected(self):
        bad=[line if not line.startswith('OFFSET,0,') else 'OFFSET,0,-2074,-2348,0' for line in self.header]
        with self.assertRaises(ValueError):
            recording.parse_profile(bad,'synthetic',self.fit)

    def test_retained_targets_are_accepted_but_unknown_offsets_are_rejected(self):
        retained=[f'BOOT_OFFSET,{line.split(",")[1]},{self.targets[int(line.split(",")[1])]}' if line.startswith('BOOT_OFFSET,') else line for line in self.header]
        profile=recording.parse_profile(retained,'synthetic',self.fit)
        self.assertEqual(profile['boot_offsets'],self.targets)
        corrupted=[line if not line.startswith('BOOT_OFFSET,0,') else 'BOOT_OFFSET,0,-2350' for line in retained]
        with self.assertRaises(ValueError):
            recording.parse_profile(corrupted,'synthetic',self.fit)

    def test_bad_source_and_output_are_rejected(self):
        with self.assertRaises(ValueError):
            recording.parse_profile(self.header,'wrong-source',self.fit)
        for line in ('ACCEL,100,0,0,4096,0,0,0','ACCEL,100,0,0,4096,nan,0,0','ACCEL,100,-32768,0,0,0,0,0'):
            with self.assertRaises(ValueError):
                recording.validate_sample(line,self.profile)


if __name__=='__main__':
    unittest.main(verbosity=2)
