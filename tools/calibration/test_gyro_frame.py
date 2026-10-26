"""Algebra/metadata checks of provisional gyro rotation; not physical axis validation."""
import io
import json
import unittest
import numpy as np

import record_gyro as gyro
import test_gyro_bias as bias


class FrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bias.GyroBiasTests.setUpClass()
        cls.rotation=np.array(json.loads((gyro.ROOT/'config/gyro-frame.json').read_text(encoding='utf-8'))['rotation_sensor_to_drone'])
        cls.rows=['GYRO_ROT,'+str(i)+','+','.join(f'{v:.9f}' for v in row) for i,row in enumerate(cls.rotation)]

    def test_proper_rotation_preserves_norm_and_inverse(self):
        r=self.rotation
        self.assertLess(np.max(np.abs(r@r.T-np.eye(3))),1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1,places=12)
        vectors=np.random.default_rng(1000).normal(size=(1000,3))
        transformed=vectors@r.T
        self.assertLess(np.max(np.abs(np.linalg.norm(vectors,axis=1)-np.linalg.norm(transformed,axis=1))),1e-12)

    def test_rotation_is_polar_factor_not_accelerometer_scale(self):
        c=np.array(bias.accel.AccelTests.fit['correction_sensor_to_body'])
        u,_,vt=np.linalg.svd(c)
        self.assertLess(np.max(np.abs(self.rotation-u@vt)),1e-12)
        self.assertGreater(np.max(np.abs(c-self.rotation)),.001)

    def test_reject_missing_scaled_or_nonfinite_rotation_metadata(self):
        gyro.parse_gyro_rotation(self.rows)
        bads=[self.rows[:2],[line.replace('0.953336617','nan') for line in self.rows],
              ['GYRO_ROT,'+str(i)+','+','.join(str(v*1.1) for v in row) for i,row in enumerate(self.rotation)]]
        for bad in bads:
            with self.assertRaises(ValueError):
                gyro.parse_gyro_rotation(bad)

    def test_full_fifo_v2_profile(self):
        header=[s.replace('ACCEL_XYZ_V1','FIFO1000_V2') for s in bias.accel.AccelTests.header]
        header+=bias.GyroBiasTests.header+self.rows
        header += [f'GYRO_OFFSET,{i},{bias.GyroBiasTests.targets[i]}' for i in range(3)]
        header += [gyro.CORRECTED_GYRO_SCALE,'RATE,hz=1000,packet_bytes=14,clock=mpu_fifo,diagnostic_divider=100','READY,FIFO1000_V2']
        class Device:
            def __init__(self):
                self.data=io.BytesIO(('QUERY_REPLY\n'+'\n'.join(header)+'\n').encode())
            def write(self,data):
                assert data==b'?'
            def readline(self):
                return self.data.readline()
        _,profile=gyro.read_header(Device(),io.BytesIO(),'synthetic',bias.accel.AccelTests.fit)
        self.assertIn('gyro_to_drone',profile)
        self.assertTrue(profile['fifo_acquisition'])


if __name__=='__main__':
    unittest.main()
