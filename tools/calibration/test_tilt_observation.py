"""Nominal gravity geometry/protocol checks; not MCU execution or hardware accuracy."""
import io
import math
import unittest

import record_gyro as recording
import test_gyro_bias as bias


class TiltTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bias.GyroBiasTests.setUpClass()
        cls.profile=dict(bias.GyroBiasTests.profile)
        cls.profile['tilt_observation']=True

    def packet(self, body):
        fit=bias.accel.AccelTests.fit
        before=[round((fit['bias_sensor_m_s2'][i]+sum(fit['forward_response'][i][j]*body[j] for j in range(3)))*4096/9.80665) for i in range(3)]
        counts=[before[i]-2*bias.accel.AccelTests.decrements[i] for i in range(3)]
        corrected=bias.accel.AccelTests().simulated_source_math(counts)
        yz=math.hypot(corrected[1],corrected[2])
        angles=[math.degrees(math.atan2(corrected[1],corrected[2])), math.degrees(math.atan2(-corrected[0],yz))]
        rates=[-v for v in self.profile['gyro_remaining_bias']]
        line=','.join(['IMU','100',*map(str,counts),'0','0','0','0',*(f'{v:.6f}' for v in corrected),*(f'{v:.7f}' for v in rates),'36.530',*(f'{v:.3f}' for v in angles)])
        return line

    def test_level_and_all_four_signs(self):
        g=9.80665
        angle=math.radians(30)
        cases=[([0,0,g],[0,0]),([0,g*math.sin(angle),g*math.cos(angle)],[30,0]),
               ([0,-g*math.sin(angle),g*math.cos(angle)],[-30,0]),
               ([-g*math.sin(angle),0,g*math.cos(angle)],[0,30]),
               ([g*math.sin(angle),0,g*math.cos(angle)],[0,-30])]
        for vector,expected in cases:
            actual=recording.parse_sample(self.packet(vector),self.profile)['tilt_deg']
            self.assertLess(max(abs(actual[i]-expected[i]) for i in range(2)),.05)

    def test_simultaneous_roll_and_pitch(self):
        r,p=map(math.radians,[20,-25])
        vector=[-9.80665*math.sin(p),9.80665*math.sin(r)*math.cos(p),9.80665*math.cos(r)*math.cos(p)]
        actual=recording.parse_sample(self.packet(vector),self.profile)['tilt_deg']
        self.assertLess(max(abs(actual[i]-[20,-25][i]) for i in range(2)),.05)

    def test_wrong_emitted_angle_rejected(self):
        fields=self.packet([0,0,9.80665]).split(',')
        fields[16]='45.000'
        with self.assertRaises(ValueError):
            recording.parse_sample(','.join(fields),self.profile)

    def test_v3_header_and_historical_packet_separation(self):
        original=bias.accel.AccelTests
        header=[line.replace('ACCEL_XYZ_V1','IMU_XYZ_V3') for line in original.header]
        header+=bias.GyroBiasTests.header
        header += [f'GYRO_OFFSET,{i},{bias.GyroBiasTests.targets[i]}' for i in range(3)]
        header += [recording.CORRECTED_GYRO_SCALE,'READY,IMU_XYZ_V3']
        class Device:
            def __init__(self):
                self.data=io.BytesIO(('QUERY_REPLY\n'+'\n'.join(header)+'\n').encode())
            def write(self,value):
                assert value==b'?'
            def readline(self):
                return self.data.readline()
        _,profile=recording.read_header(Device(),io.BytesIO(),'synthetic',original.fit)
        self.assertTrue(profile['tilt_observation'])
        with self.assertRaises(ValueError):
            recording.parse_sample(self.packet([0,0,9.80665]),bias.GyroBiasTests.profile)


if __name__=='__main__':
    unittest.main()
