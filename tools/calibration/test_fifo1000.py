"""Synthetic transport/fault checks; not MCU execution or actual 1 kHz hardware timing."""
import io
import math
import random
import unittest

import record_fifo1000 as fifo
import record_gyro as gyro
import test_gyro_bias as bias


class FifoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bias.GyroBiasTests.setUpClass()
        cls.profile=bias.GyroBiasTests.profile

    def sample(self,sequence=0,counts=None):
        return 'S,'+str(sequence)+','+','.join(map(str,counts or [1000,-100,4000,0,1,-2,3]))

    def diagnostic(self,stream):
        c=stream.rows[-1]
        centred=[c[i]*9.80665/4096-self.profile['remaining_bias'][i] for i in range(3)]
        body=[sum(row[i]*centred[i] for i in range(3)) for row in self.profile['matrix']]
        rates=[c[i+4]*math.pi/(180*65.5)-self.profile['gyro_remaining_bias'][i] for i in range(3)]
        values=body+rates+[math.degrees(math.atan2(body[1],body[2])),math.degrees(math.atan2(-body[0],math.hypot(body[1],body[2])))]
        return 'D,'+str(len(stream.rows)-1)+','+','.join(f'{v:.9f}' for v in values)

    def complete(self):
        stream=fifo.Samples(self.profile)
        rng=random.Random(1000)
        for i in range(fifo.COUNT):
            stream.add(self.sample(i,[rng.randint(-1200,1200),rng.randint(-1200,1200),rng.randint(3000,5000),10,2,-3,4]))
            if i%100==0:
                stream.diagnostic(self.diagnostic(stream))
        return stream

    def trailer(self,stream,elapsed=20000000,handler=900):
        return f'END,20000,{elapsed},28,1200,40,{handler},{stream.checksum:08X}'

    def test_full_20000_sample_stream_and_mcu_math_diagnostics(self):
        stream=self.complete()
        result=stream.finish(self.trailer(stream))
        self.assertEqual(result['observed_capture_rate_hz'],1000)
        self.assertTrue(result['handler_within_1000us_budget'])
        self.assertEqual(result['verified_mcu_diagnostics'],200)

    def test_slow_handler_is_reported_not_hidden_by_average_rate(self):
        stream=self.complete()
        result=stream.finish(self.trailer(stream,handler=1700))
        self.assertTrue(result['average_rate_within_2_percent'])
        self.assertFalse(result['handler_within_1000us_budget'])

    def test_bad_checksum_rejected(self):
        stream=self.complete()
        with self.assertRaises(ValueError):
            stream.finish(self.trailer(stream)[:-8]+'00000000')

    def test_truncated_stream_rejected(self):
        stream=fifo.Samples(self.profile)
        stream.add(self.sample())
        with self.assertRaises(ValueError):
            stream.finish(self.trailer(stream))

    def test_lost_reordered_or_duplicate_samples_rejected(self):
        stream=fifo.Samples(self.profile)
        with self.assertRaises(ValueError):
            stream.add(self.sample(1))
        stream.add(self.sample())
        with self.assertRaises(ValueError):
            stream.add(self.sample())

    def test_corrupt_mcu_calculation_and_duplicate_diagnostic_rejected(self):
        stream=fifo.Samples(self.profile)
        stream.add(self.sample())
        good=self.diagnostic(stream)
        fields=good.split(',');fields[2]='99'
        with self.assertRaises(ValueError):
            stream.diagnostic(','.join(fields))
        stream.diagnostic(good)
        with self.assertRaises(ValueError):
            stream.diagnostic(good)

    def test_sensor_saturation_rejected(self):
        stream=fifo.Samples(self.profile)
        for value in [-32768,32767,40000]:
            with self.assertRaises(ValueError):
                stream.add(self.sample(counts=[value,0,0,0,0,0,0]))

    def test_fifo_backlog_limit_rejected(self):
        stream=self.complete()
        with self.assertRaises(ValueError):
            stream.finish(self.trailer(stream).replace(',28,',',1024,'))

    def test_source_metadata_keeps_calibration_and_fifo_rate(self):
        header=[line.replace('ACCEL_XYZ_V1','FIFO1000_V1') for line in bias.accel.AccelTests.header]
        header+=bias.GyroBiasTests.header
        header += [f'GYRO_OFFSET,{i},{bias.GyroBiasTests.targets[i]}' for i in range(3)]
        header += [gyro.CORRECTED_GYRO_SCALE,'RATE,hz=1000,packet_bytes=14,clock=mpu_fifo,diagnostic_divider=100','READY,FIFO1000_V1']
        class Device:
            def __init__(self,lines):
                self.data=io.BytesIO(('QUERY_REPLY\n'+'\n'.join(lines)+'\n').encode())
            def write(self,value):
                assert value==b'?'
            def readline(self):
                return self.data.readline()
        _,profile=gyro.read_header(Device(header),io.BytesIO(),'synthetic',bias.accel.AccelTests.fit)
        self.assertTrue(profile['fifo_acquisition'])
        self.assertEqual(profile['gyro_offsets'],[-6,9,27])
        with self.assertRaises(ValueError):
            gyro.read_header(Device([s for s in header if not s.startswith('RATE,')]),io.BytesIO(),'synthetic',bias.accel.AccelTests.fit)


if __name__=='__main__':
    unittest.main()
