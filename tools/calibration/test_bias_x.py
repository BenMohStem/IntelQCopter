"""Analytic checks for the low-rate X-bias bench comparison; no sensor/emulator."""
import unittest
import io
import record_bias_x as bench


class BiasTests(unittest.TestCase):
    def test_interpolation_removes_linear_drift(self):
        phases = {}
        for name, times, shift in (("raw_before",[1000,2000,3000],0),
                                   ("corrected",[10000,11000,12000],bench.EXPECTED_SHIFT),
                                   ("raw_after",[24000,25000,26000],0)):
            phases[name] = [{"ms":t,"x":4.0+0.00001*t-shift} for t in times]
        result = bench.summarize(phases)
        self.assertAlmostEqual(result['observed_x_decrease_m_s2'],bench.EXPECTED_SHIFT)

    def test_no_analog_response_is_not_reported_as_corrected(self):
        phases = {name:[{'ms':start+i*100,'x':4.0} for i in range(10)]
                  for name,start in (('raw_before',1000),('corrected',10000),('raw_after',20000))}
        result = bench.summarize(phases)
        self.assertEqual(result['observed_x_decrease_m_s2'],0)
        self.assertAlmostEqual(result['observed_minus_expected_m_s2'],-bench.EXPECTED_SHIFT)
        self.assertIn('UNREVIEWED',result['acceptance'])

    def test_timestamp_wrap_is_handled(self):
        origin = 0xFFFFFF00
        phases = {name:[{'ms':(origin+start+i*100)&0xFFFFFFFF,'x':4.0-shift} for i in range(5)]
                  for name,start,shift in (('raw_before',0,0),('corrected',1000,bench.EXPECTED_SHIFT),('raw_after',2000,0))}
        self.assertAlmostEqual(bench.summarize(phases)['observed_x_decrease_m_s2'],bench.EXPECTED_SHIFT)

    def test_stream_errors_are_rejected_before_statistics(self):
        class Device:
            def __init__(self, raw):
                self.raw = io.BytesIO(raw)
            def readline(self):
                return self.raw.readline()
        bad_streams = [b"AX,100,0,100,0.239420\n",
                       b"AX,100,-2074,100,999\n",
                       b"AX,100,-2074,0,0\nAX,100,-2074,0,0\n",
                       b"ERROR,I2C_READ_FAILED\n"]
        for raw in bad_streams:
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                bench.collect(Device(raw), io.BytesIO(), bench.BASELINE, 5)

    def test_invalid_phase_order_rejected(self):
        phases={name:[{'ms':100,'x':1},{'ms':200,'x':2}] for name in ('raw_before','corrected','raw_after')}
        with self.assertRaises(ValueError):
            bench.summarize(phases)


if __name__=='__main__':
    unittest.main(verbosity=2)
