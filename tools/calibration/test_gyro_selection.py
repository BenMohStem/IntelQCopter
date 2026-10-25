"""Offline selection-boundary regression checks; no hardware."""
import unittest
from analyze_gyro_runs import verify_selection


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.original=[{'ms': i*12,'value': i} for i in range(10)]

    def test_exact_suffix(self):
        self.assertEqual(verify_selection(self.original,self.original[2:],10),0)

    def test_valid_post_measurement_packet_preserved_outside_statistics(self):
        self.assertEqual(verify_selection(self.original,self.original[2:9],9),1)

    def test_edited_or_missing_selected_sample_rejected(self):
        bad=[dict(row) for row in self.original[2:9]]
        bad[3]['value']=999
        with self.assertRaises(ValueError):
            verify_selection(self.original,bad,9)
        with self.assertRaises(ValueError):
            verify_selection(self.original,self.original[2:5]+self.original[6:9],9)

    def test_wrong_boundary_count_rejected(self):
        with self.assertRaises(ValueError):
            verify_selection(self.original,self.original[2:9],10)

    def test_invalid_trailing_timestamp_rejected(self):
        bad=[dict(row) for row in self.original]
        bad[-1]['ms']=bad[-2]['ms']
        with self.assertRaises(ValueError):
            verify_selection(bad,bad[2:9],9)


if __name__=='__main__':
    unittest.main()
