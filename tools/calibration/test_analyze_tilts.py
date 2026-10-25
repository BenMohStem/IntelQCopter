"""Known-vector checks of operator-reference clarification; no measured data or hardware."""
import unittest
import analyze_tilts as analysis
import record_tilts as recorder


class TiltAnalysisTests(unittest.TestCase):
    def test_confirmed_raised_sides_have_correct_gravity_sign(self):
        expected = {'X+': [1, 0, 1], 'X-': [-1, 0, 1], 'Y+': [0, 1, 1], 'Y-': [0, -1, 1]}
        for side, signs in expected.items():
            vector = recorder.reference_vector(*analysis.clarified_angles(side, 45))
            h = 9.80665 / (2 ** 0.5)
            for actual, sign in zip(vector, signs):
                self.assertAlmostEqual(actual, sign*h)

    def test_wrong_axis_explains_sixty_degree_difference_without_refit(self):
        candidate = {'bias_sensor_m_s2': [0, 0, 0], 'correction_sensor_to_body': [[1,0,0],[0,1,0],[0,0,1]]}
        h = 9.80665 / (2 ** 0.5)
        mean = {'ax': h, 'ay': 0, 'az': h}
        original = recorder.compare(mean, candidate, [45, 0])
        clarified = recorder.compare(mean, candidate, analysis.clarified_angles('X+',45))
        self.assertAlmostEqual(original['direction_difference_deg'],60)
        self.assertAlmostEqual(clarified['direction_difference_deg'],0,places=5)
        self.assertEqual(candidate['bias_sensor_m_s2'],[0,0,0])

    def test_reference_estimate_does_not_reduce_observed_error(self):
        # compare() accepts only measured mean, fixed coefficients and angles:
        # no placement-uncertainty subtraction or hidden fitting is performed.
        candidate = {'bias_sensor_m_s2':[0,0,0], 'correction_sensor_to_body':[[1,0,0],[0,1,0],[0,0,1]]}
        vector = recorder.reference_vector(44,0)
        result = recorder.compare(dict(zip(('ax','ay','az'),vector)),candidate,[45,0])
        self.assertAlmostEqual(result['direction_difference_deg'],1,places=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
