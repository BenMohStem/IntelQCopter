"""Analytic estimator checks and DOE identifiability; no physical acceptance."""
import unittest
import numpy as np
from prepare_attitude import kalman_angle, pooled_covariance, lag_correlation, factorial_cases, wrap


class AttitudeTests(unittest.TestCase):
    def test_exact_discrete_trajectory_and_angle_wrap(self):
        truth = np.linspace(np.radians(170), np.radians(190), 1000)
        rate = np.r_[0.0, np.diff(truth) / 0.001]
        estimate, covariance, _ = kalman_angle(wrap(truth), rate, 0.001, 1e-8, 1e-5)
        self.assertLess(np.max(np.abs(wrap(estimate - truth))), 1e-12)
        self.assertTrue(np.all(covariance >= 0))

    def test_riccati_fixed_point_and_white_noise_assumption(self):
        q = 0.01
        r = 0.04
        expected = (-q + np.sqrt(q * q + 4 * q * r)) / 2
        _, covariance, gains = kalman_angle(np.zeros(1000), np.zeros(1000), 0.001, q, r)
        self.assertAlmostEqual(covariance[-1], expected, places=14)
        self.assertAlmostEqual(gains[-1], (expected + q) / (expected + q + r), places=14)
        _, covariance, _ = kalman_angle(np.zeros(100), np.zeros(100), 0.001, 0, r)
        self.assertAlmostEqual(covariance[-1], r / 100, places=14)

    def test_invalid_inputs_cannot_make_silent_estimates(self):
        for angles, rates, dt, q, r in [
            ([0], [0], .001, 1, 1), ([0, 0], [0], .001, 1, 1),
            ([0, np.nan], [0, 0], .001, 1, 1), ([0, 0], [0, np.inf], .001, 1, 1),
            ([0, 0], [0, 0], 0, 1, 1), ([0, 0], [0, 0], .001, -1, 1),
            ([0, 0], [0, 0], .001, 1, 0), ([0, 0], [0, 0], .001, np.inf, 1),
        ]:
            with self.subTest(dt=dt, q=q, r=r):
                with self.assertRaises(ValueError):
                    kalman_angle(angles, rates, dt, q, r)

    def test_pose_means_do_not_enter_pooled_noise_covariance(self):
        values = np.array([[1., -2.], [2., 0.], [3., 2.]])
        expected = np.array([[1., 2.], [2., 4.]])
        np.testing.assert_allclose(pooled_covariance([values, values + 1000]), expected)
        alternating = np.tile([[1.], [-1.]], (500, 1))
        self.assertAlmostEqual(lag_correlation(alternating)[1, 0], -0.999)

    def test_four_cases_identify_both_main_effects_and_interaction(self):
        factors = np.array(factorial_cases()) / 0.05 - 1
        design = np.column_stack((np.ones(4), factors, factors[:, 0] * factors[:, 1]))
        self.assertEqual(np.linalg.matrix_rank(design), 4)
        np.testing.assert_array_equal(design.T @ design, 4 * np.eye(4))

    def test_constant_acceleration_is_not_identifiable_as_tilt(self):
        # A constant 0.1 g lateral acceleration produces atan(0.1) apparent roll.
        apparent = np.arctan(0.1)
        estimates, _, _ = kalman_angle(np.full(1000, apparent), np.zeros(1000), .001, 1e-8, 1e-5)
        self.assertAlmostEqual(estimates[-1], apparent, places=12)
        self.assertGreater(np.degrees(estimates[-1]), 5)


if __name__ == '__main__':
    unittest.main()
