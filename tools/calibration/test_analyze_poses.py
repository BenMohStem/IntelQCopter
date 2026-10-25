"""Analytic/synthetic checks of calibration and noise analysis; no real hardware."""
import unittest

import numpy as np
import analyze_poses as analysis

analysis.np = np


class AnalysisTests(unittest.TestCase):
    def test_recovers_known_affine_calibration(self):
        response = np.array([[1.02, 0.1, -0.2], [-0.05, 0.98, 0.02], [0.2, 0.01, 1.01]])
        bias = np.array([0.3, -0.2, 0.1])
        measurements = {}
        for repeat in (1, 2, 3):
            for pose in analysis.POSES:
                data = np.zeros((5, 7))
                data[:, :3] = bias + response @ analysis.target(pose)
                measurements[analysis.label(repeat, pose)] = {"data": data}
        fitted_bias, correction, _ = analysis.fit_calibration(measurements)
        np.testing.assert_allclose(fitted_bias, bias, atol=1e-12)
        np.testing.assert_allclose(correction @ response, np.eye(3), atol=1e-12)
        # Changing reserved observations must not change learned parameters.
        for pose in analysis.POSES:
            measurements[analysis.label(3, pose)]["data"][:, :3] += 100
        updated_bias, updated_correction, _ = analysis.fit_calibration(measurements)
        np.testing.assert_array_equal(updated_bias, fitted_bias)
        np.testing.assert_array_equal(updated_correction, correction)

    def test_covariance_transformation_matches_transformed_samples(self):
        data = np.random.default_rng(17).normal(size=(1000, 6))
        stats, _ = analysis.noise_statistics(data)
        covariance = np.array(stats["covariance"])
        transform = np.array([[1, 2, -1], [0.2, 1, 0], [0, 1, 2]])
        expected = np.cov(data[:, :3] @ transform.T, rowvar=False, ddof=1)
        np.testing.assert_allclose(transform @ covariance[:3, :3] @ transform.T, expected, atol=1e-12)

    def test_temporal_dependence_is_detected_without_cross_axis_assumption(self):
        t = np.arange(2000)
        data = np.column_stack([np.sin(2 * np.pi * t / (40 + j)) for j in range(6)])
        stats, acf = analysis.noise_statistics(data)
        self.assertTrue(np.all(np.array(stats["acf_lag1"]) > 0.95))
        np.testing.assert_allclose(acf[0], 1)

    def test_missing_session_and_constant_channel_are_rejected(self):
        from pathlib import Path
        import tempfile
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)
            (path / "session.json").write_text('{"completed": []}')
            with self.assertRaisesRegex(ValueError, "18 completed"):
                analysis.read_session(path)
        with self.assertRaisesRegex(ValueError, "Zero-variance"):
            analysis.noise_statistics(np.zeros((100, 6)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
