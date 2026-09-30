"""Synthetic velocity checks and existing four-state EKF regressions."""

import math
import unittest

import numpy as np

from edgenode_localization.ekf import EKF, normalize_angle


class TestEKF(unittest.TestCase):
    def test_velocity_scalar_update_from_negative_prior(self):
        ekf = EKF(gps_velocity_std_mps=0.50)
        ekf.initialize(10.0, 20.0, 0.3, v=-1.43)
        ekf.update_velocity(5.0)
        gain = 4.0 / (4.0 + 0.50 ** 2)
        np.testing.assert_allclose(
            ekf.get_state(), [10.0, 20.0, 0.3, -1.43 + gain * (5.0 + 1.43)],
        )
        self.assertAlmostEqual(ekf.get_covariance()[3, 3], (1.0 - gain) * 4.0)
        self.assertGreater(ekf.get_state()[3], 0.0)

    def test_velocity_noise_is_configurable(self):
        velocities = []
        for std in (0.5, 2.0):
            ekf = EKF(gps_velocity_std_mps=std)
            ekf.initialize(0.0, 0.0, 0.0)
            ekf.update_velocity(5.0)
            velocities.append(ekf.get_state()[3])
            self.assertAlmostEqual(velocities[-1], 5.0 * 4.0 / (4.0 + std ** 2))
        self.assertGreater(velocities[0], velocities[1])

    def test_velocity_requires_initialization_and_valid_input(self):
        ekf = EKF()
        with self.assertRaises(RuntimeError):
            ekf.update_velocity(1.0)
        ekf.initialize(0.0, 0.0, 0.0)
        state, covariance = ekf.get_state(), ekf.get_covariance()
        for value in (math.nan, math.inf, -math.inf, -1.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ekf.update_velocity(value)
        np.testing.assert_array_equal(ekf.get_state(), state)
        np.testing.assert_array_equal(ekf.get_covariance(), covariance)
        ekf.update_velocity(0.0)
        self.assertEqual(ekf.get_state()[3], 0.0)

    def test_velocity_noise_requires_finite_positive_std(self):
        for std in (0.0, -0.5, math.nan, math.inf):
            with self.subTest(std=std), self.assertRaises(ValueError):
                EKF(gps_velocity_std_mps=std)

    def test_position_update_regression(self):
        ekf = EKF()
        ekf.initialize(1.0, 2.0, 0.4, v=3.0)
        ekf.update_position(3.0, 4.0)
        np.testing.assert_allclose(ekf.get_state(), [2.6, 3.6, 0.4, 3.0])
        np.testing.assert_allclose(ekf.get_covariance()[:2, :2], np.eye(2) * 0.2)

    def test_yaw_wrap_update_regression(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                ekf = EKF()
                initial = sign * math.radians(179.0)
                ekf.initialize(1.0, 2.0, initial, v=3.0)
                ekf.update_yaw(-initial)
                expected = normalize_angle(initial + sign * math.radians(2.0) * 25.0 / 29.0)
                self.assertAlmostEqual(ekf.get_state()[2], expected)
                np.testing.assert_allclose(ekf.get_state()[[0, 1, 3]], [1.0, 2.0, 3.0])
                self.assertAlmostEqual(
                    ekf.get_covariance()[2, 2], math.radians(5.0) ** 2 * 4.0 / 29.0,
                )

    def test_predict_yaw_wrap_regression(self):
        ekf = EKF()
        yaw = math.radians(179.0)
        ekf.initialize(1.0, 2.0, yaw, v=3.0)
        ekf.predict(math.radians(20.0), 0.2)
        np.testing.assert_allclose(ekf.get_state(), [
            1.0 + 0.6 * math.cos(yaw), 2.0 + 0.6 * math.sin(yaw),
            math.radians(-177.0), 3.0,
        ])

    def test_mixed_updates_converge_with_symmetric_psd_covariance(self):
        ekf = EKF()
        ekf.initialize(0.0, 0.0, 0.0, v=-1.43)
        for step in range(1, 501):
            ekf.predict(0.0, 0.02)
            ekf.update_yaw(0.0)
            if step % 10 == 0:
                ekf.update_position(5.0 * step * 0.02, 0.0)
                ekf.update_velocity(5.0)
            covariance = ekf.get_covariance()
            self.assertTrue(np.all(np.isfinite(ekf.get_state())))
            np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), -1e-12)
        np.testing.assert_allclose(ekf.get_state(), [50.0, 0.0, 0.0, 5.0], atol=0.01)


if __name__ == '__main__':
    unittest.main()
