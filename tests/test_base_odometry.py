import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np
from tf.transformations import euler_matrix, rotation_matrix

SCRIPT = Path(__file__).resolve().parents[1] / 'src/lidar_rgbd_capture/scripts/base_odometry.py'
spec = importlib.util.spec_from_file_location('base_odometry', str(SCRIPT))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GeometryTests(unittest.TestCase):
    def test_full_pose_composition_with_rotation_and_lever_arm(self):
        body = rotation_matrix(math.pi / 2, [0, 0, 1])
        body[:3, 3] = [3., 4., 2.]
        extra = rotation_matrix(math.pi / 2, [1, 0, 0])
        extra[:3, 3] = [.2, -.3, .4]
        base, _ = module.convert(body, extra, np.eye(6))
        np.testing.assert_allclose(base[:3, 3], [3.3, 4.2, 2.4], atol=1e-12)
        np.testing.assert_allclose(base[:3, :3], [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)

    def test_covariance_order_and_lever_arm_against_numerical_jacobian(self):
        body = euler_matrix(.2, -.3, .8)
        body[:3, 3] = [2, -1, .5]
        extra = euler_matrix(-.1, .15, -.5)
        extra[:3, 3] = [.3, -.2, .1]
        generator = np.arange(36, dtype=float).reshape(6, 6) / 100
        native = generator @ generator.T + np.diag([.01, .02, .03, .04, .05, .06])
        order = [3, 4, 5, 0, 1, 2]
        _, actual = module.convert(body, extra, native[np.ix_(order, order)])
        baseline = body @ extra
        jacobian = np.zeros((6, 6))
        epsilon = 1e-7
        for column in range(6):
            perturbed = body.copy()
            if column < 3:
                perturbed[column, 3] += epsilon
            else:
                axis = np.eye(3)[column - 3]
                perturbed[:3, :3] = body[:3, :3] @ rotation_matrix(epsilon, axis)[:3, :3]
            changed = perturbed @ extra
            delta_rotation = changed[:3, :3] @ baseline[:3, :3].T
            angular = np.array([delta_rotation[2, 1] - delta_rotation[1, 2],
                                delta_rotation[0, 2] - delta_rotation[2, 0],
                                delta_rotation[1, 0] - delta_rotation[0, 1]]) / 2
            jacobian[:, column] = np.r_[changed[:3, 3] - baseline[:3, 3], angular] / epsilon
        np.testing.assert_allclose(actual, jacobian @ native @ jacobian.T, atol=1e-7)

    def test_stationary_base_center_during_offset_imu_rotation(self):
        extra = euler_matrix(.1, -.2, .4)
        extra[:3, 3] = [.3, -.2, .1]
        bases = []
        for yaw in (1., 1.02):
            base = rotation_matrix(yaw, [0, 0, 1]);base[:3, 3] = [4, 5, .1]
            body = base @ np.linalg.inv(extra)
            actual, _ = module.convert(body, extra, np.eye(6))
            bases.append(actual)
        linear, angular = module.difference(*bases, .1)
        np.testing.assert_allclose(linear, [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(angular, [0, 0, .2], atol=1e-12)

    def test_wrap_at_pi_and_velocity_in_current_base_frame(self):
        previous = rotation_matrix(math.pi - .01, [0, 0, 1])
        current = rotation_matrix(-math.pi + .01, [0, 0, 1])
        current[:3, 3] = [-.01, 0, 0]
        linear, angular = module.difference(previous, current, .1)
        self.assertAlmostEqual(angular[2], .2)
        self.assertAlmostEqual(linear[0], .1 * math.cos(.01))

    def test_zero_covariance_does_not_claim_perfect_pose(self):
        _, covariance = module.convert(np.eye(4), np.eye(4), np.zeros((6, 6)))
        np.testing.assert_array_equal(np.diag(covariance), np.full(6, 1e6))

    def test_invalid_quaternion_covariance_and_time_are_rejected(self):
        for quaternion in ([0, 0, 0, 0], [0, 0, float('nan'), 1]):
            with self.assertRaises(ValueError):module.pose_matrix([0, 0, 0], quaternion)
        for covariance in (np.full((6, 6), float('nan')), -np.eye(6)):
            with self.assertRaises(ValueError):module.convert(np.eye(4), np.eye(4), covariance)
        for dt in (0, -.1, float('nan')):
            with self.assertRaises(ValueError):module.difference(np.eye(4), np.eye(4), dt)


if __name__ == '__main__':unittest.main()
