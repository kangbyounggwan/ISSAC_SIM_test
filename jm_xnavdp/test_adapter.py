import unittest
import numpy as np
from jm_xnavdp.adapter import local_goal, world_path, propose_command


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.path = np.zeros((24, 3)); self.path[:, 0] = np.linspace(.05, 1.2, 24)

    def test_yaw90_forward_and_left(self):
        np.testing.assert_allclose(local_goal([10, 20], np.pi/2, [9, 22]), [2, 1, 0], atol=1e-6)

    def test_world_rotation(self):
        out = world_path(self.path, [10, 20], np.pi/2)
        np.testing.assert_allclose(out[:, 0], 10)
        np.testing.assert_allclose(out[:, 1], 20 + self.path[:, 0])

    def test_default_never_executes(self):
        r = propose_command(self.path, 0.)
        self.assertFalse(r['execute']); self.assertEqual(r['command'], [0., 0., 0.])

    def test_stale_rejected(self):
        self.assertIn('STALE_OBSERVATION', propose_command(self.path, .51)['reasons'])

    def test_nan_rejected(self):
        self.path[0, 1] = np.nan
        self.assertEqual(propose_command(self.path, 0)['reasons'], ['INVALID_TRAJECTORY'])

    def test_shape_rejected(self):
        self.assertFalse(propose_command(self.path[:3], 0)['execute'])

    def test_reverse_rejected(self):
        self.path[:3, 0] = -.1
        self.assertIn('UNVALIDATED_REVERSE', propose_command(self.path, 0)['reasons'])

    def test_no_lateral_command(self):
        self.path[:, 1] = .1
        r = propose_command(self.path, 0., True, True, True)
        self.assertEqual(r['proposed_command_only'][1], 0.)
        self.assertLessEqual(r['proposed_command_only'][0], .4)
        self.assertLessEqual(abs(r['proposed_command_only'][2]), .45)

    def test_invalid_goal(self):
        with self.assertRaises(ValueError):
            local_goal([0, 0], 0, [np.nan, 1])


if __name__ == '__main__':
    unittest.main()
