import math
from pathlib import Path
import unittest
import tempfile
from types import SimpleNamespace
import numpy as np
from jm_xnavdp.docking import dock_command
from jm_xnavdp.h1_contract import observation, load_constants, gains, RuntimeAudit


class DockTests(unittest.TestCase):
    def test_26cm_does_not_align_final_yaw(self):
        cmd, phase = dock_command([0, 0], 0, [0, .26], 0)
        self.assertEqual(phase, 'DOCK_POSITION_RECOVERY')
        np.testing.assert_allclose(cmd, [0, 0, .3])

    def test_drift_returns_to_position(self):
        goal = np.array([0., 0.])
        self.assertEqual(dock_command([.1, 0], 0, goal, 0)[1], 'DOCK_ALIGN_STOP')
        self.assertEqual(dock_command([.2, 0], 0, goal, 0)[1], 'DOCK_POSITION_RECOVERY')
        np.testing.assert_array_equal(goal, [0, 0])

    def test_no_reverse_or_lateral_and_bounds(self):
        for yaw in np.linspace(-math.pi, math.pi, 30):
            cmd, _ = dock_command([0, 0], yaw, [1, 1], 0)
            self.assertTrue(0 <= cmd[0] <= .3)
            self.assertEqual(cmd[1], 0)
            self.assertLessEqual(abs(cmd[2]), .3)

    def test_boundary_not_success(self):
        self.assertEqual(dock_command([.18, 0], 0, [0, 0], 0)[1], 'DOCK_POSITION_RECOVERY')

    def test_contract_observation_order(self):
        c = load_constants(Path(__file__).resolve().parents[1])
        q = np.asarray(c['WALK_DEFAULT_19'])
        obs = observation(np.ones(3), np.ones(3)*2, np.ones(3)*3,
                          np.ones(3)*4, q, np.ones(19)*6, np.ones(19)*7, q)
        self.assertEqual(obs.shape, (69,))
        np.testing.assert_allclose(obs[12:31], 0)
        np.testing.assert_allclose(obs[-19:], 7)
        self.assertEqual(gains('left_knee_joint'), (200., 5., 300.))

    def test_invalid_observation(self):
        with self.assertRaises(ValueError):
            observation(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3),
                        np.zeros(18), np.zeros(18), np.zeros(18), np.zeros(18))

    def test_readback_detects_post_step_pd_overwrite(self):
        values = [np.array([200.]), np.array([5.])]
        robot = SimpleNamespace(get_articulation_controller=lambda: SimpleNamespace(get_gains=lambda: values),
            _articulation_view=SimpleNamespace(get_max_efforts=lambda: np.array([[300.]])))
        with tempfile.TemporaryDirectory() as tmp:
            audit = RuntimeAudit(Path(tmp)/'audit.json', ['left_knee_joint'], [0])
            audit.check(robot, 0, 0)
            values[0][0] = 35809.
            with self.assertRaisesRegex(RuntimeError, 'RUNTIME_CONTRACT_MISMATCH'):
                audit.check(robot, 1, .005)
            self.assertFalse(audit.report['passed'])

    def test_readback_rejects_wrong_physics_clock(self):
        robot = SimpleNamespace(get_articulation_controller=lambda: SimpleNamespace(
            get_gains=lambda: (np.array([200.]), np.array([5.]))),
            _articulation_view=SimpleNamespace(get_max_efforts=lambda: np.array([[300.]])))
        with tempfile.TemporaryDirectory() as tmp:
            audit = RuntimeAudit(Path(tmp)/'audit.json', ['left_knee_joint'], [0])
            with self.assertRaises(RuntimeError): audit.check(robot, 1, .016)


if __name__ == '__main__': unittest.main()
