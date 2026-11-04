import json
from pathlib import Path
import unittest
import numpy as np
from keti_navigation.planner import astar, line_free, random_obstacle, segment_box, lookahead_target


class PlannerTests(unittest.TestCase):
    def test_direct_route(self):
        result = astar([0, 0], [4, 0], [], [-1, -2, 5, 2], .1, .4)
        self.assertEqual(result["path_xy"], [[0., 0.], [4., 0.]])
        self.assertFalse(result["direct_path_blocked"])

    def test_obstacle_creates_checked_detour(self):
        obstacles = [{"min": [1.7, -.3, 0], "max": [2.3, .3, 1]}]
        result = astar([0, 0], [4, 0], obstacles, [-1, -2, 5, 2], .1, .4)
        self.assertTrue(result["direct_path_blocked"])
        self.assertGreater(result["length_m"], 4)
        for a, b in zip(result["path_xy"], result["path_xy"][1:]):
            self.assertTrue(line_free(a, b, obstacles, .4))

    def test_no_path_is_failure(self):
        with self.assertRaisesRegex(RuntimeError, "NO_PATH"):
            astar([0, 0], [4, 0], [{"min": [1.7, -3, 0], "max": [2.3, 3, 1]}], [-1, -2, 5, 2], .1, .4)

    def test_occupied_start_is_failure(self):
        with self.assertRaisesRegex(RuntimeError, "occupied"):
            astar([0, 0], [4, 0], [{"min": [-1, -1], "max": [1, 1]}], [-1, -2, 5, 2], .1, .4)

    def test_seed_is_reproducible_not_constant_across_seeds(self):
        config = json.loads(Path(__file__).with_name("config.json").read_text())
        self.assertEqual(random_obstacle(config, 42), random_obstacle(config, 42))
        self.assertNotEqual(random_obstacle(config, 42)["center"], random_obstacle(config, 43)["center"])

    def test_corner_tangent_is_collision(self):
        self.assertTrue(segment_box([0, 0], [2, 2], [1, 0], [2, 1]))

    def test_goal_is_not_moved(self):
        goal = [4, 0]
        obstacle = {"min": [3.5, -.5], "max": [4.5, .5]}
        with self.assertRaises(RuntimeError):
            astar([0, 0], goal, [obstacle], [-1, -2, 5, 2], .1, .4)
        self.assertEqual(goal, [4, 0])

    def test_lookahead_and_progress(self):
        target, progress = lookahead_target(np.array([1., 0.]), np.array([[0., 0.], [4., 0.]]), 0., .45)
        np.testing.assert_allclose(target, [1.45, 0])
        self.assertEqual(progress, 1.)


if __name__ == "__main__":
    unittest.main(verbosity=2)
