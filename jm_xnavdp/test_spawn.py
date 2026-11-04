import unittest
from jm_xnavdp.spawn import sample_spawn, BOUNDS


class SpawnTests(unittest.TestCase):
    def test_seed_replay(self):
        self.assertEqual(sample_spawn([],42),sample_spawn([],42))

    def test_seed_variation_and_bounds(self):
        samples=[sample_spawn([],i)['selected_xy'] for i in range(100)]
        self.assertEqual(len({tuple(p) for p in samples}),100)
        self.assertTrue(all(BOUNDS[0]<=x<=BOUNDS[2] and BOUNDS[1]<=y<=BOUNDS[3] for x,y in samples))

    def test_geometry_reject_before_episode(self):
        obstacles=[dict(min=[35.5,15.9],max=[35.6,16.0])]
        r=sample_spawn(obstacles,42)
        self.assertGreaterEqual(r['obstacle_distance_m'],r['margin_m'])
        self.assertFalse(r['yaw_randomized'])
        self.assertTrue(r['initialization_only'])

    def test_blocked_no_fallback(self):
        with self.assertRaisesRegex(RuntimeError,'NO_VALID_SPAWN'):
            sample_spawn([dict(min=[30,10],max=[40,20])],42,max_attempts=5)

    def test_bad_seed(self):
        for seed in (-1,2**32,2.5):
            with self.assertRaises(ValueError): sample_spawn([],seed)


if __name__=='__main__': unittest.main()
