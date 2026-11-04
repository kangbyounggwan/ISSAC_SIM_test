import unittest
import numpy as np
from jm_xnavdp.physical_planner import select_path, truncate_goal_plane, follow


class PhysicalPlannerTests(unittest.TestCase):
    def path(self):
        return np.c_[np.linspace(.04,1.2,24),np.zeros((24,2))]

    def test_goal_plane(self):
        raw=self.path(); path=truncate_goal_plane(raw,np.array([.8,0]))
        self.assertAlmostEqual(path[-1,0],.8)
        self.assertEqual(raw[-1,0],1.2)

    def test_best_valid_not_best_q(self):
        good=self.path(); bad=good.copy(); bad[4,0]=-.2
        result=select_path([bad,good],[10.,1.],np.array([2.,0]),np.empty((0,2)))
        self.assertTrue(result['pass']); self.assertEqual(result['selected_id'],1)

    def test_no_collision_fallback(self):
        result=select_path([self.path()],[1.],np.array([2.,0]),np.array([[.6,0.]]))
        self.assertFalse(result['pass'])
        self.assertIn('OBSERVED_SURFACE_CLEARANCE',result['candidates'][0]['reject'])

    def test_nonfinite(self):
        bad=self.path(); bad[0,0]=np.nan
        self.assertFalse(select_path([bad],[1.],np.array([2.,0]),np.empty((0,2)))['pass'])

    def test_follower_envelope(self):
        command=follow(self.path()[:,:2],[0,0],0,[2,0])
        self.assertLessEqual(command[0],.4); self.assertEqual(command[1],0.)
        self.assertLessEqual(abs(command[2]),.45)

    def test_receding_prefix_does_not_claim_tail_safe(self):
        result=select_path([self.path()],[1.],np.array([2.,0]),np.array([[1.2,0.]]),commit_horizon=.6)
        self.assertTrue(result['pass'])
        self.assertAlmostEqual(result['path_body_xy'][-1][0],.6)
        self.assertEqual(result['candidates'][0]['raw_endpoint'],[1.2,0.])
        self.assertFalse(select_path([self.path()],[1.],np.array([2.,0]),np.array([[.4,0.]]),commit_horizon=.6)['pass'])


if __name__=='__main__': unittest.main()
