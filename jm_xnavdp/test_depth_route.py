import unittest
import numpy as np
from jm_xnavdp.depth_route import DepthRoute


class RouteTests(unittest.TestCase):
    def test_detour_keeps_goal(self):
        xs,ys=np.meshgrid(np.linspace(2.3,2.9,13),np.linspace(-.3,.3,13))
        guide=DepthRoute(); r=guide.update(np.c_[xs.ravel(),ys.ravel()],[0,0],[5,0])
        self.assertTrue(r['pass']); self.assertEqual(r['final_goal_world_xy'],[5.,0.])
        self.assertGreater(max(abs(np.asarray(r['path_world_xy'])[:,1])),.8)

    def test_memory_and_goal_lock(self):
        guide=DepthRoute(); guide.update(np.array([[2.,0.]]),[0,0],[5,0])
        r=guide.update(np.empty((0,2)),[.1,0],[5,0]); self.assertEqual(r['observed_voxels'],1)
        with self.assertRaises(ValueError): guide.update(np.empty((0,2)),[.1,0],[5.1,0])
        guide.reset(); self.assertEqual(len(guide.points),0)

    def test_blocked_start_fails_no_relocation(self):
        r=DepthRoute().update(np.array([[0.,0.]]),[0,0],[5,0])
        self.assertFalse(r['pass']); self.assertEqual(r['reason'],'DEPTH_ROUTE_ENDPOINT_BLOCKED')

    def test_exit_planning_padding_retains_actual_clearance(self):
        r=DepthRoute().update(np.array([[0.,.58]]),[0,0],[5,0])
        self.assertTrue(r['pass'])
        r=DepthRoute().update(np.array([[0.,.25]]),[0,0],[5,0])
        self.assertFalse(r['pass'])

if __name__=='__main__': unittest.main()
