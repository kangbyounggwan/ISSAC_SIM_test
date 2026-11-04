import math
import unittest
import torch
from jm_wb5_turn.goal_math import (wrap_angle, phase_from_steps, relative_xy,
    goal_features, feedback_command, success_now, expand_actor_state)


class GoalPoseTests(unittest.TestCase):
    def test_integer_phase_boundaries(self):
        torch.testing.assert_close(phase_from_steps(torch.tensor([0,99,100,699,700,899])), torch.tensor([0,0,1,1,2,2]))

    def test_yaw_wrap_increment(self):
        got = wrap_angle(torch.tensor([math.radians(-358), math.radians(358)]))
        torch.testing.assert_close(got, torch.tensor([math.radians(2),math.radians(-2)]))

    def test_body_frame_position(self):
        got = relative_xy(torch.tensor([[1.,0.],[1.,0.]]), torch.zeros(2,2), torch.tensor([0.,math.pi/2]))
        torch.testing.assert_close(got, torch.tensor([[1.,0.],[0.,-1.]]), atol=1e-6, rtol=1e-5)

    def test_signed_180_observation(self):
        got = goal_features(torch.zeros(2,2), torch.tensor([math.pi,-math.pi]), torch.ones(2,dtype=torch.bool),torch.ones(2,dtype=torch.long))
        self.assertEqual(got.shape, (2,7))
        torch.testing.assert_close(got[:,4],torch.tensor([1.,-1.]))

    def test_legacy_walk_observation_zero(self):
        got = goal_features(torch.ones(2,2), torch.ones(2), torch.zeros(2,dtype=torch.bool), torch.ones(2,dtype=torch.long))
        self.assertEqual(int(torch.count_nonzero(got)),0)

    def test_feedback_bounds_sign_brake_legacy(self):
        raw=torch.tensor([[.8,0.,.2]]*5)
        got=feedback_command(raw,torch.tensor([0,1,2,1,3]),torch.tensor([1,1,1,2,2]),torch.ones(5,2),torch.tensor([1.,3.,-3.,1.,1.]))
        torch.testing.assert_close(got[0],torch.zeros(3))
        torch.testing.assert_close(got[1],torch.tensor([.15,.15,.6]))
        torch.testing.assert_close(got[2],torch.tensor([.15,.15,-.6]))
        torch.testing.assert_close(got[3],torch.zeros(3))
        torch.testing.assert_close(got[4],raw[4])

    def test_success_requires_all_conditions(self):
        n=9
        xy=torch.zeros(n); yaw=torch.zeros(n); speed=torch.zeros(n); rate=torch.zeros(n)
        tilt=torch.zeros(n); height=torch.ones(n)*.9; feet=torch.ones(n,dtype=torch.bool)
        mask=feet.clone(); phase=torch.ones(n,dtype=torch.long)
        xy[1]=.11; yaw[2]=math.radians(6); speed[3]=.051; rate[4]=.051
        tilt[5]=math.radians(11); height[6]=.79; feet[7]=False; phase[8]=0
        got=success_now(xy,yaw,speed,rate,tilt,height,feet,mask,phase)
        self.assertEqual(got.tolist(),[True]+[False]*8)

    def test_zero_append_preserves_actor(self):
        original=torch.nn.Sequential(torch.nn.Linear(69,128),torch.nn.ELU(),torch.nn.Linear(128,19))
        expanded=torch.nn.Sequential(torch.nn.Linear(76,128),torch.nn.ELU(),torch.nn.Linear(128,19))
        expanded.load_state_dict(expand_actor_state(original.state_dict(),expanded.state_dict()))
        x=torch.randn(64,76)
        torch.testing.assert_close(original(x[:,:69]),expanded(x),atol=1e-6,rtol=1e-6)


if __name__=='__main__': unittest.main()
