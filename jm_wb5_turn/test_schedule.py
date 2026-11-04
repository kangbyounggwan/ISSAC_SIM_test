import unittest
import torch
from jm_wb5_turn.schedule import scheduled_velocity


class ScheduleTests(unittest.TestCase):
    def test_balanced_signs_no_translation(self):
        raw=torch.tensor([[.5,.1,.2]]*4); group=torch.arange(4)
        out=scheduled_velocity(raw,group,torch.ones(4)*3)
        torch.testing.assert_close(out[0],torch.zeros(3))
        torch.testing.assert_close(out[1],torch.tensor([0.,0.,.2]))
        torch.testing.assert_close(out[2],torch.tensor([0.,0.,-.2]))
        torch.testing.assert_close(out[3],raw[3])
        torch.testing.assert_close(raw,torch.tensor([[.5,.1,.2]]*4))

    def test_stop_turn_stop_boundaries(self):
        raw=torch.tensor([[0.,0.,.3]]*5); group=torch.ones(5,dtype=torch.long)
        out=scheduled_velocity(raw,group,torch.tensor([0.,1.99,2.,9.99,10.]))
        torch.testing.assert_close(out[:,2],torch.tensor([0.,0.,.3,.3,0.]))

    def test_mixed_episode_ages_do_not_share_state(self):
        raw=torch.tensor([[0.,0.,.2]]*3); group=torch.tensor([1,2,1])
        out=scheduled_velocity(raw,group,torch.tensor([3.,1.,11.]))
        torch.testing.assert_close(out[:,2],torch.tensor([.2,0.,0.]))


if __name__=='__main__': unittest.main()
