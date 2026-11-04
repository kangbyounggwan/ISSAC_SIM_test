import math
import json
import unittest
from jm_wb5_turn.jm_sequence import WalkTurnSequence,ContinuousWalkTurnSequence


class TestSequence(unittest.TestCase):
    def test_fixed_goals(self):
        s=WalkTurnSequence([2.,3.])
        self.assertEqual(s.a,[2.,4.]); self.assertEqual(s.b,[1.,4.])

    def test_turn_before_walk_b(self):
        s=WalkTurnSequence([0.,0.])
        for t in (0.,1.,2.): s.update(t,[0.,0.],math.pi/2,0.,0.,0.)
        self.assertEqual(s.phase,'MOVE_A')
        s.update(3.,[0.,1.],math.pi/2,0.,0.,0.)
        self.assertEqual(s.phase,'BRAKE_A')
        s.update(3.1,[0.,1.],math.pi/2,0.,0.,0.)
        s.update(4.2,[0.,1.],math.pi/2,0.,0.,0.)
        self.assertEqual(s.phase,'LEFT_TURN')
        self.assertGreater(s.update(4.3,[0.,1.],math.pi/2,0.,0.,0.)[2],0)
        s.update(8.,[0.,1.],math.pi,0.,0.,0.)
        self.assertEqual(s.phase,'BRAKE_TURN')
        s.update(8.1,[0.,1.],math.pi,0.,0.,0.)
        s.update(9.2,[0.,1.],math.pi,0.,0.,0.)
        self.assertEqual(s.phase,'MOVE_B')
        s.update(12.,[-1.,1.],math.pi,0.,0.,0.)
        s.update(12.1,[-1.,1.],math.pi,0.,0.,0.)
        s.update(13.2,[-1.,1.],math.pi,0.,0.,0.)
        self.assertEqual(s.phase,'DONE')

    def test_timeout_does_not_relax(self):
        s=WalkTurnSequence([0.,0.]); s.update(7.,[0.,0.],0.,1.,1.,0.)
        self.assertEqual(s.failure,'SETTLE_TIMEOUT')
        self.assertEqual(s.a,[0.,1.])

    def test_no_turn_if_body_moving(self):
        s=WalkTurnSequence([0.,0.]); s.phase='BRAKE_A'
        for t in (0.,1.,2.,3.): s.update(t,[0.,1.],math.pi/2,.2,0.,0.)
        self.assertEqual(s.phase,'BRAKE_A')

    def test_turn_drift_abort(self):
        s=WalkTurnSequence([0.,0.]); s.phase='LEFT_TURN'; s.turn_origin=[0.,1.]
        s.update(1.,[.36,1.],math.pi/2,0.,0.,0.)
        self.assertEqual(s.failure,'TURN_DRIFT_ABORT')

    def test_invalid_state(self):
        s=WalkTurnSequence([0.,0.]); s.update(0.,[float('nan'),0.],0.,0.,0.,0.)
        self.assertEqual(s.failure,'NONFINITE_STATE')

    def test_continuous_is_separate_no_stop_claim(self):
        s=ContinuousWalkTurnSequence([0.,0.])
        s.update(0.,[0.,0.],math.pi/2,0.,0.,0.)
        s.update(2.,[0.,0.],math.pi/2,.11,.16,5.)
        self.assertEqual(s.phase,'MOVE_A')
        s.update(4.,[0.,1.],math.pi/2,.11,.16,5.)
        self.assertEqual(s.phase,'LEFT_TURN')
        s.update(8.,[0.,1.],math.pi,.11,.16,5.)
        self.assertEqual(s.phase,'MOVE_B')
        s.update(12.,[-1.,1.],math.pi,.11,.16,5.)
        self.assertEqual(s.phase,'STOP_AUDIT')
        s.update(14.,[-1.,1.],math.pi,.11,.16,5.)
        self.assertEqual(s.phase,'DONE')
        self.assertEqual(s.LIMITS,WalkTurnSequence.LIMITS)

    def test_events_json_serializable(self):
        import numpy as np
        s=ContinuousWalkTurnSequence([0.,0.])
        s.update(2.,np.array([0.,0.],dtype=np.float32),math.pi/2,0.,0.,0.)
        self.assertIn('MOVE_A',json.dumps(s.events))


if __name__=='__main__': unittest.main()
