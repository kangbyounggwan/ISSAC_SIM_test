import unittest
import math
import json
import numpy as np
from jm_xnavdp.transfer import RackTransfer


class TransferTests(unittest.TestCase):
    def make(self):
        return RackTransfer([36.17,18.2], [38.42,18.2], math.pi/2, [])

    def test_explicit_assumption_and_hold(self):
        m=self.make()
        self.assertFalse(m.assumed_grasp)
        self.assertFalse(m.start_transfer([36.17,18.2],0))
        m.pickup_complete(np.array([36.17,18.2,6.95],dtype=np.float32),2)
        json.dumps(m.events)
        self.assertFalse(m.events[0]['physical_grasp'])
        self.assertFalse(m.start_transfer([36.17,18.2],2.9))
        self.assertTrue(m.start_transfer([36.17,18.2],3))
        np.testing.assert_allclose(m.path[-1],m.unload)

    def test_turn_before_translation(self):
        m=self.make(); m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        command,_=m.command([36.17,18.2],math.pi/2)
        self.assertEqual(command[0],0)
        self.assertLess(command[2],0)
        self.assertGreater(m.command([36.17,18.2],0)[0][0],0)

    def test_dock_latch_not_fake_success(self):
        m=self.make(); m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        command,phase=m.command([38.35,18.2],0)
        self.assertEqual(phase,'UNLOAD_ALIGN_STOP')
        self.assertEqual(command[0],0)
        self.assertEqual(m.state,'ASTAR_TRANSFER')
        np.testing.assert_allclose(m.unload,[38.42,18.2])

    def test_occupied_goal_fails_without_relocation(self):
        m=self.make(); m.obstacles=[dict(min=[38.3,18.1],max=[38.5,18.3])]
        m.pickup_complete([36.17,18.2],0)
        with self.assertRaises(RuntimeError): m.start_transfer([36.17,18.2],1)
        np.testing.assert_allclose(m.unload,[38.42,18.2])

    def test_dock_drift_resumes_path(self):
        m=self.make(); m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        self.assertEqual(m.command([38.35,18.2],0)[1],'UNLOAD_ALIGN_STOP')
        command,phase=m.command([38.15,18.2],0)
        self.assertEqual(phase,'ASTAR_TRANSFER')
        self.assertGreater(command[0],0)
        self.assertFalse(m.dock_latched)

    def test_positive_departure_is_velocity_command_only(self):
        m=RackTransfer([36.17,18.2],[38.42,18.2],math.pi/2,[],departure_turn='positive')
        m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        command,phase=m.command([36.17,18.2],math.pi/2)
        np.testing.assert_allclose(command,[0,0,.3])
        self.assertEqual(phase,'ASTAR_DEPARTURE_TURN_POSITIVE')
        command,_=m.command([36.17,18.2],0)
        self.assertGreater(command[0],0)

    def test_backoff_measured_then_plan(self):
        m=RackTransfer([36.17,18.2],[38.42,18.2],math.pi/2,[],backoff=.25)
        m.pickup_complete([36.17,18.2],0)
        self.assertFalse(m.start_transfer([36.17,18.2],1))
        self.assertEqual(m.state,'RETREAT_FROM_RACK')
        self.assertLess(m.command([36.17,18.2],math.pi/2)[0][0],0)
        self.assertFalse(m.start_transfer([36.17,18.1],2))
        self.assertTrue(m.start_transfer([36.17,17.94],3))
        np.testing.assert_allclose(m.path[0],[36.17,17.94])
        np.testing.assert_allclose(m.unload,[38.42,18.2])

    def test_backoff_timeout_not_teleport(self):
        m=RackTransfer([36.17,18.2],[38.42,18.2],math.pi/2,[],backoff=.25)
        m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        self.assertFalse(m.start_transfer([36.17,18.2],11.1))
        self.assertEqual(m.failure,'BACKOFF_TIMEOUT')
        self.assertIsNone(m.path)

    def test_strafe_retains_rack_heading(self):
        m=RackTransfer([36.17,18.2],[38.42,18.2],math.pi/2,[],mode='strafe')
        m.pickup_complete([36.17,18.2],0); m.start_transfer([36.17,18.2],1)
        cmd,phase=m.command([36.17,18.2],math.pi/2)
        self.assertEqual(phase,'ASTAR_STRAFE')
        self.assertAlmostEqual(cmd[0],0)
        self.assertEqual(cmd[1],-.15)
        self.assertEqual(cmd[2],0)

if __name__=='__main__': unittest.main()
