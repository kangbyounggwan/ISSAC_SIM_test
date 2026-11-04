import unittest
import math
from jm_xnavdp.turn_audit import audit_turn


class TurnAuditTests(unittest.TestCase):
    def rows(self):
        return [dict(time_s=i/10,yaw_deg=math.degrees(.3*i/10),yaw_rate_rps=.3,
                     position_world=[0,0,1],contact_N=0,tilt_deg=0,actual_physics_elapsed_s=i/10) for i in range(141)]

    def test_sign_and_good_turn(self):
        self.assertTrue(audit_turn(self.rows(),1)['pass'])
        self.assertFalse(audit_turn(self.rows(),-1)['pass'])

    def test_drift(self):
        r=self.rows()
        for row in r: row['position_world'][0]=row['time_s']*.1
        self.assertFalse(audit_turn(r,1)['pass'])

    def test_extra_physics(self):
        r=self.rows()
        for row in r: row['actual_physics_elapsed_s']*=2
        self.assertFalse(audit_turn(r,1)['pass'])

    def test_late_stall_and_incomplete(self):
        r=self.rows()
        for row in r:
            if row['time_s']>=9: row['yaw_rate_rps']=0
        self.assertFalse(audit_turn(r,1)['pass'])
        self.assertFalse(audit_turn([],1)['pass'])
