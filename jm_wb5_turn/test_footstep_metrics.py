import unittest
from jm_wb5_turn.footstep_metrics import foot_events,angle_travel


def fixture(air_start=None,air_end=None,rise=0.):
    rows=[]
    for i in range(901):
        t=i*.02
        air=air_start is not None and air_start<=t<air_end
        rows.append(dict(time_s=t,foot_force_n=[0. if air else 300.,300.],
            foot_position_world=[[.03 if air_end is not None and t>=air_end else 0.,0.,.1+(rise if air else 0.)],[0.,.2,.1]],
            foot_yaw_deg=[0.,0.]))
    return rows


class FootstepTest(unittest.TestCase):
    def test_static_is_not_a_step(self):
        self.assertEqual(foot_events(fixture(),0),[])

    def test_already_airborne_at_interval_start_is_not_new_step(self):
        self.assertEqual(foot_events(fixture(1.9,2.2,.03),0),[])

    def test_noise_contact_loss_is_not_step(self):
        self.assertEqual(foot_events(fixture(3.,3.04,.03),0),[])

    def test_no_height_change_not_lift(self):
        self.assertFalse(foot_events(fixture(3.,3.2,.001),0)[0]['verified_lift_landing'])

    def test_supported_lift_landing(self):
        event=foot_events(fixture(3.,3.2,.03),0)[0]
        self.assertTrue(event['verified_lift_landing'])
        self.assertAlmostEqual(event['landing_xy_displacement_m'],.03)

    def test_never_lands_is_not_step(self):
        self.assertFalse(foot_events(fixture(3.,19.,.03),0)[0]['verified_lift_landing'])

    def test_angle_wrap(self):
        self.assertAlmostEqual(angle_travel([{'yaw':179.},{'yaw':-179.}],'yaw'),2.)


if __name__=='__main__':unittest.main()
