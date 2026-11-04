import unittest
from jm_wb5_turn.live_command import decode, slew


class LiveCommandTest(unittest.TestCase):
    def command(self, **kwargs):
        return dict(session='s', at=10., enabled=True, velocity=[.18,0.,.25], view='third', **kwargs)

    def test_fresh_and_stale(self):
        self.assertTrue(decode(self.command(), 's', 10.2)['enabled'])
        self.assertFalse(decode(self.command(), 's', 10.6)['enabled'])
        self.assertFalse(decode(self.command(), 'other', 10.2)['enabled'])
        self.assertFalse(decode(self.command(), 's', 9.)['enabled'])

    def test_invalid_and_disarmed(self):
        for velocity in ([.19,0.,0.], [0.,.01,0.], [0.,0.,float('nan')], 'unsafe'):
            c=self.command();c['velocity']=velocity
            self.assertFalse(decode(c,'s',10.1)['enabled'])
        c=self.command();c['enabled']=False
        self.assertEqual(decode(c,'s',10.1)['velocity'],[0.,0.,0.])

    def test_slew_does_not_modify_joints(self):
        self.assertEqual(slew([0.,0.,0.],[.18,0.,.25]),[.008,0.,.012])


if __name__ == '__main__': unittest.main()
