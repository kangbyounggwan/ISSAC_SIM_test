import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from jm_wb5_turn.operator_bridge import OperatorBridge


class BridgeTest(unittest.TestCase):
    def test_disabled_has_no_side_effects(self):
        with tempfile.TemporaryDirectory() as d:
            b=OperatorBridge(d,False);b.publish({'phase':'TEST'})
            self.assertFalse(list(Path(d).iterdir()))
            (Path(d)/'cancel.request').touch()
            self.assertFalse(b.cancelled())

    def test_atomic_state_and_cancellation(self):
        with tempfile.TemporaryDirectory() as d:
            b=OperatorBridge(d,True);b.publish({'phase':'MOVE_A','time_s':2.})
            row=json.loads((Path(d)/'live_status.json').read_text())
            self.assertEqual(row['sequence'],1)
            self.assertEqual(row['phase'],'MOVE_A')
            self.assertFalse((Path(d)/'live_status.json.tmp').exists())
            self.assertFalse(b.cancelled());(Path(d)/'cancel.request').touch()
            self.assertTrue(b.cancelled())

    def test_file_contention_is_reported_not_a_physics_exception(self):
        with tempfile.TemporaryDirectory() as d:
            b=OperatorBridge(d,True)
            with patch('jm_wb5_turn.operator_bridge.os.replace',side_effect=PermissionError()):
                b.publish({'phase':'MOVE_A'})
            self.assertEqual(b.write_errors,1)
            b.publish({'phase':'MOVE_A'})
            self.assertEqual(json.loads((Path(d)/'live_status.json').read_text())['write_errors'],1)

    def test_sensor_images_have_fixed_separate_paths(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as d:
            b=OperatorBridge(d,True)
            for kind in ['third','rgb','depth','lidar']:b.frame(Image.new('RGB',(1280,720)),kind)
            self.assertEqual(len(list(Path(d).glob('live_*.jpg'))),4)
            with self.assertRaises(ValueError):b.frame(Image.new('RGB',(2,2)),'../../secret')
            b.sensors({'time_s':1.,'streams':{'depth':{'ready':True}}})
            self.assertEqual(json.loads((Path(d)/'live_sensors.json').read_text())['time_s'],1.)


if __name__=='__main__':unittest.main()
