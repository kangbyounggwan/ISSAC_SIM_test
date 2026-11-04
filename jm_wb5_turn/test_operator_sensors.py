import unittest
import numpy as np
from jm_wb5_turn.operator_sensors import depth_preview,filter_lidar,lidar_preview


class SensorDataTests(unittest.TestCase):
    def test_depth_invalid_values_are_not_fake_distances(self):
        depth=np.array([[np.nan,np.inf,0.,21.,1.,10.]],dtype=np.float32)
        rgb,valid=depth_preview(depth)
        self.assertEqual(valid.tolist(),[[False,False,False,False,True,True]])
        self.assertTrue((rgb[~valid]==0).all())
        self.assertFalse(np.array_equal(rgb[0,4],rgb[0,5]))

    def test_no_hit_and_nonfinite_lidar_points_are_excluded(self):
        points=np.array([[1.,0,0],[20.,0,0],[np.nan,0,0],[.1,0,0]])
        filtered,mask=filter_lidar(points,[1.,20.,np.nan,.1])
        np.testing.assert_allclose(filtered,[[1,0,0]])
        self.assertEqual(mask.tolist(),[True,False,False,False])

    def test_wrong_lidar_coordinate_frame_is_rejected(self):
        with self.assertRaises(ValueError):filter_lidar([[30.,40,7]],[2.])
        with self.assertRaises(ValueError):filter_lidar([[1.,0,0]],[1.,2.])

    def test_point_cloud_display_remains_720p(self):
        self.assertEqual(lidar_preview(np.array([[2.,1.,0.]])).size,(1280,720))


if __name__=='__main__':unittest.main()
