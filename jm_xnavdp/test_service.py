import io
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from jm_xnavdp.server import create_app, decode_packet


class ServiceTests(unittest.TestCase):
    def packet(self, **overrides):
        data = dict(rgb=np.zeros((720, 1280, 3), np.uint8), depth_m=np.ones((720, 1280), np.float32),
                    base_world=np.zeros(3), yaw_rad=np.array(0.), goal_world_xy=np.array([2., 0.]),
                    intrinsic=np.array([[650., 0., 640.], [0., 650., 360.], [0., 0., 1.]]),
                    captured_unix_s=np.array(time.time()))
        data.update(overrides)
        stream = io.BytesIO(); np.savez_compressed(stream, **data)
        return stream.getvalue()

    def test_packet(self):
        self.assertEqual(decode_packet(self.packet())['rgb'].shape, (720, 1280, 3))

    def test_nan_pose(self):
        with self.assertRaises(ValueError): decode_packet(self.packet(yaw_rad=np.array(np.nan)))

    def test_unknown_fields(self):
        with self.assertRaises(ValueError): decode_packet(self.packet(obstacle_ground_truth=np.zeros(3)))

    def test_wrong_color_type(self):
        with self.assertRaises(ValueError): decode_packet(self.packet(rgb=np.zeros((720, 1280, 3), np.float32)))

    def test_health_does_not_load_model(self):
        client = create_app().test_client()
        self.assertFalse(client.get('/health').get_json()['model_loaded'])

    def test_planner_configuration_audit(self):
        info=create_app(samples=16,floor_z=6.075,commit_horizon=.8).test_client().get('/health').get_json()
        self.assertEqual(info['configuration']['floor_z'],6.075)
        self.assertEqual(info['configuration']['samples'],16)
        self.assertEqual(info['configuration']['reset_seed'],42)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError): create_app(guidance_scale=2.)
        with self.assertRaises(ValueError): create_app(commit_horizon=-1.)
        with self.assertRaises(ValueError): create_app(depth_memory_frames=99)

    def test_depth_memory_retains_observations_and_reset_clears(self):
        class FakeRuntime:
            def __init__(self, intrinsic):
                self.agent=SimpleNamespace(reset=lambda n:None,_update_last_trajectory_state=lambda *a:None)
                self.audit={'model_sha256':'unit-test-only'}
            def infer(self, *args, **kwargs):
                path=np.c_[np.linspace(.01,.8,24),np.zeros((24,2))].tolist()
                return {'trajectory_local_m':path,'candidates_local_m':[path]*8,'q_values':[1.]*8}
        with patch('jm_xnavdp.server.Runtime',FakeRuntime):
            client=create_app(samples=8,floor_z=0.,depth_memory_frames=8).test_client()
            first=client.post('/infer',data=self.packet()).get_json()
            second=client.post('/infer',data=self.packet()).get_json()
            self.assertGreater(first['depth_surface_points_current'],0)
            self.assertEqual(second['depth_surface_points_with_memory'],2*first['depth_surface_points_current'])
            client.post('/reset')
            reset=client.post('/infer',data=self.packet()).get_json()
            self.assertEqual(reset['depth_surface_points_with_memory'],first['depth_surface_points_current'])

    def test_corrupt_packet_no_trajectory(self):
        client = create_app().test_client()
        response = client.post('/infer', data=b'not an npz')
        self.assertEqual(response.status_code, 400)
        self.assertNotIn('trajectory_local_m', response.get_json())

    def test_guard_fallback_is_explicit_and_commits_executed_path(self):
        committed=[]
        class FakeRuntime:
            def __init__(self,intrinsic):
                self.agent=SimpleNamespace(reset=lambda n:None,_update_last_trajectory_state=lambda path,*a:committed.append(path))
                self.audit={'model_sha256':'test'}
            def infer(self,*a,**kw):
                path=np.c_[np.linspace(.02,.8,24),np.linspace(.01,.4,24),np.zeros(24)].tolist()
                return {'trajectory_local_m':path,'candidates_local_m':[path]*8,'q_values':[1.]*8}
        with patch('jm_xnavdp.server.Runtime',FakeRuntime), patch('jm_xnavdp.server.depth_points',return_value=np.empty((0,2))):
            client=create_app(samples=8,floor_z=0.,route_guide=True,route_guard=True,commit_horizon=.8).test_client()
            result=client.post('/infer',data=self.packet()).get_json()
            self.assertTrue(result['simulation_selection']['pass'])
            self.assertEqual(result['execution_selection']['source'],'DEPTH_ASTAR_REFERENCE')
            self.assertTrue(result['execution_selection']['pass'])
            self.assertTrue(np.allclose(committed[-1][0,:,1],0.))


if __name__ == '__main__': unittest.main()
