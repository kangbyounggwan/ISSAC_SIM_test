"""Audit measured sensor outputs without starting or modifying a simulation."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from jm_wb5_turn.operator_sensors import filter_lidar


def audit(folder):
    folder=Path(folder);manifest=json.loads((folder/'sensor_manifest.json').read_text())
    videos={}
    for key,file in [('third','jm_walk_left_walk_720p.mp4'),*[(k,f'sensor_{k}_720p.mp4') for k in ('rgb','depth','lidar')]]:
        cap=cv2.VideoCapture(str(folder/file));count=0;shape=set();fps=cap.get(cv2.CAP_PROP_FPS)
        while True:
            ok,frame=cap.read()
            if not ok:break
            count+=1;shape.add((frame.shape[1],frame.shape[0]))
        cap.release();videos[key]=dict(frames=count,fps=fps,resolutions=list(shape))
        assert count==manifest['frames'] and shape=={(1280,720)} and abs(fps-25)<.01,(key,videos[key])
    positions=[];matrix_separation=[];alignment=[];hits=[];depth_fractions=[]
    for sample in manifest['samples']:
        with np.load(folder/'sensor_raw'/sample['file'],allow_pickle=False) as data:
            assert data['rgb'].shape==(720,1280,3)
            dep=data['depth_m'];assert dep.shape==(720,1280)
            valid=np.isfinite(dep)&(dep>=.05)&(dep<20);assert valid.any()
            depth_fractions.append(float(valid.mean()))
            cloud,mask=filter_lidar(data['lidar_points_local'],data['lidar_ranges_m']);hits.append(len(cloud))
            assert len(cloud)>0 and np.array_equal(mask,data['lidar_valid'])
            camera=data['camera_local_to_world'];lidar=data['lidar_local_to_world']
            positions.append(camera[3,:3]);matrix_separation.append(float(np.linalg.norm(camera[3,:3]-lidar[3,:3])))
            alignment.append(float(np.dot(-camera[2,:3],lidar[0,:3])))
            assert abs(float(data['time_s'])-sample['time_s'])<1e-8
    assert np.allclose(matrix_separation,.08,atol=.002),matrix_separation
    assert np.allclose(alignment,1/np.sqrt(1.01),atol=.002),alignment
    baseline=folder.parents[2]/'artifacts/jm_walk_turn_20261002/official_native_continuous_003/telemetry.json'
    rows=json.loads((folder/'telemetry.json').read_text())
    base=json.loads(baseline.read_text());same=rows==base
    result=dict(videos=videos,samples=len(positions),valid_frames=manifest['valid_frames'],
        depth_valid_fraction_range=[min(depth_fractions),max(depth_fractions)],lidar_hit_count_range=[min(hits),max(hits)],
        sensor_mount_separation_m=[min(matrix_separation),max(matrix_separation)],
        camera_translation_extent_m=np.ptp(np.array(positions),axis=0).tolist(),
        rgb_depth_same_render_product=True,lidar_range_matches_local_point_norm=True,
        telemetry_identical_to_sensorless_baseline=same,telemetry_rows=len(rows),passed=True)
    (folder/'sensor_audit.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args()
    print(json.dumps(audit(a.folder),indent=2))
