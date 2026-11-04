"""Independent oracle geometry check AFTER inference; never feeds the model."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dense_path(path, spacing=.02):
    pieces = []
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b-a) / spacing)))
        pieces.append(np.linspace(a, b, n+1))
    return np.concatenate(pieces)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists(): raise RuntimeError('Use new output')
    rows = []
    for name, capture in [('jm', 'jm_capture_002'), ('keti', 'keti_capture_001')]:
        folder = ROOT / 'artifacts/xnavdp_20260929'
        meta = json.loads((folder/capture/'observation.json').read_text())
        inference = json.loads((folder/(name+'_inference_001')/'result.json').read_text())
        result = inference['results'][-1]
        if name == 'jm':
            scene = json.loads((ROOT/'artifacts/jm_stop_pose_20260929/scene_001.json').read_text())
            obstacles = scene['obstacles']; floor = scene['floor_z']
        else:
            scene = json.loads((ROOT/'artifacts/keti_navigation_20260929/plan_002/scenarios.json').read_text())
            trial = next(s for s in scene['scenarios'] if s['seed'] == 42)
            obstacles = scene['obstacles'] + [trial['random_obstacle']]; floor = scene['config']['floor_z']
        points = dense_path(np.vstack([meta['base_world'][:2], result['world_path_xy']]))
        nearest = None
        for obstacle in obstacles:
            lo, hi = np.array(obstacle['min']), np.array(obstacle['max'])
            if hi[2] <= floor+.15 or lo[2] >= floor+1.75 or '/H1Fleet/H1_1/' in obstacle['path']:
                continue
            delta = np.maximum(np.maximum(lo[:2]-points, points-hi[:2]), 0.)
            distance = np.linalg.norm(delta, axis=1)
            idx = int(np.argmin(distance))
            candidate = {'path': obstacle['path'], 'centerline_distance_m': float(distance[idx]),
                         'proxy_clearance_m': float(distance[idx]-.38), 'point_xy': points[idx].tolist()}
            if nearest is None or candidate['proxy_clearance_m'] < nearest['proxy_clearance_m']:
                nearest = candidate
        path_local = np.array(result['trajectory_local_m'])
        goal_local = np.array(meta['goal_local_m'])
        projection = path_local[:, :2] @ (goal_local[:2] / np.linalg.norm(goal_local[:2]))
        layers_ok = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p,h in meta['source_sha256'].items())
        row = {'world': name, 'scope': 'post-inference oracle AABB cylinder-proxy audit, NOT physical collision proof',
               'oracle_geometry_was_model_input': False, 'body_proxy_radius_m': .38,
               'nearest_obstacle': nearest, 'sample_spacing_max_m': .02,
               'source_layers_unchanged': layers_ok, 'robot_commands_sent': 0,
               'smallest_initial_forward_increment_m': float(np.diff(np.r_[0., path_local[:8, 0]]).min()),
               'max_goal_direction_overshoot_m': float(max(0., projection.max()-np.linalg.norm(goal_local[:2]))),
               'output_horizon_progress_m': float(path_local[-1, 0]),
               'whole_task_success_verified': False}
        if name == 'keti':
            row['random_obstacle_front_world_x'] = trial['random_obstacle']['min'][0]
            row['path_reaches_random_obstacle_front'] = bool(points[:, 0].max() >= trial['random_obstacle']['min'][0])
        rows.append(row)
    args.out.write_text(json.dumps({'results': rows, 'robot_motion_tested': False}, indent=2), encoding='utf-8')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__': main()
