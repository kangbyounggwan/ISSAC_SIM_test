"""Decode recorded video and summarize measured detour evidence without editing it."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import imageio.v2 as imageio


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    output = args.run / 'video_review.json'
    if output.exists():
        raise RuntimeError('Review already exists')
    read = lambda name: json.loads((args.run/name).read_text(encoding='utf-8'))
    manifest, result, rows = read('manifest.json'), read('result.json'), read('telemetry.json')
    config, scenario = manifest['config'], manifest['scenario']
    start, goal = np.array(config['start_xy']), np.array(config['goal_xy'])
    forward = (goal-start)/np.linalg.norm(goal-start)
    lateral = np.array([-forward[1],forward[0]])
    positions = np.array([r['position_world'][:2] for r in rows])
    obstacle = scenario['random_obstacle']
    box_corners = [np.array([x,y]) for x in (obstacle['min'][0],obstacle['max'][0])
                   for y in (obstacle['min'][1],obstacle['max'][1])]
    obstacle_far = max(float((p-start)@forward) for p in box_corners)
    pass_threshold = obstacle_far + config['runtime_footprint_radius_m']
    passed = next((r['time_s'] for r,p in zip(rows,positions) if (p-start)@forward > pass_threshold), None)
    video = args.run/'keti_detour_720p.mp4'
    reader = imageio.get_reader(str(video))
    metadata = reader.get_meta_data()
    frame_count = reader.count_frames()
    reader.close()
    report = {
        'video': str(video.resolve()), 'sha256': hashlib.sha256(video.read_bytes()).hexdigest(),
        'width': metadata['size'][0], 'height': metadata['size'][1], 'fps': metadata['fps'],
        'codec': metadata['codec'], 'duration_s': metadata['duration'], 'decoded_frames': frame_count,
        'video_contract_pass': metadata['size']==(1280,720) and metadata['fps']==24. and frame_count==result['frames'],
        'obstacle_far_edge_passed_s': passed,
        'max_lateral_deviation_from_direct_m': float(np.max(abs((positions-start)@lateral))),
        'final_arrival_pass': result['pass'], 'final_failure': result['failure'],
        'max_recorded_obstacle_contact_N': result['max_random_obstacle_contact_N'],
        'min_recorded_footprint_proxy_clearance_m': result['min_footprint_proxy_clearance_m'],
        'scope': 'Measured detour geometry and video integrity, not full mesh collision/perception certification',
        'source_world_unchanged': result['source_world_unchanged'],
        'no_root_teleport': not result['root_writes_during_motion'],
        'map_source': result['map_source'],
    }
    output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
