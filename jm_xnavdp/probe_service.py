"""Exercise the real loopback model service with saved RGB-D; no actuation."""
import argparse
import io
import json
from pathlib import Path
import time
import numpy as np
import requests

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--observation', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--port', type=int, default=8891)
args = p.parse_args()
if args.out.exists(): raise RuntimeError('Use a new output file')
meta = json.loads((args.observation / 'observation.json').read_text())
data = np.load(args.observation / 'observation.npz', allow_pickle=False)
stream = io.BytesIO()
# Preserve the actual old file time; do not claim that replayed data is fresh.
np.savez_compressed(stream, rgb=data['rgb'], depth_m=data['depth_m'], base_world=np.asarray(meta['base_world']),
                    yaw_rad=np.array(meta['yaw_rad']), goal_world_xy=np.asarray(meta['goal_world_xy']),
                    intrinsic=np.asarray(meta['intrinsic']),
                    captured_unix_s=np.array((args.observation / 'observation.npz').stat().st_mtime))
url = f'http://127.0.0.1:{args.port}'
health = requests.get(url + '/health', timeout=5); health.raise_for_status()
reset = requests.post(url + '/reset', timeout=5); reset.raise_for_status()
start = time.perf_counter()
response = requests.post(url + '/infer', data=stream.getvalue(), headers={'Content-Type': 'application/octet-stream'}, timeout=120)
response.raise_for_status()
result = response.json()
assert result['robot_control_enabled'] is False
assert result['command_gate']['execute'] is False
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps({'scope': 'HTTP model inference with replayed observation; no robot commands',
                                'roundtrip_wall_s': time.perf_counter() - start, 'response': result}, indent=2), encoding='utf-8')
print(json.dumps({'saved': str(args.out), 'model_time_s': result['inference_wall_s'], 'gate': result['command_gate']}))
