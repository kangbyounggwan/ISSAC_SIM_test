"""One-variable sample-count audit on the exact failed physical observation."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from jm_xnavdp.runtime import Runtime
from jm_xnavdp.adapter import local_goal
from jm_xnavdp.physical_planner import depth_points, select_path
p=argparse.ArgumentParser(); p.add_argument('--observation',type=Path,required=True); p.add_argument('--out',type=Path,required=True); a=p.parse_args()
if a.out.exists(): raise RuntimeError('Fresh output required')
d=np.load(a.observation,allow_pickle=False); runtime=Runtime(d['intrinsic']); rows=[]
import torch
for count in (8,16,32):
    torch.manual_seed(42); np.random.seed(42); runtime.agent.reset(1)
    raw=runtime.infer(d['rgb'],d['depth_m'],local_goal(d['base_world'][:2],float(d['yaw_rad']),d['goal_world_xy']),d['base_world'],float(d['yaw_rad']),samples=count)
    selected=select_path(raw['candidates_local_m'],raw['q_values'],local_goal(d['base_world'][:2],float(d['yaw_rad']),d['goal_world_xy']),depth_points(d['depth_m'],d['intrinsic'],d['base_world'][2]))
    rows.append({'sample_count':count,'raw':raw,'selection':selected})
    print(count,selected['pass'],[(c['id'],c['reject']) for c in selected['candidates']],flush=True)
a.out.write_text(json.dumps({'input':str(a.observation),'seed':42,'history':'reset: single frozen observation, not temporal rollout','reports':rows},indent=2),encoding='utf-8')
