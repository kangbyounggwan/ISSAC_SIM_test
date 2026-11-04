"""One-variable 8 vs 16 samples on fixed actual RGB-D observations."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from jm_xnavdp.runtime import Runtime
from jm_xnavdp.physical_planner import depth_points, select_path

p=argparse.ArgumentParser(description=__doc__); p.add_argument('--out',type=Path,required=True); args=p.parse_args()
if args.out.exists(): raise RuntimeError('Fresh output required')
args.out.mkdir(parents=True)
runtime=None; reports=[]
for name,folder in [('jm','jm_capture_002'),('keti','keti_capture_001')]:
    capture=ROOT/'artifacts/xnavdp_20260929'/folder
    meta=json.loads((capture/'observation.json').read_text()); data=np.load(capture/'observation.npz',allow_pickle=False)
    if runtime is None: runtime=Runtime(meta['intrinsic'])
    cloud=depth_points(data['depth_m'],meta['intrinsic'],meta['base_world'][2]-(6.075 if name=='jm' else 0))
    for samples in [8,16]:
        import torch
        torch.manual_seed(42); np.random.seed(42); runtime.agent.reset(1)
        raw=runtime.infer(data['rgb'],data['depth_m'],meta['goal_local_m'],meta['base_world'],meta['yaw_rad'],samples=samples)
        selected=select_path(raw['candidates_local_m'],raw['q_values'],np.asarray(meta['goal_local_m']),cloud)
        row={'world':name,'sample_count':samples,'raw':raw,'selection':selected,'observed_obstacle_points':len(cloud)}
        reports.append(row)
        print(name,samples,selected['pass'],[(r['id'],r['reject']) for r in selected['candidates']],flush=True)
(args.out/'comparison.json').write_text(json.dumps({'changed_variable':'sample_count','fixed_seed':42,'reports':reports},indent=2),encoding='utf-8')
