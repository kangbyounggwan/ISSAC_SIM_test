"""Replay ONLY the safety filter on identical saved candidates and observations."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from jm_xnavdp.physical_planner import depth_points, select_path
from jm_xnavdp.adapter import local_goal
p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True); p.add_argument('--out',type=Path,required=True); a=p.parse_args()
if a.out.exists(): raise RuntimeError('Fresh output required')
memory=[]; reports=[]
for path in sorted(a.run.glob('observation_*.npz')):
    data=np.load(path,allow_pickle=False); raw=json.loads(path.with_name(path.name.replace('observation','inference').replace('.npz','.json')).read_text())
    base=data['base_world']; yaw=float(data['yaw_rad']); c,s=np.cos(yaw),np.sin(yaw); rot=np.array([[c,s],[-s,c]])
    cloud=depth_points(data['depth_m'],data['intrinsic'],base[2]); memory.append(cloud@rot+base[:2]); memory=memory[-8:]
    combined=(np.concatenate(memory)-base[:2])@rot.T
    goal=local_goal(base[:2],yaw,data['goal_world_xy'])
    result=select_path(raw['candidates_local_m'],raw['q_values'],goal,combined,commit_horizon=.8)
    old=raw['simulation_selection'].get('selected_id')
    check=next((r for r in result['candidates'] if r['id']==old),None)
    reports.append({'time_s':raw['simulation_time_s'],'original_selected_id':old,'original_selected_after_memory':check,
                    'memory_filter_result':result,'current_points':len(cloud),'memory_points':len(combined)})
    print(raw['simulation_time_s'],old,check['reject'] if check else None,result['pass'],flush=True)
a.out.write_text(json.dumps({'scope':'Same saved RGB-D, pose and model candidates; only depth memory changes. NOT a new rollout.',
                            'run':str(a.run),'memory_frames':8,'reports':reports},indent=2),encoding='utf-8')
