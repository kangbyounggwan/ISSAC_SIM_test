"""Read-only JM rack geometry extraction; no source world mutation."""
from pathlib import Path
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import _usd_boot
from pxr import Usd,UsdGeom

source=ROOT/'jm_factory_world_atlas_h1.usd'
stage=Usd.Stage.Open(str(source))
bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'])
slots=[('slot_1','H1_1','H1_1','cont_008'),('slot_2','Atlas_1','H1_3','cont_009'),
       ('slot_3','H1_2','H1_2','cont_000'),('slot_4','Atlas_2','H1_4','cont_015')]
goals=[]
for slot,robot,new_robot,rack in slots:
    path='/World/Dadong/F2/Products/RACK_'+rack
    prim=stage.GetPrimAtPath(path)
    if not prim: raise RuntimeError('Missing rack '+path)
    bound=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
    lo,hi=bound.GetMin(),bound.GetMax()
    x=(lo[0]+hi[0])/2
    y=lo[1]-.60
    goals.append({'slot':slot,'original_robot':robot,'robot':new_robot,
                  'robot_prim':'/World/H1Fleet/'+new_robot,'rack_path':path,
                  'rack_aabb_min':list(lo),'rack_aabb_max':list(hi),
                  'dock_world_xy':[x,y],'dock_yaw_deg':90,
                  'proposed_reset_world_xy':[x,y-1.2],
                  'reset_is_episode_initialization_not_motion':True,
                  'full_world_corridor_collision_validated':False})
result={'world':str(source),'world_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'status':'EXTRACTED_GOALS_NOT_YET_FOUR_ROBOT_VALIDATED',
        'floor_z':6.075,'dock_pelvis_to_rack_front_m':.60,
        'training_scope':'compact single-rack primitive; factory evaluation is separate',
        'composition':'4x H1, user approved 2026-09-09; original identities retained for traceability',
        'goals':goals}
out=ROOT/'jm_coop4_rl/rack_goals.json'
out.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
