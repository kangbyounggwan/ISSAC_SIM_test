"""Create an additive four-H1 JM world; never rewrite the original factory."""
from pathlib import Path
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import _usd_boot
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf

project=ROOT/'jm_coop4_rl'
goals=json.loads((project/'rack_goals.json').read_text(encoding='utf-8'))
source=Path(goals['world'])
asset=ROOT/'jm_wb7_pinkik/assets/h1_wb7_standalone.usd'
output=project/'assets/jm_world_h1_fleet.usda'
output.parent.mkdir(parents=True,exist_ok=True)
source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
layer=Sdf.Layer.CreateNew(str(output)) if not output.exists() else Sdf.Layer.FindOrOpen(str(output))
layer.Clear()
layer.subLayerPaths=[source.as_posix()]
stage=Usd.Stage.Open(layer)
stage.SetEditTarget(layer)
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
stage.SetMetadata('metersPerUnit',1.)
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
stage.GetPrimAtPath('/World/Robots').SetActive(False)
if stage.GetPrimAtPath('/World/Graphs'):
    stage.GetPrimAtPath('/World/Graphs').SetActive(False)
if stage.GetPrimAtPath('/World/Dadong/Roof'):
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/Dadong/Roof')).MakeInvisible()
fleet=UsdGeom.Xform.Define(stage,'/World/H1Fleet')
fleet.GetPrim().SetCustomDataByKey('jm:controller','frozen WB5 walker + learned rack navigation')
for item in goals['goals']:
    prim=UsdGeom.Xform.Define(stage,item['robot_prim']).GetPrim()
    prim.GetReferences().AddReference(asset.as_posix())
    xf=UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    x,y=item['proposed_reset_world_xy']
    xf.AddTranslateOp().Set(Gf.Vec3d(x,y,goals['floor_z']+.98))
    xf.AddRotateZOp().Set(item['dock_yaw_deg'])
    prim.SetCustomDataByKey('jm:slot',item['slot'])
    prim.SetCustomDataByKey('jm:originalRobot',item['original_robot'])
    prim.SetCustomDataByKey('jm:rack',item['rack_path'])
    prim.SetCustomDataByKey('jm:goalXY',Gf.Vec2d(*item['dock_world_xy']))
layer.Save()

# Static integrity check of the composed world, not just the authored layer.
roots=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(roots)==4,[str(p.GetPath()) for p in roots]
records=[]
for item in goals['goals']:
    robot=stage.GetPrimAtPath(item['robot_prim'])
    prims=list(Usd.PrimRange(robot))
    fixed=[str(p.GetPath()) for p in prims if p.IsA(UsdPhysics.Joint) and not UsdPhysics.Joint(p).GetBody0Rel().GetTargets()]
    kin=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get()]
    count=sum(p.IsA(UsdPhysics.RevoluteJoint) for p in prims)
    colliders=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims)
    assert not fixed and not kin and count==33 and colliders>0,(item['robot'],fixed,kin,count,colliders)
    records.append({'robot':item['robot'],'path':item['robot_prim'],'rack':item['rack_path'],
                    'revolute_joints':count,'colliders':colliders,'fixed_to_world':fixed,'kinematic_bodies':kin})
assert source_hash==hashlib.sha256(source.read_bytes()).hexdigest(),'Source world unexpectedly changed'
result={'composition_pass':True,'dynamic_motion_tested':False,'all_four_rack_arrivals_validated':False,
        'world':str(output),'source_world_preserved':True,'source_sha256':source_hash,
        'robot_asset':str(asset),'robot_asset_sha256':hashlib.sha256(asset.read_bytes()).hexdigest(),
        'active_articulation_roots':[str(p.GetPath()) for p in roots],
        'robots':records,
        'warning':'Use the fleet RL evaluator to apply the audited initial joints and drives. Raw USD Play alone does not run a walking policy.'}
(project/'assets/fleet_manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
