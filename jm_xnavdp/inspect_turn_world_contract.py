"""Read-only USD scene/material comparison. No simulation, no source writes."""
import json
from pathlib import Path
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
try:
    from pxr import Usd, UsdPhysics, PhysxSchema
    root=Path(__file__).resolve().parents[1]
    meta=json.loads((root/'artifacts/xnavdp_20260929/jm_capture_002/observation.json').read_text())
    stage=Usd.Stage.Open(meta['world'])
    paths=['/World/PhysicsScene','/World/Dadong/F2/slab','/World/H1Fleet/H1_1']
    records=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        selected=path in paths or prim.HasAPI(UsdPhysics.MaterialAPI) or prim.IsA(UsdPhysics.Scene)
        if selected:
            records.append(dict(path=path,type=prim.GetTypeName(),
                attributes={a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physx','xformOp:'))},
                relationships={r.GetName():[str(t) for t in r.GetTargets()] for r in prim.GetRelationships()}))
    robot=[]
    prim=stage.GetPrimAtPath('/World/H1Fleet/H1_1')
    for item in Usd.PrimRange(prim):
        if item.HasAPI(UsdPhysics.RigidBodyAPI) or item.IsA(UsdPhysics.RevoluteJoint):
            robot.append(dict(path=str(item.GetPath()),type=item.GetTypeName(),
                physics={a.GetName():str(a.Get()) for a in item.GetAttributes() if a.GetName().startswith(('physics:','physx','drive:'))}))
    out=root/'artifacts/xnavdp_20260930/turn_comparison_review'
    out.mkdir(exist_ok=True)
    (out/'jm_world_authored_contract.json').write_text(json.dumps(records,indent=2))
    (out/'jm_world_robot_inventory.json').write_text(json.dumps(robot,indent=2))
    print(json.dumps(records,indent=2))
finally:
    app.close()
