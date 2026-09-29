#!/usr/bin/env python3
"""Make a visualization-only USD layer from saved candidate transforms and gripper mesh."""
import argparse
import json
from pxr import Usd, UsdGeom, Gf

p=argparse.ArgumentParser()
p.add_argument('--geometry',required=True)
p.add_argument('--output',required=True)
a=p.parse_args()
with open(a.geometry) as f: d=json.load(f)
s=Usd.Stage.CreateNew(a.output)
UsdGeom.SetStageMetersPerUnit(s,1.0)
UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
UsdGeom.Xform.Define(s,'/World/Task1GraspCandidates')
for i,t in enumerate(d['T_W_G']):
    m=UsdGeom.Mesh.Define(s,f'/World/Task1GraspCandidates/candidate_{i+1:02d}')
    m.CreatePointsAttr([Gf.Vec3f(*v) for v in d['vertices']])
    m.CreateFaceVertexCountsAttr([3]*len(d['faces']))
    m.CreateFaceVertexIndicesAttr([v for f in d['faces'] for v in f])
    m.CreateSubdivisionSchemeAttr('none')
    m.CreateDisplayColorAttr([Gf.Vec3f(0.05,0.5,1) if i==0 else Gf.Vec3f(1,0.6,0.05)])
    m.CreateDisplayOpacityAttr([1.0 if i==0 else 0.35])
    UsdGeom.Xformable(m).AddTransformOp().Set(Gf.Matrix4d(*[t[c][r] for r in range(4) for c in range(4)]))
    m.GetPrim().SetCustomDataByKey('model_score',float(d['scores'][i]))
    m.GetPrim().SetCustomDataByKey('role','Inference candidate only; no H1 attachment, IK or collision validation')
s.GetRootLayer().Save()
print(a.output)
