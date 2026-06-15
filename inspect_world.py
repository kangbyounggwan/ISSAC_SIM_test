#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspect_world.py — dump the prim tree / composition of the flattened world
so we can plan a layered (sublayer + reference) split. Pure-USD, no SimulationApp.
  C:\\isaacsim\\python.bat inspect_world.py
"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401  (bootstraps pxr without Kit)
from pxr import Usd, UsdGeom, Sdf, Gf

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
OUT  = os.path.join(PROJ, "inspect_world_report.json")

stage = Usd.Stage.Open(USD)
rep = {"usd": USD, "default_prim": str(stage.GetDefaultPrim().GetPath())}

# used layers (any external composition still present?)
rep["used_layers"] = [l.identifier for l in stage.GetUsedLayers()]
root = stage.GetRootLayer()
rep["root_sublayers"] = list(root.subLayerPaths)

# top-level + 2nd/3rd level scope inventory with child counts + type
def summarize(prim, depth, maxdepth):
    path = str(prim.GetPath())
    children = prim.GetChildren()
    node = {"path": path, "type": prim.GetTypeName(),
            "n_children": len(children),
            "n_descendants": sum(1 for _ in Usd.PrimRange(prim)) - 1}
    # composition arcs on this prim
    refs = prim.GetReferences() and prim.GetMetadata("references")
    arcs = []
    pspec = root.GetPrimAtPath(prim.GetPath())
    if pspec is not None:
        if pspec.referenceList.GetAddedOrExplicitItems():
            arcs.append("references")
        if pspec.payloadList.GetAddedOrExplicitItems():
            arcs.append("payload")
    if arcs:
        node["arcs"] = arcs
    if depth < maxdepth:
        node["children"] = [summarize(c, depth + 1, maxdepth) for c in children]
    return node

rep["tree"] = summarize(stage.GetPseudoRoot(), 0, 4)

# robot xforms (so we can recreate as reference + transform)
robots = {}
rp = stage.GetPrimAtPath("/World/Robots")
if rp and rp.IsValid():
    for c in rp.GetChildren():
        xf = UsdGeom.Xformable(c)
        ops = []
        for op in xf.GetOrderedXformOps():
            ops.append({"name": op.GetOpName(), "value": str(op.Get())})
        robots[c.GetName()] = {"path": str(c.GetPath()),
                               "n_mesh": sum(1 for q in Usd.PrimRange(c) if q.GetTypeName() == "Mesh"),
                               "xformOps": ops}
rep["robots"] = robots

# F2 scopes (the equipment/labels concern)
f2 = stage.GetPrimAtPath("/World/Dadong/F2")
rep["F2_children"] = []
if f2 and f2.IsValid():
    for c in f2.GetChildren():
        rep["F2_children"].append({"name": c.GetName(), "type": c.GetTypeName(),
                                   "n_children": len(c.GetChildren()),
                                   "n_descendants": sum(1 for _ in Usd.PrimRange(c)) - 1})

with open(OUT, "w", encoding="utf-8") as f:
    json.dump(rep, f, ensure_ascii=False, indent=2)
print("wrote", OUT)
print("default_prim:", rep["default_prim"])
print("used_layers:", len(rep["used_layers"]))
for l in rep["used_layers"]:
    print("   ", l)
