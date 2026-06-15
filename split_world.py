#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
split_world.py — decompose the FLATTENED monolithic world into a layered
(sublayer + reference) composition so each concern can be edited & GUI-reloaded
independently (no full 24MB reload). Pure USD / Sdf surgery, no Kit/SimulationApp.

  C:\\isaacsim\\python.bat split_world.py

Source of truth : jm_factory_world_atlas_h1.usd   (flattened crate, defaultPrim=/World)
Output (all in project root so relative asset paths keep resolving):
  jm_factory_world_atlas_h1.usd        <- rewritten THIN root (subLayers + defaultPrim)
  jm_factory_world_atlas_h1.flat.usd   <- byte backup of the original flattened file
  world_base.usd        structural shell (Ground/Fence/Dadong/Nadong/Gadong/Physics)
  world_equipment.usd   F2/{Equip,Conveyors,PCBStacks,Levelers,Developers,Models}
  world_labels.usda     F2/Labels
  world_robots.usda     /World/Robots/* = references to clean robot USDs + transforms
  world_staging.usda    /World/Lighting + cameras
"""
import os, sys, json, shutil
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401  (bootstraps pxr without Kit)
from pxr import Usd, UsdGeom, Sdf, Gf

PROJ      = r"C:\Users\USER\ISSAC_SIM_test"
ROOT_USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
BACKUP    = os.path.join(PROJ, "jm_factory_world_atlas_h1.flat.usd")
REPORT    = os.path.join(PROJ, "split_world_report.json")

L_BASE    = os.path.join(PROJ, "world_base.usd")
L_EQUIP   = os.path.join(PROJ, "world_equipment.usd")
L_LABELS  = os.path.join(PROJ, "world_labels.usda")
L_ROBOTS  = os.path.join(PROJ, "world_robots.usda")
L_STAGING = os.path.join(PROJ, "world_staging.usda")

ATLAS_REF = "C:/tmp/atlas-mujoco/atlas_clean.usd"
H1_REF    = "C:/tmp/unitree_ros/robots/h1_description/h1_clean.usd"

# robots are recreated as references (NOT copied geometry) + their captured transforms
ROBOTS = [
    {"name": "Atlas_1", "ref": ATLAS_REF},
    {"name": "H1_1",    "ref": H1_REF},
    {"name": "Atlas_2", "ref": ATLAS_REF},
    {"name": "H1_2",    "ref": H1_REF},
]

# subtree paths extracted to their own layers
EQUIP_PATHS = ["/World/Dadong/F2/Equip", "/World/Dadong/F2/Conveyors",
               "/World/Dadong/F2/PCBStacks", "/World/Dadong/F2/Levelers",
               "/World/Dadong/F2/Developers", "/World/Dadong/F2/Models"]
LABEL_PATHS = ["/World/Dadong/F2/Labels"]
STAGING_PATHS = ["/World/Lighting", "/World/RenderCamWide", "/World/RenderCam",
                 "/World/FloorCam", "/World/LevelerCam", "/World/DevCam"]
ROBOT_ROOT = "/World/Robots"

report = {"out": {}, "notes": []}


def remove_spec(layer, path):
    """Remove a prim spec by path from an Sdf layer (handles root + nested)."""
    p = Sdf.Path(path)
    spec = layer.GetPrimAtPath(p)
    if not spec:
        return False
    parent = p.GetParentPath()
    if parent == Sdf.Path.absoluteRootPath:
        del layer.rootPrims[p.name]
    else:
        del layer.GetPrimAtPath(parent).nameChildren[p.name]
    return True


def extract(dst_layer, paths):
    """Copy full subtrees from src into dst_layer; ancestors created as 'over'."""
    copied = []
    for path in paths:
        if not src.GetPrimAtPath(path):
            report["notes"].append("extract: source missing %s" % path)
            continue
        Sdf.CreatePrimInLayer(dst_layer, Sdf.Path(path).GetParentPath())
        ok = Sdf.CopySpec(src, Sdf.Path(path), dst_layer, Sdf.Path(path))
        copied.append((path, bool(ok)))
    return copied


# ---------------------------------------------------------------- open source
# Source of truth is the FLATTENED file. On first run that is ROOT_USD; we copy it
# to BACKUP and read from BACKUP so the split is idempotent and never mutates the
# only flattened copy. Re-runs always read BACKUP.
if not os.path.exists(BACKUP):
    if not os.path.exists(ROOT_USD):
        sys.exit("missing both %s and %s" % (ROOT_USD, BACKUP))
    shutil.copyfile(ROOT_USD, BACKUP)
    report["notes"].append("created backup from ROOT (first run)")
src = Sdf.Layer.FindOrOpen(BACKUP)
if src is None:
    sys.exit("could not open flattened source %s" % BACKUP)
report["src"] = BACKUP
report["src_default_prim"] = src.defaultPrim

# capture original transform op attrs for robots BEFORE we touch anything
robot_ops = {}
for r in ROBOTS:
    base = ROBOT_ROOT + "/" + r["name"]
    ops = {}
    for attr in ("xformOp:translate", "xformOp:rotateZ", "xformOpOrder"):
        sp = src.GetAttributeAtPath(base + "." + attr)
        if sp is not None:
            ops[attr] = sp.default
    robot_ops[r["name"]] = ops
report["robot_ops"] = {k: {a: str(v) for a, v in d.items()} for k, d in robot_ops.items()}

# ---------------------------------------------------------------- world_base
# full copy then strip out everything that moved to its own layer, plus all the
# Flattened_Prototype_* root prims (instanced robot mesh data, 24MB) — robots are
# rebuilt as references so those prototypes are regenerated on composition.
base = Sdf.Layer.CreateNew(L_BASE)
base.TransferContent(src)
to_strip = list(EQUIP_PATHS) + list(LABEL_PATHS) + list(STAGING_PATHS) + [ROBOT_ROOT]
stripped = [(p, remove_spec(base, p)) for p in to_strip]
proto_names = [p.name for p in base.rootPrims if p.name.startswith("Flattened_Prototype_")]
for nm in proto_names:
    del base.rootPrims[nm]
base.Save()
report["out"]["world_base"] = {"path": L_BASE, "stripped": stripped,
                               "prototypes_removed": len(proto_names),
                               "bytes": os.path.getsize(L_BASE)}

# ---------------------------------------------------------------- equipment
equip = Sdf.Layer.CreateNew(L_EQUIP)
report["out"]["world_equipment"] = {"path": L_EQUIP, "copied": extract(equip, EQUIP_PATHS)}
equip.Save()
report["out"]["world_equipment"]["bytes"] = os.path.getsize(L_EQUIP)

# ---------------------------------------------------------------- labels
labels = Sdf.Layer.CreateNew(L_LABELS)
report["out"]["world_labels"] = {"path": L_LABELS, "copied": extract(labels, LABEL_PATHS)}
labels.Save()
report["out"]["world_labels"]["bytes"] = os.path.getsize(L_LABELS)

# ---------------------------------------------------------------- staging (lights+cams)
staging = Sdf.Layer.CreateNew(L_STAGING)
report["out"]["world_staging"] = {"path": L_STAGING, "copied": extract(staging, STAGING_PATHS)}
staging.Save()
report["out"]["world_staging"]["bytes"] = os.path.getsize(L_STAGING)

# ---------------------------------------------------------------- robots (references!)
robots = Sdf.Layer.CreateNew(L_ROBOTS)
rscope = Sdf.CreatePrimInLayer(robots, Sdf.Path(ROBOT_ROOT))
rscope.specifier = Sdf.SpecifierDef
rscope.typeName = "Xform"
robot_recs = []
for r in ROBOTS:
    p = Sdf.Path(ROBOT_ROOT + "/" + r["name"])
    ps = Sdf.CreatePrimInLayer(robots, p)
    ps.specifier = Sdf.SpecifierDef
    ps.typeName = "Xform"
    ps.referenceList.prependedItems.append(Sdf.Reference(r["ref"]))
    # copy original xform op attributes verbatim (type + value preserved)
    for attr in ("xformOp:translate", "xformOp:rotateZ", "xformOpOrder"):
        srcpath = ROBOT_ROOT + "/" + r["name"] + "." + attr
        if src.GetAttributeAtPath(srcpath):
            Sdf.CopySpec(src, Sdf.Path(srcpath), robots, Sdf.Path(srcpath))
    robot_recs.append({"name": r["name"], "ref": r["ref"]})
robots.Save()
report["out"]["world_robots"] = {"path": L_ROBOTS, "robots": robot_recs,
                                 "bytes": os.path.getsize(L_ROBOTS)}

# ---------------------------------------------------------------- thin root
# Overwrite ROOT_USD (the old flattened file or a prior thin root) with a thin
# layer holding only subLayers (strong -> weak) + defaultPrim. The flattened
# original is preserved at BACKUP.
report["backup"] = BACKUP
default_prim = src.defaultPrim or "World"
root = Sdf.Layer.FindOrOpen(ROOT_USD)
if root is None:
    root = Sdf.Layer.CreateNew(ROOT_USD)
root.Clear()
root.subLayerPaths.clear()
for rel in ("./world_staging.usda", "./world_labels.usda", "./world_equipment.usd",
            "./world_robots.usda", "./world_base.usd"):
    root.subLayerPaths.append(rel)
root.defaultPrim = default_prim
root.Save()
report["out"]["root"] = {"path": ROOT_USD, "subLayers": list(root.subLayerPaths),
                         "defaultPrim": root.defaultPrim, "bytes": os.path.getsize(ROOT_USD)}

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)

print("=== split complete ===")
for k, v in report["out"].items():
    print("  %-16s %8d bytes  %s" % (k, v.get("bytes", 0), os.path.basename(v["path"])))
print("backup:", BACKUP, os.path.getsize(BACKUP), "bytes")
print("report:", REPORT)
