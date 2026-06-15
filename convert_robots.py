#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
convert_robots.py — Atlas / H1 URDF를 각각 독립 스테이지에서 임포트(이동 없음)해
평탄화된 self-contained USD로 내보낸다. 비주얼 메시 보존 검증용 리포트(json) 기록.

  C:\\isaacsim\\python.bat convert_robots.py

출력:
  C:\\tmp\\atlas-mujoco\\atlas_clean.usd
  C:\\tmp\\unitree_ros\\robots\\h1_description\\h1_clean.usd
  convert_robots_report.json   (각 로봇 Mesh prim 수 / bbox / 파일크기)
"""
import os, sys, json

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"

# URDF (원본) + 메시경로 패치본
ATLAS_SRC = r"C:\tmp\atlas-mujoco\model\atlas_minimal_contact.urdf"
ATLAS_PAT = r"C:\tmp\atlas-mujoco\model\atlas_isaac.urdf"
ATLAS_OUT = r"C:\tmp\atlas-mujoco\atlas_clean.usd"
H1_SRC    = r"C:\tmp\unitree_ros\robots\h1_description\urdf\h1.urdf"
H1_PAT    = r"C:\tmp\unitree_ros\robots\h1_description\urdf\h1_isaac.urdf"
H1_OUT    = r"C:\tmp\unitree_ros\robots\h1_description\h1_clean.usd"
REPORT    = os.path.join(PROJ, "convert_robots_report.json")

def patch(src, dst, frm, to):
    with open(src, "r", encoding="utf-8") as f:
        t = f.read()
    n = t.count(frm)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(t.replace(frm, to))
    return n

n1 = patch(ATLAS_SRC, ATLAS_PAT, "package://Atlas/urdf/meshes/", "C:/tmp/atlas-mujoco/model/meshes/")
n2 = patch(H1_SRC, H1_PAT, "package://h1_description/", "C:/tmp/unitree_ros/robots/h1_description/")

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import carb
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, Gf

enable_extension("isaacsim.asset.importer.urdf")
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()

def import_config():
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = True
    cfg.convex_decomp = False
    cfg.fix_base = True
    cfg.make_default_prim = True
    cfg.create_physics_scene = False
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.self_collision = False
    return cfg

report = {"patch_refs": {"atlas": n1, "h1": n2}, "robots": {}}

def convert(name, urdf, out):
    carb.log_warn("[convert] %s <- %s" % (name, urdf))
    ctx.new_stage()
    for _ in range(8):
        sim.update()
    cfg = import_config()
    status, robot_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf, import_config=cfg,
        get_articulation_root=True)
    for _ in range(15):
        sim.update()
    stage = ctx.get_stage()
    root_seg = "/" + robot_path.strip("/").split("/")[0]
    root = stage.GetPrimAtPath(root_seg)
    # 디폴트 prim 보장(레퍼런스 대상)
    if root and root.IsValid():
        stage.SetDefaultPrim(root)
    n_mesh = 0
    n_pts = 0
    for p in stage.Traverse():
        if p.GetTypeName() == "Mesh":
            n_mesh += 1
            pts = p.GetAttribute("points")
            if pts and pts.IsValid():
                v = pts.Get()
                n_pts += (len(v) if v else 0)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    size = [round(mx[i] - mn[i], 3) for i in range(3)]
    stage.Export(out)
    sz = os.path.getsize(out) if os.path.exists(out) else 0
    report["robots"][name] = {
        "status": bool(status), "robot_path": robot_path, "root": root_seg,
        "mesh_prims": n_mesh, "total_points": n_pts, "bbox_size": size,
        "out": out, "out_bytes": sz,
    }
    carb.log_warn("[convert] %s -> mesh=%d points=%d bbox=%s bytes=%d"
                  % (name, n_mesh, n_pts, size, sz))

convert("atlas", ATLAS_PAT, ATLAS_OUT)
convert("h1", H1_PAT, H1_OUT)

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[convert] report -> " + REPORT)

sim.close()
