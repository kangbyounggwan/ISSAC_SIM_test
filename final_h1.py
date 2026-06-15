#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
final_h1.py — jm_factory_world_h1.usd(H1 임포트됨)을 열어 조명/노출을 교정하고
히어로(클로즈업)+컨텍스트(와이드) 카메라 2대로 렌더한 뒤, 교정본 USD를 재저장.
배치/카메라 수치는 h1_placement.json 으로 기록(stdout 캡처 불가 대비).
"""
import os
import sys
import json
import math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ      = r"C:\Users\USER\ISSAC_SIM_test"
USD       = os.path.join(PROJ, "jm_factory_world_h1.usd")
HERO_DIR  = os.path.join(PROJ, "render_h1_hero")
WIDE_DIR  = os.path.join(PROJ, "render_h1_context")
INFO_JSON = os.path.join(PROJ, "h1_placement.json")
ROBOT     = "/h1_description"

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1600, "height": 1000})

import carb
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdLux, Gf

enable_extension("omni.replicator.core")
carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(USD)
for _ in range(15):
    sim.update()
stage = ctx.get_stage()

# ---- robot bbox ----
robot = stage.GetPrimAtPath(ROBOT)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
rng = cache.ComputeWorldBound(robot).ComputeAlignedRange()
bmin, bmax = rng.GetMin(), rng.GetMax()
ctr = Gf.Vec3d((bmin[0]+bmax[0])/2, (bmin[1]+bmax[1])/2, (bmin[2]+bmax[2])/2)
size = Gf.Vec3d(bmax[0]-bmin[0], bmax[1]-bmin[1], bmax[2]-bmin[2])
diag = math.sqrt(size[0]**2 + size[1]**2 + size[2]**2)

# ---- lighting fix ----
fp = stage.GetPrimAtPath("/World/Lighting/H1Fill")
if fp and fp.IsValid():
    stage.RemovePrim(fp.GetPath())
sun = stage.GetPrimAtPath("/World/Lighting/Sun")
if sun and sun.IsValid():
    UsdLux.DistantLight(sun).GetIntensityAttr().Set(1600.0)
sky = stage.GetPrimAtPath("/World/Lighting/Sky")
if sky and sky.IsValid():
    UsdLux.DomeLight(sky).GetIntensityAttr().Set(300.0)
key = UsdLux.SphereLight.Define(stage, "/World/Lighting/H1Key")
key.CreateRadiusAttr(0.8); key.CreateIntensityAttr(6000.0)
key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
UsdGeom.Xformable(key.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(ctr[0]-2.5, ctr[1]-2.5, bmax[2]+1.0))

# ---- camera helpers ----
def nrm(v):
    n = math.sqrt(sum(c*c for c in v)) or 1.0
    return Gf.Vec3d(v[0]/n, v[1]/n, v[2]/n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def make_cam(path, target, view_dir, dist, focal):
    vd = nrm(view_dir)
    eye = Gf.Vec3d(target[0]-vd[0]*dist, target[1]-vd[1]*dist, target[2]-vd[2]*dist)
    up = Gf.Vec3d(0, 0, 1)
    cz = nrm((eye[0]-target[0], eye[1]-target[1], eye[2]-target[2]))
    cx = nrm(crs(up, cz)); cy = crs(cz, cx)
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955*1000.0/1600.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 3000.0))
    cam.CreateFocusDistanceAttr(dist)
    M = Gf.Matrix4d(1.0)
    M.SetRow(0, Gf.Vec4d(cx[0], cx[1], cx[2], 0.0))
    M.SetRow(1, Gf.Vec4d(cy[0], cy[1], cy[2], 0.0))
    M.SetRow(2, Gf.Vec4d(cz[0], cz[1], cz[2], 0.0))
    M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
    xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(M)
    return eye

tgt = Gf.Vec3d(ctr[0], ctr[1], ctr[2])
hero_eye = make_cam("/World/RenderCam", tgt, (0.6, 0.62, -0.42), max(4.2, diag*1.9), 30.0)
wide_tgt = Gf.Vec3d(ctr[0], ctr[1], ctr[2]+0.5)
wide_eye = make_cam("/World/RenderCamWide", wide_tgt, (0.55, 0.62, -0.62), 15.0, 22.0)

# ---- info dump ----
info = dict(
    robot=ROBOT, floor="DA-2F",
    bbox_min=[round(v, 3) for v in bmin], bbox_max=[round(v, 3) for v in bmax],
    center=[round(v, 3) for v in ctr], size=[round(v, 3) for v in size],
    feet_z=round(bmin[2], 3), head_z=round(bmax[2], 3),
    hero_eye=[round(v, 2) for v in hero_eye], wide_eye=[round(v, 2) for v in wide_eye],
)
with open(INFO_JSON, "w", encoding="utf-8") as f:
    json.dump(info, f, ensure_ascii=False, indent=2)
print("[info]", json.dumps(info, ensure_ascii=False))

# ---- save corrected USD ----
stage.Export(USD)
print("[saved]", USD)

for _ in range(10):
    sim.update()

# ---- render both cameras ----
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
os.makedirs(HERO_DIR, exist_ok=True); os.makedirs(WIDE_DIR, exist_ok=True)
rp_h = rep.create.render_product("/World/RenderCam", (1600, 1000))
rp_w = rep.create.render_product("/World/RenderCamWide", (1600, 1000))
wh = rep.WriterRegistry.get("BasicWriter"); wh.initialize(output_dir=HERO_DIR, rgb=True); wh.attach([rp_h])
ww = rep.WriterRegistry.get("BasicWriter"); ww.initialize(output_dir=WIDE_DIR, rgb=True); ww.attach([rp_w])
for _ in range(60):
    sim.update()
rep.orchestrator.step(rt_subframes=64)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
print("[done] hero ->", HERO_DIR, " wide ->", WIDE_DIR)
sim.close()
