#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rerender_h1.py — 이미 H1 이 배치된 jm_factory_world_h1.usd 를 다시 열어
(1) 로봇 실제 bbox 진단, (2) 노출/조명 교정, (3) 로봇 타이트 재프레이밍 후 재렌더.
URDF 재임포트 없음(빠름).
"""
import os
import sys
import math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ    = r"C:\Users\USER\ISSAC_SIM_test"
IN_USD  = os.path.join(PROJ, "jm_factory_world_h1.usd")
OUT_DIR = os.path.join(PROJ, "render_h1b")
ROBOT   = "/h1_description"

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1600, "height": 1000})

import carb
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdLux, Gf

enable_extension("omni.replicator.core")

# RTX 자동노출(히스토그램) 끄기 — 밝은 바닥/하늘에서 화이트아웃 방지
s = carb.settings.get_settings()
s.set("/rtx/post/histogram/enabled", False)
s.set("/rtx/pathtracing/spp", 16)
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(IN_USD)
for _ in range(15):
    sim.update()
stage = ctx.get_stage()
print("[diag] opened:", IN_USD)

# --- 로봇 bbox 진단 ---
robot = stage.GetPrimAtPath(ROBOT)
print("[diag] robot prim valid:", bool(robot and robot.IsValid()))
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
rng = cache.ComputeWorldBound(robot).ComputeAlignedRange()
bmin, bmax = rng.GetMin(), rng.GetMax()
ctr = Gf.Vec3d((bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, (bmin[2] + bmax[2]) / 2)
size = Gf.Vec3d(bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2])
diag = math.sqrt(size[0]**2 + size[1]**2 + size[2]**2)
print("[diag] bbox min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)" %
      (bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2]))
print("[diag] center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) diag=%.2f" %
      (ctr[0], ctr[1], ctr[2], size[0], size[1], size[2], diag))

# --- 조명 교정 ---
fill = stage.GetPrimAtPath("/World/Lighting/H1Fill")
if fill and fill.IsValid():
    stage.RemovePrim(fill.GetPath())   # 과노출 주범 제거
    print("[fix] removed H1Fill")
sun = stage.GetPrimAtPath("/World/Lighting/Sun")
if sun and sun.IsValid():
    UsdLux.DistantLight(sun).GetIntensityAttr().Set(1600.0)
sky = stage.GetPrimAtPath("/World/Lighting/Sky")
if sky and sky.IsValid():
    UsdLux.DomeLight(sky).GetIntensityAttr().Set(300.0)
# 로봇 전용 소프트 필(적당 강도) — 정면(남서)에서
key = UsdLux.SphereLight.Define(stage, "/World/Lighting/H1Key")
key.CreateRadiusAttr(0.8)
key.CreateIntensityAttr(6000.0)
key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
UsdGeom.Xformable(key.GetPrim()).AddTranslateOp().Set(
    Gf.Vec3d(ctr[0] - 2.5, ctr[1] - 2.5, bmax[2] + 1.0))

# --- 카메라 타이트 재프레이밍(남서-상공, 살짝 하향) ---
def nrm(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return Gf.Vec3d(v[0] / n, v[1] / n, v[2] / n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

target = Gf.Vec3d(ctr[0], ctr[1], ctr[2])
view_dir = nrm((0.6, 0.62, -0.42))     # 카메라->타깃 (살짝 하향, 거의 눈높이)
DIST = max(4.2, diag * 1.9)
eye = Gf.Vec3d(target[0] - view_dir[0]*DIST,
               target[1] - view_dir[1]*DIST,
               target[2] - view_dir[2]*DIST)
up = Gf.Vec3d(0, 0, 1)
camZ = nrm((eye[0]-target[0], eye[1]-target[1], eye[2]-target[2]))
camX = nrm(crs(up, camZ)); camY = crs(camZ, camX)

cam = UsdGeom.Camera.Define(stage, "/World/RenderCam")
cam.GetFocalLengthAttr().Set(30.0) if cam.GetFocalLengthAttr() else cam.CreateFocalLengthAttr(30.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * 1000.0 / 1600.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
cam.CreateFocusDistanceAttr(DIST)
M = Gf.Matrix4d(1.0)
M.SetRow(0, Gf.Vec4d(camX[0], camX[1], camX[2], 0.0))
M.SetRow(1, Gf.Vec4d(camY[0], camY[1], camY[2], 0.0))
M.SetRow(2, Gf.Vec4d(camZ[0], camZ[1], camZ[2], 0.0))
M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
cxf = UsdGeom.Xformable(cam.GetPrim())
cxf.ClearXformOpOrder()
cxf.AddTransformOp().Set(M)
print("[fix] cam eye=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f) dist=%.1f" %
      (eye[0], eye[1], eye[2], target[0], target[1], target[2], DIST))

for _ in range(10):
    sim.update()

# --- 렌더 ---
os.makedirs(OUT_DIR, exist_ok=True)
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
rp = rep.create.render_product("/World/RenderCam", (1600, 1000))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([rp])
for _ in range(60):
    sim.update()
rep.orchestrator.step(rt_subframes=64)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
print("[done] rerender ->", OUT_DIR)
sim.close()
