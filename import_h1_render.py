#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
import_h1_render.py — Unitree H1(URDF)을 jm_factory_world 에 임포트해 다동 2층(DA-2F)에
배치하고 Isaac Sim(헤드리스/RTX)으로 PNG 렌더 + 로봇 포함 USD 저장.

실행:
  C:\\isaacsim\\python.bat import_h1_render.py

산출물:
  jm_factory_world_h1.usd   (원본 + H1 + 카메라, 평탄화)
  render_h1\\rgb_XXXX.png    (뷰포트 렌더)
"""
import os
import sys
import io
import math

# Windows 콘솔(cp949)에서 비-ASCII 출력시 크래시 방지 — UTF-8 강제(가능한 경우)
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# ----------------------------------------------------------------- 경로 상수
PROJ      = r"C:\Users\USER\ISSAC_SIM_test"
WORLD_USD = os.path.join(PROJ, "jm_factory_world.usd")
URDF_SRC  = r"C:\tmp\unitree_ros\robots\h1_description\urdf\h1.urdf"
URDF_DIR  = os.path.dirname(URDF_SRC)
PKG_ROOT  = r"C:/tmp/unitree_ros/robots/h1_description"   # package://h1_description -> 이 폴더
URDF_PATCHED = os.path.join(URDF_DIR, "h1_isaac.urdf")
OUT_USD   = os.path.join(PROJ, "jm_factory_world_h1.usd")
OUT_DIR   = os.path.join(PROJ, "render_h1")

FLOOR_Z   = 6.075   # DA-2F 슬래브 top(z=6.0 중심, t=0.15 -> top 6.075). 발 안착 높이.

# ----------------------------------------------------------------- 0) URDF 메시경로 패치
# package://h1_description/ -> 절대경로. 임포터의 package:// 해석 모호성 제거.
with open(URDF_SRC, "r", encoding="utf-8") as f:
    urdf_txt = f.read()
urdf_txt = urdf_txt.replace("package://h1_description/", PKG_ROOT + "/")
with open(URDF_PATCHED, "w", encoding="utf-8") as f:
    f.write(urdf_txt)
print("[step] patched URDF ->", URDF_PATCHED)

# ----------------------------------------------------------------- 1) 빈 공간 탐색(장비 회피)
# dadong_3f_layout.world_boxes() 의 장비 풋프린트를 피해 가장 여유있는 (x,y) 선택.
sys.path.insert(0, PROJ)
try:
    from dadong_3f_layout import world_boxes
    eq = []
    for b in world_boxes():
        cx, cy, _ = b["center"]; sx, sy, _ = b["size"]
        eq.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
except Exception as e:
    print("[warn] 장비 데이터 로드 실패, 기본 위치 사용:", e)
    eq = []

def clearance(px, py):
    best = 1e9
    for (x0, x1, y0, y1) in eq:
        dx = max(x0 - px, 0.0, px - x1)
        dy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(dx, dy))
    return best

# 장비에서 최소 MIN_CLEAR 이상 떨어진(=빈 통로) 후보 중 건물 중심에 가장 가까운 점 선택
MIN_CLEAR = 2.0
CX, CY = 41.5, 15.0   # 다동 중심
cands = []
gx = 16.0
while gx <= 67.0:
    gy = 3.0
    while gy <= 27.0:
        cands.append((gx, gy, clearance(gx, gy)))
        gy += 0.5
    gx += 0.5
viable = [t for t in cands if t[2] >= MIN_CLEAR]
if viable:
    RX, RY, RC = min(viable, key=lambda t: math.hypot(t[0] - CX, t[1] - CY))
else:
    RX, RY, RC = max(cands, key=lambda t: t[2])
print("[step] H1 placement (DA-2F): x=%.2f y=%.2f  clearance=%.2fm" % (RX, RY, RC))

# ----------------------------------------------------------------- 2) Isaac Sim 시작(헤드리스)
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1600, "height": 1000})

import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("omni.replicator.core")
for _ in range(5):
    sim.update()

# ----------------------------------------------------------------- 3) 공장 월드 열기
ctx = omni.usd.get_context()
ctx.open_stage(WORLD_USD)
for _ in range(10):
    sim.update()
stage = ctx.get_stage()
print("[step] opened world. default prim =", stage.GetDefaultPrim().GetPath())

# ----------------------------------------------------------------- 4) H1 URDF 임포트(인라인)
status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True
cfg.convex_decomp = False
cfg.fix_base = True               # 정적 렌더(낙하 방지)
cfg.make_default_prim = False     # /World 가 default 유지
cfg.create_physics_scene = False  # 공장에 이미 PhysicsScene 존재
cfg.import_inertia_tensor = True
cfg.distance_scale = 1.0
cfg.self_collision = False

status, robot_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATCHED,
    import_config=cfg,
    get_articulation_root=True,
)
print("[step] URDF import status =", status, " prim =", robot_path)
for _ in range(10):
    sim.update()

# 최상위(루트) prim 찾기
root_seg = "/" + robot_path.strip("/").split("/")[0]
robot_prim = stage.GetPrimAtPath(root_seg)
if not robot_prim or not robot_prim.IsValid():
    robot_prim = stage.GetPrimAtPath(robot_path)
print("[step] robot root prim =", robot_prim.GetPath())

# ----------------------------------------------------------------- 5) 배치(수평 + yaw) 후 발 안착
# 카메라(남서-상공)를 바라보도록 정면을 SW로 향함
yaw_deg = 230.0
xf = UsdGeom.Xformable(robot_prim)
xf.ClearXformOpOrder()
t_op = xf.AddTranslateOp()
r_op = xf.AddRotateZOp()
t_op.Set(Gf.Vec3d(RX, RY, 0.0))
r_op.Set(yaw_deg)
for _ in range(20):
    sim.update()

# 월드 바운딩박스로 발바닥 z 계산 -> 슬래브 top 에 안착
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
rng = cache.ComputeWorldBound(robot_prim).ComputeAlignedRange()
min_z = rng.GetMin()[2]
top_z = rng.GetMax()[2]
dz = FLOOR_Z - min_z
t_op.Set(Gf.Vec3d(RX, RY, dz))
robot_top = top_z + dz
print("[step] seat: dz=%.3f  feet_z=%.2f  head_z=%.2f" % (dz, FLOOR_Z, robot_top))
for _ in range(10):
    sim.update()

# ----------------------------------------------------------------- 6) 지붕 숨김(돌하우스) + 필 라이트
roof = stage.GetPrimAtPath("/World/Dadong/Roof")
if roof and roof.IsValid():
    UsdGeom.Imageable(roof).MakeInvisible()
    print("[step] 다동 지붕 숨김")

fill = UsdLux.SphereLight.Define(stage, "/World/Lighting/H1Fill")
fill.CreateRadiusAttr(0.6)
fill.CreateIntensityAttr(120000.0)
fill.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.9))
UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(RX - 2.0, RY - 2.0, max(robot_top + 1.5, 9.5)))

# ----------------------------------------------------------------- 7) 카메라(남서-상공 -> 북동-하향)
def normalize(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return Gf.Vec3d(v[0] / n, v[1] / n, v[2] / n)

def cross(a, b):
    return Gf.Vec3d(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

target = Gf.Vec3d(RX, RY, (FLOOR_Z + robot_top) / 2.0)
view_dir = normalize((1.0, 1.2, -1.6))      # 시선(카메라->타깃)
DIST = 16.0
eye = Gf.Vec3d(target[0] - view_dir[0] * DIST,
               target[1] - view_dir[1] * DIST,
               target[2] - view_dir[2] * DIST)
up = Gf.Vec3d(0, 0, 1)
camZ = normalize((eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]))
camX = normalize(cross(up, camZ))
camY = cross(camZ, camX)

cam = UsdGeom.Camera.Define(stage, "/World/RenderCam")
cam.CreateFocalLengthAttr(35.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * 1000.0 / 1600.0)
cam.CreateFocusDistanceAttr(DIST)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000.0))
M = Gf.Matrix4d(1.0)
M.SetRow(0, Gf.Vec4d(camX[0], camX[1], camX[2], 0.0))
M.SetRow(1, Gf.Vec4d(camY[0], camY[1], camY[2], 0.0))
M.SetRow(2, Gf.Vec4d(camZ[0], camZ[1], camZ[2], 0.0))
M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
cxf = UsdGeom.Xformable(cam.GetPrim())
cxf.ClearXformOpOrder()
cxf.AddTransformOp().Set(M)
print(f"[step] camera eye=({eye[0]:.1f},{eye[1]:.1f},{eye[2]:.1f}) target=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f})")

# ----------------------------------------------------------------- 8) USD 저장(원본 보존, 평탄화)
stage.Export(OUT_USD)
print("[step] saved ->", OUT_USD)

# ----------------------------------------------------------------- 9) 렌더(Replicator BasicWriter)
os.makedirs(OUT_DIR, exist_ok=True)
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
rp = rep.create.render_product("/World/RenderCam", (1600, 1000))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([rp])

for _ in range(60):     # 씬/텍스처 로드 + RT 워밍업
    sim.update()
rep.orchestrator.step(rt_subframes=64)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
print("[step] render written to", OUT_DIR)

sim.close()
print("[done] H1 imported + rendered.")
