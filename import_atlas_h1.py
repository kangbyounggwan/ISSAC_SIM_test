#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
import_atlas_h1.py — Atlas(v5, URDF) 2대 + Unitree H1 2대를 jm_factory_world(base)에
임포트해 다동 2층(DA-2F)의 빈 통로 4곳에 배치하고, USD 저장 + 와이드 프리뷰 PNG 렌더.

실행:
  C:\\isaacsim\\python.bat import_atlas_h1.py

산출물:
  jm_factory_world_atlas_h1.usd   (원본 + Atlas x2 + H1 x2 + 카메라, 평탄화)
  render_atlas_h1\\rgb_0000.png    (4로봇 와이드 샷)
  atlas_h1_placement.json         (4로봇 배치/바운딩 기록)

주의(메모리): Windows 콘솔 cp949 — print() ASCII만 사용(한글은 cp949로 OK).
"""
import os
import sys
import json
import math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# ----------------------------------------------------------------- 경로 상수
PROJ      = r"C:\Users\USER\ISSAC_SIM_test"
WORLD_USD = os.path.join(PROJ, "jm_factory_world.usd")   # base 공장 월드

# H1 (Unitree)
H1_SRC    = r"C:\tmp\unitree_ros\robots\h1_description\urdf\h1.urdf"
H1_PKG    = r"C:/tmp/unitree_ros/robots/h1_description"
H1_PATCH  = r"C:\tmp\unitree_ros\robots\h1_description\urdf\h1_isaac.urdf"

# Atlas (v5, lvjonok/atlas-mujoco, drake 유래)
AT_SRC    = r"C:\tmp\atlas-mujoco\model\atlas_minimal_contact.urdf"
AT_PKGPFX = "package://Atlas/urdf/meshes/"
AT_ABSPFX = "C:/tmp/atlas-mujoco/model/meshes/"
AT_PATCH  = r"C:\tmp\atlas-mujoco\model\atlas_isaac.urdf"

OUT_USD   = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
OUT_DIR   = os.path.join(PROJ, "render_atlas_h1")
PLACE_JSON= os.path.join(PROJ, "atlas_h1_placement.json")

FLOOR_Z   = 6.075   # DA-2F 슬래브 top. 발 안착 높이.

# ----------------------------------------------------------------- 0) URDF 메시경로 패치
def patch(src, dst, frm, to):
    with open(src, "r", encoding="utf-8") as f:
        txt = f.read()
    n = txt.count(frm)
    txt = txt.replace(frm, to)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(txt)
    print("[patch] %s  (%d refs)  -> %s" % (os.path.basename(src), n, dst))

patch(H1_SRC, H1_PATCH, "package://h1_description/", H1_PKG + "/")
patch(AT_SRC, AT_PATCH, AT_PKGPFX, AT_ABSPFX)

# ----------------------------------------------------------------- 1) 빈 공간 4곳 탐색(장비 회피 + 상호 이격)
sys.path.insert(0, PROJ)
try:
    from dadong_3f_layout import world_boxes
    eq = []
    for b in world_boxes():
        cx, cy, _ = b["center"]; sx, sy, _ = b["size"]
        eq.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
    print("[step] equipment footprints:", len(eq))
except Exception as e:
    print("[warn] layout load failed, fallback empty:", e)
    eq = []

def clearance(px, py):
    best = 1e9
    for (x0, x1, y0, y1) in eq:
        dx = max(x0 - px, 0.0, px - x1)
        dy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(dx, dy))
    return best

CX, CY = 41.5, 15.0          # 다동 중심
cands = []
gx = 16.0
while gx <= 67.0:
    gy = 3.0
    while gy <= 27.0:
        cands.append((gx, gy, clearance(gx, gy)))
        gy += 0.5
    gx += 0.5

def pick_spots(n, min_clear, sep):
    viable = [t for t in cands if t[2] >= min_clear]
    viable.sort(key=lambda t: math.hypot(t[0] - CX, t[1] - CY))   # 중심 우선
    picked = []
    for (x, y, c) in viable:
        if all(math.hypot(x - px, y - py) >= sep for (px, py, _) in picked):
            picked.append((x, y, c))
            if len(picked) == n:
                break
    return picked

spots = []
for mc, sp in [(2.0, 2.6), (1.5, 2.4), (1.2, 2.0), (1.0, 1.6), (0.6, 1.2)]:
    spots = pick_spots(4, mc, sp)
    if len(spots) == 4:
        print("[step] 4 spots @ min_clear=%.1f sep=%.1f" % (mc, sp))
        break
if len(spots) < 4:
    # 최후: 가장 여유있는 4개(이격 무시)
    spots = sorted(cands, key=lambda t: -t[2])[:4]
    print("[warn] relaxed: top-clearance 4 spots")

# 로봇<->스팟 매핑 (Atlas 2 + H1 2). yaw 다양화로 동일 포즈 회피.
ROBOTS = [
    {"name": "Atlas_1", "urdf": AT_PATCH, "yaw": 235.0},
    {"name": "H1_1",    "urdf": H1_PATCH, "yaw": 205.0},
    {"name": "Atlas_2", "urdf": AT_PATCH, "yaw": 250.0},
    {"name": "H1_2",    "urdf": H1_PATCH, "yaw": 220.0},
]
for r, (x, y, c) in zip(ROBOTS, spots):
    r["x"], r["y"], r["clear"] = x, y, c
    print("[plan] %-7s x=%.2f y=%.2f clear=%.2fm yaw=%.0f" % (r["name"], x, y, c, r["yaw"]))

# ----------------------------------------------------------------- 2) Isaac Sim 시작(헤드리스)
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1920, "height": 1080})

import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdLux, Gf

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

# 로봇 부모 스코프
UsdGeom.Xform.Define(stage, "/World/Robots")

def import_config():
    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = True
    cfg.convex_decomp = False
    cfg.fix_base = True               # 정적(낙하 방지)
    cfg.make_default_prim = False
    cfg.create_physics_scene = False  # 공장에 PhysicsScene 존재
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.self_collision = False
    return cfg

def move_prim(src, dst):
    last = None
    try:
        omni.kit.commands.execute("MovePrim", path_from=src, path_to=dst)
        return True
    except Exception as e:
        last = e
    try:
        omni.kit.commands.execute("MovePrims", paths_to_move={src: dst})
        return True
    except Exception as e:
        last = e
    print("[warn] move failed %s -> %s : %s" % (src, dst, last))
    return False

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])

results = []
for r in ROBOTS:
    cfg = import_config()
    status, robot_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=r["urdf"], import_config=cfg,
        get_articulation_root=True)
    for _ in range(8):
        sim.update()
    root_seg = "/" + robot_path.strip("/").split("/")[0]
    dst = "/World/Robots/" + r["name"]
    move_prim(root_seg, dst)
    for _ in range(5):
        sim.update()
    prim = stage.GetPrimAtPath(dst)
    if not prim or not prim.IsValid():
        prim = stage.GetPrimAtPath(robot_path)   # move 실패시 원위치 사용
        dst = str(prim.GetPath())
    print("[step] imported %-7s status=%s root=%s -> %s" % (r["name"], status, root_seg, dst))

    # 배치(수평+yaw) 후 발 안착
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp()
    rz_op = xf.AddRotateZOp()
    t_op.Set(Gf.Vec3d(r["x"], r["y"], 0.0))
    rz_op.Set(r["yaw"])
    for _ in range(15):
        sim.update()
    bbox_cache.Clear()
    rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    min_z, top_z = rng.GetMin()[2], rng.GetMax()[2]
    dz = FLOOR_Z - min_z
    t_op.Set(Gf.Vec3d(r["x"], r["y"], dz))
    for _ in range(8):
        sim.update()
    head_z = top_z + dz
    print("[seat] %-7s feet_z=%.2f head_z=%.2f height=%.2fm" %
          (r["name"], FLOOR_Z, head_z, head_z - FLOOR_Z))
    results.append({"name": r["name"], "path": dst, "x": r["x"], "y": r["y"],
                    "yaw": r["yaw"], "feet_z": FLOOR_Z, "head_z": round(head_z, 3),
                    "height": round(head_z - FLOOR_Z, 3), "clear": round(r["clear"], 3)})

# ----------------------------------------------------------------- 4) 지붕 숨김 + 그룹 필라이트
roof = stage.GetPrimAtPath("/World/Dadong/Roof")
if roof and roof.IsValid():
    UsdGeom.Imageable(roof).MakeInvisible()
    print("[step] 다동 지붕 숨김(돌하우스)")

gx = sum(r["x"] for r in ROBOTS) / 4.0
gy = sum(r["y"] for r in ROBOTS) / 4.0
maxh = max(rr["head_z"] for rr in results)
fill = UsdLux.SphereLight.Define(stage, "/World/Lighting/GroupFill")
fill.CreateRadiusAttr(0.8)
fill.CreateIntensityAttr(150000.0)
fill.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.9))
UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(
    Gf.Vec3d(gx - 2.0, gy - 2.0, max(maxh + 1.5, 9.5)))

# ----------------------------------------------------------------- 5) 와이드 카메라(4로봇 프레이밍)
def normalize(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return Gf.Vec3d(v[0] / n, v[1] / n, v[2] / n)
def cross(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

spread = 0.0
for i in range(4):
    for j in range(i + 1, 4):
        spread = max(spread, math.hypot(ROBOTS[i]["x"] - ROBOTS[j]["x"],
                                        ROBOTS[i]["y"] - ROBOTS[j]["y"]))
target = Gf.Vec3d(gx, gy, FLOOR_Z + 1.0)
view_dir = normalize((1.0, 1.2, -1.45))
DIST = max(15.0, spread * 2.3 + 9.0)
eye = Gf.Vec3d(target[0] - view_dir[0]*DIST, target[1] - view_dir[1]*DIST,
               target[2] - view_dir[2]*DIST)
up = Gf.Vec3d(0, 0, 1)
camZ = normalize((eye[0]-target[0], eye[1]-target[1], eye[2]-target[2]))
camX = normalize(cross(up, camZ))
camY = cross(camZ, camX)
cam = UsdGeom.Camera.Define(stage, "/World/RenderCam")
cam.CreateFocalLengthAttr(32.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * 1080.0 / 1920.0)
cam.CreateFocusDistanceAttr(DIST)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 3000.0))
M = Gf.Matrix4d(1.0)
M.SetRow(0, Gf.Vec4d(camX[0], camX[1], camX[2], 0.0))
M.SetRow(1, Gf.Vec4d(camY[0], camY[1], camY[2], 0.0))
M.SetRow(2, Gf.Vec4d(camZ[0], camZ[1], camZ[2], 0.0))
M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
cxf = UsdGeom.Xformable(cam.GetPrim())
cxf.ClearXformOpOrder()
cxf.AddTransformOp().Set(M)
print("[step] camera eye=(%.1f,%.1f,%.1f) dist=%.1f spread=%.1f" %
      (eye[0], eye[1], eye[2], DIST, spread))

# ----------------------------------------------------------------- 6) 저장 + 배치 json
stage.Export(OUT_USD)
print("[step] saved ->", OUT_USD)
with open(PLACE_JSON, "w", encoding="utf-8") as f:
    json.dump({"floor": "DA-2F", "floor_z": FLOOR_Z, "robots": results,
               "camera_eye": [round(eye[0],2), round(eye[1],2), round(eye[2],2)]},
              f, ensure_ascii=False, indent=2)
print("[step] placement json ->", PLACE_JSON)

# ----------------------------------------------------------------- 7) 렌더
os.makedirs(OUT_DIR, exist_ok=True)
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
rp = rep.create.render_product("/World/RenderCam", (1920, 1080))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([rp])
for _ in range(80):
    sim.update()
rep.orchestrator.step(rt_subframes=64)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
print("[step] render ->", OUT_DIR)

sim.close()
print("[done] 2 Atlas + 2 H1 placed on DA-2F.")
