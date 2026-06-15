#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
add_conveyors.py — 다동 2F 모든 라인의 투입/수취(io) 장비를 컨베이어로 교체하고,
각 컨베이어 옆에 PCB 적재 스테이션(카트 + 적층 PCB 패널 + 랙 포스트)을 생성.
로봇 4대가 있는 현재 월드(jm_factory_world_atlas_h1.usd)를 직접 편집해 덮어쓴다.

  C:\\isaacsim\\python.bat add_conveyors.py

검증: 오버헤드(전층) + 로봇통로 2뷰 렌더 + add_conveyors_report.json
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
FLOOR_DIR = os.path.join(PROJ, "render_conveyor_top")
AISLE_DIR = os.path.join(PROJ, "render_conveyor_aisle")
REPORT = os.path.join(PROJ, "add_conveyors_report.json")

FLOOR = 6.075   # DA-2F 슬래브 top

# 색상
BELT = (0.13, 0.13, 0.16); RAIL = (0.58, 0.60, 0.63); LEG = (0.34, 0.34, 0.38)
CART = (0.40, 0.40, 0.44); POST = (0.55, 0.57, 0.60); PCB = (0.11, 0.44, 0.20)

# io 스테이션 축(라인 흐름 방향) + 역할
AXIS = {"wet_recv": "x", "psr_in": "x", "psr_recv": "x", "df_in": "x", "df_recv": "x",
        "psr_dev_in": "y", "tunnel_rv": "y"}
ROLE = {"psr_in": "in", "df_in": "in", "psr_dev_in": "in",
        "wet_recv": "out", "psr_recv": "out", "df_recv": "out", "tunnel_rv": "out"}

sys.path.insert(0, PROJ)
from dadong_3f_layout import EQUIPMENT, grid_to_world
io_stations = []
for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
    if zone == "io":
        x, y = grid_to_world(cc, cr)
        io_stations.append(dict(id=eid, kr=kr, x=x, y=y, axis=AXIS.get(eid, "x"),
                                role=ROLE.get(eid, "in")))

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1920, "height": 1080})
import carb, carb.settings
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

enable_extension("omni.replicator.core")
carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(USD)
for _ in range(10):
    sim.update()
stage = ctx.get_stage()

sys.path.insert(0, PROJ); import world_layers as wl
wl.set_edit_layer(stage, "equipment")  # Conveyors/PCBStacks -> world_equipment.usd

def box(path, center, size, color, collider=False, cat=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if cat:
        c.GetPrim().CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set(cat)
        c.GetPrim().CreateAttribute("jm:floor", Sdf.ValueTypeNames.String).Set("DA-2F")

def conveyor(prefix, cx, cy, axis):
    L, W = 2.6, 0.8
    deck_top = FLOOR + 0.80; deck_t = 0.10
    if axis == "x":
        dsz = (L, W, deck_t)
        box(prefix + "/railA", (cx, cy - W/2 + 0.04, deck_top + 0.06), (L, 0.06, 0.12), RAIL, cat="conveyor")
        box(prefix + "/railB", (cx, cy + W/2 - 0.04, deck_top + 0.06), (L, 0.06, 0.12), RAIL, cat="conveyor")
        legxy = [(cx - L/2 + 0.15, cy - W/2 + 0.1), (cx + L/2 - 0.15, cy - W/2 + 0.1),
                 (cx - L/2 + 0.15, cy + W/2 - 0.1), (cx + L/2 - 0.15, cy + W/2 - 0.1)]
    else:
        dsz = (W, L, deck_t)
        box(prefix + "/railA", (cx - W/2 + 0.04, cy, deck_top + 0.06), (0.06, L, 0.12), RAIL, cat="conveyor")
        box(prefix + "/railB", (cx + W/2 - 0.04, cy, deck_top + 0.06), (0.06, L, 0.12), RAIL, cat="conveyor")
        legxy = [(cx - W/2 + 0.1, cy - L/2 + 0.15), (cx + W/2 - 0.1, cy - L/2 + 0.15),
                 (cx - W/2 + 0.1, cy + L/2 - 0.15), (cx + W/2 - 0.1, cy + L/2 - 0.15)]
    box(prefix + "/belt", (cx, cy, deck_top - deck_t/2), dsz, BELT, collider=True, cat="conveyor")
    legh = (deck_top - deck_t) - FLOOR
    for i, (lx, ly) in enumerate(legxy):
        box(prefix + "/leg%d" % i, (lx, ly, FLOOR + legh/2), (0.08, 0.08, legh), LEG, collider=True, cat="conveyor")

def pcb_stack(prefix, cx, cy):
    box(prefix + "/cart", (cx, cy, FLOOR + 0.06), (0.75, 0.60, 0.12), CART, collider=True, cat="pcb_stack")
    top = FLOOR + 0.12
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
        box(prefix + "/post%d" % i, (cx + sx*0.34, cy + sy*0.26, top + 0.28), (0.04, 0.04, 0.56), POST, cat="pcb_stack")
    for i in range(8):
        z = top + 0.05 + i*0.05
        box(prefix + "/pcb%d" % i, (cx, cy, z), (0.60, 0.46, 0.02), PCB, cat="pcb_stack")

report = {"stations": [], "deleted": []}
for s in io_stations:
    eid, cx, cy, axis = s["id"], s["x"], s["y"], s["axis"]
    # 1) 기존 io 박스 제거
    epath = "/World/Dadong/F2/Equip/" + eid
    if stage.GetPrimAtPath(epath):
        stage.RemovePrim(Sdf.Path(epath)); report["deleted"].append(epath)
    # 2) 컨베이어
    conveyor("/World/Dadong/F2/Conveyors/" + eid, cx, cy, axis)
    # 3) PCB 적재 스테이션(건물 중심 방향 cross-offset)
    if axis == "x":
        sy = cy + (-1.5 if cy > 15.0 else 1.5)
        sx = cx
    else:
        sx = cx + (-1.5 if cx > 41.5 else 1.5)
        sy = cy
    pcb_stack("/World/Dadong/F2/PCBStacks/" + eid, sx, sy)
    report["stations"].append(dict(id=eid, kr=s["kr"], role=s["role"], axis=axis,
                                   conv=[round(cx, 2), round(cy, 2)], pcb=[round(sx, 2), round(sy, 2)]))
    carb.log_warn("[conv] %-11s axis=%s conv=(%.1f,%.1f) pcb=(%.1f,%.1f)" % (eid, axis, cx, cy, sx, sy))

for _ in range(10):
    sim.update()

# ----------------------------------------------------------------- 카메라(오버헤드 + 로봇통로)
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
    cam.CreateFocalLengthAttr(focal); cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955*1080.0/1920.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 4000.0)); cam.CreateFocusDistanceAttr(dist)
    M = Gf.Matrix4d(1.0)
    M.SetRow(0, Gf.Vec4d(cx[0], cx[1], cx[2], 0.0)); M.SetRow(1, Gf.Vec4d(cy[0], cy[1], cy[2], 0.0))
    M.SetRow(2, Gf.Vec4d(cz[0], cz[1], cz[2], 0.0)); M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
    xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(M)
    return [round(eye[0], 1), round(eye[1], 1), round(eye[2], 1)]

# 다동 2F 전층 오버헤드(남상공 → 북하향), roof 는 이미 숨김
wl.set_edit_layer(stage, "staging")  # persistent cameras -> world_staging.usda
report["floor_cam_eye"] = make_cam("/World/FloorCam", (41.5, 15.0, 6.6), (0.0, 0.55, -1.0), 46.0, 17.0)
report["aisle_cam_eye"] = make_cam("/World/RenderCam", (41.5, 14.5, 6.9), (1.0, 1.15, -1.25), 18.0, 30.0)

wl.save(stage)  # save world_equipment.usd + world_staging.usda in place; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("equipment"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[conv] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

# ----------------------------------------------------------------- 렌더 2뷰
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
os.makedirs(FLOOR_DIR, exist_ok=True); os.makedirs(AISLE_DIR, exist_ok=True)
rp_f = rep.create.render_product("/World/FloorCam", (1920, 1080))
rp_a = rep.create.render_product("/World/RenderCam", (1920, 1080))
wf = rep.WriterRegistry.get("BasicWriter"); wf.initialize(output_dir=FLOOR_DIR, rgb=True); wf.attach([rp_f])
wa = rep.WriterRegistry.get("BasicWriter"); wa.initialize(output_dir=AISLE_DIR, rgb=True); wa.attach([rp_a])
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = True

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[conv] report -> " + REPORT)
sim.close()
