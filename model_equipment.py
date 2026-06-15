#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
model_equipment.py — 다동 2F의 각 설비를 개별 모델로 교체.
공통 레시피: 외부 인클로저(베이스+후면벽+전면 로우월+측벽, 상단 개방=내부 가시) + 설비 종류별 내부.
이미 상세화된 정면기2/현상기1, 컨베이어(io), 룸 패드는 건너뜀(유지).

  C:\\isaacsim\\python.bat model_equipment.py
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
REPORT = os.path.join(PROJ, "model_equipment_report.json")
FLOOR = 6.075

# 내부 부품 색
ROLLER=(0.62,0.64,0.67); PCB=(0.11,0.44,0.20); SPRAY=(0.52,0.58,0.66); HEAT=(0.90,0.40,0.16)
GLASS=(0.55,0.70,0.86); GRANITE=(0.45,0.46,0.50); TANK=(0.50,0.60,0.68); METAL=(0.60,0.62,0.66)
DARK=(0.30,0.31,0.34); LOUVER=(0.52,0.54,0.58); SHELF=(0.70,0.66,0.55); HMI=(0.10,0.16,0.34)
FILM=(0.85,0.82,0.55); BED=(0.40,0.42,0.46)

sys.path.insert(0, PROJ)
from dadong_3f_layout import EQUIPMENT, grid_to_world, ZONE_H, ZONE_COLOR

SKIP_ID = {"psr_lvl","df_lvl","psr_dev","psr_in","df_in","psr_dev_in",
           "wet_recv","psr_recv","df_recv","tunnel_rv"}
SKIP_ZONE = {"room", "io"}

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
wl.set_edit_layer(stage, "equipment")  # author Models into world_equipment.usd (no re-flatten)

def box(path, center, size, color, collider=False):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0); c.CreateExtentAttr([(-0.5,-0.5,-0.5),(0.5,0.5,0.5)])
    c.AddTranslateOp().Set(Gf.Vec3d(*center)); c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

def cyl(path, center, radius, height, axis, color):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius); c.CreateHeightAttr(height); c.CreateAxisAttr(axis)
    r, h = radius, height/2.0
    ext = {"Z":[(-r,-r,-h),(r,r,h)], "Y":[(-r,-h,-r),(r,h,r)], "X":[(-h,-r,-r),(h,r,r)]}[axis]
    c.CreateExtentAttr(ext); c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])

# 외부 인클로저(상단 개방 컷어웨이). 반환: (bz0, bh, ax, racross)
def enclosure(pfx, cx, cy, w, d, z0, h, wall):
    base_h = min(0.22, h*0.2)
    box(pfx+"/base", (cx, cy, z0+base_h/2), (w, d, base_h), DARK, True)
    bz0 = z0 + base_h; bh = h - base_h
    t = 0.06
    box(pfx+"/wall_back",  (cx, cy+d/2-t/2, bz0+bh/2), (w, t, bh), wall, True)
    box(pfx+"/wall_wA",    (cx-w/2+t/2, cy, bz0+bh/2), (t, d, bh), wall, True)
    box(pfx+"/wall_wB",    (cx+w/2-t/2, cy, bz0+bh/2), (t, d, bh), wall, True)
    lip = min(0.35, bh*0.45)
    box(pfx+"/lip_front",  (cx, cy-d/2+t/2, bz0+lip/2), (w, t, lip), wall)
    ax = "x" if w >= d else "y"
    racross = "Y" if ax == "x" else "X"
    return bz0, bh, ax, racross

def conveyor_inside(pfx, cx, cy, w, d, z0, h, ax, racross, kind):
    span = (w if ax == "x" else d) - 0.4
    cross = (d if ax == "x" else w)
    ct = z0 + min(0.65, h*0.45)
    n = max(4, int(span/0.5))
    for i in range(n):
        t = -span/2 + (i+0.5)*span/n
        c0 = (cx+t, cy) if ax == "x" else (cx, cy+t)
        cyl(pfx+"/roll%d" % i, (c0[0], c0[1], ct), 0.045, cross-0.22, racross, ROLLER)
    for i in range(max(2, int(span/1.3))):
        t = -span/2 + 0.5 + i*1.2
        c0 = (cx+t, cy) if ax == "x" else (cx, cy+t)
        sz = (0.42, cross-0.5, 0.02) if ax == "x" else (cross-0.5, 0.42, 0.02)
        box(pfx+"/pcb%d" % i, (c0[0], c0[1], ct+0.05), sz, PCB)
    nb = max(2, int(span/1.1))
    for i in range(nb):
        t = -span/2 + (i+0.5)*span/nb
        c0 = (cx+t, cy) if ax == "x" else (cx, cy+t)
        if kind == "heat":
            cyl(pfx+"/heat%d" % i, (c0[0], c0[1], ct+0.40), 0.04, cross-0.30, racross, HEAT)
        elif kind == "film":
            cyl(pfx+"/film%d" % i, (c0[0], c0[1], ct+0.45), 0.10, cross-0.30, racross, FILM)
        else:  # spray (wet)
            cyl(pfx+"/spray%d" % i, (c0[0], c0[1], ct+0.42), 0.03, cross-0.30, racross, SPRAY)

def print_inside(pfx, cx, cy, w, d, z0, h, ax, racross):
    bz = z0 + min(0.6, h*0.4)
    box(pfx+"/bed", (cx, cy, bz), (w-0.4, d-0.4, 0.08), BED)
    box(pfx+"/pcb", (cx, cy, bz+0.06), (min(w-0.6,1.0), min(d-0.6,0.8), 0.02), PCB)
    cyl(pfx+"/squeegee", (cx, cy, bz+0.35), 0.05, (d if ax=="x" else w)-0.4, racross, METAL)

def expose_inside(pfx, cx, cy, w, d, z0, h, ax, racross):
    bz = z0 + min(0.7, h*0.45)
    box(pfx+"/glass", (cx, cy, bz), (w-0.4, d-0.4, 0.06), GLASS)
    cyl(pfx+"/lamp", (cx, cy, bz+0.55), 0.05, (d if ax=="x" else w)-0.4, racross, (0.95,0.93,0.8))

def ldi_inside(pfx, cx, cy, w, d, z0, h, ax, racross):
    bz = z0 + min(0.7, h*0.4)
    box(pfx+"/stage", (cx, cy, bz), (w-0.4, d-0.4, 0.06), BED)
    top = z0 + h - 0.3
    box(pfx+"/gantry", (cx, cy, top), (w-0.3, 0.18, 0.18) if ax=="x" else (0.18, d-0.3, 0.18), METAL)
    box(pfx+"/head", (cx, cy, top-0.18), (0.25,0.25,0.22), DARK)

def cmm_inside(pfx, cx, cy, w, d, z0):  # 개방 정반 + 갠트리(벽 없음)
    box(pfx+"/granite", (cx, cy, z0+0.35), (w, d, 0.30), GRANITE, True)
    for i, (sx, sy) in enumerate(((-1,-1),(1,-1),(-1,1),(1,1))):
        box(pfx+"/leg%d"%i, (cx+sx*(w/2-0.12), cy+sy*(d/2-0.12), z0+0.10), (0.12,0.12,0.20), DARK)
    bz = z0+0.5
    box(pfx+"/bridge", (cx, cy, bz+0.7), (w-0.2, 0.16, 0.5), METAL)
    box(pfx+"/probe", (cx, cy, bz+0.45), (0.1,0.1,0.3), DARK)

def reagent_tanks(pfx, cx, cy, w, d, z0, h):
    box(pfx+"/bund", (cx, cy, z0+0.06), (w, d, 0.12), DARK, True)
    n = max(1, int(max(w,d)/0.9))
    th = h-0.2
    for i in range(n):
        t = -max(w,d)/2 + (i+0.5)*max(w,d)/n
        c0 = (cx+t, cy) if w>=d else (cx, cy+t)
        cyl(pfx+"/tank%d"%i, (c0[0], c0[1], z0+0.12+th/2), min(0.30, min(w,d)*0.4), th, "Z", TANK)

def ctrl_panel(pfx, cx, cy, w, d, z0, h):
    box(pfx+"/cab", (cx, cy, z0+h/2), (w, d, h), METAL, True)
    box(pfx+"/hmi", (cx, cy-d/2-0.02, z0+h*0.62), (min(w*0.7,1.4), 0.04, h*0.3), HMI)

def ac_unit(pfx, cx, cy, w, d, z0, h):
    box(pfx+"/body", (cx, cy, z0+h/2), (w, d, h), (0.85,0.86,0.88), True)
    for i in range(4):
        box(pfx+"/louver%d"%i, (cx, cy-d/2-0.01, z0+0.25+i*0.18), (w*0.8, 0.03, 0.06), LOUVER)

def table_unit(pfx, cx, cy, w, d, z0, h):
    tt = z0 + min(0.85, h)
    box(pfx+"/top", (cx, cy, tt), (w, d, 0.06), SHELF)
    for i, (sx, sy) in enumerate(((-1,-1),(1,-1),(-1,1),(1,1))):
        box(pfx+"/leg%d"%i, (cx+sx*(w/2-0.08), cy+sy*(d/2-0.08), z0+(tt-z0)/2), (0.06,0.06,tt-z0), DARK)

def shelf_rack(pfx, cx, cy, w, d, z0, h):
    for i, (sx, sy) in enumerate(((-1,-1),(1,-1),(-1,1),(1,1))):
        box(pfx+"/post%d"%i, (cx+sx*(w/2-0.05), cy+sy*(d/2-0.05), z0+h/2), (0.05,0.05,h), DARK)
    n = max(2, int(h/0.6))
    for i in range(n):
        box(pfx+"/shelf%d"%i, (cx, cy, z0+0.2+i*(h-0.2)/n), (w-0.06, d-0.06, 0.04), SHELF)

def air_unit(pfx, cx, cy, w, d, z0, h):
    box(pfx+"/body", (cx, cy, z0+(h-0.3)/2), (w, d, h-0.3), (0.78,0.80,0.82), True)
    cyl(pfx+"/tank", (cx, cy, z0+h-0.15), 0.15, w-0.2 if w>=d else d-0.2, "X" if w>=d else "Y", METAL)

def cabinet(pfx, cx, cy, w, d, z0, h, color):
    box(pfx+"/body", (cx, cy, z0+h/2), (w, d, h), color, True)
    box(pfx+"/door", (cx, cy-d/2-0.01, z0+h*0.5), (w*0.8, 0.03, h*0.7), tuple(min(1,cc*0.85) for cc in color))

def build_model(eid, kr, x, y, w, d, zone):
    pfx = "/World/Dadong/F2/Models/" + eid
    wall = ZONE_COLOR[zone]; h = ZONE_H.get(zone, 1.5)
    if zone == "measure":
        cmm_inside(pfx, x, y, w, d, FLOOR); return
    if zone == "reagent":
        if eid == "ctrl_pnl":
            ctrl_panel(pfx, x, y, w, d, FLOOR, h)
        else:
            reagent_tanks(pfx, x, y, w, d, FLOOR, h)
        return
    if zone == "util":
        if eid.startswith("ac"):
            ac_unit(pfx, x, y, w, d, FLOOR, h)
        elif eid.startswith("table"):
            table_unit(pfx, x, y, w, d, FLOOR, h)
        elif eid.startswith("aw"):
            shelf_rack(pfx, x, y, w, d, FLOOR, h)
        elif eid.startswith("air"):
            air_unit(pfx, x, y, w, d, FLOOR, h)
        else:
            cabinet(pfx, x, y, w, d, FLOOR, h, wall)
        return
    if zone == "misc":
        if eid.startswith("rack"):
            shelf_rack(pfx, x, y, w, d, FLOOR, h)
        else:
            cabinet(pfx, x, y, w, d, FLOOR, h, wall)
        return
    # 인클로저형 (wet / dryer / df(lamin) / printer / silk / expose / ldi)
    bz0, bh, ax, rac = enclosure(pfx, x, y, w, d, FLOOR, h, wall)
    if zone in ("wet",):
        conveyor_inside(pfx, x, y, w, d, FLOOR, h, ax, rac, "spray")
    elif zone in ("dryer",):
        conveyor_inside(pfx, x, y, w, d, FLOOR, h, ax, rac, "heat")
    elif zone in ("df",):
        conveyor_inside(pfx, x, y, w, d, FLOOR, h, ax, rac, "film")
    elif zone in ("printer", "silk"):
        print_inside(pfx, x, y, w, d, FLOOR, h, ax, rac)
    elif zone == "expose":
        expose_inside(pfx, x, y, w, d, FLOOR, h, ax, rac)
    elif zone == "ldi":
        ldi_inside(pfx, x, y, w, d, FLOOR, h, ax, rac)
    else:
        conveyor_inside(pfx, x, y, w, d, FLOOR, h, ax, rac, "spray")

report = {"built": [], "skipped": [], "failed": []}
# 클린 재빌드: 이전 Models 스코프 제거(중복 xformOp 방지)
if stage.GetPrimAtPath("/World/Dadong/F2/Models"):
    stage.RemovePrim(Sdf.Path("/World/Dadong/F2/Models"))
for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
    if eid in SKIP_ID or zone in SKIP_ZONE:
        report["skipped"].append(eid); continue
    x, y = grid_to_world(cc, cr)
    epath = "/World/Dadong/F2/Equip/" + eid
    try:
        if stage.GetPrimAtPath(epath):
            stage.RemovePrim(Sdf.Path(epath))
        build_model(eid, kr, x, y, w, d, zone)
        report["built"].append(dict(id=eid, kr=kr, zone=zone))
    except Exception as e:
        report["failed"].append(dict(id=eid, zone=zone, err=str(e)))
        carb.log_warn("[model] FAIL %s (%s): %s" % (eid, zone, e))

carb.log_warn("[model] built=%d skipped=%d failed=%d"
              % (len(report["built"]), len(report["skipped"]), len(report["failed"])))
for _ in range(10):
    sim.update()

wl.save(stage)  # save world_equipment.usd in place; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("equipment"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[model] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

# 검증 렌더
import math as _m
def nrm(v):
    n = _m.sqrt(sum(c*c for c in v)) or 1.0
    return Gf.Vec3d(v[0]/n, v[1]/n, v[2]/n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def make_cam(path, target, vd, dist, focal):
    vd = nrm(vd); eye = Gf.Vec3d(target[0]-vd[0]*dist, target[1]-vd[1]*dist, target[2]-vd[2]*dist)
    up = Gf.Vec3d(0,0,1); cz = nrm((eye[0]-target[0],eye[1]-target[1],eye[2]-target[2]))
    cx = nrm(crs(up,cz)); cy = crs(cz,cx)
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal); cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955*1080.0/1920.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1,4000.0)); cam.CreateFocusDistanceAttr(dist)
    M = Gf.Matrix4d(1.0)
    M.SetRow(0,Gf.Vec4d(cx[0],cx[1],cx[2],0.0)); M.SetRow(1,Gf.Vec4d(cy[0],cy[1],cy[2],0.0))
    M.SetRow(2,Gf.Vec4d(cz[0],cz[1],cz[2],0.0)); M.SetRow(3,Gf.Vec4d(eye[0],eye[1],eye[2],1.0))
    xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(M)

make_cam("/World/WetCam", (33.0, 26.0, 7.0), (0.2, 0.9, -0.5), 16.0, 24.0)   # 습식 라인(북)
make_cam("/World/MidCam", (30.0, 15.0, 7.0), (0.6, 0.4, -0.5), 18.0, 22.0)   # 노광/LDI/DF수취

import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
cams = []
for cam, dname in (("/World/FloorCam","render_eq_top"), ("/World/WetCam","render_eq_wet"),
                   ("/World/MidCam","render_eq_mid"), ("/World/DevCam","render_eq_dev")):
    if stage.GetPrimAtPath(cam):
        dd = os.path.join(PROJ, dname); os.makedirs(dd, exist_ok=True)
        rp = rep.create.render_product(cam, (1920,1080))
        w_ = rep.WriterRegistry.get("BasicWriter"); w_.initialize(output_dir=dd, rgb=True); w_.attach([rp])
        cams.append(dname)
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = cams

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[model] done -> " + REPORT)
sim.close()
