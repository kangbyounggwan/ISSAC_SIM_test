#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
add_developer.py — PSR 현상기(psr_dev) 박스 플레이스홀더를 절차적 상세 모델로 교체.
축-범용 인라인 설비 빌더(정면기와 동일 패턴, axis 자동) + 현상기 특화(약액탱크/점검창/배기4).
현재 월드 직접편집 + 덮어쓰기 + 검증렌더. *반드시 정면기 작업(add_levelers) 종료 후 단독 실행*.

  C:\\isaacsim\\python.bat add_developer.py
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
CAM_DIR = os.path.join(PROJ, "render_developer")
TOP_DIR = os.path.join(PROJ, "render_developer_top")
REPORT  = os.path.join(PROJ, "add_developer_report.json")
FLOOR = 6.075

BASE=(0.30,0.31,0.34); BODY=(0.74,0.76,0.79); DOOR=(0.66,0.68,0.72); SLOT=(0.10,0.10,0.13)
ROLLER=(0.62,0.64,0.67); FRAME=(0.40,0.41,0.45); EXH=(0.50,0.51,0.55); CAB=(0.55,0.57,0.61)
HMI=(0.10,0.16,0.34); SIG=[(0.85,0.12,0.12),(0.88,0.72,0.12),(0.12,0.70,0.22)]
POLE=(0.25,0.25,0.28); TANK=(0.46,0.56,0.64)

# 대상: PSR 현상기(psr_dev)
sys.path.insert(0, PROJ)
from dadong_3f_layout import EQUIPMENT, grid_to_world
TARGETS = []
for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
    if eid == "psr_dev":
        x, y = grid_to_world(cc, cr)
        axis = "y" if d >= w else "x"
        length = d if axis == "y" else w
        depth = w if axis == "y" else d
        TARGETS.append(dict(id=eid, kr=kr, x=x, y=y, axis=axis, length=length, depth=depth))

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
wl.set_edit_layer(stage, "equipment")  # Developers -> world_equipment.usd

def box(path, center, size, color, collider=False):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0); c.CreateExtentAttr([(-0.5,-0.5,-0.5),(0.5,0.5,0.5)])
    c.AddTranslateOp().Set(Gf.Vec3d(*center)); c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetPrim().CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set("developer")

def cyl(path, center, radius, height, axis, color):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius); c.CreateHeightAttr(height); c.CreateAxisAttr(axis)
    r, h = radius, height/2.0
    ext = {"Z": [(-r,-r,-h),(r,r,h)], "Y": [(-r,-h,-r),(r,h,r)], "X": [(-h,-r,-r),(h,r,r)]}[axis]
    c.CreateExtentAttr(ext)
    c.AddTranslateOp().Set(Gf.Vec3d(*center)); c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    c.GetPrim().CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set("developer")

def build_machine(prefix, cx, cy, length, depth, z0, axis, developer=True):
    front = -depth/2.0
    roller_axis = "Y" if axis == "x" else "X"
    def W(a, c):
        return (cx+a, cy+c) if axis == "x" else (cx+c, cy+a)
    def Sz(la, cr, h):
        return (la, cr, h) if axis == "x" else (cr, la, h)
    def bx(name, a, c, z, la, cr, h, color, col=False):
        px, py = W(a, c); box(prefix+name, (px, py, z), Sz(la, cr, h), color, col)
    def cz(name, a, c, z, r, h, color):
        px, py = W(a, c); cyl(prefix+name, (px, py, z), r, h, "Z", color)
    def rol(name, a, z, r, h):
        px, py = W(a, 0); cyl(prefix+name, (px, py, z), r, h, roller_axis, ROLLER)

    base_h = 0.30
    bx("/base", 0, 0, z0+base_h/2, length, depth, base_h, BASE, True)
    bz0 = z0 + base_h
    bh = 1.75 if developer else 1.55
    blen = length - 2.6
    bx("/body", 0, 0, bz0+bh/2, blen, depth, bh, BODY, True)
    ct = z0 + 0.95
    bx("/slot", 0, front-0.01, ct, blen*0.95, 0.05, 0.13, SLOT)
    nd = max(3, int(round(blen/2.0))); dw = blen/nd
    for i in range(nd):
        a = -blen/2 + dw*(i+0.5)
        bx("/door%d" % i, a, front-0.02, bz0+0.30+(bh-0.55)/2, dw*0.82, 0.04, bh-0.55, DOOR)
        if developer:
            bx("/win%d" % i, a, front-0.02, bz0+bh-0.22, dw*0.5, 0.05, 0.16, SLOT)
    nex = 4 if developer else 3
    for i in range(nex):
        a = -blen/2 + blen*(i+0.5)/nex
        cz("/exh%d" % i, a, 0, bz0+bh+0.40, 0.12, 0.80, EXH)
    if developer:
        for i in range(3):
            a = -blen/3.0 + i*(blen/3.0)
            cz("/tank%d" % i, a, depth/2+0.45, z0+0.65, 0.32, 1.25, TANK)
    for end, sgn in (("in", -1), ("out", 1)):
        e0 = sgn*(blen/2.0); e1 = sgn*(length/2.0)
        seg_a = (e0+e1)/2.0; seg_l = abs(e1-e0)
        for s, lab in ((-1, "A"), (1, "B")):
            bx("/%s_frame%s" % (end, lab), seg_a, s*(depth/2-0.06), ct-0.07, seg_l, 0.06, 0.12, FRAME, True)
        for i in range(4):
            a = e0 + (e1-e0)*(i+0.5)/4.0
            rol("/%s_roll%d" % (end, i), a, ct, 0.05, depth-0.12)
    ce = -(length/2 - 0.6)
    bx("/cab", ce, front-0.45, z0+0.85, 0.9, 0.8, 1.6, CAB, True)
    bx("/hmi", ce, front-0.86, z0+1.35, 0.5, 0.04, 0.4, HMI)
    cz("/sig_pole", ce, front-0.45, z0+1.65+0.25, 0.03, 0.5, POLE)
    for j, col in enumerate(SIG):
        cz("/sig%d" % j, ce, front-0.45, z0+1.95+0.085*j, 0.07, 0.08, col)

report = {"targets": [], "deleted": []}
for t in TARGETS:
    epath = "/World/Dadong/F2/Equip/" + t["id"]
    if stage.GetPrimAtPath(epath):
        stage.RemovePrim(Sdf.Path(epath)); report["deleted"].append(epath)
    pfx = "/World/Dadong/F2/Developers/" + t["id"]
    build_machine(pfx, t["x"], t["y"], t["length"], t["depth"], FLOOR, t["axis"], developer=True)
    report["targets"].append(dict(id=t["id"], kr=t["kr"], center=[round(t["x"],2), round(t["y"],2)],
                                  axis=t["axis"], length=t["length"], depth=t["depth"], prefix=pfx))
    carb.log_warn("[dev] %s at (%.1f,%.1f) axis=%s len=%.1f" % (t["id"], t["x"], t["y"], t["axis"], t["length"]))

for _ in range(10):
    sim.update()

def nrm(v):
    n = math.sqrt(sum(c*c for c in v)) or 1.0
    return Gf.Vec3d(v[0]/n, v[1]/n, v[2]/n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def make_cam(path, target, view_dir, dist, focal):
    vd = nrm(view_dir)
    eye = Gf.Vec3d(target[0]-vd[0]*dist, target[1]-vd[1]*dist, target[2]-vd[2]*dist)
    up = Gf.Vec3d(0,0,1); cz_ = nrm((eye[0]-target[0],eye[1]-target[1],eye[2]-target[2]))
    cx_ = nrm(crs(up,cz_)); cy_ = crs(cz_,cx_)
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal); cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955*1080.0/1920.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1,4000.0)); cam.CreateFocusDistanceAttr(dist)
    M = Gf.Matrix4d(1.0)
    M.SetRow(0,Gf.Vec4d(cx_[0],cx_[1],cx_[2],0.0)); M.SetRow(1,Gf.Vec4d(cy_[0],cy_[1],cy_[2],0.0))
    M.SetRow(2,Gf.Vec4d(cz_[0],cz_[1],cz_[2],0.0)); M.SetRow(3,Gf.Vec4d(eye[0],eye[1],eye[2],1.0))
    xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(M)
    return [round(eye[0],1),round(eye[1],1),round(eye[2],1)]

wl.set_edit_layer(stage, "staging")  # persistent camera -> world_staging.usda
t0 = TARGETS[0]
report["dev_cam_eye"] = make_cam("/World/DevCam", (t0["x"], t0["y"], FLOOR+1.1),
                                 (0.75, 0.12, -0.45), 15.0, 20.0)

wl.save(stage)  # save world_equipment.usd + world_staging.usda; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("equipment"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[dev] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
os.makedirs(CAM_DIR, exist_ok=True); os.makedirs(TOP_DIR, exist_ok=True)
prods = [("/World/DevCam", CAM_DIR)]
if stage.GetPrimAtPath("/World/FloorCam"):
    prods.append(("/World/FloorCam", TOP_DIR))
for cam, dd in prods:
    rp = rep.create.render_product(cam, (1920,1080))
    w = rep.WriterRegistry.get("BasicWriter"); w.initialize(output_dir=dd, rgb=True); w.attach([rp])
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = [d for _, d in prods]

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[dev] done -> " + REPORT)
sim.close()
