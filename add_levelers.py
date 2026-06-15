#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
add_levelers.py — 정면기(psr_lvl/df_lvl) 박스 플레이스홀더를 절차적 상세 모델로 교체.
인라인 정면기: 베이스 + 스테인리스 본체 + 점검도어 + 관통 슬롯 + 인입/인출 롤러 +
상부 배기덕트 + 제어캐비닛/HMI + 신호등 타워. 현재 월드 직접편집 + 덮어쓰기 + 검증렌더.

  C:\\isaacsim\\python.bat add_levelers.py
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
CAM_DIR = os.path.join(PROJ, "render_leveler")
TOP_DIR = os.path.join(PROJ, "render_leveler_top")
REPORT  = os.path.join(PROJ, "add_levelers_report.json")
FLOOR = 6.075

# 색상
BASE=(0.30,0.31,0.34); BODY=(0.74,0.76,0.79); DOOR=(0.66,0.68,0.72); SLOT=(0.10,0.10,0.13)
ROLLER=(0.62,0.64,0.67); FRAME=(0.40,0.41,0.45); EXH=(0.50,0.51,0.55); CAB=(0.55,0.57,0.61)
HMI=(0.10,0.16,0.34); SIG=[(0.85,0.12,0.12),(0.88,0.72,0.12),(0.12,0.70,0.22)]; POLE=(0.25,0.25,0.28)

# 대상 정면기 (layout: psr_lvl, df_lvl) — 월드 좌표/치수 재계산
sys.path.insert(0, PROJ)
from dadong_3f_layout import EQUIPMENT, grid_to_world
LEVELERS = []
for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
    if eid in ("psr_lvl", "df_lvl"):
        x, y = grid_to_world(cc, cr)
        LEVELERS.append(dict(id=eid, kr=kr, x=x, y=y, L=w, D=d))

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
wl.set_edit_layer(stage, "equipment")  # Levelers -> world_equipment.usd

def box(path, center, size, color, collider=False):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0); c.CreateExtentAttr([(-0.5,-0.5,-0.5),(0.5,0.5,0.5)])
    c.AddTranslateOp().Set(Gf.Vec3d(*center)); c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetPrim().CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set("leveler")

def cyl(path, center, radius, height, axis, color):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius); c.CreateHeightAttr(height); c.CreateAxisAttr(axis)
    r, h = radius, height/2.0
    if axis == "Z":
        ext = [(-r,-r,-h),(r,r,h)]
    elif axis == "Y":
        ext = [(-r,-h,-r),(r,h,r)]
    else:
        ext = [(-h,-r,-r),(h,r,r)]
    c.CreateExtentAttr(ext)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    c.GetPrim().CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set("leveler")

def build_leveler(prefix, cx, cy, L, D, z0):
    fy = cy - D/2.0            # front face(남측, -Y)
    base_h = 0.30
    box(prefix+"/base", (cx, cy, z0+base_h/2), (L, D, base_h), BASE, collider=True)
    body_z0 = z0 + base_h
    body_h = 1.55
    body_len = L - 2.6
    box(prefix+"/body", (cx, cy, body_z0+body_h/2), (body_len, D, body_h), BODY, collider=True)
    ct_z = z0 + 0.95           # 관통 컨베이어 높이
    # 관통 슬롯(전면)
    box(prefix+"/slot", (cx, fy-0.01, ct_z), (body_len*0.95, 0.05, 0.13), SLOT)
    # 점검 도어(전면)
    nd = max(3, int(round(body_len/2.0)))
    dw = body_len/nd
    for i in range(nd):
        dx = cx - body_len/2 + dw*(i+0.5)
        box(prefix+("/door%d"%i), (dx, fy-0.02, body_z0+0.30+ (body_h-0.55)/2),
            (dw*0.82, 0.04, body_h-0.55), DOOR)
    # 상부 배기덕트
    for i in range(3):
        ex = cx - body_len/2 + body_len*(i+0.5)/3.0
        cyl(prefix+("/exh%d"%i), (ex, cy, body_z0+body_h+0.40), 0.12, 0.80, "Z", EXH)
    # 인입/인출 롤러 컨베이어(양 끝)
    for end, sgn in (("in", -1), ("out", 1)):
        e0 = cx + sgn*(body_len/2.0)
        e1 = cx + sgn*(L/2.0)
        seg_cx = (e0+e1)/2.0; seg_len = abs(e1-e0)
        for s, lab in ((-1, "A"), (1, "B")):
            box(prefix+("/%s_frame%s"%(end, lab)), (seg_cx, cy+s*(D/2-0.06), ct_z-0.07),
                (seg_len, 0.06, 0.12), FRAME, collider=True)
        for i in range(4):
            rx = e0 + (e1-e0)*(i+0.5)/4.0
            cyl(prefix+("/%s_roll%d"%(end, i)), (rx, cy, ct_z), 0.05, D-0.12, "Y", ROLLER)
    # 제어 캐비닛 + HMI (인출측 전면)
    cab_x = cx + L/2 - 0.6
    box(prefix+"/cab", (cab_x, fy-0.45, z0+0.85), (0.9, 0.8, 1.6), CAB, collider=True)
    box(prefix+"/hmi", (cab_x, fy-0.86, z0+1.35), (0.5, 0.04, 0.4), HMI)
    # 신호등 타워
    cyl(prefix+"/sig_pole", (cab_x, fy-0.45, z0+1.65+0.25), 0.03, 0.5, "Z", POLE)
    for j, col in enumerate(SIG):
        cyl(prefix+("/sig%d"%j), (cab_x, fy-0.45, z0+1.95+0.085*j), 0.07, 0.08, "Z", col)

report = {"levelers": [], "deleted": []}
for lv in LEVELERS:
    epath = "/World/Dadong/F2/Equip/" + lv["id"]
    if stage.GetPrimAtPath(epath):
        stage.RemovePrim(Sdf.Path(epath)); report["deleted"].append(epath)
    pfx = "/World/Dadong/F2/Levelers/" + lv["id"]
    build_leveler(pfx, lv["x"], lv["y"], lv["L"], lv["D"], FLOOR)
    report["levelers"].append(dict(id=lv["id"], kr=lv["kr"], center=[round(lv["x"],2), round(lv["y"],2)],
                                   L=lv["L"], D=lv["D"], prefix=pfx))
    carb.log_warn("[lvl] %s at (%.1f,%.1f) L=%.1f" % (lv["id"], lv["x"], lv["y"], lv["L"]))

for _ in range(10):
    sim.update()

# ---- 카메라(정면기 클로즈업 + 오버헤드) ----
def nrm(v):
    n = math.sqrt(sum(c*c for c in v)) or 1.0
    return Gf.Vec3d(v[0]/n, v[1]/n, v[2]/n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def make_cam(path, target, view_dir, dist, focal):
    vd = nrm(view_dir)
    eye = Gf.Vec3d(target[0]-vd[0]*dist, target[1]-vd[1]*dist, target[2]-vd[2]*dist)
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
    return [round(eye[0],1),round(eye[1],1),round(eye[2],1)]

wl.set_edit_layer(stage, "staging")  # persistent camera -> world_staging.usda
df = [l for l in LEVELERS if l["id"] == "df_lvl"][0]
report["leveler_cam_eye"] = make_cam("/World/LevelerCam", (df["x"], df["y"], FLOOR+1.1),
                                     (0.35, 0.85, -0.42), 13.0, 26.0)

wl.save(stage)  # save world_equipment.usd + world_staging.usda; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("equipment"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[lvl] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
os.makedirs(CAM_DIR, exist_ok=True); os.makedirs(TOP_DIR, exist_ok=True)
prods = [("/World/LevelerCam", CAM_DIR)]
if stage.GetPrimAtPath("/World/FloorCam"):
    prods.append(("/World/FloorCam", TOP_DIR))
writers = []
for cam, d in prods:
    rp = rep.create.render_product(cam, (1920,1080))
    w = rep.WriterRegistry.get("BasicWriter"); w.initialize(output_dir=d, rgb=True); w.attach([rp])
    writers.append(w)
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = [d for _, d in prods]

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[lvl] done. report -> " + REPORT)
sim.close()
