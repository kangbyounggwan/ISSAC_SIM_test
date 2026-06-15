#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detail_and_label.py — (1) 정면기2+현상기1을 개방형(상단 개방 + 전면 로우월) 컷어웨이로 재구성해
내부 컨베이어 롤러 / PCB 패널 / 스프레이바(현상기)·브러시(정면기)를 노출,
(2) 7개 투입/배출(io) 스테이션에 한글 텍스트 표지판(textures/label_*.png) + 바닥 화살표 표기.
현재 월드 직접편집 + 덮어쓰기 + 검증렌더.

  C:\\isaacsim\\python.bat detail_and_label.py
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
TEX_IN  = "C:/Users/USER/ISSAC_SIM_test/textures/label_in.png"
TEX_OUT = "C:/Users/USER/ISSAC_SIM_test/textures/label_out.png"
REPORT  = os.path.join(PROJ, "detail_and_label_report.json")
FLOOR = 6.075

BASE=(0.30,0.31,0.34); BODY=(0.74,0.76,0.79); SLOT=(0.10,0.10,0.13); ROLLER=(0.62,0.64,0.67)
FRAME=(0.40,0.41,0.45); EXH=(0.50,0.51,0.55); CAB=(0.55,0.57,0.61); HMI=(0.10,0.16,0.34)
SIG=[(0.85,0.12,0.12),(0.88,0.72,0.12),(0.12,0.70,0.22)]; POLE=(0.25,0.25,0.28)
TANK=(0.46,0.56,0.64); PCB=(0.11,0.44,0.20); SPRAY=(0.52,0.58,0.66); BRUSH=(0.64,0.52,0.34)
TROUGH=(0.50,0.51,0.55); ARROW_IN=(0.12,0.55,0.25); ARROW_OUT=(0.85,0.45,0.10); POLE2=(0.30,0.31,0.34)

sys.path.insert(0, PROJ)
from dadong_3f_layout import EQUIPMENT, grid_to_world

MACHINES = []   # 정면기/현상기
IO = {
 "psr_in":    dict(role="in",  dirx=1.0,  diry=0.0),
 "df_in":     dict(role="in",  dirx=-1.0, diry=0.0),
 "psr_dev_in":dict(role="in",  dirx=0.0,  diry=1.0),
 "wet_recv":  dict(role="out", dirx=1.0,  diry=0.0),
 "psr_recv":  dict(role="out", dirx=1.0,  diry=0.0),
 "df_recv":   dict(role="out", dirx=-1.0, diry=0.0),
 "tunnel_rv": dict(role="out", dirx=0.0,  diry=-1.0),
}
IO_ST = []
for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
    x, y = grid_to_world(cc, cr)
    if eid in ("psr_lvl", "df_lvl", "psr_dev"):
        axis = "y" if d >= w else "x"
        MACHINES.append(dict(id=eid, kr=kr, x=x, y=y, axis=axis,
                             length=(d if axis=="y" else w), depth=(w if axis=="y" else d),
                             kind=("developer" if eid=="psr_dev" else "leveler")))
    if eid in IO:
        axis = "y" if d >= w else "x"
        IO_ST.append(dict(id=eid, x=x, y=y, axis=axis, **IO[eid]))

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1920, "height": 1080})
import carb, carb.settings
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

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

# ---- 개방형 컷어웨이 설비 ----
def build_open_machine(prefix, cx, cy, length, depth, z0, axis, kind):
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
    def across(name, a, z, r, color):     # 폭 방향(cross) 원통: 롤러/스프레이바/브러시
        px, py = W(a, 0); cyl(prefix+name, (px, py, z), r, depth-0.20, roller_axis, color)

    bz0 = z0 + 0.30
    bh = 1.75 if kind == "developer" else 1.55
    blen = length - 2.6
    ct = z0 + 0.95
    bx("/base", 0, 0, z0+0.15, length, depth, 0.30, BASE, True)
    bx("/trough", 0, 0, bz0+0.06, blen-0.1, depth-0.14, 0.10, TROUGH)        # 내부 바닥
    bx("/wall_back", 0, depth/2-0.04, bz0+bh/2, blen, 0.08, bh, BODY, True)  # 후면 풀월
    bx("/lip_front", 0, front+0.04, bz0+0.20, blen, 0.08, 0.40, BODY)        # 전면 로우월(컷어웨이)
    for s, lab in ((-1,"A"), (1,"B")):
        bx("/wall_end%s"%lab, s*(blen/2-0.04), 0, bz0+bh/2, 0.08, depth, bh, BODY, True)
    # 내부 관통 컨베이어 롤러
    nr = max(6, int(blen/0.55))
    for i in range(nr):
        a = -blen/2 + (i+0.5)*blen/nr
        across("/cv_roll%d"%i, a, ct, 0.05, ROLLER)
    # 이송 중 PCB 패널
    npcb = max(3, int(blen/1.4))
    for i in range(npcb):
        a = -blen/2 + 0.7 + i*(blen-1.4)/max(1,(npcb-1))
        bx("/pcb%d"%i, a, 0, ct+0.06, 0.45, depth-0.55, 0.02, PCB)
    # 공정 모듈(현상기=스프레이바 / 정면기=브러시롤러 상하)
    nmod = max(3, int(blen/1.1))
    for i in range(nmod):
        a = -blen/2 + (i+0.5)*blen/nmod
        if kind == "developer":
            across("/spray%d"%i, a, ct+0.42, 0.035, SPRAY)
            cz("/nz_%d"%i, a, 0, ct+0.30, 0.02, 0.16, SPRAY)
        else:
            across("/brush_t%d"%i, a, ct+0.17, 0.10, BRUSH)
            across("/brush_b%d"%i, a, ct-0.12, 0.10, BRUSH)
    # 상부 배기 + (현상기) 약액탱크
    nex = 4 if kind == "developer" else 3
    for i in range(nex):
        a = -blen/2 + blen*(i+0.5)/nex
        cz("/exh%d"%i, a, depth/2-0.16, bz0+bh+0.40, 0.12, 0.80, EXH)
    if kind == "developer":
        for i in range(3):
            a = -blen/3.0 + i*(blen/3.0)
            cz("/tank%d"%i, a, depth/2+0.45, z0+0.65, 0.32, 1.25, TANK)
    # 인입/인출 외부 롤러
    for end, sgn in (("in",-1), ("out",1)):
        e0 = sgn*(blen/2.0); e1 = sgn*(length/2.0); seg_a=(e0+e1)/2.0; seg_l=abs(e1-e0)
        for s, lab in ((-1,"A"),(1,"B")):
            bx("/%s_frame%s"%(end,lab), seg_a, s*(depth/2-0.06), ct-0.07, seg_l, 0.06, 0.12, FRAME, True)
        for i in range(4):
            a = e0 + (e1-e0)*(i+0.5)/4.0
            across("/%s_roll%d"%(end,i), a, ct, 0.05, ROLLER)
    # 제어 캐비닛 + HMI + 신호등
    ce = -(length/2 - 0.6)
    bx("/cab", ce, front-0.45, z0+0.85, 0.9, 0.8, 1.6, CAB, True)
    bx("/hmi", ce, front-0.86, z0+1.35, 0.5, 0.04, 0.4, HMI)
    cz("/sig_pole", ce, front-0.45, z0+1.65+0.25, 0.03, 0.5, POLE)
    for j, col in enumerate(SIG):
        cz("/sig%d"%j, ce, front-0.45, z0+1.95+0.085*j, 0.07, 0.08, col)

# ---- 텍스트 표지판(텍스처 평면) ----
def make_sign(prefix, x, y, z, yaw_deg, tex, w=1.0, h=0.5):
    m = UsdGeom.Mesh.Define(stage, prefix+"/board")
    pts = [(-w/2,0,-h/2),(w/2,0,-h/2),(w/2,0,h/2),(-w/2,0,h/2)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4]); m.CreateFaceVertexIndicesAttr([0,1,2,3])
    m.CreateExtentAttr([(-w/2,-0.02,-h/2),(w/2,0.02,h/2)]); m.CreateDoubleSidedAttr(True)
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([(0,0),(1,0),(1,1),(0,1)])
    xf = UsdGeom.Xformable(m); xf.AddTranslateOp().Set(Gf.Vec3d(x,y,z)); xf.AddRotateZOp().Set(yaw_deg)
    mat = UsdShade.Material.Define(stage, prefix+"/mat")
    sh = UsdShade.Shader.Define(stage, prefix+"/mat/surf"); sh.CreateIdAttr("UsdPreviewSurface")
    tx = UsdShade.Shader.Define(stage, prefix+"/mat/tex"); tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
    tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    rd = UsdShade.Shader.Define(stage, prefix+"/mat/st"); rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.CreateOutput("result", Sdf.ValueTypeNames.Float2))
    rgb = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    mat.CreateSurfaceOutput().ConnectToSource(sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(m).Bind(mat)
    box(prefix+"/pole", (x, y, (FLOOR + (z-h/2))/2.0), (0.05, 0.05, (z-h/2)-FLOOR), POLE2)

# ---- 바닥 화살표(흐름 방향) ----
def make_arrow(prefix, x, y, dx, dy, color):
    poly = [(-0.55,-0.09),(0.10,-0.09),(0.10,-0.24),(0.58,0.0),(0.10,0.24),(0.10,0.09),(-0.55,0.09)]
    m = UsdGeom.Mesh.Define(stage, prefix+"/arrow")
    m.CreatePointsAttr([Gf.Vec3f(px,py,0.0) for (px,py) in poly])
    m.CreateFaceVertexCountsAttr([len(poly)]); m.CreateFaceVertexIndicesAttr(list(range(len(poly))))
    m.CreateExtentAttr([(-0.6,-0.3,-0.01),(0.6,0.3,0.01)]); m.CreateDoubleSidedAttr(True)
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    ang = math.degrees(math.atan2(dy, dx))
    xf = UsdGeom.Xformable(m); xf.AddTranslateOp().Set(Gf.Vec3d(x, y, FLOOR+0.03)); xf.AddRotateZOp().Set(ang)

report = {"machines": [], "labels": []}

# 1) 설비 재구성 -> world_equipment.usd
wl.set_edit_layer(stage, "equipment")
for mc in MACHINES:
    pfx = ("/World/Dadong/F2/Levelers/" if mc["kind"]=="leveler" else "/World/Dadong/F2/Developers/") + mc["id"]
    if stage.GetPrimAtPath(pfx):
        stage.RemovePrim(Sdf.Path(pfx))
    build_open_machine(pfx, mc["x"], mc["y"], mc["length"], mc["depth"], FLOOR, mc["axis"], mc["kind"])
    report["machines"].append(dict(id=mc["id"], kind=mc["kind"], axis=mc["axis"], prefix=pfx))
    carb.log_warn("[detail] %s (%s) rebuilt open" % (mc["id"], mc["kind"]))

# 2) 투입/배출 표기 -> world_labels.usda
wl.set_edit_layer(stage, "labels")
for st in IO_ST:
    x, y, axis, role = st["x"], st["y"], st["axis"], st["role"]
    tex = TEX_IN if role == "in" else TEX_OUT
    acol = ARROW_IN if role == "in" else ARROW_OUT
    if axis == "x":
        sy = y + (-0.85 if y > 15 else 0.85); sx = x; face = 0.0 if y > 15 else 180.0
        ay = y + (-1.1 if y > 15 else 1.1); ax = x
    else:
        sx = x + (-0.85 if x > 41.5 else 0.85); sy = y; face = 270.0 if x > 41.5 else 90.0
        ax = x + (-1.1 if x > 41.5 else 1.1); ay = y
    p = "/World/Dadong/F2/Labels/" + st["id"]
    make_sign(p, sx, sy, FLOOR+2.25, face, tex)
    make_arrow(p, ax, ay, st["dirx"], st["diry"], acol)
    report["labels"].append(dict(id=st["id"], role=role, sign=[round(sx,2),round(sy,2)], face=face,
                                 arrow=[round(ax,2),round(ay,2)], dir=[st["dirx"],st["diry"]]))
    carb.log_warn("[label] %s role=%s sign=(%.1f,%.1f)" % (st["id"], role, sx, sy))

for _ in range(10):
    sim.update()

wl.save(stage)  # save world_equipment.usd + world_labels.usda; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("labels"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[detail] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

# 검증 렌더 (오버헤드 + 정면기/현상기 클로즈업; 기존 카메라 재사용)
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
cams = []
for cam, d in (("/World/FloorCam","render_dl_top"), ("/World/LevelerCam","render_dl_lvl"),
               ("/World/DevCam","render_dl_dev")):
    if stage.GetPrimAtPath(cam):
        dd = os.path.join(PROJ, d); os.makedirs(dd, exist_ok=True)
        rp = rep.create.render_product(cam, (1920,1080))
        w = rep.WriterRegistry.get("BasicWriter"); w.initialize(output_dir=dd, rgb=True); w.attach([rp])
        cams.append(d)
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = cams

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[detail] done -> " + REPORT)
sim.close()
