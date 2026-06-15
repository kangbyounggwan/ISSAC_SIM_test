#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_world.py — clean 로봇 USD(atlas_clean / h1_clean)를 공장 월드에 '레퍼런스'로
4체(Atlas x2 + H1 x2) 배치. 이동(MovePrim) 없이 reference만 사용 → 비주얼 메시 보존.
노출 정상화(RTX 히스토그램 오토노출 off, 거대 필라이트 제거) 후 와이드 렌더.

  C:\\isaacsim\\python.bat build_world.py

출력:
  jm_factory_world_atlas_h1.usd   (factory + Atlas x2 + H1 x2, 평탄화)
  render_atlas_h1\\rgb_0000.png
  atlas_h1_placement.json / build_world_report.json
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ      = r"C:\Users\USER\ISSAC_SIM_test"
BASE_USD  = os.path.join(PROJ, "jm_factory_world_h1.usd")   # known-good, self-contained
ATLAS_USD = r"C:/tmp/atlas-mujoco/atlas_clean.usd"
H1_USD    = r"C:/tmp/unitree_ros/robots/h1_description/h1_clean.usd"
OUT_USD   = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
OUT_DIR   = os.path.join(PROJ, "render_atlas_h1")
PLACE_JSON= os.path.join(PROJ, "atlas_h1_placement.json")
REPORT    = os.path.join(PROJ, "build_world_report.json")
FLOOR_Z   = 6.075

report = {"base": BASE_USD, "robots": [], "notes": []}

# ----------------------------------------------------------------- 배치 후보 4곳
sys.path.insert(0, PROJ)
try:
    from dadong_3f_layout import world_boxes
    eq = []
    for b in world_boxes():
        cx, cy, _ = b["center"]; sx, sy, _ = b["size"]
        eq.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
except Exception as e:
    eq = []
    report["notes"].append("layout load failed: %s" % e)

def clearance(px, py):
    best = 1e9
    for (x0, x1, y0, y1) in eq:
        dx = max(x0 - px, 0.0, px - x1); dy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(dx, dy))
    return best

CX, CY = 41.5, 15.0
cands = []
gx = 16.0
while gx <= 67.0:
    gy = 3.0
    while gy <= 27.0:
        cands.append((gx, gy, clearance(gx, gy))); gy += 0.5
    gx += 0.5

def pick(n, mc, sep):
    v = sorted([t for t in cands if t[2] >= mc], key=lambda t: math.hypot(t[0]-CX, t[1]-CY))
    out = []
    for (x, y, c) in v:
        if all(math.hypot(x-px, y-py) >= sep for (px, py, _) in out):
            out.append((x, y, c))
            if len(out) == n:
                break
    return out

spots = []
for mc, sp in [(2.0, 3.0), (1.5, 2.6), (1.2, 2.2), (1.0, 1.8)]:
    spots = pick(4, mc, sp)
    if len(spots) == 4:
        report["notes"].append("4 spots @ mc=%.1f sep=%.1f" % (mc, sp)); break
if len(spots) < 4:
    spots = sorted(cands, key=lambda t: -t[2])[:4]

ROBOTS = [
    {"name": "Atlas_1", "usd": ATLAS_USD, "yaw": 235.0},
    {"name": "H1_1",    "usd": H1_USD,    "yaw": 200.0},
    {"name": "Atlas_2", "usd": ATLAS_USD, "yaw": 255.0},
    {"name": "H1_2",    "usd": H1_USD,    "yaw": 215.0},
]
for r, (x, y, c) in zip(ROBOTS, spots):
    r["x"], r["y"], r["clear"] = x, y, c

# ----------------------------------------------------------------- Isaac 시작
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1920, "height": 1080})
import carb, carb.settings
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf
import world_layers as wl  # layered-world I/O helpers (sys.path already has PROJ)

enable_extension("omni.replicator.core")
carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)  # 오토노출 off(블로우아웃 방지) — final_h1 검증 레시피
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(wl.ROOT)   # open the LAYERED root; re-author into sublayers (no re-flatten)
for _ in range(10):
    sim.update()
stage = ctx.get_stage()
report["default_prim"] = str(stage.GetDefaultPrim().GetPath())
wl.set_edit_layer(stage, "staging")  # old-light/cam cleanup targets world_staging.usda

# 베이스에 외부 의존(configuration) 있는지 점검
ext_refs = set()
for lyr in stage.GetUsedLayers():
    ip = lyr.identifier
    if "configuration" in ip or ip.endswith("_base.usd"):
        ext_refs.add(ip)
report["base_external_layers"] = sorted(ext_refs)

# 기존 H1 / 거대 필라이트 / 옛 카메라 제거
to_del = []
for p in stage.Traverse():
    path = str(p.GetPath())
    tn = p.GetTypeName()
    if path == "/h1_description" or path.startswith("/h1_description/"):
        if path == "/h1_description":
            to_del.append(path)
    if tn in ("SphereLight", "DiskLight") :
        li = UsdLux.LightAPI(p) if hasattr(UsdLux, "LightAPI") else None
        inten = p.GetAttribute("inputs:intensity").Get() if p.GetAttribute("inputs:intensity") else None
        if inten is None and p.GetAttribute("intensity"):
            inten = p.GetAttribute("intensity").Get()
        if inten and inten >= 50000:
            to_del.append(path)
            report["notes"].append("removed bright light %s (%.0f)" % (path, inten))
if stage.GetPrimAtPath("/World/RenderCam"):
    to_del.append("/World/RenderCam")
to_del = sorted(set(to_del))
if to_del:
    omni.kit.commands.execute("DeletePrims", paths=to_del)
report["deleted"] = to_del
for _ in range(8):
    sim.update()

# 로봇 부모 스코프 -> world_robots.usda (clear-then-readd: 중복 reference/xformOp 방지)
wl.set_edit_layer(stage, "robots")
if stage.GetPrimAtPath("/World/Robots"):
    stage.RemovePrim(Sdf.Path("/World/Robots"))
UsdGeom.Xform.Define(stage, "/World/Robots")

bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                         [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])

for r in ROBOTS:
    ppath = "/World/Robots/" + r["name"]
    prim = UsdGeom.Xform.Define(stage, ppath).GetPrim()
    ok = prim.GetReferences().AddReference(r["usd"])   # defaultPrim(robot) 인입
    for _ in range(8):
        sim.update()
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp(); rz = xf.AddRotateZOp()
    t_op.Set(Gf.Vec3d(r["x"], r["y"], 0.0)); rz.Set(r["yaw"])
    for _ in range(12):
        sim.update()
    bbox.Clear()
    rng = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    size = [round(mx[i]-mn[i], 3) for i in range(3)]
    dz = FLOOR_Z - mn[2]
    t_op.Set(Gf.Vec3d(r["x"], r["y"], dz))
    for _ in range(6):
        sim.update()
    head_z = mx[2] + dz
    n_mesh = sum(1 for q in Usd.PrimRange(prim) if q.GetTypeName() == "Mesh")
    rec = {"name": r["name"], "path": ppath, "ref": r["usd"], "x": r["x"], "y": r["y"],
           "yaw": r["yaw"], "feet_z": FLOOR_Z, "head_z": round(head_z, 3),
           "bbox_size": size, "mesh_prims": n_mesh, "clear": round(r["clear"], 3)}
    report["robots"].append(rec)
    carb.log_warn("[build] %-7s mesh=%d bbox=%s head_z=%.2f" % (r["name"], n_mesh, size, head_z))

# ----------------------------------------------------------------- 노출/조명 정상화 (final_h1 레시피)
wl.set_edit_layer(stage, "staging")  # lighting + camera -> world_staging.usda
sun = stage.GetPrimAtPath("/World/Lighting/Sun")
if sun and sun.IsValid():
    UsdLux.DistantLight(sun).GetIntensityAttr().Set(1600.0)
sky = stage.GetPrimAtPath("/World/Lighting/Sky")
if sky and sky.IsValid():
    UsdLux.DomeLight(sky).GetIntensityAttr().Set(300.0)
report["notes"].append("histogram autoexposure OFF; Sun=1600 Sky=300")

# 지붕 숨김(이미 숨겨졌을 수 있음) + 약한 그룹 필라이트
roof = stage.GetPrimAtPath("/World/Dadong/Roof")
if roof and roof.IsValid():
    UsdGeom.Imageable(roof).MakeInvisible()
gx = sum(r["x"] for r in ROBOTS)/4.0
gy = sum(r["y"] for r in ROBOTS)/4.0
maxh = max(rr["head_z"] for rr in report["robots"])
fill = UsdLux.SphereLight.Define(stage, "/World/Lighting/GroupFill")
fill.CreateRadiusAttr(1.0)
fill.CreateIntensityAttr(8000.0)                     # 약하게(메모리: 1e5는 블로우아웃)
fill.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(gx-1.5, gy-1.5, max(maxh+1.5, 9.5)))

# ----------------------------------------------------------------- 와이드 카메라
def nrm(v):
    n = math.sqrt(sum(c*c for c in v)) or 1.0
    return Gf.Vec3d(v[0]/n, v[1]/n, v[2]/n)
def crs(a, b):
    return Gf.Vec3d(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
spread = max(math.hypot(ROBOTS[i]["x"]-ROBOTS[j]["x"], ROBOTS[i]["y"]-ROBOTS[j]["y"])
             for i in range(4) for j in range(i+1, 4))
target = Gf.Vec3d(gx, gy, FLOOR_Z + 0.9)
vdir = nrm((1.0, 1.15, -1.25))
DIST = max(14.0, spread*2.0 + 8.0)
eye = Gf.Vec3d(target[0]-vdir[0]*DIST, target[1]-vdir[1]*DIST, target[2]-vdir[2]*DIST)
up = Gf.Vec3d(0, 0, 1)
cz = nrm((eye[0]-target[0], eye[1]-target[1], eye[2]-target[2]))
cx_ = nrm(crs(up, cz)); cy_ = crs(cz, cx_)
cam = UsdGeom.Camera.Define(stage, "/World/RenderCam")
cam.CreateFocalLengthAttr(30.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955*1080.0/1920.0)
cam.CreateFocusDistanceAttr(DIST)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 3000.0))
M = Gf.Matrix4d(1.0)
M.SetRow(0, Gf.Vec4d(cx_[0], cx_[1], cx_[2], 0.0))
M.SetRow(1, Gf.Vec4d(cy_[0], cy_[1], cy_[2], 0.0))
M.SetRow(2, Gf.Vec4d(cz[0], cz[1], cz[2], 0.0))
M.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder(); cxf.AddTransformOp().Set(M)
report["camera_eye"] = [round(eye[i], 2) for i in range(3)]

# ----------------------------------------------------------------- 저장 + json
wl.save(stage)  # save world_robots.usda + world_staging.usda in place; thin root preserved
report["out_usd"] = OUT_USD
report["out_bytes"] = os.path.getsize(wl.layer_path("robots"))
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[build] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)
with open(PLACE_JSON, "w", encoding="utf-8") as f:
    json.dump({"floor": "DA-2F", "floor_z": FLOOR_Z,
               "robots": report["robots"], "camera_eye": report["camera_eye"]},
              f, ensure_ascii=False, indent=2)

# ----------------------------------------------------------------- 렌더
os.makedirs(OUT_DIR, exist_ok=True)
import omni.replicator.core as rep
rep.orchestrator.set_capture_on_play(False)
rp = rep.create.render_product("/World/RenderCam", (1920, 1080))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True)
writer.attach([rp])
for _ in range(120):
    sim.update()
rep.orchestrator.step(rt_subframes=96)
rep.orchestrator.wait_until_complete()
for _ in range(10):
    sim.update()
report["rendered"] = True

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[build] report -> " + REPORT)

sim.close()
