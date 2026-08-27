#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_smic_world.py — SMIC World GENESIS.

KETI 컨셉(안) 셀(레이저 커팅 → 중력 슬라이드 → 휴머노이드 분류 → 프레스 → AMR 반출)을
thin 루트 + 6 서브레이어 구조로 생성한다. 순수 pxr — Kit/GPU 불필요(~2초).

  C:\\isaacsim\\python.bat build_smic_world.py             # 없을 때만 생성
  set SMIC_FORCE=1 & C:\\isaacsim\\python.bat build_smic_world.py   # 전체 재생성

모든 치수는 smic_layout.py 에서만 온다. 이 파일에는 배치 로직만 둔다.
"""
import os, sys, json, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401
import world_layers as wl
import smic_layout as L
import smic_models as M
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Sdf, Gf

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "build_smic_world_report.json")

if os.path.exists(wl.ROOT) and not os.environ.get("SMIC_FORCE"):
    sys.exit("[smic] %s already exists. set SMIC_FORCE=1 to rebuild from scratch."
             % os.path.basename(wl.ROOT))

report = {"stations": [], "counts": {}, "notes": []}

# ============================================================ 1. 레이어 골격
for key in wl.LAYER_KEYS:
    p = wl.layer_path(key)
    lyr = Sdf.Layer.FindOrOpen(p) or Sdf.Layer.CreateNew(p)
    lyr.Clear()
    lyr.Save()

root = Sdf.Layer.FindOrOpen(wl.ROOT) or Sdf.Layer.CreateNew(wl.ROOT)
root.Clear()
root.subLayerPaths[:] = wl.SUBLAYERS
root.defaultPrim = "World"
root.Save()

stage = Usd.Stage.Open(wl.ROOT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.GetRootLayer().Save()          # 루트에 Z-up/미터 각인 (안 하면 GUI 가 Y-up)

BAY = L.BAY
CX = (BAY["x0"] + BAY["x1"]) / 2.0
CY = (BAY["y0"] + BAY["y1"]) / 2.0
BW = BAY["x1"] - BAY["x0"]
BD = BAY["y1"] - BAY["y0"]

# ============================================================ 2. base 레이어
wl.set_edit_layer(stage, "base")
UsdGeom.Xform.Define(stage, "/World")

scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# --- 물리 머티리얼: 중력 슈트는 저마찰이어야 판재가 실제로 미끄러진다 -------------
#     (경사 18.2° → tanθ=0.33. 기본 마찰 0.5 로는 안 내려간다.)
UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
CHUTE_MAT = "/World/PhysicsMaterials/ChuteLowFriction"
_m = UsdShade.Material.Define(stage, CHUTE_MAT)
_pm = UsdPhysics.MaterialAPI.Apply(_m.GetPrim())
_pm.CreateStaticFrictionAttr(0.12)
_pm.CreateDynamicFrictionAttr(0.10)
_pm.CreateRestitutionAttr(0.0)

GRIP_MAT = "/World/PhysicsMaterials/GripHighFriction"
_g = UsdShade.Material.Define(stage, GRIP_MAT)
_gm = UsdPhysics.MaterialAPI.Apply(_g.GetPrim())
_gm.CreateStaticFrictionAttr(1.10)
_gm.CreateDynamicFrictionAttr(0.95)
_gm.CreateRestitutionAttr(0.0)

# --- 지면 슬래브 ---------------------------------------------------------------
UsdGeom.Xform.Define(stage, "/World/Site")
M.box(stage, "/World/Site/Ground",
      (CX, CY, L.FLOOR_Z - 0.10),
      (BW + 2 * L.SLAB_MARGIN, BD + 2 * L.SLAB_MARGIN, 0.20),
      L.C_CONCRETE, coll=True)

# --- 건물 셸: 후면(창문 벽) + 좌/우 벽 + 기둥 + (숨김) 지붕 -----------------------
#     전면(-Y)은 개념도처럼 컷어웨이 — 카메라가 셀 안을 보도록 벽을 세우지 않는다.
WT, WH = 0.25, BAY["h"]
UsdGeom.Xform.Define(stage, "/World/Site/Building")
SILL_Z, HEAD_Z = 2.60, 5.40
M.box(stage, "/World/Site/Building/wall_back_sill",
      (CX, BAY["y1"] + WT / 2, SILL_Z / 2), (BW + 2 * WT, WT, SILL_Z), L.C_WALL, coll=True)
M.box(stage, "/World/Site/Building/wall_back_head",
      (CX, BAY["y1"] + WT / 2, (HEAD_Z + WH) / 2), (BW + 2 * WT, WT, WH - HEAD_Z),
      L.C_WALL, coll=True)
N_BAYS, PIER = 5, 0.50
span = BW / N_BAYS
for i in range(N_BAYS + 1):
    px = BAY["x0"] + i * span
    M.box(stage, "/World/Site/Building/pier_%d" % i,
          (px, BAY["y1"] + WT / 2, (SILL_Z + HEAD_Z) / 2), (PIER, WT, HEAD_Z - SILL_Z),
          L.C_WALL, coll=True)
for i in range(N_BAYS):
    gx = BAY["x0"] + (i + 0.5) * span
    M.box(stage, "/World/Site/Building/glass_%d" % i,
          (gx, BAY["y1"] + WT / 2, (SILL_Z + HEAD_Z) / 2),
          (span - PIER, 0.06, HEAD_Z - SILL_Z), L.C_GLASS)
for sx, nm in ((-1, "wall_left"), (1, "wall_right")):
    x = (BAY["x0"] - WT / 2) if sx < 0 else (BAY["x1"] + WT / 2)
    M.box(stage, "/World/Site/Building/" + nm, (x, CY, WH / 2), (WT, BD, WH),
          L.C_WALL, coll=True)
for i, cxx in enumerate((2.50, 7.00, 18.50)):     # 후면부 구조 기둥(개념도의 진회색 프레임)
    M.box(stage, "/World/Site/Building/column_%d" % i, (cxx, BAY["y1"] - 1.10, WH / 2),
          (0.35, 0.35, WH), (0.36, 0.37, 0.39), coll=True)
roof = M.box(stage, "/World/Site/Building/roof", (CX, CY, WH + 0.15), (BW + 2 * WT, BD + WT, 0.30),
             (0.50, 0.51, 0.53))
UsdGeom.Imageable(roof.GetPrim()).MakeInvisible()   # 렌더/조작 시야 확보 (JM 관례)

# --- 노란 바닥 라인 마킹 --------------------------------------------------------
UsdGeom.Scope.Define(stage, "/World/Site/Marking")
for i, (a, b) in enumerate(L.LANE_SEGMENTS):
    mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    ln = math.hypot(dx, dy)
    if dx == 0:
        s = (L.LANE_WIDTH, ln, 0.006)
    elif dy == 0:
        s = (ln, L.LANE_WIDTH, 0.006)
    else:                                       # 사선 구간은 아직 없음(있으면 회전 필요)
        s = (abs(dx) or L.LANE_WIDTH, abs(dy) or L.LANE_WIDTH, 0.006)
    M.box(stage, "/World/Site/Marking/lane_%02d" % i, (mx, my, 0.003), s, L.C_YELLOW)
report["counts"]["lane_segments"] = len(L.LANE_SEGMENTS)

# ============================================================ 3. equipment 레이어
wl.set_edit_layer(stage, "equipment")
stage.OverridePrim("/World")
UsdGeom.Xform.Define(stage, "/World/Cell")
UsdGeom.Scope.Define(stage, "/World/Cell/Equip")
UsdGeom.Scope.Define(stage, "/World/Cell/Fence")

for s in L.STATIONS:
    path = "/World/Cell/Equip/" + s["id"]
    M.build_station(stage, path, s)
    x0, x1, y0, y1 = L.station_bbox(s)
    report["stations"].append({"id": s["id"], "cls": s["cls"], "name": s["name"],
                               "center": list(s["center"]), "size": list(s["size"]),
                               "yaw": s.get("yaw", 0.0),
                               "bbox_xy": [round(v, 2) for v in (x0, y0, x1, y1)]})
report["counts"]["stations"] = len(L.STATIONS)

# 슈트 데크에 저마찰 머티리얼 바인딩 (판재가 실제로 흘러내리도록)
slide_path = "/World/Cell/Equip/SLIDE"
chute_mat = UsdShade.Material.Get(stage, CHUTE_MAT)
n_chute = 0
for child in stage.GetPrimAtPath(slide_path).GetChildren():
    if child.GetName().startswith(("chute_", "land_")):
        UsdShade.MaterialBindingAPI.Apply(child).Bind(
            chute_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        n_chute += 1
report["counts"]["chute_lowfriction_bindings"] = n_chute

# --- 안전 펜스: 선분을 2.0 m 패널로 분할 ------------------------------------------
PANEL = 2.0
n_panel = 0
for seg in L.FENCE_SEGMENTS:
    (ax, ay), (bx, by) = seg["a"], seg["b"]
    ln = math.hypot(bx - ax, by - ay)
    yaw = math.degrees(math.atan2(by - ay, bx - ax))
    n = max(1, int(round(ln / PANEL)))
    plen = ln / n
    for k in range(n):
        t = (k + 0.5) / n
        px, py = ax + (bx - ax) * t, ay + (by - ay) * t
        p = "/World/Cell/Fence/%s_p%d" % (seg["id"], k)
        xf = UsdGeom.Xform.Define(stage, p)
        xf.AddTranslateOp().Set(Gf.Vec3d(px, py, 0.0))
        xf.AddRotateZOp().Set(yaw)
        xf.GetPrim().SetCustomDataByKey("segment", seg["id"])
        M.build_fence_panel(stage, p, plen, seg["h"])
        n_panel += 1
report["counts"]["fence_panels"] = n_panel

# ============================================================ 4. products 레이어
wl.set_edit_layer(stage, "products")
stage.OverridePrim("/World")
stage.OverridePrim("/World/Cell")
UsdGeom.Scope.Define(stage, "/World/Cell/Products")
PROD = "/World/Cell/Products"

grip_mat = UsdShade.Material.Get(stage, GRIP_MAT)


def _bind_grip(prim):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        grip_mat, UsdShade.Tokens.weakerThanDescendants, "physics")


boxes = []          # (path, role, station, slot)


def put_box(name, center, role, station, slot, color=None):
    p = "%s/%s" % (PROD, name)
    prim = M.build_container(stage, p, center, color=color)
    prim.SetCustomDataByKey("role", role)
    prim.SetCustomDataByKey("station", station)
    prim.SetCustomDataByKey("slot", slot)
    _bind_grip(prim)
    boxes.append((p, role, station, slot))
    return p


# 분류 테이블 3칸: 제품#1 / 제품#2 / 프레스 대기
st = L.STATION_BY_ID["SORT_TABLE"]
for i, (bx, by, bz) in enumerate(L.table_slot_points("SORT_TABLE")):
    put_box("BOX_SORT_%d" % i, (bx, by, bz), "sort_bin", "SORT_TABLE",
            st["slot_labels"][i])

# Press 완료 적재대 2칸
for i, (bx, by, bz) in enumerate(L.table_slot_points("PRESS_OUT")):
    put_box("BOX_PRESSOUT_%d" % i, (bx, by, bz), "press_out_bin", "PRESS_OUT",
            "Press 완료 %d" % (i + 1))

# 빈 박스 랙 3단
rk = L.STATION_BY_ID["EMPTY_RACK"]
for i, z in enumerate(rk["shelves"]):
    put_box("BOX_EMPTY_%d" % i, (rk["center"][0], rk["center"][1], z + 0.018),
            "empty_bin", "EMPTY_RACK", "shelf_%d" % i)

# AMR Port 3개
for pid in ("AMR_PORT_P1", "AMR_PORT_P2", "AMR_PORT_PR"):
    ps = L.STATION_BY_ID[pid]
    put_box("BOX_%s" % pid, (ps["center"][0], ps["center"][1], ps["size"][2]),
            "amr_bin", pid, ps["name"])
report["counts"]["containers"] = len(boxes)

# --- 슬라이드 위 판재: 레인별 랜딩 1 + 슈트 3 -------------------------------------
sl = L.STATION_BY_ID["SLIDE"]
sw, s_run_total, s_drop = sl["size"]
land = sl["landing_len"]
run = s_run_total - land
theta = math.degrees(math.atan2(s_drop, run))
lane_w = sl["lane_w"]
lane_x = [(-(sl["lanes"] - 1) / 2.0 + i) * (lane_w + 0.06) for i in range(sl["lanes"])]
n_plate = 0
for i, lx in enumerate(lane_x):
    wx = sl["center"][0] + lx
    # 랜딩(픽 대기 위치)
    M.build_plate(stage, "%s/PLATE_L%d_PICK" % (PROD, i),
                  (wx, sl["center"][1] + land / 2, sl["deck_z"] + L.PLATE["t"]),
                  yaw=0.0, pitch=0.0)
    n_plate += 1
    for k, f in enumerate((0.25, 0.55, 0.85)):
        ly = land + run * f
        lz = sl["deck_z"] + s_drop * f + L.PLATE["t"]
        M.build_plate(stage, "%s/PLATE_L%d_Q%d" % (PROD, i, k),
                      (wx, sl["center"][1] + ly, lz), yaw=0.0, pitch=theta)
        n_plate += 1

# --- 박스 안 적재 판재 (제품#1/#2 진행중, Press 완료 1박스) --------------------------
for bname, cnt in (("BOX_SORT_0", 3), ("BOX_SORT_1", 2), ("BOX_PRESSOUT_0", 2)):
    bp = "%s/%s" % (PROD, bname)
    bprim = stage.GetPrimAtPath(bp)
    t = UsdGeom.Xformable(bprim).GetOrderedXformOps()[0].Get()
    for k in range(cnt):
        M.build_plate(stage, "%s/%s_part_%d" % (PROD, bname, k),
                      (t[0], t[1], t[2] - L.BOX["h"] / 2 + 0.020 + k * 0.008))
        n_plate += 1
# --- 겐트리 진공 리프터가 물고 있는 판재 (정적 콜라이더 — 낙하 금지) ---------------
gt = L.STATION_BY_ID["GANTRY"]
held = M.build_plate(stage, "%s/PLATE_GANTRY_HELD" % PROD,
                     (gt["center"][0] + gt["carriage_x"], gt["center"][1],
                      gt["head_z"] - 0.03 - L.PLATE["t"] / 2.0),
                     dynamic=False)
held.SetCustomDataByKey("role", "gantry_held")
held.SetCustomDataByKey("station", "GANTRY")
n_plate += 1
report["counts"]["plates"] = n_plate
report["gantry"] = {k: (list(v) if isinstance(v, tuple) else v)
                    for k, v in L.gantry_points().items()}

# ============================================================ 5. labels 레이어
wl.set_edit_layer(stage, "labels")
stage.OverridePrim("/World")
stage.OverridePrim("/World/Cell")
UsdGeom.Scope.Define(stage, "/World/Cell/Labels")

for co in L.CALLOUTS:
    a = L.STATION_BY_ID[co["anchor"]]
    p = "/World/Cell/Labels/" + co["id"]
    xf = UsdGeom.Xform.Define(stage, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(a["center"][0], a["center"][1],
                                     a["size"][2] + 1.10))
    pr = xf.GetPrim()
    pr.SetCustomDataByKey("title", co["title"])
    pr.SetCustomDataByKey("body", co["body"])
    pr.SetCustomDataByKey("anchor", co["anchor"])
    pr.SetCustomDataByKey("source", "KETI 컨셉(안) 캡처 판독 — 원본 슬라이드로 검증 필요")

UsdGeom.Scope.Define(stage, "/World/Cell/Labels/StandSpots")
report["reach"] = []
for sp in L.STAND_SPOTS:
    p = "/World/Cell/Labels/StandSpots/" + sp["id"]
    xf = UsdGeom.Xform.Define(stage, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(sp["xy"][0], sp["xy"][1], L.FLOOR_Z))
    xf.AddRotateZOp().Set(sp["yaw"])
    pr = xf.GetPrim()
    pr.SetCustomDataByKey("desc", sp["desc"])
    clr = round(L.clearance(*sp["xy"]), 3)
    pr.SetCustomDataByKey("clearance_m", clr)
    tgt = L.REACH_TARGETS.get(sp["id"])
    d = None
    if tgt:
        d = round(math.hypot(tgt[0] - sp["xy"][0], tgt[1] - sp["xy"][1]), 3)
        pr.SetCustomDataByKey("target", Gf.Vec3d(*[float(v) for v in tgt]))
        pr.SetCustomDataByKey("reach_m", d)
        pr.SetCustomDataByKey("reach_ok", bool(d <= L.REACH_MAX))
    report["reach"].append({"id": sp["id"], "reach_m": d, "clearance_m": clr,
                            "ok": (d is not None and d <= L.REACH_MAX)})

flow = UsdGeom.Scope.Define(stage, "/World/Cell/Labels/TaskFlow").GetPrim()
flow.SetCustomDataByKey("steps", json.dumps(
    [{"id": a, "from": b, "to": c, "desc": d} for a, b, c, d in L.TASK_FLOW],
    ensure_ascii=False))
report["counts"]["callouts"] = len(L.CALLOUTS)
report["counts"]["stand_spots"] = len(L.STAND_SPOTS)

# ============================================================ 6. robots 레이어
wl.set_edit_layer(stage, "robots")
stage.OverridePrim("/World")
UsdGeom.Xform.Define(stage, "/World/Robots")

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                                UsdGeom.Tokens.proxy])
report["robots"] = []
for r in L.ROBOTS:
    p = "/World/Robots/" + r["name"]
    prim = UsdGeom.Xform.Define(stage, p).GetPrim()
    ref = r["ref"] if os.path.exists(r["ref"]) else L.H1_CLEAN_USD
    if ref != r["ref"]:
        report["notes"].append("%s: %s missing -> fell back to h1_clean" % (r["name"], r["ref"]))
    ok = prim.GetReferences().AddReference(ref)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp(); xf.AddRotateZOp().Set(r["yaw"])
    t_op.Set(Gf.Vec3d(r["stand"][0], r["stand"][1], L.FLOOR_Z))
    dz = 0.0
    try:                              # 발바닥을 슬래브 top 에 정렬
        bbox_cache.Clear()
        rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not rng.IsEmpty():
            dz = L.FLOOR_Z - rng.GetMin()[2]
            t_op.Set(Gf.Vec3d(r["stand"][0], r["stand"][1], dz))
    except Exception as e:
        report["notes"].append("%s bbox failed: %s" % (r["name"], e))
    prim.SetCustomDataByKey("role", r["role"])
    prim.SetCustomDataByKey("stand_xy", Gf.Vec2d(float(r["stand"][0]), float(r["stand"][1])))
    report["robots"].append({"name": r["name"], "ref": ref, "ok": bool(ok),
                             "xy": list(r["stand"]), "yaw": r["yaw"], "z_offset": round(dz, 4),
                             "clearance_m": round(L.clearance(*r["stand"]), 3)})

# ============================================================ 7. staging 레이어
wl.set_edit_layer(stage, "staging")
stage.OverridePrim("/World")
sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
sun.CreateIntensityAttr(1600.0); sun.CreateAngleAttr(0.53)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 28.0))
sky = UsdLux.DomeLight.Define(stage, "/World/Lighting/Sky")
sky.CreateIntensityAttr(300.0)
for i, (lx, ly) in enumerate(((5.5, 5.0), (5.5, 11.0), (16.5, 5.0), (16.5, 11.0))):
    rl = UsdLux.RectLight.Define(stage, "/World/Lighting/Bay_%d" % i)
    rl.CreateWidthAttr(6.0); rl.CreateHeightAttr(4.0)
    rl.CreateIntensityAttr(2600.0); rl.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.94))
    lxf = UsdGeom.Xformable(rl)
    lxf.AddTranslateOp().Set(Gf.Vec3d(lx, ly, BAY["h"] - 0.6))
    lxf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))       # 아래로 조사


def nrm(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return Gf.Vec3d(v[0] / n, v[1] / n, v[2] / n)


def crs(a, b):
    return Gf.Vec3d(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0])


def make_cam(path, eye, target, focal, up=(0, 0, 1)):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * 1080.0 / 1920.0)
    d = math.dist(eye, target)
    cam.CreateFocusDistanceAttr(d)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 3000.0))
    cz = nrm((eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]))
    upv = Gf.Vec3d(*up)
    if abs(cz[0] * upv[0] + cz[1] * upv[1] + cz[2] * upv[2]) > 0.999:
        upv = Gf.Vec3d(0, 1, 0)
    cx_ = nrm(crs(upv, cz)); cy_ = crs(cz, cx_)
    mtx = Gf.Matrix4d(1.0)
    mtx.SetRow(0, Gf.Vec4d(cx_[0], cx_[1], cx_[2], 0.0))
    mtx.SetRow(1, Gf.Vec4d(cy_[0], cy_[1], cy_[2], 0.0))
    mtx.SetRow(2, Gf.Vec4d(cz[0], cz[1], cz[2], 0.0))
    mtx.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
    cxf = UsdGeom.Xformable(cam); cxf.ClearXformOpOrder(); cxf.AddTransformOp().Set(mtx)
    return cam


r1 = L.ROBOTS[0]["stand"]; r2 = L.ROBOTS[1]["stand"]
CAMS = [
    # 개념도와 같은 프레이밍: 전면 상공에서 셀 전체(레이저~AMR)를 한 화면에
    ("/World/ConceptCam", (10.5, -11.5, 10.0), (11.0, 7.5, 0.8), 16.0, (0, 0, 1)),
    ("/World/TopCam",     (11.0, 7.5, 26.0), (11.0, 7.5, 0.0), 24.0, (0, 1, 0)),
    ("/World/R1Cam",      (r1[0] - 2.6, r1[1] - 2.6, 2.6), (r1[0], r1[1] + 0.9, 1.1), 28.0, (0, 0, 1)),
    ("/World/R2Cam",      (r2[0] - 3.0, r2[1] - 3.0, 2.8), (r2[0], r2[1] + 1.4, 1.3), 28.0, (0, 0, 1)),
]
for path, eye, tgt, f, up in CAMS:
    make_cam(path, eye, tgt, f, up)
report["cameras"] = [c[0] for c in CAMS]

# ============================================================ 8. 저장
saved = wl.save(stage)
wl.stamp_root_axis(stage)

report["root"] = wl.ROOT
report["layers"] = {k: os.path.getsize(wl.layer_path(k)) for k in wl.LAYER_KEYS}
report["saved"] = [os.path.basename(s) for s in saved]
with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)

print("=== SMIC World built ===")
print("  root:", os.path.basename(wl.ROOT), os.path.getsize(wl.ROOT), "bytes (thin)")
for k in wl.LAYER_KEYS:
    print("   %-12s %9d bytes  %s" % (k, os.path.getsize(wl.layer_path(k)),
                                      os.path.basename(wl.layer_path(k))))
print("  stations=%d  fence_panels=%d  containers=%d  plates=%d"
      % (report["counts"]["stations"], report["counts"]["fence_panels"],
         report["counts"]["containers"], report["counts"]["plates"]))
for rb in report["robots"]:
    print("   robot %-8s xy=%s yaw=%.0f  z=%.3f  clearance=%.2f m  ref_ok=%s"
          % (rb["name"], rb["xy"], rb["yaw"], rb["z_offset"], rb["clearance_m"], rb["ok"]))
bad = [r for r in report["reach"] if not r["ok"]]
print("  reach (H1 base->target, limit %.2f m): %d/%d OK"
      % (L.REACH_MAX, len(report["reach"]) - len(bad), len(report["reach"])))
for r in bad:
    print("   REACH FAIL %-12s %.3f m" % (r["id"], r["reach_m"] or -1))
for n in report["notes"]:
    print("   NOTE:", n)
print("  report:", os.path.basename(REPORT))
