#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_smic_world.py — SMIC World 합성/기하/리치 검증. 순수 USD, Kit 불필요.

  C:\\isaacsim\\python.bat verify_smic_world.py

검사 항목
  1) 루트가 thin 인가 (로컬 콘텐츠 프림 0), 서브레이어 6개 합성, defaultPrim 해석
  2) upAxis=Z / metersPerUnit=1 (JM 함정: 루트에 없으면 GUI 가 Y-up 으로 읽는다)
  3) 설비 스테이션 bbox 상호 침범
  4) 작업 스팟: 리치 ≤ REACH_MAX, 설비 클리어런스 ≥ MIN_CLEAR
  5) 제품(컨테이너/판재): 바닥 아래로 내려간 것/베이 밖으로 나간 것
  6) 로봇 reference 해석 + 발바닥이 슬래브 top 에 있는가
  7) 동적 강체에 콜라이더가 붙어 있는가
실패는 FAIL 로 집계되고 exit code 1.
"""
import os, sys, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401
import world_layers as wl
import smic_layout as L
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

MIN_CLEAR = 0.15          # 로봇 발 기준 설비까지 최소 여유
FOOT_TOL = 0.02

fails, warns = [], []


def check(cond, msg):
    if cond:
        print("   [ok]   " + msg)
    else:
        print("   [FAIL] " + msg)
        fails.append(msg)


def warn(cond, msg):
    if not cond:
        print("   [warn] " + msg)
        warns.append(msg)


if not os.path.exists(wl.ROOT):
    sys.exit("[verify] %s missing — run build_smic_world.py first" % wl.ROOT)

stage = Usd.Stage.Open(wl.ROOT)
rl = stage.GetRootLayer()

# ---------------------------------------------------------------- 1) 합성
print("=== 1. composition ===")
print("   subLayers:", list(rl.subLayerPaths))
check(len(rl.subLayerPaths) == len(wl.SUBLAYERS),
      "subLayer count = %d (expected %d)" % (len(rl.subLayerPaths), len(wl.SUBLAYERS)))
local = [p.name for p in rl.rootPrims]
check(not local, "root-local content prims empty (found: %s)" % (local or "none"))
try:
    errs = stage.GetCompositionErrors()
except Exception:
    errs = []
check(not errs, "no composition errors (%d)" % len(errs))
dp = stage.GetDefaultPrim()
check(bool(dp and dp.IsValid()), "defaultPrim resolves: %s" % (dp.GetPath() if dp else None))

# ---------------------------------------------------------------- 2) 단위/축
print("=== 2. stage axis / units ===")
check(UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z,
      "upAxis = %s" % UsdGeom.GetStageUpAxis(stage))
check(abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9,
      "metersPerUnit = %s" % UsdGeom.GetStageMetersPerUnit(stage))
check(bool(stage.GetPrimAtPath("/World/PhysicsScene")), "/World/PhysicsScene present")

# ---------------------------------------------------------------- 3) 설비 침범
print("=== 3. station overlap (접지 풋프린트 기준) ===")
n_ov = 0
for i in range(len(L.STATIONS)):
    for j in range(i + 1, len(L.STATIONS)):
        a, b = L.STATIONS[i], L.STATIONS[j]
        for (ax0, ax1, ay0, ay1) in L.station_ground_boxes(a):
            for (bx0, bx1, by0, by1) in L.station_ground_boxes(b):
                ox = min(ax1, bx1) - max(ax0, bx0)
                oy = min(ay1, by1) - max(ay0, by0)
                if ox > 1e-6 and oy > 1e-6:
                    n_ov += 1
                    print("   [FAIL] %s <-> %s overlap %.2f x %.2f m"
                          % (a["id"], b["id"], ox, oy))
check(n_ov == 0, "no station footprint overlap (%d pairs checked)"
      % (len(L.STATIONS) * (len(L.STATIONS) - 1) // 2))

# 겐트리는 다른 설비를 타넘는 구조물 — 브리지가 그 아래 설비 **실제 최고점**을
# 비켜가는지. size[2] 는 못 쓴다(슬라이드의 size[2] 는 높이가 아니라 낙차).
_bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
g = L.STATION_BY_ID.get("GANTRY")
if g:
    gx0, gx1, gy0, gy1 = L.station_bbox(g)
    under = []
    for s in L.STATIONS:
        if s["id"] == "GANTRY":
            continue
        sx0, sx1, sy0, sy1 = L.station_bbox(s)
        if not (min(gx1, sx1) > max(gx0, sx0) and min(gy1, sy1) > max(gy0, sy0)):
            continue
        pr = stage.GetPrimAtPath("/World/Cell/Equip/" + s["id"])
        rng = _bc.ComputeWorldBound(pr).ComputeAlignedRange() if pr else None
        top = rng.GetMax()[2] if (rng and not rng.IsEmpty()) else s["size"][2]
        under.append((s["id"], top))
    tallest = max([h for _, h in under], default=0.0)
    gap = g["size"][2] - tallest
    print("   under bridge: %s" % ", ".join("%s(top=%.2f)" % u for u in under))
    check(gap >= 0.50, "bridge underside %.2f m vs tallest straddled top %.2f m -> gap %.2f m"
          % (g["size"][2], tallest, gap))

# 펜스 라인 vs 설비 접지부 (겐트리 다리처럼 통로에 서는 구조물이 펜스를 뚫지 않는지)
FENCE_T = 0.10
n_fov = 0
for seg in L.FENCE_SEGMENTS:
    (ax, ay), (bx, by) = seg["a"], seg["b"]
    fx0, fx1 = min(ax, bx) - FENCE_T / 2, max(ax, bx) + FENCE_T / 2
    fy0, fy1 = min(ay, by) - FENCE_T / 2, max(ay, by) + FENCE_T / 2
    for s in L.STATIONS:
        for (sx0, sx1, sy0, sy1) in L.station_ground_boxes(s):
            ox = min(fx1, sx1) - max(fx0, sx0)
            oy = min(fy1, sy1) - max(fy0, sy0)
            if ox > 1e-6 and oy > 1e-6:
                n_fov += 1
                print("   [FAIL] fence %s <-> %s overlap %.2f x %.2f m"
                      % (seg["id"], s["id"], ox, oy))
check(n_fov == 0, "no fence/equipment footprint clash (%d segments)" % len(L.FENCE_SEGMENTS))

# ---------------------------------------------------------------- 4) 리치/클리어런스
print("=== 4. stand spots (reach <= %.2f m, clearance >= %.2f m) ===" % (L.REACH_MAX, MIN_CLEAR))
for sp in L.STAND_SPOTS:
    tgt = L.REACH_TARGETS.get(sp["id"])
    d = math.hypot(tgt[0] - sp["xy"][0], tgt[1] - sp["xy"][1]) if tgt else None
    clr = L.clearance(*sp["xy"])
    ok = (d is not None and d <= L.REACH_MAX + 1e-6)
    print("   %-12s reach=%s  clearance=%.2f  %s"
          % (sp["id"], ("%.3f" % d) if d is not None else "  n/a", clr,
             "ok" if ok else "REACH FAIL"))
    if not ok:
        fails.append("reach %s = %s" % (sp["id"], d))
    warn(clr >= MIN_CLEAR, "%s clearance %.2f m < %.2f (설비에 너무 붙음)" % (sp["id"], clr, MIN_CLEAR))

# ---------------------------------------------------------------- 5) 제품 위치
print("=== 5. products ===")
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
prod = stage.GetPrimAtPath("/World/Cell/Products")
check(bool(prod and prod.IsValid()), "/World/Cell/Products present")
n_prod = n_below = n_outside = 0
if prod and prod.IsValid():
    for p in prod.GetChildren():
        rng = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        n_prod += 1
        mn, mx = rng.GetMin(), rng.GetMax()
        if mn[2] < -1e-3:
            n_below += 1
            print("   [FAIL] %s below floor (min z=%.3f)" % (p.GetName(), mn[2]))
        if not (L.BAY["x0"] - 1 <= mn[0] and mx[0] <= L.BAY["x1"] + 1
                and L.BAY["y0"] - 1 <= mn[1] and mx[1] <= L.BAY["y1"] + 1):
            n_outside += 1
            print("   [FAIL] %s outside bay" % p.GetName())
check(n_below == 0, "no product below floor (%d products)" % n_prod)
check(n_outside == 0, "no product outside bay")

# ---------------------------------------------------------------- 6) 로봇
print("=== 6. robots ===")
for r in L.ROBOTS:
    p = stage.GetPrimAtPath("/World/Robots/" + r["name"])
    if not (p and p.IsValid()):
        check(False, "%s prim missing" % r["name"])
        continue
    has_ref = bool(p.GetPrimStack() and any(s.referenceList.prependedItems or s.referenceList.explicitItems
                                            for s in p.GetPrimStack()))
    check(has_ref, "%s has a reference" % r["name"])
    cache.Clear()
    rng = cache.ComputeWorldBound(p).ComputeAlignedRange()
    if rng.IsEmpty():
        check(False, "%s composed bbox empty (reference did not resolve?)" % r["name"])
        continue
    mn, mx = rng.GetMin(), rng.GetMax()
    check(abs(mn[2] - L.FLOOR_Z) <= FOOT_TOL,
          "%s feet on slab (min z=%.3f, head z=%.2f)" % (r["name"], mn[2], mx[2]))
    print("        stand=%s yaw=%.0f  bbox=%.2f x %.2f x %.2f"
          % (list(r["stand"]), r["yaw"], mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]))

# ---------------------------------------------------------------- 7) 물리
print("=== 7. physics ===")
# 이 월드가 **직접 author 한** 프림만 검사한다. /World/Robots 아래는 외부 H1 에셋
# (reference)이라 이 프로젝트의 책임 범위가 아니다 — 별도로 경고만 낸다.
OWNED = ("/World/Cell", "/World/Site")


def _apis(p):
    return p.GetPrimTypeInfo().GetAppliedAPISchemas() or []


n_rb = n_rb_nocoll = n_coll = 0
for p in stage.Traverse():
    path = str(p.GetPath())
    if not any(path.startswith(o) for o in OWNED):
        continue
    if "PhysicsCollisionAPI" in _apis(p):
        n_coll += 1
    if "PhysicsRigidBodyAPI" in _apis(p):
        n_rb += 1
        if not any("PhysicsCollisionAPI" in _apis(q) for q in Usd.PrimRange(p)):
            n_rb_nocoll += 1
            print("   [FAIL] rigid body without collider: %s" % path)
check(n_rb_nocoll == 0,
      "every owned rigid body has a collider (rigid=%d, colliders=%d)" % (n_rb, n_coll))
check(n_rb > 0, "rigid bodies authored (%d)" % n_rb)

# 참조된 H1 에셋의 콜라이더 상태 — 파지/보행에 직접 영향이 있어 눈에 보이게 둔다.
for r in L.ROBOTS:
    rp = stage.GetPrimAtPath("/World/Robots/" + r["name"])
    if not (rp and rp.IsValid()):
        continue
    links = sum(1 for q in Usd.PrimRange(rp) if "PhysicsRigidBodyAPI" in _apis(q))
    cols = sum(1 for q in Usd.PrimRange(rp) if "PhysicsCollisionAPI" in _apis(q))
    print("   %s (external asset): links=%d colliders=%d" % (r["name"], links, cols))
    warn(cols > 0,
         "%s 에셋에 콜리전 프림이 없다 (h1_gripper.usda 의 */collisions 가 비어 있음). "
         "물리 파지/보행 전에 콜라이더 보강 필요 — 월드 문제 아님, 로봇 에셋 문제." % r["name"])

# ---------------------------------------------------------------- summary
print("\n=== summary ===")
print("   prims: %d   owned colliders: %d   owned rigid bodies: %d"
      % (sum(1 for _ in stage.Traverse()), n_coll, n_rb))
print("   warnings: %d   failures: %d" % (len(warns), len(fails)))
print("\n================", "PASS" if not fails else "FAIL", "================")
sys.exit(0 if not fails else 1)
