# -*- coding: utf-8 -*-
"""가짜 pxr 스텁으로 jm_factory_world.main() 로직 검증 (USD 미설치 환경용).
- 모든 .Define(stage,path) 경로 수집 -> prim 수 / 중복 경로 / 빌딩별 카운트 검증.
- 모든 Cube center/size 수집 -> 좌표 범위(extent) 검증.
"""
import sys, types

prims = []          # (typename, path)
cubes = []          # (path, center, size)
collisions = []     # path
dup = set()
_seen = set()


class Op:
    def Set(self, *a, **k):
        # 마지막 Define 된 cube 에 translate/scale 기록
        if a and _cur_cube[0] is not None:
            v = a[0]
            vals = tuple(getattr(v, "_v", v))
            if self._kind == "T":
                _cur_cube[1]["center"] = vals
            elif self._kind == "S":
                _cur_cube[1]["size"] = vals
        return self

    def __init__(self, kind="?"):
        self._kind = kind


class Attr:
    def Set(self, *a, **k):
        return self


class Prim:
    def __init__(self, path):
        self.path = path

    def CreateAttribute(self, *a, **k):
        return Attr()


_cur_cube = [None, None]  # (path, dict) 현재 처리중 cube


class Schema:
    def __init__(self, path, typ):
        self._prim = Prim(path)
        self._typ = typ

    def GetPrim(self):
        return self._prim

    def AddTranslateOp(self, *a, **k):
        return Op("T")

    def AddScaleOp(self, *a, **k):
        return Op("S")

    def AddRotateXYZOp(self, *a, **k):
        return Op("R")

    def __getattr__(self, name):
        # CreateXxxAttr(...) -> Attr ; 기타 -> 무해한 호출
        def f(*a, **k):
            return Attr()
        return f


def _define(typ):
    def define(stage, path, *a, **k):
        p = str(path)
        if p in _seen:
            dup.add(p)
        _seen.add(p)
        prims.append((typ, p))
        sch = Schema(p, typ)
        if typ == "Cube":
            d = {"center": None, "size": None}
            cubes.append((p, d))
            _cur_cube[0] = p
            _cur_cube[1] = d
        else:
            _cur_cube[0] = None
            _cur_cube[1] = None
        return sch
    return define


class Vec:
    def __init__(self, *v):
        self._v = tuple(v)


class FakeStage:
    def __init__(self, path):
        self.path = path

    def SetDefaultPrim(self, *a):
        pass

    def SetMetadata(self, *a):
        pass

    def GetMetadata(self, *a):
        return None

    def Traverse(self):
        return [Prim(p) for _, p in prims]

    def GetRootLayer(self):
        return types.SimpleNamespace(Save=lambda *a, **k: None)


def build_module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


# ---- pxr 패키지 구성 ----
pxr = types.ModuleType("pxr")

Tokens = types.SimpleNamespace(z="z", none="none", convexHull="convexHull")

UsdGeom = build_module("pxr.UsdGeom",
                       Tokens=Tokens,
                       SetStageUpAxis=lambda *a, **k: None,
                       SetStageMetersPerUnit=lambda *a, **k: None,
                       Xform=types.SimpleNamespace(Define=_define("Xform")),
                       Cube=types.SimpleNamespace(Define=_define("Cube")),
                       Mesh=types.SimpleNamespace(Define=_define("Mesh")),
                       Xformable=lambda prim: Schema(getattr(prim, "path", "?"), "Xformable"))

UsdPhysics = build_module(
    "pxr.UsdPhysics",
    Tokens=Tokens,
    SetStageKilogramsPerUnit=lambda *a, **k: None,
    Scene=types.SimpleNamespace(Define=_define("PhysicsScene")),
    CollisionAPI=types.SimpleNamespace(Apply=lambda prim: collisions.append(getattr(prim, "path", "?")) or Schema(getattr(prim, "path", "?"), "CollisionAPI")),
    MeshCollisionAPI=types.SimpleNamespace(Apply=lambda prim: Schema(getattr(prim, "path", "?"), "MeshCollisionAPI")),
)

UsdLux = build_module("pxr.UsdLux",
                      DistantLight=types.SimpleNamespace(Define=_define("DistantLight")),
                      DomeLight=types.SimpleNamespace(Define=_define("DomeLight")))

Usd = build_module("pxr.Usd",
                   Stage=types.SimpleNamespace(CreateNew=lambda path: FakeStage(path)),
                   GetVersion=lambda: (0, 0, 0))
Sdf = build_module("pxr.Sdf", ValueTypeNames=types.SimpleNamespace(String="string"))
Gf = build_module("pxr.Gf", Vec3d=Vec, Vec3f=Vec, Quatf=lambda *a: Vec(*a))

for nm, mod in [("pxr", pxr), ("pxr.Usd", Usd), ("pxr.UsdGeom", UsdGeom),
                ("pxr.UsdPhysics", UsdPhysics), ("pxr.UsdLux", UsdLux),
                ("pxr.Sdf", Sdf), ("pxr.Gf", Gf)]:
    sys.modules[nm] = mod
for sub in ("Usd", "UsdGeom", "UsdPhysics", "UsdLux", "Sdf", "Gf"):
    setattr(pxr, sub, sys.modules["pxr." + sub])

# ---- 실행 ----
import jm_factory_world as W
W.main("__fake__.usda")

# ---- 검증 리포트 ----
from collections import Counter
print("\n=== VALIDATION REPORT ===")
print("total prims :", len(prims))
print("by type     :", dict(Counter(t for t, _ in prims)))
print("duplicate paths:", sorted(dup) if dup else "NONE  ✓")
print("collision APIs :", len(collisions))


def under(pfx):
    return [p for _, p in prims if p.startswith(pfx)]


for b in ("/World/Dadong", "/World/Nadong", "/World/Gadong", "/World/Ground", "/World/Fence", "/World/Lighting"):
    print(f"  {b:22s}: {len(under(b))} prims")

# Cube 좌표 범위
xs, ys, zs = [], [], []
missing = 0
for p, d in cubes:
    c, s = d["center"], d["size"]
    if c is None or s is None:
        missing += 1
        continue
    for i, (cc, ss) in enumerate(zip(c, s)):
        lo, hi = cc - ss / 2, cc + ss / 2
        (xs, ys, zs)[i].extend([lo, hi])
print("\ncubes total :", len(cubes), " (center/size captured for all:", "YES ✓" if missing == 0 else f"MISSING {missing}", ")")
print("X range m   : %.1f .. %.1f" % (min(xs), max(xs)))
print("Y range m   : %.1f .. %.1f" % (min(ys), max(ys)))
print("Z range m   : %.1f .. %.1f" % (min(zs), max(zs)))

# 층 태그/경로 sanity (신 경로)
from collections import Counter as _Cn
fl = _Cn(p["floor"] for p in __import__("world_geometry").build_scene())
print("\nfloors:", dict(fl))
ncol = len([p for _, p in prims if "/Dadong/F1/col_" in p]) + len([p for _, p in prims if "/Dadong/F2/col_" in p])
print("Dadong columns(1F+2F):", ncol, "(expect 60+60=120)", "✓" if ncol == 120 else "✗")
gslab = len([p for _, p in prims if p.startswith("/World/Gadong/") and "/slab" in p])
print("Gadong slabs  :", gslab, "(expect 3 floor + 1 roof = 4)", "✓" if gslab == 4 else "✗")
print("OK - generator logic ran end-to-end with no exceptions.")
