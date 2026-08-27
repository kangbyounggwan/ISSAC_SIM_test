#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_smic_obj.py — SMIC World 합성 스테이지를 **OBJ + MTL** 로 내보낸다(Blender 용).

  C:\\isaacsim\\python.bat export_smic_obj.py

순수 pxr — Kit/GPU 불필요(~2 s). 이 월드는 대부분 `UsdGeom.Cube`/`Cylinder` 같은
**암시적 프리미티브**라 OBJ 로 나가려면 테셀레이션이 필요하다. 이 스크립트가
Cube(6면) / Cylinder(N각 기둥 + 캡) / Mesh(그대로)를 월드 변환 적용해 폴리곤으로 굽는다.

출력  export/smic_world.obj  +  smic_world.mtl

환경변수
  SMIC_OBJ_ZUP=1        Z-up 원본 좌표 그대로 (기본은 Y-up 변환 — 아래 참조)
  SMIC_OBJ_NO_ROBOTS=1  /World/Robots 제외 (H1 메시가 무거울 때)
  SMIC_OBJ_SEG=24       실린더 분할 수
  SMIC_OBJ_OUT=<path>   출력 .obj 경로

축(중요)
  USD 월드는 **Z-up**, OBJ 관행은 **Y-up**. 기본값은 Y-up 으로 변환해 내보내므로
  Blender 에서 **기본 설정 그대로**(Forward −Z / Up Y) 임포트하면 바로 선다.
  USD 좌표와 숫자를 맞춰야 하면 SMIC_OBJ_ZUP=1 로 뽑고 Blender 임포트 설정을
  Forward=Y / Up=Z 로 바꾼다.

한계 (OBJ 포맷 자체의 한계)
  - 카메라·조명·물리·프림 계층/메타데이터는 **안 나간다**. 그게 필요하면
    Blender 의 USD 임포터로 `smic_world.usd` 를 직접 여는 편이 낫다(서브레이어 자동 합성).
  - 색은 displayColor → MTL `Kd` 로만 옮긴다. 텍스처/PBR 없음.
  - 숨김 프림(지붕)과 guide purpose 는 제외한다.
"""
import os, sys, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401
import world_layers as wl
from pxr import Usd, UsdGeom, Gf

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_OBJ = os.environ.get("SMIC_OBJ_OUT") or os.path.join(HERE, "export", "smic_world.obj")
ZUP = bool(os.environ.get("SMIC_OBJ_ZUP"))
NO_ROBOTS = bool(os.environ.get("SMIC_OBJ_NO_ROBOTS"))
SEG = int(os.environ.get("SMIC_OBJ_SEG") or 24)

CUBE_FACES = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
              (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
CUBE_CORNERS = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]


def cube_geom(prim):
    size = UsdGeom.Cube(prim).GetSizeAttr().Get()
    h = (2.0 if size is None else float(size)) / 2.0
    return [(c[0] * h, c[1] * h, c[2] * h) for c in CUBE_CORNERS], list(CUBE_FACES)


def cylinder_geom(prim, seg=SEG):
    c = UsdGeom.Cylinder(prim)
    r = float(c.GetRadiusAttr().Get() or 1.0)
    ht = float(c.GetHeightAttr().Get() or 2.0)
    axis = (c.GetAxisAttr().Get() or "Z").upper()
    h2 = ht / 2.0
    lo, hi = [], []
    for i in range(seg):
        a = 2.0 * math.pi * i / seg
        u, v = r * math.cos(a), r * math.sin(a)
        if axis == "Z":
            lo.append((u, v, -h2)); hi.append((u, v, h2))
        elif axis == "Y":
            lo.append((u, -h2, v)); hi.append((u, h2, v))
        else:                                    # X
            lo.append((-h2, u, v)); hi.append((h2, u, v))
    pts = lo + hi
    faces = [(i, (i + 1) % seg, seg + (i + 1) % seg, seg + i) for i in range(seg)]
    faces.append(tuple(range(seg - 1, -1, -1)))  # 하단 캡
    faces.append(tuple(range(seg, 2 * seg)))     # 상단 캡
    return pts, faces


def sphere_geom(prim, seg=SEG):
    r = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0)
    nlat = max(3, seg // 2)
    pts, faces = [], []
    for j in range(1, nlat):                       # 극점 제외한 위도 링
        phi = math.pi * j / nlat
        z, rr = r * math.cos(phi), r * math.sin(phi)
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            pts.append((rr * math.cos(a), rr * math.sin(a), z))
    top = len(pts); pts.append((0, 0, r))
    bot = len(pts); pts.append((0, 0, -r))
    for j in range(nlat - 2):
        for i in range(seg):
            a0 = j * seg + i;            a1 = j * seg + (i + 1) % seg
            b0 = (j + 1) * seg + i;      b1 = (j + 1) * seg + (i + 1) % seg
            faces.append((a0, a1, b1, b0))
    for i in range(seg):                           # 극 캡
        faces.append((top, (i + 1) % seg, i))
        faces.append((bot, (nlat - 2) * seg + i, (nlat - 2) * seg + (i + 1) % seg))
    return pts, faces


def mesh_geom(prim):
    m = UsdGeom.Mesh(prim)
    pts = m.GetPointsAttr().Get()
    counts = m.GetFaceVertexCountsAttr().Get()
    idx = m.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or not idx:
        return None, None
    faces, k = [], 0
    for n in counts:
        faces.append(tuple(idx[k:k + n]))
        k += n
    return [(p[0], p[1], p[2]) for p in pts], faces


def display_color(prim):
    """displayColor 를 프림 → 조상 순으로 찾는다(없으면 기본 회색)."""
    p = prim
    while p and p.IsValid() and str(p.GetPath()) != "/":
        g = UsdGeom.Gprim(p)
        if g:
            a = g.GetDisplayColorAttr()
            v = a.Get() if a else None
            if v is not None and len(v) > 0:
                return (round(float(v[0][0]), 4), round(float(v[0][1]), 4), round(float(v[0][2]), 4))
        p = p.GetParent()
    return (0.62, 0.63, 0.65)


# ------------------------------------------------------------------ 수집
if not os.path.exists(wl.ROOT):
    sys.exit("[obj] %s missing — run build_smic_world.py first" % wl.ROOT)

stage = Usd.Stage.Open(wl.ROOT)
xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
pred = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)

vlines, flines = [], []
mats = {}
voff = 1
stats = {"Cube": 0, "Cylinder": 0, "Sphere": 0, "Mesh": 0, "skipped_invisible": 0,
         "skipped_guide": 0, "skipped_other_type": {}}


def mat_name(col):
    if col not in mats:
        mats[col] = "m%02d" % len(mats)
    return mats[col]


for prim in stage.Traverse(pred):
    path = str(prim.GetPath())
    if NO_ROBOTS and path.startswith("/World/Robots"):
        continue
    tn = prim.GetTypeName()
    if tn not in ("Cube", "Cylinder", "Sphere", "Mesh"):
        if tn and tn not in ("Xform", "Scope", "Camera", "PhysicsScene", "Material",
                             "DistantLight", "DomeLight", "RectLight", "SphereLight"):
            stats["skipped_other_type"][tn] = stats["skipped_other_type"].get(tn, 0) + 1
        continue

    img = UsdGeom.Imageable(prim)
    if img.ComputeVisibility() == UsdGeom.Tokens.invisible:
        stats["skipped_invisible"] += 1
        continue
    if img.ComputePurpose() == UsdGeom.Tokens.guide:
        stats["skipped_guide"] += 1
        continue

    if tn == "Cube":
        pts, faces = cube_geom(prim)
    elif tn == "Cylinder":
        pts, faces = cylinder_geom(prim)
    elif tn == "Sphere":
        pts, faces = sphere_geom(prim)
    else:
        pts, faces = mesh_geom(prim)
        if pts is None:
            continue
    stats[tn] += 1

    M = xcache.GetLocalToWorldTransform(prim)
    flip = M.GetDeterminant() < 0        # 미러 변환이면 와인딩 뒤집기
    for p in pts:
        w = M.Transform(Gf.Vec3d(*p))
        # USD Z-up -> OBJ Y-up:  (x, y, z) -> (x, z, -y)
        vlines.append("v %.5f %.5f %.5f"
                      % ((w[0], w[1], w[2]) if ZUP else (w[0], w[2], -w[1])))

    # `o` 와 `g` 를 둘 다 쓴다. Blender OBJ 임포터의 기본값은
    # use_split_objects=True / use_split_groups=False 라 `g` 만 있으면 전체가
    # **한 덩어리**로 들어온다. `o` 를 같이 써야 기본 설정에서 프림별로 쪼개진다.
    oname = path.strip("/").replace("/", "_")
    flines.append("usemtl " + mat_name(display_color(prim)))
    flines.append("o " + oname)
    flines.append("g " + oname)
    for f in faces:
        seq = tuple(reversed(f)) if flip else f
        flines.append("f " + " ".join(str(voff + i) for i in seq))
    voff += len(pts)

# ------------------------------------------------------------------ 기록
os.makedirs(os.path.dirname(OUT_OBJ), exist_ok=True)
mtl_path = OUT_OBJ[:-4] + ".mtl"
with open(OUT_OBJ, "w", encoding="utf-8") as fo:
    fo.write("# SMIC World  (exported from %s)\n" % os.path.basename(wl.ROOT))
    fo.write("# axis: %s   units: meters\n" % ("Z-up (USD native)" if ZUP else "Y-up (OBJ convention)"))
    fo.write("mtllib %s\n" % os.path.basename(mtl_path))
    fo.write("\n".join(vlines))
    fo.write("\n")
    fo.write("\n".join(flines))
    fo.write("\n")
with open(mtl_path, "w", encoding="utf-8") as fm:
    fm.write("# SMIC World materials (from USD displayColor)\n")
    for c, name in mats.items():
        r, g, b = c
        fm.write("newmtl %s\nKd %.4f %.4f %.4f\nKa %.4f %.4f %.4f\nKs 0.0 0.0 0.0\n"
                 "Ns 10.0\nd 1.0\nillum 2\n\n"
                 % (name, r, g, b, r * 0.25, g * 0.25, b * 0.25))

print("=== SMIC World -> OBJ ===")
print("  obj : %s  (%.1f MB)" % (OUT_OBJ, os.path.getsize(OUT_OBJ) / 1e6))
print("  mtl : %s  (materials=%d)" % (mtl_path, len(mats)))
print("  verts=%d  Cube=%d  Cylinder=%d  Sphere=%d  Mesh=%d"
      % (voff - 1, stats["Cube"], stats["Cylinder"], stats["Sphere"], stats["Mesh"]))
print("  skipped: invisible=%d guide=%d" % (stats["skipped_invisible"], stats["skipped_guide"]))
if stats["skipped_other_type"]:
    print("  skipped types:", stats["skipped_other_type"])
print("  axis: %s" % ("Z-up (Blender 임포트를 Forward=Y / Up=Z 로)"
                      if ZUP else "Y-up (Blender 기본 설정 그대로 임포트)"))
