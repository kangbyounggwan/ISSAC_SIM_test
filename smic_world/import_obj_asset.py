#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
import_obj_asset.py — 블렌더에서 손본 **OBJ 를 다시 SMIC World 로 편입**한다.

  C:\\isaacsim\\python.bat import_obj_asset.py <obj> --name PRESS_HQ --replace PRESS

동작 2단계
  1) OBJ(+MTL) -> `assets/<name>.usd` 로 변환. `g` 그룹 × 머티리얼 단위로 UsdGeom.Mesh
     를 만들고 MTL 의 Kd 를 displayColor 로 옮긴다. 축은 Y-up(OBJ 관행) -> Z-up(USD)
     로 되돌린다(export_smic_obj.py 의 역변환). 이미 Z-up 이면 --zup.
  2) 그 에셋을 `/World/Cell/Custom/<name>` 에 **reference** 로 걸고 지정 서브레이어에
     저장한다. 메시 자체는 이 저장소에 안 들어온다(JM/서한의 로봇 참조와 같은 방식).

--replace <STATION_ID> 를 주면 원래 절차적 설비를 **숨김 처리**한다.
  USD 에서 visibility 는 물리와 무관하므로 **콜라이더는 그대로 살아 있다** →
  블렌더 메시는 보기용, 기존 박스는 충돌용 프록시. 임포트 메시에 콜리전을 직접
  붙이고 싶으면 --collision convexHull | convexDecomposition.

주의
  - `build_smic_world.py` 를 SMIC_FORCE=1 로 다시 돌리면 이 편입은 **지워진다**
    (제네시스가 서브레이어를 비우기 때문). 그래서 매 편입을 `smic_custom_assets.json`
    에 기록해 두고 재빌드 후 --replay 로 다시 태울 수 있게 했다.
  - 치수 단일 진실원(smic_layout.py)이 그 설비에 대해서는 더 이상 형상을 지배하지
    않는다. 교체 사실이 USD customData 와 위 JSON 양쪽에 남는다.
"""
import os, sys, json, argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa: F401
import world_layers as wl
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.join(HERE, "assets")
LEDGER = os.path.join(HERE, "smic_custom_assets.json")


# ------------------------------------------------------------------ OBJ 파서
def parse_mtl(path):
    """newmtl -> (r,g,b) (Kd)."""
    out = {}
    if not os.path.exists(path):
        return out
    cur = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            t = line.split()
            if not t:
                continue
            if t[0] == "newmtl":
                cur = t[1]
                out.setdefault(cur, (0.62, 0.63, 0.65))
            elif t[0] == "Kd" and cur and len(t) >= 4:
                out[cur] = (float(t[1]), float(t[2]), float(t[3]))
    return out


def parse_obj(path):
    """-> (verts, mtl_map, chunks). chunk = {group, material, faces:[[idx0,...]]}
    faces 인덱스는 0-based 전역 정점 인덱스."""
    verts, chunks, mtl_map = [], [], {}
    group, material = "default", None
    cur = None

    def flush():
        if cur and cur["faces"]:
            chunks.append(cur)

    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            t = line.split()
            if not t:
                continue
            k = t[0]
            if k == "v":
                verts.append((float(t[1]), float(t[2]), float(t[3])))
            elif k == "mtllib":
                mtl_map.update(parse_mtl(os.path.join(os.path.dirname(path), t[1])))
            elif k in ("g", "o"):
                flush()
                group = "_".join(t[1:]) or "default"
                cur = {"group": group, "material": material, "faces": []}
            elif k == "usemtl":
                flush()
                material = t[1] if len(t) > 1 else None
                cur = {"group": group, "material": material, "faces": []}
            elif k == "f":
                if cur is None:
                    cur = {"group": group, "material": material, "faces": []}
                idx = []
                for tok in t[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)   # 음수 = 상대참조
                if len(idx) >= 3:
                    cur["faces"].append(idx)
    flush()
    return verts, mtl_map, chunks


def safe(name):
    s = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return ("m_" + s) if (not s or s[0].isdigit()) else s


# ------------------------------------------------------------------ OBJ -> USD
def obj_to_usd(obj_path, out_usd, name, zup=False, scale=1.0, collision="none"):
    verts, mtls, chunks = parse_obj(obj_path)
    if not verts or not chunks:
        sys.exit("[import] no geometry parsed from %s" % obj_path)

    if os.path.exists(out_usd):
        os.remove(out_usd)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/" + name)
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("source_obj", os.path.basename(obj_path))

    used, n_mesh, n_tri = {}, 0, 0
    for ci, ch in enumerate(chunks):
        # 이 청크가 쓰는 정점만 추려서 로컬 인덱스로 재매핑 (Blender 오브젝트 분리 유지)
        remap, pts = {}, []
        counts, indices = [], []
        for face in ch["faces"]:
            counts.append(len(face))
            n_tri += len(face) - 2
            for gi in face:
                if gi not in remap:
                    v = verts[gi]
                    # OBJ(Y-up) -> USD(Z-up):  (x, y, z) -> (x, -z, y)
                    p = (v[0], v[1], v[2]) if zup else (v[0], -v[2], v[1])
                    remap[gi] = len(pts)
                    pts.append(tuple(c * scale for c in p))
                indices.append(remap[gi])
        if not pts:
            continue
        base = safe(ch["group"] or "grp")
        if ch["material"]:
            base += "__" + safe(ch["material"])
        nm = base if base not in used else "%s_%d" % (base, ci)
        used[nm] = True

        m = UsdGeom.Mesh.Define(stage, "/%s/%s" % (name, nm))
        m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(indices)
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lo = [min(p[i] for p in pts) for i in range(3)]
        hi = [max(p[i] for p in pts) for i in range(3)]
        m.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
        col = mtls.get(ch["material"], (0.62, 0.63, 0.65))
        m.CreateDisplayColorAttr([Gf.Vec3f(*col)])
        if collision != "none":
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(collision)
        n_mesh += 1

    stage.GetRootLayer().Save()
    return {"meshes": n_mesh, "tris": n_tri, "verts": len(verts),
            "materials": len(mtls), "usd": out_usd}


# ------------------------------------------------------------------ 월드 편입
def wire_into_world(name, asset_usd, layer, at, yaw, replace=None):
    if not os.path.exists(wl.ROOT):
        sys.exit("[import] %s missing — run build_smic_world.py first" % wl.ROOT)
    stage = Usd.Stage.Open(wl.ROOT)
    wl.set_edit_layer(stage, layer)
    stage.OverridePrim("/World")
    stage.OverridePrim("/World/Cell")
    UsdGeom.Scope.Define(stage, "/World/Cell/Custom")

    path = "/World/Cell/Custom/" + name
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(Sdf.Path(path))
    xf = UsdGeom.Xform.Define(stage, path)
    prim = xf.GetPrim()
    rel = "./" + os.path.relpath(asset_usd, wl.WORLD_DIR).replace("\\", "/")
    prim.GetReferences().AddReference(rel)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*at))
    xf.AddRotateZOp().Set(float(yaw))
    prim.SetCustomDataByKey("source_asset", rel)
    if replace:
        prim.SetCustomDataByKey("replaces", replace)

    hidden = None
    if replace:
        st = stage.GetPrimAtPath("/World/Cell/Equip/" + replace)
        if not (st and st.IsValid()):
            sys.exit("[import] --replace: station %s not found" % replace)
        UsdGeom.Imageable(st).MakeInvisible()      # 콜라이더는 살아 있다(가시성≠물리)
        st.SetCustomDataByKey("replaced_by", path)
        hidden = str(st.GetPath())

    saved = wl.save(stage)
    wl.stamp_root_axis(stage)
    return {"prim": path, "layer": layer, "hidden": hidden,
            "saved": [os.path.basename(s) for s in saved]}


# ------------------------------------------------------------------ 원장
def unwire(name, keep_asset=False):
    """편입 취소: Custom 프림 제거 + 숨긴 원본 스테이션 복구 + 원장에서 삭제."""
    stage = Usd.Stage.Open(wl.ROOT)
    rec = None
    if os.path.exists(LEDGER):
        rec = next((d for d in json.load(open(LEDGER, encoding="utf-8"))
                    if d.get("name") == name), None)
    layer = (rec or {}).get("layer", "equipment")
    wl.set_edit_layer(stage, layer)
    path = "/World/Cell/Custom/" + name
    removed = bool(stage.GetPrimAtPath(path))
    if removed:
        stage.RemovePrim(Sdf.Path(path))
    restored = None
    if rec and rec.get("replace"):
        st = stage.GetPrimAtPath("/World/Cell/Equip/" + rec["replace"])
        if st and st.IsValid():
            UsdGeom.Imageable(st).MakeVisible()
            st.ClearCustomDataByKey("replaced_by")
            restored = str(st.GetPath())
    wl.save(stage)
    wl.stamp_root_axis(stage)
    if os.path.exists(LEDGER):
        data = [d for d in json.load(open(LEDGER, encoding="utf-8")) if d.get("name") != name]
        with open(LEDGER, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    asset = os.path.join(ASSET_DIR, name + ".usd")
    if not keep_asset and os.path.exists(asset):
        os.remove(asset)
    return {"removed": removed, "restored": restored, "asset_deleted": not keep_asset}


def ledger_write(rec):
    data = []
    if os.path.exists(LEDGER):
        try:
            data = json.load(open(LEDGER, encoding="utf-8"))
        except Exception:
            data = []
    data = [d for d in data if d.get("name") != rec["name"]] + [rec]
    with open(LEDGER, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return LEDGER


def main():
    ap = argparse.ArgumentParser(description="Blender OBJ -> SMIC World")
    ap.add_argument("obj", nargs="?", help="가져올 .obj 경로")
    ap.add_argument("--name", help="에셋/프림 이름 (영숫자+_)")
    ap.add_argument("--replace", help="대체할 스테이션 ID (예: PRESS) — 원본은 숨김 처리")
    ap.add_argument("--layer", default="equipment", choices=list(wl.LAYER_KEYS),
                    help="편입할 서브레이어 (기본 equipment)")
    ap.add_argument("--at", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"), help="배치 위치(월드, m)")
    ap.add_argument("--yaw", type=float, default=0.0, help="Z축 회전(도)")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--zup", action="store_true", help="OBJ 가 이미 Z-up 인 경우")
    ap.add_argument("--collision", default="none",
                    choices=["none", "convexHull", "convexDecomposition", "meshSimplification"],
                    help="임포트 메시에 직접 콜리전 부여 (기본 none = 기존 박스를 프록시로 사용)")
    ap.add_argument("--replay", action="store_true",
                    help="smic_custom_assets.json 의 모든 편입을 재적용(재빌드 후)")
    ap.add_argument("--remove", metavar="NAME",
                    help="편입 취소: Custom 프림 제거 + 숨긴 원본 복구 + 원장 삭제")
    ap.add_argument("--keep-asset", action="store_true", help="--remove 시 assets/*.usd 는 남긴다")
    a = ap.parse_args()

    os.makedirs(ASSET_DIR, exist_ok=True)

    if a.remove:
        r = unwire(safe(a.remove), keep_asset=a.keep_asset)
        print("[remove] %s: prim_removed=%s restored=%s asset_deleted=%s"
              % (a.remove, r["removed"], r["restored"], r["asset_deleted"]))
        return

    if a.replay:
        if not os.path.exists(LEDGER):
            sys.exit("[import] no ledger to replay: %s" % LEDGER)
        for rec in json.load(open(LEDGER, encoding="utf-8")):
            w = wire_into_world(rec["name"], rec["asset"], rec.get("layer", "equipment"),
                                rec.get("at", [0, 0, 0]), rec.get("yaw", 0.0),
                                rec.get("replace"))
            print("[replay] %-16s -> %s%s" % (rec["name"], w["prim"],
                                              ("  (hid %s)" % w["hidden"]) if w["hidden"] else ""))
        return

    if not a.obj or not a.name:
        ap.error("obj 와 --name 은 필수 (또는 --replay)")
    if not os.path.exists(a.obj):
        sys.exit("[import] no such obj: %s" % a.obj)
    name = safe(a.name)

    out_usd = os.path.join(ASSET_DIR, name + ".usd")
    conv = obj_to_usd(a.obj, out_usd, name, zup=a.zup, scale=a.scale, collision=a.collision)
    wire = wire_into_world(name, out_usd, a.layer, a.at, a.yaw, a.replace)
    led = ledger_write({"name": name, "obj": os.path.abspath(a.obj), "asset": out_usd,
                        "layer": a.layer, "at": list(a.at), "yaw": a.yaw,
                        "scale": a.scale, "zup": bool(a.zup),
                        "collision": a.collision, "replace": a.replace})

    print("=== OBJ -> SMIC World ===")
    print("  asset : %s  (%.2f MB)" % (out_usd, os.path.getsize(out_usd) / 1e6))
    print("  meshes=%d  tris=%d  materials=%d" % (conv["meshes"], conv["tris"], conv["materials"]))
    print("  prim  : %s  in %s  at %s yaw=%.1f" % (wire["prim"], wire["layer"], a.at, a.yaw))
    if wire["hidden"]:
        print("  hidden: %s  (콜라이더는 유지 — 충돌 프록시로 계속 동작)" % wire["hidden"])
    print("  collision on imported mesh:", a.collision)
    print("  saved :", wire["saved"])
    print("  ledger:", led, "(재빌드 후 --replay 로 재적용)")


if __name__ == "__main__":
    main()
