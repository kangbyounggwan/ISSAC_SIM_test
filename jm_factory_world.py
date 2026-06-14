#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jm_factory_world.py — 제이엠일렉트로닉스 시화공장 USD 월드 생성기 (Isaac Sim / Omniverse).

지오메트리는 world_geometry.build_scene() (단일 소스)에서 가져오고, 여기서는
USD 스테이지 + 물리 씬 + 정적 충돌체 + 조명을 구성해 .usd/.usda 로 저장합니다.

단위 meter, Z-up. 용도: 로봇 주행/AMR 시뮬레이션.

실행:
  <isaac>/python.bat jm_factory_world.py --out jm_factory_world.usd
  python jm_factory_world.py --out jm_factory_world.usda     # USD 설치 파이썬
"""
import argparse
from world_geometry import build_scene, C  # noqa


def main(out_path="jm_factory_world.usda"):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, Sdf

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    # 부모 스코프(Xform) 자동 생성
    defined = {"/World"}

    def ensure_parents(path):
        segs = path.strip("/").split("/")
        cur = ""
        for s in segs[:-1]:
            cur += "/" + s
            if cur not in defined:
                UsdGeom.Xform.Define(stage, cur)
                defined.add(cur)

    def tag(prim, p):
        # 층/분류 메타 (Isaac Sim 필터·층 선택용)
        prim.CreateAttribute("jm:floor", Sdf.ValueTypeNames.String).Set(p["floor"])
        prim.CreateAttribute("jm:category", Sdf.ValueTypeNames.String).Set(p["cat"])

    parts = build_scene()
    for p in parts:
        ensure_parents(p["path"])
        if p["type"] == "box":
            c = UsdGeom.Cube.Define(stage, p["path"])
            c.CreateSizeAttr(1.0)
            c.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
            c.AddTranslateOp().Set(Gf.Vec3d(*p["center"]))
            if p.get("rot"):
                qx, qy, qz, qw = p["rot"]
                c.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
            c.AddScaleOp().Set(Gf.Vec3f(*p["size"]))
            c.CreateDisplayColorAttr([Gf.Vec3f(*p["color"])])
            if p["collider"]:
                UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            tag(c.GetPrim(), p)
        else:  # mesh
            m = UsdGeom.Mesh.Define(stage, p["path"])
            m.CreatePointsAttr([Gf.Vec3f(*v) for v in p["points"]])
            m.CreateFaceVertexCountsAttr([len(f) for f in p["faces"]])
            idx = [i for f in p["faces"] for i in f]
            m.CreateFaceVertexIndicesAttr(idx)
            m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            m.CreateDoubleSidedAttr(True)
            xs = [v[0] for v in p["points"]]; ys = [v[1] for v in p["points"]]; zs = [v[2] for v in p["points"]]
            m.CreateExtentAttr([(min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))])
            m.CreateDisplayColorAttr([Gf.Vec3f(*p["color"])])
            if p["collider"]:
                UsdPhysics.CollisionAPI.Apply(m.GetPrim())
                mc = UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim())
                mc.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
            tag(m.GetPrim(), p)

    # 조명
    UsdGeom.Xform.Define(stage, "/World/Lighting")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(0.53)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.9))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 25.0))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Sky")
    dome.CreateIntensityAttr(800.0)
    dome.CreateColorAttr(Gf.Vec3f(0.8, 0.85, 0.95))

    stage.GetRootLayer().Save()
    print("USD world written:", out_path)
    print("  prims:", sum(1 for _ in stage.Traverse()))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="JM Electronics 시화공장 USD 월드 생성")
    ap.add_argument("--out", default="jm_factory_world.usda", help="출력 USD 경로 (.usd/.usda)")
    args = ap.parse_args()
    main(args.out)
