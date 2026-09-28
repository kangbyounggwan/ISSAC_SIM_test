"""Read-only USD dependency/composition validation; optional source parity check."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from package_world_snapshot import bootstrap, collect, ROOTS


def inspect(stage, root):
    from pxr import UsdGeom, UsdPhysics
    # Isaac Sim's bundled USD exposes errors on each composed prim index.
    errors = sorted({str(e) for p in stage.TraverseAll() for e in p.GetPrimIndex().localErrors})
    external_layers = [p.realPath for p in stage.GetUsedLayers()
                       if p.realPath and not Path(p.realPath).resolve().is_relative_to(root)]
    prims = list(stage.Traverse())
    return {
        "composition_errors": errors,
        "external_layers": external_layers,
        "default_prim": str(stage.GetDefaultPrim().GetPath()),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "prim_count": len(prims),
        "colliders": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),
        "rigid_bodies": sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),
        "joints": sum(p.IsA(UsdPhysics.Joint) for p in prims),
    }


def compare(source, packaged):
    from pxr import Sdf, UsdGeom
    a = {str(p.GetPath()): p for p in source.Traverse()}
    b = {str(p.GetPath()): p for p in packaged.Traverse()}
    differences = []
    if a.keys() != b.keys():
        differences.append("Composed prim paths differ")
    ca, cb = UsdGeom.XformCache(), UsdGeom.XformCache()
    transform_error, compared = 0.0, 0
    for path in a.keys() & b.keys():
        pa, pb = a[path], b[path]
        if pa.GetTypeName() != pb.GetTypeName() or pa.GetAppliedSchemas() != pb.GetAppliedSchemas():
            differences.append(f"Type/schema differs: {path}")
        if pa.IsA(UsdGeom.Xformable):
            ma, mb = ca.GetLocalToWorldTransform(pa), cb.GetLocalToWorldTransform(pb)
            transform_error = max(transform_error, max(abs(ma[i][j] - mb[i][j])
                                                        for i in range(4) for j in range(4)))
        for attr in pa.GetAttributes():
            other = pb.GetAttribute(attr.GetName())
            va, vb = attr.Get(), other.Get() if other else None
            # Asset paths are deliberately re-anchored, all other values must match.
            if isinstance(va, Sdf.AssetPath) or attr.GetTypeName() == Sdf.ValueTypeNames.AssetArray:
                continue
            compared += 1
            if str(va) != str(vb):
                differences.append(f"Attribute differs: {attr.GetPath()}")
        for rel in pa.GetRelationships():
            other = pb.GetRelationship(rel.GetName())
            if not other or rel.GetTargets() != other.GetTargets():
                differences.append(f"Relationship differs: {rel.GetPath()}")
    if transform_error > 1e-9:
        differences.append("World-space transforms differ")
    return {"compared_attributes": compared, "maximum_transform_error": transform_error,
            "differences": differences, "pass": not differences}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--source", type=Path)
    parser.add_argument("--isaac-root", type=Path, default=Path("C:/isaacsim"))
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    bootstrap(args.isaac_root)
    from pxr import Usd
    root = args.root.resolve()
    graph, builtins = collect(root)
    manifest = json.loads((root / "WORLD_SNAPSHOT_MANIFEST.json").read_text(encoding="utf-8"))
    hash_failures = []
    for item in manifest["files"]:
        if hashlib.sha256((root / item["path"]).read_bytes()).hexdigest() != item["packaged_sha256"]:
            hash_failures.append(item["path"])
    results = []
    for entry in ROOTS:
        stage = Usd.Stage.Open(str(root / entry), load=Usd.Stage.LoadAll)
        if not stage:
            raise RuntimeError(f"Cannot open: {entry}")
        result = {"entry": entry, **inspect(stage, root)}
        result["pass"] = (not result["composition_errors"] and not result["external_layers"]
                          and result["up_axis"] == "Z" and result["meters_per_unit"] == 1
                          and result["default_prim"] == "/World")
        if args.source:
            original = Usd.Stage.Open(str(args.source.resolve() / entry), load=Usd.Stage.LoadAll)
            result["source_parity"] = compare(original, stage)
            result["pass"] &= result["source_parity"]["pass"]
        results.append(result)
    report = {"pass": not hash_failures and all(r["pass"] for r in results),
              "file_count": len(graph), "hash_failures": hash_failures,
              "builtin_material_dependencies": builtins, "worlds": results,
              "scope": "USD dependency closure, composition, scene properties; not a dynamic simulation"}
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
