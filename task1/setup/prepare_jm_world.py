#!/usr/bin/env python3
"""Create a portable JM scene without editing the upstream checkout."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from pxr import Sdf, Usd, UsdGeom

BASE = Path(__file__).resolve().parents[1]
REPO = BASE.parent
OUTPUT = BASE / "workspace" / "jm_world"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assets = OUTPUT / "assets"
    textures = OUTPUT / "textures"
    assets.mkdir(parents=True, exist_ok=True)
    textures.mkdir(exist_ok=True)
    source = REPO / "jm_factory_world_atlas_h1.flat.usd"
    target = assets / source.name
    shutil.copy2(source, target)
    for name in ["label_in.png", "label_out.png"]:
        shutil.copy2(REPO / "textures" / name, textures / name)

    # Keep the original binary geometry intact. Author only Linux texture paths
    # and stage metadata in a small, readable root layer.
    root_path = OUTPUT / "jm_factory_world.usda"
    root = Sdf.Layer.CreateNew(str(root_path)) if not root_path.exists() else Sdf.Layer.FindOrOpen(str(root_path))
    root.Clear()
    root.subLayerPaths = ["./assets/" + source.name]
    original = Usd.Stage.Open(str(source))
    stage = Usd.Stage.Open(root)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(original))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(original))
    patches = []
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            if value and value.path.startswith("C:"):
                name = value.path.replace("\\", "/").rsplit("/", 1)[-1]
                if not (textures / name).is_file():
                    raise RuntimeError(f"Unknown external asset: {value.path}")
                replacement = "./textures/" + name
                patches.append({"attribute": str(attr.GetPath()), "from": value.path, "to": replacement})
                attr.Set(Sdf.AssetPath(replacement))
    root.Save()

    errors = [str(e) for e in stage.GetCompositionErrors()]
    if errors:
        raise RuntimeError(errors)
    robots = []
    for name in ["Atlas_1", "H1_1", "Atlas_2", "H1_2"]:
        prim = stage.GetPrimAtPath("/World/Robots/" + name)
        if not prim:
            raise RuntimeError(f"Missing robot: {name}")
        parts = list(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        meshes = [p for p in parts if p.GetTypeName() == "Mesh"]
        if not meshes:
            raise RuntimeError(f"Missing robot geometry: {name}")
        robots.append({"name": name, "mesh_count": len(meshes), "vertices": sum(len(p.GetAttribute("points").Get() or []) for p in meshes)})
    report = {
        "source_commit": subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True).strip(),
        "source_file": str(source),
        "source_sha256": sha256(source),
        "copied_geometry_sha256": sha256(target),
        "root_stage": str(root_path),
        "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "texture_overrides": patches,
        "robots": robots,
        "cameras": [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == "Camera"],
        "composition_errors": errors,
        "usd_version": list(Usd.GetVersion()),
    }
    (OUTPUT / "prepare_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
