"""Copy authored USD dependency graphs into a repository; never edit the source.

Run with Isaac Sim's Python. Built-in OmniPBR.mdl remains supplied by Isaac Sim.
This packages environments, not training checkpoints, recordings or credentials.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOTS = [
    "jm_factory_world_atlas_h1.usd",
    "jm_coop4_rl/assets/jm_world_h1_fleet.usda",
    "jm_factory_world_h1_ring.usda",
    "jm_factory_world_h1_ring_physical_gripper.usda",
    "smic_world/smic_world.usd",
]
USD_EXTENSIONS = {".usd", ".usda", ".usdc"}
BUILTINS = {"OmniPBR.mdl"}
DLL_HANDLES = []


def bootstrap(isaac_root):
    for pattern in ("omni.usd.libs-*", "omni.usd.schema.physx-*"):
        folders = sorted((isaac_root / "extscache").glob(pattern))
        if not folders:
            raise RuntimeError(f"Missing USD runtime: {pattern}")
        folder = folders[-1]
        sys.path.insert(0, str(folder))
        if os.name == "nt":
            DLL_HANDLES.append(os.add_dll_directory(str(folder / "bin")))
    from pxr import Plug
    for info in (folder / "plugins").glob("*/resources/plugInfo.json"):
        Plug.Registry().RegisterPlugins(str(info))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def destination(path, source):
    if path.is_relative_to(source):
        return path.relative_to(source)
    # Stable labels, without shipping a developer's absolute filesystem paths.
    for prefix, name in [
        (Path("C:/tmp/unitree_ros"), "unitree_ros"),
        (Path("C:/tmp/atlas-mujoco"), "atlas_mujoco"),
        (Path("C:/tmp/robotiq_2f85"), "robotiq_2f85"),
    ]:
        if path.is_relative_to(prefix):
            return Path("assets/vendor") / name / path.relative_to(prefix)
    raise RuntimeError(f"Unreviewed external dependency: {path}")


def collect(source):
    from pxr import Sdf, UsdUtils
    graph, builtin_refs = {}, []
    pending = [(source / p).resolve() for p in ROOTS]
    while pending:
        path = pending.pop()
        if path in graph:
            continue
        if not path.is_file():
            raise FileNotFoundError(path)
        graph[path] = {}
        if path.suffix.lower() not in USD_EXTENSIONS:
            continue
        layer = Sdf.Layer.FindOrOpen(str(path))
        if not layer:
            raise RuntimeError(f"Cannot read layer: {path}")
        for refs in UsdUtils.ExtractExternalReferences(str(path)):
            for asset in refs:
                if asset in BUILTINS:
                    builtin_refs.append({"layer": destination(path, source).as_posix(), "asset": asset})
                    continue
                resolved = Path(Sdf.ComputeAssetPathRelativeToLayer(layer, asset)).resolve()
                if not resolved.is_file():
                    raise FileNotFoundError(f"{path}: {asset} -> {resolved}")
                destination(resolved, source)  # Explicit allowlist before copying.
                graph[path][asset] = resolved
                pending.append(resolved)
    return graph, builtin_refs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--isaac-root", type=Path, default=Path("C:/isaacsim"))
    args = parser.parse_args()
    bootstrap(args.isaac_root)
    from pxr import Sdf, UsdUtils
    source = args.source.resolve()
    graph, builtins = collect(source)
    total = sum(p.stat().st_size for p in graph)
    print(json.dumps({"files": len(graph), "source_bytes": total,
                      "largest": sorted([(destination(p, source).as_posix(), p.stat().st_size)
                                         for p in graph], key=lambda p: p[1], reverse=True)[:12],
                      "builtin_assets": builtins}, indent=2))
    if args.output is None:
        return
    out = args.output.resolve()
    if out == source or not (out / ".git").is_file():
        raise RuntimeError("Output must be a separate Git worktree")
    records = []
    for path, deps in graph.items():
        rel = destination(path, source)
        target = out / rel
        before = digest(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix.lower() in USD_EXTENSIONS:
            layer = Sdf.Layer.CreateAnonymous()
            layer.TransferContent(Sdf.Layer.FindOrOpen(str(path)))
            replacements = {asset: Path(os.path.relpath(out / destination(dep, source), target.parent)).as_posix()
                            for asset, dep in deps.items()}
            UsdUtils.ModifyAssetPaths(layer, lambda asset: replacements.get(asset, asset))
            if not layer.Export(str(target)):
                raise RuntimeError(f"Export failed: {rel}")
        else:
            shutil.copy2(path, target)
        if digest(path) != before:
            raise RuntimeError(f"Source changed during copy: {path}")
        records.append({"path": rel.as_posix(), "source_sha256": before,
                        "packaged_sha256": digest(target), "bytes": target.stat().st_size,
                        "dependencies": [destination(d, source).as_posix() for d in deps.values()]})
    report = {"schema": 1, "roots": ROOTS, "builtin_assets": builtins,
              "scope": "USD scenes and their complete local dependency graph; no training runs",
              "source_modified": False, "files": sorted(records, key=lambda r: r["path"])}
    (out / "WORLD_SNAPSHOT_MANIFEST.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Packaged {len(records)} files in {out}")


if __name__ == "__main__":
    main()
