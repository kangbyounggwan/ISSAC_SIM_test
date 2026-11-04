"""Read KETI world, draw pre-start obstacles, persist three deterministic A* plans."""
from pathlib import Path
import argparse
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "smic_world"))
import _usd_boot
from pxr import Usd, UsdGeom, UsdPhysics
from keti_navigation.planner import astar, random_obstacle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise RuntimeError("Use a new output directory")
    config_path = ROOT / "keti_navigation/config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    source = ROOT / "smic_world/smic_world.usd"
    stage = Usd.Stage.Open(str(source))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise RuntimeError("KETI world unavailable or coordinate contract invalid")
    hashes = {str(Path(l.realPath).resolve()): hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()
              for l in stage.GetUsedLayers() if l.realPath}
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True)
    obstacles = []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if str(prim.GetPath()).startswith("/World/Robots/") or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            raise RuntimeError("Missing collider extent: " + str(prim.GetPath()))
        low, high = list(box.GetMin()), list(box.GetMax())
        if high[2] <= config["floor_z"] + .04 or low[2] > config["floor_z"] + 2.:
            continue
        obstacles.append({"path": str(prim.GetPath()), "min": low, "max": high})
    source_plan = astar(config["start_xy"], config["goal_xy"], obstacles, config["bounds_xy"],
                        config["grid_resolution_m"], config["planning_inflation_m"])
    scenarios = []
    for seed in config["seeds"]:
        obstacle = random_obstacle(config, seed)
        plan = astar(config["start_xy"], config["goal_xy"], obstacles + [obstacle], config["bounds_xy"],
                     config["grid_resolution_m"], config["planning_inflation_m"])
        scenarios.append({"seed": seed, "random_obstacle": obstacle, "plan": plan})
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
        raise RuntimeError("Source changed during preparation")
    report = {"world": str(source), "source_sha256": hashes, "config": config,
              "planner_source_sha256": {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in [Path(__file__), Path(__file__).with_name("planner.py")]},
              "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
              "obstacles": obstacles, "baseline_plan_without_random_obstacle": source_plan,
              "scenarios": scenarios, "map_source": "oracle authored USD, NOT camera/LiDAR",
              "dynamic_walk_validated": False, "original_world_modified": False,
              "scope": "1 dynamic H1 in a session overlay; original display robots disabled, background props static"}
    args.out.mkdir(parents=True)
    (args.out / "scenarios.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"obstacle_boxes": len(obstacles), "baseline_length_m": source_plan["length_m"],
                      "random_scenarios": [{"seed": x["seed"], "center": x["random_obstacle"]["center"],
                           "path_xy": x["plan"]["path_xy"], "length_m": x["plan"]["length_m"],
                           "direct_path_blocked": x["plan"]["direct_path_blocked"]} for x in scenarios]}, indent=2))


if __name__ == "__main__":
    main()
