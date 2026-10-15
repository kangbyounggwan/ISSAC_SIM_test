#!/usr/bin/env python3
"""Open the portable JM world in Isaac Sim and render three verification views."""
import argparse
import json
from pathlib import Path
import time
import traceback

parser = argparse.ArgumentParser()
parser.add_argument("--world", required=True, type=Path)
parser.add_argument("--output", required=True, type=Path)
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
args, _ = parser.parse_known_args()
args.output.mkdir(parents=True, exist_ok=True)
report_path = args.output / "render_report.json"
report = {"status": "starting", "world": str(args.world.resolve()), "gpu_index": args.gpu, "views": []}
report_path.write_text(json.dumps(report, indent=2) + "\n")
started = time.monotonic()

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True,
    "renderer": "RaytracedLighting",
    "width": args.width,
    "height": args.height,
    "active_gpu": args.gpu,
    "physics_gpu": args.gpu,
    "multi_gpu": False,
    "sync_loads": True,
})

try:
    import carb
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, Usd, UsdGeom

    carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
    context = omni.usd.get_context()
    context.open_stage(str(args.world.resolve()))
    for _ in range(30):
        app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("The world did not open")
    # The USD bundled with Isaac Sim 4.5 predates Stage.GetCompositionErrors.
    errors = list(dict.fromkeys(
        str(error)
        for prim in stage.Traverse()
        for error in prim.GetPrimIndex().localErrors
    ))
    if errors:
        raise RuntimeError(errors)
    # The source stage is read-only for this run; added cameras live in session.
    stage.SetEditTarget(stage.GetSessionLayer())
    report["composition_errors"] = errors
    report["usd_version"] = list(Usd.GetVersion())
    report["robot_meshes"] = {}
    for name in ["Atlas_1", "H1_1", "Atlas_2", "H1_2"]:
        prim = stage.GetPrimAtPath("/World/Robots/" + name)
        if not prim:
            raise RuntimeError("Missing robot " + name)
        count = sum(p.GetTypeName() == "Mesh" for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        if not count:
            raise RuntimeError("Missing geometry " + name)
        report["robot_meshes"][name] = count

    # Add a close view so robot geometry is visually reviewable as well as counted.
    robot = stage.GetPrimAtPath("/World/Robots/H1_1")
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bounds = bbox.ComputeWorldBound(robot).ComputeAlignedRange()
    center = (bounds.GetMin() + bounds.GetMax()) * 0.5
    eye = center + Gf.Vec3d(-2.5, -3.0, 1.4)
    matrix = Gf.Matrix4d().SetLookAt(eye, center, Gf.Vec3d(0, 0, 1)).GetInverse()
    camera = UsdGeom.Camera.Define(stage, "/World/VerificationH1Cam")
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(20.955 * args.height / args.width)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    UsdGeom.Xformable(camera).AddTransformOp().Set(matrix)

    rep.orchestrator.set_capture_on_play(False)
    for name, camera_path in [("factory_top", "/World/FloorCam"), ("factory_aisle", "/World/RenderCam"), ("h1_close", "/World/VerificationH1Cam")]:
        folder = args.output / name
        folder.mkdir(exist_ok=True)
        product = rep.create.render_product(camera_path, (args.width, args.height))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(folder), rgb=True)
        writer.attach([product])
        for _ in range(30):
            app.update()
        rep.orchestrator.step(rt_subframes=32)
        rep.orchestrator.wait_until_complete()
        writer.detach()
        product.destroy()
        files = sorted(folder.glob("rgb*.png"))
        if not files:
            raise RuntimeError("No image was produced for " + name)
        report["views"].append({"name": name, "camera": camera_path, "files": [str(p.resolve()) for p in files]})
        print("[FACTOR] rendered " + name, flush=True)
    report["status"] = "success"
    report["elapsed_seconds"] = round(time.monotonic() - started, 2)
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print("[FACTOR] SUCCESS " + str(report_path), flush=True)
except Exception:
    report["status"] = "failed"
    report["error"] = traceback.format_exc()
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    traceback.print_exc()
    raise
finally:
    app.close()
