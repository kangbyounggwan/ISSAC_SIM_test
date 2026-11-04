"""Capture real rendered RGB-D, without advancing physics or moving a robot."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--world', choices=['jm', 'keti'], default='jm')
p.add_argument('--out', type=Path, required=True)
args = p.parse_args()
if args.out.exists():
    raise RuntimeError('Use a new output directory')
args.out.mkdir(parents=True)
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 1280, 'height': 720})
ok = False
try:
    import numpy as np
    import cv2
    import carb.settings
    import omni.usd
    import omni.timeline
    import omni.replicator.core as rep
    from pxr import Usd, UsdGeom, Gf
    from jm_xnavdp.adapter import local_goal
    world = ROOT / ('jm_factory_world_h1_ring_physical_gripper.usda' if args.world == 'jm' else 'smic_world/smic_world.usd')
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.open_stage(str(world))
    for _ in range(20):
        app.update()
    stage = ctx.get_stage()
    layers = {l.realPath: hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()
              for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()}
    stage.SetEditTarget(stage.GetSessionLayer())
    if args.world == 'jm':
        robot = stage.GetPrimAtPath('/World/H1Fleet/H1_1')
        matrix = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        base = np.array(matrix.ExtractTranslation(), dtype=float)
        forward = np.array(matrix.TransformDir(Gf.Vec3d(1, 0, 0)))
        yaw = math.atan2(forward[1], forward[0])
        report = json.loads((ROOT / 'artifacts/jm_stop_pose_20260929/scan_002/stop_pose_report.json').read_text())
        goal = report['measured']['nav_handoff']['base_world'][:2]
        pose_source = 'H1_1 authored episode-initialization pose; physics not advanced'
    else:
        manifest = json.loads((ROOT / 'artifacts/keti_navigation_20260929/video_seed42_004/manifest.json').read_text())
        config, scenario = manifest['config'], manifest['scenario']
        base = np.array([*config['start_xy'], config['floor_z'] + .98])
        yaw = math.radians(config['start_yaw_deg'])
        goal = config['goal_xy']
        obs = scenario['random_obstacle']
        cube = UsdGeom.Cube.Define(stage, '/World/XNavDPObservation/RandomBox')
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*obs['center']))
        cube.AddScaleOp().Set(Gf.Vec3f(*obs['size']))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.94, .38, .07)])
        pose_source = 'Existing seed42 start; observation camera only, no moving robot'
    # Match upstream D455 field of view at doubled 1280x720 resolution.
    focal = 1.93
    fx = 326.398559570312 * 2
    intrinsic = [[fx, 0., 640.], [0., fx, 360.], [0., 0., 1.]]
    pitch = math.radians(8.)
    heading = np.array([math.cos(yaw), math.sin(yaw), 0.])
    eye = base + .12 * heading + np.array([0., 0., .55])
    view = math.cos(pitch) * heading + np.array([0., 0., -math.sin(pitch)])
    camera = UsdGeom.Camera.Define(stage, '/World/XNavDPObservation/Camera')
    camera.CreateFocalLengthAttr(focal)
    camera.CreateHorizontalApertureAttr(1280 * focal / fx)
    camera.CreateVerticalApertureAttr(720 * focal / fx)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
    transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*(eye + view)), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera.AddTransformOp().Set(transform)
    carb.settings.get_settings().set('/rtx/post/histogram/enabled', False)
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()), (1280, 720))
    rgb_ann = rep.AnnotatorRegistry.get_annotator('rgb')
    depth_ann = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
    rgb_ann.attach([product])
    depth_ann.attach([product])
    for _ in range(20):
        app.update()
    rep.orchestrator.step(rt_subframes=16, delta_time=0., pause_timeline=True)
    rep.orchestrator.wait_until_complete()
    rgb = np.asarray(rgb_ann.get_data())[:, :, :3].copy()
    depth = np.asarray(depth_ann.get_data()).reshape(720, 1280).copy()
    if rgb.shape != (720, 1280, 3) or rgb.mean() < 1:
        raise RuntimeError('Invalid rendered RGB')
    valid = np.isfinite(depth) & (depth >= .1) & (depth <= 5.)
    if valid.mean() < .01:
        raise RuntimeError('Insufficient valid rendered depth')
    np.savez_compressed(args.out / 'observation.npz', rgb=rgb, depth_m=depth)
    cv2.imwrite(str(args.out / 'rgb_720p.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    dep_vis = np.clip(np.nan_to_num(depth, nan=5., posinf=5.) / 5., 0., 1.)
    cv2.imwrite(str(args.out / 'depth_720p.png'), cv2.applyColorMap((dep_vis * 255).astype(np.uint8), cv2.COLORMAP_TURBO))
    unchanged = all(hashlib.sha256(Path(path).read_bytes()).hexdigest() == h for path, h in layers.items())
    if not unchanged:
        raise RuntimeError('Source layer changed during capture')
    meta = {'status': 'STATIC_RGBD_CAPTURE_ONLY', 'world': str(world), 'world_key': args.world,
            'physics_stepped': False, 'robot_motion_validated': False, 'pose_source': pose_source,
            'base_world': base.tolist(), 'yaw_rad': yaw, 'goal_world_xy': goal,
            'goal_local_m': local_goal(base[:2], yaw, goal).tolist(), 'intrinsic': intrinsic,
            'camera_eye_world': eye.tolist(), 'camera_to_world_usd_row_matrix': [list(r) for r in transform],
            'camera_pitch_down_deg': 8., 'camera_offset_body_m': [.12, 0., .55],
            'resolution': [1280, 720], 'color_order': 'RGB', 'depth_units': 'metres',
            'depth_type': 'distance_to_image_plane', 'depth_valid_0p1_to_5m_fraction': float(valid.mean()),
            'model_obstacle_ground_truth_supplied': False, 'self_pose_source': 'simulator ground truth; NOT SLAM',
            'source_layers_unchanged': unchanged, 'source_sha256': layers}
    (args.out / 'observation.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    print(json.dumps({k: meta[k] for k in ['status', 'base_world', 'goal_local_m', 'depth_valid_0p1_to_5m_fraction']}), flush=True)
    ok = True
except BaseException:
    error = traceback.format_exc()
    (args.out / 'failure.json').write_text(json.dumps({'error': error}, indent=2), encoding='utf-8')
    print(error, flush=True)
finally:
    app.close()
sys.exit(0 if ok else 1)
