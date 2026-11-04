"""KETI random static obstacle demo: A* + measured path tracking + frozen H1 walker.

Map information is authored USD ground truth, NOT camera/LiDAR. No training,
root teleport during motion, obstacle removal, or simulated success animation.
"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import math
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scenarios", type=Path, required=True)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--no-video", action="store_true")
parser.add_argument("--terminal-controller", choices=["path", "rack_rl", "measured_support_hold", "commanded_support_hold"], default="path",
                    help="Single-variable trial: original path stop or existing learned rack approach inside 0.8m")
args = parser.parse_args()
if args.out.exists():
    raise RuntimeError("Use a new output path")
scenario_set = json.loads(args.scenarios.read_text(encoding="utf-8"))
config = scenario_set["config"]
scenario = next(s for s in scenario_set["scenarios"] if s["seed"] == args.seed)
for path, expected in scenario_set["source_sha256"].items():
    if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
        raise RuntimeError("Source world changed after planning: " + path)
contract_path = ROOT / "jm_coop4_rl/h1_rack_env.py"
constants = {}
for node in ast.parse(contract_path.read_text(encoding="utf-8")).body:
    if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
        name = node.targets[0].id
        if name in {"BODY_19", "WALK_DEFAULT_19", "WALKER_SHA256"}:
            constants[name] = ast.literal_eval(node.value)
walker_path = ROOT / "artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt"
robot_asset = ROOT / "jm_wb7_pinkik/assets/h1_wb7_standalone.usd"
nav_path = ROOT / "jm_coop4_rl/runs/h1_nav_001/model_200.pt"
if hashlib.sha256(walker_path.read_bytes()).hexdigest() != constants["WALKER_SHA256"]:
    raise RuntimeError("Frozen walker contract mismatch")
args.out.mkdir(parents=True)
out = args.out.resolve()
manifest = {"arguments": vars(args), "config": config, "scenario": scenario,
            "map_source": scenario_set["map_source"], "original_world_modified": False,
            "source_sha256": {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              [Path(__file__), Path(__file__).with_name("planner.py"), args.scenarios,
                               contract_path, walker_path, robot_asset, nav_path]},
            "world_source_sha256": scenario_set["source_sha256"]}
(out / "manifest.json").write_text(json.dumps(manifest, default=str, indent=2), encoding="utf-8")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": 1280, "height": 720})
import numpy as np
import torch
import carb.settings
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, RigidPrim
from isaacsim.core.utils.types import ArticulationAction
from keti_navigation.planner import clearance, lookahead_target, line_free

carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
ctx = omni.usd.get_context()
ctx.open_stage(scenario_set["world"])
for _ in range(10):
    app.update()
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
stage.GetPrimAtPath("/World/Robots").SetActive(False)
# Static background is explicit in this navigation-only overlay, not saved to source.
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)

robot_path = "/World/KetiNav/H1"
robot_prim = UsdGeom.Xform.Define(stage, robot_path).GetPrim()
robot_prim.GetReferences().AddReference(robot_asset.as_posix())
xf = UsdGeom.Xformable(robot_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(*config["start_xy"], config["floor_z"] + .98))
xf.AddRotateZOp().Set(config["start_yaw_deg"])


def box(path, center, size, color, collision=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


obstacle = scenario["random_obstacle"]
box(obstacle["path"], obstacle["center"], obstacle["size"], (.94, .38, .07), True)
top = obstacle["max"][2]
for k in range(3):
    center = [obstacle["center"][0] - .2 + .2 * k, obstacle["center"][1], top + .003]
    box(f'/World/KetiNav/Stripe{k}', center, [.08, obstacle["size"][1], .006], (.12, .12, .12))


def curve(path, points, color, width):
    line = UsdGeom.BasisCurves.Define(stage, path)
    line.CreateTypeAttr("linear")
    line.CreateCurveVertexCountsAttr([len(points)])
    line.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), .018) for p in points])
    line.CreateWidthsAttr([width])
    line.SetWidthsInterpolation("constant")
    line.CreateDisplayColorAttr([color])
    return line


plan = np.array(scenario["plan"]["path_xy"])
start, goal = np.array(config["start_xy"]), np.array(config["goal_xy"])
for k in range(18):
    a, b = start + (goal - start) * k / 18, start + (goal - start) * (k + .55) / 18
    curve(f'/World/KetiNav/Direct{k}', [a, b], (.9, .17, .12), .024)
curve('/World/KetiNav/PlannedPath', plan, (.05, .72, .18), .035)
for name, p, color in (("Start", start, (.15, .5, .95)), ("Goal", goal, (.2, .9, .35))):
    box('/World/KetiNav/' + name, [*p, .009], [.45, .45, .014], color)

scene_prim = stage.GetPrimAtPath('/World/PhysicsScene')
ps = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
ps.CreateEnableGPUDynamicsAttr(False)
ps.CreateBroadphaseTypeAttr('MBP')
ps.CreateTimeStepsPerSecondAttr(200)


def gains(name):
    if 'ankle' in name:
        return 20., 4., 100.
    if 'hip_yaw' in name or 'hip_roll' in name:
        return 150., 5., 300.
    if any(s in name for s in ('hip_pitch', 'knee', 'torso')):
        return 200., 5., 300.
    if 'shoulder' in name or 'elbow' in name:
        return 40., 10., 300.
    return 30., 1., 30.


body_paths = []
collider_count = 0
for prim in Usd.PrimRange(robot_prim):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        props = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        props.CreateEnabledSelfCollisionsAttr(False)
        props.CreateSolverPositionIterationCountAttr(4)
        props.CreateSolverVelocityIterationCountAttr(4)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        collider_count += 1
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        body_paths.append(str(prim.GetPath()))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.)
        rb.CreateLinearDampingAttr(0.)
        rb.CreateAngularDampingAttr(0.)
    if prim.IsA(UsdPhysics.RevoluteJoint):
        stiffness, damping, force = gains(prim.GetName())
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateStiffnessAttr(stiffness * math.pi / 180.)
        drive.CreateDampingAttr(damping * math.pi / 180.)
        drive.CreateMaxForceAttr(force)
if collider_count == 0:
    raise RuntimeError('No physical robot colliders')

world = World(physics_dt=.005, rendering_dt=1/24, stage_units_in_meters=1.,
              physics_prim_path='/World/PhysicsScene', backend='numpy')
robot = world.scene.add(SingleArticulation(robot_path, name='keti_h1'))
contact = world.scene.add(RigidPrim(body_paths, name='obstacle_contact_audit', reset_xform_properties=False,
            track_contact_forces=True, contact_filter_prim_paths_expr=[[obstacle['path']] for _ in body_paths],
            max_contact_count=512))
world.reset()
if not contact._contact_view.is_physics_handle_valid():
    raise RuntimeError('Contact sensor not initialized; do not execute')
ids = [robot.get_dof_index(name) for name in constants['BODY_19']]
defaults = np.array(constants['WALK_DEFAULT_19'], dtype=np.float32)
targets = np.zeros(robot.num_dof, dtype=np.float32)
targets[ids] = defaults
robot.set_joint_positions(targets)
robot.set_joint_velocities(np.zeros_like(targets))
robot.set_world_velocity(np.zeros(6))  # initialization only; never called in the motion loop
kp, kd = robot.get_articulation_controller().get_gains()
expected = np.array([gains(name)[:2] for name in robot.dof_names])
if not np.allclose(kp, expected[:, 0], rtol=1e-5) or not np.allclose(kd, expected[:, 1], rtol=1e-5):
    raise RuntimeError('Actual PD gains differ from frozen walking policy')
(out / 'robot_audit.json').write_text(json.dumps({'joint_names': robot.dof_names,
        'colliders': collider_count, 'contact_body_count': len(body_paths), 'self_collisions': False,
        'gains_match_training': True, 'body_order': constants['BODY_19']}, indent=2), encoding='utf-8')
walker = torch.jit.load(str(walker_path), map_location='cpu').eval()
terminal_actor = None
if args.terminal_controller == 'rack_rl':
    if hashlib.sha256(nav_path.read_bytes()).hexdigest() != '56fb8ddaec05da9342d0acaf0160048cd8361efc7b5b6525677c096dd06f49a9':
        raise RuntimeError('Rack actor checkpoint mismatch')
    payload = torch.load(str(nav_path), map_location='cpu', weights_only=True)
    terminal_actor = torch.nn.Sequential(torch.nn.Linear(18,128),torch.nn.ELU(),
            torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,3))
    terminal_actor.load_state_dict({k.removeprefix('actor.'):v for k,v in payload['model_state_dict'].items()
                                   if k.startswith('actor.')}, strict=True)
    terminal_actor.eval()


def inv_rotate(q, v):
    qv, v = np.asarray(q[1:]), np.asarray(v)
    uv = np.cross(qv, v)
    return v - 2 * q[0] * uv + 2 * np.cross(qv, uv)


def measure():
    p, q = robot.get_world_pose()
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z))
    return dict(pos=np.array(p), quat=np.array(q), yaw=yaw, lin=inv_rotate(q, robot.get_linear_velocity()),
                ang=inv_rotate(q, robot.get_angular_velocity()), gravity=inv_rotate(q, [0, 0, -1]))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


writers, products, annotators = [], [], []
if not args.no_video:
    import omni.replicator.core as rep
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont
    rep.orchestrator.set_capture_on_play(False)
    camera = UsdGeom.Camera.Define(stage, '/World/KetiNav/Camera')
    camera.CreateFocalLengthAttr(18.)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(20.955 * 9 / 16)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 200))
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(7.7, -3.5, 4.4),
            Gf.Vec3d(7.7, 3.8, .65), Gf.Vec3d(0, 0, 1)).GetInverse())
    product = rep.create.render_product(str(camera.GetPath()), (1280, 720))
    annotator = rep.AnnotatorRegistry.get_annotator('rgb')
    annotator.attach([product])
    products.append(product)
    annotators.append(annotator)
    writers.append(imageio.get_writer(str(out / 'keti_detour_720p.mp4'), fps=24, codec='libx264', quality=8, macro_block_size=None))
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 20)
    small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 16)
    rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=False)

velocity = np.zeros(3, dtype=np.float32)
last = np.zeros(19, dtype=np.float32)
progress = 0.
terminal_latched = False
stance_hold = None
stance_capture = None
foot_ids = [body_paths.index(robot_path+'/'+name) for name in ('left_ankle_link','right_ankle_link')]
phase, failure, success = 'SETTLE', None, False
dwell, frames, next_frame = 0., 0, 0.
records, trace_points = [], []
max_contact, min_clearance, traveled, previous = 0., float('inf'), 0., None
obstacles = scenario_set['obstacles'] + [obstacle]


def annotate(raw, state, t):
    img = Image.fromarray(raw[:, :, :3])
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, 1280, 82), fill=(9, 18, 30))
    draw.text((18, 8), f'KETI | H1 obstacle detour | seed {args.seed} | {phase} | t={t:.2f}s', fill='white', font=font)
    draw.text((18, 36), 'USD map (not LiDAR) -> A* -> velocity tracking -> frozen RL walking', fill=(166, 212, 245), font=small)
    draw.text((18, 58), f'Goal {np.linalg.norm(state["pos"][:2]-goal):.2f}m | obstacle contact {max_contact:.2f}N | no teleport', fill='white', font=small)
    # Plan/actual trace in a small world-coordinate map, not a fabricated robot animation.
    x0, y0, x1, y1 = 948, 444, 1260, 700
    draw.rectangle((x0, y0, x1, y1), fill=(12, 23, 35), outline=(105, 132, 151), width=2)
    bounds = config['bounds_xy']
    def px(p):
        return (x0 + 12 + (p[0]-bounds[0])/(bounds[2]-bounds[0])*(x1-x0-24),
                y1 - 30 - (p[1]-bounds[1])/(bounds[3]-bounds[1])*(y1-y0-60))
    draw.text((x0+12, y0+8), 'Plan / measured root trajectory', fill='white', font=small)
    for item in obstacles:
        lo, hi = item['min'], item['max']
        if hi[0] < bounds[0] or lo[0] > bounds[2] or hi[1] < bounds[1] or lo[1] > bounds[3]:
            continue
        a, b = px([max(bounds[0],lo[0]), min(bounds[3],hi[1])]), px([min(bounds[2],hi[0]), max(bounds[1],lo[1])])
        draw.rectangle((*a, *b), fill=(229, 105, 37) if item['path']==obstacle['path'] else (87, 94, 103))
    draw.line([px(start), px(goal)], fill=(230, 68, 65), width=2)
    draw.line([px(p) for p in plan], fill=(62, 226, 112), width=3)
    if len(trace_points)>1:
        draw.line([px(p) for p in trace_points], fill=(72, 178, 255), width=3)
    p = px(state['pos'][:2]); draw.ellipse((p[0]-4,p[1]-4,p[0]+4,p[1]+4), fill='white')
    draw.text((x0+12,y1-24), 'RED direct  GREEN plan  BLUE actual', fill='white', font=small)
    return img


try:
    for step in range(round(config['duration_s'] / .005)):
        t = step * .005
        state = measure()
        p, yaw = state['pos'], state['yaw']
        distance = float(np.linalg.norm(p[:2] - goal))
        force = np.asarray(contact.get_contact_force_matrix(dt=.005))
        contact_N = float(np.linalg.norm(force.reshape(-1, 3), axis=1).max(initial=0))
        max_contact = max(max_contact, contact_N)
        if contact_N > config['obstacle_contact_failure_N']:
            failure = 'CONTACT_WITH_RANDOM_OBSTACLE'
        if p[2] < .65 or -state['gravity'][2] < math.cos(math.radians(40)):
            failure = 'FALL'
        if step % 20 == 0:
            feet = np.asarray(contact.get_net_contact_forces(dt=.005))[foot_ids,2]
            body_clearance = clearance(p[:2], obstacles) - config['runtime_footprint_radius_m']
            min_clearance = min(min_clearance, body_clearance)
            if body_clearance <= 0:
                failure = 'FOOTPRINT_EXCLUSION'
            target, progress = lookahead_target(p[:2], plan, progress, config['lookahead_m'])
            desired = np.zeros(3)
            if t >= config['settle_s']:
                phase = 'DETOUR'
                delta = target - p[:2]
                heading_error = wrap(math.atan2(delta[1], delta[0]) - yaw)
                desired[0] = min(config['max_forward_mps'], distance * .65) * max(0., math.cos(heading_error)) ** 2
                desired[2] = np.clip(1.6 * heading_error, -config['max_yaw_rate_rps'], config['max_yaw_rate_rps'])
                if distance < config['arrival_distance_m']:
                    phase = 'STOP'
                    desired[0] = 0.
                    desired[2] = np.clip(1.6 * wrap(math.radians(config['goal_yaw_deg']) - yaw), -.3, .3)
                if terminal_actor is not None and (terminal_latched or distance < .8):
                    terminal_latched = True
                    phase = 'LEARNED_DOCK'
                    delta = goal - p[:2]
                    local_delta = np.array([math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1],
                                           -math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]])
                    heading = wrap(math.radians(config['goal_yaw_deg']) - yaw)
                    obs = np.r_[state['lin'], state['ang'], state['gravity'], local_delta,
                                math.sin(heading), math.cos(heading), velocity, dwell, distance].astype(np.float32)
                    with torch.inference_mode():
                        action = torch.tanh(terminal_actor(torch.from_numpy(obs).unsqueeze(0))).numpy()[0]
                    desired = np.array([np.clip(action[0],0,1)*.6, action[1]*.2, action[2]*.45])
                if args.terminal_controller in ('measured_support_hold', 'commanded_support_hold'):
                    eligible = (distance < config['arrival_distance_m'] and np.all(feet > 50.)
                                and abs(wrap(math.radians(config['goal_yaw_deg'])-yaw)) < math.radians(10)
                                and np.linalg.norm(state['lin'][:2]) < .20
                                and -state['gravity'][2] > math.cos(math.radians(15)))
                    if stance_hold is None and eligible:
                        measured_q = robot.get_joint_positions().copy()
                        stance_hold = (targets.copy() if args.terminal_controller == 'commanded_support_hold' else measured_q)
                        stance_capture = {'time_s':t,'measured_joint_positions':measured_q.tolist(),
                                          'held_joint_targets':stance_hold.tolist(),
                                          'foot_vertical_contact_N':feet.tolist(),'position_world':p.tolist()}
                    if stance_hold is not None:
                        desired[:] = 0.
                        phase = 'DOUBLE_SUPPORT_HOLD'
            limit = np.array([config['linear_acceleration_mps2'], config['linear_acceleration_mps2'], config['angular_acceleration_rps2']]) * .1
            velocity += np.clip(desired - velocity, -limit, limit)
            stable = (distance < config['arrival_distance_m'] and np.linalg.norm(state['lin'][:2]) < config['arrival_speed_mps']
                      and abs(wrap(math.radians(config['goal_yaw_deg'])-yaw)) < math.radians(config['arrival_yaw_deg'])
                      and abs(state['ang'][2]) < config['arrival_yaw_rate_rps']
                      and -state['gravity'][2] > math.cos(math.radians(15)))
            dwell = dwell + .1 if stable else 0.
            if dwell >= config['arrival_hold_s'] - 1e-8:
                success, phase = True, 'SUCCESS'
            if previous is not None:
                traveled += float(np.linalg.norm(p[:2] - previous))
            previous = p[:2].copy()
            trace_points.append(p[:2].tolist())
            row = {'time_s': t, 'phase': phase, 'position_world': p.tolist(), 'yaw_deg': math.degrees(yaw),
                   'goal_distance_m': distance, 'speed_mps': float(np.linalg.norm(state['lin'][:2])),
                   'velocity_command': velocity.tolist(), 'progress_m': progress, 'contact_N': contact_N,
                   'footprint_proxy_clearance_m': body_clearance, 'dwell_s': dwell}
            row['foot_vertical_contact_N'] = feet.tolist()
            records.append(row)
            if step % 200 == 0:
                print('KETI_PROGRESS', json.dumps(row), flush=True)
        if step % 4 == 0:
            observation = np.r_[state['lin'], state['ang'], state['gravity'], velocity,
                                robot.get_joint_positions()[ids] - defaults,
                                robot.get_joint_velocities()[ids], last].astype(np.float32)
            with torch.inference_mode():
                last = walker(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
            if not np.isfinite(last).all():
                raise RuntimeError('NONFINITE_POLICY_ACTION')
            targets[ids] = defaults + .5 * last
        if stance_hold is not None:
            targets[:] = stance_hold
        robot.apply_action(ArticulationAction(joint_positions=targets))
        world.step(render=False)
        if not args.no_video and (t + 1e-9 >= next_frame or failure or success):
            rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False)
            raw = np.asarray(annotators[0].get_data())
            if raw.ndim != 3 or raw.shape[:2] != (720,1280):
                raise RuntimeError('Invalid 720p frame')
            img = annotate(raw, state, t)
            writers[0].append_data(np.asarray(img))
            if frames % 48 == 0 or failure or success:
                img.save(out / f'frame_{frames:04d}.jpg')
            frames += 1
            next_frame = frames / 24
        if failure or success:
            break
    if not success and not failure:
        failure = 'TIMEOUT'
    unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p,h in scenario_set['source_sha256'].items())
    result = {'pass': success and failure is None and unchanged, 'failure': failure, 'seed': args.seed,
              'time_s': t, 'frames': frames, 'goal_distance_m': distance, 'traveled_m': traveled,
              'direct_distance_m': float(np.linalg.norm(goal-start)), 'planned_length_m': scenario['plan']['length_m'],
              'max_random_obstacle_contact_N': max_contact, 'min_footprint_proxy_clearance_m': min_clearance,
              'dynamic_H1': True, 'root_writes_during_motion': False, 'obstacle_moved_during_motion': False,
              'map_source': scenario_set['map_source'], 'source_world_unchanged': unchanged,
              'new_RL_training': False, 'self_collision_validated': False,
              'terminal_controller': args.terminal_controller,
              'measured_stance_capture': stance_capture,
              'scope': 'One static-oracle-map detour rollout, not general sensor navigation success rate'}
    (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False), encoding='utf-8')
    (out/'telemetry.json').write_text(json.dumps(records,indent=2,allow_nan=False), encoding='utf-8')
    print('KETI_RESULT',json.dumps(result),flush=True)
except BaseException:
    trace = traceback.format_exc()
    print(trace,flush=True)
    (out/'exception.txt').write_text(trace,encoding='utf-8')
    (out/'telemetry_partial.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    raise
finally:
    for writer in writers:
        writer.close()
    for product in products:
        product.destroy()
    world.stop()
    app.close()
if not success or failure:
    raise SystemExit(2)
