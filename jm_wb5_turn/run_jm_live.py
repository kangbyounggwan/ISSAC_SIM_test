"""Persistent local JM/H1 teleoperation session. No hardware, teleport, or training.

The verified fixed mission runner is separate and unchanged. Paused means that
physics time is paused, not that a zero-command H1 has proven stationary feet.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from jm_wb5_turn.live_command import decode, slew
from jm_wb5_turn.operator_bridge import OperatorBridge

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--session', required=True)
args = p.parse_args()
out = args.out.resolve()
out.mkdir(parents=True, exist_ok=True)
bridge = OperatorBridge(out, True)
started = time.time()


def write_status(state, **data):
    bridge._replace(out/'status.json', lambda f: f.write_text(json.dumps(dict(
        session=args.session, state=state, wall_time_unix=time.time(),
        process_id=os.getpid(), scope='local Isaac Sim only', **data)), encoding='utf-8'))


write_status('loading', phase='INITIALIZING')
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 1280, 'height': 720})
sensors = None
log = None
failure = None
try:
    import numpy as np
    import torch
    import omni.usd
    import omni.client
    import omni.replicator.core as rep
    import carb.settings
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    from isaacsim.core.api import World
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    from isaacsim.robot.policy.examples.controllers.config_loader import get_robot_joint_properties
    from isaacsim.storage.native import get_assets_root_path
    from jm_xnavdp.spawn import sample_spawn
    from jm_xnavdp.contact_audit import ContactAudit
    from keti_navigation.planner import clearance
    from jm_wb5_turn.operator_sensors import OperatorSensors

    meta = json.loads((ROOT/'artifacts/xnavdp_20260929/jm_capture_002/observation.json').read_text())
    scene = json.loads((ROOT/'artifacts/jm_stop_pose_20260929/scene_001.json').read_text())
    for file, digest in meta['source_sha256'].items():
        if hashlib.sha256(Path(file).read_bytes()).hexdigest() != digest:
            raise RuntimeError('JM source changed: '+file)
    floor = float(scene['floor_z'])
    obstacles = [o for o in scene['obstacles'] if o['max'][2] > floor+.15 and o['min'][2] < floor+1.75]
    spawn = sample_spawn(obstacles, 2159858089)['selected_xy']
    torch.set_num_threads(1)
    carb.settings.get_settings().set('/rtx/post/histogram/enabled', False)
    ctx = omni.usd.get_context()
    ctx.open_stage(meta['world'])
    for _ in range(10): app.update()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for name in ('/World/H1Fleet', '/World/Robots'):
        group = stage.GetPrimAtPath(name)
        if group: group.SetActive(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
    physics = stage.GetPrimAtPath('/World/PhysicsScene')
    if not physics: physics = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene').GetPrim()
    ps = PhysxSchema.PhysxSceneAPI.Apply(physics)
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr('MBP')
    ps.CreateTimeStepsPerSecondAttr(200)
    world = World(physics_dt=.005, rendering_dt=.04, stage_units_in_meters=1.,
                  physics_prim_path='/World/PhysicsScene', backend='numpy')
    asset_root = get_assets_root_path()
    usd = asset_root+'/Isaac/Robots/Unitree/H1/h1.usd'
    robot_path = '/World/JMLive/H1'
    reference = H1FlatTerrainPolicy(robot_path, usd_path=usd,
        position=np.array([*spawn, floor+1.05]),
        orientation=np.array([math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)]))
    robot = world.scene.add(reference.robot)
    body = None
    for node in ast.parse((ROOT/'jm_coop4_rl/h1_rack_env.py').read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'BODY_19':
            body = ast.literal_eval(node.value)
    native = [n.removesuffix('_joint') for n in body]
    contract = get_robot_joint_properties(reference.policy_env_params, native)
    rp = stage.GetPrimAtPath(robot_path)
    for prim in Usd.PrimRange(rp):
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in native:
            j = native.index(prim.GetName())
            drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
            drive.CreateStiffnessAttr(contract[2][j]*math.pi/180)
            drive.CreateDampingAttr(contract[3][j]*math.pi/180)
            drive.CreateMaxForceAttr(contract[0][j])
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(math.degrees(contract[1][j]))
    contacts = ContactAudit(stage, robot_path, out)
    ground = '/World/Dadong/F2/slab'
    if not stage.GetPrimAtPath(ground): raise RuntimeError('JM ground collider missing')
    sensors = OperatorSensors(stage, robot_path, out, bridge)
    world.reset()
    if robot.dof_names != native: raise RuntimeError('Native joint order mismatch')
    assert reference._dt == .005 and reference._decimation == 4
    defaults = np.asarray(contract[4], dtype=np.float32)
    controller = robot.get_articulation_controller()
    controller.set_effort_modes('force')
    controller.switch_control_mode('position')
    robot.set_joint_positions(defaults)
    robot.set_joint_velocities(np.zeros(19))
    robot.set_world_velocity(np.zeros(6))
    official = bytes(omni.client.read_file(asset_root+'/Isaac/Samples/Policies/H1_Policies/h1_policy.pt')[2])
    policy_hash = hashlib.sha256(official).hexdigest()
    if policy_hash != 'a734d8ae13b8c3690138baf8d01d7d497f987616da2c53e00a43e0ceb31452d7':
        raise RuntimeError('Official policy differs from verified baseline')
    (out/'manifest.json').write_text(json.dumps(dict(session=args.session, world=meta['world'],
        policy_sha256=policy_hash, dt=.005, decimation=4, initial_spawn=spawn, source_sha256=meta['source_sha256'],
        physical_root_writes_during_motion=0, sensor_resolution=[1280,720], target_preview_fps=10,
        command_limits=dict(forward=.18, reverse=.10, lateral=0., yaw=.25),
        scope='Native H1 simulator, no WB5 checkpoint, no gripper, no training, no real robot'), indent=2))
    camera = UsdGeom.Camera.Define(stage, '/World/JMLive/Camera')
    camera.CreateFocalLengthAttr(27.)
    camera.CreateHorizontalApertureAttr(30.)
    camera.CreateVerticalApertureAttr(30*9/16)
    center = np.array([spawn[0]-.5, spawn[1]+.5, floor+.9])
    eye = center+np.array([4., -5.5, 3.])
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0,0,1)).GetInverse())
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()), (1280,720))
    ann = rep.AnnotatorRegistry.get_annotator('rgb'); ann.attach([product])
    clock = [0.]
    world.add_physics_callback('live_clock', lambda dt: clock.__setitem__(0, clock[0]+dt))
    rep.orchestrator.step(rt_subframes=3, delta_time=0., pause_timeline=False)
    clock_start = clock[0]
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 19)
    last = np.zeros(19, dtype=np.float32)
    cmd = [0.,0.,0.]
    frame_sequence = 0; frame_wall = None; last_capture = 0.; last_log = 0.
    frame_times = []; t = 0.; max_force = 0.; view = 'third'; state = 'paused'
    log = (out/'telemetry.jsonl').open('a', encoding='utf-8')
    ready_at = time.perf_counter(); last_tick = ready_at; physics_due = ready_at
    status = {}
    while app.is_running() and not (out/'stop.request').exists():
        now = time.time()
        try: payload = json.loads((out/'command.json').read_text())
        except (OSError, ValueError): payload = None
        control = decode(payload, args.session, now)
        view = control['view']
        if failure: control['enabled'] = False
        pos,q = robot.get_world_pose()
        R = quat_to_rot_matrix(q).T
        lin = R@robot.get_linear_velocity(); ang = R@robot.get_angular_velocity()
        grav = R@np.array([0.,0.,-1.])
        w,x,y,z = map(float,q)
        yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        tilt = math.degrees(math.acos(np.clip(-grav[2],-1.,1.)))
        if not np.isfinite(np.r_[pos,q,lin,ang]).all(): failure = 'NONFINITE_STATE'
        if tilt >= 15.: failure = 'POSTURE_ABORT'
        if pos[2] < floor+.65: failure = 'ROOT_HEIGHT_ABORT'
        if clearance(pos[:2],obstacles) < .38: failure = 'ENVIRONMENT_ENVELOPE_ABORT'
        if math.dist(pos[:2],spawn) > 2.: failure = 'LOCAL_RADIUS_ABORT'
        if failure: state = 'failed'
        elif control['enabled']: state = 'running'
        else: state = 'paused'
        if state == 'running':
            # Catch up at most 100ms of physics between images, still 200Hz physics / 50Hz policy.
            ticks = min(5, max(1, int((time.perf_counter()-physics_due)/.02)))
            for _tick in range(ticks):
                if not decode(payload,args.session,time.time())['enabled']: break
                pos,q = robot.get_world_pose();R = quat_to_rot_matrix(q).T
                lin = R@robot.get_linear_velocity();ang = R@robot.get_angular_velocity();grav = R@np.array([0.,0.,-1.])
                if math.degrees(math.acos(np.clip(-grav[2],-1.,1.)))>=15.: failure='POSTURE_ABORT'
                if pos[2]<floor+.65: failure='ROOT_HEIGHT_ABORT'
                if clearance(pos[:2],obstacles)<.38: failure='ENVIRONMENT_ENVELOPE_ABORT'
                if failure: break
                cmd = slew(cmd, control['velocity'])
                obs = np.r_[lin,ang,grav,cmd,robot.get_joint_positions()-defaults,robot.get_joint_velocities(),last].astype(np.float32)
                with torch.inference_mode(): last = reference.policy(torch.from_numpy(obs)[None]).numpy()[0]
                if not np.isfinite(last).all(): raise RuntimeError('Nonfinite policy action')
                targets = defaults+.5*last
                for _ in range(4):
                    robot.apply_action(ArticulationAction(joint_positions=targets))
                    world.step(render=False);t += .005
                    contacts.sample(t)
                    for pair in contacts.pairs.values():
                        if abs(pair['last_s']-t) > 1e-7: continue
                        a,b = pair['colliders']
                        if a.startswith(robot_path+'/') and b.startswith(robot_path+'/'): continue
                        other = b if a.startswith(robot_path+'/') else a
                        if other.startswith(ground): continue
                        force = pair['max_impulse_Ns']/.005;max_force = max(max_force,force)
                        if force > .20: failure = 'ENVIRONMENT_CONTACT_ABORT'
                physics_due += .02
            physics_due = max(physics_due,time.perf_counter()-.1)
            if abs(clock[0]-clock_start-t) > .01: raise RuntimeError('Physics clock mismatch')
        else:
            cmd = [0.,0.,0.]
            physics_due = time.perf_counter()
        if t>=600.: failure='ACTIVE_TIME_LIMIT_600S'
        if failure: state='failed';cmd=[0.,0.,0.]
        if time.perf_counter()-last_capture >= .1:
            last_capture = time.perf_counter()
            pos,q=robot.get_world_pose();R=quat_to_rot_matrix(q).T
            lin=R@robot.get_linear_velocity();grav=R@np.array([0.,0.,-1.])
            w,x,y,z=map(float,q);yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            tilt=math.degrees(math.acos(np.clip(-grav[2],-1.,1.)))
            before = clock[0]
            rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False)
            if abs(clock[0]-before) > 1e-9: raise RuntimeError('Live rendering advanced physics')
            try:
                image = Image.fromarray(np.asarray(ann.get_data())[:,:,:3].copy()) if view == 'third' else sensors.preview(view)
                if image.size != (1280,720): raise ValueError('Unexpected frame size')
                d = ImageDraw.Draw(image);d.rectangle((0,0,1280,66),fill=(8,18,28))
                d.text((14,7),f'JM LIVE | {view.upper()} | {state.upper()} | SIM t={t:.2f}s',font=font,fill='white')
                d.text((14,35),'Native H1 | physical policy | NO teleport | SIMULATION ONLY',font=font,fill=(140,220,240))
                bridge._replace(out/f'live-{view}.jpg', lambda f:image.save(f,format='JPEG',quality=87))
                frame_wall = time.time();frame_sequence += 1
                frame_times.append(frame_wall);frame_times = frame_times[-30:]
                camera_error = None
            except Exception as exc:
                camera_error = str(exc)
            fps = (len(frame_times)-1)/(frame_times[-1]-frame_times[0]) if len(frame_times)>1 else 0.
            status = dict(phase=state.upper(),time_s=t,position_world=pos.tolist(),yaw_deg=math.degrees(yaw),
                tilt_deg=tilt,speed_mps=float(np.linalg.norm(lin[:2])),command=cmd,requested=control['velocity'],
                command_reason=control['reason'],command_sequence=control['sequence'],view=view,
                frame_sequence=frame_sequence,frame_wall_time_unix=frame_wall,preview_fps=fps,camera_error=camera_error,
                realtime_factor=t/max(.001,time.perf_counter()-ready_at),failure=failure,
                max_contact_proxy_n=max_force,physics_paused=state!='running',physical_root_teleport=False)
            write_status(state,**status)
        if time.time()-last_log >= .1 and status:
            log.write(json.dumps(dict(status,wall_time_unix=time.time()))+'\n');log.flush();last_log = time.time()
        # Do not run faster than real time. Rendering may make the measured rate slower.
        target = last_tick+.02
        if time.perf_counter()<target: time.sleep(target-time.perf_counter())
        last_tick = time.perf_counter()
    write_status('failed' if failure else 'stopped', **dict(status,failure=failure))
    contacts.save()
    print('JM_LIVE_RESULT', json.dumps(status), flush=True)
except Exception:
    (out/'exception.txt').write_text(traceback.format_exc())
    write_status('failed', failure='RUNTIME_EXCEPTION')
    raise
finally:
    if log: log.close()
    if sensors: sensors.close()
    app.close()
