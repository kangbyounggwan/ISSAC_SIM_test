"""Isolated, reproducible H1 turn comparison; never modifies source assets.

Run each combination in a fresh process. Model-only comparisons use the same
explicit WB5 deployment contract. The official baseline uses its bundled env.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--policy', choices=['official', 'wb5'], required=True)
p.add_argument('--asset', choices=['official', 'jm'], required=True)
p.add_argument('--contract', choices=['official', 'wb5'], default='wb5')
p.add_argument('--sign', type=int, choices=[-1, 1], required=True)
p.add_argument('--audit-feet',action='store_true',help='50Hz foot contact/pose audit and 25fps 720p recording; contact helper sets foot sleep thresholds to zero')
p.add_argument('--render-retries',type=int,choices=range(1,21),default=3,
    help='Bounded rerender attempts at the SAME physics state for a dark frame; never duplicate a previous frame')
p.add_argument('--ground-restitution',type=float,choices=[0.,.8],default=.8,
               help='Single-variable ground response audit; .8 is the official example API default')
p.add_argument('--initial-root-z',type=float,choices=[.98,1.05],default=1.05,
               help='Initial spawn only: .98 from WB5 training source, 1.05 from NVIDIA test')
p.add_argument('--out', type=Path, required=True)
args = p.parse_args()
args.out.mkdir(parents=True, exist_ok=False)
out = args.out.resolve()

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 1280, 'height': 720})
writer = None
rows = []
try:
    import numpy as np
    import torch
    import omni.usd
    import omni.client
    import omni.replicator.core as rep
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    from PIL import Image, ImageDraw, ImageFont
    import imageio.v2 as imageio
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    from isaacsim.robot.policy.examples.controllers.config_loader import get_robot_joint_properties
    from isaacsim.storage.native import get_assets_root_path
    from jm_xnavdp.turn_audit import audit_turn

    torch.set_num_threads(1)
    constants = {}
    for node in ast.parse((ROOT/'jm_coop4_rl/h1_rack_env.py').read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id in {'BODY_19', 'WALK_DEFAULT_19'}:
            constants[node.targets[0].id] = ast.literal_eval(node.value)
    body = list(constants['BODY_19'])
    asset_root = get_assets_root_path()
    official_usd = asset_root + '/Isaac/Robots/Unitree/H1/h1.usd'
    usd = official_usd if args.asset == 'official' else str(ROOT/'jm_wb7_pinkik/assets/h1_wb7_physical_grasp_pad12_dual_mimic.usd')
    world = World(physics_dt=.005, rendering_dt=1/24, stage_units_in_meters=1.)
    world.scene.add_default_ground_plane(restitution=args.ground_restitution)
    stage = omni.usd.get_context().get_stage()
    reference = H1FlatTerrainPolicy('/World/H1', usd_path=usd, position=np.array([0., 0., args.initial_root_z]))
    # Do not initialize the official controller on the 33-DoF modified robot:
    # its loader assumes exactly 19 actuated joints. Map by joint name instead.
    robot = world.scene.add(reference.robot)
    prim = stage.GetPrimAtPath('/World/H1')
    mimics = []
    def gains(n):
        if 'ankle' in n: return 20., 4., 100.
        if 'hip_yaw' in n or 'hip_roll' in n: return 150., 5., 300.
        if any(s in n for s in ('hip_pitch', 'knee', 'torso')): return 200., 5., 300.
        if 'shoulder' in n or 'elbow' in n: return 40., 10., 300.
        return 30., 1., 30.
    official_contract = get_robot_joint_properties(reference.policy_env_params, [n.removesuffix('_joint') for n in body])
    if args.contract == 'wb5':
        for item in Usd.PrimRange(prim):
            if item.HasAPI(UsdPhysics.ArticulationRootAPI):
                a = PhysxSchema.PhysxArticulationAPI.Apply(item)
                a.CreateEnabledSelfCollisionsAttr(False)
                a.CreateSolverPositionIterationCountAttr(4)
                a.CreateSolverVelocityIterationCountAttr(4)
            if item.HasAPI(UsdPhysics.RigidBodyAPI):
                r = PhysxSchema.PhysxRigidBodyAPI.Apply(item)
                r.CreateMaxDepenetrationVelocityAttr(1.)
                r.CreateLinearDampingAttr(0.)
                r.CreateAngularDampingAttr(0.)
    for item in Usd.PrimRange(prim):
        if item.HasAPI(PhysxSchema.PhysxMimicJointAPI):
            mimics.append(item.GetName())
        if item.IsA(UsdPhysics.RevoluteJoint):
            name=item.GetName()
            k,d,f=gains(name)
            canonical=name if name.endswith('_joint') else name+'_joint'
            if args.contract=='official' and canonical in body:
                j=body.index(canonical)
                k,d,f=official_contract[2][j],official_contract[3][j],official_contract[0][j]
                PhysxSchema.PhysxJointAPI.Apply(item).CreateMaxJointVelocityAttr(math.degrees(official_contract[1][j]))
            if name in mimics: k=d=f=0.
            # Persist in this in-memory stage BEFORE initializing physics. Runtime
            # gain setters alone were overwritten by the next USD/Fabric sync.
            drive=UsdPhysics.DriveAPI.Apply(item,'angular')
            drive.CreateStiffnessAttr(k*math.pi/180)
            drive.CreateDampingAttr(d*math.pi/180)
            drive.CreateMaxForceAttr(f)
    foot_views=[];torso_view=None
    if args.audit_feet:
        from isaacsim.core.prims import RigidPrim
        # Keep contact stabilization and transforms unchanged. The SDK contact
        # helper sets foot rigid-body sleep thresholds to zero for reporting.
        for side in ('left','right'):
            foot_views.append(world.scene.add(RigidPrim('/World/H1/'+side+'_ankle_link',
                name=side+'_foot_audit',reset_xform_properties=False,
                track_contact_forces=True,disable_stablization=False)))
        torso_view=world.scene.add(RigidPrim('/World/H1/torso_link',name='torso_audit',
            reset_xform_properties=False,prepare_contact_sensors=False))
    world.reset()
    print('NATIVE_DOF_NAMES', robot.dof_names, flush=True)
    (out/'native_dof_names.json').write_text(json.dumps(robot.dof_names))
    native_body = [n if n in robot.dof_names else n.removesuffix('_joint') for n in body]
    ids = [robot.get_dof_index(n) for n in native_body]
    official = get_robot_joint_properties(reference.policy_env_params, native_body)
    defaults = np.asarray(official[4] if args.contract == 'official' else constants['WALK_DEFAULT_19'], dtype=np.float32)
    targets = np.zeros(robot.num_dof, dtype=np.float32)
    targets[ids] = defaults
    kp, kd, effort = (np.array([gains(n)[i] for n in robot.dof_names]) for i in range(3))
    if args.contract == 'official':
        kp[ids], kd[ids], effort[ids] = official[2], official[3], official[0]
    for i, n in enumerate(robot.dof_names):
        if n in mimics: kp[i] = kd[i] = effort[i] = 0.
    robot.get_articulation_controller().set_effort_modes('force')
    robot.get_articulation_controller().switch_control_mode('position')
    robot.set_joint_positions(targets)
    robot.set_joint_velocities(np.zeros_like(targets))
    robot.set_world_velocity(np.zeros(6))
    command_ids = [i for i,n in enumerate(robot.dof_names) if n not in mimics]
    policy_path = ROOT/'artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt'
    policy = reference.policy if args.policy == 'official' else torch.jit.load(str(policy_path), map_location='cpu').eval()
    official_url = asset_root + '/Isaac/Samples/Policies/H1_Policies/h1_policy.pt'
    official_bytes = bytes(omni.client.read_file(official_url)[2])
    # Trusted official artifact copied for reproducible local comparisons.
    (out/'official_policy.pt').write_bytes(official_bytes)
    official_env_url = asset_root + '/Isaac/Samples/Policies/H1_Policies/h1_env.yaml'
    (out/'official_env.yaml').write_bytes(bytes(omni.client.read_file(official_env_url)[2]))
    inventory = []
    for item in Usd.PrimRange(prim):
        if item.HasAPI(UsdPhysics.RigidBodyAPI) or item.IsA(UsdPhysics.RevoluteJoint):
            inventory.append({'path': str(item.GetPath()), 'type': item.GetTypeName(),
                'physics': {a.GetName(): str(a.Get()) for a in item.GetAttributes() if a.GetName().startswith(('physics:', 'physx', 'drive:'))}})
    manifest = dict(arguments=vars(args), usd=usd, policy_sha256=hashlib.sha256(official_bytes if args.policy=='official' else policy_path.read_bytes()).hexdigest(),
        body_order=body, actual_dof_order=robot.dof_names, joint_defaults=defaults.tolist(), stiffness=kp.tolist(), damping=kd.tolist(),
        official_defaults=official[4], official_stiffness=official[2], official_damping=official[3], official_effort=official[0],
        official_dt=reference._dt, official_decimation=reference._decimation, dt=.005, decimation=4,
        physics_root_writes_during_motion=False, drive_authoring='stage_before_physics_initialization', experiment='2s zero + 10s yaw +/-0.30 + 2s zero; explicit initial spawn and ground contract',
        source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    manifest['foot_audit_enabled']=args.audit_feet
    manifest['contact_stabilization_not_disabled']=True
    if args.audit_feet:
        manifest['contact_preparation']='RigidPrim contact reporting sets foot sleep thresholds to zero; do not claim sensor-free physical parity'
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, default=str))
    (out/'asset_inventory.json').write_text(json.dumps(inventory, indent=2))
    if args.asset == 'official' and robot.dof_names != native_body:
        raise RuntimeError('Official policy native joint order differs; mapping must be reviewed')
    assert reference._dt == .005 and reference._decimation == 4
    camera = UsdGeom.Camera.Define(stage, '/World/CompareCamera')
    camera.CreateFocalLengthAttr(23.)
    camera.CreateHorizontalApertureAttr(30.)
    camera.CreateVerticalApertureAttr(30*9/16)
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(3.5,-4.5,2.6), Gf.Vec3d(.1,0,.9), Gf.Vec3d(0,0,1)).GetInverse())
    from pxr import UsdLux
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(1500.)
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()), (1280,720))
    ann = rep.AnnotatorRegistry.get_annotator('rgb')
    ann.attach([product])
    physics_time = [0.]
    world.add_physics_callback('comparison_clock', lambda dt: physics_time.__setitem__(0,physics_time[0]+dt))
    rep.orchestrator.step(rt_subframes=3, delta_time=0., pause_timeline=False)
    actual_kp, actual_kd = robot.get_articulation_controller().get_gains()
    manifest['measured_stiffness'] = np.asarray(actual_kp).tolist()
    manifest['measured_damping'] = np.asarray(actual_kd).tolist()
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, default=str))
    if not np.allclose(np.asarray(actual_kp)[ids], kp[ids]) or not np.allclose(np.asarray(actual_kd)[ids], kd[ids]):
        raise RuntimeError('Measured PD does not match comparison contract')
    t0 = physics_time[0]
    fps=25 if args.audit_feet else 24
    writer = imageio.get_writer(str(out/'turn_720p.mp4'), fps=fps, codec='libx264', quality=8, macro_block_size=None)
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20)
    last = np.zeros(19, dtype=np.float32)
    frames = 0
    dark_frames = []
    render_retries = []
    failure = None
    for step in range(2801):
        t = step*.005
        pos,q = robot.get_world_pose()
        R = quat_to_rot_matrix(q).T
        lin = R @ robot.get_linear_velocity()
        ang = R @ robot.get_angular_velocity()
        grav = R @ np.array([0.,0.,-1.])
        w,x,y,z = q
        yaw = math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
        tilt = math.degrees(math.acos(np.clip(-grav[2],-1,1)))
        cmd = np.array([0.,0., args.sign*.30 if 2 <= t < 12 else 0.], dtype=np.float32)
        if step % (4 if args.audit_feet else 20) == 0:
            rows.append(dict(time_s=t,position_world=pos.tolist(),yaw_deg=yaw,yaw_rate_rps=float(ang[2]),tilt_deg=tilt,
                speed_mps=float(np.linalg.norm(lin[:2])),velocity_command=cmd.tolist(),contact_N=0.,
                contact_scope='No racks or obstacles in isolated plane; contact force not measured',
                actual_physics_elapsed_s=physics_time[0]-t0))
            if args.audit_feet:
                def yaw_of(quat):
                    w,x,y,z=map(float,quat)
                    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
                feet=[view.get_world_poses() for view in foot_views]
                force=[np.asarray(view.get_net_contact_forces(dt=.005))[0] for view in foot_views]
                rows[-1].update(command_applied=cmd.tolist(),
                    foot_force_n=[float(np.linalg.norm(f)) for f in force],
                    foot_force_world_n=[f.tolist() for f in force],
                    foot_position_world=[np.asarray(f[0])[0].tolist() for f in feet],
                    foot_yaw_deg=[yaw_of(np.asarray(f[1])[0]) for f in feet],
                    foot_velocity_world=[np.asarray(view.get_velocities())[0,:3].tolist() for view in foot_views],
                    torso_yaw_world_deg=yaw_of(np.asarray(torso_view.get_world_poses()[1])[0]),
                    joint_position=robot.get_joint_positions()[ids].tolist(),
                    contact_scope='Ankle foot net forces measured; no rack contacts exist')
        if tilt >= 15 or pos[2] < .65:
            failure = 'POSTURE_ABORT'
        if t >= frames/fps-1e-9 or failure:
            rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
            rgb = np.asarray(ann.get_data())[:,:,:3].copy()
            for retry in range(args.render_retries):
                if np.mean(rgb[100:650,100:1180]) >= 12: break
                before = physics_time[0]
                rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
                if physics_time[0] != before: raise RuntimeError('Render retry advanced physics')
                rgb = np.asarray(ann.get_data())[:,:,:3].copy()
                render_retries.append({'frame':frames, 'attempt':retry+1})
            if np.mean(rgb[100:650,100:1180]) < 12: dark_frames.append(frames)
            img = Image.fromarray(rgb)
            draw = ImageDraw.Draw(img)
            draw.rectangle((0,0,1280,84), fill=(8,18,28))
            direction='LEFT' if args.sign>0 else 'RIGHT'
            draw.text((15,8),f'H1 {direction} TURN | {args.policy} policy | {args.asset} asset | {args.contract} contract | t={t:.2f}s',font=font,fill='white')
            draw.text((15,35),f'yaw command {cmd[2]:+.2f} | measured {ang[2]:+.3f} rad/s | XY {pos[0]:+.3f}, {pos[1]:+.3f}m | tilt {tilt:.1f}deg',font=font,fill='white')
            draw.text((15,59),'PHYSICAL SIMULATION | no root teleport | no navigation / grasp | fixed initial pose',font=font,fill=(190,218,245))
            if args.audit_feet:
                fn=rows[-1]['foot_force_n']
                draw.text((15,93),f'FOOT CONTACT L/R {fn[0]:.0f}/{fn[1]:.0f}N | official asset reference, not JM deployment',font=font,fill='white',stroke_width=1,stroke_fill='black')
            writer.append_data(np.asarray(img))
            if frames%96==0 or failure or step==2800: img.save(out/f'frame_{frames:04d}.jpg')
            frames += 1
        if failure or step == 2800: break
        if step%4==0:
            obs = np.r_[lin,ang,grav,cmd,robot.get_joint_positions()[ids]-defaults,robot.get_joint_velocities()[ids],last].astype(np.float32)
            with torch.inference_mode(): last = policy(torch.from_numpy(obs)[None]).numpy()[0]
            if not np.isfinite(last).all(): raise RuntimeError('Nonfinite action')
            targets[ids] = defaults+.5*last
        robot.apply_action(ArticulationAction(joint_positions=targets[command_ids],joint_indices=command_ids))
        world.step(render=False)
        if step in (0,19,39):
            runtime_kp,runtime_kd=robot.get_articulation_controller().get_gains()
            (out/f'pd_after_step_{step+1}.json').write_text(json.dumps(dict(
                kp=np.asarray(runtime_kp).tolist(),kd=np.asarray(runtime_kd).tolist(),
                expected_kp=kp.tolist(),expected_kd=kd.tolist()),indent=2))
            if not np.allclose(np.asarray(runtime_kp)[ids],kp[ids]) or not np.allclose(np.asarray(runtime_kd)[ids],kd[ids]):
                raise RuntimeError('Body PD overwritten after physics step; INVALID comparison')
    audit = audit_turn(rows,args.sign)
    startup = [r for r in rows if r['time_s']<=2.]
    result = dict(completed=failure is None, failure=failure, turn_audit=audit,
        start_drift_m=float(np.linalg.norm(np.array(startup[-1]['position_world'])[:2]-np.array(startup[0]['position_world'])[:2])),
        final_speed_mps=rows[-1]['speed_mps'], frames=frames, dark_main_scene_frames=dark_frames,
        same_physics_state_render_retries=render_retries,
        elapsed_simulation_s=rows[-1]['time_s'], collision_validated=False, physical_grasp_validated=False)
    (out/'telemetry.json').write_text(json.dumps(rows,indent=2))
    (out/'result.json').write_text(json.dumps(result,indent=2))
    if args.audit_feet:
        from jm_wb5_turn.footstep_metrics import foot_events,angle_travel,THRESHOLDS
        active=[r for r in rows if 2.<=r['time_s']<=12.]
        events=[foot_events(rows,k,2.,12.) for k in range(2)]
        detail=dict(thresholds=THRESHOLDS,foot_events=events,
            verified_lift_landing_counts=[sum(e['verified_lift_landing'] for e in es) for es in events],
            pelvis_turn_deg=angle_travel(active,'yaw_deg') if len(active)>1 else None,
            torso_turn_deg=angle_travel(active,'torso_yaw_world_deg') if len(active)>1 else None,
            foot_turn_deg=[angle_travel(active,'foot_yaw_deg',k) for k in range(2)] if len(active)>1 else None,
            failure=failure,scope='official reference protocol 2s stand + 10s yaw at 0.30rad/s + 2s stand; not goal-pose matched comparison')
        (out/'footstep_audit.json').write_text(json.dumps(detail,indent=2))
        print('OFFICIAL_FOOT_AUDIT',json.dumps(detail),flush=True)
    print('COMPARISON_RESULT',json.dumps(result),flush=True)
except Exception:
    (out/'exception.txt').write_text(traceback.format_exc())
    raise
finally:
    if writer: writer.close()
    app.close()
