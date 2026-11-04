"""Bounded SIMULATION trial: fresh RGB-D -> X-NavDP -> frozen H1 walker.

Physics deliberately pauses during synchronous model calls. This is not a
real-time hardware controller. USD geometry is an independent abort audit,
never a model input or a replacement path. No source world is saved.
"""
import argparse
import hashlib
import io
import json
import math
import secrets
from pathlib import Path
import sys
import time
import traceback
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from jm_xnavdp.h1_contract import load_constants, gains, observation, RuntimeAudit, PHYSICS_DT, POLICY_DECIMATION, ACTION_SCALE
from jm_xnavdp.docking import dock_command
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--world', choices=['jm', 'keti'], required=True)
p.add_argument('--port', type=int, default=8891)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--max-speed', type=float, default=.4)
p.add_argument('--terminal', choices=['path', 'rack_rl', 'goal_pose'], default='path')
p.add_argument('--docking-controller',choices=['legacy','position_first'],default='legacy',
               help='position_first remains experimental: failed JM safety-envelope trial')
p.add_argument('--mission', choices=['single', 'jm_transfer'], default='single')
p.add_argument('--transfer-turn-rate', type=float, default=.30)
p.add_argument('--departure-turn', choices=['shortest','positive'], default='shortest')
p.add_argument('--transfer-backoff', type=float, choices=[0.,.25], default=0.)
p.add_argument('--transfer-mode', choices=['forward','strafe'], default='forward')
p.add_argument('--spawn', choices=['random','authored'], default=None,
               help='JM transfer defaults to random corridor; authored is explicit historical replay')
p.add_argument('--spawn-seed', type=int, default=None, help='Replay an initial spawn; omitted generates and records a new seed')
p.add_argument('--keep-other-robots', action='store_true', help='Explicit historical multi-robot replay; JM defaults to H1_1 only')
p.add_argument('--turn-probe', choices=['none','negative','positive'], default='none',
               help='Isolated 2s stand / 10s yaw / 2s stand diagnostic, no navigation')
p.add_argument('--turn-policy', choices=['wb5','official'], default='wb5',
               help='Diagnostic only: compare official weights with otherwise unchanged JM deployment')
p.add_argument('--replay-run',type=Path,help='Diagnostic: replay recorded 10Hz velocity commands, never root poses')
p.add_argument('--contact-audit',action='store_true',help='Record unfiltered PhysX contact pairs and points')
p.add_argument('--diagnostic-seconds',type=float,default=24.,help='Replay-only recording length')
p.add_argument('--dock-probe-start',type=float,help='Replay velocity prefix, then test terminal docking only; no grasp/mission success')
args = p.parse_args()
if args.dock_probe_start is not None and (not args.replay_run or not 1 <= args.dock_probe_start < args.diagnostic_seconds):
    p.error('Dock probe requires replay and handoff time in [1, diagnostic-seconds)')
if args.dock_probe_start is not None and args.docking_controller!='position_first':
    p.error('Dock probe requires explicit --docking-controller position_first')
if args.turn_policy!='wb5' and args.turn_probe=='none': p.error('Policy comparison is restricted to isolated turn diagnostics')
replay_rows=None
if args.replay_run:
    if args.turn_probe!='none' or args.mission!='jm_transfer': p.error('Replay requires JM transfer and no turn probe')
    replay_manifest=json.loads((args.replay_run/'manifest.json').read_text())
    replay_rows=json.loads((args.replay_run/'telemetry.json').read_text())
    if args.spawn_seed!=replay_manifest['arguments']['spawn_seed']: p.error('Replay must use original spawn seed')
    if not 0<args.diagnostic_seconds<=replay_rows[-1]['time_s']: p.error('Replay duration outside recorded samples')
args.spawn=args.spawn or ('random' if args.mission=='jm_transfer' else 'authored')
if args.spawn=='random' and args.mission!='jm_transfer': p.error('Random spawn currently verified only for JM transfer setup')
if args.spawn_seed is not None and (args.spawn!='random' or not 0<=args.spawn_seed<2**32): p.error('spawn-seed requires random spawn and uint32 seed')
if args.mission=='jm_transfer' and (args.world!='jm' or args.terminal!='goal_pose'):
    p.error('jm_transfer requires --world jm --terminal goal_pose')
if args.out.exists(): raise RuntimeError('Fresh output directory required')
if not 0 < args.max_speed <= .4: raise ValueError('Outside existing velocity envelope')
args.out.mkdir(parents=True)
out = args.out.resolve()
service = f'http://127.0.0.1:{args.port}'
def post(route, data=b'', content_type='application/octet-stream'):
    req = urllib.request.Request(service+route, data=data, headers={'Content-Type':content_type})
    with urllib.request.urlopen(req, timeout=120) as response: return json.load(response)
if not replay_rows and args.turn_probe=='none': post('/reset')
constants = load_constants(ROOT)
walker_path = ROOT/'artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt'
if hashlib.sha256(walker_path.read_bytes()).hexdigest()!=constants['WALKER_SHA256']: raise RuntimeError('Walker changed')
if args.turn_policy=='official':
    walker_path=ROOT/'artifacts/xnavdp_20260930/turn_compare_official_native_neg_002/official_policy.pt'
    expected='a734d8ae13b8c3690138baf8d01d7d497f987616da2c53e00a43e0ceb31452d7'
    if hashlib.sha256(walker_path.read_bytes()).hexdigest()!=expected: raise RuntimeError('Official comparison weights changed')
    constants['WALKER_SHA256']=expected
capture = ROOT/'artifacts/xnavdp_20260929'/('jm_capture_002' if args.world=='jm' else 'keti_capture_001')
meta = json.loads((capture/'observation.json').read_text())
floor = 6.075 if args.world=='jm' else 0.
if replay_rows: service_health={'configuration':replay_manifest['service_configuration']}
elif args.turn_probe!='none': service_health={'configuration':{'floor_z':floor,'diagnostic_only':True,'inference_service_used':False}}
else:
    with urllib.request.urlopen(service+'/health',timeout=10) as response: service_health=json.load(response)
if service_health.get('configuration',{}).get('floor_z') != floor:
    raise RuntimeError('Planner service must use the matching world floor; restart the matching service')
if args.mission=='jm_transfer' and (service_health['configuration'].get('route_guide') or service_health['configuration'].get('route_guard')):
    raise RuntimeError('Pickup approach must not use A* guidance in this mission')
floor_yaw = math.radians(90.86196657 if args.world=='jm' else 0.)
if args.world=='keti':
    scene = json.loads((ROOT/'artifacts/keti_navigation_20260929/plan_002/scenarios.json').read_text())
    scenario = next(s for s in scene['scenarios'] if s['seed']==42)
    obstacle = scenario['random_obstacle']
    all_obstacles = scene['obstacles']+[obstacle]
else:
    scene = json.loads((ROOT/'artifacts/jm_stop_pose_20260929/scene_001.json').read_text())
    all_obstacles = scene['obstacles']
obstacles = [o for o in all_obstacles if o['max'][2]>floor+.15 and o['min'][2]<floor+1.75]
source_hashes = meta['source_sha256']
for path,h in source_hashes.items():
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=h: raise RuntimeError('World changed since observation: '+path)

from isaacsim import SimulationApp
app = SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1280,'height':720})
writers=[]; products=[]; records=[]; inference_records=[]; world=None; detailed_contacts=None
try:
    import numpy as np
    import torch
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont
    import carb.settings
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, RigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from jm_xnavdp.physical_planner import follow
    from keti_navigation.planner import clearance
    carb.settings.get_settings().set('/rtx/post/histogram/enabled',False)
    ctx=omni.usd.get_context(); ctx.open_stage(meta['world'])
    for _ in range(10): app.update()
    stage=ctx.get_stage(); stage.SetEditTarget(stage.GetSessionLayer())
    robot_path='/World/H1Fleet/H1_1' if args.world=='jm' else '/World/XNavDPTrial/H1'
    removed_robots=[]
    if args.world=='jm' and not args.keep_other_robots:
        for group_path in ('/World/H1Fleet','/World/Robots'):
            group=stage.GetPrimAtPath(group_path)
            if group:
                for child in list(group.GetChildren()):
                    if str(child.GetPath())!=robot_path and child.IsActive():
                        removed_robots.append(str(child.GetPath())); child.SetActive(False)
    if args.world=='keti': stage.GetPrimAtPath('/World/Robots').SetActive(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not str(prim.GetPath()).startswith(robot_path+'/'):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
    if args.world=='keti':
        rp=UsdGeom.Xform.Define(stage,robot_path).GetPrim()
        rp.GetReferences().AddReference((ROOT/'jm_wb7_pinkik/assets/h1_wb7_standalone.usd').as_posix())
        xf=UsdGeom.Xformable(rp); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*meta['base_world'])); xf.AddRotateZOp().Set(math.degrees(meta['yaw_rad']))
        box=UsdGeom.Cube.Define(stage,obstacle['path']); box.CreateSizeAttr(1.)
        box.AddTranslateOp().Set(Gf.Vec3d(*obstacle['center'])); box.AddScaleOp().Set(Gf.Vec3f(*obstacle['size']))
        box.CreateDisplayColorAttr([(.94,.38,.07)]); UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    rp=stage.GetPrimAtPath(robot_path)
    mission=None
    spawn_audit=dict(mode='authored',initialization_only=True,yaw_randomized=False)
    if args.mission=='jm_transfer':
        from jm_xnavdp.transfer import RackTransfer
        rack_goals=json.loads((ROOT/'jm_coop4_rl/rack_goals.json').read_text())['goals']
        source=next(g for g in rack_goals if g['robot']=='H1_1')
        dest=next(g for g in rack_goals if g['robot']=='H1_3')
        unload=np.asarray(meta['goal_world_xy'])+(np.asarray(dest['dock_world_xy'])-source['dock_world_xy'])
        # Original static export excluded robots: add their ACTUAL authored colliders.
        bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],False,True)
        for prim in stage.Traverse():
            path=str(prim.GetPath())
            if path.startswith(('/World/H1Fleet/','/World/Robots/')) and not path.startswith(robot_path+'/') and prim.HasAPI(UsdPhysics.CollisionAPI):
                if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False: continue
                box=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
                if not box.IsEmpty() and box.GetMax()[2]>floor+.15 and box.GetMin()[2]<floor+1.75:
                    obstacles.append(dict(path=path,min=list(box.GetMin()),max=list(box.GetMax())))
        mission=RackTransfer(meta['goal_world_xy'],unload,floor_yaw,obstacles,args.transfer_turn_rate,args.departure_turn,args.transfer_backoff,args.transfer_mode)
        if args.spawn=='random':
            from jm_xnavdp.spawn import sample_spawn
            seed=args.spawn_seed if args.spawn_seed is not None else secrets.randbits(32)
            spawn_audit=sample_spawn(obstacles,seed)
            spawn_audit['original_authored_xy']=meta['base_world'][:2]
            meta['base_world'][:2]=spawn_audit['selected_xy']
            (out/'spawn.json').write_text(json.dumps(spawn_audit,indent=2),encoding='utf-8')
    phys=PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath('/World/PhysicsScene'))
    phys.CreateEnableGPUDynamicsAttr(False); phys.CreateBroadphaseTypeAttr('MBP'); phys.CreateTimeStepsPerSecondAttr(200)
    body_paths=[]; mimics=[]; colliders=0
    for prim in Usd.PrimRange(rp):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            props=PhysxSchema.PhysxArticulationAPI.Apply(prim)
            props.CreateEnabledSelfCollisionsAttr(False)
            props.CreateSolverPositionIterationCountAttr(4); props.CreateSolverVelocityIterationCountAttr(4)
        if prim.HasAPI(UsdPhysics.CollisionAPI): colliders+=1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body_paths.append(str(prim.GetPath()))
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateMaxDepenetrationVelocityAttr(1.); rb.CreateLinearDampingAttr(0.); rb.CreateAngularDampingAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            kp,kd,force=gains(prim.GetName())
            if prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
                kp,kd,force=0.,0.,0.; mimics.append(prim.GetName())
            drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
            drive.CreateStiffnessAttr(kp*math.pi/180); drive.CreateDampingAttr(kd*math.pi/180); drive.CreateMaxForceAttr(force)
    # Contact audit covers nearby environment colliders; ground supports are excluded.
    start=np.asarray(meta['base_world'][:2]); goal=np.asarray(meta['goal_world_xy'])
    filters=[o['path'] for o in obstacles if stage.GetPrimAtPath(o['path']).HasAPI(UsdPhysics.CollisionAPI)
             and np.linalg.norm((np.asarray(o['min'][:2])+o['max'][:2])/2-(start+goal)/2)<6.]
    if not filters or not colliders: raise RuntimeError('Missing physical collision audit')
    world=World(physics_dt=PHYSICS_DT,rendering_dt=1/24,stage_units_in_meters=1.,physics_prim_path='/World/PhysicsScene',backend='numpy')
    robot=world.scene.add(SingleArticulation(robot_path,name='xnav_h1'))
    contact=world.scene.add(RigidPrim(body_paths,name='environment_contacts',reset_xform_properties=False,
        track_contact_forces=True,contact_filter_prim_paths_expr=[filters for _ in body_paths],max_contact_count=2048))
    if args.contact_audit:
        from jm_xnavdp.contact_audit import ContactAudit
        detailed_contacts=ContactAudit(stage,robot_path,out)
    world.reset()
    physics_audit={'steps':0,'seconds':0.}
    def audit_physics(dt):
        physics_audit['steps']+=1; physics_audit['seconds']+=float(dt)
    world.add_physics_callback('xnav_clock_audit',callback_fn=audit_physics)
    if args.spawn=='random':
        initial_position,initial_orientation=robot.get_world_pose()
        initial_position=np.asarray(initial_position).copy()
        initial_position[:2]=spawn_audit['selected_xy']
        # Exactly once at episode initialization, before warm-up/camera capture/stepping.
        robot.set_world_pose(position=initial_position,orientation=initial_orientation)
        measured_position,measured_orientation=robot.get_world_pose()
        if np.linalg.norm(np.asarray(measured_position)[:2]-spawn_audit['selected_xy'])>.001:
            raise RuntimeError('Spawn pose write/read mismatch')
        spawn_audit.update(measured_initial_position=np.asarray(measured_position).tolist(),
                           measured_initial_orientation_wxyz=np.asarray(measured_orientation).tolist(),
                           initialization_root_pose_writes=1,root_writes_during_motion=0)
        (out/'spawn.json').write_text(json.dumps(spawn_audit,indent=2),encoding='utf-8')
    if not contact._contact_view.is_physics_handle_valid(): raise RuntimeError('Invalid contact sensor')
    ids=[robot.get_dof_index(n) for n in constants['BODY_19']]
    defaults=np.asarray(constants['WALK_DEFAULT_19'],dtype=np.float32)
    targets=np.zeros(robot.num_dof,dtype=np.float32); targets[ids]=defaults
    command_ids=[i for i,n in enumerate(robot.dof_names) if n not in mimics]
    robot.set_joint_positions(targets); robot.set_joint_velocities(np.zeros_like(targets)); robot.set_world_velocity(np.zeros(6))
    kp,kd=robot.get_articulation_controller().get_gains()
    if not np.allclose(np.asarray(kp)[ids],[gains(n)[0] for n in constants['BODY_19']],rtol=1e-5): raise RuntimeError('Walker stiffness mismatch')
    if not np.allclose(np.asarray(kd)[ids],[gains(n)[1] for n in constants['BODY_19']],rtol=1e-5): raise RuntimeError('Walker damping mismatch')
    runtime_audit=RuntimeAudit(out/'runtime_contract.json',constants['BODY_19'],ids)
    walker=torch.jit.load(str(walker_path),map_location='cpu').eval()
    terminal=None
    if args.terminal=='rack_rl':
        ckpt=ROOT/'jm_coop4_rl/runs/h1_nav_001/model_200.pt'
        if hashlib.sha256(ckpt.read_bytes()).hexdigest()!='56fb8ddaec05da9342d0acaf0160048cd8361efc7b5b6525677c096dd06f49a9': raise RuntimeError('Terminal model changed')
        terminal=torch.nn.Sequential(torch.nn.Linear(18,128),torch.nn.ELU(),torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,3))
        terminal.load_state_dict({k.removeprefix('actor.'):v for k,v in torch.load(ckpt,map_location='cpu',weights_only=True)['model_state_dict'].items() if k.startswith('actor.')},strict=True)
        terminal.eval()
    manifest={'world':args.world,'arguments':vars(args),'source_world_sha256':source_hashes,
        'runtime_contract':dict(physics_dt=PHYSICS_DT,policy_decimation=POLICY_DECIMATION,
            action_scale=ACTION_SCALE,observation_order=['body_lin','body_ang','body_gravity','velocity_command','q_minus_default','qd','previous_action'],
            continuing_readback='steps 1,20,40 and every 200; body19 PD/effort/clock',training_parity_verified=False),
        'spawn':spawn_audit,
        'removed_robot_roots_session_only':removed_robots,
        'service_configuration':service_health['configuration'],
        'walker_sha256':constants['WALKER_SHA256'],'goal_xy':goal.tolist(),'goal_yaw_rad':floor_yaw,
        'command_joints':[robot.dof_names[i] for i in command_ids],'mimic_followers_not_commanded':mimics,
        'contact_filter_paths':filters,'colliders':colliders,'self_collisions':False,
        'simulator_pose_ground_truth':True,'usd_audit_abort_only':True,'physics_paused_during_inference':True,
        'replan_sim_s':.5,'robot_control_hz':50,'physics_hz':200,
        'source_code_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),ROOT/'jm_xnavdp/h1_contract.py',ROOT/'jm_xnavdp/docking.py',ROOT/'jm_xnavdp/physical_planner.py',ROOT/'jm_xnavdp/server.py',ROOT/'jm_xnavdp/runtime.py']}}
    (out/'manifest.json').write_text(json.dumps(manifest,default=str,indent=2),encoding='utf-8')
    if mission:
        manifest['mission']=dict(pickup_rack=source['rack_path'],unload_rack=dest['rack_path'],
            pickup_xy=mission.pickup.tolist(),unload_xy=mission.unload.tolist(),
            physical_grasp=False,assumption_only=True,payload_dynamics=False,
            transfer_planner='A* on authored USD AABBs; not RGB-D mapping',
            other_robot_colliders_included=True)
        manifest['usd_audit_abort_only']=False
        manifest['source_code_sha256'][str(ROOT/'jm_xnavdp/transfer.py')]=hashlib.sha256((ROOT/'jm_xnavdp/transfer.py').read_bytes()).hexdigest()
        manifest['source_code_sha256'][str(ROOT/'jm_xnavdp/spawn.py')]=hashlib.sha256((ROOT/'jm_xnavdp/spawn.py').read_bytes()).hexdigest()
        (out/'manifest.json').write_text(json.dumps(manifest,default=str,indent=2),encoding='utf-8')
    def inv(q,v):
        qv=np.asarray(q[1:]); v=np.asarray(v); uv=np.cross(qv,v)
        return v-2*q[0]*uv+2*np.cross(qv,uv)
    def measure():
        p,q=robot.get_world_pose(); w,x,y,z=q
        return np.asarray(p),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)),inv(q,robot.get_linear_velocity()),inv(q,robot.get_angular_velocity()),inv(q,[0,0,-1])
    def wrap(a): return math.atan2(math.sin(a),math.cos(a))
    rep.orchestrator.set_capture_on_play(False)
    def camera(path,focal,ha):
        cam=UsdGeom.Camera.Define(stage,path); cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(ha); cam.CreateVerticalApertureAttr(ha*9/16)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.05,200.)); op=cam.AddTransformOp()
        product=rep.create.render_product(path,(1280,720)); products.append(product)
        ann=rep.AnnotatorRegistry.get_annotator('rgb'); ann.attach([product])
        return cam,op,ann,product
    cam,camop,rgb_ann,product=camera('/World/XNavDPTrial/Sensor',1.93,1280*1.93/meta['intrinsic'][0][0])
    dep_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane'); dep_ann.attach([product])
    _,viewop,view_ann,_=camera('/World/XNavDPTrial/ThirdPerson',18.,20.955)
    mid=(start+goal)/2
    eye=np.array([mid[0]+3.,mid[1]-3.4,floor+2.8]) if args.world=='jm' else np.array([7.7,-3.5,4.4])
    focus=np.r_[mid,floor+.8]
    if mission:
        mid=(start+mission.unload)/2
        eye=np.array([mid[0]+3.7,mid[1]-4.8,floor+3.3]); focus=np.r_[mid,floor+.8]
    if args.turn_probe!='none':
        eye=np.r_[start+[3.,-3.5],floor+2.6]; focus=np.r_[start,floor+.8]
    viewop.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*focus),Gf.Vec3d(0,0,1)).GetInverse())
    def update_sensor(pos,yaw):
        forward=np.array([math.cos(yaw),math.sin(yaw),0.]); pitch=math.radians(8)
        eye=pos+.12*forward+np.array([0,0,.55]); view=math.cos(pitch)*forward+np.array([0,0,-math.sin(pitch)])
        camop.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*(eye+view)),Gf.Vec3d(0,0,1)).GetInverse())
    latest_input={}
    def infer(pos,yaw,t):
        debug=stage.GetPrimAtPath('/World/XNavDPTrial/Debug')
        if debug: UsdGeom.Imageable(debug).MakeInvisible()
        update_sensor(pos,yaw); rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
        rgb=np.asarray(rgb_ann.get_data())[:,:,:3].copy(); depth=np.asarray(dep_ann.get_data()).reshape(720,1280).copy()
        if debug: UsdGeom.Imageable(debug).MakeVisible()
        latest_input.update(rgb=rgb,time_s=t)
        packet=dict(rgb=rgb,depth_m=depth,base_world=pos,yaw_rad=np.array(yaw),goal_world_xy=goal,
                    intrinsic=np.asarray(meta['intrinsic']),captured_unix_s=np.array(time.time()))
        buffer=io.BytesIO(); np.savez_compressed(buffer,**packet)
        tic=time.perf_counter(); result=post('/infer',buffer.getvalue()); result['client_roundtrip_s']=time.perf_counter()-tic
        idx=len(inference_records); result['simulation_time_s']=t; result['base_world']=pos.tolist(); result['yaw_rad']=yaw
        inference_records.append(result)
        np.savez_compressed(out/f'observation_{idx:03d}.npz',**packet)
        (out/f'inference_{idx:03d}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return result
    # Load and warm up at initialization, then reset all model history before motion.
    pos,yaw,*_=measure()
    if args.turn_probe=='none' and not replay_rows:
        infer(pos,yaw,-1.); post('/reset'); inference_records.clear()
    else:
        update_sensor(pos,yaw); rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
        latest_input.update(rgb=np.asarray(rgb_ann.get_data())[:,:,:3].copy(),time_s=-1.)
    initial_physics_time=physics_audit['seconds']
    initial_physics_steps=physics_audit['steps']
    runtime_audit.check(robot,0,0.)
    writer=imageio.get_writer(str(out/f'{args.world}_xnavdp_physical_720p.mp4'),fps=24,codec='libx264',quality=8,macro_block_size=None); writers.append(writer)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
    small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',15)
    plan=None; velocity=np.zeros(3,dtype=np.float32); last=np.zeros(19,dtype=np.float32)
    failure=None; success=False; phase='SETTLE'; dwell=0.; frames=0; next_frame=0.; max_contact=0.; min_margin=1e9
    trace=[]; terminal_latched=False; selected_id=None; execution_source='NONE'
    UsdGeom.Xform.Define(stage,'/World/XNavDPTrial/Debug')
    path_curve=UsdGeom.BasisCurves.Define(stage,'/World/XNavDPTrial/Debug/SelectedPath')
    path_curve.CreateTypeAttr('linear'); path_curve.CreateWidthsAttr([.027]); path_curve.SetWidthsInterpolation('constant'); path_curve.CreateDisplayColorAttr([(.1,.7,1.)])
    guide_curve=UsdGeom.BasisCurves.Define(stage,'/World/XNavDPTrial/Debug/DepthGuide')
    guide_curve.CreateTypeAttr('linear'); guide_curve.CreateWidthsAttr([.018]); guide_curve.SetWidthsInterpolation('constant'); guide_curve.CreateDisplayColorAttr([(.8,.3,.9)])
    ring=UsdGeom.BasisCurves.Define(stage,'/World/XNavDPTrial/Debug/Goal')
    ring.CreateTypeAttr('linear'); ring.CreateWidthsAttr([.025]); ring.SetWidthsInterpolation('constant'); ring.CreateDisplayColorAttr([(.1,.9,.2)])
    circle=[Gf.Vec3f(float(goal[0]+.18*math.cos(a)),float(goal[1]+.18*math.sin(a)),floor+.027) for a in np.linspace(0,2*math.pi,33)]
    ring.CreateCurveVertexCountsAttr([33]); ring.CreatePointsAttr(circle)
    for step in range(16000 if mission else 9000):
        t=step*.005; pos,yaw,lin,ang,gravity=measure(); distance=float(np.linalg.norm(goal-pos[:2]))
        forces=np.asarray(contact.get_contact_force_matrix(dt=.005)).reshape(-1,3)
        force=float(np.linalg.norm(forces,axis=1).max(initial=0)); max_contact=max(max_contact,force)
        if force>.20: failure='ENVIRONMENT_CONTACT'
        if pos[2]<floor+.65 or -gravity[2]<math.cos(math.radians(40)): failure='FALL'
        if step%20==0:
            margin=clearance(pos[:2],obstacles)-.38; min_margin=min(min_margin,margin)
            if margin<=0: failure='FOOTPRINT_EXCLUSION'
            if args.turn_probe=='none' and not replay_rows and not failure and t>=1. and step%100==0 and distance>=.18 and not terminal_latched and (not mission or mission.state=='PICKUP_APPROACH'):
                result=infer(pos,yaw,t); selection=result.get('execution_selection',result['simulation_selection'])
                execution_source=selection.get('source','XNAVDP_MODEL')
                guide_data=result.get('depth_guide')
                if guide_data and guide_data['pass']:
                    guide_curve.CreateCurveVertexCountsAttr([len(guide_data['path_world_xy'])])
                    guide_curve.CreatePointsAttr([Gf.Vec3f(float(x),float(y),floor+.020) for x,y in guide_data['path_world_xy']])
                if not selection['pass']: failure=selection.get('reason','NO_ADMISSIBLE_MODEL_PATH')
                else:
                    local=np.asarray(selection['path_body_xy']); c,s=math.cos(yaw),math.sin(yaw)
                    plan=local@np.array([[c,s],[-s,c]])+pos[:2]; selected_id=selection['selected_id']
                    path_curve.CreateCurveVertexCountsAttr([len(plan)])
                    path_curve.CreatePointsAttr([Gf.Vec3f(float(x),float(y),floor+.025) for x,y in plan])
            desired=np.zeros(3)
            if t>=1. and not failure:
                phase='REFERENCE_TRACK' if execution_source=='DEPTH_ASTAR_REFERENCE' else 'XNAVDP_TRACK'
                if plan is not None: desired=follow(plan,pos,yaw,goal,args.max_speed)
                if distance<.18:
                    phase='STOP'; desired=np.array([0.,0.,np.clip(1.6*wrap(floor_yaw-yaw),-.3,.3)])
                if terminal is not None and (terminal_latched or distance<.8):
                    terminal_latched=True; phase='TERMINAL_RL'
                    delta=goal-pos[:2]; c,s=math.cos(yaw),math.sin(yaw)
                    local=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
                    heading=wrap(floor_yaw-yaw)
                    obs=np.r_[lin,ang,gravity,local,math.sin(heading),math.cos(heading),velocity,dwell,distance].astype(np.float32)
                    with torch.inference_mode(): action=torch.tanh(terminal(torch.from_numpy(obs)[None])).numpy()[0]
                    # Forward-only interface: lateral gait remains outside this trial.
                    desired=np.array([min(args.max_speed,max(0.,action[0])*.6),0.,action[2]*.45])
                if args.terminal=='goal_pose' and (terminal_latched or distance<.8):
                    terminal_latched=True
                    if args.docking_controller=='position_first':
                        desired,phase=dock_command(pos,yaw,goal,floor_yaw,args.max_speed)
                    else:
                        # Preserve prior deployment until the candidate passes physical gates.
                        phase='GOAL_POSE_DOCK'
                        delta=goal-pos[:2]; c,s=math.cos(yaw),math.sin(yaw)
                        forward=c*delta[0]+s*delta[1]
                        heading=wrap((floor_yaw if distance<.3 else math.atan2(delta[1],delta[0]))-yaw)
                        desired=np.array([min(args.max_speed,.3,.8*max(0.,forward-.08))*max(0.,math.cos(heading))**2,
                                          0.,np.clip(1.6*heading,-.3,.3)])
                        if distance<.18: desired[0]=0.
            if mission and mission.state!='PICKUP_APPROACH':
                if mission.state in ('ASSUMED_GRASP','RETREAT_FROM_RACK') and mission.start_transfer(pos,t):
                    goal=mission.unload.copy(); distance=float(np.linalg.norm(goal-pos[:2])); dwell=0.
                    path_curve.CreateCurveVertexCountsAttr([len(mission.path)])
                    path_curve.CreatePointsAttr([Gf.Vec3f(float(x),float(y),floor+.025) for x,y in mission.path])
                    circle=[Gf.Vec3f(float(goal[0]+.18*math.cos(a)),float(goal[1]+.18*math.sin(a)),floor+.027) for a in np.linspace(0,2*math.pi,33)]
                    ring.CreatePointsAttr(circle)
                desired,phase=mission.command(pos,yaw)
                execution_source=('ASSUMPTION_ONLY' if mission.state=='ASSUMED_GRASP' else
                                  'EXPERIMENTAL_BACKOFF' if mission.state=='RETREAT_FROM_RACK' else 'USD_ASTAR')
                if mission.failure: failure=mission.failure; desired=np.zeros(3)
            if args.turn_probe!='none':
                desired=np.zeros(3); phase='PROBE_STAND'
                if 2.<=t<12.:
                    desired[2]=-.30 if args.turn_probe=='negative' else .30; phase='PROBE_TURN'
                execution_source='ISOLATED_VELOCITY_COMMAND'
            if args.dock_probe_start is not None and t>=args.dock_probe_start:
                desired,phase=dock_command(pos,yaw,goal,floor_yaw,args.max_speed)
                execution_source='TERMINAL_DOCK_DIAGNOSTIC'
            velocity+=np.clip(desired-velocity,-np.array([.05,.05,.075]),np.array([.05,.05,.075]))
            if replay_rows and (args.dock_probe_start is None or t<args.dock_probe_start):
                original=replay_rows[step//20]
                if abs(original['time_s']-t)>1e-5: raise RuntimeError('Replay timestamp mismatch')
                velocity=np.asarray(original['velocity_command'],dtype=np.float32)
                phase='CONTACT_COMMAND_REPLAY'; execution_source='RECORDED_VELOCITY_COMMAND'
            stable=not failure and distance<.18 and np.linalg.norm(lin[:2])<.10 and abs(wrap(floor_yaw-yaw))<math.radians(15) and abs(ang[2])<.15 and -gravity[2]>math.cos(math.radians(15))
            dwell=dwell+.1 if stable else 0.
            if dwell>=1.-1e-8 and args.turn_probe=='none' and not replay_rows:
                if mission and mission.state=='PICKUP_APPROACH':
                    mission.pickup_complete(pos,t); phase='ASSUMED_GRASP'; dwell=0.
                elif not mission or mission.state=='ASTAR_TRANSFER':
                    success=True; phase='SUCCESS'
                    if mission: mission.finish(pos,t); phase='UNLOAD_POSITION_REACHED'
            trace.append(pos[:2].tolist())
            row=dict(time_s=t,phase=phase,position_world=pos.tolist(),yaw_deg=math.degrees(yaw),goal_distance_m=distance,
                velocity_command=velocity.tolist(),speed_mps=float(np.linalg.norm(lin[:2])),contact_N=force,
                footprint_proxy_clearance_m=margin,dwell_s=dwell,selected_id=selected_id,execution_source=execution_source,
                yaw_error_deg=math.degrees(wrap(floor_yaw-yaw)),yaw_rate_rps=float(ang[2]),
                tilt_deg=math.degrees(math.acos(np.clip(-gravity[2],-1,1))))
            row.update(actual_physics_elapsed_s=physics_audit['seconds']-initial_physics_time,
                       actual_physics_steps=physics_audit['steps']-initial_physics_steps)
            records.append(row)
            if mission:
                row.update(mission_state=mission.state,assumed_grasp=mission.assumed_grasp,
                           physical_grasp=False,active_goal_xy=goal.tolist())
                if len(records)>50 and all(r.get('mission_state')=='ASTAR_TRANSFER' for r in records[-50:]):
                    recent=records[-50:]
                    turning=all(abs(r['velocity_command'][2])>=.29 and abs(r['velocity_command'][0])<.01 for r in recent)
                    yaw_motion=abs(wrap(math.radians(recent[-1]['yaw_deg']-recent[0]['yaw_deg'])))
                    if turning and yaw_motion<math.radians(1): failure='TURN_STALL_5S'
            if step%200==0: print('XNAV_PROGRESS',json.dumps(row),flush=True)
            if args.turn_probe!='none' and t>=14. and not failure:
                success=True; phase='TURN_PROBE_COMPLETE'
            if replay_rows and t>=args.diagnostic_seconds and not failure:
                success=True; phase='DOCK_PROBE_COMPLETE' if args.dock_probe_start is not None else 'CONTACT_REPLAY_COMPLETE'
        # Record the last safe/failure state before ending, without taking a stale action.
        if t>=next_frame-1e-9 or failure or success:
            update_sensor(pos,yaw); rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False)
            img=Image.fromarray(np.asarray(view_ann.get_data())[:,:,:3]); draw=ImageDraw.Draw(img)
            draw.rectangle((0,0,1280,84),fill=(8,18,28))
            method='Depth A* guide + X-NavDP + H1' if service_health['configuration'].get('route_guide') else 'X-NavDP + frozen H1 gait'
            if mission: method='X-NavDP pickup -> ASSUMED GRASP -> A* unload'
            if args.turn_probe!='none': method=f'ISOLATED H1 TURN | {args.turn_policy} | {args.turn_probe} 0.30 rad/s'
            if replay_rows: method='PHYSX CONTACT AUDIT | recorded velocity commands'
            if args.dock_probe_start is not None: method='DOCK TEST | replay approach -> position -> yaw; NO grasp'
            draw.text((15,7),f'{args.world.upper()} | {method} | {failure or phase} | SIM t={t:.2f}s',font=font,fill='white')
            draw.text((15,34),f'Goal {distance:.3f}m | contact {force:.2f}N | physical gait, NO root teleport | candidate {selected_id}',font=small,fill='white')
            draw.text((15,58),'SIMULATION PILOT: physics pauses during RGB-D inference; USD geometry used only for abort audit',font=small,fill=(180,215,245))
            if mission:
                draw.rectangle((0,55,1280,84),fill=(8,18,28))
                draw.text((15,58),f'Grasp: {"ASSUMED (not physical)" if mission.assumed_grasp else "NOT YET"} | no payload physics | A* uses known USD map | no actual release',font=small,fill=(255,211,100))
            if args.turn_probe!='none':
                draw.rectangle((0,30,1280,84),fill=(8,18,28))
                draw.text((15,34),f'Command yaw {velocity[2]:+.2f} rad/s | measured {ang[2]:+.3f} rad/s | contact {force:.2f} N',font=small,fill='white')
                draw.text((15,58),'ISOLATED DIAGNOSTIC: no navigation inference / A* / grasp; physical motion, no root teleport',font=small,fill=(255,211,100))
            inset=Image.fromarray(latest_input['rgb']).resize((320,180)); img.paste(inset,(945,520))
            inset_label=(f"Model input t={latest_input['time_s']:.1f}s (no debug lines)" if args.turn_probe=='none' and not replay_rows else 'Initial camera view; NOT used for control')
            draw.text((949,500),inset_label,font=small,fill='white')
            writer.append_data(np.asarray(img))
            if frames%48==0 or failure or success: img.save(out/f'frame_{frames:04d}.jpg')
            frames+=1; next_frame=frames/24
        if failure or success: break
        if step%POLICY_DECIMATION==0:
            obs=observation(lin,ang,gravity,velocity,robot.get_joint_positions()[ids],robot.get_joint_velocities()[ids],last,defaults)
            with torch.inference_mode(): last=walker(torch.from_numpy(obs)[None]).numpy()[0]
            if not np.isfinite(last).all(): raise RuntimeError('NONFINITE_POLICY_ACTION')
            targets[ids]=defaults+ACTION_SCALE*last
        robot.apply_action(ArticulationAction(joint_positions=targets[command_ids],joint_indices=command_ids))
        world.step(render=False)
        if step+1 in (1,20,40) or (step+1)%200==0:
            runtime_audit.check(robot,step+1,physics_audit['seconds']-initial_physics_time)
        if detailed_contacts: detailed_contacts.sample(t+.005)
    if not success and not failure: failure='TIMEOUT'
    unchanged=all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in source_hashes.items())
    result=dict(pass_=success and failure is None and unchanged,failure=failure,time_s=t,frames=frames,goal_distance_m=distance,
        measured_displacement_m=float(np.linalg.norm(pos[:2]-start)),max_environment_contact_N=max_contact,min_footprint_margin_m=min_margin,
        inference_count=len(inference_records),actual_dynamic_H1=True,root_writes_during_motion=False,source_world_unchanged=unchanged,
        real_time_validated=False,self_collision_validated=False,model_obstacle_ground_truth_supplied=False,
        scope='Single seed simulation; navigation only, no grasp; synchronous sensing; oracle abort audit')
    result['pass']=result.pop('pass_')
    if mission:
        result.update(mission_state=mission.state,assumed_grasp=mission.assumed_grasp,
                      actual_grasp=False,actual_unload=False,payload_dynamics=False,
                      mission_events=mission.events,unload_goal_xy=mission.unload.tolist())
        result['scope']='JM navigation-only mission: X-NavDP pickup + logical grasp assumption + known USD map A* transfer; no physical grasp, payload dynamics, or release'
        (out/'mission_events.json').write_text(json.dumps(mission.events,indent=2),encoding='utf-8')
    result['removed_robot_roots_session_only']=removed_robots
    result['physics_elapsed_s']=physics_audit['seconds']-initial_physics_time
    result['runtime_contract_passed']=runtime_audit.report['passed']
    result['training_deployment_parity_verified']=False
    if detailed_contacts:
        audit=detailed_contacts.save()
        result['raw_contact_pair_count']=len(audit['pairs'])
        result['raw_rack_contact_point_count']=len(audit['rack_contact_points'])
    if replay_rows:
        result['navigation_attempted']=False
        result['scope']='Contact diagnostic replay of stored velocity commands; no new navigation inference or mission success'
        result['diagnostic_completed']=success and failure is None
        result['pass']=False
        result['replay_max_position_difference_m']=float(max(np.linalg.norm(np.asarray(r['position_world'])-replay_rows[i]['position_world']) for i,r in enumerate(records)))
        if args.dock_probe_start is not None:
            prefix=[(i,r) for i,r in enumerate(records) if r['time_s']<=args.dock_probe_start]
            result['replay_prefix_max_position_difference_m']=float(max(np.linalg.norm(np.asarray(r['position_world'])-replay_rows[i]['position_world']) for i,r in prefix))
            result['dock_gate_passed']=any(r['time_s']>=args.dock_probe_start and r['dwell_s']>=1.-1e-8 for r in records)
            result['scope']='Recorded velocity approach prefix then new docking controller; no new XNav inference, physical grasp or unload'
    if args.turn_probe!='none':
        from jm_xnavdp.turn_audit import audit_turn
        result['turn_audit']=audit_turn(records,1 if args.turn_probe=='positive' else -1)
        result['diagnostic_completed']=success and failure is None
        result['pass']=result['diagnostic_completed'] and result['turn_audit']['pass']
        result['navigation_attempted']=False
        result['scope']='Isolated yaw command diagnostic; no navigation, grasp, A* or unload'
        if not result['pass'] and failure is None: result['failure']='TURN_TRACKING_GATE_FAILED'
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (out/'telemetry.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    print('XNAV_RESULT',json.dumps(result),flush=True)
except BaseException:
    text=traceback.format_exc(); print(text,flush=True)
    (out/'exception.txt').write_text(text,encoding='utf-8')
    (out/'telemetry_partial.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    raise
finally:
    if detailed_contacts: detailed_contacts.save()
    for writer in writers: writer.close()
    for product in products: product.destroy()
    if world is not None: world.stop()
    app.close()
