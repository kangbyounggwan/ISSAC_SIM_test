"""Bounded four-H1 evaluation in the actual JM factory, without training edits."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--checkpoint',required=True)
p.add_argument('--out',required=True)
p.add_argument('--world',type=Path,default=None,
               help='Optional additive USD world. Default preserves the original four-H1 fleet world.')
p.add_argument('--seconds',type=float,default=20.)
p.add_argument('--no-video',action='store_true')
p.add_argument('--physical-gripper',action='store_true',
               help='Command only the two 2F-85 primary joints; never send targets to mimic followers.')
p.add_argument('--gripper-close-audit',action='store_true',
               help='After all robots dock, ramp both physical grippers from open to the calibrated close target.')
p.add_argument('--gripper-close-target',type=float,default=.36)
p.add_argument('--gripper-audit-seconds',type=float,default=2.0)
p.add_argument('--ring-grasp',action='store_true',help='H1_1 only: measured Pink approach, insertion and physical grasp; no lift.')
p.add_argument('--lift-transfer',action='store_true',help='After validated grasp, physically lift 10 cm and translate left 15 cm; no release.')
p.add_argument('--transport-grip-feedback',action='store_true',help='For lift/transfer only: close using measured 2 N per-finger contact feedback.')
p.add_argument('--ring-grip-effort-cap',type=float,default=.5,help='Primary gripper torque cap in Nm; default preserves the validated 0.5 Nm trial.')
p.add_argument('--payload-compensation',action='store_true',help='Add measured ring-mass gravity feedforward through arm Jacobian after verified grasp.')
p.add_argument('--payload-ramp-seconds',type=float,default=.3)
p.add_argument('--grip-contact-target',type=float,default=2.)
p.add_argument('--upper-gravity-compensation',action='store_true',help='Bounded model-computed upper-body feedforward torque after STOP.')
p.add_argument('--stance-control',choices=['walker_zero','measured_hold'],default='walker_zero',
               help='After dock: zero-velocity walker, or dynamic PD hold of measured lower-body joints.')
p.add_argument('--ik-wrist',action='store_true',help='Include native left wrist rotation in the measured Pink solve (0.5 rad/s cap).')
p.add_argument('--walker-arm-shield',action='store_true',help='Reuse WB7 hierarchy: keep upper-body walker observations at measured dock state during IK.')
args=p.parse_args()
if not .5<=args.grip_contact_target<=10.: raise ValueError('Grip contact target must be in [0.5, 10] N')
if args.payload_ramp_seconds<=0: raise ValueError('Payload ramp must be positive')
if not 0.<args.ring_grip_effort_cap<=2.: raise ValueError('Ring grip effort cap must be in (0, 2] Nm')
if args.lift_transfer and not args.ring_grasp:
    raise ValueError('--lift-transfer requires --ring-grasp')
if args.transport_grip_feedback and not args.lift_transfer:
    raise ValueError('--transport-grip-feedback requires --lift-transfer')
if args.payload_compensation and not (args.lift_transfer and args.upper_gravity_compensation):
    raise ValueError('--payload-compensation requires lift-transfer and upper-gravity-compensation')
if args.upper_gravity_compensation and not args.ring_grasp:
    raise ValueError('--upper-gravity-compensation requires --ring-grasp')
if args.ring_grasp:
    if not args.physical_gripper or args.gripper_close_audit:
        raise ValueError('--ring-grasp requires --physical-gripper and excludes empty CLOSE audit')
    sys.path.insert(0,str(ROOT/'jm_wb7_pinkik'))
    from local_runtime import configure_local_pink_runtime
    configure_local_pink_runtime(required=True)
out=Path(args.out).resolve()
if out.exists(): raise RuntimeError('Output exists; use a new run directory')
out.mkdir(parents=True)
(out/'arguments.json').write_text(json.dumps(vars(args),indent=2,default=str))
source_files=[Path(__file__)]
if args.ring_grasp:
    source_files += [ROOT/'jm_h1_ring_pickplace'/f for f in ['jm_grasp_controller.py','tcp_axis_task.py','ik_limits.py']]
if args.lift_transfer:
    source_files.append(ROOT/'jm_h1_ring_pickplace/jm_transport_controller.py')
if args.payload_compensation:
    source_files.append(ROOT/'jm_h1_ring_pickplace/payload_compensation.py')
(out/'source_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files},indent=2))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1280,'height':720})
import numpy as np
import torch
import carb.settings
import omni.usd
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
sys.path.insert(0,str(ROOT/'jm_coop4_rl'))
from h1_rack_env import BODY_19,WALK_DEFAULT_19,WALKER_PATH,H1_RACK_ROBOT_CFG

goals=json.loads((ROOT/'jm_coop4_rl/rack_goals.json').read_text(encoding='utf-8'))
items=goals['goals']
floor=goals['floor_z']
ckpt=Path(args.checkpoint).resolve()
payload=torch.load(str(ckpt),map_location='cpu',weights_only=False)
actor=torch.nn.Sequential(torch.nn.Linear(18,128),torch.nn.ELU(),torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,128),torch.nn.ELU(),torch.nn.Linear(128,3))
actor.load_state_dict({k[len('actor.'):]:v for k,v in payload['model_state_dict'].items() if k.startswith('actor.')})
actor.eval()
walker=torch.jit.load(WALKER_PATH,map_location='cpu').eval()
worldpath=(args.world.resolve() if args.world else ROOT/'jm_coop4_rl/assets/jm_world_h1_fleet.usda')
if not worldpath.is_file():
    raise FileNotFoundError(f'World file not found: {worldpath}')
ctx=omni.usd.get_context()
ctx.open_stage(str(worldpath))
for _ in range(10): app.update()
stage=ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
carb.settings.get_settings().set('/rtx/post/histogram/enabled',False)
scene=UsdPhysics.Scene(stage.GetPrimAtPath('/World/PhysicsScene'))
ps=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
ps.CreateEnableGPUDynamicsAttr(False)
ps.CreateBroadphaseTypeAttr('MBP')
ps.CreateTimeStepsPerSecondAttr(200)

# Exactly the WB5 training actuator contract (not the old signal-demo gains).
for item in items:
    for prim in Usd.PrimRange(stage.GetPrimAtPath(item['robot_prim'])):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            props=PhysxSchema.PhysxArticulationAPI.Apply(prim)
            props.CreateEnabledSelfCollisionsAttr(False)
            props.CreateSolverPositionIterationCountAttr(4)
            props.CreateSolverVelocityIterationCountAttr(4)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateMaxDepenetrationVelocityAttr(1.)
            rb.CreateLinearDampingAttr(0.)
            rb.CreateAngularDampingAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            n=prim.GetName()
            if 'ankle' in n: stiffness,damping,force=20.,4.,100.
            elif 'hip_yaw' in n or 'hip_roll' in n: stiffness,damping,force=150.,5.,300.
            elif any(s in n for s in ('hip_pitch','knee','torso')): stiffness,damping,force=200.,5.,300.
            elif 'shoulder' in n or 'elbow' in n: stiffness,damping,force=40.,10.,300.
            else: stiffness,damping,force=30.,1.,30.
            if args.physical_gripper and prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
                # Followers are constrained by the primary, not independently driven.
                stiffness,damping,force=0.,0.,0.
            if args.ring_grasp and n=='finger_joint': force=args.ring_grip_effort_cap
            drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
            # USD angular drives use degree-domain gains; PhysX tensors and
            # Isaac Lab use radians. Writing the same numeric value in USD
            # makes the effective gain 180/pi times too large.
            drive.CreateStiffnessAttr(stiffness*math.pi/180.)
            drive.CreateDampingAttr(damping*math.pi/180.)
            drive.CreateMaxForceAttr(force)
world=World(physics_dt=.005,rendering_dt=1/24,stage_units_in_meters=1.,physics_prim_path='/World/PhysicsScene',backend='numpy')
robots=[world.scene.add(SingleArticulation(i['robot_prim'],name=i['robot'])) for i in items]
grasp=None
if args.ring_grasp:
    sys.path.insert(0,str(ROOT/'jm_h1_ring_pickplace'))
    from jm_grasp_controller import JMGraspController
    if args.lift_transfer:
        from jm_transport_controller import JMTransportController as JMGraspController
    extra={'grip_feedback':args.transport_grip_feedback,'effort_cap':args.ring_grip_effort_cap,
           'payload_compensation':args.payload_compensation,'payload_ramp_seconds':args.payload_ramp_seconds,
           'grip_contact_target':args.grip_contact_target} if args.lift_transfer else {}
    grasp=JMGraspController(world,robots[0],H1_RACK_ROBOT_CFG,out,gravity_compensation=args.upper_gravity_compensation,use_wrist=args.ik_wrist,**extra)
world.reset()
if grasp is not None:
    if not grasp.fingers._contact_view.is_physics_handle_valid():
        raise RuntimeError('Grasp contact sensor failed initialization; no motion permitted')
    if args.lift_transfer and not grasp.gripper_contacts._contact_view.is_physics_handle_valid():
        raise RuntimeError('Gripper fixture contact sensor failed initialization; no motion permitted')
defaults=np.array(WALK_DEFAULT_19,dtype=np.float32)
ids=[]
command_ids=[]
gripper_ids=[]
targets=[]
gain_audit=[]
for r in robots:
    indices=[r.get_dof_index(n) for n in BODY_19]
    q=np.zeros(r.num_dof,dtype=np.float32)
    q[indices]=defaults
    r.set_joint_positions(q)
    r.set_joint_velocities(np.zeros_like(q))
    r.set_world_velocity(np.zeros(6))  # episode initialization only
    if args.physical_gripper:
        names=('finger_joint','finger_joint_0')
        missing=[name for name in names if name not in r.dof_names]
        if missing: raise RuntimeError(f'{r.name}: physical gripper joints missing: {missing}')
        fingers=[r.get_dof_index(name) for name in names]
        apply_ids=indices+fingers
        if args.ring_grasp:
            apply_ids+= [r.get_dof_index(n) for n in ('left_wrist_roll_joint','right_wrist_roll_joint')]
    else:
        fingers=[]
        apply_ids=list(range(r.num_dof))
    r.apply_action(ArticulationAction(joint_positions=q[apply_ids],joint_indices=apply_ids))
    ids.append(indices)
    command_ids.append(apply_ids)
    gripper_ids.append(fingers)
    targets.append(q)
    kp,kd=r.get_articulation_controller().get_gains()
    expected_kp=[]; expected_kd=[]
    for n in r.dof_names:
        if 'ankle' in n: k,d=20.,4.
        elif 'hip_yaw' in n or 'hip_roll' in n: k,d=150.,5.
        elif any(s in n for s in ('hip_pitch','knee','torso')): k,d=200.,5.
        elif 'shoulder' in n or 'elbow' in n: k,d=40.,10.
        else: k,d=30.,1.
        if args.physical_gripper and any(s in n for s in ('outer_knuckle','inner_finger')):
            k,d=0.,0.
        expected_kp.append(k); expected_kd.append(d)
    if not np.allclose(kp,expected_kp,rtol=1e-5) or not np.allclose(kd,expected_kd,rtol=1e-5):
        raise RuntimeError(f'{r.name}: runtime gains do not match training: {kp}, {kd}')
    gain_audit.append({'robot':r.name,'joint_names':r.dof_names,'stiffness':np.asarray(kp).tolist(),'damping':np.asarray(kd).tolist(),
                       'command_indices':apply_ids,'primary_indices':fingers,
                       'limits':np.asarray(r._articulation_view.get_dof_limits()).tolist()})
(out/'gain_audit.json').write_text(json.dumps(gain_audit,indent=2),encoding='utf-8')

def inv_rotate(q,v):
    qv=np.asarray(q[1:]); v=np.asarray(v)
    uv=np.cross(qv,v)
    return v-2*q[0]*uv+2*np.cross(qv,uv)

def measure():
    result=[]
    for i,(r,item) in enumerate(zip(robots,items)):
        pos,quat=r.get_world_pose()
        lin=inv_rotate(quat,r.get_linear_velocity())
        ang=inv_rotate(quat,r.get_angular_velocity())
        gravity=inv_rotate(quat,[0.,0.,-1.])
        w,x,y,z=quat
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        dx,dy=np.array(item['dock_world_xy'])-pos[:2]
        delta=np.array([math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy])
        heading=math.atan2(math.sin(math.pi/2-yaw),math.cos(math.pi/2-yaw))
        result.append(dict(pos=np.asarray(pos),lin=lin,ang=ang,gravity=gravity,delta=delta,
                           heading=heading,distance=float(np.hypot(dx,dy)),speed=float(np.linalg.norm(lin[:2]))))
    return result

def camera(path,eye,target,focal=18.):
    cam=UsdGeom.Camera.Define(stage,path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955*9/16)
    cam.CreateClippingRangeAttr(Gf.Vec2f(.05,200))
    mat=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(mat)
    return path

products=[]; annotators=[]; writers=[]
if not args.no_video:
    import omni.replicator.core as rep
    import imageio.v2 as imageio
    from PIL import Image,ImageDraw
    rep.orchestrator.set_capture_on_play(False)
    cams=[camera('/World/FleetCameras/Overview',(38,1,19),(38,17.8,7))]
    for item in items:
        x,y=item['proposed_reset_world_xy']
        cams.append(camera('/World/FleetCameras/'+item['robot'],(x+2.8,y-3.6,floor+2.7),(x,y+.55,floor+.9)))
    if args.ring_grasp:
        x,y=items[0]['dock_world_xy']
        cams=[camera('/World/FleetCameras/GraspFront',(x-1.4,y+1.3,floor+1.6),(x-.1,y,floor+.85),12.)]
        # Stay on H1_1's left; the old +3.3m view crossed H1_3 and hid the hand.
        cams.append(camera('/World/FleetCameras/GraspSide',(x-1.35,y-.25,floor+1.45),(x-.15,y+.2,floor+1.0),12.))
    for i,cam in enumerate(cams):
        product=rep.create.render_product(cam,(1280,720) if i==0 or args.ring_grasp else (640,360))
        a=rep.AnnotatorRegistry.get_annotator('rgb'); a.attach([product])
        products.append(product); annotators.append(a)
    for name in (['grasp_front_720p.mp4','grasp_side_720p.mp4'] if args.ring_grasp else ['fleet_overview_720p.mp4','fleet_four_view_720p.mp4']):
        writers.append(imageio.get_writer(str(out/name),fps=24,codec='libx264',quality=8,macro_block_size=None))
    rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)

velocity=np.zeros((4,3),dtype=np.float32)
last_low=np.zeros((4,19),dtype=np.float32)
dwell=np.zeros(4)
records=[]; frames=0; next_frame=0.; failure=None
first_success=[None]*4
walker_hold=None
gripper_audit_start=None
try:
    for step in range(int(args.seconds*200)):
        t=step*.005
        state=measure()
        if step%20==0:
            obs=[]
            for i,s in enumerate(state):
                stable=(s['distance']<.18 and abs(s['heading'])<math.radians(15) and s['speed']<.10 and abs(s['ang'][2])<.15 and -s['gravity'][2]>math.cos(math.radians(15)))
                dwell[i]=dwell[i]+.1 if stable and step>0 else 0.
                if dwell[i]>=1.-1e-6 and first_success[i] is None: first_success[i]=t
                obs.append(np.concatenate([s['lin'],s['ang'],s['gravity'],s['delta'],[math.sin(s['heading']),math.cos(s['heading'])],velocity[i],[dwell[i],s['distance']]]))
            with torch.inference_mode():
                a=torch.tanh(actor(torch.tensor(np.array(obs),dtype=torch.float32))).numpy()
            velocity[:,0]=np.clip(a[:,0],0,1)*.6
            velocity[:,1]=a[:,1]*.2
            velocity[:,2]=a[:,2]*.45
        if grasp is not None and all(v is not None for v in first_success):
            # Dock completion latches a zero locomotion request for the grasp robot.
            # Keep dynamic balance control active; never freeze/reset root poses.
            velocity[0]=0.
        if step%4==0:
            obs=[]
            for i,(r,s) in enumerate(zip(robots,state)):
                qrel=r.get_joint_positions()[ids[i]]-defaults
                qvel=r.get_joint_velocities()[ids[i]].copy()
                history=last_low[i].copy()
                if i==0 and args.walker_arm_shield and walker_hold is not None:
                    columns=walker_hold['columns']
                    qrel[columns]=walker_hold['qrel'][columns]
                    qvel[columns]=0.
                    history[columns]=walker_hold['action'][columns]
                obs.append(np.concatenate([s['lin'],s['ang'],s['gravity'],velocity[i],qrel,qvel,history]))
            with torch.inference_mode(): last_low=walker(torch.tensor(np.array(obs),dtype=torch.float32)).numpy()
            if not np.isfinite(last_low).all(): raise RuntimeError('NONFINITE_ACTION')
            for i in range(4): targets[i][ids[i]]=defaults+.5*last_low[i]
        nav_complete=bool(np.all(dwell>=1.-1e-6))
        if args.gripper_close_audit and nav_complete:
            if not args.physical_gripper:
                raise RuntimeError('--gripper-close-audit requires --physical-gripper')
            if gripper_audit_start is None: gripper_audit_start=t
            alpha=min(1.,(t-gripper_audit_start)/args.gripper_audit_seconds)
            for q,fingers in zip(targets,gripper_ids): q[fingers]=alpha*args.gripper_close_target
        if grasp is not None:
            if nav_complete and grasp.phase=='NAV' and not grasp.done:
                grasp.start(t)
                walker_hold={'columns':[c for c,n in enumerate(BODY_19) if 'shoulder' in n or 'elbow' in n],
                             'qrel':grasp.safe[ids[0]]-defaults,'action':last_low[0].copy()}
            if args.stance_control=='measured_hold' and grasp.phase!='NAV':
                lower=[i for i in ids[0] if i not in grasp.upper]
                targets[0][lower]=grasp.safe[lower]
            if step%4==0 and grasp.phase!='NAV': grasp.update(t,.02,targets[0])
            if grasp.failure: failure=grasp.failure
        for r,q,apply_ids in zip(robots,targets,command_ids):
            r.apply_action(ArticulationAction(joint_positions=q[apply_ids],joint_indices=apply_ids))
        world.step(render=False)
        if step%20==0:
            row={'time_s':t,'robots':[]}
            for i,s in enumerate(state):
                item=items[i]
                local=s['pos'][:2]-np.array(item['dock_world_xy'])
                if s['pos'][2]-floor<.65 or -s['gravity'][2]<math.cos(math.radians(45)): failure=item['robot']+':FALL'
                if abs(local[0])<.68 and .28<local[1]<1.54: failure=item['robot']+':RACK_EXCLUSION'
                grip_values=[]
                if args.physical_gripper:
                    current=robots[i].get_joint_positions()
                    grip_values=[float(current[index]) for index in gripper_ids[i]]
                row['robots'].append({'robot':item['robot'],'position':s['pos'].tolist(),'distance_m':s['distance'],'heading_deg':math.degrees(s['heading']),'speed_mps':s['speed'],'dwell_s':float(dwell[i]),'primary_gripper_rad':grip_values,
                    'velocity_command':velocity[i].tolist(),'grasp_phase':grasp.phase if grasp is not None and i==0 else None,
                    'joint_positions':robots[i].get_joint_positions().tolist() if args.physical_gripper else []})
            records.append(row)
            if step%200==0: print('FLEET_PROGRESS',json.dumps(row),flush=True)
        if not args.no_video and t+1e-8>=next_frame:
            rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False)
            raw=[np.asarray(a.get_data()) for a in annotators]
            if any(d.ndim!=3 for d in raw): raise RuntimeError('EMPTY_FRAME')
            overview=Image.fromarray(raw[0][:,:,:3])
            mosaic=Image.new('RGB',(1280,720))
            for i,d in enumerate(raw[1:5]):
                img=Image.fromarray(d[:,:,:3]); draw=ImageDraw.Draw(img)
                draw.rectangle((0,0,640,40),fill=(8,15,24))
                draw.text((10,7),f"{items[i]['robot']} -> {items[i]['rack_path'].split('_')[-1]} | t={t:.2f}s",fill='white')
                draw.text((10,23),f"distance={state[i]['distance']:.3f}m | hold={dwell[i]:.1f}s | RL checkpoint {ckpt.stem}",fill=(90,235,190))
                mosaic.paste(img,((i%2)*640,(i//2)*360))
            draw=ImageDraw.Draw(overview); draw.rectangle((0,0,1280,48),fill=(8,15,24))
            draw.text((18,12),f"JM H1 x4 | learned rack navigation + frozen walker | t={t:.2f}s",fill='white',font_size=20)
            if grasp is None:
                for writer,img in zip(writers,[overview,mosaic]): writer.append_data(np.asarray(img))
            if grasp is not None:
                for writer,data in zip(writers,raw):
                    img=Image.fromarray(data[:,:,:3]); draw=ImageDraw.Draw(img)
                    draw.rectangle((0,0,1280,70),fill=(8,15,24))
                    scope='PHYSICAL LIFT + SIDE / NO ATTACH' if args.lift_transfer else 'NO ATTACH / NO LIFT'
                    draw.text((16,10),f'JM H1 RING | {grasp.phase} | t={t:.2f}s | {scope}',fill='white',font_size=20)
                    draw.text((16,40),f"error={grasp.last.get('error_m',0)*1000:.1f}mm | rise={grasp.last.get('object_rise_m',0)*1000:.1f}mm | contact={np.round(grasp.last.get('contact_N',[]),2)} N",fill='white',font_size=18)
                    writer.append_data(np.asarray(img))
                    if frames%24==0 or failure: img.save(out/f'{"front" if writer is writers[0] else "side"}_{frames:04d}.jpg')
            if frames%48==0: mosaic.save(out/f'frame_{frames:04d}.jpg')
            if failure: mosaic.save(out/'failure_frame.jpg')
            frames+=1; next_frame=frames/24
        audit_complete=(not args.gripper_close_audit) or (gripper_audit_start is not None and t-gripper_audit_start>=args.gripper_audit_seconds)
        if failure or (grasp.done if grasp is not None else (nav_complete and audit_complete)): break
    gripper_audit=[]
    gripper_audit_pass=not args.gripper_close_audit
    if args.physical_gripper:
        for r,fingers in zip(robots,gripper_ids):
            values=r.get_joint_positions()
            primary=[float(values[index]) for index in fingers]
            gripper_audit.append({'robot':r.name,'primary_joint_rad':primary,'primary_target_rad':args.gripper_close_target if args.gripper_close_audit else 0.0})
        if args.gripper_close_audit:
            gripper_audit_pass=all(all(abs(value-args.gripper_close_target)<=.05 for value in record['primary_joint_rad']) for record in gripper_audit)
            if not gripper_audit_pass and failure is None: failure='GRIPPER_TARGET_TRACKING_FAILED'
    result={'pass':bool(np.all(dwell>=1.-1e-6) and not failure and gripper_audit_pass),'failure':failure,
            'checkpoint':str(ckpt),'checkpoint_sha256':hashlib.sha256(ckpt.read_bytes()).hexdigest(),
            'world':str(worldpath),'world_sha256':hashlib.sha256(worldpath.read_bytes()).hexdigest(),
            'duration_s':t,'frames':frames,'first_success_s':first_success,
            'all_four_dynamic':True,'root_writes_during_motion':False,
            'physical_gripper_primary_only':args.physical_gripper,
            'gripper_close_audit':gripper_audit,
            'gripper_close_audit_pass':gripper_audit_pass,
            'contact_safety_validated':False,'self_collisions':False,'validation_scope':'single four-robot rollout, geometry/fall/dock gates; not robust contact-safety proof'}
    if grasp is not None:
        result['grasp']=grasp.save(); result['pass']=grasp.success and not failure
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (out/'telemetry.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    print('FLEET_RESULT',json.dumps(result),flush=True)
except BaseException:
    trace=traceback.format_exc(); print(trace,flush=True)
    (out/'exception.txt').write_text(trace,encoding='utf-8')
    if grasp is not None:
        grasp.failure='EXECUTION_EXCEPTION: '+trace.splitlines()[-1]; grasp.done=True
finally:
    if grasp is not None: grasp.save()
    for w in writers: w.close()
    for product in products: product.destroy()
    world.stop(); app.close()
