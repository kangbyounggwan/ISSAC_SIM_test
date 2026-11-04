"""Official native H1 locomotion baseline in the unmodified JM equipment world.

Session-only changes; physical movement, no root teleport, no grasp claim.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from jm_wb5_turn.jm_sequence import WalkTurnSequence,ContinuousWalkTurnSequence
from jm_wb5_turn.operator_bridge import OperatorBridge

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--seed',type=int,default=2159858089)
p.add_argument('--mode',choices=['strict_stops','continuous'],default='strict_stops')
p.add_argument('--operator-bridge',action='store_true',help='Publish local status/JPEG and honor cooperative simulation cancellation')
p.add_argument('--operator-sensors',action='store_true',help='Observe real JM RGB-D and PhysX lidar; does not change the policy')
args=p.parse_args()
args.out.mkdir(parents=True,exist_ok=False)
out=args.out.resolve()
bridge=OperatorBridge(out,args.operator_bridge)
bridge.publish(dict(phase='INITIALIZING',time_s=0.,state='starting'))
meta=json.loads((ROOT/'artifacts/xnavdp_20260929/jm_capture_002/observation.json').read_text())
scene=json.loads((ROOT/'artifacts/jm_stop_pose_20260929/scene_001.json').read_text())


def check_sources():
    checks={path:hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest
            for path,digest in meta['source_sha256'].items()}
    if not all(checks.values()):
        raise RuntimeError('JM source changed: '+str([k for k,v in checks.items() if not v]))
    return checks


check_sources()
from jm_xnavdp.spawn import sample_spawn
from keti_navigation.planner import line_free,clearance
floor=float(scene['floor_z'])
obstacles=[o for o in scene['obstacles'] if o['max'][2]>floor+.15 and o['min'][2]<floor+1.75]
spawn=sample_spawn(obstacles,args.seed)
fsm=(WalkTurnSequence if args.mode=='strict_stops' else ContinuousWalkTurnSequence)(spawn['selected_xy'])
if not all(line_free(a,b,obstacles,.65) for a,b in ((fsm.spawn,fsm.a),(fsm.a,fsm.b))):
    raise RuntimeError('Fixed L route blocked; no automatic goal or seed change')
(out/'plan.json').write_text(json.dumps(dict(mode=args.mode,spawn=spawn,goal_a=fsm.a,goal_b=fsm.b,
    phases=fsm.PHASES,limits=fsm.LIMITS,timeouts=fsm.TIMEOUTS,route_inflation_m=.65),indent=2))

from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1280,'height':720})
writer=None
sensors=None
rows=[]
try:
    import numpy as np
    import torch
    import omni.usd
    import omni.client
    import omni.replicator.core as rep
    import carb.settings
    import imageio.v2 as imageio
    from PIL import Image,ImageDraw,ImageFont
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    from isaacsim.robot.policy.examples.controllers.config_loader import get_robot_joint_properties
    from isaacsim.storage.native import get_assets_root_path
    from jm_xnavdp.contact_audit import ContactAudit
    from jm_wb5_turn.footstep_metrics import foot_events,angle_travel

    torch.set_num_threads(1)
    carb.settings.get_settings().set('/rtx/post/histogram/enabled',False)
    ctx=omni.usd.get_context()
    ctx.open_stage(meta['world'])
    for _ in range(10): app.update()
    stage=ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    removed=[]
    for path in ('/World/H1Fleet','/World/Robots'):
        group=stage.GetPrimAtPath(path)
        if group and group.IsActive():
            removed.append(path)
            group.SetActive(False)
    # Keep equipment colliders; static products are not part of this motion trial.
    static_bodies=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            static_bodies.append(str(prim.GetPath()))
    physics=stage.GetPrimAtPath('/World/PhysicsScene')
    if not physics: physics=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
    ps=PhysxSchema.PhysxSceneAPI.Apply(physics)
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr('MBP')
    ps.CreateTimeStepsPerSecondAttr(200)
    world=World(physics_dt=.005,rendering_dt=.04,stage_units_in_meters=1.,
                physics_prim_path='/World/PhysicsScene',backend='numpy')
    asset_root=get_assets_root_path()
    usd=asset_root+'/Isaac/Robots/Unitree/H1/h1.usd'
    robot_path='/World/JMWalkTurn/H1'
    reference=H1FlatTerrainPolicy(robot_path,usd_path=usd,
        position=np.array([*fsm.spawn,floor+1.05]),
        orientation=np.array([math.cos(math.pi/4),0.,0.,math.sin(math.pi/4)]))
    robot=world.scene.add(reference.robot)
    body=None
    for node in ast.parse((ROOT/'jm_coop4_rl/h1_rack_env.py').read_text(encoding='utf-8')).body:
        if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='BODY_19':
            body=ast.literal_eval(node.value)
    native=[n.removesuffix('_joint') for n in body]
    contract=get_robot_joint_properties(reference.policy_env_params,native)
    rp=stage.GetPrimAtPath(robot_path)
    for prim in Usd.PrimRange(rp):
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in native:
            j=native.index(prim.GetName())
            drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
            drive.CreateStiffnessAttr(contract[2][j]*math.pi/180)
            drive.CreateDampingAttr(contract[3][j]*math.pi/180)
            drive.CreateMaxForceAttr(contract[0][j])
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(math.degrees(contract[1][j]))
    feet=[]
    for side in ('left','right'):
        feet.append(world.scene.add(RigidPrim(robot_path+'/'+side+'_ankle_link',
            name=side+'_foot',reset_xform_properties=False,track_contact_forces=True,
            disable_stablization=False)))
    torso=world.scene.add(RigidPrim(robot_path+'/torso_link',name='torso',
        reset_xform_properties=False,prepare_contact_sensors=False))
    contacts=ContactAudit(stage,robot_path,out)
    ground_path='/World/Dadong/F2/slab'
    if not stage.GetPrimAtPath(ground_path): raise RuntimeError('JM ground collider missing')
    if args.operator_sensors:
        from jm_wb5_turn.operator_sensors import OperatorSensors
        sensors=OperatorSensors(stage,robot_path,out,bridge)
    world.reset()
    if robot.dof_names!=native: raise RuntimeError('Native joint order mismatch')
    assert reference._dt==.005 and reference._decimation==4
    defaults=np.asarray(contract[4],dtype=np.float32)
    kp,kd=np.asarray(contract[2]),np.asarray(contract[3])
    controller=robot.get_articulation_controller()
    controller.set_effort_modes('force')
    controller.switch_control_mode('position')
    robot.set_joint_positions(defaults)
    robot.set_joint_velocities(np.zeros(19))
    robot.set_world_velocity(np.zeros(6))
    official_bytes=bytes(omni.client.read_file(asset_root+'/Isaac/Samples/Policies/H1_Policies/h1_policy.pt')[2])
    policy_hash=hashlib.sha256(official_bytes).hexdigest()
    if policy_hash!='a734d8ae13b8c3690138baf8d01d7d497f987616da2c53e00a43e0ceb31452d7':
        raise RuntimeError('Official policy differs from verified turn baseline')
    (out/'official_policy.pt').write_bytes(official_bytes)
    manifest=dict(mode=args.mode,world=meta['world'],asset=usd,policy_sha256=policy_hash,
        robot_path=robot_path,native_joint_order=native,defaults=defaults.tolist(),
        stiffness=kp.tolist(),damping=kd.tolist(),effort=contract[0],dt=.005,decimation=4,
        deactivated_groups=removed,static_product_bodies=static_bodies,
        source_sha256=meta['source_sha256'],source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        controller_sha256=hashlib.sha256((ROOT/'jm_wb5_turn/jm_sequence.py').read_bytes()).hexdigest(),
        physics_root_writes_during_motion=0,scope='Official native H1 baseline in JM; no WB5 deployment, gripper, grasp, payload, A*, X-NavDP or new training',
        contact_preparation='Foot reporting sets foot sleep threshold to zero; contact stabilization preserved. Raw contact report enabled on all robot bodies.')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    inventory=[]
    for prim in [physics,stage.GetPrimAtPath(ground_path),*list(Usd.PrimRange(rp))]:
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.RevoluteJoint) or prim==physics or str(prim.GetPath())==ground_path:
            inventory.append(dict(path=str(prim.GetPath()),attributes={a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physx','drive:'))}))
    (out/'asset_inventory.json').write_text(json.dumps(inventory,indent=2))

    camera=UsdGeom.Camera.Define(stage,'/World/JMWalkTurn/Camera')
    camera.CreateFocalLengthAttr(27.)
    camera.CreateHorizontalApertureAttr(30.)
    camera.CreateVerticalApertureAttr(30*9/16)
    center=np.array([(fsm.spawn[0]+fsm.b[0])/2,(fsm.spawn[1]+fsm.b[1])/2,floor+.9])
    eye=center+np.array([4.,-5.5,3.])
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*center),Gf.Vec3d(0,0,1)).GetInverse())
    # Visual-only waypoints: no collision, no influence on policy observations.
    for name,xy,color in (('Start',fsm.spawn,(.15,.6,1.)),('A',fsm.a,(1.,.65,.1)),('B',fsm.b,(.15,1.,.3))):
        marker=UsdGeom.Cylinder.Define(stage,'/World/JMWalkTurn/'+name)
        marker.CreateRadiusAttr(.13); marker.CreateHeightAttr(.005)
        marker.AddTranslateOp().Set(Gf.Vec3d(xy[0],xy[1],floor+.012))
        marker.CreateDisplayColorAttr([color])
    rep.orchestrator.set_capture_on_play(False)
    product=rep.create.render_product(str(camera.GetPath()),(1280,720))
    ann=rep.AnnotatorRegistry.get_annotator('rgb'); ann.attach([product])
    clock=[0.]
    world.add_physics_callback('jm_sequence_clock',lambda dt:clock.__setitem__(0,clock[0]+dt))
    rep.orchestrator.step(rt_subframes=3,delta_time=0.,pause_timeline=False)
    t0=clock[0]
    writer=imageio.get_writer(str(out/'jm_walk_left_walk_720p.mp4'),fps=25,codec='libx264',quality=8,macro_block_size=None)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
    last=np.zeros(19,dtype=np.float32); targets=defaults.copy()
    frames=0; retries=[]; dark=[]; failure=None; runtime=[]; max_env_force=0.
    phase_frames=[]
    seen_phases=set()

    def yaw_of(q):
        w,x,y,z=map(float,q)
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

    for step in range(12001):
        t=step*.005
        pos,q=robot.get_world_pose()
        R=quat_to_rot_matrix(q).T
        lin=R@robot.get_linear_velocity(); ang=R@robot.get_angular_velocity()
        grav=R@np.array([0.,0.,-1.])
        yaw=yaw_of(q); tilt=math.degrees(math.acos(np.clip(-grav[2],-1.,1.)))
        speed=float(np.linalg.norm(lin[:2]))
        cmd=np.array(fsm.update(t,pos[:2],yaw,speed,float(ang[2]),tilt),dtype=np.float32)
        if fsm.failure: failure=fsm.failure
        if step%20==0 and bridge.cancelled(): failure='OPERATOR_CANCELLED'
        if pos[2]<floor+.65: failure='ROOT_HEIGHT_ABORT'
        contacts.sample(t)
        current_env_force=0.
        for pair in contacts.pairs.values():
            if abs(pair['last_s']-t)>1e-7: continue
            a,b=pair['colliders']
            if a.startswith(robot_path+'/') and b.startswith(robot_path+'/'): continue
            other=b if a.startswith(robot_path+'/') else a
            if other.startswith(ground_path): continue
            current_env_force=max(current_env_force,pair['max_impulse_Ns']/.005)
        max_env_force=max(max_env_force,current_env_force)
        if current_env_force>.20: failure='ENVIRONMENT_CONTACT_ABORT'
        if clearance(pos[:2],obstacles)<.38: failure='ENVIRONMENT_ENVELOPE_ABORT'
        if step%200==0 or step in (1,20,40):
            actual_kp,actual_kd=controller.get_gains()
            good=np.allclose(actual_kp,kp) and np.allclose(actual_kd,kd)
            runtime.append(dict(time_s=t,pd_ok=bool(good),physics_clock_error_s=clock[0]-t0-t))
            if not good: failure='PD_CONTRACT_INVALID'
            if abs(clock[0]-t0-t)>.01: failure='PHYSICS_CLOCK_INVALID'
        if step%4==0 or failure or fsm.phase=='DONE':
            forces=[np.asarray(v.get_net_contact_forces(dt=.005))[0] for v in feet]
            poses=[v.get_world_poses() for v in feet]
            goal,heading=fsm.target()
            rows.append(dict(time_s=t,phase=fsm.phase,position_world=pos.tolist(),yaw_deg=math.degrees(yaw),
                yaw_rate_rps=float(ang[2]),tilt_deg=tilt,speed_mps=speed,velocity_command=cmd.tolist(),
                goal_xy=goal,goal_distance_m=math.dist(pos[:2],goal),
                heading_error_deg=math.degrees((heading-yaw+math.pi)%(2*math.pi)-math.pi),
                environment_force_proxy_n=current_env_force,environment_aabb_distance_m=clearance(pos[:2],obstacles),
                foot_force_n=[float(np.linalg.norm(f)) for f in forces],
                foot_force_world_n=[f.tolist() for f in forces],
                foot_position_world=[np.asarray(x[0])[0].tolist() for x in poses],
                foot_yaw_deg=[math.degrees(yaw_of(np.asarray(x[1])[0])) for x in poses],
                foot_velocity_world=[np.asarray(v.get_velocities())[0,:3].tolist() for v in feet],
                torso_yaw_world_deg=math.degrees(yaw_of(np.asarray(torso.get_world_poses()[1])[0])),
                joint_position=robot.get_joint_positions().tolist(),actual_physics_elapsed_s=clock[0]-t0))
            if step%40==0 or failure or fsm.phase=='DONE':
                bridge.publish(dict(rows[-1],state='cancelled' if failure=='OPERATOR_CANCELLED' else 'failed' if failure else 'completed' if fsm.phase=='DONE' else 'running',failure=failure))
        if step%8==0 or failure or fsm.phase=='DONE':
            before=clock[0]
            rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
            rgb=np.asarray(ann.get_data())[:,:,:3].copy()
            for retry in range(12):
                if np.mean(rgb[110:690,40:1240])>=12: break
                rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
                rgb=np.asarray(ann.get_data())[:,:,:3].copy()
                retries.append(dict(frame=frames,attempt=retry+1))
            if clock[0]!=before: raise RuntimeError('Recording advanced physics')
            if np.mean(rgb[110:690,40:1240])<12: dark.append(frames)
            img=Image.fromarray(rgb); d=ImageDraw.Draw(img)
            d.rectangle((0,0,1280,103),fill=(8,18,28))
            d.text((14,6),f'JM WORLD | WALK -> LEFT TURN -> WALK | {fsm.phase} | t={t:.2f}s | {args.mode}',font=font,fill='white')
            r=rows[-1]
            d.text((14,30),f'Goal error {r["goal_distance_m"]:.3f}m | yaw {math.degrees(yaw):.1f}deg | tilt {tilt:.1f}deg | speed {speed:.3f}m/s',font=font,fill='white')
            d.text((14,54),f'Command vx/vy/wz {cmd[0]:+.2f}/{cmd[1]:+.2f}/{cmd[2]:+.2f} | foot L/R {r["foot_force_n"][0]:.0f}/{r["foot_force_n"][1]:.0f}N | env {current_env_force:.2f}N',font=font,fill='white')
            d.text((14,78),'Official native H1 | physical gait, NO root teleport | no gripper / payload | blue START, orange A, green B',font=font,fill=(180,220,245))
            if failure:
                d.rectangle((0,660,1280,720),fill=(130,20,20)); d.text((14,675),'ABORT: '+failure,font=font,fill='white')
            writer.append_data(np.asarray(img))
            if frames%10==0 or failure or fsm.phase=='DONE': bridge.frame(img)
            if sensors: sensors.capture(t,publish=frames%10==0 or bool(failure) or fsm.phase=='DONE')
            if clock[0]!=before: raise RuntimeError('Sensor capture advanced physics')
            if fsm.phase not in seen_phases or frames%125==0 or failure or fsm.phase=='DONE':
                name=f'frame_{frames:04d}_{fsm.phase}.jpg'; img.save(out/name)
                phase_frames.append(dict(frame=frames,time_s=t,phase=fsm.phase,file=name))
                seen_phases.add(fsm.phase)
            frames+=1
        if failure or fsm.phase=='DONE': break
        if step%4==0:
            obs=np.r_[lin,ang,grav,cmd,robot.get_joint_positions()-defaults,robot.get_joint_velocities(),last].astype(np.float32)
            with torch.inference_mode(): last=reference.policy(torch.from_numpy(obs)[None]).numpy()[0]
            if not np.isfinite(last).all(): raise RuntimeError('Nonfinite policy action')
            targets=defaults+.5*last
        robot.apply_action(ArticulationAction(joint_positions=targets))
        world.step(render=False)
        if step%1000==0: print('JM_PROGRESS',t,fsm.phase,'distance',rows[-1]['goal_distance_m'],flush=True)
    if failure is None and fsm.phase!='DONE': failure='TOTAL_TIMEOUT'
    raw=contacts.save()
    final=rows[-1]
    by_phase={}
    for name in fsm.PHASES:
        rr=[r for r in rows if r['phase']==name]
        if not rr: continue
        events=[foot_events(rows,k,rr[0]['time_s'],rr[-1]['time_s']) if len(rows)>1 else [] for k in range(2)]
        by_phase[name]=dict(start_s=rr[0]['time_s'],end_s=rr[-1]['time_s'],
            verified_footsteps=[sum(e['verified_lift_landing'] for e in es) for es in events],
            pelvis_turn_deg=angle_travel(rr,'yaw_deg'),torso_turn_deg=angle_travel(rr,'torso_yaw_world_deg'),
            final_distance_m=rr[-1]['goal_distance_m'],max_tilt_deg=max(r['tilt_deg'] for r in rr))
    result=dict(mode=args.mode,completed=fsm.phase=='DONE' and failure is None,failure=failure,final_phase=fsm.phase,
        elapsed_simulation_s=final['time_s'],frames=frames,fps=25,resolution=[1280,720],
        final_position_world=final['position_world'],final_distance_m=math.dist(final['position_world'][:2],fsm.b),
        final_yaw_deg=final['yaw_deg'],final_speed_mps=final['speed_mps'],turn_max_drift_m=fsm.turn_max_drift,
        in_place_20cm_pass=fsm.turn_origin is not None and fsm.turn_max_drift<=.20,
        max_tilt_deg=max(r['tilt_deg'] for r in rows),max_environment_contact_impulse_over_dt_n=max_env_force,
        events=fsm.events,phase_audit=by_phase,dark_main_scene_frames=dark,render_retries=retries,
        source_unchanged=check_sources(),physical_root_teleport=False,
        strict_body_stop_validated=args.mode=='strict_stops' and fsm.phase=='DONE' and failure is None,
        precise_feet_still_stop_validated=False,grasp_or_payload_validated=False,
        contact_scope='200Hz PhysX reports; non-ground external point impulse/dt proxy (not summed force); feet net force 50Hz')
    (out/'telemetry.json').write_text(json.dumps(rows,indent=2))
    (out/'runtime_contract.json').write_text(json.dumps(runtime,indent=2))
    (out/'phase_frames.json').write_text(json.dumps(phase_frames,indent=2))
    (out/'result.json').write_text(json.dumps(result,indent=2))
    bridge.publish(dict(final,state='cancelled' if failure=='OPERATOR_CANCELLED' else 'failed' if failure else 'completed',failure=failure,result=result))
    print('JM_RESULT',json.dumps(result),flush=True)
except Exception:
    (out/'exception.txt').write_text(traceback.format_exc())
    bridge.publish(dict(phase='ERROR',state='failed',failure='RUNTIME_EXCEPTION'))
    raise
finally:
    if writer: writer.close()
    if sensors: sensors.close()
    app.close()
