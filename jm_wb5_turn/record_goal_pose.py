"""720p goal-turn audits, stopping BEFORE auto reset. No model promotion."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--policy',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--angles',type=float,nargs='+',default=[45,-45,90,-90,180,-180])
p.add_argument('--seed',type=int,default=20261002)
p.add_argument('--policy-inputs',type=int,choices=[69,76],default=76,
    help='69 for the unchanged WB5 baseline; 76 for the goal-pose candidate')
p.add_argument('--label',default='WB5 GOAL-TURN CANDIDATE')
AppLauncher.add_app_launcher_args(p)
args=p.parse_args()
args.headless=True
args.enable_cameras=True
if args.out.exists(): p.error('Output must be fresh')
out=args.out.resolve();out.mkdir(parents=True)
app=AppLauncher(args).app
env=None;writer=None


def save(path,data):
    path.write_text(json.dumps(data,indent=2),encoding='utf-8')


try:
    import numpy as np
    import torch
    import imageio.v2 as imageio
    from PIL import Image,ImageDraw,ImageFont
    from jm_wb5_turn.goal_env import GoalTurnEnvCfg,GoalTurnEnv
    from jm_xnavdp.h1_contract import load_constants,gains

    class EpisodeEnd(Exception): pass

    class AuditEnv(GoalTurnEnv):
        audit=False
        def _reset_idx(self,env_ids):
            if self.audit and len(env_ids):
                raise EpisodeEnd()
            super()._reset_idx(env_ids)

    cfg=GoalTurnEnvCfg();cfg.scene.num_envs=1;cfg.seed=args.seed;cfg.sim.device=args.device
    cfg.observations.policy.enable_corruption=False
    cfg.events.reset_base.params['pose_range']={'x':(0.,0.),'y':(0.,0.),'yaw':(0.,0.)}
    cfg.viewer.resolution=(1280,720);cfg.sim.render_interval=8
    cfg.viewer.eye=(3.1,-3.9,2.3);cfg.viewer.lookat=(0.,0.,.9)
    env=AuditEnv(cfg=cfg,render_mode='rgb_array')
    robot=env.scene['robot']
    sensor=env.scene.sensors['contact_forces']
    const=load_constants(ROOT)
    joint_ids,joint_names=robot.find_joints(list(const['BODY_19']),preserve_order=True)
    foot_ids,foot_names=robot.find_bodies(['left_ankle_link','right_ankle_link'],preserve_order=True)
    contact_ids,_=sensor.find_bodies(foot_names,preserve_order=True)
    torso_ids,torso_names=robot.find_bodies('torso_link')
    if joint_names!=list(const['BODY_19']) or len(torso_ids)!=1:
        raise RuntimeError('Diagnostic joint/body mapping mismatch')
    expected=torch.tensor([gains(n) for n in joint_names],device=env.device)
    def contract():
        actual=torch.stack([robot.root_physx_view.get_dof_stiffnesses().to(env.device)[:,joint_ids],
            robot.root_physx_view.get_dof_dampings().to(env.device)[:,joint_ids],
            robot.root_physx_view.get_dof_max_forces().to(env.device)[:,joint_ids]],dim=-1)
        if not torch.allclose(actual,expected.expand_as(actual),atol=1e-5,rtol=1e-5):
            raise RuntimeError('Runtime PD/effort mismatch; comparison invalid')
    def yaw_deg(quat):
        w,x,y,z=[float(v) for v in quat]
        return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    policy=torch.jit.load(str(args.policy),map_location=env.device).eval()
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',21)
    small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    results={}
    save(out/'manifest.json',dict(policy=str(args.policy.resolve()),
        policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),angles=args.angles,
        seed=args.seed,resolution=[1280,720],fps=25,phase_seconds=[2,12,4],
        policy_inputs=args.policy_inputs,label=args.label,joint_order=joint_names,feet=foot_names,
        torso=torso_names,telemetry_hz=50,physics_dt=.005,policy_dt=.02,
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        footstep_diagnostic=dict(unloaded_force_n=5,landing_force_n=20,
            min_air_s=.08,min_lift_m=.02,min_landing_s=.08),
        automatic_reset_blocked=True,root_teleport_during_motion=False,
        camera='position-follow only, world displacement explicitly logged',
        scope='fixed initial pose, one seed; not multi-seed or rack clearance certification'))
    for angle in args.angles:
        label=('left' if angle>0 else 'right')+'_'+str(abs(angle)).replace('.0','')
        folder=out/label;folder.mkdir()
        env.audit=False
        obs,_=env.reset(seed=args.seed)
        cmd=env.command_manager.get_term('base_velocity')
        cmd.groups[:]=1 if angle>0 else 2
        cmd.target_delta[:]=math.radians(angle)
        cmd.raw[:]=0.
        cmd._update_command()
        obs=env.observation_manager.compute()
        env.audit=True
        contract()
        # Initialize the RGB render product before renderer-only warmup.
        env.render()
        for _ in range(8):env.sim.render()
        writer=imageio.get_writer(str(folder/'turn_720p.mp4'),fps=25,codec='libx264',quality=8,macro_block_size=None)
        rows=[];frames=0;ended=False;failure=None;last_command=[0.,0.,0.];last_action=[0.]*19
        for step in range(901):
            xy,error,phase,travel=cmd.live()
            pos=robot.data.root_pos_w[0].cpu().numpy().copy()
            row=dict(time_s=step*.02,phase=['SETTLE','TURN_TO_GOAL','BRAKE_HOLD'][int(phase[0])],
                goal_deg=angle,yaw_deg=float(travel[0])*180/math.pi,
                yaw_error_deg=float(error[0])*180/math.pi,drift_m=float(xy[0].norm()),
                speed_mps=float(robot.data.root_lin_vel_w[0,:2].norm()),
                yaw_rate_rps=float(robot.data.root_ang_vel_w[0,2]),height_m=float(pos[2]),
                command_applied=last_command,position_world=pos.tolist(),
                success_now=bool(cmd.instantaneous_success()[0]),hold_steps=int(cmd.hold_steps[0]),
                failure=failure)
            foot_force=sensor.data.net_forces_w[0,contact_ids]
            row.update(foot_force_n=foot_force.norm(dim=-1).cpu().tolist(),
                foot_force_world_n=foot_force.cpu().tolist(),
                foot_position_world=robot.data.body_pos_w[0,foot_ids].cpu().tolist(),
                foot_velocity_world=robot.data.body_lin_vel_w[0,foot_ids].cpu().tolist(),
                foot_quat_world=robot.data.body_quat_w[0,foot_ids].cpu().tolist(),
                foot_yaw_deg=[yaw_deg(q) for q in robot.data.body_quat_w[0,foot_ids]],
                foot_air_time_s=sensor.data.current_air_time[0,contact_ids].cpu().tolist(),
                foot_contact_time_s=sensor.data.current_contact_time[0,contact_ids].cpu().tolist(),
                pelvis_yaw_world_deg=yaw_deg(robot.data.root_quat_w[0]),
                torso_yaw_world_deg=yaw_deg(robot.data.body_quat_w[0,torso_ids[0]]),
                joint_position=robot.data.joint_pos[0,joint_ids].cpu().tolist(),
                joint_velocity=robot.data.joint_vel[0,joint_ids].cpu().tolist(),
                joint_target=robot.data.joint_pos_target[0,joint_ids].cpu().tolist(),
                action_applied=last_action,
                tilt_deg=math.degrees(math.acos(float((-robot.data.projected_gravity_b[0,2]).clamp(-1,1)))))
            rows.append(row)
            if step%100==0:
                contract()
                print('FOOT_AUDIT_PROGRESS',args.label,label,step,flush=True)
            if step%2==0 or ended:
                env.sim.set_camera_view(eye=(pos+np.array([3.1,-3.9,1.4])).tolist(),target=(pos+np.array([0.,0.,.05])).tolist())
                env.sim.render();rgb=env.render()
                if rgb.shape!=(720,1280,3) or float(rgb[100:].mean())<4:raise RuntimeError('Invalid 720p frame')
                img=Image.fromarray(rgb);draw=ImageDraw.Draw(img)
                draw.rectangle((0,0,1280,100),fill=(9,19,30))
                draw.text((15,8),f'{args.label} | target={angle:+.0f}deg | {row["phase"]} | t={step*.02:.2f}s',font=font,fill='white')
                draw.text((15,38),f'yaw={row["yaw_deg"]:+.1f}deg | error={row["yaw_error_deg"]:+.1f}deg | anchor drift={row["drift_m"]*100:.1f}cm',font=font,fill='white')
                draw.text((15,67),f'speed={row["speed_mps"]:.3f}m/s | yaw rate={row["yaw_rate_rps"]:.3f}rad/s | NO TELEPORT | NO ACTION NOISE',font=small,fill=(200,222,240))
                draw.text((15,106),f'FOOT CONTACT L/R: {row["foot_force_n"][0]:.0f}/{row["foot_force_n"][1]:.0f} N | air: {row["foot_air_time_s"][0]:.2f}/{row["foot_air_time_s"][1]:.2f}s',font=small,fill='white',stroke_width=1,stroke_fill='black')
                draw.text((15,689),'CAMERA FOLLOWS POSITION | inspect world drift, not screen centering | PROVISIONAL FLAT-GROUND TEST',font=small,fill=(255,235,90),stroke_width=1,stroke_fill='black')
                if failure:draw.text((15,120),'FAILURE: '+failure,font=font,fill=(255,70,70))
                writer.append_data(np.asarray(img));frames+=1
                if step%100==0 or ended:img.save(folder/f'frame_{step:04d}.jpg')
            if ended or step==900:break
            with torch.inference_mode():action=policy(obs['policy'][:,:args.policy_inputs])
            if not torch.isfinite(action).all():raise RuntimeError('Nonfinite action')
            last_action=action[0].cpu().tolist()
            last_command=cmd.command[0].cpu().tolist()
            try:obs,_,_,_,_=env.step(action)
            except EpisodeEnd:
                ended=True
                names=[name for name in env.termination_manager.active_terms if bool(env.termination_manager.get_term(name)[0]) and name!='time_out']
                failure=' / '.join(names) if names else None
                cmd._update_metrics()
                cmd._update_command()
        writer.close();writer=None
        save(folder/'telemetry.json',rows)
        max_drift=max(r['drift_m'] for r in rows)
        success=(failure is None and len(rows)==901 and bool(cmd.hold_steps[0]>=38) and max_drift<=.20)
        result=dict(scenario=label,success=success,failure=failure,frames=frames,
            duration_s=rows[-1]['time_s'],final_yaw_error_deg=rows[-1]['yaw_error_deg'],
            max_drift_m=max_drift,final_speed_mps=rows[-1]['speed_mps'],
            final_yaw_rate_rps=rows[-1]['yaw_rate_rps'],hold_steps=int(cmd.hold_steps[0]),
            runtime_contract_passed=True,production_replaced=False)
        save(folder/'result.json',result);results[label]=result
        save(out/'results.json',results)
        print('GOAL_VIDEO_RESULT',json.dumps(result),flush=True)
    save(out/'status.json',dict(state='EVALUATION_COMPLETE',scenarios=results,production_replaced=False))
except BaseException:
    error=traceback.format_exc();save(out/'status.json',dict(state='FAILED',error=error));print(error,flush=True)
    raise
finally:
    if writer is not None:writer.close()
    if env is not None:env.close()
    app.close()
