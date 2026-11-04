"""Deterministic WB5 turn comparison. No training, asset edits, or model promotion."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--checkpoint', type=Path, required=True)
p.add_argument('--seed', type=int, default=20261001)
p.add_argument('--camera', choices=['fixed','follow'], default='fixed')
p.add_argument('--scenarios', nargs='+', default=['original_left', 'original_right', 'candidate_left', 'candidate_right'])
AppLauncher.add_app_launcher_args(p)
args = p.parse_args()
args.headless = True
args.enable_cameras = True
if args.out.exists(): p.error('Output must be fresh')
out = args.out.resolve()
out.mkdir(parents=True)
app = AppLauncher(args).app
env = None
writer = None


def save(path, data):
    path.write_text(json.dumps(data, indent=2, default=str), encoding='utf-8')


try:
    import numpy as np
    import torch
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont
    from isaaclab.envs import ManagerBasedRLEnv
    from jm_wb5_turn.env import TurnEnvCfg, ASSET
    from jm_xnavdp.h1_contract import load_constants, gains

    class TerminalStop(Exception):
        pass

    class AuditEnv(ManagerBasedRLEnv):
        audit_running = False

        def _reset_idx(self, env_ids):
            # Stop before an auto reset can hide a fall or teleport in the recording.
            if self.audit_running and len(env_ids):
                raise TerminalStop('Training termination condition reached; auto-reset blocked')
            return super()._reset_idx(env_ids)

    const = load_constants(ROOT)
    source = ROOT/'artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt'
    original_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if original_hash != const['WALKER_SHA256']: raise RuntimeError('Original WB5 hash mismatch')
    ckpt = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    candidate = torch.nn.Sequential(torch.nn.Linear(69, 128), torch.nn.ELU(),
        torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 19))
    candidate.load_state_dict({k.removeprefix('actor.'):v for k,v in ckpt['model_state_dict'].items() if k.startswith('actor.')}, strict=True)
    candidate.to(args.device).eval()
    original = torch.jit.load(str(source), map_location=args.device).eval()
    cfg = TurnEnvCfg()
    cfg.seed = args.seed
    cfg.scene.num_envs = 1
    cfg.sim.device = args.device
    cfg.observations.policy.enable_corruption = False
    cfg.events.reset_base.params['pose_range'] = {'x':(0.,0.), 'y':(0.,0.), 'yaw':(0.,0.)}
    cfg.viewer.resolution = (1280, 720)
    cfg.viewer.eye = (2.6, -3.3, 2.1)
    cfg.viewer.lookat = (0., 0., .95)
    cfg.viewer.origin_type = 'env'
    cfg.viewer.env_index = 0
    # Only render every two policy steps, retaining unchanged .005s physics.
    cfg.sim.render_interval = 8
    env = AuditEnv(cfg=cfg, render_mode='rgb_array')
    robot = env.scene['robot']
    sensor = env.scene.sensors['contact_forces']
    ids, names = robot.find_joints(list(const['BODY_19']), preserve_order=True)
    foot_ids, foot_names = robot.find_bodies(['left_ankle_link', 'right_ankle_link'], preserve_order=True)
    contact_ids, _ = sensor.find_bodies(['left_ankle_link', 'right_ankle_link'], preserve_order=True)
    if names != list(const['BODY_19']): raise RuntimeError('Joint order mismatch')
    manifest = dict(source=str(source), source_sha256=original_hash, checkpoint=str(args.checkpoint.resolve()),
        checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(), checkpoint_iteration=ckpt['iter'],
        asset=str(ASSET), asset_sha256=hashlib.sha256(ASSET.read_bytes()).hexdigest(), seed=args.seed,
        comparison='same fixed initial root pose; no observation or action noise',
        command='2s stand, 8s pure yaw +/-0.25rad/s, 2s stand',
        resolution=[1280,720], fps=25, camera=args.camera, physics_dt=.005, policy_dt=.02,
        root_writes_during_motion=False, automatic_reset_blocked=True, feet=foot_names,
        limitations=['single seed fixed-heading test', 'no racks, payload or self-collision validation',
                    'no walk-turn, 180-degree, reversal or multi-seed validation'],
        eval_config=cfg.to_dict(), code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    save(out/'manifest.json', manifest)
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 21)
    small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 18)
    all_results = {}
    for scenario in args.scenarios:
        if scenario not in ['original_left','original_right','candidate_left','candidate_right']:
            raise ValueError(scenario)
        folder = out/scenario
        folder.mkdir()
        sign = 1 if scenario.endswith('left') else -1
        policy = original if scenario.startswith('original') else candidate
        env.audit_running = False
        obs, _ = env.reset(seed=args.seed)
        cmd = env.command_manager.get_term('base_velocity')
        cmd.groups[:] = 1 if sign > 0 else 2
        cmd.raw[:] = torch.tensor([0.,0.,.25],device=env.device)
        cmd.elapsed[:] = 0.
        cmd.vel_command_b[:] = 0.
        obs = env.observation_manager.compute()
        env.audit_running = True
        expected = torch.tensor([gains(n) for n in names],device=env.device)
        def contract():
            actual = torch.stack([robot.root_physx_view.get_dof_stiffnesses().to(env.device)[:,ids],
                robot.root_physx_view.get_dof_dampings().to(env.device)[:,ids],
                robot.root_physx_view.get_dof_max_forces().to(env.device)[:,ids]],dim=-1)
            if not torch.allclose(actual,expected.expand_as(actual),atol=1e-5,rtol=1e-5):
                raise RuntimeError('Runtime PD/effort mismatch')
        contract()
        # Renderer warmup only; no physics stepping or unrecorded motion.
        env.render()
        for _ in range(8): env.sim.render()
        writer = imageio.get_writer(str(folder/'turn_720p.mp4'), fps=25, codec='libx264', quality=8, macro_block_size=None)
        rows = []
        frames = 0
        failure = None
        last_yaw = None
        unwrapped_yaw = 0.
        origin_xy = None
        last_command = [0.,0.,0.]
        for i in range(601):
            t = i*.02
            pos = robot.data.root_pos_w[0].cpu().numpy().copy()
            q = robot.data.root_quat_w[0].cpu().numpy()
            w,x,y,z = [float(v) for v in q]
            yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            if last_yaw is None: last_yaw=yaw; origin_xy=pos[:2].copy()
            unwrapped_yaw += math.atan2(math.sin(yaw-last_yaw), math.cos(yaw-last_yaw))
            last_yaw=yaw
            grav = robot.data.projected_gravity_b[0]
            force = sensor.data.net_forces_w[0,contact_ids].norm(dim=-1)
            foot_velocity = robot.data.body_lin_vel_w[0,foot_ids,:2].norm(dim=-1)
            joint_position = robot.data.joint_pos[0,ids]
            limits = robot.data.joint_pos_limits[0,ids]
            row = dict(time_s=t, position_world=pos.tolist(), yaw_deg=math.degrees(unwrapped_yaw),
                yaw_rate_world_rps=float(robot.data.root_ang_vel_w[0,2]),
                yaw_rate_body_rps=float(robot.data.root_ang_vel_b[0,2]),
                command_applied=last_command, next_command=cmd.command[0].cpu().tolist(),
                drift_m=float(np.linalg.norm(pos[:2]-origin_xy)),
                speed_mps=float(robot.data.root_lin_vel_w[0,:2].norm()),
                tilt_deg=math.degrees(math.acos(float((-grav[2]).clamp(-1,1)))),
                foot_force_n=force.cpu().tolist(), foot_speed_xy_mps=foot_velocity.cpu().tolist(),
                foot_position_world=robot.data.body_pos_w[0,foot_ids].cpu().tolist(),
                contact_slip_proxy_mps=float((foot_velocity*(force>1.)).max()),
                joint_position=joint_position.cpu().tolist(),
                hard_limit_excess_rad=float(torch.maximum(limits[:,0]-joint_position,joint_position-limits[:,1]).clamp_min(0).max()),
                termination=failure)
            rows.append(row)
            if i%2==0 or failure:
                if args.camera=='follow':
                    # Translate camera only; never rotate it with the robot or alter physics.
                    env.sim.set_camera_view(eye=(pos+np.array([2.6,-3.3,1.25])).tolist(),
                        target=(pos+np.array([0.,0.,.10])).tolist())
                    env.sim.render()
                rgb=env.render()
                if rgb.shape != (720,1280,3): raise RuntimeError('720p render shape mismatch')
                if float(rgb[100:,:,:].mean())<4.: raise RuntimeError('Dark/empty render; do not deliver')
                img=Image.fromarray(rgb)
                draw=ImageDraw.Draw(img)
                draw.rectangle((0,0,1280,100), fill=(9,19,30))
                phase='SETTLE' if t<2 else ('TURN' if t<10 else 'STOP')
                label='ORIGINAL WB5' if scenario.startswith('original') else f'WB5 TURN CANDIDATE ckpt={ckpt["iter"]}'
                draw.text((15,8), f'{label} | {scenario.split("_")[-1].upper()} | {phase} | t={t:.2f}s',font=font,fill='white')
                draw.text((15,36), f'yaw={row["yaw_deg"]:+.1f} deg | cmd={last_command[2]:+.2f} measured={row["yaw_rate_world_rps"]:+.3f} rad/s | drift={row["drift_m"]*100:.1f} cm',font=font,fill='white')
                draw.text((15,65), f'height={pos[2]:.3f}m | tilt={row["tilt_deg"]:.1f}deg | feet={force[0]:.0f}/{force[1]:.0f}N | NO TELEPORT / NO ACTION NOISE',font=small,fill=(200,222,240))
                if failure: draw.text((20,120),'FAILURE: '+failure,font=font,fill=(255,50,50))
                if args.camera=='follow':
                    draw.text((15,690),'CAMERA FOLLOWS POSITION ONLY | world drift is measured above; camera does NOT imply in-place turning',font=small,fill=(255,235,90),stroke_width=1,stroke_fill='black')
                writer.append_data(np.asarray(img))
                if i%100==0 or failure or i==600: img.save(folder/f'frame_{i:04d}.jpg')
                frames+=1
            if failure or i==600: break
            with torch.inference_mode(): action=policy(obs['policy'])
            if not torch.isfinite(action).all(): raise RuntimeError('Nonfinite action')
            last_command=cmd.command[0].cpu().tolist()
            try:
                obs,_,_,_,_=env.step(action)
            except TerminalStop:
                failure=' / '.join(n for n in env.termination_manager.active_terms if bool(env.termination_manager.get_term(n)[0]))
            if i%100==0:
                contract()
                print('AUDIT_PROGRESS',scenario,i,flush=True)
        writer.close(); writer=None
        save(folder/'telemetry.json',rows)
        active=[r for r in rows if 2.02<=r['time_s']<=10.02]
        settle=next((r for r in rows if r['time_s']>=2.),rows[-1])
        endturn=next((r for r in rows if r['time_s']>=10.02),rows[-1])
        stop=[r for r in rows if r['time_s']>=11.]
        initial_xy=np.asarray(settle['position_world'][:2])
        result=dict(completed=failure is None,failure=failure,scenario=scenario,frames=frames,
            duration_sim_s=rows[-1]['time_s'], requested_turn_deg=sign*math.degrees(.25*8),
            measured_turn_deg=endturn['yaw_deg']-settle['yaw_deg'],
            max_drift_during_turn_m=max(float(np.linalg.norm(np.asarray(r['position_world'][:2])-initial_xy)) for r in active) if active else None,
            active_yaw_rate_mae_rps=float(np.mean([abs(r['yaw_rate_world_rps']-r['command_applied'][2]) for r in active])) if active else None,
            stop_speed_mean_mps=float(np.mean([r['speed_mps'] for r in stop])) if stop else None,
            stop_yaw_rate_abs_mean_rps=float(np.mean([abs(r['yaw_rate_world_rps']) for r in stop])) if stop else None,
            min_root_height_m=min(r['position_world'][2] for r in rows),max_tilt_deg=max(r['tilt_deg'] for r in rows),
            max_hard_joint_limit_excess_rad=max(r['hard_limit_excess_rad'] for r in rows),
            foot_contact_transition_counts=[sum((rows[j]['foot_force_n'][k]>1)!=(rows[j-1]['foot_force_n'][k]>1) for j in range(1,len(rows))) for k in range(2)],
            contact_slip_proxy_mean_mps=float(np.mean([r['contact_slip_proxy_mps'] for r in active])) if active else None,
            runtime_contract_passed=True, production_replaced=False)
        save(folder/'result.json', result)
        all_results[scenario]=result
        save(out/'results.json',all_results)
        print('AUDIT_RESULT',json.dumps(result),flush=True)
    if hashlib.sha256(source.read_bytes()).hexdigest()!=original_hash: raise RuntimeError('Original changed')
    save(out/'status.json',dict(state='EVALUATION_COMPLETE',scenarios=list(all_results),production_replaced=False))
except BaseException:
    error=traceback.format_exc()
    save(out/'status.json',dict(state='FAILED',error=error))
    print(error,flush=True)
    raise
finally:
    if writer: writer.close()
    if env is not None: env.close()
    app.close()
