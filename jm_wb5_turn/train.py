"""Bounded WB5 actor-only PPO warm start. Never overwrites the WB5 source."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser()
p.add_argument('--out',type=Path,required=True)
p.add_argument('--num_envs',type=int,default=512)
p.add_argument('--max_iterations',type=int,default=1000)
p.add_argument('--seed',type=int,default=20261001)
p.add_argument('--preflight-only',action='store_true')
p.add_argument('--preflight-steps',type=int,default=700)
p.add_argument('--readiness',type=Path,help='Completed preflight run required before PPO')
AppLauncher.add_app_launcher_args(p)
args=p.parse_args(); args.headless=True
if not args.preflight_only:
    if not args.readiness: p.error('Training requires --readiness pointing to a preflight run')
    readiness=json.loads((args.readiness/'status.json').read_text())
    if readiness.get('state')!='PREFLIGHT_COMPLETE' or not readiness.get('contract_passed') or readiness.get('termination_events')!=0:
        p.error('Preflight runtime/initial-stability gate failed')
if args.out.exists(): p.error('Use a fresh output directory')
out=args.out.resolve(); out.mkdir(parents=True)
app=AppLauncher(args).app
env=None; started=time.time()


def save(name,data):
    (out/name).write_text(json.dumps(data,indent=2,default=str),encoding='utf-8')


try:
    import torch
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from isaaclab_tasks.manager_based.locomotion.velocity.config.h1.agents.rsl_rl_ppo_cfg import H1FlatPPORunnerCfg
    from rsl_rl.runners import OnPolicyRunner
    from jm_wb5_turn.env import TurnEnvCfg, ASSET
    from jm_xnavdp.h1_contract import load_constants, gains
    constants=load_constants(ROOT)
    source=ROOT/'artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt'
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    if source_hash!=constants['WALKER_SHA256']: raise RuntimeError('WB5 source hash mismatch')
    cfg=TurnEnvCfg(); cfg.scene.num_envs=args.num_envs; cfg.seed=args.seed
    cfg.sim.device=args.device
    # Preflight observes deterministic actor response, not noise-corrupted observations.
    # Training still retains the inherited observation corruption.
    if args.preflight_only: cfg.observations.policy.enable_corruption=False
    agent=H1FlatPPORunnerCfg(); agent.seed=args.seed; agent.max_iterations=args.max_iterations
    agent.save_interval=50; agent.logger='tensorboard'
    agent.obs_groups={'policy':['policy'],'critic':['policy']}
    # New optimizer/critic: conservative actor adaptation, explicitly not original PPO resume.
    agent.policy.init_noise_std=.2
    agent.algorithm.learning_rate=1e-4; agent.algorithm.schedule='fixed'
    agent.algorithm.entropy_coef=.005
    manifest=dict(created_unix=time.time(),arguments=vars(args),source_actor=str(source),source_sha256=source_hash,
        warm_start='WB5 actor only; new critic, optimizer and action std',full_ppo_resume=False,
        critic_only_initial_iterations=25,
        asset=str(ASSET),asset_sha256=hashlib.sha256(ASSET.read_bytes()).hexdigest(),
        command_groups=['stand','positive pure yaw','negative pure yaw','legacy forward+turn'],
        command_cycle_s={'stand_before':2,'turn':8,'stand_after':2},turn_range_rps=[.15,.30],
        rewards='WB5 posture_v1 retained; no new turning/drift reward',
        deployment_parity_verified=False,production_model_replaced=False,agent=agent.to_dict(),
        source_files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),ROOT/'jm_wb5_turn/env.py',ROOT/'jm_wb5_turn/commands.py',ROOT/'jm_wb5_turn/schedule.py',ROOT/'jm_xnavdp/h1_contract.py',ROOT/'jm_coop4_rl/h1_rack_env.py']})
    if not args.preflight_only:
        ready_manifest=json.loads((args.readiness/'manifest.json').read_text())
        for key in ('source_sha256','asset_sha256'):
            if manifest[key]!=ready_manifest[key]: raise RuntimeError('Readiness artifact differs: '+key)
        for filename in ['env.py','commands.py','schedule.py']:
            file=str(ROOT/'jm_wb5_turn'/filename)
            if ready_manifest['source_files'].get(file)!=manifest['source_files'][file]:
                raise RuntimeError('Environment changed after preflight: '+filename)
    save('manifest.json',manifest); save('environment_config.json',cfg.to_dict())
    env=ManagerBasedRLEnv(cfg=cfg)
    obs,_=env.reset(); robot=env.scene['robot']
    ids,names=robot.find_joints(list(constants['BODY_19']),preserve_order=True)
    action=env.action_manager.get_term('joint_pos')
    if names!=list(constants['BODY_19']) or action._joint_names!=names: raise RuntimeError('Joint order mismatch')
    if obs['policy'].shape[-1]!=69 or env.action_manager.total_action_dim!=19: raise RuntimeError('69/19 contract mismatch')
    expected=torch.tensor([gains(n) for n in names],device=env.device)
    readbacks=[]
    def check_contract(step):
        kp=robot.root_physx_view.get_dof_stiffnesses().to(env.device)[:,ids]
        kd=robot.root_physx_view.get_dof_dampings().to(env.device)[:,ids]
        force=robot.root_physx_view.get_dof_max_forces().to(env.device)[:,ids]
        measured=torch.stack([kp,kd,force],dim=-1)
        valid=bool(torch.allclose(measured,expected.expand_as(measured),rtol=1e-5,atol=1e-5))
        defaults=robot.data.default_joint_pos[:,ids]
        valid=valid and bool(torch.allclose(defaults,torch.tensor(constants['WALK_DEFAULT_19'],device=env.device).expand_as(defaults)))
        readbacks.append(dict(step=step,passed=valid,first_env=measured[0].tolist()))
        save('runtime_contract.json',dict(names=names,indices=ids,samples=readbacks,
            physics_dt=env.physics_dt,policy_dt=env.step_dt,all_joint_names=robot.joint_names))
        if not valid: raise RuntimeError('Runtime joint defaults/PD/effort mismatch')
    check_contract(0)
    actor=torch.jit.load(str(source),map_location=env.device).eval()

    def metrics():
        cmd=env.command_manager.get_term('base_velocity'); result={}
        for group,label in enumerate(['stand','left','right','walk']):
            mask=cmd.groups==group
            if not mask.any(): continue
            result[label]=dict(count=int(mask.sum()),
                yaw_error_mean_rps=float((robot.data.root_ang_vel_b[mask,2]-cmd.command[mask,2]).abs().mean()),
                yaw_rate_mean_rps=float(robot.data.root_ang_vel_b[mask,2].mean()),
                command_yaw_mean_rps=float(cmd.command[mask,2].mean()),
                speed_mean_mps=float(robot.data.root_lin_vel_b[mask,:2].norm(dim=1).mean()),
                root_height_mean_m=float((robot.data.root_pos_w[mask,2]-env.scene.env_origins[mask,2]).mean()))
            active=mask & (cmd.command[:,2].abs()>.01)
            result[label]['active_yaw_count']=int(active.sum())
            result[label]['active_yaw_error_mean_rps']=float((robot.data.root_ang_vel_b[active,2]-cmd.command[active,2]).abs().mean()) if active.any() else None
        return result

    if args.preflight_only:
        rows=[]; falls=torch.zeros(args.num_envs,device=env.device); terminations=0
        for i in range(args.preflight_steps):
            with torch.inference_mode(): a=actor(obs['policy'])
            if not torch.isfinite(a).all(): raise RuntimeError('Nonfinite actor')
            obs,reward,terminated,truncated,extra=env.step(a)
            falls+=terminated; terminations+=int(terminated.sum())
            if i in (0,19,39) or (i+1)%100==0: check_contract(i+1)
            if i%25==0:
                row=dict(step=i+1,time_s=(i+1)*env.step_dt,metrics=metrics(),termination_events=terminations)
                rows.append(row); save('preflight_telemetry.json',rows)
                print('TURN_PREFLIGHT',json.dumps(row),flush=True)
        save('status.json',dict(state='PREFLIGHT_COMPLETE',contract_passed=True,
            environments_terminated_fraction=float((falls>0).float().mean()),termination_events=terminations,
            locomotion_performance_passed=False,scope='runtime readiness; not turning success'))
    else:
        class StatusRunner(OnPolicyRunner):
            def log(self,locs,width=80,pad=35):
                super().log(locs,width,pad)
                losses={k:float(v) for k,v in locs['loss_dict'].items()}
                if not all(math.isfinite(v) for v in losses.values()): raise RuntimeError('Nonfinite loss')
                row=dict(state='TRAINING',iteration=int(locs['it'])+1,target_iterations=args.max_iterations,
                    learning_phase='CRITIC_WARMUP' if int(locs['it'])<25 else 'ACTOR_CRITIC_PPO',
                    updated_unix=time.time(),wall_s=time.time()-started,loss=losses,metrics=metrics(),
                    collection_s=float(locs['collection_time']),learning_s=float(locs['learn_time']),
                    mean_reward=float(sum(locs['rewbuffer'])/len(locs['rewbuffer'])) if locs['rewbuffer'] else None)
                save('status.json',row)
                with (out/'history.jsonl').open('a',encoding='utf-8') as stream: stream.write(json.dumps(row)+'\n')
                if row['iteration']%50==0: check_contract(row['iteration']*24)
                if row['iteration']==25:
                    for param in self.alg.policy.actor.parameters(): param.requires_grad_(True)
                    self.alg.policy.std.requires_grad_(True)
                    print('WB5_ACTOR_UPDATES_ENABLED_AFTER_CRITIC_WARMUP',flush=True)
        wrapped=RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions)
        runner=StatusRunner(wrapped,agent.to_dict(),log_dir=str(out),device=args.device)
        runner.alg.policy.actor.load_state_dict(actor.state_dict(),strict=True)
        for param in runner.alg.policy.actor.parameters(): param.requires_grad_(False)
        runner.alg.policy.std.requires_grad_(False)
        with torch.inference_mode():
            reference=actor(obs['policy']); loaded=runner.alg.policy.actor(obs['policy'])
        error=float((reference-loaded).abs().max())
        if error>1e-6: raise RuntimeError('Actor warm start mismatch')
        save('warm_start.json',dict(actor_max_output_difference=error,critic_restored=False,optimizer_restored=False))
        # RSL-RL normally initializes this inside learn(), but this snapshot precedes it.
        runner.logger_type=agent.logger
        runner.save(str(out/'model_initial_actor_warmstart.pt'))
        runner.learn(num_learning_iterations=args.max_iterations,init_at_random_ep_len=False)
        trained=copy.deepcopy(runner.alg.policy.actor).cpu().eval()
        torch.jit.save(torch.jit.script(trained),str(out/'policy_wb5_turn_candidate.pt'))
        save('status.json',dict(state='TRAINING_COMPLETE',iterations=args.max_iterations,
            wall_s=time.time()-started,production_model_replaced=False,turn_success_validated=False))
    if hashlib.sha256(source.read_bytes()).hexdigest()!=source_hash: raise RuntimeError('Source model changed')
except BaseException:
    error=traceback.format_exc(); print(error,flush=True)
    save('status.json',dict(state='FAILED',error=error)); (out/'exception.txt').write_text(error,encoding='utf-8')
    raise
finally:
    if env is not None: env.close()
    app.close()
