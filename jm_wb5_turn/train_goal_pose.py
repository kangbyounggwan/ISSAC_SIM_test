"""Bounded, separate 76-observation goal-turn experiment. No production replacement."""
import argparse
import copy
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
from isaaclab.app import AppLauncher

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--num_envs', type=int, default=512)
p.add_argument('--max_iterations', type=int, default=1000)
p.add_argument('--seed', type=int, default=20261002)
p.add_argument('--preflight-only', action='store_true')
p.add_argument('--readiness', type=Path)
AppLauncher.add_app_launcher_args(p)
args = p.parse_args()
args.headless = True
if args.out.exists(): p.error('Use a fresh output directory')
if args.num_envs < 4 or args.num_envs % 4: p.error('num_envs must be a positive multiple of four')
if not args.preflight_only and not args.readiness: p.error('Training requires --readiness')
if not args.preflight_only and args.max_iterations < 1: p.error('iterations must be positive')
out = args.out.resolve()
out.mkdir(parents=True)
started = time.time()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, value):
    temporary = out / (name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, default=str), encoding='utf-8')
    temporary.replace(out / name)


def status(state, **data):
    save('status.json', dict(state=state, pid=os.getpid(), updated_unix=time.time(),
        started_unix=started, production_model_replaced=False, **data))


status('STARTING')
app = None
env = None
try:
    app = AppLauncher(args).app
    import torch
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from isaaclab_tasks.manager_based.locomotion.velocity.config.h1.agents.rsl_rl_ppo_cfg import H1FlatPPORunnerCfg
    from rsl_rl.runners import OnPolicyRunner
    from jm_wb5_turn.goal_env import GoalTurnEnvCfg, GoalTurnEnv
    from jm_wb5_turn.goal_math import expand_actor_state
    from jm_wb5_turn.env import ASSET
    from jm_xnavdp.h1_contract import load_constants, gains

    constants = load_constants(ROOT)
    source = ROOT / 'artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt'
    source_hash = digest(source)
    if source_hash != constants['WALKER_SHA256']: raise RuntimeError('WB5 source hash mismatch')
    cfg = GoalTurnEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    cfg.sim.device = args.device
    if args.preflight_only: cfg.observations.policy.enable_corruption = False
    if cfg.sim.dt * cfg.decimation != .02: raise RuntimeError('Integer phase schedule requires 50Hz control')
    agent = H1FlatPPORunnerCfg()
    agent.seed = args.seed
    agent.max_iterations = args.max_iterations
    agent.save_interval = 50
    agent.logger = 'tensorboard'
    agent.obs_groups = {'policy':['policy'], 'critic':['policy']}
    agent.policy.init_noise_std = .2
    agent.algorithm.learning_rate = 1e-4
    agent.algorithm.schedule = 'fixed'
    agent.algorithm.entropy_coef = .005
    files = ['jm_wb5_turn/train_goal_pose.py', 'jm_wb5_turn/goal_env.py',
        'jm_wb5_turn/goal_commands.py', 'jm_wb5_turn/goal_math.py', 'jm_wb5_turn/env.py',
        'jm_wb5_turn/commands.py', 'jm_wb5_turn/schedule.py',
        'jm_xnavdp/h1_contract.py', 'jm_coop4_rl/h1_rack_env.py']
    manifest = dict(experiment='goal_pose_v1', arguments=vars(args), source_actor=str(source),
        source_sha256=source_hash, asset=str(ASSET), asset_sha256=digest(ASSET),
        source_files={name:digest(ROOT/name) for name in files},
        observation_contract='WB5 69 unchanged + 7 goal features = 76; 19 joint actions unchanged',
        warm_start='WB5 actor; appended input weights zero; new critic/optimizer/std',
        full_ppo_resume=False, critic_warmup_iterations=25,
        groups=['stand','left pose turn','right pose turn','legacy walk'],
        turn_targets_deg=[45,90,180], phases_s=[2,12,4], episode_s=18,
        goal_feedback_limits={'vx_vy_mps':.15,'yaw_rps':.6},
        success_gate={'position_m':.10,'yaw_deg':5,'speed_mps':.05,'yaw_rate_rps':.05,
            'tilt_deg':10,'min_height_m':.80,'both_feet_contact_n':5,'hold_s':.75,
            'max_episode_drift_m':.20,'scope':'provisional flat-ground benchmark, not rack certification'},
        agent=agent.to_dict(), not_paper_reproduction=True, production_model_replaced=False)
    if not args.preflight_only:
        ready = json.loads((args.readiness/'status.json').read_text())
        previous = json.loads((args.readiness/'manifest.json').read_text())
        if ready.get('state') != 'PREFLIGHT_COMPLETE' or not ready.get('runtime_ready'):
            raise RuntimeError('Readiness gate failed')
        for key in ('source_sha256','asset_sha256','source_files','observation_contract'):
            if manifest[key] != previous[key]: raise RuntimeError('Changed since preflight: '+key)
    save('manifest.json', manifest)
    save('environment_config.json', cfg.to_dict())
    env = GoalTurnEnv(cfg=cfg)
    obs, _ = env.reset()
    robot = env.scene['robot']
    cmd = env.command_manager.get_term('base_velocity')
    ids, names = robot.find_joints(list(constants['BODY_19']), preserve_order=True)
    if names != list(constants['BODY_19']) or env.action_manager.get_term('joint_pos')._joint_names != names:
        raise RuntimeError('Joint ordering mismatch')
    if obs['policy'].shape[-1] != 76 or env.action_manager.total_action_dim != 19:
        raise RuntimeError('76/19 contract mismatch')
    if env.observation_manager.active_terms['policy'][-1] != 'goal_pose':
        raise RuntimeError('Goal features are not appended after WB5 observations')
    expected = torch.tensor([gains(name) for name in names],device=env.device)
    default = torch.tensor(constants['WALK_DEFAULT_19'],device=env.device)
    contracts = []

    def contract(step):
        actual = torch.stack((robot.root_physx_view.get_dof_stiffnesses().to(env.device)[:,ids],
            robot.root_physx_view.get_dof_dampings().to(env.device)[:,ids],
            robot.root_physx_view.get_dof_max_forces().to(env.device)[:,ids]),dim=-1)
        valid = torch.allclose(actual,expected.expand_as(actual),atol=1e-5,rtol=1e-5)
        valid = valid and torch.allclose(robot.data.default_joint_pos[:,ids],default.expand(args.num_envs,-1))
        contracts.append(dict(step=step,passed=bool(valid)))
        save('runtime_contract.json',dict(names=names,samples=contracts,physics_dt=env.physics_dt,
            policy_dt=env.step_dt,observation_terms=env.observation_manager.active_terms['policy']))
        if not valid: raise RuntimeError('PD/effort/default mismatch')

    contract(0)
    source_actor = torch.jit.load(str(source),map_location=env.device).eval()
    if args.preflight_only:
        early_failures = 0
        total_failures = 0
        rows = []
        for step in range(900):
            with torch.inference_mode(): action = source_actor(obs['policy'][:,:69])
            if not torch.isfinite(action).all(): raise RuntimeError('Nonfinite action')
            obs,reward,terminated,truncated,_ = env.step(action)
            if not torch.isfinite(obs['policy']).all() or not torch.isfinite(reward).all():
                raise RuntimeError('Nonfinite simulation data')
            total_failures += int(terminated.sum())
            if step < 100: early_failures += int(terminated.sum())
            if (step+1)%100 == 0:
                contract(step+1)
                row=dict(step=step+1,metrics=cmd.summary(),termination_events=total_failures)
                rows.append(row)
                save('preflight_telemetry.json',rows)
                status('PREFLIGHT',step=step+1,termination_events=total_failures)
                print('GOAL_PREFLIGHT',json.dumps(row),flush=True)
        status('PREFLIGHT_COMPLETE',runtime_ready=early_failures==0,contract_passed=True,
            initial_2s_termination_events=early_failures,termination_events=total_failures,
            metrics=cmd.summary(),turn_performance_passed=False,
            note='Initial WB5 failures during goals are recorded, not claimed as successful turns')
        save('terminal_episode_audit.json',{k:v.cpu().tolist() for k,v in cmd.last_terminal.items()})
    else:
        class GoalRunner(OnPolicyRunner):
            def log(self,locs,width=80,pad=35):
                super().log(locs,width,pad)
                losses={k:float(v) for k,v in locs['loss_dict'].items()}
                if not all(math.isfinite(v) for v in losses.values()): raise RuntimeError('Nonfinite loss')
                iteration=int(locs['it'])+1
                row=dict(iteration=iteration,target_iterations=args.max_iterations,
                    phase='CRITIC_WARMUP' if iteration<=25 else 'ACTOR_CRITIC_PPO',
                    loss=losses,metrics=cmd.summary(),wall_s=time.time()-started,
                    collection_s=float(locs['collection_time']),learning_s=float(locs['learn_time']),
                    total_transitions=iteration*args.num_envs*agent.num_steps_per_env)
                status('TRAINING',**row)
                with (out/'history.jsonl').open('a',encoding='utf-8') as f:
                    f.write(json.dumps(dict(updated_unix=time.time(),**row))+'\n')
                if iteration%50==0: contract(iteration*agent.num_steps_per_env)
                if iteration==25:
                    for param in self.alg.policy.actor.parameters(): param.requires_grad_(True)
                    self.alg.policy.std.requires_grad_(True)
                    print('GOAL_ACTOR_UPDATES_ENABLED',flush=True)
        runner=GoalRunner(RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions),
            agent.to_dict(),log_dir=str(out),device=args.device)
        runner.alg.policy.actor.load_state_dict(expand_actor_state(
            source_actor.state_dict(),runner.alg.policy.actor.state_dict()),strict=True)
        with torch.inference_mode():
            original=source_actor(obs['policy'][:,:69])
            adapted=runner.alg.policy.actor(obs['policy'])
            error=float((original-adapted).abs().max())
        if error>1e-6: raise RuntimeError('Expanded actor output mismatch')
        save('warm_start.json',dict(max_output_difference=error,zero_appended_weights=True,
            critic_restored=False,optimizer_restored=False))
        for param in runner.alg.policy.actor.parameters(): param.requires_grad_(False)
        runner.alg.policy.std.requires_grad_(False)
        runner.logger_type=agent.logger
        runner.save(str(out/'model_initial_actor_warmstart.pt'))
        runner.learn(num_learning_iterations=args.max_iterations,init_at_random_ep_len=False)
        trained=copy.deepcopy(runner.alg.policy.actor).cpu().eval()
        torch.jit.save(torch.jit.script(trained),str(out/'policy_goal_pose_candidate.pt'))
        status('TRAINING_COMPLETE',iterations=args.max_iterations,wall_s=time.time()-started,
            metrics=cmd.summary(),turn_success_validated=False)
    if digest(source)!=source_hash: raise RuntimeError('Original WB5 was modified')
except BaseException:
    error=traceback.format_exc()
    status('FAILED',error=error)
    print(error,flush=True)
    raise
finally:
    if env is not None: env.close()
    if app is not None: app.close()
