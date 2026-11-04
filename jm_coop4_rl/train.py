"""Fresh high-level rack navigation PPO; retain verified H1 locomotion weights."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import os
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from isaaclab.app import AppLauncher

p=argparse.ArgumentParser()
p.add_argument("--num_envs",type=int,default=512)
p.add_argument("--max_iterations",type=int,default=1000)
p.add_argument("--out",required=True)
p.add_argument("--seed",type=int,default=20260909)
p.add_argument("--resume",default="")
p.add_argument("--smoke",action="store_true")
AppLauncher.add_app_launcher_args(p)
args=p.parse_args()
args.headless=True
out=Path(args.out).resolve()
out.mkdir(parents=True,exist_ok=True)
app=AppLauncher(args).app

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.manager_based.locomotion.velocity.config.h1.agents.rsl_rl_ppo_cfg import H1FlatPPORunnerCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner
from h1_rack_env import H1RackEnvCfg, WALKER_PATH

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

class LocalStatusRunner(OnPolicyRunner):
    def log(self,locs,width=80,pad=35):
        super().log(locs,width,pad)
        vals = {}
        for item in locs.get("ep_infos",[]):
            for k,v in item.items():
                try:
                    vals.setdefault(k,[]).append(float(torch.as_tensor(v).float().mean().item()))
                except (ValueError,TypeError):
                    pass
        data={"state":"TRAINING","iteration":int(locs["it"])+1,
              "target_iterations":args.max_iterations,"num_envs":args.num_envs,
              "collection_s":float(locs["collection_time"]),"learning_s":float(locs["learn_time"]),
              "total_wall_s":time.time()-started,"updated_unix":time.time(),
              "metrics":{k:sum(v)/len(v) for k,v in vals.items()},
              "loss":{k:float(v) for k,v in locs["loss_dict"].items()}}
        data["task"]=env._jm_rack_action.metrics()
        reward = list(locs.get("rewbuffer",[]))
        if reward: data["mean_reward"]=sum(reward)/len(reward)
        if not all(math.isfinite(v) for v in data["loss"].values()):
            raise RuntimeError("NONFINITE_PPO_LOSS")
        (out/"status.json").write_text(json.dumps(data,indent=2),encoding="utf-8")
        with (out/"training_history.jsonl").open("a",encoding="utf-8") as f:
            f.write(json.dumps(data)+"\n")

started=time.time()
env=None
try:
    cfg=H1RackEnvCfg()
    cfg.scene.num_envs=args.num_envs
    cfg.seed=args.seed
    agent=H1FlatPPORunnerCfg()
    agent.max_iterations=args.max_iterations
    agent.seed=args.seed
    agent.save_interval=50
    agent.experiment_name="jm_coop4_h1_rack_nav"
    agent.logger="tensorboard"
    agent.obs_groups={"policy":["policy"],"critic":["policy"]}
    agent.policy.init_noise_std=.4
    agent.algorithm.entropy_coef=.005
    manifest={"created_unix":started,"scope":"fresh rack-navigation PPO over frozen H1 walker",
              "not_claimed":["Atlas training","four-robot factory success","grasp"],
              "smoke":args.smoke,"num_envs":args.num_envs,"max_iterations":args.max_iterations,
              "seed":args.seed,"resume":args.resume or None,
              "source_sha256":{str(f):sha(f) for f in [Path(__file__),ROOT/'h1_rack_env.py']},
              "robot_usd":cfg.scene.robot.spawn.usd_path,
              "robot_sha256":sha(cfg.scene.robot.spawn.usd_path),
              "frozen_walker":WALKER_PATH,"frozen_walker_sha256":sha(WALKER_PATH),
              "agent":agent.to_dict()}
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2,default=str),encoding="utf-8")
    (out/"environment_config.txt").write_text(repr(cfg.to_dict()),encoding="utf-8")
    env=ManagerBasedRLEnv(cfg=cfg)
    obs,_=env.reset()
    print("RACK_RL_ENV_READY", {k:list(v.shape) for k,v in obs.items()},flush=True)
    wrapped=RslRlVecEnvWrapper(env,clip_actions=agent.clip_actions)
    runner=LocalStatusRunner(wrapped,agent.to_dict(),log_dir=str(out),device=args.device)
    if args.resume:
        runner.load(args.resume,load_optimizer=True)
    runner.learn(num_learning_iterations=args.max_iterations,init_at_random_ep_len=True)
    actor=copy.deepcopy(runner.alg.policy.actor).cpu().eval()
    torch.jit.save(torch.jit.script(actor),str(out/"policy_rack_nav.pt"))
    finished={"state":"SMOKE_COMPLETE" if args.smoke else "TRAINING_COMPLETE",
              "iterations_requested":args.max_iterations,"wall_s":time.time()-started,
              "policy":str(out/"policy_rack_nav.pt"),"task_success_validated":False}
    (out/"completion.json").write_text(json.dumps(finished,indent=2),encoding="utf-8")
    (out/"status.json").write_text(json.dumps(finished,indent=2),encoding="utf-8")
    print("RACK_RL_COMPLETE",json.dumps(finished),flush=True)
except BaseException:
    trace=traceback.format_exc()
    print(trace,flush=True)
    (out/"exception.txt").write_text(trace,encoding="utf-8")
    (out/"status.json").write_text(json.dumps({"state":"FAILED","detail":trace},indent=2),encoding="utf-8")
finally:
    if env is not None: env.close()
    app.close()
