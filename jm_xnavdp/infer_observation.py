"""Actual checkpoint inference on saved simulator RGB-D; no robot commands."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from jm_xnavdp.adapter import world_path, propose_command
from jm_xnavdp.runtime import Runtime, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observation', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--seed', type=int, default=42)
    args = p.parse_args()
    if args.out.exists():
        raise RuntimeError('Use a fresh output directory')
    args.out.mkdir(parents=True)
    meta = json.loads((args.observation / 'observation.json').read_text())
    data = np.load(args.observation / 'observation.npz', allow_pickle=False)
    runtime = Runtime(meta['intrinsic'], seed=args.seed)
    (args.out / 'load_audit.json').write_text(json.dumps(runtime.audit, indent=2), encoding='utf-8')
    results = []
    # Two cold-history repeats at the same pose for warm-up timing, NOT a moving video/history.
    for repeat in range(2):
        runtime.agent.reset(1)
        import torch
        torch.manual_seed(args.seed)
        np.random.seed(args.seed)
        result = runtime.infer(data['rgb'], data['depth_m'], meta['goal_local_m'], meta['base_world'], meta['yaw_rad'])
        result['repeat'] = repeat
        result['world_path_xy'] = world_path(result['trajectory_local_m'], meta['base_world'][:2], meta['yaw_rad']).tolist()
        # Saved observation is not live; command execution must stay blocked.
        result['command_gate'] = propose_command(result['trajectory_local_m'], observation_age_s=1e9)
        results.append(result)
    record = {'status': 'MODEL_INFERENCE_PASS_EXECUTION_BLOCKED', 'scope': 'static RGB-D, cold history',
              'source_observation': str(args.observation.resolve()),
              'source_sha256': {name: sha(args.observation / name) for name in ['observation.json', 'observation.npz']},
              'robot_commands_sent': 0, 'new_training': False, 'h1_transfer_validated': False,
              'results': results, 'repeat_max_abs_difference_m': float(np.max(np.abs(
                  np.asarray(results[0]['trajectory_local_m']) - np.asarray(results[1]['trajectory_local_m']))))}
    (args.out / 'result.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    render(data['rgb'], data['depth_m'], meta, results[-1], args.out / 'inference_720p.png')
    print(json.dumps({'status': record['status'], 'seconds': [r['inference_wall_s'] for r in results],
                      'command_gate': results[-1]['command_gate']}, indent=2))


def render(rgb, depth, meta, result, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor='#101820')
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=.22)
    ax = fig.add_subplot(gs[0, 0]); ax.imshow(rgb); ax.axis('off'); ax.set_title('Actual simulator RGB observation', color='white')
    ax = fig.add_subplot(gs[1, 0]); ax.imshow(depth, vmin=0, vmax=5, cmap='turbo'); ax.axis('off'); ax.set_title('Measured image-plane depth | 0-5m', color='white')
    ax = fig.add_subplot(gs[:, 1]); ax.set_facecolor('#eef3f5')
    for candidate in result['candidates_local_m']:
        c = np.asarray(candidate); ax.plot(-c[:, 1], c[:, 0], color='#94a3b8', alpha=.65)
    selected = np.asarray(result['trajectory_local_m'])
    ax.plot(-selected[:, 1], selected[:, 0], color='#006ac3', linewidth=2.5, label='Selected model trajectory')
    goal = meta['goal_local_m']; ax.scatter(-goal[1], goal[0], marker='*', s=180, color='#e57f13', label='Goal')
    ax.scatter(0, 0, marker='^', color='black', label='Body origin')
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.3)
    ax.set_xlabel('right [m]', color='white'); ax.set_ylabel('forward [m]', color='white'); ax.tick_params(colors='white')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title('X-NavDP | G1 embedding on H1 input', color='white', fontsize=11)
    fig.suptitle(f"{meta['world_key'].upper()} | X-NavDP checkpoint inference | NOT robot motion", color='white', fontsize=15)
    fig.text(.5, .025, f"Static observation; cold history | {result['inference_wall_s']:.2f}s/inference | EXECUTION BLOCKED: safety/stop/transfer unverified",
             color='#ffcc77', ha='center', fontsize=9)
    fig.subplots_adjust(left=.025, right=.98, top=.9, bottom=.12)
    fig.savefig(path, dpi=100, facecolor=fig.get_facecolor()); plt.close(fig)


if __name__ == '__main__':
    main()
