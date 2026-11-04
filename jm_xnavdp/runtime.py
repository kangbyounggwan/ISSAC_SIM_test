"""Pinned upstream inference with tensor-only loading and complete-key audit."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


class Runtime:
    def __init__(self, intrinsic, device='cuda:0', seed=42):
        import torch
        lock = json.loads((ROOT / 'jm_xnavdp/upstream_lock.json').read_text())
        repo = ROOT / lock['code_path']
        commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain', '--untracked-files=no'], text=True)
        if commit != lock['code_commit'] or dirty.strip():
            raise RuntimeError('Upstream source differs from pinned clean commit')
        checkpoint = ROOT / lock['model_path']
        if checkpoint.stat().st_size != lock['model_bytes'] or sha(checkpoint) != lock['model_sha256']:
            raise RuntimeError('Checkpoint checksum mismatch')
        sys.path.insert(0, str(repo / 'baselines/x-navdp/eval/src'))
        from policy_agent import NavDP_Agent
        from policy_network_embodiment import NavDP_Policy_Embodiment
        torch.manual_seed(seed)
        np.random.seed(seed)
        torch.set_num_threads(4)
        self.device = device
        model = NavDP_Policy_Embodiment(image_size=224, memory_size=8, predict_size=24,
                                        temporal_depth=16, heads=8, token_dim=384, device=device)
        state = torch.load(checkpoint, map_location='cpu', weights_only=True)
        expected = model.state_dict()
        missing = sorted(set(expected) - set(state))
        mismatched = [k for k in expected if k in state and expected[k].shape != state[k].shape]
        if missing or mismatched:
            raise RuntimeError(f'Incomplete inference weights: missing={missing}, mismatched={mismatched}')
        # Unused training/other-goal entries are audited; inference loads strictly.
        extra = sorted(set(state) - set(expected))
        model.load_state_dict({k: state[k] for k in expected}, strict=True)
        del state
        model.to(device).eval()
        # Reuse upstream preprocessing/history/guidance and trajectory selection.
        # Bypass its permissive strict=False loader and positional device argument.
        self.agent = NavDP_Agent.__new__(NavDP_Agent)
        self.agent.image_intrinsic = np.asarray(intrinsic)
        self.agent.device = device
        self.agent.predict_size = 24
        self.agent.image_size = 224
        self.agent.memory_size = 8
        self.agent.embodiment = 1
        self.agent.is_real = False
        self.agent.navi_former = model
        self.agent.current_scene_name = None
        self.agent._occ_cache = {}
        self.agent.save_counter = np.zeros(1000, dtype=np.int32)
        self.agent.save_plan_dir = None
        self.agent.reset(1)
        # Omit upstream debug plotting only; does not change model/selection.
        self.agent.project_trajectory_2d = lambda *a, **k: np.zeros((1, 1, 3), dtype=np.uint8)
        self.agent._trajectory_debug_panel = lambda *a, **k: np.zeros((1, 1, 3), dtype=np.uint8)
        self.audit = {**lock, 'device': device, 'torch_version': torch.__version__,
                      'gpu': torch.cuda.get_device_name() if device.startswith('cuda') else None,
                      'seed': seed, 'inference_tensor_count': len(expected),
                      'missing_keys': missing, 'shape_mismatches': mismatched,
                      'unused_checkpoint_keys': extra, 'strict_inference_load': True,
                      'source_modified': False, 'weights_only': True}

    def infer(self, rgb, depth_m, goal_local, base_world, yaw, samples=None, commit_guidance=True, guidance_scale=.5):
        import torch
        rgb = np.asarray(rgb)
        depth = np.asarray(depth_m, dtype=np.float32).copy()
        goal = np.asarray(goal_local, dtype=np.float32)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[-1] != 3 or depth.shape != rgb.shape[:2]:
            raise ValueError('Expected uint8 RGB and matching distance-to-image-plane metres')
        if goal.shape != (3,) or not np.isfinite(goal).all() or np.linalg.norm(goal) < 1e-6:
            raise ValueError('Invalid or zero-distance goal; handle arrival outside the planner')
        valid = np.isfinite(depth) & (depth >= .1) & (depth <= 5.)
        if valid.mean() < .01:
            raise ValueError('Insufficient valid depth')
        depth[~valid] = 0.
        if self.device.startswith('cuda'):
            torch.cuda.synchronize()
            torch.cuda.reset_peak_memory_stats()
        start = time.perf_counter()
        # Quaternion convention: scipy xyzw. Level body yaw for static captures.
        q = np.array([[0., 0., np.sin(yaw / 2), np.cos(yaw / 2)]])
        if samples is None:
            trajectory, candidates, values, _ = self.agent.step_pointgoal_with_guidance(
                goal[None], rgb[None], depth[None, :, :, None], np.asarray(base_world)[None], q)
        else:
            if samples not in (8, 16, 32):
                raise ValueError('Use a predefined candidate count')
            a = self.agent
            images = a._update_and_sample_history(a.process_image(rgb[None]), num_samples=8)
            depths = a.process_depth(depth[None, :, :, None])
            pos = np.asarray(base_world)[None]
            a._update_stuck_state(pos)
            prev, valid_len, _ = a.get_guidance_trajectory(pos, q)
            if not 0 <= guidance_scale <= 1:
                raise ValueError('Invalid guidance scale')
            factors = np.full((1, samples), guidance_scale, dtype=np.float32)
            factors[:, -2:] = guidance_scale * .1
            factors[a.is_stuck] = 0.
            candidates, values, best, _ = a.navi_former.predict_pointgoal_action_with_guidance(
                a.process_pointgoal(goal[None]), images, depths, sample_num=samples,
                valid_segment_len=valid_len, prev_action=prev, start_index=0, end_index=23,
                guidance_factor=factors, guidance_step=5, prefix_attention_schedule='exp', embodiment=1)
            trajectory = best[:, 0]
            # No upstream random stuck-path replacement. Downstream hard gates choose
            # among all candidates; the raw Q-best is retained for audit only.
            if commit_guidance:
                a._update_last_trajectory_state(trajectory, pos, q)
        if self.device.startswith('cuda'):
            torch.cuda.synchronize()
        if trajectory.shape != (1, 24, 3) or not all(np.isfinite(x).all() for x in (trajectory, candidates, values)):
            raise RuntimeError('Non-finite or malformed model output')
        return {'trajectory_local_m': trajectory[0].tolist(), 'candidates_local_m': candidates[0].tolist(),
                'q_values': values[0].tolist(), 'inference_wall_s': time.perf_counter() - start,
                'peak_allocated_gpu_bytes': torch.cuda.max_memory_allocated() if self.device.startswith('cuda') else None,
                'depth_valid_fraction': float(valid.mean()), 'waypoint_dt_s': .25,
                'frame': 'body x forward, y left; z ignored by planar follower',
                'stuck': bool(self.agent.is_stuck[0]), 'sample_count': samples or 8,
                'guidance_scale': guidance_scale,
                'selection_mode': 'raw_q_best_for_external_gating' if samples else 'upstream_default'}
