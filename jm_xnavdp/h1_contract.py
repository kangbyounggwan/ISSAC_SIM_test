"""Dependency-light WB5 inference contract; no simulator state mutation."""
import ast
import json
from pathlib import Path
import numpy as np

PHYSICS_DT = .005
POLICY_DECIMATION = 4
ACTION_SCALE = .5


def load_constants(root):
    result = {}
    for node in ast.parse((Path(root)/'jm_coop4_rl/h1_rack_env.py').read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in {'BODY_19', 'WALK_DEFAULT_19', 'WALKER_SHA256'}:
                result[node.targets[0].id] = ast.literal_eval(node.value)
    if len(result['BODY_19']) != 19 or len(set(result['BODY_19'])) != 19:
        raise ValueError('Invalid body joint order')
    return result


def gains(name):
    if 'ankle' in name: return 20., 4., 100.
    if 'hip_yaw' in name or 'hip_roll' in name: return 150., 5., 300.
    if any(s in name for s in ('hip_pitch', 'knee', 'torso')): return 200., 5., 300.
    if 'shoulder' in name or 'elbow' in name: return 40., 10., 300.
    return 30., 1., 30.


def observation(lin, ang, gravity, command, q, qd, previous, defaults):
    obs = np.r_[lin, ang, gravity, command, q-defaults, qd, previous].astype(np.float32)
    if obs.shape != (69,) or not np.isfinite(obs).all():
        raise ValueError('Invalid WB5 observation')
    return obs


class RuntimeAudit:
    """Read-only body-drive/clock audit. NOT certification of training parity."""
    def __init__(self, path, names, ids):
        self.path = Path(path)
        self.ids = list(ids)
        self.expected = np.asarray([gains(n) for n in names])
        self.report = dict(scope='body19 PD, effort and physics dt only',
                           training_deployment_parity_verified=False,
                           body_names=list(names), body_indices=self.ids,
                           expected_kp_kd_effort=self.expected.tolist(), samples=[])

    def check(self, robot, step, elapsed):
        kp, kd = robot.get_articulation_controller().get_gains()
        efforts = np.asarray(robot._articulation_view.get_max_efforts()).reshape(-1)
        measured = np.column_stack([np.asarray(kp).reshape(-1)[self.ids],
                                    np.asarray(kd).reshape(-1)[self.ids], efforts[self.ids]])
        clock_error = abs(elapsed-step*PHYSICS_DT)
        passed = bool(np.allclose(measured, self.expected, rtol=1e-5, atol=1e-5) and clock_error < 1e-5)
        self.report['samples'].append(dict(step=step, elapsed_s=elapsed,
            kp_kd_effort=measured.tolist(), clock_error_s=clock_error, passed=passed))
        self.report['passed'] = all(x['passed'] for x in self.report['samples'])
        self.path.write_text(json.dumps(self.report, indent=2), encoding='utf-8')
        if not passed: raise RuntimeError('RUNTIME_CONTRACT_MISMATCH; see runtime_contract.json')
