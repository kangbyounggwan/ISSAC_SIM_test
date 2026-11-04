"""Frame/command boundary. A proposed command is NOT permission to execute."""
import math
import numpy as np


def local_goal(base_xy, yaw, goal_xy):
    delta = np.asarray(goal_xy, dtype=float) - np.asarray(base_xy, dtype=float)
    if delta.shape != (2,) or not np.isfinite(delta).all() or not math.isfinite(yaw):
        raise ValueError('Invalid goal/pose')
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1], 0.], dtype=np.float32)


def world_path(path, base_xy, yaw):
    path = np.asarray(path, dtype=float)
    if path.shape != (24, 3) or not np.isfinite(path).all():
        raise ValueError('Expected finite 24x3 local trajectory')
    c, s = math.cos(yaw), math.sin(yaw)
    return path[:, :2] @ np.array([[c, s], [-s, c]]) + np.asarray(base_xy)


def propose_command(path, observation_age_s, clearance_verified=False,
                    stopping_verified=False, h1_transfer_verified=False):
    """Conservative forward-only preview. Fail closed; no goal/tolerance relaxation.

    All three certification flags are false in the initial deployment. Bounds
    match the existing KETI trial, not a new locomotion training range.
    """
    path = np.asarray(path, dtype=float)
    reasons = []
    if path.shape != (24, 3) or not np.isfinite(path).all():
        return {'execute': False, 'command': [0., 0., 0.], 'reasons': ['INVALID_TRAJECTORY']}
    if not math.isfinite(observation_age_s) or observation_age_s < 0 or observation_age_s > .5:
        reasons.append('STALE_OBSERVATION')
    if np.any(path[:8, 0] < -.02) or np.any(np.diff(np.r_[0., path[:8, 0]]) < -.02):
        reasons.append('UNVALIDATED_REVERSE')
    if not clearance_verified:
        reasons.append('SENSOR_CORRIDOR_NOT_VERIFIED')
    if not stopping_verified:
        reasons.append('H1_STOP_NOT_VERIFIED')
    if not h1_transfer_verified:
        reasons.append('G1_TO_H1_TRANSFER_NOT_VERIFIED')
    distances = np.linalg.norm(path[:, :2], axis=1)
    ids = np.flatnonzero(distances >= .4)
    target = path[int(ids[0]) if len(ids) else -1, :2]
    angle = math.atan2(target[1], target[0])
    if abs(angle) > math.radians(60):
        reasons.append('HEADING_OUTSIDE_PREVIEW_ENVELOPE')
    proposed = [float(min(.4, distances[-1]) * max(0., math.cos(angle))), 0.,
                float(np.clip(1.2 * angle, -.45, .45))]
    return {'execute': not reasons, 'command': [0., 0., 0.] if reasons else proposed,
            'proposed_command_only': proposed, 'reasons': reasons,
            'controller': 'forward-only path adapter; NOT upstream MPC'}
