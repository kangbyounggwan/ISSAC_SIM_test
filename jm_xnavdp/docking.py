"""Forward-only terminal positioning. Success remains an external measured gate."""
import math
import numpy as np


def dock_command(position, yaw, goal, goal_yaw, max_speed=.3, radius=.18):
    delta = np.asarray(goal, dtype=float)[:2]-np.asarray(position, dtype=float)[:2]
    if not np.isfinite(np.r_[delta, yaw, goal_yaw, max_speed, radius]).all():
        raise ValueError('Nonfinite docking input')
    if not 0 < max_speed <= .4 or not 0 < radius <= .18:
        raise ValueError('Invalid docking envelope')
    distance = float(np.linalg.norm(delta))
    aligned_position = distance < radius
    target_yaw = goal_yaw if aligned_position else math.atan2(delta[1], delta[0])
    error = math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw))
    forward = math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1]
    speed = min(max_speed, .3, .8*max(0., forward-.08))*max(0., math.cos(error))**2
    # No reverse/lateral gait. Correct position before aligning the final yaw.
    if aligned_position or abs(error) > math.radians(25): speed = 0.
    phase = 'DOCK_ALIGN_STOP' if aligned_position else 'DOCK_POSITION_RECOVERY'
    return np.array([speed, 0., np.clip(1.6*error, -.3, .3)]), phase
