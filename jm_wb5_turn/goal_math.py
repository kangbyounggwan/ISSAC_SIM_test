"""Goal-turn tensor math. No simulator import, no robot-state writes."""
import math
import torch

BASE_OBS = 69
GOAL_OBS = 7


def wrap_angle(value):
    return torch.atan2(torch.sin(value), torch.cos(value))


def phase_from_steps(steps, settle_steps=100, brake_steps=700):
    return (steps >= settle_steps).long() + (steps >= brake_steps).long()


def relative_xy(anchor, position, yaw):
    delta = anchor - position
    c, s = torch.cos(yaw), torch.sin(yaw)
    return torch.stack((c * delta[:, 0] + s * delta[:, 1],
                        -s * delta[:, 0] + c * delta[:, 1]), dim=-1)


def goal_features(xy, remaining, pose_mask, phase):
    features = torch.cat((xy.clamp(-2., 2.), torch.sin(remaining)[:, None],
        torch.cos(remaining)[:, None], (remaining / math.pi).clamp(-2., 2.)[:, None],
        pose_mask.float()[:, None], (phase.float() / 2.)[:, None]), dim=-1)
    return features * pose_mask[:, None]


def feedback_command(raw, groups, phase, xy, remaining):
    """Bounded velocity hint; policy also receives the actual goal error.

    Zero-velocity brake is deliberate even for failed goals. Never change the goal.
    """
    command = raw.clone()
    pose = groups != 3
    command[pose] = 0.
    turn = ((groups == 1) | (groups == 2)) & (phase == 1)
    command[turn, :2] = (0.8 * xy[turn]).clamp(-.15, .15)
    command[turn, 2] = (1.0 * remaining[turn]).clamp(-.6, .6)
    return command


def success_now(xy_error, yaw_error, speed, yaw_rate, tilt, height,
                both_feet_contact, pose_mask, phase):
    # Fixed initial flat-ground benchmark, NOT a certified rack-clearance envelope.
    return (pose_mask & (phase > 0) & (xy_error <= .10)
        & (yaw_error.abs() <= math.radians(5)) & (speed <= .05)
        & (yaw_rate.abs() <= .05) & (tilt <= math.radians(10))
        & (height >= .80) & both_feet_contact)


def expand_actor_state(source_state, target_state):
    """Append zero first-layer weights, preserving WB5's initial output exactly."""
    expanded = {key: value.clone() for key, value in source_state.items()}
    key = '0.weight'
    original = source_state[key]
    target = target_state[key]
    if original.shape[1] != BASE_OBS or target.shape != (original.shape[0], BASE_OBS + GOAL_OBS):
        raise ValueError('Expected a 69 -> 76 observation adapter')
    expanded[key] = torch.zeros_like(target)
    expanded[key][:, :BASE_OBS] = original
    return expanded
