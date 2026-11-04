"""Experimental goal-conditioned WB5 adapter; not a No More Marching reproduction."""
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import ObservationTermCfg, RewardTermCfg, TerminationTermCfg
from isaaclab.utils import configclass
from jm_wb5_turn.env import TurnEnvCfg
from jm_wb5_turn.goal_commands import GoalTurnCommandCfg


def goal_observation(env):
    return env.command_manager.get_term('base_velocity').observation()


def goal_heading(env):
    cmd = env.command_manager.get_term('base_velocity')
    _, error, phase, _ = cmd.live()
    return torch.exp(-error.abs()) * (cmd.groups != 3) * (phase == 1)


def anchor_error(env):
    cmd = env.command_manager.get_term('base_velocity')
    xy, _, _, _ = cmd.live()
    return xy.square().sum(dim=-1) * (cmd.groups != 3)


def stable_goal(env):
    return env.command_manager.get_term('base_velocity').instantaneous_success().float()


def brake_motion(env):
    cmd = env.command_manager.get_term('base_velocity')
    _, _, phase, _ = cmd.live()
    data = env.scene['robot'].data
    return (data.root_lin_vel_w[:, :2].square().sum(dim=-1) + data.root_ang_vel_w[:, 2].square()) * (cmd.groups != 3) * (phase == 2)


def feet_air_time_turn_aware(env, command_name, threshold, sensor_cfg):
    """Isaac Lab biped air-time form; activate for angular OR linear movement.

    Based on Isaac Lab locomotion mdp/rewards.py (BSD-3-Clause). Preserve the
    contact/air-time formula, changing only the movement-command gate.
    """
    sensor = env.scene.sensors[sensor_cfg.name]
    air = sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact = sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    touching = contact > 0.
    single = touching.sum(dim=1) == 1
    reward = torch.where(single[:, None], torch.where(touching, contact, air), 0.).min(dim=1)[0]
    command = env.command_manager.get_command(command_name)
    moving = (command[:, :2].norm(dim=-1) > .1) | (command[:, 2].abs() > .1)
    return reward.clamp(max=threshold) * moving


def anchor_escape(env):
    cmd = env.command_manager.get_term('base_velocity')
    xy, _, _, _ = cmd.live()
    return (xy.norm(dim=-1) > 1.) & (cmd.groups != 3)


@configclass
class GoalTurnEnvCfg(TurnEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 18.
        self.commands.base_velocity = GoalTurnCommandCfg(asset_name='robot',
            resampling_time_range=(1000., 1000.), rel_standing_envs=0., rel_heading_envs=0.,
            heading_command=False, debug_vis=False,
            ranges=GoalTurnCommandCfg.Ranges(lin_vel_x=(0., 1.), lin_vel_y=(0., 0.), ang_vel_z=(-1., 1.)))
        # Append after WB5's unchanged 69 features; assert layout at runtime.
        self.observations.policy.goal_pose = ObservationTermCfg(func=goal_observation)
        self.rewards.feet_air_time.func = feet_air_time_turn_aware
        self.rewards.goal_heading = RewardTermCfg(func=goal_heading, weight=2.)
        self.rewards.anchor_error = RewardTermCfg(func=anchor_error, weight=-4.)
        self.rewards.stable_goal = RewardTermCfg(func=stable_goal, weight=2.)
        self.rewards.brake_motion = RewardTermCfg(func=brake_motion, weight=-2.)
        self.terminations.anchor_escape = TerminationTermCfg(func=anchor_escape)


class GoalTurnEnv(ManagerBasedRLEnv):
    def _reset_idx(self, env_ids):
        if hasattr(self, 'command_manager'):
            cmd = self.command_manager.get_term('base_velocity')
            # Initial reset is not an episode and must not inflate the denominator.
            ids = env_ids[self.episode_length_buf[env_ids] > 0]
            if len(ids):
                cmd.record_completed(ids, self.termination_manager.terminated)
        super()._reset_idx(env_ids)
        # Isaac Lab clears episode_length_buf after command_manager.reset.
        # Rebuild the reset observation's velocity hint using the new phase zero.
        self.command_manager.get_term('base_velocity')._update_command()
