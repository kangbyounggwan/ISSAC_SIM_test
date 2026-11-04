"""Fixed experiment: stand / positive yaw / negative yaw / legacy walking.

No position or joint writes. Schedule and bounds never adapt after failure.
"""
import torch
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass
from jm_wb5_turn.schedule import scheduled_velocity


class TurnBalancedCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.groups = torch.arange(self.num_envs, device=self.device) % 4
        self.raw = torch.zeros_like(self.vel_command_b)
        self.elapsed = torch.zeros(self.num_envs, device=self.device)

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        self.raw[env_ids] = self.vel_command_b[env_ids]
        self.elapsed[env_ids] = 0.
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        turning = ids[(self.groups[ids] == 1) | (self.groups[ids] == 2)]
        self.raw[turning, 2] = torch.empty(len(turning), device=self.device).uniform_(.15, .30)
        # Reset observations must already see the scheduled command, not the
        # intermediate uniform sample produced by the base implementation.
        self.vel_command_b[ids] = scheduled_velocity(self.raw[ids], self.groups[ids], self.elapsed[ids])

    def _update_command(self):
        self.vel_command_b[:] = scheduled_velocity(self.raw, self.groups, self.elapsed)
        self.elapsed += self._env.step_dt


@configclass
class TurnBalancedCommandCfg(UniformVelocityCommandCfg):
    class_type: type = TurnBalancedCommand
