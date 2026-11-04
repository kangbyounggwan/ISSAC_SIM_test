"""Fixed-anchor goal turns, with signed/unwrapped yaw to disambiguate +/-180 deg."""
import math
import torch
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass
from jm_wb5_turn.goal_math import (wrap_angle, phase_from_steps, relative_xy,
    goal_features, feedback_command, success_now)


class GoalTurnCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.groups = torch.arange(self.num_envs, device=self.device) % 4
        self.raw = torch.zeros_like(self.vel_command_b)
        self.anchor_xy = torch.zeros(self.num_envs, 2, device=self.device)
        self.last_yaw = torch.zeros(self.num_envs, device=self.device)
        self.yaw_travel = torch.zeros_like(self.last_yaw)
        self.target_delta = torch.zeros_like(self.last_yaw)
        self.hold_steps = torch.zeros_like(self.groups)
        self.ever_success = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.max_drift = torch.zeros_like(self.last_yaw)
        self._last_metric_step = torch.full_like(self.groups, -1)
        self.contact_ids, _ = env.scene.sensors['contact_forces'].find_bodies(
            ['left_ankle_link', 'right_ankle_link'], preserve_order=True)
        # Completed-episode totals persist across resets. Failures stay in denominator.
        self.completed = torch.zeros(4, device=self.device)
        self.successes = torch.zeros_like(self.completed)
        self.failures = torch.zeros_like(self.completed)
        self.final_yaw_sum = torch.zeros_like(self.completed)
        self.drift_sum = torch.zeros_like(self.completed)
        self.by_angle_completed = torch.zeros(2, 3, device=self.device)
        self.by_angle_success = torch.zeros_like(self.by_angle_completed)
        self.by_angle_error = torch.zeros_like(self.by_angle_completed)
        self.last_terminal = {}

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        self.raw[ids] = self.vel_command_b[ids]
        self.anchor_xy[ids] = self.robot.data.root_pos_w[ids, :2]
        self.last_yaw[ids] = self.robot.data.heading_w[ids]
        self.yaw_travel[ids] = 0.
        choices = torch.tensor([45., 90., 180.], device=self.device) * math.pi / 180.
        angle = choices[torch.randint(3, (len(ids),), device=self.device)]
        sign = torch.where(self.groups[ids] == 1, 1., -1.)
        self.target_delta[ids] = torch.where(
            (self.groups[ids] == 1) | (self.groups[ids] == 2), angle * sign, 0.)
        self.hold_steps[ids] = 0
        self.ever_success[ids] = False
        self.max_drift[ids] = 0.
        self._last_metric_step[ids] = -1
        self._update_command()

    def live(self):
        yaw = self.robot.data.heading_w
        travel = self.yaw_travel + wrap_angle(yaw - self.last_yaw)
        phase = phase_from_steps(self._env.episode_length_buf)
        # During initial settling, keep initial heading; final target stays immutable.
        target = torch.where(phase == 0, 0., self.target_delta)
        remaining = target - travel
        xy = relative_xy(self.anchor_xy, self.robot.data.root_pos_w[:, :2], yaw)
        return xy, remaining, phase, travel

    def observation(self):
        xy, remaining, phase, _ = self.live()
        return goal_features(xy, remaining, self.groups != 3, phase)

    def instantaneous_success(self):
        xy, remaining, phase, _ = self.live()
        data = self.robot.data
        sensor = self._env.scene.sensors['contact_forces']
        contact = sensor.data.net_forces_w[:, self.contact_ids].norm(dim=-1) > 5.
        return success_now(xy.norm(dim=-1), remaining,
            data.root_lin_vel_w[:, :2].norm(dim=-1), data.root_ang_vel_w[:, 2],
            torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1., 1.)),
            data.root_pos_w[:, 2] - self._env.scene.env_origins[:, 2],
            contact.all(dim=-1), self.groups != 3, phase)

    def _update_metrics(self):
        super()._update_metrics()
        xy, _, _, _ = self.live()
        self.max_drift = torch.maximum(self.max_drift, xy.norm(dim=-1))
        new_step = self._last_metric_step != self._env.episode_length_buf
        valid = self.instantaneous_success()
        count = torch.where(valid, self.hold_steps + 1, 0)
        self.hold_steps = torch.where(new_step, count, self.hold_steps)
        self._last_metric_step.copy_(self._env.episode_length_buf)
        self.ever_success |= self.hold_steps >= math.ceil(.75 / self._env.step_dt)

    def _update_command(self):
        xy, remaining, phase, travel = self.live()
        self.vel_command_b[:] = feedback_command(self.raw, self.groups, phase, xy, remaining)
        self.yaw_travel.copy_(travel)
        self.last_yaw.copy_(self.robot.data.heading_w)

    def record_completed(self, ids, terminated):
        # Called BEFORE reset events overwrite the terminal robot state.
        self._update_metrics()
        _, _, phase, travel = self.live()
        final_error = (self.target_delta - travel).abs()
        # Passing briefly then moving/falling is not a completed-task success.
        final_ok = (self.hold_steps >= math.ceil(.75 / self._env.step_dt)) & (phase == 2)
        final_ok &= self.max_drift <= .20
        final_ok &= ~terminated
        self.last_terminal = dict(env_ids=ids.clone(), group=self.groups[ids].clone(),
            target_deg=self.target_delta[ids].clone()*180/math.pi,
            final_yaw_error_deg=final_error[ids].clone()*180/math.pi,
            max_drift_m=self.max_drift[ids].clone(), hold_steps=self.hold_steps[ids].clone(),
            success=final_ok[ids].clone(), terminated=terminated[ids].clone())
        for group in range(4):
            selected = ids[self.groups[ids] == group]
            self.completed[group] += len(selected)
            self.failures[group] += terminated[selected].sum()
            self.successes[group] += final_ok[selected].sum()
            self.final_yaw_sum[group] += final_error[selected].sum()
            self.drift_sum[group] += self.max_drift[selected].sum()
            if group in (1,2):
                for index, degrees in enumerate((45,90,180)):
                    angle_ids=selected[torch.isclose(self.target_delta[selected].abs(),
                        torch.tensor(math.radians(degrees),device=self.device))]
                    self.by_angle_completed[group-1,index] += len(angle_ids)
                    self.by_angle_success[group-1,index] += final_ok[angle_ids].sum()
                    self.by_angle_error[group-1,index] += final_error[angle_ids].sum()

    def summary(self):
        xy, error, phase, _ = self.live()
        result = {}
        for group, name in enumerate(('stand', 'left', 'right', 'walk')):
            mask = self.groups == group
            if not bool(mask.any()):
                continue
            count = float(self.completed[group])
            result[name] = dict(environments=int(mask.sum()),
                active_turn_samples=int((mask & (phase == 1)).sum()) if group in (1,2) else 0,
                current_yaw_error_deg=float(error[mask].abs().mean() * 180 / math.pi) if group != 3 else None,
                current_anchor_error_m=float(xy[mask].norm(dim=-1).mean()) if group != 3 else None,
                completed_episodes=int(count), failure_episodes=int(self.failures[group]),
                pose_success_episodes=int(self.successes[group]) if group != 3 else None,
                pose_success_rate=float(self.successes[group] / count) if count and group != 3 else None,
                final_yaw_error_mean_deg=float(self.final_yaw_sum[group] / count * 180 / math.pi) if count and group != 3 else None,
                max_drift_mean_m=float(self.drift_sum[group] / count) if count and group != 3 else None)
            if group in (1,2):
                result[name]['by_angle']={}
                for index,degrees in enumerate((45,90,180)):
                    n=float(self.by_angle_completed[group-1,index])
                    result[name]['by_angle'][str(degrees)]=dict(completed=int(n),
                        successes=int(self.by_angle_success[group-1,index]),
                        final_error_mean_deg=float(self.by_angle_error[group-1,index]/n*180/math.pi) if n else None)
        return result


@configclass
class GoalTurnCommandCfg(UniformVelocityCommandCfg):
    class_type: type = GoalTurnCommand
