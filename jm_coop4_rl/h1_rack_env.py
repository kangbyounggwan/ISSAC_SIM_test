"""Isolated H1 rack navigation RL, using the immutable WB5 low-level walker.

Import only after Isaac Lab's AppLauncher has started.  This module does not
modify the JM world, old experiment code, or the walker checkpoint.  The small
training rack is a collidable proxy; transfer to the actual four-robot world
and Atlas locomotion are separate validations.  Self collision stays OFF to
preserve the frozen walker's audited training contract.  Unfiltered body
contact forces and a root/rack exclusion box are conservative checks, not a
proof of mesh-level contact safety or inter-robot collision avoidance.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import torch
import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    ActionTerm, ActionTermCfg, EventTermCfg as EventTerm,
    ObservationGroupCfg as ObsGroup, ObservationTermCfg as ObsTerm,
    RewardTermCfg as RewTerm, TerminationTermCfg as DoneTerm,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat


ROOT = Path(__file__).resolve().parents[1]
WALKER_PATH = str(ROOT / "artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt")
ROBOT_PATH = str(ROOT / "jm_wb7_pinkik/assets/h1_wb7_standalone.usd")
WALKER_SHA256 = "e1caf0201a43a82ef8d9bb8d597bffa631c14ab6a3312938afd6eeb41e2b3e0a"

BODY_19 = (
    "left_hip_yaw_joint", "right_hip_yaw_joint", "torso_joint",
    "left_hip_roll_joint", "right_hip_roll_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_ankle_joint", "right_ankle_joint", "left_elbow_joint", "right_elbow_joint",
)
WALK_DEFAULT_19 = (
    0., 0., 0., 0., 0., .28, .28, -.28, -.28, 0., 0., .79, .79,
    0., 0., -.52, -.52, .52, .52,
)


H1_RACK_ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.environ.get("JM_COOP4_H1_USD", ROBOT_PATH),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=False,
            linear_damping=0., angular_damping=0.,
            max_linear_velocity=1000., max_angular_velocity=1000.,
            max_depenetration_velocity=1.,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=False, enabled_self_collisions=False,
            solver_position_iteration_count=4, solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0., -1., .98),
        rot=(math.sqrt(.5), 0., 0., math.sqrt(.5)),
        joint_pos={
            ".*_hip_yaw_joint": 0., ".*_hip_roll_joint": 0.,
            ".*_hip_pitch_joint": -.28, ".*_knee_joint": .79,
            ".*_ankle_joint": -.52, "torso_joint": 0.,
            ".*_shoulder_pitch_joint": .28, ".*_shoulder_roll_joint": 0.,
            ".*_shoulder_yaw_joint": 0., ".*_elbow_joint": .52,
            ".*(finger|knuckle|wrist).*": 0.,
        },
        joint_vel={".*": 0.},
    ),
    soft_joint_pos_limit_factor=.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_knee_joint", "torso_joint"],
            effort_limit_sim=300,
            stiffness={".*_hip_yaw_joint": 150., ".*_hip_roll_joint": 150., ".*_hip_pitch_joint": 200., ".*_knee_joint": 200., "torso_joint": 200.},
            damping={".*_hip_yaw_joint": 5., ".*_hip_roll_joint": 5., ".*_hip_pitch_joint": 5., ".*_knee_joint": 5., "torso_joint": 5.},
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_joint"], effort_limit_sim=100,
            stiffness={".*_ankle_joint": 20.}, damping={".*_ankle_joint": 4.},
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint", ".*_elbow_joint"],
            effort_limit_sim=300, stiffness=40., damping=10.,
        ),
        "appendage": ImplicitActuatorCfg(
            joint_names_expr=[".*(finger|knuckle|wrist).*"],
            effort_limit_sim=30, stiffness=30., damping=1.,
        ),
    },
)


class H1RackVelocityAction(ActionTerm):
    """New high-level velocity action, physical motion from frozen 69→19 actor."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        path = Path(cfg.walker_path)
        if not path.is_file():
            raise FileNotFoundError(f"Local frozen walker missing: {path}")
        import hashlib
        if hashlib.sha256(path.read_bytes()).hexdigest() != WALKER_SHA256:
            raise RuntimeError("WB5 walker hash mismatch; do not silently replace its contract")
        self.walker = torch.jit.load(str(path), map_location=env.device).eval()
        for parameter in self.walker.parameters():
            parameter.requires_grad_(False)
        self.robot = env.scene[cfg.asset_name]
        names = self.robot.joint_names
        # Explicit lookup, not regex order: these are the actor's 19 columns.
        self.body_ids = [names.index(name) for name in BODY_19]
        self.walk_default = torch.tensor(WALK_DEFAULT_19, device=self.device).unsqueeze(0)
        actual_default = self.robot.data.default_joint_pos[:, self.body_ids]
        if not torch.allclose(actual_default, self.walk_default.expand_as(actual_default), atol=1e-6):
            raise RuntimeError("Articulation defaults differ from WB5 observation/action contract")
        n = self.num_envs
        self._raw = torch.zeros(n, 3, device=self.device)
        self._velocity = torch.zeros_like(self._raw)
        self._last_velocity = torch.zeros_like(self._raw)
        self._walker_last = torch.zeros(n, 19, device=self.device)
        self._target = self.robot.data.default_joint_pos.clone()
        self._distance_before = torch.zeros(n, device=self.device)
        self._dwell = torch.zeros(n, device=self.device)
        self._cache_tick = torch.full((n,), -1, dtype=torch.long, device=self.device)
        self._state = {}
        self._low_counter = 0
        self.completed = {key: 0 for key in ("episodes", "success", "fall", "rack", "timeout")}
        self._illegal_ids = None
        env._jm_rack_action = self

    @property
    def action_dim(self):
        return 3

    @property
    def raw_actions(self):
        return self._raw

    @property
    def processed_actions(self):
        return self._velocity

    def process_actions(self, actions):
        if not torch.isfinite(actions).all():
            raise RuntimeError("Non-finite high-level action")
        self._distance_before[:] = self._goal_geometry()[0]
        self._last_velocity[:] = self._velocity
        self._raw[:] = actions
        # Zero is an actual STOP. Negative vx is clamped, never a teleport.
        normalized = torch.tanh(actions)
        self._velocity[:, 0] = normalized[:, 0].clamp(0., 1.) * self.cfg.max_forward
        self._velocity[:, 1] = normalized[:, 1] * self.cfg.max_lateral
        self._velocity[:, 2] = normalized[:, 2] * self.cfg.max_yaw_rate

    def apply_actions(self):
        if self._low_counter % self.cfg.walker_decimation == 0:
            data = self.robot.data
            obs = torch.cat((
                data.root_lin_vel_b, data.root_ang_vel_b, data.projected_gravity_b,
                self._velocity, data.joint_pos[:, self.body_ids] - self.walk_default,
                data.joint_vel[:, self.body_ids], self._walker_last,
            ), dim=-1)
            if obs.shape[1] != 69:
                raise RuntimeError(f"Frozen walker observation mismatch: {obs.shape}")
            with torch.inference_mode():
                self._walker_last[:] = self.walker(obs)
            if not torch.isfinite(self._walker_last).all():
                raise RuntimeError("Non-finite frozen-walker action")
            self._target[:] = data.default_joint_pos
            self._target[:, self.body_ids] = self.walk_default + self.cfg.joint_action_scale * self._walker_last
        # Joint drives only. No base pose/velocity writes during motion.
        self.robot.set_joint_position_target(self._target)
        self._low_counter = (self._low_counter + 1) % self.cfg.walker_decimation

    def _goal_geometry(self):
        data = self.robot.data
        local = data.root_pos_w - self._env.scene.env_origins
        delta = torch.tensor(self.cfg.goal_xy, device=self.device) - local[:, :2]
        yaw = euler_xyz_from_quat(data.root_quat_w)[2]
        c, s = torch.cos(yaw), torch.sin(yaw)
        delta_body = torch.stack((c * delta[:, 0] + s * delta[:, 1], -s * delta[:, 0] + c * delta[:, 1]), dim=-1)
        heading = torch.atan2(torch.sin(self.cfg.goal_heading - yaw), torch.cos(self.cfg.goal_heading - yaw))
        return torch.linalg.vector_norm(delta, dim=-1), delta_body, heading, local

    def task_state(self):
        """Cache dwell updates once per environment step, including partial resets."""
        tick = int(getattr(self._env, "common_step_counter", 0))
        dist, delta, heading, local = self._goal_geometry()
        data = self.robot.data
        speed = torch.linalg.vector_norm(data.root_lin_vel_b[:, :2], dim=-1)
        angular = torch.abs(data.root_ang_vel_b[:, 2])
        upright = -data.projected_gravity_b[:, 2]
        fall = (local[:, 2] < self.cfg.fall_height) | (upright < math.cos(self.cfg.fall_tilt))
        rack = ((torch.abs(local[:, 0]) < self.cfg.rack_half_width + self.cfg.root_margin)
                & (local[:, 1] > self.cfg.rack_center_y - self.cfg.rack_half_depth - self.cfg.root_margin)
                & (local[:, 1] < self.cfg.rack_center_y + self.cfg.rack_half_depth + self.cfg.root_margin))
        bounds = (torch.abs(local[:, 0]) > 2.5) | (local[:, 1] < -3.0) | (local[:, 1] > 2.0)
        sensor = self._env.scene["body_contacts"]
        if self._illegal_ids is None:
            self._illegal_ids = [i for i, name in enumerate(sensor.body_names)
                                 if any(token in name for token in ("pelvis", "torso", "hip", "shoulder", "elbow", "wrist", "gripper", "finger", "knuckle"))]
            if not self._illegal_ids:
                raise RuntimeError("Contact sensor has no monitored H1 body names")
        force = torch.linalg.vector_norm(sensor.data.net_forces_w[:, self._illegal_ids], dim=-1).amax(dim=-1)
        illegal_contact = force > self.cfg.illegal_contact_force
        safe = ~(fall | rack | bounds | illegal_contact)
        stable = (safe & (dist < self.cfg.goal_radius)
                  & (torch.abs(heading) < self.cfg.goal_heading_tolerance)
                  & (speed < self.cfg.goal_speed) & (angular < self.cfg.goal_yaw_rate)
                  & (upright > math.cos(self.cfg.goal_tilt)))
        advanced = self._cache_tick != tick
        ep_len = getattr(self._env, "episode_length_buf", torch.zeros_like(self._dwell))
        count = advanced & (ep_len > 0)
        self._dwell[count] = torch.where(stable[count], self._dwell[count] + self._env.step_dt, torch.zeros_like(self._dwell[count]))
        self._cache_tick[:] = tick
        success = stable & (self._dwell >= self.cfg.goal_hold_s - 1e-6)
        self._state = dict(distance=dist, goal_body=delta, heading=heading, speed=speed,
                           angular_speed=angular, upright=upright, height=local[:, 2],
                           force=force, stable=stable, dwell=self._dwell.clone(), success=success,
                           fall=fall, rack=rack | illegal_contact, bounds=bounds)
        return self._state

    def metrics(self):
        state = self.task_state()
        return {
            "distance_m": float(state["distance"].mean()),
            "heading_error_deg": float(torch.rad2deg(state["heading"].abs()).mean()),
            "speed_mps": float(state["speed"].mean()),
            "dwell_s": float(state["dwell"].mean()),
            "current_stable_fraction": float(state["stable"].float().mean()),
            "completed": dict(self.completed),
            "contact_safety_validated": False,
        }

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        # Managers reset after the reset event; retain cached terminal state,
        # rather than inspecting already-respawned root poses for statistics.
        if self._state:
            active = self._env.episode_length_buf[ids] > 0
            self.completed["episodes"] += int(active.sum())
            for key in ("success", "fall", "rack"):
                self.completed[key] += int((self._state[key][ids] & active).sum())
            no_terminal = ~(self._state["success"][ids] | self._state["fall"][ids] | self._state["rack"][ids] | self._state["bounds"][ids])
            self.completed["timeout"] += int((no_terminal & active).sum())
        self._raw[ids] = 0.
        self._velocity[ids] = 0.
        self._last_velocity[ids] = 0.
        self._walker_last[ids] = 0.
        self._target[ids] = self.robot.data.default_joint_pos[ids]
        self._dwell[ids] = 0.
        self._cache_tick[ids] = int(getattr(self._env, "common_step_counter", 0))


@configclass
class H1RackVelocityActionCfg(ActionTermCfg):
    class_type: type = H1RackVelocityAction
    asset_name: str = "robot"
    walker_path: str = WALKER_PATH
    walker_decimation: int = 4
    joint_action_scale: float = .5
    max_forward: float = .6
    max_lateral: float = .2
    max_yaw_rate: float = .45
    goal_xy: tuple = (0., 0.)
    goal_heading: float = math.pi / 2
    goal_radius: float = .18
    goal_heading_tolerance: float = math.radians(15.)
    goal_speed: float = .10
    goal_yaw_rate: float = .15
    goal_tilt: float = math.radians(15.)
    goal_hold_s: float = 1.
    fall_height: float = .65
    fall_tilt: float = math.radians(45.)
    # Measured original rack dimensions, floor-relative; front face y=.60.
    rack_center_y: float = .91
    rack_half_width: float = .36
    rack_half_depth: float = .31
    root_margin: float = .32
    illegal_contact_force: float = 25.


def task_observation(env):
    """18 floats: body motion9 + goal XY2 + heading sin/cos2 + action3 + dwell1 + distance1."""
    action = env._jm_rack_action
    state = action.task_state()
    data = env.scene["robot"].data
    return torch.cat((data.root_lin_vel_b, data.root_ang_vel_b, data.projected_gravity_b,
                      state["goal_body"], torch.sin(state["heading"]).unsqueeze(-1),
                      torch.cos(state["heading"]).unsqueeze(-1), action.processed_actions,
                      (state["dwell"] / action.cfg.goal_hold_s).unsqueeze(-1),
                      state["distance"].unsqueeze(-1)), dim=-1)


def task_success(env):
    return env._jm_rack_action.task_state()["success"]


def task_fall(env):
    return env._jm_rack_action.task_state()["fall"]


def task_collision(env):
    return env._jm_rack_action.task_state()["rack"]


def task_out_of_bounds(env):
    return env._jm_rack_action.task_state()["bounds"]


def progress_reward(env):
    action = env._jm_rack_action
    return (action._distance_before - action.task_state()["distance"]) / env.step_dt


def distance_reward(env):
    return -env._jm_rack_action.task_state()["distance"]


def heading_reward(env):
    return -env._jm_rack_action.task_state()["heading"].abs()


def dwell_progress_reward(env):
    # Reward is bounded in time: success terminates after one second.
    return env._jm_rack_action.task_state()["stable"].float()


def terminal_success_reward(env):
    return task_success(env).float() / env.step_dt


def terminal_failure_reward(env):
    state = env._jm_rack_action.task_state()
    return (state["fall"] | state["rack"] | state["bounds"]).float() / env.step_dt


def velocity_action_rate(env):
    action = env._jm_rack_action
    return torch.square(action.processed_actions - action._last_velocity).sum(dim=-1)


@configclass
class H1RackSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", collision_group=-1)
    robot = H1_RACK_ROBOT_CFG
    rack = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Rack",
        spawn=sim_utils.CuboidCfg(
            size=(.72, .62, 1.14), collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.10, .50, .24)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0., .91, .57)),
    )
    body_contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", update_period=0., history_length=3)
    sky = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=1800.))


@configclass
class H1RackActionsCfg:
    navigate = H1RackVelocityActionCfg()


@configclass
class H1RackObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        state = ObsTerm(func=task_observation)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy = PolicyCfg()


@configclass
class H1RackRewardsCfg:
    progress = RewTerm(func=progress_reward, weight=6.)
    distance = RewTerm(func=distance_reward, weight=1.)
    heading = RewTerm(func=heading_reward, weight=.4)
    stop_dwell = RewTerm(func=dwell_progress_reward, weight=3.)
    success = RewTerm(func=terminal_success_reward, weight=30.)
    failure = RewTerm(func=terminal_failure_reward, weight=-20.)
    action_rate = RewTerm(func=velocity_action_rate, weight=-.15)


@configclass
class H1RackEventsCfg:
    reset_joints = EventTerm(func=mdp.reset_joints_by_scale, mode="reset", params={"position_range": (1., 1.), "velocity_range": (0., 0.)})
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform, mode="reset",
        # Installed Isaac Lab composes rotation with init_state: yaw=90±15.
        # XY offsets are added to init_state (0,-1): final y=[-1.8,-.8].
        params={"pose_range": {"x": (-.3, .3), "y": (-.8, .2), "yaw": (math.radians(-15.), math.radians(15.))}, "velocity_range": {}},
    )


@configclass
class H1RackTerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(func=task_success)
    fall = DoneTerm(func=task_fall)
    rack_collision = DoneTerm(func=task_collision)
    out_of_bounds = DoneTerm(func=task_out_of_bounds)


@configclass
class H1RackEnvCfg(ManagerBasedRLEnvCfg):
    scene: H1RackSceneCfg = H1RackSceneCfg(num_envs=256, env_spacing=5.)
    observations: H1RackObservationsCfg = H1RackObservationsCfg()
    actions: H1RackActionsCfg = H1RackActionsCfg()
    rewards: H1RackRewardsCfg = H1RackRewardsCfg()
    events: H1RackEventsCfg = H1RackEventsCfg()
    terminations: H1RackTerminationsCfg = H1RackTerminationsCfg()

    def __post_init__(self):
        self.sim.dt = .005
        self.decimation = 20  # 10 Hz RL; nested walker 50 Hz; physics 200 Hz.
        self.sim.render_interval = 20
        self.episode_length_s = 20.
        self.viewer.eye = (3., -4., 2.4)
        self.viewer.lookat = (0., -.3, .9)
        self.viewer.resolution = (1280, 720)
