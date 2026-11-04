"""Local WB5 actor adaptation environment; import after Isaac AppLauncher."""
from pathlib import Path
import torch
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.managers import SceneEntityCfg, RewardTermCfg, TerminationTermCfg
from isaaclab_tasks.manager_based.locomotion.velocity.config.h1.flat_env_cfg import H1FlatEnvCfg
from isaaclab_tasks.manager_based.locomotion.velocity import mdp
from jm_coop4_rl.h1_rack_env import H1_RACK_ROBOT_CFG, BODY_19
from jm_wb5_turn.commands import TurnBalancedCommandCfg

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT/'jm_wb7_pinkik/assets/h1_wb7_physical_grasp_pad12_dual_mimic.usd'


def base_height_deficit_l2(env, target_height=.90):
    height = env.scene['robot'].data.root_pos_w[:, 2]-env.scene.env_origins[:, 2]
    return torch.square(torch.clamp(target_height-height, min=0.))


@configclass
class TurnEnvCfg(H1FlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = H1_RACK_ROBOT_CFG.copy()
        self.scene.robot.spawn.usd_path = str(ASSET)
        self.scene.robot.init_state.pos = (0., 0., .98)
        self.scene.robot.init_state.rot = (1., 0., 0., 0.)
        # Drive only gripper leaders; passive mimic followers must not fight their constraints.
        self.scene.robot.actuators['appendage'] = ImplicitActuatorCfg(
            joint_names_expr=['.*wrist.*', 'finger_joint(_0)?'], effort_limit_sim=30, stiffness=30., damping=1.)
        self.scene.robot.actuators['mimic_followers'] = ImplicitActuatorCfg(
            joint_names_expr=['.*(knuckle|inner_finger).*'], effort_limit_sim=0., stiffness=0., damping=0.)
        self.scene.env_spacing = 3.
        for name in ('joint_pos', 'joint_vel'):
            getattr(self.observations.policy, name).params['asset_cfg'] = SceneEntityCfg(
                'robot', joint_names=list(BODY_19), preserve_order=True)
        self.actions.joint_pos = mdp.JointPositionActionCfg(asset_name='robot',
            joint_names=list(BODY_19), preserve_order=True, scale=.5, use_default_offset=True)
        self.rewards.dof_pos_limits.params['asset_cfg'] = SceneEntityCfg('robot', joint_names='.*_ankle_joint')
        self.rewards.joint_deviation_hip.params['asset_cfg'] = SceneEntityCfg('robot', joint_names=['.*_hip_yaw_joint', '.*_hip_roll_joint'])
        self.rewards.joint_deviation_arms.params['asset_cfg'] = SceneEntityCfg('robot', joint_names=['.*_shoulder_.*_joint', '.*_elbow_joint'])
        self.rewards.joint_deviation_torso.params['asset_cfg'] = SceneEntityCfg('robot', joint_names='torso_joint')
        self.rewards.flat_orientation_l2.weight = -5.
        self.rewards.base_height_deficit_l2 = RewardTermCfg(func=base_height_deficit_l2,
            weight=-20., params={'target_height': .90})
        self.terminations.base_height = TerminationTermCfg(func=mdp.root_height_below_minimum,
            params={'minimum_height': .55})
        # Keep WB5 posture rewards, including existing feet-air-time gating, unchanged.
        self.commands.base_velocity = TurnBalancedCommandCfg(asset_name='robot',
            resampling_time_range=(12.,12.), rel_standing_envs=0., rel_heading_envs=0.,
            heading_command=False, debug_vis=False,
            ranges=TurnBalancedCommandCfg.Ranges(lin_vel_x=(0.,1.),lin_vel_y=(0.,0.),ang_vel_z=(-1.,1.)))
