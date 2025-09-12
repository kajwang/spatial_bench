# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import utils.cli_args as cli_args
import argparse

from isaaclab.app import AppLauncher
import gymnasium as gym

# add argparse arguments
parser = argparse.ArgumentParser(description="Run an RL agent in sim environment.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--keyboard", action="store_true", default=False, help="Whether to use keyboard.")
parser.add_argument("--cfg", type=str, default="configs/tasks/pick_and_place.yaml", help="task cfg file.")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app，MUST BEFORE OMNI.ISAAC IMPORT
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni
import carb
import torch
import logging
logger = logging.getLogger(__name__)

import rl_tasks  # noqa: F401
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from rsl_rl.runners import OnPolicyRunner

from utils.config_utils import Config
import utils.teleop_utils as teleop_utils
from tasks import load_task

from ros2.sensors import create_front_cam_omnigraph, create_hand_cam_omnigraph, create_rtx_lidar
from ros2.node import MajorRobot
import time

def main():
    # ======================== Setup Task Environment ========================
    # Load configuration from the YAML file
    cfg = Config.load_from_yaml(args_cli.cfg)   
    task = load_task(cfg)

    # ======================== Setup RSL-RL Environment ========================
    if cfg.scene.type == "indoors":
        rl_task =  "Isaac-WBC-Go2Arx5-Indoors"
    elif cfg.scene.type == "outdoors":
        rl_task =  "Isaac-WBC-Go2Arx5-Outdoors"
    else:
        raise ValueError("Invalid RL task type.")
    
    env_cfg = parse_env_cfg(
        rl_task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )

    if args_cli.keyboard:
        from omni.appwindow import get_default_app_window
        # support only one robot rigging
        env_cfg.scene.num_envs = 1
        env_cfg.commands.base_velocity.debug_vis = False
        cmd_vel = torch.zeros((env_cfg.scene.num_envs, 3), dtype=torch.float32)
        cmd_ee = torch.tensor([[0.35, 0, 0.35, 1, 0, 0, 0]], dtype=torch.float32)
        cmd_gripper = [False]*env_cfg.scene.num_envs
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            get_default_app_window().get_keyboard(),
            lambda event: teleop_utils.sub_keyboard_event(event, cmd_vel, cmd_ee, cmd_gripper, task),
        )
        env_cfg.observations.policy.velocity_commands = ObsTerm(
            func=lambda env: cmd_vel.clone().to(env.device),
        )
        env_cfg.observations.policy.ee_pose_commands = ObsTerm(
            func=lambda env: cmd_ee.clone().to(env.device),
        )

    # create isaac environment
    env = gym.make(rl_task, cfg=env_cfg, render_mode="rgb_array")
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)

    # ======================== Setup Agent Policy ========================
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(rl_task, args_cli)
    
    resume_path = f"./robots/go2arx5/checkpoints/model_{cfg.scene.type}.pt"
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)
    
    # ======================== Setup ROS2 Data Convey ========================
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

    # set up sensors
    # camera - front, hand
    for i in range(env_cfg.scene.num_envs):
        create_front_cam_omnigraph(i)
        create_hand_cam_omnigraph(i)
    
    # rtx-idar - head
    annotators = create_rtx_lidar(env_cfg.scene.num_envs, False)

    # initialize ROS2 nodes
    base_node = MajorRobot(env, cmd_vel, cmd_ee, cmd_gripper)

    # ======================== Simulation Main Loop ========================
    # reset environment
    # task.bind_env(env.unwrapped) # do not bind when scanning map
    task.bind_env(env.unwrapped, base_node) # bind task with wrapped rl env
    task.reset()

    obs, _ = env.get_observations()

    start_time = time.time()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            if args_cli.keyboard:
                # gripper stepping
                teleop_utils.step_cmd_gripper(env, cmd_gripper)
            
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)
            base_node.node_pub.pub_robot_data(env_cfg.scene.num_envs, env, annotators, start_time)

            if env.unwrapped.common_step_counter % 25 == 0: # check every half sec
                # if robot still, ask for control
                if base_node.check_still():
                    task.control()
                    
                checks = task.check_success()
                print(f"[TASK] success: {checks}")

                if all(checks):
                    task.next()
                

        # if args_cli.keyboard:
        #     teleop_utils.camera_follow(env)

    # close the simulator
    env.close()

if __name__ == '__main__':
    main()


