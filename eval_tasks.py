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

import carb
import torch
import logging
logger = logging.getLogger(__name__)

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext

from utils.config_utils import Config
import utils.teleop_utils as teleop_utils
from tasks import load_task


def main():
    # ======================== Setup Task Environment ========================
    # Load configuration from the YAML file
    cfg = Config.load_from_yaml(args_cli.cfg)   
    task = load_task(cfg)

    if args_cli.keyboard:
        from omni.appwindow import get_default_app_window
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            get_default_app_window().get_keyboard(),
            lambda event: teleop_utils.sub_keyboard_event(event, task),
        )

    # ======================== Simulation Main Loop ========================

    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # reset environment
    task.bind_sim(sim) # bind task with wrapped rl env
    task.reset()
    
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # # print the root position
        # if count % 200 == 0:
        #     print(f"Sim-time: {sim_time:.3f}")
        #     task.update_status()

    # close the simulator
    simulation_app.close()

if __name__ == '__main__':
    main()


