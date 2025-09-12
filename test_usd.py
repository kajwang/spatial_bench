# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import logging
logger = logging.getLogger(__name__)

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
parser.add_argument("--task", type=str, default="Isaac-WBC-Go2Arx5", help="Name of the task.")
parser.add_argument("--keyboard", action="store_true", default=False, help="Whether to use keyboard.")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app，MUST BEFORE OMNI.ISAAC IMPORT
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics, Gf

def analyze_usd_file(file_path):

    # Open the USD stage
    stage = Usd.Stage.Open(file_path)
    if not stage:
        print(f"Could not open USD file: {file_path}")
        return None


    print(f"Opened USD file: {file_path}")


    prim_data = {}
    for prim in stage.Traverse():

        prim_path = str(prim.GetPath())
        prim_type = prim.GetTypeName()
        prim_name = prim.GetName()
        print(f"Analyzing Prim: {prim_name} | Type: {prim_type} | Path: {prim_path}")


        properties = {} #use this section to see the properties of the USD
        for prop in prim.GetProperties():
            prop_name = prop.GetName()
            prop_value = prim.GetAttribute(prop_name).Get()
            print(f"Property: {prop_name} | Value: {prop_value}")

        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, prim_path)

        if rigid_body_api:
            print(f"The prim '{prim_path}' has RigidBodyAPI applied.")
        else:
            print(f"The prim '{prim_path}' does NOT have RigidBodyAPI applied.")
            if prim.CanApplyAPI(UsdPhysics.RigidBodyAPI):
                print("Applying RigidBodyAPI...")
                UsdPhysics.RigidBodyAPI.Apply(prim)
                print("RigidBodyAPI applied.")

        mass_api = UsdPhysics.MassAPI.Get(stage, prim_path)
        if mass_api:
            print(f"The prim '{prim_path}' has MassAPI applied.")
        else:
            print(f"The prim '{prim_path}' does NOT have MassAPI applied.")
            if prim.CanApplyAPI(UsdPhysics.MassAPI):
                print("Applying MassAPI...")
                UsdPhysics.MassAPI.Apply(prim)
                print("MassAPI applied.")


        collision_api = UsdPhysics.CollisionAPI.Get(stage, prim_path)
        if collision_api:
            print(f"The prim '{prim_path}' has CollisionAPI applied.")
        else:
            print(f"The prim '{prim_path}' does NOT have CollisionAPI applied.")
            if prim.CanApplyAPI(UsdPhysics.CollisionAPI):
                print("Applying CollisionAPI...")
                UsdPhysics.CollisionAPI.Apply(prim)
                print("CollisionAPI applied.")
        stage.GetRootLayer().Save()
        break # as we are just doing it for the root


    return prim_data


def main():

    usd_file_path = "/home/kaijun/code/mobile_bench/assets/object/fruit/apple/apple_test.usd"

    prim_data = analyze_usd_file(usd_file_path)



if __name__ == "__main__":
    main()