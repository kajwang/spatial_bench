# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import torch
import carb
import math
from scipy.spatial.transform import Rotation as R
import isaaclab.utils.math as math_utils

from tasks.base_task import BaseTask

def sub_keyboard_event(event, task) -> bool:
    """
    Keyboard event call back
    """
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        # Update the velocity commands for the first environment (0th index)
        if event.input.name in ["W", "S", "A", "D", "Q", "E", "U", "I", "O", "J", "K", "L"]:
            sub_keyboard_task(event, task)
        
        elif event.input.name in ["LEFT", "RIGHT", "DOWN"]:
            sub_keyboard_setting(event, task)

    return True

def sub_keyboard_task(event, task: BaseTask) -> bool:
    """
    Let specific task handle the keyboard event
    """
    task.keyboard_callback(event.input.name)

    return True

def sub_keyboard_setting(event, task: BaseTask) -> bool:
    """
    This function is subscribed to keyboard events and updates the task setting.
    """
    if event.input.name == "LEFT":
        task.previous()
    elif event.input.name == "RIGHT":
        task.next()
    elif event.input.name == "DOWN":
        task.reset()

    return True


def camera_follow(env):
    if not hasattr(camera_follow, "smooth_camera_positions"):
        camera_follow.smooth_camera_positions = []
    robot_pos = env.unwrapped.scene["robot"].data.root_pos_w[0]
    robot_quat = env.unwrapped.scene["robot"].data.root_quat_w[0]

    # front
    # camera_offset = torch.tensor([1.5, 0.4, 1.2], dtype=torch.float32, device=env.device)
    # lookat_offset = torch.tensor([0.4, 0.0, 0.2], dtype=torch.float32, device=env.device)

    # back
    # camera_offset = torch.tensor([-0.5, 0.5, 1.0], dtype=torch.float32, device=env.device)
    # lookat_offset = torch.tensor([0.5, 0.0, 0.0], dtype=torch.float32, device=env.device)

    # left
    camera_offset = torch.tensor([-1.1, 1.2, 0.75], dtype=torch.float32, device=env.device)
    lookat_offset = torch.tensor([0.7, 0.0, 0.0], dtype=torch.float32, device=env.device)

    # back2
    # camera_offset = torch.tensor([-0.8, 0.0, 1.5], dtype=torch.float32, device=env.device)
    # lookat_offset = torch.tensor([0.5, 0.0, 0.0], dtype=torch.float32, device=env.device)

    camera_pos = math_utils.transform_points(
        camera_offset.unsqueeze(0), pos=robot_pos.unsqueeze(0), quat=robot_quat.unsqueeze(0)
    ).squeeze(0)
    camera_pos[2] = torch.clamp(camera_pos[2], min=0.0)

    lookat_pos = math_utils.transform_points(
        lookat_offset.unsqueeze(0),
        pos=robot_pos.unsqueeze(0),
        quat=robot_quat.unsqueeze(0),
    ).squeeze(0)

    window_size = 75
    camera_follow.smooth_camera_positions.append(camera_pos)
    if len(camera_follow.smooth_camera_positions) > window_size:
        camera_follow.smooth_camera_positions.pop(0)
    smooth_camera_pos = torch.mean(torch.stack(camera_follow.smooth_camera_positions), dim=0)
    env.unwrapped.viewport_camera_controller.set_view_env_index(env_index=0)
    env.unwrapped.viewport_camera_controller.update_view_location(
        eye=smooth_camera_pos.cpu().numpy(), lookat=lookat_pos.cpu().numpy()
    )
