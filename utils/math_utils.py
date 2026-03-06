import torch
from isaaclab.utils.math import quat_inv, quat_mul, quat_apply, copysign

def compute_relative_pose(
    t_src: torch.Tensor, q_src: torch.Tensor, 
    t_tgt: torch.Tensor, q_tgt: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """
    Compute the relative pose of src in tgt coordinate frame.

    Args:
        t_src: Position of src in world frame. Shape: (N, 3)
        q_src: Quaternion of src in world frame (w, x, y, z). Shape: (N, 4)
        t_tgt: Position of tgt in world frame. Shape: (N, 3)
        q_tgt: Quaternion of tgt in world frame (w, x, y, z). Shape: (N, 4)

    Returns:
        t_rel: Position of src in tgt frame. Shape: (N, 3)
        q_rel: Quaternion of src in tgt frame. Shape: (N, 4)
    """
    # q_rel = q_tgt_inv * q_src
    q_tgt_inv = quat_inv(q_tgt)
    q_rel = quat_mul(q_tgt_inv, q_src)

    # t_rel = q_tgt_inv * (t_src - t_tgt)
    delta_t = t_src - t_tgt
    t_rel = quat_apply(q_tgt_inv, delta_t)

    return t_rel, q_rel


@torch.jit.script
def euler_xyz_from_quat(quat: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Convert rotations given as quaternions to Euler angles in radians.
    Different from isaaclab.utils.math.euler_xyz_from_quat, this function wraps to pi

    Note:
        The euler angles are assumed in XYZ convention.

    Args:
        quat: The quaternion orientation in (w, x, y, z). Shape is (N, 4).

    Returns:
        A tuple containing roll-pitch-yaw. Each element is a tensor of shape (N,).

    Reference:
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    q_w, q_x, q_y, q_z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    # roll (x-axis rotation)
    sin_roll = 2.0 * (q_w * q_x + q_y * q_z)
    cos_roll = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = torch.atan2(sin_roll, cos_roll)

    # pitch (y-axis rotation)
    sin_pitch = 2.0 * (q_w * q_y - q_z * q_x)
    pitch = torch.where(torch.abs(sin_pitch) >= 1, copysign(torch.pi / 2.0, sin_pitch), torch.asin(sin_pitch))

    # yaw (z-axis rotation)
    sin_yaw = 2.0 * (q_w * q_z + q_x * q_y)
    cos_yaw = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = torch.atan2(sin_yaw, cos_yaw)

    return roll, pitch, yaw

