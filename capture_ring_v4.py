import argparse
import os
import math
import json
import importlib
import numpy as np
import torch
from PIL import Image

# [1] launch app (must be before importing any isaaclab.sim or isaaclab.sensors modules)
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--cfg", type=str, default="configs/tasks/object_movement/arnold_room_0.yaml")
parser.add_argument("--setting-id", type=int, default=1)
parser.add_argument("--target", type=str, default="apple")
parser.add_argument("--out", type=str, default="outputs/captures")
parser.add_argument("--radius", type=float, default=0.6) 
parser.add_argument("--views", type=int, default=24)
parser.add_argument("--rings", type=int, default=3)
parser.add_argument("--view-center-z-offset", type=float, default=0.05,
                    help="Z offset added to target center for orbit/look-at center")
parser.add_argument("--overview-height-scale", type=float, default=1.2,
                    help="Scale for overview camera height relative to orbit horizontal radius")
parser.add_argument("--overview-radius-scale", type=float, default=0.6,
                    help="Extra overview camera height scale relative to orbit radius")
parser.add_argument("--overview-min-height", type=float, default=1.0,
                    help="Minimum height for overview camera")
parser.add_argument("--overview-side-scale", type=float, default=1.8,
                    help="Horizontal side offset scale for oblique overview camera")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# force enable camera module
args_cli.enable_cameras = True 

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# [2] import isaaclab.sim and isaaclab.sensors after app is launched
from isaaclab.sim import SimulationContext
import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg
from utils.config_utils import Config
from utils.api_utils import get_world_aabb_prim
from tasks import load_task

try:
    debug_draw = importlib.import_module("isaacsim.util.debug_draw")._debug_draw
    HAS_DEBUG_DRAW = True
except Exception:
    debug_draw = None
    HAS_DEBUG_DRAW = False

############################################

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    HAS_MPL = True
except Exception:
    HAS_MPL = False

def _build_bbox_dict(aabb):
    bbox_min = aabb[:3]
    bbox_max = aabb[3:]
    bbox_center = (bbox_min + bbox_max) * 0.5
    bbox_size = bbox_max - bbox_min
    return {
        "aabb_min": bbox_min.tolist(),
        "aabb_max": bbox_max.tolist(),
        "center": bbox_center.tolist(),
        "size": bbox_size.tolist(),
    }


def _save_setup_visualization(center_pos, radius_m, num_rings, num_views, object_bbox, out_dir):
    if not HAS_MPL:
        print("[WARN] matplotlib 不可用，跳过环拍区域可视化图输出。")
        return

    points = []
    for r in range(num_rings):
        elev_deg = 20 + r * 15
        elev = math.radians(elev_deg)
        for i in range(num_views):
            theta = 2.0 * math.pi * i / num_views
            cx = center_pos[0] + radius_m * math.cos(elev) * math.cos(theta)
            cy = center_pos[1] + radius_m * math.cos(elev) * math.sin(theta)
            cz = center_pos[2] + radius_m * math.sin(elev)
            points.append([cx, cy, cz])

    points = np.array(points)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=12, c="tab:blue", alpha=0.8, label="camera poses")
    ax.scatter([center_pos[0]], [center_pos[1]], [center_pos[2]], s=80, c="tab:red", label="target center")

    bbox_min = np.array(object_bbox["aabb_min"])
    bbox_max = np.array(object_bbox["aabb_max"])
    v = np.array([
        [bbox_min[0], bbox_min[1], bbox_min[2]],
        [bbox_max[0], bbox_min[1], bbox_min[2]],
        [bbox_max[0], bbox_max[1], bbox_min[2]],
        [bbox_min[0], bbox_max[1], bbox_min[2]],
        [bbox_min[0], bbox_min[1], bbox_max[2]],
        [bbox_max[0], bbox_min[1], bbox_max[2]],
        [bbox_max[0], bbox_max[1], bbox_max[2]],
        [bbox_min[0], bbox_max[1], bbox_max[2]],
    ])
    faces = [
        [v[0], v[1], v[2], v[3]],
        [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]],
        [v[2], v[3], v[7], v[6]],
        [v[1], v[2], v[6], v[5]],
        [v[0], v[3], v[7], v[4]],
    ]
    ax.add_collection3d(Poly3DCollection(faces, facecolors="tab:orange", linewidths=0.8, edgecolors="k", alpha=0.15))

    init_cam = points[0]
    look_vec = np.array(center_pos) - init_cam
    look_vec = look_vec / (np.linalg.norm(look_vec) + 1e-9)
    ax.quiver(
        init_cam[0], init_cam[1], init_cam[2],
        look_vec[0], look_vec[1], look_vec[2],
        length=0.2,
        color="tab:green",
        linewidth=2.0,
        label="initial perspective"
    )

    all_pts = np.vstack([points, np.array(center_pos).reshape(1, 3), v])
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    span = np.max(maxs - mins)
    mid = 0.5 * (maxs + mins)
    ax.set_xlim(mid[0] - 0.6 * span, mid[0] + 0.6 * span)
    ax.set_ylim(mid[1] - 0.6 * span, mid[1] + 0.6 * span)
    ax.set_zlim(mid[2] - 0.6 * span, mid[2] + 0.6 * span)

    ax.view_init(elev=30, azim=-60)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Capture setup: object bbox, camera poses and initial perspective")
    ax.legend(loc="upper right")
    plt.tight_layout()

    vis_path = os.path.join(os.path.abspath(out_dir), "capture_setup_visualization.png")
    plt.savefig(vis_path, dpi=200)
    plt.close(fig)
    print(f">>> Visualization saved: {vis_path}")

##################################

def capture_with_isaaclab_camera(sim, camera, center_pos, target_entity, out_dir, num_rings, num_views, radius_m):
    # 基础输出目录
    base_dir = os.path.abspath(out_dir)
    os.makedirs(base_dir, exist_ok=True)
    
    captured_count = 0
    total = num_rings * num_views
    target_pos_np = np.array(center_pos)

    ##########################
    object_pose_pos = target_entity.data.root_pos_w[0].detach().cpu().numpy()
    object_pose_quat = target_entity.data.root_quat_w[0].detach().cpu().numpy()
    object_aabb = np.array(get_world_aabb_prim(target_entity.cfg.prim_path), dtype=np.float64)
    object_bbox_dict = _build_bbox_dict(object_aabb)

    _save_setup_visualization(
        center_pos=center_pos,
        radius_m=radius_m,
        num_rings=num_rings,
        num_views=num_views,
        object_bbox=object_bbox_dict,
        out_dir=out_dir,
    )
    ################################

    print(f">>> Start Orbit Capture (Radius: {radius_m}m)")
    print(f">>> Saving to: {base_dir}")

    drawer = debug_draw.acquire_debug_draw_interface() if HAS_DEBUG_DRAW else None

    for r in range(num_rings):
        elev_deg = 20 + r * 15
        elev = math.radians(elev_deg)
        
        for i in range(num_views):
            theta = 2.0 * math.pi * i / num_views
            
            # --- 1. 计算位姿 (保持上一版修正后的正确逻辑) ---
            cx = center_pos[0] + radius_m * math.cos(elev) * math.cos(theta)
            cy = center_pos[1] + radius_m * math.cos(elev) * math.sin(theta)
            cz = center_pos[2] + radius_m * math.sin(elev)
            cam_pos_np = np.array([cx, cy, cz])
            
            forward = target_pos_np - cam_pos_np
            forward /= np.linalg.norm(forward)
            
            # 定义相机坐标系：+Z 朝向目标，+X 向右，+Y 向下
            z_vec = forward # 正前方 (+Z)
            global_up = np.array([0.0, 0.0, 1.0])
            x_vec = np.cross(z_vec, global_up) # 右侧 (+X)
            if np.linalg.norm(x_vec) < 1e-6:
                x_vec = np.array([0.0, 1.0, 0.0])
            else:
                x_vec /= np.linalg.norm(x_vec)
            y_vec = np.cross(z_vec, x_vec) # 下方 (+Y)
            
            R_mat = np.column_stack((x_vec, y_vec, z_vec))
            
            from scipy.spatial.transform import Rotation
            quat_xyzw = Rotation.from_matrix(R_mat).as_quat()
            quat_np = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            
            cam_pos_tensor = torch.tensor(cam_pos_np, dtype=torch.float32, device=sim.device).unsqueeze(0)
            cam_quat_tensor = torch.tensor(quat_np, dtype=torch.float32, device=sim.device).unsqueeze(0)
            
            camera.set_world_poses(cam_pos_tensor, cam_quat_tensor)

            if drawer is not None:
                drawer.clear_points()
                drawer.draw_points(
                    [tuple(cam_pos_np.tolist())],
                    [(1.0, 0.0, 0.0, 1.0)],
                    [24.0],
                )
            
            # --- 2. 仿真步进 ---
            sim.step()
            camera.update(dt=sim.get_physics_dt())
            
            # --- 3. 创建当前视角的独立文件夹 ---
            # 格式例如: output/r00_v023/
            current_folder_name = f"r{r:02d}_v{i:03d}"
            save_path = os.path.join(base_dir, current_folder_name)
            os.makedirs(save_path, exist_ok=True)

            # --- 4. 保存 RGB ---
            rgb_tensor = camera.data.output["rgb"][0] 
            rgb_array = rgb_tensor.cpu().numpy()
            
            # 确保 RGB 是 uint8
            if rgb_array.dtype != np.uint8:
                rgb_array = (rgb_array * 255.0).astype(np.uint8) if rgb_array.max() <= 1.0 else rgb_array.astype(np.uint8)
            
            # 处理通道 (3 vs 4)
            if rgb_array.shape[-1] == 4:
                rgb_img = Image.fromarray(rgb_array, 'RGBA').convert('RGB')
            else:
                rgb_img = Image.fromarray(rgb_array, 'RGB')
            
            rgb_img.save(os.path.join(save_path, "rgb.png"))

            # --- 5. 保存 Depth (深度图) ---
            # IsaacLab 输出的深度形状通常是 (H, W, 1)
            depth_tensor = camera.data.output["distance_to_image_plane"][0]
            
            # 【核心修复】：使用 squeeze() 挤掉大小为 1 的维度，变成 (H, W)
            depth_array = depth_tensor.squeeze().cpu().numpy() 
            
            # 替换无限远 (inf) 为 0
            depth_array[np.isinf(depth_array)] = 0 
            
            # 归一化深度图
            d_min = depth_array.min()
            d_max = depth_array.max()
            
            if d_max - d_min > 1e-5:
                depth_norm = (depth_array - d_min) / (d_max - d_min) * 255.0
            else:
                depth_norm = depth_array * 0
                
            depth_img = Image.fromarray(depth_norm.astype(np.uint8), mode='L')
            depth_img.save(os.path.join(save_path, "depth.png"))

            ############################
            metadata = {
                "ring_id": r,
                "view_id": i,
                "camera": {
                    "position_xyz": cam_pos_np.tolist(),
                    "quaternion_wxyz": quat_np.tolist(),
                    "look_at_target_xyz": target_pos_np.tolist(),
                },
                "object": {
                    "name": args_cli.target,
                    "position_xyz": object_pose_pos.tolist(),
                    "quaternion_wxyz": object_pose_quat.tolist(),
                    "bounding_box": object_bbox_dict,
                },
            }
            with open(os.path.join(save_path, "metadata.json"), "w", encoding="utf-8") as f:
                json.dump(metadata, f, ensure_ascii=False, indent=2)
            ############################

            # 如果需要保存真实物理距离数据：
            # np.save(os.path.join(save_path, "depth_raw.npy"), depth_array)

            captured_count += 1
            if captured_count % 10 == 0 or captured_count == total:
                print(f"Captured {captured_count}/{total} views...")

    if drawer is not None:
        drawer.clear_points()

    print(">>> Camera capture completed.")


def main():
    cfg = Config.load_from_yaml(args_cli.cfg)   
    task = load_task(cfg)
    task.setting_id = args_cli.setting_id
    
    # 1. 初始化仿真环境 (此时时间线处于 Stopped 状态)
    sim = SimulationContext(sim_utils.SimulationCfg(device="cuda:0"))
    
    # ================= 核心修改 =================
    # 2. 必须在 sim.reset() 之前，时间线静止时实例化相机
    camera_cfg = CameraCfg(
        prim_path="/World/OrbitCamera",
        update_period=0.0,
        height=1024,
        width=1024,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, 
            focus_distance=400.0, 
            horizontal_aperture=20.955, 
            clipping_range=(0.01, 1000.0)
        ),
    )
    camera = Camera(cfg=camera_cfg)
    # ============================================

    task.bind_sim(sim)
    
    # 3. 这里的 reset 会启动时间线 (Play)，底层渲染管线此时才会合法地绑定到我们刚创建的相机上
    sim.reset()
    task.reset()
    camera.reset() # 激活相机数据流
    
    # 4. 物理预热
    for _ in range(50):
        sim.step()
    
    # 5. 寻址目标
    target_entity = task.scene_entities.get(args_cli.target)
    if not target_entity:
        print(f"Error: Target '{args_cli.target}' not found.")
        return
    
    center = target_entity.data.root_pos_w[0].cpu().numpy()
    view_center = center.copy()
    view_center[2] += args_cli.view_center_z_offset
    print(f">>> Target Found at: {center}")

    max_elev_deg = 20 + (args_cli.rings - 1) * 15
    max_horizontal_radius = args_cli.radius * math.cos(math.radians(max_elev_deg))
    overview_height = max(
        args_cli.overview_min_height,
        max_horizontal_radius * args_cli.overview_height_scale + args_cli.radius * args_cli.overview_radius_scale,
    )
    side_offset = max_horizontal_radius * args_cli.overview_side_scale
    overview_eye = np.array([
        view_center[0] + side_offset,
        view_center[1] - side_offset,
        view_center[2] + overview_height,
    ])
    overview_target = np.array([
        view_center[0],
        view_center[1],
        view_center[2],
    ])
    sim.set_camera_view(eye=overview_eye, target=overview_target)
    print(f">>> Viewport camera set to oblique overview at: {overview_eye}, target: {overview_target}")
    
    # 6. 执行拍摄 (传入实例化好的 camera)
    capture_with_isaaclab_camera(
        sim=sim,
        camera=camera,
        center_pos=view_center,
        target_entity=target_entity,
        out_dir=args_cli.out,
        num_rings=args_cli.rings,
        num_views=args_cli.views,
        radius_m=args_cli.radius,
    )
    
    print(f">>> Done! Images saved to {args_cli.out}")
    simulation_app.close()


if __name__ == "__main__":
    main()