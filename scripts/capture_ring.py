"""
Ring capture utility for Isaac Sim / Isaac Lab

This script places a camera on circular trajectories around a target prim and
captures RGB images from the active viewport for several elevation rings.

Notes:
- Intended for Isaac Sim 5.x / Isaac Lab 2.2.x (uses omni.isaac.kit + omni.usd + omni.kit.commands).
- The scene must already contain the target prim at `--target` path (default: /World/Target).
- Outputs are saved under `--out` as PNGs.

Usage example:
  python scripts/capture_ring.py --target /World/Target --out outputs/captures --radius 0.5 --views 24

"""
import os
import argparse
import math
import numpy as np

try:
    # Isaac Lab launcher pattern (same as eval_tasks.py)
    from isaaclab.app import AppLauncher
except Exception:
    AppLauncher = None

try:
    from omni.isaac.kit import SimulationApp
    from omni.usd import get_context
    import omni.kit.commands
    from omni.kit.viewport.utility import get_active_viewport_window
    from pxr import Gf, UsdGeom, Usd
except Exception:
    SimulationApp = None


def create_camera_prim(stage, prim_path):
    if stage.GetPrimAtPath(prim_path):
        return UsdGeom.Camera(stage.GetPrimAtPath(prim_path))
    cam_prim = UsdGeom.Camera.Define(stage, prim_path)
    cam = UsdGeom.Camera(cam_prim.GetPrim())
    cam.GetFocalLengthAttr().Set(35.0)
    cam.GetHorizontalApertureAttr().Set(36.0)
    return cam


def capture_viewport_image(output_path):
    vp = get_active_viewport_window()
    if vp is None:
        print('No active viewport available; skipping capture for', output_path)
        return False
    omni.kit.commands.execute('ViewportSaveView', viewport=vp, filepath=output_path)
    return True


def look_at_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    # compute a right-handed look-at matrix for camera where camera forward = (target - eye)
    eye = np.array(eye, dtype=float)
    target = np.array(target, dtype=float)
    up = np.array(up, dtype=float)
    f = target - eye
    f = f / np.linalg.norm(f)
    r = np.cross(f, up)
    if np.linalg.norm(r) < 1e-6:
        # up and f are parallel; pick a different up
        up = np.array([0.0, 1.0, 0.0])
        r = np.cross(f, up)
    r = r / np.linalg.norm(r)
    u = np.cross(r, f)
    # columns: r, u, f
    mat = np.eye(4, dtype=float)
    mat[0:3, 0] = r
    mat[0:3, 1] = u
    mat[0:3, 2] = f
    mat[0:3, 3] = eye
    return mat


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', default='/World/Target', help='Prim path of the target object')
    parser.add_argument('--out', default='outputs/captures', help='Output directory')
    parser.add_argument('--radius', type=float, default=0.5, help='Distance from object center')
    parser.add_argument('--views', type=int, default=24, help='Views per ring')
    parser.add_argument('--rings', type=int, default=3, help='Number of elevation rings')
    parser.add_argument('--elev_start', type=float, default=10.0, help='Lowest elevation angle (deg)')
    parser.add_argument('--elev_delta', type=float, default=20.0, help='Degrees added per ring')
    # AppLauncher args will be added below if available
    if AppLauncher is not None:
        AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def main():
    args = parse_args()

    if AppLauncher is None:
        print('Cannot find `isaaclab.app.AppLauncher`. Please run this script from the Isaac Lab / Isaac Sim environment (see README).')
        return

    # Launch app using AppLauncher (ensures proper extensions and environment)
    app_launcher = AppLauncher(args)
    sim_app = app_launcher.app
    stage = get_context().get_stage()

    target_prim = stage.GetPrimAtPath(args.target)
    if not target_prim:
        print(f'Target prim {args.target} not found on stage.')
        sim_app.close()
        return

    # Attempt to read approximate target center; default to origin
    center_pos = (0.0, 0.0, 0.0)
    try:
        xformable = UsdGeom.Xformable(target_prim)
        ops = xformable.GetOrderedXformOps()
        for op in ops:
            if op.GetOpName() == 'xformOp:translate' or op.GetOpName() == 'translate':
                val = op.Get()
                center_pos = (float(val[0]), float(val[1]), float(val[2]))
                break
    except Exception:
        center_pos = (0.0, 0.0, 0.0)

    out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)

    cam_path = '/World/RingCamera'
    cam = create_camera_prim(stage, cam_path)
    cam_prim = cam.GetPrim()

    views = args.views
    rings = args.rings
    for r in range(rings):
        elev = args.elev_start + r * args.elev_delta
        elev_rad = math.radians(elev)
        z = math.sin(elev_rad) * args.radius
        planar_r = math.cos(elev_rad) * args.radius

        ring_dir = os.path.join(out_dir, f'ring_{r:02d}')
        os.makedirs(ring_dir, exist_ok=True)

        for i in range(views):
            theta = 2.0 * math.pi * float(i) / float(views)
            x = center_pos[0] + planar_r * math.cos(theta)
            y = center_pos[1] + planar_r * math.sin(theta)
            z_pos = center_pos[2] + z + 0.2

            # Set camera transform: compute look-at matrix and set via USD xform
            mat = look_at_matrix((x, y, z_pos), center_pos)
            try:
                xform = UsdGeom.Xformable(cam_prim)
                xform.MakeMatrixXform().Set(Gf.Matrix4d(mat.tolist()))
            except Exception:
                # Fallback to TransformPrimCommand for translation only
                omni.kit.commands.execute('TransformPrimCommand', path=cam_path, new_translation=(x, y, z_pos))

            rgb_path = os.path.join(ring_dir, f'view_{i:03d}_rgb.png')
            ok = capture_viewport_image(rgb_path)
            if ok:
                print('Saved', rgb_path)

    sim_app.close()


if __name__ == '__main__':
    main()
