import re
import typing
import numpy as np
import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.bounds as bounds_utils
import isaaclab.sim as sim_utils
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from omni.physx.scripts import utils


def get_world_transform_xform(prim_path: str) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    """
    Get the local transformation of a prim using Xformable.
    See https://openusd.org/release/api/class_usd_geom_xformable.html
    Args:
        prim_path: The path of prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    stage = stage_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default() # The time at which we compute the bounding box
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    return translation, rotation, scale

def get_world_aabb_prim(prim_path: str) -> np.array:
    cache = bounds_utils.create_bbox_cache()
    return bounds_utils.compute_aabb(cache, prim_path)


def apply_prim_rigid(prim_path):
    stage = stage_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

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

def apply_prim_collision(prim_path, approximation='triangleMesh'):
    stage = stage_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    collision_api = UsdPhysics.CollisionAPI.Get(stage, prim_path)
    if collision_api:
        print(f"The prim '{prim_path}' has CollisionAPI applied.")
    else:
        print(f"The prim '{prim_path}' does NOT have CollisionAPI applied.")
        if prim.CanApplyAPI(UsdPhysics.CollisionAPI):
            print("Applying CollisionAPI...")
            UsdPhysics.CollisionAPI.Apply(prim)
            print("CollisionAPI applied.")
    
    if approximation != 'triangleMesh':
        utils.setCollider(prim, approximationShape=approximation)
        # TODO: not work, may be overwritten by isaaclab load function

def find_matching_prims(root_prim_path: str, names_expr: str) -> list[str]:
    # compile pattern
    pattern = re.compile(names_expr)

    # get root prim
    root_prim = stage_utils.get_current_stage().GetPrimAtPath(root_prim_path)

    # find matching prims by recursion
    matching_paths = []
    find_matching_prims_rec(root_prim, pattern, matching_paths)

    return matching_paths

# traverse all children path
def find_matching_prims_rec(prim, pattern, matching_paths) -> None:
    for child in prim.GetChildren():
        # check if match the patter
        if pattern.match(child.GetName()):
            matching_paths.append(child.GetPath())
        
        find_matching_prims_rec(child, pattern, matching_paths)


def apply_joint_attr(actuator_cfg):
    stage = stage_utils.get_current_stage()

    found_prims = []
    for prim in stage.Traverse():
        if prim.GetName() in actuator_cfg.joint_names_expr:
            found_prims.append(prim)
    
    # sorted by joint_names_expr
    prim_dict = {prim.GetName(): prim for prim in found_prims}
    found_prims = [prim_dict[name] for name in actuator_cfg.joint_names_expr if name in prim_dict]

    if len(found_prims) > 0:
        for i, found_prim in enumerate(found_prims):
            # print(f"Found joint: {found_joint.GetPrim().GetPath()}")
            # set joint attribute
            prim_type = found_prim.GetTypeName()
            if prim_type in ["PhysicsRevoluteJoint" , "PhysicsPrismaticJoint"]:
                if prim_type == "PhysicsRevoluteJoint":
                    # revoluet joint
                    joint = UsdPhysics.RevoluteJoint(found_prim)
                    drive = UsdPhysics.DriveAPI.Get(found_prim, "angular")
                else:
                    # prismatic joint
                    joint = UsdPhysics.PrismaticJoint(found_prim)
                    # joint.GetUpperLimitAttr().Set(joint.GetUpperLimitAttr().Get()*cfg.spawn.scale[2]/100.0)
                    drive = UsdPhysics.DriveAPI.Get(found_prim, "linear")
                
                # set joint limits
                joint.GetUpperLimitAttr().Set(actuator_cfg.joint_limits[i][1])
                joint.GetLowerLimitAttr().Set(actuator_cfg.joint_limits[i][0])

                # set drive attribute
                drive.CreateStiffnessAttr().Set(actuator_cfg.stiffness)
                drive.CreateDampingAttr().Set(actuator_cfg.damping)
            
            else:
                raise ValueError

            # get connected link
            child_links = joint.GetBody1Rel().GetTargets()

            if child_links:
                child_prim = stage.GetPrimAtPath(child_links[0])

                # set mass to 0.1 for easier manip
                if child_prim.IsValid():
                    mass_api = UsdPhysics.MassAPI.Apply(child_prim)
                    mass_api.CreateMassAttr().Set(0.1)

                    phys_mat=sim_utils.RigidBodyMaterialCfg(
                        friction_combine_mode="multiply",
                        restitution_combine_mode="multiply",
                        static_friction=2.0,
                        dynamic_friction=2.0,
                    )
                    phys_mat.func(f"{child_links[0]}/PhysMat", phys_mat)
                    
                    # TODO: load from config
                    phys_paths = find_matching_prims(child_links[0], "handle")
                    for path in phys_paths:
                        apply_prim_collision(path)
                        sim_utils.bind_physics_material(f"{path}", f"{child_links[0]}/PhysMat")
            else:
                child_prim = None
                print("Child link not set.")

    else:
        print(f"Joint named '{actuator_cfg.joint_names_expr}' not found.")