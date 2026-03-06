import torch

import isaaclab.sim as sim_utils
import isaacsim.core.utils.prims as prims_utils
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.sim.simulation_context import SimulationContext
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from utils.asset_utils import ASSET_SCENE_ROOT, ASSET_OBJECT_ROOT, ASSET_ROOT
from utils.api_utils import apply_prim_collision, apply_joint_attr

class BaseTask():
    material_library = {}
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.scene_entities = {}    # isaac objects
        self.setting_id = 0

        self.env = None
        self.node = None
    
    def set_up_task(self) -> None:
        raise NotImplementedError

    def bind_sim(self, sim: SimulationContext) -> None:
        self.sim = sim
    
    def reset(self) -> None:
        # delete loaded objects
        for entity in self.scene_entities.keys():
            prim_path = self.scene_entities[entity].cfg.prim_path
            prims_utils.delete_prim(prim_path)
            print(f"[INFO] Delete Prim Path at {prim_path}")
        self.scene_entities = {}
        self.extra_parts = {}
        
        self.set_up_task()
        self.sim.reset()

    def next(self) -> None:
        # next setting
        self.setting_id = (self.setting_id+1) % len(self.cfg.tasks.settings)
        self.reset()

    def previous(self) -> None:
        # previous setting
        self.setting_id = (self.setting_id-1) % len(self.cfg.tasks.settings)
        self.reset()

    def update(self) -> None:
        # update sim data
        for name in self.scene_entities:
            self.scene_entities[name].write_data_to_sim()
            self.scene_entities[name].update(self.sim.get_physics_dt())

    def control(self) -> None:
        if self.node:
            if self.node.plan_done: # if plan done, take action
                self.node.action()
            else:
                setting = self.cfg.tasks.settings[self.setting_id]
                scene_graph, pose = self.map_render.render(
                    robot_pose=setting.robot.translation+setting.robot.orientation,
                    objects=self.scene_entities,
                    furnitures=self.furnitures,
                    extra_parts=self.extra_parts
                )
                self.node.plan(
                    text=setting.text, 
                    scene_graph=scene_graph, 
                    pose=pose, 
                    map_path=f"{ASSET_SCENE_ROOT}{self.cfg.scene.map_dir}"
                )
    
    def load_scene(self) -> None:

        # light TODO:load light from yaml
        cfg_light = sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        )
        # cfg_light.func("/World/Light", cfg_light, orientation=(0, 0, 0, 1))
        cfg_light.func("/World/Light", cfg_light, orientation=(0.93301, 0.25, 0.25, 0.06699)) # 30 degree
        
        if "arnold" in self.cfg.name:
            usd_path = f"{ASSET_ROOT}/{self.cfg.scene.spawn.usd_path}"
        else:
            usd_path = f"{ASSET_SCENE_ROOT}{self.cfg.scene.spawn.usd_path}"

        # supposed only one scene prim for each task
        cfg_scene = sim_utils.UsdFileCfg(
            usd_path=usd_path,
            scale=self.cfg.scene.spawn.scale
        )
        cfg_scene.func(
            f"/Scene{self.cfg.scene.prim_path}", 
            cfg_scene, 
            translation=self.cfg.scene.spawn.translation,
            orientation=self.cfg.scene.spawn.orientation,
        )

        # ground
        if hasattr(self.cfg.scene, 'ground'):
            self.add_physics(self.cfg.scene.ground)
            self.add_visual(self.cfg.scene.ground)

        # wall
        if hasattr(self.cfg.scene, 'wall'):
            self.add_physics(self.cfg.scene.wall)

        self.furnitures = {}
        # furnitures
        if hasattr(self.cfg.scene, 'furniture'):
            for furniture_name in vars(self.cfg.scene.furniture):
                furniture_cfg = getattr(self.cfg.scene.furniture, furniture_name)
                self.add_physics(furniture_cfg)

                self.furnitures[furniture_name] = f'/Scene{furniture_cfg.prim_path}'

    def load_rigid(self, obj) -> None:
        # Rigid Object
        if "arnold" in self.cfg.name:
            usd_path = f"{ASSET_ROOT}/{obj.spawn.usd_path}"
        else:
            usd_path = f"{ASSET_OBJECT_ROOT}{obj.spawn.usd_path}"

        cfg_obj = RigidObjectCfg(
            prim_path=f"/Scene{obj.prim_path}",
            spawn=sim_utils.UsdFileCfg(
                usd_path=usd_path,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # kinematic_enabled=True if obj.type == "static" else False,
                    disable_gravity = True if obj.type == "static" else False,
                ),
                # collision_props = sim_utils.CollisionPropertiesCfg(
                #     collision_enabled=obj.collision_enabled,
                # ),
                mass_props=sim_utils.MassPropertiesCfg(
                    mass=obj.mass
                ),
                scale = obj.spawn.scale
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=obj.spawn.translation,
                rot=obj.spawn.orientation
            ),
        )

        obj_entity = RigidObject(cfg=cfg_obj)

        sim_utils.define_rigid_body_properties(f"/Scene{obj.prim_path}", cfg_obj.spawn.rigid_props)
        sim_utils.define_mass_properties(f"/Scene{obj.prim_path}", cfg_obj.spawn.mass_props)

        # attach physics
        if hasattr(obj, "physics_material"):
            self.add_physics(obj)

        # extra parts
        if hasattr(obj, 'parts'):
            for part_name in vars(obj.parts):
                part_cfg = getattr(obj.parts, part_name)
                self.extra_parts[part_name] = f'/Scene{part_cfg.prim_path}'

        return obj_entity
    
    def load_deformable(self, obj) -> None:
        raise NotImplementedError

    def load_articulation(self, obj) -> None:
        if "arnold" in self.cfg.name:
            usd_path = f"{ASSET_ROOT}/{obj.spawn.usd_path}"
        else:
            usd_path = f"{ASSET_OBJECT_ROOT}{obj.spawn.usd_path}"
        # Articulation Object
        cfg_obj = ArticulationCfg(
            prim_path=f"/Scene{obj.prim_path}",
            spawn=sim_utils.UsdFileCfg(
                usd_path=usd_path,
                # collision_props = sim_utils.CollisionPropertiesCfg(
                #     collision_enabled=obj.collision_enabled,
                # ),
                activate_contact_sensors=False,
                scale = obj.spawn.scale
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=obj.spawn.translation,
                rot=obj.spawn.orientation,
                joint_pos={
                    joint_name: state for joint_name, state in 
                    zip(obj.joint_names, obj.joint_init_state)
                },
            )
        )
        
        # Set actuators if available
        if hasattr(obj, 'actuators'):
            actuators = {}
            for actuator_name in vars(obj.actuators):
                actuator_cfg = getattr(obj.actuators, actuator_name)
                if actuator_cfg.type == "implicit":
                    actuators[actuator_name] = ImplicitActuatorCfg(
                        joint_names_expr=actuator_cfg.joint_names_expr,
                        # effort_limit=actuator_cfg.effort_limit,
                        # velocity_limit=actuator_cfg.velocity_limit,
                        stiffness=actuator_cfg.stiffness,
                        damping=actuator_cfg.damping,
                        friction=actuator_cfg.friction,
                    )
                else:
                    raise NotImplementedError
            
            cfg_obj.actuators = actuators

        obj_entity = Articulation(cfg=cfg_obj)

        # modify joint attr
        if hasattr(obj, 'actuators'):
            for actuator_name in vars(obj.actuators):
                actuator_cfg = getattr(obj.actuators, actuator_name)
                apply_joint_attr(actuator_cfg)

        # attach physics
        # if hasattr(obj, "physics_material"):
        #     self.add_physics(obj)

        return obj_entity
    
    def add_physics(self, cfg) -> None:        
        # attach physical material to object
        if cfg.type == "static" or cfg.type == "rigid":
            if hasattr(cfg, 'collision_approximation'):
                apply_prim_collision(f"/Scene{cfg.prim_path}", cfg.collision_approximation)

            if hasattr(cfg, 'physics_material'):
                # define rigid material
                phys_mat=sim_utils.RigidBodyMaterialCfg(
                    friction_combine_mode="multiply",
                    restitution_combine_mode="multiply",
                    static_friction=cfg.physics_material.static_friction,
                    dynamic_friction=cfg.physics_material.dynamic_friction,
                )
                phys_mat.func(f"/Scene{cfg.prim_path}/PhysMat", phys_mat)

                sim_utils.bind_physics_material(f"/Scene{cfg.prim_path}", f"/Scene{cfg.prim_path}/PhysMat")
        
        # elif cfg.type == "articulation":
        #     # add physics to specified bodies
        #     prim_paths = find_matching_prims(
        #         root_prim_path=f"/Scene{cfg.prim_path}",
        #         names_expr=cfg.physics_material.body_names_expr
        #     )
        #     for prim_path in prim_paths:
        #         sim_utils.bind_physics_material(prim_path, f"/Scene{cfg.prim_path}/PhysMat")
        

    def add_visual(self, cfg) -> None:        
        # attach visual material to object
        if hasattr(cfg, 'visual_material'):
            if "arnold" in cfg.visual_material.mdl_path:
                mdl_path = f"{ASSET_ROOT}/{cfg.visual_material.mdl_path}"
            else:
                mdl_path = f"{ASSET_SCENE_ROOT}{cfg.visual_material.mdl_path}"

            # define visual material
            vis_mat=sim_utils.MdlFileCfg(
                mdl_path=mdl_path,
                project_uvw=True
            )
            vis_mat.func(f"/Scene{cfg.prim_path}/VisMat", vis_mat)

            sim_utils.bind_visual_material(f"/Scene{cfg.prim_path}", f"/Scene{cfg.prim_path}/VisMat")

