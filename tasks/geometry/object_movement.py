from ..base_task import BaseTask

import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
 
class ObjectMoveTask(BaseTask):
    def __init__(self, cfg) -> None:
        super().__init__(cfg)

        self.objects = []
        self.checkers = []

        # only load once for each run
        self.load_scene()
    
    def load_settings(self) -> None:
        # current setting
        setting = self.cfg.tasks.settings[self.setting_id]
        self.objects = setting.rigid

    def load_objects(self) -> None:
        for obj_name in self.objects:
            obj = getattr(self.cfg.graspable_objects, obj_name, None)
            # avoid multiple initialize of same entity
            if obj_name in self.scene_entities:
                continue

            elif obj.type in ["rigid", "static"]:
                obj_entity = self.load_rigid(obj)
            else:
                raise NotImplementedError
            
            self.scene_entities[obj_name] = obj_entity


    def set_up_task(self) -> None:
        # clear buffer
        self.instructions = []
        self.checkers = []

        self.load_settings() # load settings
        self.load_objects() # load objects

        # move offset
        self.move_offset = [0, 0, 0, 0] # x, y, z, yaw

    # update object status
    def update_status(self) -> None:
        self.update()
        print("Object positions:")
        for obj_name in self.objects:
            # get world poses [x, y, z, qw, qx, qy, qz]
            obj_entity = self.scene_entities[obj_name]
            pos = obj_entity.data.root_pos_w.squeeze().cpu().numpy()
            print(f"{obj_name}: {pos}")


    # move object by keyboard (world-frame)
    def keyboard_callback(self, key: str) -> None:
        # WASD to move, QE to rotate
        if key not in ["W", "A", "S", "D", "Q", "E"]:
            return
        obj_name = self.objects[0]
        obj_entity = self.scene_entities[obj_name]

        pos = obj_entity.data.default_root_state[:, :3].squeeze().cpu().numpy()
        rot = obj_entity.data.default_root_state[:, 3:].squeeze().cpu().numpy()
        
        # rotation (keep as quaternion [w, x, y, z])
        r = R.from_quat([rot[1], rot[2], rot[3], rot[0]])

        # ---- translation in world frame ----
        if key == "W":
            self.move_offset[0] += 0.1  # +X in world frame
        elif key == "S":    
            self.move_offset[0] -= 0.1
        elif key == "A":
            self.move_offset[1] += 0.1
        elif key == "D":
            self.move_offset[1] -= 0.1

        # ---- rotation in world frame (around world Z axis) ----
        elif key == "Q":
            self.move_offset[3] += 5
        elif key == "E":
            self.move_offset[3] -= 5
        else:
            return

        # position change
        pos += np.array(self.move_offset[:3])

        # rotation change
        r_delta = R.from_euler('z', self.move_offset[3])
        r = r_delta * r     
        quat = r.as_quat()
        rot = np.array([quat[3], quat[0], quat[1], quat[2]])

        root_state = obj_entity.data.default_root_state.clone()
        root_state[0, :3] = torch.tensor(pos, device=self.sim.device)
        root_state[0, 3:7] = torch.tensor(rot, device=self.sim.device)
        obj_entity.write_root_pose_to_sim(root_state[:, :7])
        
        self.update()
        print(f"Moved {obj_name} to {pos}, {rot}")
