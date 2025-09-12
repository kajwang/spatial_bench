from pathlib import Path
import numpy as np
import os
import os.path as osp
import yaml

from scipy.spatial.transform import Rotation as R
from deepdiff import DeepDiff

asset_root = "arnold"

def multiply_quaternions_wxyz(q2, q1):
    # 转换为 scipy 的 [x, y, z, w]
    q1_xyzw = [q1[1], q1[2], q1[3], q1[0]]
    q2_xyzw = [q2[1], q2[2], q2[3], q2[0]]

    # 使用 SciPy 相乘
    r1 = R.from_quat(q1_xyzw)
    r2 = R.from_quat(q2_xyzw)
    r_new = r1 * r2  # 表示先 r2 后 r1 的旋转

    # 转换回 [w, x, y, z]
    q_new_xyzw = r_new.as_quat()
    q_new_wxyz = list(map(float, [q_new_xyzw[3], q_new_xyzw[0], q_new_xyzw[1], q_new_xyzw[2]]))
    return q_new_wxyz


def load_data(data_path):
    demos = list(Path(data_path).iterdir())
    demo_path = sorted([str(item) for item in demos if not item.is_dir()])
    data = []
    fnames = []

    for npz_path in demo_path:
        data.append(np.load(npz_path, allow_pickle=True))
        fnames.append(npz_path)
    return data, fnames

def fix_path(material_url):
    if 'omniverse' in material_url:
        material_url = material_url.split(osp.sep)
        
        path_idx = material_url.index('Base')
        path_idx += 1

        material_url = osp.join(
            asset_root, 'materials', osp.sep.join(material_url[path_idx:])
        )
    
    elif 'wasabi' in material_url:
        material_url = material_url.split(osp.sep)
        path_idx = material_url.index('materials')
        material_url = osp.join(
            asset_root, osp.sep.join(material_url[path_idx:])
        )
        
    else:
        material_url = material_url.split(osp.sep)
        path_idx = material_url.index('VRKitchen2.0')
        path_idx += 1

        material_url = osp.join(
            asset_root, osp.sep.join(material_url[path_idx:])
        )
    return material_url

def convert_position(arnold_trans, arnold_orient):
    obj_trans = [ value/100 for value in arnold_trans]
    obj_trans = [obj_trans[0], -obj_trans[2], obj_trans[1]]
    obj_orient = multiply_quaternions_wxyz(arnold_orient, [0.707, 0.707, 0.0, 0.0])
    # convert orient to list if its not (seems the case for reorient)
    if type(obj_orient) is not list:
        obj_orient = obj_orient.tolist()
    return obj_trans, obj_orient

def get_spawn(obj_params):
    arnold_trans = obj_params['object_position']
    arnold_orient = obj_params['orientation_quat']
    obj_trans, obj_orient = convert_position(arnold_trans, arnold_orient)
    return dict(
        usd_path = fix_path(obj_params['usd_path']),
        translation = obj_trans,
        orientation = obj_orient,
        scale = [ value/100 for value in obj_params['scale'].tolist()],
    )

def get_pickup_dict(obj_params):
    obj_dict = dict(
        type = "rigid",
        mass = obj_params['object_physics_properties']['mass'],
        physics_material = dict(
            static_friction = obj_params['object_physics_properties']['static_friction'],
            dynamic_friction = obj_params['object_physics_properties']['dynamic_friction'],
        ),
    )
    return obj_dict

def get_joint_dict(obj_params):
    joint_name = obj_params['object_timeline_management']['target_joint']
    physics_properties = obj_params['part_physics_properties']
    obj_dict = dict(
        type = "articulation",
        joint_names = [joint_name],
        joint_init_state = [0.0],
        actuators = dict(
            drawers = dict(
                type = "implicit",
                joint_names_expr = [joint_name], # check the expression
                joint_limits = [[0.0, 0.3]], # check the expression
                stiffness = 0.0,
                damping = 1.0,
                # damping = physics_properties["joint"]["damping_coefficient"], # check the value
                friction = physics_properties["handle"]["static_friction"],
            ),
        ),
    )
    return obj_dict

def get_reorient_dict(obj_params):
    obj_dict = dict(
        type = "rigid",
        mass = obj_params['object_physics_properties']['mass'],
        physics_material = dict(
            static_friction = obj_params['object_physics_properties']['static_friction'],
            dynamic_friction = obj_params['object_physics_properties']['dynamic_friction'],
        ),
    )
    return obj_dict

def convert_object(objects_params, task):
    objects = {}
    for obj_params in objects_params:
        obj_name = f"{obj_params['object_type']}_{obj_params['args']['task_id']}"
        obj_dict = dict(
            prim_path = f"/{obj_name}",
            spawn = get_spawn(obj_params),
            collision_enabled = True,
        )
        if task == 'reorient_object':
            obj_dict_extras = get_reorient_dict(obj_params)
        elif task in ['open_drawer', 'close_drawer']:
            obj_dict_extras = get_joint_dict(obj_params)
            # lower drawer
            obj_dict['spawn']['translation'][2] -= 0.2
        elif task == 'pickup_object':
            obj_dict_extras = get_pickup_dict(obj_params)
        obj_dict.update(obj_dict_extras)
        objects[obj_name] = obj_dict
    return objects

TASK_NAME_MAP = {
    'reorient_object': 'reorient',
    'pickup_object': 'pick_object',
    'open_drawer': 'operate_drawer',
    'close_drawer': 'operate_drawer',
}

OBJ_NAME_MAP = {
    'reorient_object': 'graspable_objects',
    'pickup_object': 'graspable_objects',
    'open_drawer': 'articulated_objects',
    'close_drawer': 'articulated_objects',
}

def convert_reorient_tasks(obj_name, task_params):
    converted_angle = 90 - task_params['target_state']  
    # arnold 0 = upright = sim 90
    # arnold 90 = flat = sim 0
    # arnold 180 = downward = sim -90
    return [
        obj_name,
        [converted_angle, 0.0, 0.0],  # r, p, y in world frame
    ]

def convert_joint_tasks(obj_name, task_params):
    return [
        obj_name,
        task_params['target_joint'],
        task_params['init_state'],
        task_params['target_state'],
    ]

def convert_pickup_tasks(obj_name, task_params):
    return [
        obj_name,
        [0, 0, task_params['target_state']/100+0.05],
    ]

def convert_task(objects_params, robot_parameters, task, obj_names=[]):
    instructions = []
    robot_trans, robot_orient = convert_position(
        robot_parameters['robot_position'],
        robot_parameters['robot_orientation_quat'])
    robot_trans[2] = 0.42  # set root height to 0.42m
    robot = dict(
        translation = robot_trans,  # Robot position in the scene
        orientation = robot_orient,  # Robot orientation in quaternion
    )
    if len(obj_names) == 0:
        obj_names = [f"{obj_params['object_type']}_{obj_params['args']['task_id']}" for obj_params in objects_params]
    
    for obj_params, obj_name in zip(objects_params, obj_names):
        task_params = obj_params['object_timeline_management']
        if task == 'open_drawer':
            drawer_instruction = convert_joint_tasks(obj_name, task_params)
            instructions.append(drawer_instruction)
            # for drawer task, we need to shift robot init state
            r = R.from_quat(robot['orientation'])
            shift = r.apply(np.array([-1, 0, 1])) * (0.3 * drawer_instruction[2] - 0.1)
            robot['translation'][0] += float(shift[0])
            robot['translation'][1] += float(shift[1])

            text = f"{task}:{drawer_instruction[0]}:Pull out the top drawer."

        elif task == 'close_drawer':
            drawer_instruction = convert_joint_tasks(obj_name, task_params)
            instructions.append(drawer_instruction)
            # for drawer task, we need to shift robot init state
            r = R.from_quat(robot['orientation'])
            shift = r.apply(np.array([-1, 0, 1])) * (0.4 * drawer_instruction[2] - 0.2)
            robot['translation'][0] += float(shift[0])
            robot['translation'][1] += float(shift[1])

            text = f"{task}:{drawer_instruction[0]}:Close the top drawer."

        elif task == 'reorient_object':
            instructions.append(convert_reorient_tasks(obj_name, task_params))

            # shift robot
            r = R.from_quat(robot['orientation'])
            shift = r.apply(np.array([-0.2, 0, 0.2]))
            robot['translation'][0] += float(shift[0])
            robot['translation'][1] += float(shift[1])

            text = f"{task}:{obj_name}:Pick and reorient the object."

        elif task == 'pickup_object':
            instructions.append(convert_pickup_tasks(obj_name, task_params))
            
            # get obj position
            obj_trans = [ value/100 for value in obj_params['object_position']]
            obj_trans = [obj_trans[0], -obj_trans[2], obj_trans[1]]

            # shift robot
            # r = R.from_quat(robot['orientation'])
            # shift = r.apply(np.array([-0.5, 0.0, 0.0]))
            shift = np.array([-0.6, 0.0, 0.0])
            r = R.from_quat([robot['orientation'][1], robot['orientation'][2], robot['orientation'][3], robot['orientation'][0]])
            shift = r.apply(shift)
            robot['translation'][0] = obj_trans[0] + float(shift[0])
            robot['translation'][1] = obj_trans[1] + float(shift[1])

            text = f"{task}:{obj_name}:Pick up the object."

    return dict(
        type = TASK_NAME_MAP[task],
        settings = [dict(text=text, robot=robot, instructions=instructions)])
    
def main():
    # task_type = 'reorient_object'
    # task_type = 'open_drawer'
    # task_type = 'close_drawer'
    task_type = 'pickup_object'
    eval_split = "test"
    data_root = "assets/arnold/data"
    data, fnames = load_data(data_path=osp.join(data_root, task_type, eval_split))
    scenes = dict()
    while len(data) > 0:
        anno = data.pop(0)
        fname = fnames.pop(0)
        # convert_single_data(anno, fname, task_type, eval_split)
        
        # group by scene
        info = anno['info'].item()
        print(f"Processing {fname}")
        scene_parameters = info['scene_parameters']
        scene_usd = fix_path(scene_parameters['usd_path'])
        if task_type == 'pickup_object':
            scene_id = fname.split('-')[7]
        elif task_type == 'reorient_object':
            scene_id = fname.split('-')[4]
        else:
            scene_id = fname.split('-')[6]
        
        if scene_usd not in scenes:
            # init scene config
            scenes[scene_usd] = dict(
                name = f"arnold_room_{scene_id}",
                scene = dict(
                    type = "indoors",
                    prim_path = "/ArnoldRoom",
                    spawn = dict(
                        usd_path = fix_path(scene_parameters['usd_path']),
                        orientation = [0.707, 0.707, 0.0, 0.0],
                        translation = [0.0, 0.0, 0.0],
                        scale = [0.01, 0.01, 0.01]
                    ),
                    ground = dict(
                        prim_path = f"/ArnoldRoom/{scene_parameters['floor_path']}",
                        type = "static",
                        collision_approximation="triangleMesh",
                        physics_material = dict(
                            static_friction = 1.0,
                            dynamic_friction = 0.8,
                        ),
                        visual_material = dict(
                            mdl_path = fix_path(scene_parameters['floor_material_url'])
                        )
                    ),
                    wall = dict(
                        prim_path = f"/ArnoldRoom/{scene_parameters['wall_path']}",
                        type = "static" if scene_id != '10' else "none", # house 10 bug,
                        collision_approximation="triangleMesh"
                    ),
                    furniture_path = scene_parameters['furniture_path'],
                    wall_material_url = fix_path(scene_parameters['wall_material_url']),
                    map_dir = "/house/arnold_0", # TODO: FIX THIS
                ),
            )
        else:
            assert(f"arnold_room_{scene_id}" == scenes[scene_usd]['name'])
        
        # add objects
        objects_params = info['objects_parameters']
        objects = convert_object(objects_params, task_type)
        obj_key = OBJ_NAME_MAP[task_type]
        
        if obj_key not in scenes[scene_usd]:
            scenes[scene_usd][obj_key] = dict()
            
        obj_dict = scenes[scene_usd][obj_key]
        obj_names = []  # these are the real names corresponding to the task
        for obj_name in objects:
            if obj_name not in obj_dict:
                obj_dict[obj_name] = objects[obj_name]
                obj_names.append(obj_name)
            else:
                # check dict are the same
                diff = DeepDiff(obj_dict[obj_name], objects[obj_name])
                dup_id = 0
                new_name = obj_name
                while len(diff) > 0: # add a duplicate index in the obj name until no conflict with existing objs
                    new_name = f"{obj_name}_{dup_id}"
                    if new_name not in obj_dict:
                        obj_dict[new_name] = objects[obj_name]
                        break
                    else:
                        dup_id += 1
                        diff = DeepDiff(obj_dict[new_name], objects[obj_name])
                obj_names.append(new_name)
        
        # add task
        tasks = convert_task(objects_params, info['robot_parameters'], task_type, obj_names)
        if "tasks" not in scenes[scene_usd]:
            scenes[scene_usd]['tasks'] = tasks
        else:
            scenes[scene_usd]['tasks']['settings'].append(tasks['settings'][0])

        # add checker
        if task_type in ['open_drawer', 'close_drawer']:
            scenes[scene_usd]['checker'] = dict(
                time_length = 3.0,
                tolerance = 1.0e-1,  # percentage
            )
        elif task_type == 'reorient_object':
            scenes[scene_usd]['checker'] = dict(
                time_length = 3.0,
                tolerance = [20.0, None, None],  # degree
                check_still = False
            )
        elif task_type == 'pickup_object':
            scenes[scene_usd]['checker'] = dict(
                time_length = 3.0,
                tolerance = [None, None, 0.05],  # meter
                check_still = False
            )

    for scene_usd in scenes:
        fname = scenes[scene_usd]['name']
        # Save the configuration to a YAML file
        yaml_path = osp.join('configs/tasks/arnold_merged', task_type, eval_split, f"{fname}.yaml")
        os.makedirs(osp.dirname(yaml_path), exist_ok=True)
        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(scenes[scene_usd], yaml_file, sort_keys=False)
        print(f"Saved to {yaml_path}")

if __name__ == '__main__':
    main()