## Setup environment
### Install Isaac Sim & Isaac Lab
Follow the instructions [Isaac_Lab_Doc](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) to install isaac sim & lab.
Currently we use Isaac-sim == 4.5.0 & Isaac-lab == 2.1.0.

### Setup ros2 extension
- Install ros2 humble version
recommand using fishros (domestic)
```bash
wget http://fishros.com/install -O fishros && . fishros
```

- Build ROS2 workspace
```bash
./ros2/build_ros2.sh
```

- Put /ros2/Unitree_L1.json in /path/to/your/isaac-sim/exts/isaacsim.sensors.rtx/data/lidar_configs

### Install dependencies
```bash
conda activate {your_isaaclab_env}
pip install -r requirements.txt
```

### Set up IDE:
To setup the IDE, please follow these instructions:
- set /path/to/your_isaaclab in .vscode/settings.json to activate code prompting

### Asset:
Setup /path/to/your_asset_root & /path/to/robot_asset in file utils/asset_utils.py.

## Convert Existing Datasets
### Arnold
Put [data and assets](https://drive.google.com/drive/folders/1yaEItqU9_MdFVQmkKA6qSvfXy_cPnKGA?usp=sharing) in the same directory. For example, `data_root/data/reorient_object`, `data_root/materials` and `data_root/sample` are valid.
```bash
# create softlink in assets
ln -s ${ARNOLD_DATA_ROOT} assets/arnold
python convert_arnold.py
```

## Run eval:
```bash
# run with parameters (mode: standard=scene test only; ros2=launch eval pipeline)
./eval_task_ros2.sh {mode} {cfg_file}

# for example
./eval_task_ros2.sh standard
./eval_task_ros2.sh ros2 configs/tasks/arnold_merged/push_button/test/arnold_room_7.yaml
```
 
