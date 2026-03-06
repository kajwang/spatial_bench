## Setup environment
### Install Isaac Sim & Isaac Lab
Follow the instructions [Isaac_Lab_Doc](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) to install isaac sim & lab.
Currently we use Isaac-sim == 5.0.0 & Isaac-lab == 2.2.x.

### Install dependencies
```bash
conda activate {your_isaaclab_env}
pip install -r requirements.txt
```

### Set up IDE:
To setup the IDE, please follow these instructions:
- set /path/to/your_isaaclab in .vscode/settings.json to activate code prompting

### Asset:
Download the assets from [Google Drive Link](https://drive.google.com/file/d/1fLYqV2lXbXy9I4VHaGTPDCeLD8CCvnrB/view?usp=sharing), and put it under root directory.

Setup /path/to/your_asset_root & /path/to/robot_asset in utils/asset_utils.py.

## Run benchmark simulation:
```bash
# run with config file
./run_sim.sh {cfg_file}

# for example
./run_sim.sh configs/tasks/object_movement/arnold_room_0.yaml
```

