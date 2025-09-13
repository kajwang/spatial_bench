export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# default cfg path
DEFAULT_CFG="configs/tasks/object_movement/arnold_room_0.yaml"
CFG_PATH=${2:-$DEFAULT_CFG}

python eval_tasks.py --cfg "$CFG_PATH" --keyboard