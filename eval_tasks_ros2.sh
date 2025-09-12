source /opt/ros/${ROS_DISTRO}/setup.bash
source ros2/${ROS_DISTRO}_ws/install/setup.bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# default cfg path
DEFAULT_CFG="configs/tasks/arnold_merged/open_drawer/test/arnold_room_0.yaml"
CFG_PATH=${2:-$DEFAULT_CFG}

# Run python script
if [ "$1" == "ros2" ]; then
    echo "Running ROS2 version with config: $CFG_PATH"
    python eval_tasks_ros2.py --cfg "$CFG_PATH" --keyboard
else
    echo "Running standard version with config: $CFG_PATH"
    python eval_tasks.py --cfg "$CFG_PATH" --keyboard
fi