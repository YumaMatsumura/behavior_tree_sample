# behavior_tree_sample
[![BuildAndTest](https://img.shields.io/github/workflow/status/YumaMatsumura/behavior_tree_sample/build%20and%20test)](https://github.com/YumaMatsumura/behavior_tree_sample/actions/workflows/build.yml)

## Setup
1. Install behaviortree-cpp-v3
```bash
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
```

2. Create workspace
```bash
mkdir -p ~/ros2_ws/src
```

3. Clone groot and behavior_tree_sample packages
```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/BehaviorTree/Groot.git
```
```bash
git clone https://github.com/YumaMatsumura/behavior_tree_sample.git
```

4. Build groot and behavior_tree_sample packages
```bash
cd ~/ros2_ws
```
```bash
colcon build --symlink-install
```

## Usage
```bash
ros2 launch behavior_tree_sample bt_engine.launch.py
```
