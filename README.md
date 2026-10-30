# PAI final project: Decision-tree navigation in Gazebo Classic

Small educational ROS 2 Humble / Gazebo Classic example.

## Files
- `robot.sdf`: differential-drive robot with Gazebo ROS diff-drive plugin
- `train.py`: synthetic supervised training data and decision tree
- `drive.py`: reads `/odom`, predicts LEFT/RIGHT/FORWARD, publishes `/cmd_vel`

## Commands inside a ROS 2 Humble container

```bash
source /opt/ros/humble/setup.bash
cd /workspace
python3 train.py
# terminal 1 (in browser desktop):
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
# terminal 2:
ros2 run gazebo_ros spawn_entity.py -entity pai_diffbot -file /workspace/robot.sdf -x 0 -y 0 -z 0.05
# terminal 3:
python3 /workspace/drive.py 2 1
```

Requires Gazebo Classic, `ros-humble-gazebo-ros-pkgs`, `python3-sklearn`, and `python3-joblib`. The example is **not tested in the course image**; check dependencies, ROS topics, and robot movement before submitting. This is an educational open-space controller, not obstacle avoidance.
