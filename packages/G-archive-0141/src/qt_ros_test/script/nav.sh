#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/HwHiAiUser/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py"

wait
exit 0


