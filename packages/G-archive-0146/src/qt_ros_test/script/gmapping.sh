#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch slam_gmapping slam_gmapping.launch.py"


wait
exit 0


