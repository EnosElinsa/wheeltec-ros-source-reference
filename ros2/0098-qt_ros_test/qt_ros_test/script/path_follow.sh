#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py"

gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_path_follow follow_path.launch.py"

wait
exit 0


