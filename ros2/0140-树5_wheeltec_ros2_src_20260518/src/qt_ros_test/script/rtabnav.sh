#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_robot_rtab rtabmap_localization.launch.py"
sleep 5
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py"
wait
exit 0


