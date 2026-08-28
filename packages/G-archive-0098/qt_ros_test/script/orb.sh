#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch orb_slam2_ros orb_slam2_Astra_rgbd_launch.py"


wait
exit 0


